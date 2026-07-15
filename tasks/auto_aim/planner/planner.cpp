#include "planner.hpp"

#include <stdexcept>
#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  // 兼容旧配置：没有下面三项时，默认使用 1 deg、1 deg、30 deg/s。
  fire_yaw_feedback_thresh_ =
    yaml["fire_yaw_feedback_thresh"]
      ? yaml["fire_yaw_feedback_thresh"].as<double>() / 57.3
      : 1.0 / 57.3;
  fire_pitch_feedback_thresh_ =
    yaml["fire_pitch_feedback_thresh"]
      ? yaml["fire_pitch_feedback_thresh"].as<double>() / 57.3
      : 1.0 / 57.3;
  fire_yaw_vel_feedback_thresh_ =
    yaml["fire_yaw_vel_feedback_thresh"]
      ? yaml["fire_yaw_vel_feedback_thresh"].as<double>() / 57.3
      : 30.0 / 57.3;
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");
  comming_angle_ = tools::read<double>(yaml, "comming_angle") / 57.3;
  leaving_angle_ = tools::read<double>(yaml, "leaving_angle") / 57.3;

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  return plan_impl(target, bullet_speed, std::nullopt);
}

Plan Planner::plan_impl(
  Target target, double bullet_speed, const std::optional<Eigen::Vector3d> & fire_feedback)
{
  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  target.predict(bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    // 每次求解只选择一次装甲板，但锁定状态跨求解周期保留，避免临界位置来回跳板。
    const int armor_id = select_armor_id(target);
    if (armor_id < 0) {
      debug_armor_id = -1;
      return {false};
    }
    debug_armor_id = armor_id;
    yaw0 = aim(target, bullet_speed, armor_id)(0);
    traj = get_trajectory(target, yaw0, bullet_speed, armor_id);
  } catch (const std::exception & e) {
    debug_armor_id = -1;
    tools::logger()->warn("Unable to plan target: {} (bullet_speed={:.2f})", e.what(), bullet_speed);
    return {false};
  }

  // ── 计时 ──
  auto t0 = std::chrono::steady_clock::now();

  // 3. Solve yaw
  // TinyMPC 在这里是开环轨迹求解器：x0 对应参考轨迹第 0 点，
  // 求解后取 HALF_HORIZON 作为当前规划输出。真实云台反馈不参与 x0。
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  int yaw_status = tiny_solve(yaw_solver_);
  bool yaw_ok = (yaw_status == 0) && (yaw_solver_->solution->solved == 1);

  // 4. Solve pitch。与 yaw 一样，从参考轨迹第 0 点开始规划。
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  int pitch_status = tiny_solve(pitch_solver_);
  bool pitch_ok = (pitch_status == 0) && (pitch_solver_->solution->solved == 1);

  auto t1 = std::chrono::steady_clock::now();
  solve_time_us_ = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

  // ── 只在状态翻转时打日志（避免刷屏）──
  if (!yaw_ok && yaw_converged_) {
    tools::logger()->warn(
      "[Planner] Yaw MPC diverged (status={} iter={} pri={:.4f} dua={:.4f})",
      yaw_status, yaw_solver_->work->iter,
      yaw_solver_->work->primal_residual_state, yaw_solver_->work->dual_residual_state);
  } else if (yaw_ok && !yaw_converged_) {
    tools::logger()->info("[Planner] Yaw MPC recovered (iter={})", yaw_solver_->work->iter);
  }

  if (!pitch_ok && pitch_converged_) {
    tools::logger()->warn(
      "[Planner] Pitch MPC diverged (status={} iter={} pri={:.4f} dua={:.4f})",
      pitch_status, pitch_solver_->work->iter,
      pitch_solver_->work->primal_residual_state, pitch_solver_->work->dual_residual_state);
  } else if (pitch_ok && !pitch_converged_) {
    tools::logger()->info("[Planner] Pitch MPC recovered (iter={})", pitch_solver_->work->iter);
  }

  yaw_converged_ = yaw_ok;
  pitch_converged_ = pitch_ok;

  // 超时告警（>5ms 可能影响 10ms 控制周期）
  if (solve_time_us_ > 5000) {
    tools::logger()->warn("[Planner] MPC solve took {} us (>5ms)", solve_time_us_);
  }

  // ── 组装 Plan（收敛 / 降级）──
  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  if (yaw_ok) {
    plan.yaw     = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
    plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
    plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);
  } else {
    // 降级：退回参考轨迹，丢弃 MPC 加速度前馈
    plan.yaw     = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
    plan.yaw_vel = traj(1, HALF_HORIZON);
    plan.yaw_acc = 0;
  }

  if (pitch_ok) {
    plan.pitch     = pitch_solver_->work->x(0, HALF_HORIZON);
    plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
    plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);
  } else {
    // 降级：退回参考轨迹，丢弃 MPC 加速度前馈
    plan.pitch     = traj(2, HALF_HORIZON);
    plan.pitch_vel = traj(3, HALF_HORIZON);
    plan.pitch_acc = 0;
  }

  // ── 射击判定 ──
  // MPC 必须正常收敛，并且实际云台 yaw、pitch、yaw_vel 都真正跟上目标。
  // 任意条件不满足或没有实际反馈时，一律禁止开火。
  constexpr int shoot_offset = 2;
  double yaw_err = 0;
  double pitch_err = 0;
  if (yaw_ok) {
    yaw_err = traj(0, HALF_HORIZON + shoot_offset) -
              yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset);
  }
  if (pitch_ok) {
    pitch_err = traj(2, HALF_HORIZON + shoot_offset) -
                pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset);
  }
  const bool mpc_ready =
    yaw_ok && pitch_ok && std::hypot(yaw_err, pitch_err) < fire_thresh_;

  bool feedback_ready = false;
  if (fire_feedback.has_value()) {
    const double actual_yaw_err = std::abs(
      tools::limit_rad(plan.target_yaw - (*fire_feedback)[0]));
    const double actual_pitch_err = std::abs(
      plan.target_pitch - (*fire_feedback)[2]);
    const double actual_yaw_vel_err = std::abs(
      traj(1, HALF_HORIZON) - (*fire_feedback)[1]);

    feedback_ready =
      actual_yaw_err < fire_yaw_feedback_thresh_ &&
      actual_pitch_err < fire_pitch_feedback_thresh_ &&
      actual_yaw_vel_err < fire_yaw_vel_feedback_thresh_;
  }

  // MPC 发散/降级、反馈缺失或实际误差超限时一律不开火。
  plan.fire = mpc_ready && feedback_ready;

  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) {
    reset_armor_lock();
    return {false};
  }

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  return plan(*target, bullet_speed);
}

Plan Planner::plan(
  Target target, double bullet_speed, float current_yaw, float current_yaw_vel,
  float current_pitch)
{
  const Eigen::Vector3d fire_feedback(current_yaw, current_yaw_vel, current_pitch);
  return plan_impl(target, bullet_speed, fire_feedback);
}

Plan Planner::plan(
  std::optional<Target> target, double bullet_speed, float current_yaw, float current_yaw_vel,
  float current_pitch)
{
  if (!target.has_value()) {
    reset_armor_lock();
    return {false};
  }

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));
  target->predict(future);

  return plan(*target, bullet_speed, current_yaw, current_yaw_vel, current_pitch);
}

Plan Planner::plan(Target target, double bullet_speed, float current_yaw, float current_pitch)
{
  Plan plan = this->plan(target, bullet_speed);
  if (!plan.control) return plan;

  plan.current_yaw = current_yaw;
  plan.current_pitch = current_pitch;

  plan.yaw = tools::limit_rad(plan.yaw - current_yaw);
  plan.target_yaw = tools::limit_rad(plan.target_yaw - current_yaw);
  plan.pitch = plan.pitch - current_pitch;
  plan.target_pitch = plan.target_pitch - current_pitch;

  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed, float current_yaw, float current_pitch)
{
  if (!target.has_value()) {
    reset_armor_lock();
    return {false};
  }

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  return plan(*target, bullet_speed, current_yaw, current_pitch);
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A(2, 2);
  A << 1, DT, 0, 1;
  Eigen::MatrixXd B(2, 1);
  B << 0, DT;
  Eigen::VectorXd f(2);
  f << 0, 0;
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  // 默认 1000 太大，10 太小可能不收敛；30 在实时性和收敛性间取平衡
  yaw_solver_->settings->max_iter = 30;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A(2, 2);
  A << 1, DT, 0, 1;
  Eigen::MatrixXd B(2, 1);
  B << 0, DT;
  Eigen::VectorXd f(2);
  f << 0, 0;
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 30;
}

void Planner::reset_armor_lock()
{
  locked_armor_id_ = -1;
  locked_target_name_.reset();
  locked_armor_count_ = 0;
  debug_armor_id = -1;
}

int Planner::select_armor_id(const Target & target)
{
  constexpr double normal_visible_angle = 60.0 / 57.3;
  constexpr double spin_speed_threshold = 2.0;

  const auto armors = target.armor_xyza_list();
  if (armors.empty()) throw std::runtime_error("Target has no armor");

  // 切换到另一辆车或装甲板数量改变时，旧锁定编号不再可信。
  if (
    !locked_target_name_.has_value() || locked_target_name_.value() != target.name ||
    locked_armor_count_ != armors.size()) {
    reset_armor_lock();
    locked_target_name_ = target.name;
    locked_armor_count_ = armors.size();
  }

  // EKF 尚未识别过装甲板跳变时，只有 0 号板来自真实观测，其余板是几何外推。
  if (!target.jumped) {
    locked_armor_id_ = 0;
    return locked_armor_id_;
  }

  const auto x = target.ekf_x();
  const double center_yaw = std::atan2(x[2], x[0]);
  const double angular_velocity = x[7];

  std::vector<double> delta_angles;
  delta_angles.reserve(armors.size());
  for (const auto & armor : armors) {
    delta_angles.push_back(tools::limit_rad(armor[3] - center_yaw));
  }

  auto lock_is_valid = [&]() {
    return locked_armor_id_ >= 0 &&
           locked_armor_id_ < static_cast<int>(armors.size());
  };

  // 普通跟随：在正面 ±60° 可射击区内持续锁定同一块板，直到它真正离开窗口。
  // 这避免两块板都接近 45° 时因 EKF 噪声反复切换。
  const bool directional_spin =
    std::abs(angular_velocity) > spin_speed_threshold || target.name == ArmorName::outpost;
  if (!directional_spin || std::abs(angular_velocity) < 1e-3) {
    if (lock_is_valid() && std::abs(delta_angles[locked_armor_id_]) <= normal_visible_angle) {
      return locked_armor_id_;
    }

    int best_id = -1;
    double best_angle = 1e10;
    for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
      const double abs_angle = std::abs(delta_angles[i]);
      if (abs_angle <= normal_visible_angle && abs_angle < best_angle) {
        best_angle = abs_angle;
        best_id = i;
      }
    }

    if (best_id < 0) return -1;
    locked_armor_id_ = best_id;
    return locked_armor_id_;
  }

  // 高速旋转：正转时装甲板从 -coming 进入，到 +leaving 离开；反转时镜像。
  // coming > leaving 形成方向相关的进出角迟滞，锁定板未越过离开边界前不切换。
  double coming_angle = comming_angle_;
  double leaving_angle = leaving_angle_;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70.0 / 57.3;
    leaving_angle = 30.0 / 57.3;
  }

  auto in_directional_window = [&](int id) {
    const double delta = delta_angles[id];
    if (angular_velocity > 0) {
      return delta >= -coming_angle && delta <= leaving_angle;
    }
    return delta >= -leaving_angle && delta <= coming_angle;
  };

  if (lock_is_valid() && in_directional_window(locked_armor_id_)) {
    return locked_armor_id_;
  }

  int best_id = -1;
  double best_angle = 1e10;
  for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
    if (!in_directional_window(i)) continue;
    const double abs_angle = std::abs(delta_angles[i]);
    if (abs_angle < best_angle) {
      best_angle = abs_angle;
      best_id = i;
    }
  }

  if (best_id < 0) return -1;
  locked_armor_id_ = best_id;
  return locked_armor_id_;
}

Eigen::Matrix<double, 2, 1> Planner::aim(
  const Target & target, double bullet_speed, int armor_id)
{
  const auto armors = target.armor_xyza_list();
  if (armor_id < 0 || armor_id >= static_cast<int>(armors.size())) {
    throw std::out_of_range("Invalid armor id");
  }

  const auto & xyza = armors[armor_id];
  const Eigen::Vector3d xyz = xyza.head<3>();
  const double yaw = xyza[3];
  const double horizontal_dist = xyza.head<2>().norm();

  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, horizontal_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), bullet_traj.pitch + pitch_offset_};
}

Trajectory Planner::get_trajectory(
  Target & target, double yaw0, double bullet_speed, int armor_id)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed, armor_id);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed, armor_id);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed, armor_id);

    auto yaw_vel_raw =
      tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);

    auto pitch_vel_raw =
      (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    // 安全限幅，单位分别为 rad/s。
    auto yaw_vel = tools::limit_min_max(yaw_vel_raw, -6.0, 6.0);
    auto pitch_vel = tools::limit_min_max(pitch_vel_raw, -3.0, 3.0);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

}  // namespace auto_aim
