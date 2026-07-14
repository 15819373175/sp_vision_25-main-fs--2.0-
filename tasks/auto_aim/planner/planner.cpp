#include "planner.hpp"

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
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  return plan_impl(target, bullet_speed, std::nullopt);
}

Plan Planner::plan_impl(
  Target target, double bullet_speed, const std::optional<Eigen::Vector3d> & gimbal_feedback)
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
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // ── 计时 ──
  auto t0 = std::chrono::steady_clock::now();

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  if (gimbal_feedback.has_value()) {
    // MPC yaw 状态使用真实云台角度和角速度，实现反馈闭环。
    x0 << tools::limit_rad((*gimbal_feedback)[0] - yaw0), (*gimbal_feedback)[1];
  } else {
    x0 << traj(0, 0), traj(1, 0);
  }
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  int yaw_status = tiny_solve(yaw_solver_);
  bool yaw_ok = (yaw_status == 0) && (yaw_solver_->solution->solved == 1);

  // 4. Solve pitch
  if (gimbal_feedback.has_value()) {
    // 反馈协议不提供 pitch_vel，使用实际 pitch 和零初始速度。
    x0 << (*gimbal_feedback)[2], 0.0;
  } else {
    x0 << traj(2, 0), traj(3, 0);
  }
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

  // ── 射击判定：MPC 优化轨迹 vs 参考轨迹的跟踪误差 ──
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
  // 降级时误差保持 0，保守允许射击（不下位机断线时卡住不开火）

  plan.fire = std::hypot(yaw_err, pitch_err) < fire_thresh_;

  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return {false};

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
  const Eigen::Vector3d gimbal_feedback(current_yaw, current_yaw_vel, current_pitch);
  return plan_impl(target, bullet_speed, gimbal_feedback);
}

Plan Planner::plan(
  std::optional<Target> target, double bullet_speed, float current_yaw, float current_yaw_vel,
  float current_pitch)
{
  if (!target.has_value()) return {false};

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
  if (!target.has_value()) return {false};

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

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), bullet_traj.pitch + pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

}  // namespace auto_aim
