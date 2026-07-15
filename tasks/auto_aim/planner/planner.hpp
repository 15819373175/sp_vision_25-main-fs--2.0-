#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <cstddef>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  float current_yaw;
  float current_pitch;
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  int debug_armor_id = -1;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);
  // 云台反馈只用于最终开火门控，不参与 MPC 轨迹求解。
  Plan plan(
    Target target, double bullet_speed, float current_yaw, float current_yaw_vel,
    float current_pitch);
  Plan plan(
    std::optional<Target> target, double bullet_speed, float current_yaw, float current_yaw_vel,
    float current_pitch);
  Plan plan(Target target, double bullet_speed, float current_yaw, float current_pitch);
  Plan plan(std::optional<Target> target, double bullet_speed, float current_yaw, float current_pitch);

  // MPC 求解器健康查询
  bool last_solve_ok() const { return yaw_converged_ && pitch_converged_; }
  int64_t last_solve_time_us() const { return solve_time_us_; }

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  // 最终开火门控阈值。配置项单位为 degree / degree/s，内部统一为 rad / rad/s。
  double fire_yaw_feedback_thresh_;
  double fire_pitch_feedback_thresh_;
  double fire_yaw_vel_feedback_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double comming_angle_, leaving_angle_;

  // 跨帧装甲板锁定。armor_xyza_list() 的下标对应 Target EKF 中稳定的物理装甲板编号。
  int locked_armor_id_ = -1;
  std::optional<ArmorName> locked_target_name_;
  std::size_t locked_armor_count_ = 0;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  // MPC 收敛诊断（每次 plan() 更新）
  bool yaw_converged_ = true;
  bool pitch_converged_ = true;
  int64_t solve_time_us_ = 0;

  Plan plan_impl(
    Target target, double bullet_speed, const std::optional<Eigen::Vector3d> & fire_feedback);

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  // DH 式有状态选板：普通跟随使用跨帧锁定，高速旋转使用带方向的进出角迟滞。
  int select_armor_id(const Target & target);
  void reset_armor_lock();
  Eigen::Matrix<double, 2, 1> aim(
    const Target & target, double bullet_speed, int armor_id);
  Trajectory get_trajectory(
    Target & target, double yaw0, double bullet_speed, int armor_id);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
