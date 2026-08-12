/**
 * @file ConvexMPCLocomotion.h
 * @brief 将步态相位、状态预测和凸 MPC 求解器组织成一次行走规划。
 *
 * 输出仍是世界坐标系下的足端反作用力和相位，不直接输出电机力矩。
 */
#ifndef MYMIT_ROBOT_MPC_CONVEX_MPC_LOCOMOTION_H_
#define MYMIT_ROBOT_MPC_CONVEX_MPC_LOCOMOTION_H_

#include <array>
#include <cstddef>

#include "MPC/Gait.h"
#include "MPC/SolverMPC.h"
#include "controller/GaitScheduler.hpp"

namespace mpc
{

template<typename T>
struct LocomotionResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DesiredState<T> command{};
  std::array<Vec3<T>, kNumLegs> reaction_forces_world{};
  std::array<T, kNumLegs> contact_phase{};
  std::array<T, kNumLegs> swing_phase{};
  std::array<T, kNumLegs> stance_time{};
  std::array<T, kNumLegs> swing_time{};
  std::array<bool, kNumLegs> contact_state{};
  /** 本控制周期是否实际执行并刷新了 MPC 优化解。 */
  bool mpc_updated = false;
  bool mpc_converged = false;
  bool valid = false;
};

template<typename T>
class ConvexMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(
    const Quadruped<T> & quadruped, T control_time_step,
    std::size_t iterations_between_mpc = 15,
    const SolverSettings<T> & settings = SolverSettings<T>{});

  void initialize() noexcept;
  void setGait(GaitType gait);
  /** 设置机身沿目标偏航方向的固定前进速度，单位 m/s。 */
  void setForwardVelocity(T velocity);
  /**
   * 设置机身坐标系速度命令：x 向前、y 向左、yaw 绕 z 轴逆时针，单位 m/s、rad/s。
   */
  void setVelocityCommand(T forward_velocity, T lateral_velocity, T yaw_rate);
  LocomotionResult<T> run(
    const StateEstimate<T> & estimate, const DesiredState<T> & desired,
    const std::array<Vec3<T>, kNumLegs> & foot_positions_world);

private:
  OffsetDurationGait & activeGait() noexcept;
  DesiredState<T> setupCommand(
    const StateEstimate<T> & estimate, const DesiredState<T> & desired);

  T control_time_step_;
  std::size_t iterations_between_mpc_;
  std::size_t iteration_ = 0;
  GaitType gait_type_ = GaitType::STAND;
  // 用户速度位于机身坐标系；commanded_* 是加速度限制后真正进入 MPC 的命令。
  Vec2<T> velocity_body_ = Vec2<T>::Zero();
  Vec2<T> commanded_velocity_body_ = Vec2<T>::Zero();
  T yaw_rate_ = T(0);
  T commanded_yaw_rate_ = T(0);
  T command_yaw_ = T(0);

  /*前后/左右速度命令的总加速度上限。由 0.75 提高到 1.0 m/s²，
    0.40 m/s 命令约 0.4 秒到达，同时保留连续斜坡以避免阶跃冲击。*/
  T maximum_linear_acceleration_ = T(1.0);
  T maximum_yaw_acceleration_ = T(0.8);
  Vec3<T> command_position_world_ = Vec3<T>::Zero();
  bool command_initialized_ = false;
  std::array<Vec3<T>, kNumLegs> cached_reaction_forces_world_{};
  bool cached_solution_valid_ = false;
  bool cached_solution_converged_ = false;
  SolverMPC<T> solver_;
  OffsetDurationGait stand_;
  OffsetDurationGait trot_;
};

extern template struct LocomotionResult<float>;
extern template struct LocomotionResult<double>;
extern template class ConvexMPCLocomotion<float>;
extern template class ConvexMPCLocomotion<double>;

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_CONVEX_MPC_LOCOMOTION_H_
