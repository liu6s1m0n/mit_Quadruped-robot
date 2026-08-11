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
  std::array<Vec3<T>, kNumLegs> reaction_forces_world{};
  std::array<T, kNumLegs> contact_phase{};
  std::array<T, kNumLegs> swing_phase{};
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
  LocomotionResult<T> run(
    const StateEstimate<T> & estimate, const DesiredState<T> & desired,
    const std::array<Vec3<T>, kNumLegs> & foot_positions_world);

private:
  OffsetDurationGait & activeGait() noexcept;

  T control_time_step_;
  std::size_t iterations_between_mpc_;
  std::size_t iteration_ = 0;
  GaitType gait_type_ = GaitType::STAND;
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
