/**
 * @file ControlFSMData.h
 * @brief 汇总各个 FSM 状态共享的输入和控制对象。
 *
 * 这里保存的是非拥有型指针：对象由 RobotRunner 创建并保证生命周期，状态类只借用。
 */
#ifndef MYMIT_ROBOT_FSM_CONTROL_FSM_DATA_H_
#define MYMIT_ROBOT_FSM_CONTROL_FSM_DATA_H_

#include <array>
#include <cmath>

#include "controller/GaitScheduler.hpp"
#include "controller/leg_controller.hpp"
#include "model/quadruped.hpp"
#include "model/robot_types.hpp"

/** Non-owning references shared by the FSM states for one control cycle. */
template < typename T >
struct ControlFSMData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  const Quadruped < T > * quadruped = nullptr;
  StateEstimate < T > *state_estimate = nullptr;
  const std::array < JointState < T >, kNumLegs > * joint_states = nullptr;
  LegController < T > *leg_controller = nullptr;
  GaitScheduler < T > *gait_scheduler = nullptr;
  DesiredState < T > *desired_state = nullptr;
  T control_time_step = T(0.001);
  bool use_wbc = true;

  bool valid() const noexcept
  {
    return quadruped != nullptr && state_estimate != nullptr &&
           joint_states != nullptr && leg_controller != nullptr &&
           gait_scheduler != nullptr && desired_state != nullptr &&
           std::isfinite(static_cast < double > (control_time_step)) &&
           control_time_step > T(0);
  }
};

#endif  // MYMIT_ROBOT_FSM_CONTROL_FSM_DATA_H_
