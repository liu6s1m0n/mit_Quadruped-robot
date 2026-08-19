/**
 * @file FSM_State_StandUp.h
 * @brief GO1 抬升机身；DM1 先贴地收腿到稳定的四足支撑预备姿态。
 */
#ifndef MYMIT_ROBOT_FSM_STATE_STAND_UP_H_
#define MYMIT_ROBOT_FSM_STATE_STAND_UP_H_

#include <array>
#include <cstddef>

#include "FSM/FSM_State.h"

template < typename T >
class FSM_State_StandUp final: public FSM_State < T >
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FSM_State_StandUp(ControlFSMData < T > * control_fsm_data);
  ~FSM_State_StandUp() override = default;

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;

private:
  std::array < Vec3 < T >, kNumLegs > initial_foot_positions_ {};
  /** DM1 点击站立按钮时四条腿的实测电机角，作为平滑收腿轨迹起点。 */
  std::array < Vec3 < T >, kNumLegs > initial_joint_positions_ {};
  std::size_t iteration_ = 0;
  std::size_t ramp_iterations_ = 1;
  bool stand_up_complete_ = false;
};

extern template class FSM_State_StandUp < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_STAND_UP_H_
