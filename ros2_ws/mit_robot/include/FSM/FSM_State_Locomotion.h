/**
 * @file FSM_State_Locomotion.h
 * @brief 行走状态：MPC 规划接触力和落脚运动，WBC 将结果转换为关节命令。
 */
#ifndef MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_
#define MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_

#include <array>
#include <memory>

#include "FSM/FSM_State.h"
#include "MPC/ConvexMPCLocomotion.h"
#include "WBC/LocomotionCtrl/LocomotionCtrl.hpp"

template < typename T >
class FSM_State_Locomotion: public FSM_State < T >
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FSM_State_Locomotion(ControlFSMData < T > * control_fsm_data);
  ~FSM_State_Locomotion() override = default;

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;

private:
  void LocomotionControlStep();
  bool locomotionSafe() const;
  std::array < Vec3 < T >, kNumLegs > footPositionsWorld() const;

  std::unique_ptr < mpc::ConvexMPCLocomotion < T >> mpc_;
  std::unique_ptr < LocomotionCtrl < T >> wbc_ctrl_;
  LocomotionCtrlData < T > wbc_data_;
  std::size_t iteration_ = 0;
};

extern template class FSM_State_Locomotion < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_
