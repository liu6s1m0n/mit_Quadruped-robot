#ifndef MYMIT_ROBOT_FSM_STATE_BALANCE_STAND_H_
#define MYMIT_ROBOT_FSM_STATE_BALANCE_STAND_H_

#include <memory>

#include "FSM/FSM_State.h"
#include "WBC/LocomotionCtrl/LocomotionCtrl.hpp"

template < typename T >
class FSM_State_BalanceStand: public FSM_State < T >
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FSM_State_BalanceStand(ControlFSMData < T > * control_fsm_data);
  ~FSM_State_BalanceStand() override = default;

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;

private:
  void BalanceStandStep();

  std::unique_ptr < LocomotionCtrl < T >> wbc_ctrl_;
  LocomotionCtrlData < T > wbc_data_;
  std::size_t iteration_ = 0;
  Vec3 < T > initial_body_position_ = Vec3 < T > ::Zero();
  Vec3 < T > initial_body_rpy_ = Vec3 < T > ::Zero();
  T last_height_command_ = T(0);
  T body_weight_ = T(0);
};

extern template class FSM_State_BalanceStand < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_BALANCE_STAND_H_
