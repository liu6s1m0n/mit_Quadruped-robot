/**
 * @file FSM_State_Passive.h
 * @brief 关闭所有腿部输出的默认安全状态。
 */
#ifndef MYMIT_ROBOT_FSM_STATE_PASSIVE_H_
#define MYMIT_ROBOT_FSM_STATE_PASSIVE_H_

#include "FSM/FSM_State.h"

template < typename T >
class FSM_State_Passive final: public FSM_State < T >
{
public:
  explicit FSM_State_Passive(ControlFSMData < T > * control_fsm_data);
  ~FSM_State_Passive() override = default;

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;
};

extern template class FSM_State_Passive < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_PASSIVE_H_
