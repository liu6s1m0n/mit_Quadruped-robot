/**
 * @file FSM_State_BalanceStand.h
 * @brief 四足均接触地面时的平衡站立状态。
 *
 * 该状态把期望机身位姿和四足支撑力交给 WBC，并保留关节 PD 作为局部稳定项。
 */
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
  
  //24～28行：FSM必须实现的接口
  /*当前状态 checkTransition()
        ↓
    如果不切换，执行 run()
    如果要切换，执行 transition()
        ↓
    旧状态 onExit()
        ↓
    新状态 onEnter()*/
  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;

private:
   //是每个控制周期真正执行站立控制的函数。
  void BalanceStandStep();
  //会自动释放，不需要手动 delete。WBC控制器
  std::unique_ptr < LocomotionCtrl < T >> wbc_ctrl_;
  LocomotionCtrlData < T > wbc_data_;
  //状态运行计数器
  std::size_t iteration_ = 0;
  Vec3 < T > initial_body_position_ = Vec3 < T > ::Zero();
  Vec3 < T > initial_body_rpy_ = Vec3 < T > ::Zero();
  T last_height_command_ = T(0);
  T body_weight_ = T(0);
};

extern template class FSM_State_BalanceStand < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_BALANCE_STAND_H_
