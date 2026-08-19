/**
 * @file ControlFSM.h
 * @brief 控制有限状态机的总入口。
 *
 * 新手阅读提示：ControlFSM 不直接计算力矩，它根据 DesiredState 中的 mode
 * 选择站立、行走、关节 PD 或被动状态，再把当前状态生成的命令写入腿控制器。
 */
#ifndef MYMIT_ROBOT_FSM_CONTROL_FSM_H_
#define MYMIT_ROBOT_FSM_CONTROL_FSM_H_

#include <array>
#include <cstddef>
#include <memory>

#include "FSM/FSM_State.h"
#include "FSM/FSM_State_BalanceStand.h"
#include "FSM/FSM_State_FrontJump.h"
#include "FSM/FSM_State_Locomotion.h"
#include "FSM/FSM_State_Passive.h"
#include "FSM/FSM_State_RecoveryStand.h"
#include "FSM/FSM_State_StandUp.h"
#include "FSM/SafetyChecker.h"

enum class FSM_OperatingMode
{
  NORMAL,
  TRANSITIONING,
  ESTOP,
  EDAMP
};

/*每个状态只创建一次，通过 unique_ptr 管理生命周期。
  joint_pd 类型是基类指针，是因为 JointPdState 是在 .cpp 中定义的内部类。*/
template < typename T >
struct FSM_StatesList
{
  std::unique_ptr < FSM_State_Passive < T >> passive;
  std::unique_ptr < FSM_State < T >> joint_pd;
  std::unique_ptr < FSM_State_StandUp < T >> stand_up;
  std::unique_ptr < FSM_State_RecoveryStand < T >> recovery_stand;
  std::unique_ptr < FSM_State_BalanceStand < T >> balance_stand;
  std::unique_ptr < FSM_State_Locomotion < T >> locomotion;
  std::unique_ptr < FSM_State_FrontJump < T >> front_jump;
};

/*这是最高层的状态机。*/
/** High-level finite state machine for the modes supported by this project. */
template < typename T >
class ControlFSM
{
public:
  ControlFSM(
    const Quadruped < T > &quadruped, StateEstimate < T > &state_estimate,
    const std::array < JointState < T >, kNumLegs > &joint_states,
    LegController < T > &leg_controller, GaitScheduler < T > &gait_scheduler,
    DesiredState < T > &desired_state, T control_time_step = T(0.001));
  ~ControlFSM() = default;

  /*禁止拷贝。原因是其中包含:多个 unique_ptr；
   大量非拥有型指针；当前状态对象。复制会造成资源和状态管理混乱。*/
  ControlFSM(const ControlFSM &) = delete;
  ControlFSM & operator = (const ControlFSM &) = delete;

  void initialize();
  void runFSM();
  FSM_OperatingMode safetyPreCheck();
  FSM_OperatingMode safetyPostCheck();
  FSM_State < T > *getNextState(FSM_StateName state_name) noexcept;
  void printInfo(int option);
  //返回当前状态名字。
  FSM_StateName currentStateName() const noexcept;
  FSM_OperatingMode operatingMode() const noexcept {return operating_mode_;}
  //打开或关闭 WBC。
  void setUseWbc(bool enabled) noexcept {data.use_wbc = enabled;}
  /** 设置 Locomotion 状态使用的固定前进速度，单位 m/s。 */
  void setLocomotionForwardVelocity(T velocity);
  /** 设置 Locomotion 的机身系前后、左右速度和偏航角速度。 */
  void setLocomotionVelocityCommand(
    T forward_velocity, T lateral_velocity, T yaw_rate);
  
  //所有状态共享的数据。
  ControlFSMData < T > data;
  //所有状态对象。
  FSM_StatesList < T > statesList;
  FSM_State < T > *currentState = nullptr;
  FSM_State < T > *nextState = nullptr;
  //下一个状态的枚举名称。
  FSM_StateName nextStateName = FSM_StateName::INVALID;
  /*状态切换信息，包括：是否完成；
                  切换持续时间；
                  切换阶段状态。*/
  TransitionData < T > transitionData;

private:
  std::unique_ptr<RobotControlParameters<T>> control_parameters_;  ///< 当前机型独立控制参数。
  std::unique_ptr < SafetyChecker < T >> safety_checker_;  ///< 状态切换前后共用的安全检查器。
  FSM_OperatingMode operating_mode_ = FSM_OperatingMode::NORMAL;  ///< 当前正常、过渡、急停或阻尼模式。
  std::size_t print_num_ = 10000;       ///< 两次周期性状态日志之间的控制周期数。
  std::size_t print_iteration_ = 0;     ///< 距离上次周期性日志经过的周期数。
  std::size_t iteration_ = 0;           ///< FSM 启动以来累计执行的控制周期数。
};

extern template class ControlFSM < float >;

#endif  // MYMIT_ROBOT_FSM_CONTROL_FSM_H_
