/**
 * @file FSM_State.h
 * @brief 所有控制状态的抽象基类，规定进入、运行、检查切换和退出四个阶段。
 *
 * 状态切换采用“先检查、再过渡”的结构，便于以后加入安全检查而不改具体算法。
 */
#ifndef MYMIT_ROBOT_FSM_STATE_H_
#define MYMIT_ROBOT_FSM_STATE_H_

#include <string>
#include <utility>

#include "FSM/ControlFSMData.h"
#include "Utilities/cppTypes.h"

enum class FSM_StateName
{
  INVALID,
  PASSIVE,
  JOINT_PD,
  STAND_UP,
  RECOVERY_STAND,
  BALANCE_STAND,
  LOCOMOTION,
  FRONT_JUMP
};

template < typename T >
struct TransitionData
{
  bool done = false;  ///< 当前状态到下一状态的过渡动作是否已经完成。
  void zero() noexcept {done = false;}
};

template < typename T >
class FSM_State
{
public:
  FSM_State(
    ControlFSMData < T > *data, FSM_StateName state_name,
    std::string state_string)
    : stateName(state_name), stateString(std::move(state_string)),
    nextStateName(state_name), _data(data) {
  }

  virtual ~FSM_State() = default;
  virtual void onEnter() = 0;
  virtual void run() = 0;
  virtual FSM_StateName checkTransition() = 0;
  virtual TransitionData < T > transition() = 0;
  virtual void onExit() = 0;

  void turnOnAllSafetyChecks() noexcept
  {
    checkSafeOrientation = true;
    checkPDesFoot = true;
    checkForceFeedForward = true;
  }
  void turnOffAllSafetyChecks() noexcept
  {
    checkSafeOrientation = false;
    checkPDesFoot = false;
    checkForceFeedForward = false;
  }

  FSM_StateName stateName;      ///< 当前对象代表的固定状态枚举。
  std::string stateString;      ///< 用于日志和界面显示的状态名称。
  FSM_StateName nextStateName;  ///< checkTransition() 本周期选择的目标状态。
  T transitionDuration = T(0);  ///< 状态过渡允许或计划使用的时间，s。
  TransitionData < T > transitionData;  ///< 本次过渡的完成标志等运行数据。
  bool checkSafeOrientation = false;     ///< 是否在该状态启用机身姿态安全检查。
  bool checkPDesFoot = false;            ///< 是否检查期望足端位置是否越界。
  bool checkForceFeedForward = false;    ///< 是否检查足端前馈力是否安全。

protected:
  ControlFSMData < T > *_data;  ///< 非拥有指针，指向所有状态共享的控制数据。
};

#endif  // MYMIT_ROBOT_FSM_STATE_H_
