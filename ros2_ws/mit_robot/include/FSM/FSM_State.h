#ifndef MYMIT_ROBOT_FSM_STATE_H_
#define MYMIT_ROBOT_FSM_STATE_H_

#include <string>
#include <utility>

#include "FSM/ControlFSMData.h"
#include "cppTypes.h"

enum class FSM_StateName
{
  INVALID,
  PASSIVE,
  JOINT_PD,
  BALANCE_STAND,
  LOCOMOTION
};

template < typename T >
struct TransitionData
{
  bool done = false;
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

  FSM_StateName stateName;
  std::string stateString;
  FSM_StateName nextStateName;
  T transitionDuration = T(0);
  TransitionData < T > transitionData;
  bool checkSafeOrientation = false;
  bool checkPDesFoot = false;
  bool checkForceFeedForward = false;

protected:
  ControlFSMData < T > *_data;
};

#endif  // MYMIT_ROBOT_FSM_STATE_H_
