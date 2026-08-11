/**
 * @file SafetyChecker.h
 * @brief FSM 输入姿态与腿部输出命令的独立安全检查器。
 */
#ifndef MYMIT_ROBOT_FSM_SAFETY_CHECKER_H_
#define MYMIT_ROBOT_FSM_SAFETY_CHECKER_H_

#include "FSM/ControlFSMData.h"

template < typename T >
class SafetyChecker
{
public:
  explicit SafetyChecker(ControlFSMData < T > * data);

  bool checkSafeOrientation() const;
  bool checkPDesFoot();
  bool checkForceFeedForward();

private:
  ControlFSMData < T > *data_;
};

extern template class SafetyChecker < float >;

#endif  // MYMIT_ROBOT_FSM_SAFETY_CHECKER_H_
