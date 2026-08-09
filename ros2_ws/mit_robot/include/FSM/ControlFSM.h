#ifndef MYMIT_ROBOT_FSM_CONTROL_FSM_H_
#define MYMIT_ROBOT_FSM_CONTROL_FSM_H_

#include <array>
#include <cstddef>
#include <memory>

#include "FSM/FSM_State.h"
#include "FSM/FSM_State_BalanceStand.h"
#include "FSM/FSM_State_Locomotion.h"

enum class FSM_OperatingMode
{
  NORMAL,
  TRANSITIONING,
  ESTOP,
  EDAMP
};

template < typename T >
struct FSM_StatesList
{
  std::unique_ptr < FSM_State < T >> passive;
  std::unique_ptr < FSM_State < T >> joint_pd;
  std::unique_ptr < FSM_State_BalanceStand < T >> balance_stand;
  std::unique_ptr < FSM_State_Locomotion < T >> locomotion;
};

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

  ControlFSM(const ControlFSM &) = delete;
  ControlFSM & operator = (const ControlFSM &) = delete;

  void initialize();
  void runFSM();
  FSM_OperatingMode safetyPreCheck();
  FSM_OperatingMode safetyPostCheck();
  FSM_State < T > *getNextState(FSM_StateName state_name) noexcept;
  void printInfo(int option);

  FSM_StateName currentStateName() const noexcept;
  FSM_OperatingMode operatingMode() const noexcept {return operating_mode_;}
  void setUseWbc(bool enabled) noexcept {data.use_wbc = enabled;}

  ControlFSMData < T > data;
  FSM_StatesList < T > statesList;
  FSM_State < T > *currentState = nullptr;
  FSM_State < T > *nextState = nullptr;
  FSM_StateName nextStateName = FSM_StateName::INVALID;
  TransitionData < T > transitionData;

private:
  FSM_OperatingMode operating_mode_ = FSM_OperatingMode::NORMAL;
  std::size_t print_num_ = 10000;
  std::size_t print_iteration_ = 0;
  std::size_t iteration_ = 0;
};

extern template class ControlFSM < float >;

#endif  // MYMIT_ROBOT_FSM_CONTROL_FSM_H_
