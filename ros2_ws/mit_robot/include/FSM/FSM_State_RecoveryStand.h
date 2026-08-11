/**
 * @file FSM_State_RecoveryStand.h
 * @brief 机器人倒地或趴伏后，通过关节空间轨迹恢复到默认站姿。
 */
#ifndef MYMIT_ROBOT_FSM_STATE_RECOVERY_STAND_H_
#define MYMIT_ROBOT_FSM_STATE_RECOVERY_STAND_H_

#include <array>
#include <cstddef>

#include "FSM/FSM_State.h"

template < typename T >
class FSM_State_RecoveryStand final: public FSM_State < T >
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FSM_State_RecoveryStand(ControlFSMData < T > * control_fsm_data);
  ~FSM_State_RecoveryStand() override = default;

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;

private:
  enum class Motion
  {
    StandUp,
    FoldLegs,
    RollOver
  };

  using JointPositions = std::array < Vec3 < T >, kNumLegs >;

  void rollOver(std::size_t motion_iteration);
  void standUp(std::size_t motion_iteration);
  void foldLegs(std::size_t motion_iteration);
  void setJointPositionInterpolated(
    std::size_t current_iteration, std::size_t maximum_iteration,
    std::size_t leg, const Vec3 < T > & initial, const Vec3 < T > & final);
  void startMotion(Motion motion, const JointPositions & initial_positions);
  bool upsideDown() const noexcept;
  std::size_t iterationsFor(T duration_seconds) const noexcept;

  std::size_t state_iteration_ = 0;
  std::size_t motion_start_iteration_ = 0;
  Motion motion_ = Motion::FoldLegs;
  bool recovery_complete_ = false;

  JointPositions fold_joint_positions_ {};
  JointPositions stand_joint_positions_ {};
  JointPositions rolling_joint_positions_ {};
  JointPositions initial_joint_positions_ {};

  std::size_t rollover_ramp_iterations_ = 1;
  std::size_t rollover_settle_iterations_ = 1;
  std::size_t fold_ramp_iterations_ = 1;
  std::size_t fold_settle_iterations_ = 1;
  std::size_t standup_ramp_iterations_ = 1;
};

extern template class FSM_State_RecoveryStand < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_RECOVERY_STAND_H_
