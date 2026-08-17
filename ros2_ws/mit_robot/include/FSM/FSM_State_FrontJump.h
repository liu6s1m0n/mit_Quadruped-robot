/**
 * @file FSM_State_FrontJump.h
 * @brief 与当前 GO1 控制链兼容的一次性向前跳状态。
 */
#ifndef MYMIT_ROBOT_FSM_STATE_FRONT_JUMP_H_
#define MYMIT_ROBOT_FSM_STATE_FRONT_JUMP_H_

#include <array>
#include <cstddef>

#include "FSM/FSM_State.h"

template<typename T>
class FSM_State_FrontJump final : public FSM_State<T>
{
public:
  explicit FSM_State_FrontJump(ControlFSMData<T> * control_fsm_data);

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData<T> transition() override;
  void onExit() override;

  bool complete() const noexcept {return jump_complete_;}

private:
  void applyTrajectory(
    const std::array<Vec3<T>, kNumLegs> & start,
    const std::array<Vec3<T>, kNumLegs> & target,
    std::size_t phase_iteration, std::size_t phase_iterations,
    T kp, T kd);
  static T smoothstep(T progress) noexcept;
  static T smoothstepRate(T progress, T duration) noexcept;

  std::array<Vec3<T>, kNumLegs> initial_positions_{};
  std::array<Vec3<T>, kNumLegs> crouch_positions_{};
  std::array<Vec3<T>, kNumLegs> thrust_positions_{};
  std::array<Vec3<T>, kNumLegs> tuck_positions_{};
  std::array<Vec3<T>, kNumLegs> landing_positions_{};
  std::size_t iteration_ = 0;
  std::size_t crouch_iterations_ = 1;
  std::size_t thrust_iterations_ = 1;
  std::size_t tuck_iterations_ = 1;
  std::size_t landing_iterations_ = 1;
  bool jump_complete_ = false;
};

extern template class FSM_State_FrontJump<float>;

#endif  // MYMIT_ROBOT_FSM_STATE_FRONT_JUMP_H_
