/**
 * @file FSM_State_Locomotion.h
 * @brief 行走状态：MPC 规划接触力和落脚运动，WBC 将结果转换为关节命令。
 */
#ifndef MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_
#define MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_

#include <array>
#include <memory>

#include "FSM/FSM_State.h"
#include "MPC/ConvexMPCLocomotion.h"
#include "WBC/LocomotionCtrl/LocomotionCtrl.hpp"
#include "controller/FootSwingTrajectory.hpp"

template < typename T >
class FSM_State_Locomotion: public FSM_State < T >
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit FSM_State_Locomotion(ControlFSMData < T > * control_fsm_data);
  ~FSM_State_Locomotion() override = default;

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;
  /** 将用户层速度命令传递给本状态持有的 MPC。 */
  void setForwardVelocity(T velocity);

  /** 返回最近一次交给 WBC 的单腿世界系足端目标，便于运行时诊断。 */
  const Vec3 < T > & footPositionTargetWorld(LegId leg) const noexcept
  {
    return wbc_data_.pFoot_des[static_cast < std::size_t > (leg)];
  }

  /** 返回指定腿是否正沿已锁存的摆动轨迹运动。 */
  bool swingLegActive(LegId leg) const noexcept
  {
    return swing_active_[static_cast < std::size_t > (leg)];
  }

private:
  void LocomotionControlStep();
  bool locomotionSafe() const;
  std::array < Vec3 < T >, kNumLegs > footPositionsWorld() const;
  void resetSwingTrajectories() noexcept;
  void startSwingTrajectory(
    std::size_t leg, const Vec3 < T > & initial_position,
    const mpc::LocomotionResult < T > & locomotion_result);

  std::unique_ptr < mpc::ConvexMPCLocomotion < T >> mpc_;
  std::unique_ptr < LocomotionCtrl < T >> wbc_ctrl_;
  LocomotionCtrlData < T > wbc_data_;
  std::array < FootSwingTrajectory < T >, kNumLegs > swing_trajectories_ {};
  std::array < bool, kNumLegs > swing_active_ {};
  T swing_height_ = T(0.06);
  T maximum_step_length_ = T(0.18);
  std::size_t iteration_ = 0;
};

extern template class FSM_State_Locomotion < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_
