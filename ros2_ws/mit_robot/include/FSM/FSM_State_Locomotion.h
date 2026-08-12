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
  //ControlFSMData<T>  状态估计器；腿部控制器；四足机器人模型；
  //期望状态；电机命令；用户控制输入；FSM 数据
  explicit FSM_State_Locomotion(ControlFSMData < T > * control_fsm_data);
  ~FSM_State_Locomotion() override = default;

  void onEnter() override;
  void run() override;
  FSM_StateName checkTransition() override;
  TransitionData < T > transition() override;
  void onExit() override;
  /** 将用户层速度命令传递给本状态持有的 MPC。 */
  void setForwardVelocity(T velocity);
  /** 设置机身系前后、左右和自转速度。 */
  void setVelocityCommand(T forward_velocity, T lateral_velocity, T yaw_rate);

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
  //核心函数
  void LocomotionControlStep();
  //检查当前是否适合继续行走。
  bool locomotionSafe() const;
  std::array < Vec3 < T >, kNumLegs > footPositionsWorld() const;
  void resetSwingTrajectories() noexcept;
  void startSwingTrajectory(
    std::size_t leg, const Vec3 < T > & initial_position,
    const mpc::LocomotionResult < T > & locomotion_result);
  /*MPC 控制器。
    负责输出：身体期望状态；每条腿的地面反作用力；接触状态；
    摆动相位；摆动时间；支撑时间。*/
  std::unique_ptr < mpc::ConvexMPCLocomotion < T >> mpc_;
  /*行走专用 WBC 控制器。
    它会把：身体任务；足端摆动任务；支撑腿接触约束；MPC 反作用力；
    转换成关节控制命令。*/
  std::unique_ptr < LocomotionCtrl < T >> wbc_ctrl_;
  /*这是 MPC 到 WBC 的数据中间层。
    包含：pBody_des,vBody_des,aBody_des,pBody_RPY_des,vBody_Ori_des
    以及每条腿的：pFoot_des,vFoot_des,aFoot_des,Fr_des,contact_state*/
  LocomotionCtrlData < T > wbc_data_;
  /*四条腿各自拥有一条摆动轨迹对象。每条腿单独保存：
  起点；终点；摆动高度；当前相位；位置；速度；加速度。*/
  std::array < FootSwingTrajectory < T >, kNumLegs > swing_trajectories_ {};
  /*记录每条腿是否已经开始当前摆动周期。*/
  std::array < bool, kNumLegs > swing_active_ {};
  /*参数1：摆动腿抬脚高度。保持 0.10 m，不再用增加高度间接提高速度。*/
  T swing_height_ = T(0.10);
  /*参数2：单个步周期内最大水平步长为 18 cm。 -> 0.20*/
  T maximum_step_length_ = T(0.20);
  /*只提高摆动腿的关节速度前馈：Hip保持原速以稳定支撑宽度，
    thigh/calf提高50%；最终仍按GO1关节速度上限裁剪。*/
  Vec3<T> swing_joint_velocity_scale_ =
    Vec3<T>(T(1), T(1.5), T(1.5));
  std::size_t iteration_ = 0;
};

extern template class FSM_State_Locomotion < float >;

#endif  // MYMIT_ROBOT_FSM_STATE_LOCOMOTION_H_
