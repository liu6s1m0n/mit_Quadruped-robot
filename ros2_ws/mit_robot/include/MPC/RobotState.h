/**
 * @file RobotState.h
 * @brief MPC 使用的紧凑机器人状态，与完整估计器数据解耦。
 *
 * 位置、线速度和足端位置统一使用世界坐标系，姿态速度使用 ZYX RPY 导数，
 * 旋转矩阵明确表示 body 到 world。
 */
#ifndef MYMIT_ROBOT_MPC_ROBOT_STATE_H_
#define MYMIT_ROBOT_MPC_ROBOT_STATE_H_

#include <array>

#include "model/quadruped.hpp"
#include "model/robot_types.hpp"

namespace mpc
{

template<typename T>
struct RobotState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vec3<T> position_world = Vec3<T>::Zero();
  Vec3<T> velocity_world = Vec3<T>::Zero();
  Vec3<T> rpy = Vec3<T>::Zero();
  Vec3<T> rpy_rate = Vec3<T>::Zero();
  Mat3<T> rotation_world_from_body = Mat3<T>::Identity();
  std::array<Vec3<T>, kNumLegs> foot_position_world{};
  T timestamp = T(0);
  bool valid = false;

  /**
   * @brief 将状态估计和足端位置整理为 MPC 状态。
   * @param estimate 浮动基座状态估计，包含位置、速度、姿态和时间戳。
   * @param feet_world 四条腿足端在世界坐标系中的位置。
   * @return MPC 使用的紧凑状态对象。
   */
  static RobotState fromEstimate(
    const StateEstimate<T> & estimate,
    const std::array<Vec3<T>, kNumLegs> & feet_world);

  /**
   * @brief 将机身坐标系角速度映射为 ZYX 欧拉角导数。
   *
   * 对 @f$\phi,\theta,\psi@f$ 使用
   * @f$\dot{rpy}=E(rpy)\omega_{body}@f$。当俯仰角接近
   * @f$\pm\pi/2@f$ 时矩阵接近奇异，函数返回 NaN 向量。
   *
   * @param rpy 当前 ZYX 欧拉角。
   * @param angular_velocity_body 机身坐标系角速度。
   * @return 欧拉角导数。
   */
  static Vec3<T> rpyRateFromBodyAngularVelocity(
    const Vec3<T> & rpy, const Vec3<T> & angular_velocity_body);

  /** @brief 检查状态标志、姿态矩阵、足端位置和全部数值是否有效。 */
  bool isValid() const noexcept;
};

extern template struct RobotState<float>;
extern template struct RobotState<double>;

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_ROBOT_STATE_H_
