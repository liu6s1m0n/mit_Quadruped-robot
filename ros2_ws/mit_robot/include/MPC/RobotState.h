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

  static RobotState fromEstimate(
    const StateEstimate<T> & estimate,
    const std::array<Vec3<T>, kNumLegs> & feet_world);

  /** 将机身系角速度准确映射为 ZYX 欧拉角导数。 */
  static Vec3<T> rpyRateFromBodyAngularVelocity(
    const Vec3<T> & rpy, const Vec3<T> & angular_velocity_body);

  bool isValid() const noexcept;
};

extern template struct RobotState<float>;
extern template struct RobotState<double>;

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_ROBOT_STATE_H_
