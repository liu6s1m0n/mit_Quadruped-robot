/*! @file unitree_go1.hpp
 *  @brief Unitree GO1 的专属模型参数。
 *
 *  数值来自当前 unitree_go1/go1.xml。修改或重新标定 GO1 时只改本文件，
 *  不影响 Quadruped 的通用接口，也不影响其他机器人参数。
 */

#ifndef MYMIT_ROBOT_MODEL_ROBOTS_UNITREE_GO1_HPP_
#define MYMIT_ROBOT_MODEL_ROBOTS_UNITREE_GO1_HPP_

#include "model/quadruped.hpp"

namespace robots
{
namespace unitree_go1
{

template<typename T>
JointModelParameters<T> makeJointParameters()
{
  JointModelParameters<T> result;
  result.lower_limit << T(-0.863), T(-0.686), T(-2.818);
  result.upper_limit << T(0.863), T(4.501), T(-0.888);
  result.velocity_limit << T(30.1), T(30.1), T(20.06);
  result.torque_limit << T(23.7), T(23.7), T(35.55);
  result.damping << T(1), T(2), T(2);
  result.friction_loss.setConstant(T(0.2));
  result.armature.setConstant(T(0.01));
  result.home_position << T(0), T(0.9), T(-1.8);
  result.joint_axes.col(0) = Vec3<T>::UnitX();
  result.joint_axes.col(1) = Vec3<T>::UnitY();
  result.joint_axes.col(2) = Vec3<T>::UnitY();
  return result;
}

template<typename T>
LegModelParameters<T> makeLeg(
  LegId leg_id, const Vec3<T> & hip_location, bool is_left, bool is_rear)
{
  LegModelParameters<T> result;
  result.leg = leg_id;
  result.hip_location_body = hip_location;
  result.hip_link_length = T(0.08);
  result.thigh_link_length = T(0.213);
  result.calf_link_length = T(0.213);
  result.foot_radius = T(0.023);
  result.foot_friction << T(0.8), T(0.02), T(0.01);
  result.joints = makeJointParameters<T>();

  // GO1 的 Hip 惯量主轴会同时随前后、左右安装位置发生镜像。
  const T hip_com_x = is_rear ? T(0.0049166) : T(-0.0049166);
  const T hip_com_y = is_left ? T(-0.00762615) : T(0.00762615);
  Eigen::Quaternion<T> hip_quaternion;
  if (!is_rear && !is_left) {
    hip_quaternion = Eigen::Quaternion<T>(T(0.507341), T(0.514169), T(0.495027), T(0.482891));
  } else if (!is_rear && is_left) {
    hip_quaternion = Eigen::Quaternion<T>(T(0.482891), T(0.495027), T(0.514169), T(0.507341));
  } else if (is_rear && !is_left) {
    hip_quaternion = Eigen::Quaternion<T>(T(0.495027), T(0.482891), T(0.507341), T(0.514169));
  } else {
    hip_quaternion = Eigen::Quaternion<T>(T(0.514169), T(0.507341), T(0.482891), T(0.495027));
  }

  result.hip_inertia = makeRigidBodyInertia(
    T(0.68), Vec3<T>(hip_com_x, hip_com_y, T(-8.865e-05)),
    hip_quaternion,
    Vec3<T>(T(0.000734064), T(0.000468438), T(0.000398719)));

  // GO1 大腿只需按左右侧选择镜像后的质心和惯性主轴。
  const T thigh_com_y = is_left ? T(-0.019315) : T(0.019315);
  const Eigen::Quaternion<T> thigh_quaternion = is_left ?
    Eigen::Quaternion<T>(T(0.753383), T(0.0775126), T(-0.0272313), T(0.65243)) :
    Eigen::Quaternion<T>(T(0.65243), T(-0.0272313), T(0.0775126), T(0.753383));
  result.thigh_inertia = makeRigidBodyInertia(
    T(1.009), Vec3<T>(T(-0.00304722), thigh_com_y, T(-0.0305004)),
    thigh_quaternion,
    Vec3<T>(T(0.00478717), T(0.00460903), T(0.000709268)));

  result.calf_inertia = makeRigidBodyInertia(
    T(0.195862),
    Vec3<T>(T(0.00429862), T(0.000976676), T(-0.146197)),
    Eigen::Quaternion<T>(T(0.691246), T(0.00357467), T(0.00511118), T(0.722592)),
    Vec3<T>(T(0.00149767), T(0.00148468), T(3.58427e-05)));
  return result;
}

/** 创建一套完整且经过一致性检查的 GO1 参数。 */
template<typename T>
Quadruped<T> makeModel()
{
  const auto body = makeRigidBodyInertia(
    T(5.204), Vec3<T>(T(0.0223), T(0.002), T(-0.0005)),
    Eigen::Quaternion<T>(
      T(-0.00342088), T(0.705204),
      T(0.000106698), T(0.708996)),
    Vec3<T>(T(0.0716565), T(0.0630105), T(0.0168101)));

  typename Quadruped<T>::LegArray legs;
  legs[static_cast<std::size_t>(LegId::FR)] = makeLeg(
    LegId::FR, Vec3<T>(T(0.1881), T(-0.04675), T(0)), false, false);
  legs[static_cast<std::size_t>(LegId::FL)] = makeLeg(
    LegId::FL, Vec3<T>(T(0.1881), T(0.04675), T(0)), true, false);
  legs[static_cast<std::size_t>(LegId::RR)] = makeLeg(
    LegId::RR, Vec3<T>(T(-0.1881), T(-0.04675), T(0)), false, true);
  legs[static_cast<std::size_t>(LegId::RL)] = makeLeg(
    LegId::RL, Vec3<T>(T(-0.1881), T(0.04675), T(0)), true, true);

  return Quadruped<T>(RobotType::UNITREE_GO1, body, legs, T(0.27));
}

}  // namespace unitree_go1
}  // namespace robots

#endif  // MYMIT_ROBOT_MODEL_ROBOTS_UNITREE_GO1_HPP_
