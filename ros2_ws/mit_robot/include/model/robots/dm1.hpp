/*! @file dm1.hpp
 *  @brief 达妙 DM1（源工程 dmgo）的机器人参数。
 *
 * 尺寸、质量、惯量、关节范围和电机限制来自
 * legged_damiao_description/urdf/dmgo/const.xacro。控制框架继续使用统一的
 * FR/FL/RR/RL 与 Hip/thigh/calf 顺序，分别映射源模型的 RF/LF/RH/LH 和
 * HAA/HFE/KFE。
 */
#ifndef MYMIT_ROBOT_MODEL_ROBOTS_DM1_HPP_
#define MYMIT_ROBOT_MODEL_ROBOTS_DM1_HPP_

#include "model/quadruped.hpp"

namespace robots
{
namespace dm1
{

template<typename T>
JointModelParameters<T> makeJointParameters(bool is_rear)
{
  JointModelParameters<T> result;
  // Home 是正常站姿沿竖直方向降到底的深蹲：HAA 保持中立，膝关节完全
  // 折叠；大腿角使小腿中心线平行地面，不再只有球形足端承担接触。
  const T thigh_zero_offset = T(-0.203);
  result.zero_offset << T(0), thigh_zero_offset, T(-2.25);
  result.lower_limit << T(-1.57), T(-1.57) - thigh_zero_offset, T(0);
  result.upper_limit << T(1.57), T(2.4) - thigh_zero_offset, T(2.75);
  result.velocity_limit << T(52.4), T(28.6), T(28.6);
  result.torque_limit << T(52.4), T(55), T(55);
  result.damping.setConstant(T(1));
  result.friction_loss.setConstant(T(0.1));
  result.armature.setConstant(T(0.01));
  // 站姿机械角保持不变；这里保存减去新趴卧零位偏置后的电机读数。
  result.home_position <<
    T(0), T(-0.597),
    is_rear ? T(1.468) : T(1.432);
  result.joint_axes.col(0) = Vec3<T>::UnitX();
  result.joint_axes.col(1) = Vec3<T>::UnitY();
  result.joint_axes.col(2) = Vec3<T>::UnitY();
  return result;
}

template<typename T>
RigidBodyInertia<T> makeInertia(
  T mass, const Vec3<T> & center, T ixx, T iyy, T izz,
  T ixy = T(0), T ixz = T(0), T iyz = T(0))
{
  RigidBodyInertia<T> result;
  result.mass = mass;
  result.center_of_mass = center;
  result.inertia_com << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
  return result;
}

template<typename T>
LegModelParameters<T> makeLeg(LegId leg_id, bool is_left, bool is_rear)
{
  const T side = is_left ? T(1) : T(-1);
  const T longitudinal = is_rear ? T(-1) : T(1);
  LegModelParameters<T> result;
  result.leg = leg_id;
  result.hip_location_body = Vec3<T>(
    longitudinal * T(0.16175), side * T(0.059997), T(-0.061498));
  // 通用三连杆模型使用源 URDF 的关节间有效距离。
  result.hip_link_length = T(0.097753);
  result.thigh_link_length = T(0.2);
  result.calf_link_length = T(0.225);
  result.hip_to_thigh = Vec3<T>(
    longitudinal * T(0.049998), side * T(0.019503), T(0));
  result.thigh_to_calf = Vec3<T>(T(-0.2), side * T(0.07825), T(0));
  result.calf_to_foot = Vec3<T>(T(-0.17362), T(0), T(-0.14312));
  result.foot_radius = T(0.02);
  result.foot_friction << T(0.8), T(0.02), T(0.01);
  result.joints = makeJointParameters<T>(is_rear);

  result.hip_inertia = makeInertia(
    T(0.706316),
    Vec3<T>(longitudinal * T(0.04756), -side * T(0.010434), T(-0.000046)),
    T(0.000403), T(0.000535), T(0.000446));
  result.thigh_inertia = makeInertia(
    T(0.977146), Vec3<T>(T(-0.023554), side * T(0.050542), T(0.001956)),
    T(0.000887), T(0.003756), T(0.003918));
  // 足部通过固定关节连接到小腿，动力学参数中将其质量并入小腿。
  result.calf_inertia = makeInertia(
    T(0.270521), Vec3<T>(T(-0.010235), T(0.000248), T(-0.054204)),
    T(0.000977), T(0.005321), T(0.004367));
  return result;
}

/** @brief 创建完整 DM1 参数模型。 */
template<typename T>
Quadruped<T> makeModel()
{
  const auto body = makeInertia(
    T(6.889103), Vec3<T>(T(-0.001955), T(0.002743), T(-0.068608)),
    T(0.035976), T(0.081109), T(0.103076),
    T(0.000077), T(-0.000148), T(0.000272));

  typename Quadruped<T>::LegArray legs;
  legs[static_cast<std::size_t>(LegId::FR)] = makeLeg<T>(LegId::FR, false, false);
  legs[static_cast<std::size_t>(LegId::FL)] = makeLeg<T>(LegId::FL, true, false);
  legs[static_cast<std::size_t>(LegId::RR)] = makeLeg<T>(LegId::RR, false, true);
  legs[static_cast<std::size_t>(LegId::RL)] = makeLeg<T>(LegId::RL, true, true);
  return Quadruped<T>(RobotType::DM_BOT1, body, legs, T(0.39));
}

}  // namespace dm1
}  // namespace robots

#endif  // MYMIT_ROBOT_MODEL_ROBOTS_DM1_HPP_
