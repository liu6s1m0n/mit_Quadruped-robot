/*! @file floating_base_model_factory.hpp
 *  @brief Convert the generic quadruped parameter model into a floating-base tree.
 */

// 模型工厂：把通用四足参数按“浮动基座 + 每腿三转动关节”组装成 WBC 动力学树，
// 同时负责把状态估计和四腿反馈转换成该动力学模型要求的状态排列。

#ifndef MYMIT_ROBOT_MODEL_FLOATING_BASE_MODEL_FACTORY_HPP_
#define MYMIT_ROBOT_MODEL_FLOATING_BASE_MODEL_FACTORY_HPP_

#include <array>
#include <stdexcept>
#include <string>

#include "WBC/FloatingBaseModel.h"
#include "model/quadruped.hpp"

namespace model
{
namespace detail
{

template<typename Derived>
ori::CoordinateAxis coordinateAxis(const Eigen::MatrixBase<Derived> & axis)
{
  using T = typename Derived::Scalar;
  static_assert(
    Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
    "joint axis must be a 3-vector");
  constexpr double kAxisTolerance = 1e-6;
  if (axis.isApprox(Vec3<T>::UnitX(), T(kAxisTolerance))) {
    return ori::CoordinateAxis::X;
  }
  if (axis.isApprox(Vec3<T>::UnitY(), T(kAxisTolerance))) {
    return ori::CoordinateAxis::Y;
  }
  if (axis.isApprox(Vec3<T>::UnitZ(), T(kAxisTolerance))) {
    return ori::CoordinateAxis::Z;
  }
  throw std::invalid_argument(
          "FloatingBaseModel currently supports positive coordinate-axis joints only");
}

template<typename T>
SpatialInertia<T> spatialInertia(const RigidBodyInertia<T> & inertia)
{
  return SpatialInertia<T>(inertia.mass, inertia.center_of_mass, inertia.inertia_com);
}

inline const char * legName(LegId leg)
{
  constexpr std::array<const char *, kNumLegs> names{"FR", "FL", "RR", "RL"};
  return names.at(static_cast<std::size_t>(leg));
}

}  // namespace detail

/** Convert the project's estimator and leg feedback types to model state. */
template<typename T>
FBModelState<T> makeFloatingBaseState(
  const StateEstimate<T> & estimate,
  const std::array<JointState<T>, kNumLegs> & joint_states)
{
  if (!estimate.valid ||
    !estimate.orientation_world_from_body.coeffs().allFinite() ||
    !estimate.position_world.allFinite() || !estimate.velocity_body.allFinite() ||
    !estimate.angular_velocity_body.allFinite())
  {
    throw std::invalid_argument("state estimate is invalid or non-finite");
  }

  FBModelState<T> result;
  // 浮动基座状态顺序为四元数、世界系位置、机身系空间速度、12 维关节状态。
  result.bodyOrientation << estimate.orientation_world_from_body.w(),
    estimate.orientation_world_from_body.x(),
    estimate.orientation_world_from_body.y(),
    estimate.orientation_world_from_body.z();
  result.bodyPosition = estimate.position_world;
  result.bodyVelocity.template head<3>() = estimate.angular_velocity_body;
  result.bodyVelocity.template tail<3>() = estimate.velocity_body;
  result.q = DVec<T>::Zero(kNumJoints);
  result.qd = DVec<T>::Zero(kNumJoints);

  for (std::size_t leg_index = 0; leg_index < kNumLegs; ++leg_index) {
    const auto & joint_state = joint_states[leg_index];
    if (!joint_state.valid || static_cast<std::size_t>(joint_state.leg) != leg_index ||
      !joint_state.position.allFinite() || !joint_state.velocity.allFinite())
    {
      throw std::invalid_argument("joint feedback is invalid, non-finite, or out of leg order");
    }
    result.q.segment(leg_index * kJointsPerLeg, kJointsPerLeg) = joint_state.position;
    result.qd.segment(leg_index * kJointsPerLeg, kJointsPerLeg) = joint_state.velocity;
  }
  return result;
}

/**
 * Build the common 6+12 DOF floating-base tree from any compatible Quadruped.
 * Robot-specific numbers remain in that robot's parameter factory.
 */
template<typename T>
FloatingBaseModel<T> makeFloatingBaseModel(const Quadruped<T> & quadruped)
{
  FloatingBaseModel<T> result;
  result.addBase(detail::spatialInertia(quadruped.bodyInertia()));

  const Mat3<T> identity = Mat3<T>::Identity();
  constexpr int kFloatingBaseBodyId = 5;
  DVec<T> joint_position_offsets = DVec<T>::Zero(kNumJoints);

  for (const auto & leg : quadruped.legs()) {
    // 每条腿依次添加髋、腿和小腿刚体，父子关系与真实运动链一致。
    const std::string prefix = detail::legName(leg.leg);
    joint_position_offsets.segment(
      static_cast<Eigen::Index>(static_cast<std::size_t>(leg.leg) * kJointsPerLeg),
      kJointsPerLeg) = leg.joints.zero_offset;
    const int hip = result.addBody(
      detail::spatialInertia(leg.hip_inertia), leg.joints.armature[0],
      kFloatingBaseBodyId, spatial::JointType::Revolute,
      detail::coordinateAxis(leg.joints.joint_axes.col(0)),
      spatial::createSXform(identity, leg.hip_location_body), prefix + "_hip");

    // GO1 保留原来的标量表达式；DM1 使用从 URDF/MJCF 提取的三维关节偏移。
    const Vec3<T> thigh_offset =
      quadruped.robotType() == RobotType::UNITREE_GO1 ?
      Vec3<T>(T(0), quadruped.sideSign(leg.leg) * leg.hip_link_length, T(0)) :
      leg.hip_to_thigh;
    const int thigh = result.addBody(
      detail::spatialInertia(leg.thigh_inertia), leg.joints.armature[1], hip,
      spatial::JointType::Revolute,
      detail::coordinateAxis(leg.joints.joint_axes.col(1)),
      spatial::createSXform(identity, thigh_offset), prefix + "_thigh");

    const Vec3<T> calf_offset =
      quadruped.robotType() == RobotType::UNITREE_GO1 ?
      Vec3<T>(T(0), T(0), -leg.thigh_link_length) : leg.thigh_to_calf;
    const int calf = result.addBody(
      detail::spatialInertia(leg.calf_inertia), leg.joints.armature[2], thigh,
      spatial::JointType::Revolute,
      detail::coordinateAxis(leg.joints.joint_axes.col(2)),
      spatial::createSXform(identity, calf_offset), prefix + "_calf");

    const Vec3<T> foot_offset =
      quadruped.robotType() == RobotType::UNITREE_GO1 ?
      Vec3<T>(T(0), T(0), -leg.calf_link_length) : leg.calf_to_foot;
    result.addGroundContactPoint(
      // 足端接触点位于小腿末端，后续雅可比、MPC/WBC 都引用该注册顺序。
      calf, foot_offset, true);
  }

  result.setJointPositionOffsets(joint_position_offsets);
  result.check();
  return result;
}

}  // namespace model

#endif  // MYMIT_ROBOT_MODEL_FLOATING_BASE_MODEL_FACTORY_HPP_
