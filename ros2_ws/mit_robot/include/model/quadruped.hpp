/*! @file quadruped.hpp
 *  @brief 与具体机型无关的四足机器人参数结构。
 *
 *  本文件只定义控制算法共同使用的数据形状，不保存 GO1 或其他机器狗的
 *  具体数值。每一种机器狗的参数应放在独立的 robots/<model>.hpp 中。
 *  当前通用拓扑固定为四腿，每腿三个关节，顺序由 robot_types.hpp 规定。
 */

#ifndef MYMIT_ROBOT_MODEL_QUADRUPED_HPP_
#define MYMIT_ROBOT_MODEL_QUADRUPED_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <eigen3/Eigen/Geometry>

#include "cppTypes.h"
#include "model/robot_types.hpp"

/** 一个刚体绕质心的质量和惯量参数。 */
template<typename T>
struct RigidBodyInertia
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  T mass = T(0);                              ///< 质量，kg
  Vec3<T> center_of_mass = Vec3<T>::Zero();  ///< 质心，表达在连杆坐标系中，m
  Mat3<T> inertia_com = Mat3<T>::Zero();      ///< 绕质心惯量，表达在连杆坐标系中

  bool isValid() const
  {
    return std::isfinite(static_cast<float>(mass)) && mass > T(0) &&
           center_of_mass.allFinite() && inertia_com.allFinite();
  }
};

/**
 * @brief 单腿三个关节的模型和安全参数。
 *
 * 所有三维向量的顺序均为 [Hip, thigh, calf]。这些字段是跨机型的公共
 * 接口；具体数值由各机型参数文件填写。
 */
template<typename T>
struct JointModelParameters
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vec3<T> lower_limit = Vec3<T>::Zero();      ///< 位置下限，rad
  Vec3<T> upper_limit = Vec3<T>::Zero();      ///< 位置上限，rad
  Vec3<T> velocity_limit = Vec3<T>::Zero();   ///< 速度绝对值上限，rad/s
  Vec3<T> torque_limit = Vec3<T>::Zero();     ///< 扭矩绝对值上限，N*m
  Vec3<T> damping = Vec3<T>::Zero();          ///< 关节粘性阻尼
  Vec3<T> friction_loss = Vec3<T>::Zero();    ///< 关节库仑摩擦
  Vec3<T> armature = Vec3<T>::Zero();         ///< 折算到关节侧的转子惯量
  Vec3<T> home_position = Vec3<T>::Zero();    ///< 默认站立关节角，rad
  Mat3<T> joint_axes = Mat3<T>::Zero();       ///< 每列为对应关节的局部转轴

  bool isValid() const
  {
    return lower_limit.allFinite() && upper_limit.allFinite() &&
           velocity_limit.allFinite() && torque_limit.allFinite() &&
           damping.allFinite() && friction_loss.allFinite() &&
           armature.allFinite() && home_position.allFinite() &&
           joint_axes.allFinite() &&
           (lower_limit.array() < upper_limit.array()).all() &&
           (velocity_limit.array() > T(0)).all() &&
           (torque_limit.array() > T(0)).all() &&
           (home_position.array() >= lower_limit.array()).all() &&
           (home_position.array() <= upper_limit.array()).all();
  }
};

/** @brief 一条腿的几何、惯量、接触和关节参数。 */
template<typename T>
struct LegModelParameters
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LegId leg = LegId::FR;
  Vec3<T> hip_location_body = Vec3<T>::Zero();  ///< Hip 在机身坐标系中的位置，m
  T hip_link_length = T(0);                     ///< Hip 横向连杆长度，m
  T thigh_link_length = T(0);                   ///< 大腿长度，m
  T calf_link_length = T(0);                    ///< 小腿长度，m
  T foot_radius = T(0);                         ///< 足端碰撞球半径，m
  Vec3<T> foot_friction = Vec3<T>::Zero();      ///< 滑动、扭转、滚动摩擦

  RigidBodyInertia<T> hip_inertia;
  RigidBodyInertia<T> thigh_inertia;
  RigidBodyInertia<T> calf_inertia;
  JointModelParameters<T> joints;

  T maximumLegLength() const
  {
    return thigh_link_length + calf_link_length;
  }

  bool isValid() const
  {
    return hip_location_body.allFinite() && hip_link_length > T(0) &&
           thigh_link_length > T(0) && calf_link_length > T(0) &&
           foot_radius > T(0) && foot_friction.allFinite() &&
           hip_inertia.isValid() && thigh_inertia.isValid() &&
           calf_inertia.isValid() && joints.isValid();
  }
};

/**
 * @brief 与具体品牌无关的四足机器人参数容器。
 *
 * Quadruped 不知道参数来自 MJCF、URDF 还是真机标定文件。创建完成后仅提供
 * 只读访问，因此控制循环中不会意外修改模型。不同机器人的控制算法可以
 * 使用同一套接口，只在程序启动时选择不同的参数工厂。
 */
template<typename T>
class Quadruped
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using LegArray = std::array<LegModelParameters<T>, kNumLegs>;

  /**
   * @brief 使用一套完整参数创建机器人模型。
   *
   * 构造函数故意不提供默认模型，防止忘记选择机型时静默使用 GO1 参数。
   */
  Quadruped(
    RobotType robot_type, const RigidBodyInertia<T> & body_inertia,
    const LegArray & legs, T nominal_body_height)
  : robot_type_(robot_type),
    body_inertia_(body_inertia),
    legs_(legs),
    nominal_body_height_(nominal_body_height)
  {
    if (!isValid()) {
      throw std::invalid_argument("invalid quadruped model parameters");
    }
  }

  RobotType robotType() const noexcept {return robot_type_;}
  const RigidBodyInertia<T> & bodyInertia() const noexcept {return body_inertia_;}
  T nominalBodyHeight() const noexcept {return nominal_body_height_;}

  const LegModelParameters<T> & leg(LegId leg_id) const
  {
    return legs_.at(legArrayIndex(leg_id));
  }

  const LegArray & legs() const noexcept {return legs_;}

  Vec3<T> hipLocation(LegId leg_id) const
  {
    return leg(leg_id).hip_location_body;
  }

  T sideSign(LegId leg_id) const noexcept
  {
    return (leg_id == LegId::FL || leg_id == LegId::RL) ? T(1) : T(-1);
  }

  bool isFrontLeg(LegId leg_id) const noexcept
  {
    return leg_id == LegId::FR || leg_id == LegId::FL;
  }

  T totalMass() const
  {
    T mass = body_inertia_.mass;
    for (const auto & parameters : legs_) {
      mass += parameters.hip_inertia.mass;
      mass += parameters.thigh_inertia.mass;
      mass += parameters.calf_inertia.mass;
    }
    return mass;
  }

  bool isValid() const
  {
    // 这里不能写死某个 RobotType，否则新增机型会被通用层拒绝。
    if (!body_inertia_.isValid() ||
      !std::isfinite(static_cast<float>(nominal_body_height_)) ||
      nominal_body_height_ <= T(0))
    {
      return false;
    }

    for (std::size_t index = 0; index < legs_.size(); ++index) {
      // 除参数有效外，还保证数组位置确实对应 FR、FL、RR、RL。
      if (!legs_[index].isValid() ||
        static_cast<std::size_t>(legs_[index].leg) != index)
      {
        return false;
      }
    }
    return true;
  }

private:
  static constexpr std::size_t legArrayIndex(LegId leg_id) noexcept
  {
    return static_cast<std::size_t>(leg_id);
  }

  RobotType robot_type_;
  RigidBodyInertia<T> body_inertia_;
  LegArray legs_;
  T nominal_body_height_;
};

/**
 * @brief 根据机器人型号创建完整参数模型。
 *
 * 工厂实现位于 quadruped.cpp，当前公共模型库预编译 float 精度。
 * 增加新机型时，在 model/robots/ 中编写参数，并在 quadruped.cpp 的
 * switch 中登记；调用者不需要包含具体机型的参数头文件。
 */
template<typename T>
Quadruped<T> makeQuadruped(RobotType robot_type);

/**
 * @brief 将 MJCF 的“惯性主轴四元数 + 主惯量”转换到连杆坐标系。
 *
 * 该转换本身与机器人品牌无关，所以保留在通用模型层供所有机型复用。
 */
template<typename T>
RigidBodyInertia<T> makeRigidBodyInertia(
  T mass, const Vec3<T> & center_of_mass,
  Eigen::Quaternion<T> principal_axes_quaternion,
  const Vec3<T> & diagonal_inertia)
{
  principal_axes_quaternion.normalize();
  const Mat3<T> rotation = principal_axes_quaternion.toRotationMatrix();

  RigidBodyInertia<T> result;
  result.mass = mass;
  result.center_of_mass = center_of_mass;
  result.inertia_com = rotation * diagonal_inertia.asDiagonal() * rotation.transpose();
  return result;
}


#endif  // MYMIT_ROBOT_MODEL_QUADRUPED_HPP_
