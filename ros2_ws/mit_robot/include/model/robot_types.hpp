/*! @file robot_types.hpp
 *  @brief 四足机器人控制核心使用的、与 ROS 2 和仿真器无关的数据类型。
 *
 *  数据组织参考 MIT Cheetah 3 的控制分层，但字段含义和关节数量针对
 *  Unitree GO1（四条腿、每腿三个关节）定义。ROS 消息、MuJoCo 数据和
 *  将来的真机数据都应先转换成这里的类型，控制算法不要直接依赖中间件。
 */

#ifndef MYMIT_ROBOT_MODEL_ROBOT_TYPES_HPP_
#define MYMIT_ROBOT_MODEL_ROBOT_TYPES_HPP_

#include <cstddef>
#include <cstdint>

#include "cppTypes.h"


/// GO1 固定为四条腿，每条腿三个电机关节，共十二个关节。
constexpr std::size_t kNumLegs = 4;
constexpr std::size_t kJointsPerLeg = 3;
constexpr std::size_t kNumJoints = kNumLegs * kJointsPerLeg;

/**
 * @brief 控制框架已经登记的机器人型号。
 *
 * 枚举只负责标识机型，具体参数由 model/robots/ 下的独立文件提供。
 */
enum class RobotType : std::uint8_t
{
  UNITREE_GO1 = 0,
  DM_BOT1 = 1
};

/**
 * @brief 腿编号。
 *
 * 此顺序是控制器内部数组、FootContact.msg 和控制器 YAML 的公共约定。
 * FR: 右前，FL: 左前，RR: 右后，RL: 左后。
 */
enum class LegId : std::uint8_t
{
  FR = 0,
  FL = 1,
  RR = 2,
  RL = 3
};

/**
 * @brief 单腿内部关节编号。
 *
 * Hip 是髋关节外展轴，thigh 是大腿俯仰轴，calf 是膝关节俯仰轴。
 * 当前 GO1 MJCF 中分别对应 hip_joint、thigh_joint、calf_joint。
 */
enum class LegJointId : std::uint8_t
{
  Hip = 0,
  thigh = 1,
  calf = 2
};

/// 将单腿关节编号转换为三维关节向量下标。
constexpr std::size_t jointIndex(LegJointId joint) noexcept
{
  return static_cast<std::size_t>(joint);
}

/// 将腿编号和单腿关节编号转换为十二维关节向量下标。
constexpr std::size_t jointIndex(LegId leg, LegJointId joint) noexcept
{
  return static_cast<std::size_t>(leg) * kJointsPerLeg +
         static_cast<std::size_t>(joint);
}

/// 控制状态编号与 SetControlMode.srv 保持一致。
enum class ControlMode : std::uint8_t
{
  Passive = 0,
  JointPd = 1,
  BalanceStand = 2,
  Locomotion = 3
};

/**
 * @brief IMU 的一次采样。
 *
 * orientation_world_from_body 是 IMU 内部或仿真器已解算的姿态，
 * 表示从机身坐标系旋转到世界坐标系，Eigen 构造顺序为 (w, x, y, z)。
 * 角速度和线加速度均在 IMU/机身坐标系中；
 * acceleration_body 是否包含重力由接入层统一处理，控制算法内不得混用。
 */
template<typename T>
struct ImuData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaternion<T> orientation_world_from_body{T(1), T(0), T(0), T(0)};
  Vec3<T> angular_velocity_body = Vec3<T>::Zero();  ///< rad/s
  Vec3<T> acceleration_body = Vec3<T>::Zero();      ///< m/s^2
  T timestamp = 0.0;  ///< 单调时钟时间，单位 s。
  bool valid = false;      ///< false 时状态估计器必须丢弃本次采样。
};

/**
 * @brief 一条腿的三个关节反馈状态。
 *
 * 三维向量下标固定为 Hip、thigh、calf。当前 GO1 MJCF 中分别对应
 * hip_joint、thigh_joint、calf_joint。position 单位 rad，velocity 单位
 * rad/s，torque_estimate 单位 N*m。LegController 在外部持有四个该结构体，
 * 本结构体本身不保存其他腿的数据。
 */
template<typename T>
struct JointState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LegId leg = LegId::FR;
  Vec3<T> position = Vec3<T>::Zero();
  Vec3<T> velocity = Vec3<T>::Zero();
  Vec3<T> torque_estimate = Vec3<T>::Zero();
  T timestamp = 0.0;  ///< 单调时钟时间，单位 s。
  bool valid = false;
};

/**
 * @brief 一条腿的三个关节期望命令。
 *
 * 期望力矩按 MIT LegController 常用形式计算：
 * tau = torque_feedforward + kp*(q_des-q) + kd*(qd_des-qd)。
 * 三维向量下标固定为 Hip、thigh、calf。
 * 实际写入 MuJoCo 或真机前仍必须经过关节限位、力矩限幅和超时保护。
 */
template<typename T>
struct JointCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LegId leg = LegId::FR;
  Vec3<T> position_desired = Vec3<T>::Zero();       ///< rad
  Vec3<T> velocity_desired = Vec3<T>::Zero();       ///< rad/s
  Vec3<T> kp = Vec3<T>::Zero();                     ///< N*m/rad
  Vec3<T> kd = Vec3<T>::Zero();                     ///< N*m/(rad/s)
  Vec3<T> torque_feedforward = Vec3<T>::Zero();     ///< N*m
  T timestamp = 0.0;  ///< 命令生成时间，用于检测控制命令超时。
  std::uint64_t sequence = 0;  ///< 单调递增的命令序号，便于发现丢帧。
  bool enabled = false;        ///< false 时硬件层必须进入安全模式。
};

/**
 * @brief 一条腿足端的接触估计结果。
 *
 * normal_force 是该足端受到的估计法向力，方向定义为地面对足端的支撑方向。
 * 转换成 FootContact.msg 时，由 ROS 适配层按照 leg 放入对应数组位置。
 */
template<typename T>
struct FootContactState
{
  LegId leg = LegId::FR;
  bool contact = false;
  T contact_probability = 0.0;  ///< [0, 1]
  T normal_force = 0.0;         ///< N
  T timestamp = 0.0;
  bool valid = false;
};

/**
 * @brief 浮动基座状态估计器的统一输出。
 *
 * 世界坐标系由仿真场景或里程计初始化过程定义，机身坐标系原点位于 base_link。
 * rotation_world_from_body 是四元数对应的缓存旋转矩阵；状态估计器更新姿态时
 * 必须同步更新它，避免控制循环中重复转换。
 */
template<typename T>
struct StateEstimate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaternion<T> orientation_world_from_body{T(1), T(0), T(0), T(0)};
  Mat3<T> rotation_world_from_body = Mat3<T>::Identity();
  Vec3<T> rpy = Vec3<T>::Zero();                 ///< roll/pitch/yaw, rad
  Vec3<T> position_world = Vec3<T>::Zero();      ///< m
  Vec3<T> velocity_world = Vec3<T>::Zero();      ///< m/s
  Vec3<T> velocity_body = Vec3<T>::Zero();       ///< m/s
  Vec3<T> angular_velocity_body = Vec3<T>::Zero();  ///< rad/s
  Vec3<T> acceleration_body = Vec3<T>::Zero();   ///< m/s^2
  Vec3<T> acceleration_world = Vec3<T>::Zero();  ///< m/s^2
  T timestamp = 0.0;
  bool valid = false;
};

/**
 * @brief 行为层交给 WBC 或 MPC 的期望机身状态。
 *
 * 机身位置、速度和加速度在世界坐标系中；角速度在机身坐标系中。
 * 本结构体只描述浮动基座，不包含任何腿或关节数组。
 */
template<typename T>
struct DesiredState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControlMode mode = ControlMode::Passive;
  Vec3<T> body_position_world = Vec3<T>::Zero();
  Vec3<T> body_velocity_world = Vec3<T>::Zero();
  Vec3<T> body_acceleration_world = Vec3<T>::Zero();
  Vec3<T> body_rpy = Vec3<T>::Zero();             ///< rad
  Vec3<T> body_angular_velocity = Vec3<T>::Zero();  ///< rad/s, body frame
  T timestamp = 0.0;
  bool valid = false;
};

/**
 * @brief 一条腿的足端笛卡尔期望状态。
 *
 * position_body 和 velocity_body 位于机身坐标系，force_feedforward_world
 * 位于世界坐标系。kp_cartesian、kd_cartesian 是三轴笛卡尔阻抗增益。
 * LegController 在外部为 FR、FL、RR、RL 各保存一个该结构体。
 */
template<typename T>
struct LegDesiredState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LegId leg = LegId::FR;
  Vec3<T> position_body = Vec3<T>::Zero();
  Vec3<T> velocity_body = Vec3<T>::Zero();
  Vec3<T> force_feedforward_world = Vec3<T>::Zero();
  Mat3<T> kp_cartesian = Mat3<T>::Zero();
  Mat3<T> kd_cartesian = Mat3<T>::Zero();
  T contact_schedule = 0.0;  ///< 0 为摆动，1 为支撑。
  T timestamp = 0.0;
  bool valid = false;
};

static_assert(kNumJoints == 12, "GO1 controller expects exactly 12 joints");
static_assert(jointIndex(LegJointId::calf) == 2, "invalid leg joint index");
static_assert(jointIndex(LegId::FR, LegJointId::Hip) == 0, "invalid FR index");
static_assert(jointIndex(LegId::RL, LegJointId::calf) == 11, "invalid RL index");

#endif  // MYMIT_ROBOT_MODEL_ROBOT_TYPES_HPP_
