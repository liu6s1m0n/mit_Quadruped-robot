// ============================================================
// 机身姿态估计器实现
// 数据来自当前工程的 ImuSensor 和四个 LegSensor；算法保留 MIT Cheetah 3
// OrientationEstimator 的四元数处理、仿真真值和 IMU 多源融合。
// ============================================================

#include "controller/OrientationEstimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

// 将传感器层固定使用的 float 关节反馈转换为估计器模板类型 T。
template<typename T>
JointState<T> castJointState(const JointState<float> & source)
{
  JointState<T> result;
  result.leg = source.leg;
  result.position = source.position.template cast<T>();
  result.velocity = source.velocity.template cast<T>();
  result.torque_estimate = source.torque_estimate.template cast<T>();
  result.timestamp = static_cast<T>(source.timestamp);
  result.valid = source.valid;
  return result;
}

}  // namespace

// ---------- 生命周期和状态复位 ----------

template<typename T>
OrientationEstimator<T>::OrientationEstimator(
  ImuSensor & imu, const LegSensors & legs,
  OrientationEstimatorMode mode, T maximum_sensor_skew,
  T accelerometer_correction_gain,
  T imu_orientation_correction_gain)
: imu_(&imu),
  legs_(legs),
  mode_(mode),
  maximum_sensor_skew_(maximum_sensor_skew),
  accelerometer_correction_gain_(accelerometer_correction_gain),
  imu_orientation_correction_gain_(imu_orientation_correction_gain)
{
  // 四条腿缺少任意一个数据源，都无法组成完整且同步的机器人反馈帧。
  for (LegSensor * leg : legs_) {
    if (leg == nullptr) {
      throw std::invalid_argument("orientation estimator leg sensor must not be null");
    }
  }

  // 时间差必须可比较且不能为负；设为 0 表示要求时间戳完全一致。
  if (!std::isfinite(static_cast<double>(maximum_sensor_skew_)) ||
    maximum_sensor_skew_ < T(0))
  {
    throw std::invalid_argument("maximum sensor skew must be finite and non-negative");
  }
  if (!std::isfinite(static_cast<double>(accelerometer_correction_gain_)) ||
    accelerometer_correction_gain_ < T(0))
  {
    throw std::invalid_argument(
            "accelerometer correction gain must be finite and non-negative");
  }
  if (!std::isfinite(static_cast<double>(imu_orientation_correction_gain_)) ||
    imu_orientation_correction_gain_ < T(0))
  {
    throw std::invalid_argument(
            "IMU orientation correction gain must be finite and non-negative");
  }
  reset();
}

template<typename T>
void OrientationEstimator<T>::reset()
{
  result_ = StateEstimate<T>{};
  joint_states_ = JointStates{};
  integrated_orientation_ = Eigen::Quaternion<T>::Identity();
  fusion_initialized_ = false;
  legs_valid_ = false;
  last_imu_timestamp_ = T(0);
}

template<typename T>
void OrientationEstimator<T>::setMode(OrientationEstimatorMode mode)
{
  mode_ = mode;
  reset();
}

// ---------- 姿态估计主流程 ----------

template<typename T>
bool OrientationEstimator<T>::run()
{
  // 每周期直接从统一接口读取；SimImu/HardwareImu 的差别不会进入估计算法。
  const ImuData<float> raw_imu = imu_->read();

  // 9 轴 IMU 样本同时提供姿态角、角速度和加速度；融合模式会使用
  // 全部三类数据，而不是只积分角速度。
  const bool imu_valid =
    raw_imu.valid &&
    raw_imu.orientation_world_from_body.coeffs().allFinite() &&
    raw_imu.angular_velocity_body.allFinite() &&
    raw_imu.acceleration_body.allFinite() &&
    std::isfinite(raw_imu.timestamp);

  if (!imu_valid) {
    result_ = StateEstimate<T>{};
    legs_valid_ = false;
    return false;
  }

  // 按 FR、FL、RR、RL 读取四条腿，并要求它们与 IMU 属于同一个时间窗口。
  const T timestamp = static_cast<T>(raw_imu.timestamp);
  if (!readLegs(timestamp)) {
    result_ = StateEstimate<T>{};
    return false;
  }

  Eigen::Quaternion<T> orientation = Eigen::Quaternion<T>::Identity();
  bool orientation_valid = false;
  switch (mode_) {
    case OrientationEstimatorMode::SIMULATION_TRUTH:
      orientation_valid = readImuOrientation(raw_imu, orientation);
      break;
    case OrientationEstimatorMode::IMU_FUSION:
      orientation_valid = computeImuFusionOrientation(
        raw_imu, timestamp, orientation);
      break;
    default:
      orientation_valid = false;
      break;
  }
  if (!orientation_valid) {
    result_ = StateEstimate<T>{};
    return false;
  }

  StateEstimate<T> estimate;
  estimate.orientation_world_from_body = orientation;
  estimate.rotation_world_from_body = orientation.toRotationMatrix();
  estimate.rpy = rotationMatrixToRpy(estimate.rotation_world_from_body);

  // 陀螺仪和加速度计测量均表达在机身/IMU 坐标系中。
  estimate.angular_velocity_body =
    raw_imu.angular_velocity_body.template cast<T>();
  estimate.acceleration_body = raw_imu.acceleration_body.template cast<T>();

  // robot_types.hpp 明确规定 rotation_world_from_body 将机身向量旋转到世界系，
  // 因此这里直接左乘 R，而不是沿用旧 Cheetah rBody 语义下的 R.transpose()。
  estimate.acceleration_world =
    estimate.rotation_world_from_body * estimate.acceleration_body;
  estimate.timestamp = timestamp;

  // OrientationEstimator 暂不估计机身位置和线速度，这些字段保持默认零值，
  // 后续可由使用足端约束的 PositionVelocityEstimator 在同一 StateEstimate 中更新。
  estimate.valid =
    estimate.orientation_world_from_body.coeffs().allFinite() &&
    estimate.rotation_world_from_body.allFinite() && estimate.rpy.allFinite() &&
    estimate.angular_velocity_body.allFinite() &&
    estimate.acceleration_body.allFinite() &&
    estimate.acceleration_world.allFinite();

  if (!estimate.valid) {
    result_ = StateEstimate<T>{};
    return false;
  }

  result_ = estimate;
  return true;
}

// ---------- 仿真真值和 IMU 姿态融合 ----------

template<typename T>
bool OrientationEstimator<T>::readImuOrientation(
  const ImuData<float> & imu, Eigen::Quaternion<T> & orientation)
{
  // 真机姿态角和仿真 FRAMEQUAT 在 ImuData 中统一为机身到世界的四元数。
  const float norm = imu.orientation_world_from_body.norm();
  if (!imu.orientation_world_from_body.coeffs().allFinite() ||
    !std::isfinite(norm) || norm <= std::numeric_limits<float>::epsilon())
  {
    return false;
  }
  orientation = imu.orientation_world_from_body.template cast<T>();
  orientation.normalize();
  return orientation.coeffs().allFinite();
}

template<typename T>
bool OrientationEstimator<T>::computeImuFusionOrientation(
  const ImuData<float> & imu, T timestamp,
  Eigen::Quaternion<T> & orientation)
{
  Eigen::Quaternion<T> imu_orientation;
  if (!readImuOrientation(imu, imu_orientation)) {
    return false;
  }

  const Vec3<T> acceleration_body = imu.acceleration_body.template cast<T>();
  const Vec3<T> angular_velocity_body =
    imu.angular_velocity_body.template cast<T>();

  if (!fusion_initialized_) {
    // 首帧直接以 IMU 内部已解算的绝对姿态初始化，包括可观测的 yaw。
    integrated_orientation_ = imu_orientation;
    last_imu_timestamp_ = timestamp;
    fusion_initialized_ = true;
    orientation = integrated_orientation_;
    return true;
  }

  const T time_step = timestamp - last_imu_timestamp_;
  if (!std::isfinite(static_cast<double>(time_step)) || time_step < T(0)) {
    fusion_initialized_ = false;
    return computeImuFusionOrientation(imu, timestamp, orientation);
  }
  constexpr T kMaximumIntegrationStep = T(0.1);
  if (time_step > kMaximumIntegrationStep) {
    return false;
  }

  // 角速度只负责周期内的高频姿态预测。
  const T angular_speed = angular_velocity_body.norm();
  if (time_step > T(0) && angular_speed > std::numeric_limits<T>::epsilon()) {
    const Vec3<T> rotation_axis = angular_velocity_body / angular_speed;
    const Eigen::Quaternion<T> delta_rotation(
      Eigen::AngleAxis<T>(angular_speed * time_step, rotation_axis));
    integrated_orientation_ = integrated_orientation_ * delta_rotation;
    integrated_orientation_.normalize();
  }

  // 保留原有加速度重力方向修正，只在模长接近重力时使用。
  constexpr T kGravity = T(9.81);
  const T acceleration_norm = acceleration_body.norm();
  const bool acceleration_is_gravity =
    acceleration_norm >= T(0.5) * kGravity &&
    acceleration_norm <= T(1.5) * kGravity;
  if (acceleration_is_gravity && time_step > T(0)) {
    const T integrated_yaw =
      rotationMatrixToRpy(integrated_orientation_.toRotationMatrix()).z();
    Eigen::Quaternion<T> acceleration_orientation;
    if (orientationFromAccelerometer(
        acceleration_body, integrated_yaw, acceleration_orientation))
    {
      const T correction_weight = std::clamp(
        accelerometer_correction_gain_ * time_step, T(0), T(1));
      integrated_orientation_ = integrated_orientation_.slerp(
        correction_weight, acceleration_orientation);
      integrated_orientation_.normalize();
    }
  }

  // 在原融合结果上额外加入 IMU 直接姿态角这一层绝对角度约束，
  // 修正 roll/pitch/yaw 的积分漂移，而不删除高频角速度预测。
  if (time_step > T(0)) {
    const T correction_weight = std::clamp(
      imu_orientation_correction_gain_ * time_step, T(0), T(1));
    integrated_orientation_ = integrated_orientation_.slerp(
      correction_weight, imu_orientation);
    integrated_orientation_.normalize();
  }

  last_imu_timestamp_ = timestamp;
  orientation = integrated_orientation_;
  return orientation.coeffs().allFinite();
}

template<typename T>
bool OrientationEstimator<T>::orientationFromAccelerometer(
  const Vec3<T> & acceleration_body, T yaw,
  Eigen::Quaternion<T> & orientation)
{
  const T norm = acceleration_body.norm();
  if (!acceleration_body.allFinite() ||
    !std::isfinite(static_cast<double>(norm)) ||
    norm <= std::numeric_limits<T>::epsilon())
  {
    return false;
  }

  const Vec3<T> up_body = acceleration_body / norm;
  const T roll = std::atan2(up_body.y(), up_body.z());
  const T pitch = std::atan2(
    -up_body.x(),
    std::sqrt(up_body.y() * up_body.y() + up_body.z() * up_body.z()));

  orientation =
    Eigen::AngleAxis<T>(yaw, Vec3<T>::UnitZ()) *
    Eigen::AngleAxis<T>(pitch, Vec3<T>::UnitY()) *
    Eigen::AngleAxis<T>(roll, Vec3<T>::UnitX());
  orientation.normalize();
  return orientation.coeffs().allFinite();
}

// ---------- 四腿反馈读取与同步检查 ----------

template<typename T>
bool OrientationEstimator<T>::readLegs(T imu_timestamp)
{
  legs_valid_ = true;

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    // LegSensor 与 ImuSensor 一样屏蔽了 MuJoCo 和真实硬件的具体读取方式。
    joint_states_[index] = castJointState<T>(legs_[index]->read());
    const auto & state = joint_states_[index];
    const auto expected_leg = static_cast<LegId>(index);

    // 除 valid 外还检查数组位置和 LegId 一致，防止把某条腿接到错误槽位。
    const bool state_valid =
      state.valid && state.leg == expected_leg &&
      state.position.allFinite() && state.velocity.allFinite() &&
      state.torque_estimate.allFinite() &&
      std::isfinite(static_cast<double>(state.timestamp));

    // IMU 与关节数据相差过大时不能视为同一状态帧；仿真中通常完全相等，
    // 真机则允许构造函数设定的小时间窗口。
    const T time_difference = std::abs(state.timestamp - imu_timestamp);
    if (!state_valid || time_difference > maximum_sensor_skew_) {
      legs_valid_ = false;
    }
  }
  return legs_valid_;
}

// ---------- 旋转矩阵到 RPY ----------

template<typename T>
Vec3<T> OrientationEstimator<T>::rotationMatrixToRpy(
  const Mat3<T> & rotation)
{
  // ZYX 欧拉角约定：R = Rz(yaw) * Ry(pitch) * Rx(roll)。
  // asin 的输入先限制到 [-1, 1]，避免浮点舍入导致定义域错误。
  const T pitch_sine = std::clamp(-rotation(2, 0), T(-1), T(1));
  Vec3<T> rpy;
  rpy.x() = std::atan2(rotation(2, 1), rotation(2, 2));
  rpy.y() = std::asin(pitch_sine);
  rpy.z() = std::atan2(rotation(1, 0), rotation(0, 0));
  return rpy;
}

// 模板实现位于 .cpp，显式生成当前工程使用的两种精度。
template class OrientationEstimator<float>;
template class OrientationEstimator<double>;
