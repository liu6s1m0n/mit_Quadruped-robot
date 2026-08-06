// ============================================================
// 足端接触估计器实现
// 当前工程根据关节力矩反推足端力，再与期望支撑力比较得到接触概率。
// MIT Cheetah 原版 ContactEstimator 主要传递步态接触相位；这里的力反推是为
// 当前尚未接入步态调度器的数据链增加的工程扩展。
// 数据全部来自当前工程的 ImuSensor、LegSensor 和 Quadruped。
// ============================================================

#include "controller/ContactEstimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <eigen3/Eigen/Cholesky>

namespace
{

  // 将传感器层固定使用的 float 关节反馈转换为估计器模板类型 T。
  template <typename T>
  JointState<T> castJointState(const JointState<float> &source)
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

} // namespace

// ---------- 参数校验 ----------

template <typename T>
bool ContactEstimatorParameters<T>::isValid() const
{
  return std::isfinite(static_cast<double>(maximum_sensor_skew)) &&
         maximum_sensor_skew >= T(0) &&
         std::isfinite(static_cast<double>(gravity)) && gravity > T(0) &&
         std::isfinite(static_cast<double>(force_ratio_gain)) &&
         force_ratio_gain > T(0) &&
         std::isfinite(static_cast<double>(force_ratio_threshold)) &&
         force_ratio_threshold >= T(0) &&
         std::isfinite(static_cast<double>(contact_probability_threshold)) &&
         contact_probability_threshold >= T(0) &&
         contact_probability_threshold <= T(1) &&
         std::isfinite(static_cast<double>(minimum_support_ratio)) &&
         minimum_support_ratio > T(0) && minimum_support_ratio <= T(1) &&
         std::isfinite(static_cast<double>(force_estimation_damping)) &&
         force_estimation_damping > T(0);
}

// ---------- 生命周期和状态复位 ----------

template <typename T>
ContactEstimator<T>::ContactEstimator(
    const Quadruped<T> &quadruped, ImuSensor &imu, const LegSensors &legs,
    const ContactEstimatorParameters<T> &parameters)
    : quadruped_(&quadruped), imu_(&imu), legs_(legs), parameters_(parameters)
{
  // 四条腿缺少任意一个数据源，都无法组成完整且同步的机器人反馈帧。
  for (LegSensor *leg : legs_)
  {
    if (leg == nullptr)
    {
      throw std::invalid_argument("contact estimator leg sensor must not be null");
    }
  }
  if (!parameters_.isValid())
  {
    throw std::invalid_argument("contact estimator parameters are invalid");
  }
  reset();
}

template <typename T>
void ContactEstimator<T>::reset()
{
  invalidateContacts(T(0));
  joint_states_ = std::array<JointState<T>, kNumLegs>{};
  legs_valid_ = false;
  timestamp_ = T(0);
}

template <typename T>
void ContactEstimator<T>::invalidateContacts(T timestamp)
{
  contacts_ = FootContacts{};
  probabilities_.fill(T(0));
  for (std::size_t index = 0; index < kNumLegs; ++index)
  {
    contacts_[index].leg = static_cast<LegId>(index);
    contacts_[index].timestamp = timestamp;
  }
}

// ---------- 四腿反馈读取与同步检查 ----------

template <typename T>
bool ContactEstimator<T>::readLegs(T imu_timestamp)
{
  legs_valid_ = true;

  for (std::size_t index = 0; index < kNumLegs; ++index)
  {
    joint_states_[index] = castJointState<T>(legs_[index]->read());
    const auto &state = joint_states_[index];
    const auto expected_leg = static_cast<LegId>(index);

    // 除 valid 外还检查数组位置和 LegId 一致，防止把某条腿接到错误槽位。
    const bool state_valid =
        state.valid && state.leg == expected_leg &&
        state.position.allFinite() && state.velocity.allFinite() &&
        state.torque_estimate.allFinite() &&
        std::isfinite(static_cast<double>(state.timestamp));

    // IMU 与关节数据相差过大时不能视为同一状态帧。
    const T time_difference = std::abs(state.timestamp - imu_timestamp);
    if (!state_valid || time_difference > parameters_.maximum_sensor_skew)
    {
      legs_valid_ = false;
    }
  }
  return legs_valid_;
}

// ---------- 接触估计主流程 ----------

template <typename T>
bool ContactEstimator<T>::run()
{
  // 每周期直接从统一接口读取；SimImu/HardwareImu 的差别不会进入估计算法。
  const ImuData<float> raw_imu = imu_->read();

  const bool imu_valid =
      raw_imu.valid &&
      raw_imu.orientation_world_from_body.coeffs().allFinite() &&
      raw_imu.acceleration_body.allFinite() &&
      std::isfinite(raw_imu.timestamp);

  if (!imu_valid)
  {
    reset();
    return false;
  }

  const T timestamp = static_cast<T>(raw_imu.timestamp);
  timestamp_ = timestamp;
  if (!readLegs(timestamp))
  {
    // 传感器帧不可靠时清除旧结果，防止控制器误用上一帧的接触状态。
    invalidateContacts(timestamp);
    return false;
  }

  // IMU 直接给出机身姿态（仿真中即真值），并据此把机身系比力旋转到世界系。
  Eigen::Quaternion<T> orientation =
      raw_imu.orientation_world_from_body.template cast<T>();
  const T orientation_norm = orientation.norm();
  if (!std::isfinite(static_cast<double>(orientation_norm)) ||
      orientation_norm <= std::numeric_limits<T>::epsilon())
  {
    legs_valid_ = false;
    invalidateContacts(timestamp);
    return false;
  }
  orientation.normalize();
  const Mat3<T> rotation_world_from_body = orientation.toRotationMatrix();
  const Vec3<T> acceleration_world =
      rotation_world_from_body * raw_imu.acceleration_body.template cast<T>();

  bool all_valid = true;
  for (std::size_t index = 0; index < kNumLegs; ++index)
  {
    all_valid = estimateLegContact(
                    index, rotation_world_from_body, acceleration_world) &&
                all_valid;
  }

  return all_valid;
}

// ---------- 单腿接触估计 ----------

template <typename T>
bool ContactEstimator<T>::estimateLegContact(
    std::size_t index, const Mat3<T> &rotation_world_from_body,
    const Vec3<T> &acceleration_world)
{
  const LegId leg_id = static_cast<LegId>(index);
  const auto &state = joint_states_[index];
  FootContactState<T> &contact = contacts_[index];
  contact.leg = leg_id;

  // 正运动学：仅需要足端雅可比 J（v = J * qd），不需要足端位置。
  Mat3<T> jacobian;
  computeLegJacobianAndPosition<T>(
      *quadruped_, state.position, &jacobian, nullptr, leg_id);

  if (!jacobian.allFinite())
  {
    contact = FootContactState<T>{};
    contact.leg = leg_id;
    contact.timestamp = timestamp_;
    probabilities_[index] = T(0);
    return false;
  }

  // 由关节力矩反推足端力：tau = J^T * F。直接计算 J^{-T} 会在腿接近
  // 伸直的奇异位形放大测量噪声，因此求解带阻尼的最小二乘问题：
  // (J*J^T + lambda^2*I) * F = J*tau。
  const T damping_squared =
      parameters_.force_estimation_damping * parameters_.force_estimation_damping;
  const Mat3<T> normal_matrix =
      jacobian * jacobian.transpose() + damping_squared * Mat3<T>::Identity();
  Eigen::LDLT<Mat3<T>> solver(normal_matrix);
  if (solver.info() != Eigen::Success)
  {
    contact = FootContactState<T>{};
    contact.leg = leg_id;
    contact.timestamp = timestamp_;
    probabilities_[index] = T(0);
    contact.valid = false;
    return false;
  }

  const Vec3<T> foot_force_body =
      solver.solve(jacobian * state.torque_estimate);
  const Vec3<T> foot_force_world = rotation_world_from_body * foot_force_body;

  if (solver.info() != Eigen::Success || !foot_force_world.allFinite())
  {
    contact = FootContactState<T>{};
    contact.leg = leg_id;
    contact.timestamp = timestamp_;
    probabilities_[index] = T(0);
    return false;
  }

  // 实测竖直支撑力：取足端力世界竖直分量的绝对值，对力的符号约定不敏感。
  const T measured_vertical = std::abs(foot_force_world.z());

  // 期望竖直支撑力：准静态下每条腿分担 1/4 体重；IMU 比力同时反映身体在
  // 动态过程中的加减载（静止时为 g，腾空时趋向 0）。
  const T mass = quadruped_->totalMass();
  const T nominal_support = (mass / T(4)) * parameters_.gravity;
  const T support_expected = (mass / T(4)) * acceleration_world.z();
  // 期望值过小（如腾空）时按名义支撑力的一部分作下限，防止比例爆炸误判。
  const T support_reference =
      std::max(support_expected, parameters_.minimum_support_ratio * nominal_support);

  // 接触概率：实测/期望支撑力比例经 sigmoid 平滑，比例越接近 1 概率越高。
  const T support_ratio =
      measured_vertical / (support_reference + std::numeric_limits<T>::epsilon());
  contact.contact_probability = sigmoid(
      parameters_.force_ratio_gain *
      (support_ratio - parameters_.force_ratio_threshold));
  contact.contact =
      contact.contact_probability >= parameters_.contact_probability_threshold;
  contact.normal_force = measured_vertical;
  contact.timestamp = timestamp_;

  contact.valid =
      std::isfinite(static_cast<double>(contact.normal_force)) &&
      contact.contact_probability >= T(0) &&
      contact.contact_probability <= T(1);

  // 同步到概率数组，供 PositionVelocityEstimator 等下游使用。
  probabilities_[index] = contact.contact_probability;
  return contact.valid;
}

// ---------- 数值工具 ----------

template <typename T>
T ContactEstimator<T>::sigmoid(T x)
{
  // 分段计算避免 exp 溢出：x >= 0 时用 1/(1+e^{-x})，x < 0 时用等价形式。
  if (x >= T(0))
  {
    const T e = std::exp(-x);
    return T(1) / (T(1) + e);
  }
  const T e = std::exp(x);
  return e / (T(1) + e);
}

// 模板实现放在 .cpp 中，显式生成当前工程使用的两种精度。
template struct ContactEstimatorParameters<float>;
template struct ContactEstimatorParameters<double>;
template class ContactEstimator<float>;
template class ContactEstimator<double>;
