// ============================================================
// 机身位置与速度估计器实现
// 18 状态线性卡尔曼滤波结构参考 MIT Cheetah 3，输入改为当前工程的统一
// ImuSensor、LegSensor、OrientationEstimator 和 Quadruped 接口。
// ============================================================

#include "controller/PositionVelocityEstimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <eigen3/Eigen/Cholesky>

#include "controller/leg_controller.hpp"

namespace
{

// 将 LegId 转换成四腿数组下标，并拒绝强制转换得到的非法枚举值。
std::size_t checkedLegIndex(LegId leg_id)
{
  const auto index = static_cast<std::size_t>(leg_id);
  if (index >= kNumLegs) {
    throw std::invalid_argument("invalid leg id");
  }
  return index;
}

}  // namespace

// ---------- 参数检查 ----------

template<typename T>
bool PositionVelocityEstimatorParameters<T>::isValid() const
{
  const std::array<T, 11> values{
    nominal_time_step, maximum_time_step,
    process_noise_position, process_noise_velocity,
    process_noise_foot_position, sensor_noise_relative_position,
    sensor_noise_relative_velocity, sensor_noise_foot_height,
    initial_covariance, gravity, suspect_noise_multiplier};
  for (T value : values) {
    if (!std::isfinite(static_cast<double>(value)) || value <= T(0)) {
      return false;
    }
  }
  return maximum_time_step >= nominal_time_step;
}

// ---------- 生命周期和固定矩阵配置 ----------

template<typename T>
PositionVelocityEstimator<T>::PositionVelocityEstimator(
  const Quadruped<T> & quadruped, ImuSensor & imu,
  const LegSensors & legs, OrientationEstimatorMode orientation_mode,
  const PositionVelocityEstimatorParameters<T> & parameters,
  T maximum_sensor_skew)
: quadruped_(&quadruped),
  orientation_estimator_(
    imu, legs, orientation_mode, maximum_sensor_skew),
  parameters_(parameters)
{
  if (!parameters_.isValid()) {
    throw std::invalid_argument("invalid position velocity estimator parameters");
  }
  contact_probabilities_.fill(T(1));
  configureObservationMatrix();
  reset();
}

template<typename T>
void PositionVelocityEstimator<T>::reset()
{
  state_.setZero();
  covariance_.setIdentity();
  covariance_ *= parameters_.initial_covariance;
  result_ = StateEstimate<T>{};
  for (auto & position : foot_position_body_) {
    position.setZero();
  }
  for (auto & velocity : foot_velocity_body_) {
    velocity.setZero();
  }
  initialized_ = false;
  last_timestamp_ = T(0);
  orientation_estimator_.reset();
}

template<typename T>
void PositionVelocityEstimator<T>::configureObservationMatrix()
{
  observation_matrix_.setZero();

  // 前 12 行观测“机身世界位置 - 足端世界位置”，即足端相对位置的负值。
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const Eigen::Index row = static_cast<Eigen::Index>(3 * leg);
    const Eigen::Index foot_state = static_cast<Eigen::Index>(6 + 3 * leg);
    observation_matrix_.template block<3, 3>(row, 0).setIdentity();
    observation_matrix_.template block<3, 3>(row, foot_state) =
      -Mat3<T>::Identity();

    // 中间 12 行直接观测机身世界速度；支撑足静止时 v_body=-v_foot_relative。
    observation_matrix_.template block<3, 3>(12 + row, 3).setIdentity();

    // 最后 4 行分别观测四个足端的世界 z 高度，支撑地面默认 z=0。
    observation_matrix_(24 + static_cast<Eigen::Index>(leg), foot_state + 2) = T(1);
  }
}

// ---------- 对外配置 ----------

template<typename T>
void PositionVelocityEstimator<T>::setContactProbabilities(
  const ContactProbabilities & probabilities)
{
  for (std::size_t index = 0; index < kNumLegs; ++index) {
    setContactProbability(static_cast<LegId>(index), probabilities[index]);
  }
}

template<typename T>
void PositionVelocityEstimator<T>::setContactProbability(
  LegId leg_id, T probability)
{
  if (!std::isfinite(static_cast<double>(probability))) {
    throw std::invalid_argument("contact probability must be finite");
  }
  contact_probabilities_[checkedLegIndex(leg_id)] =
    std::clamp(probability, T(0), T(1));
}

template<typename T>
void PositionVelocityEstimator<T>::setOrientationMode(
  OrientationEstimatorMode mode)
{
  orientation_estimator_.setMode(mode);
  reset();
}

// ---------- 足端运动学 ----------

template<typename T>
bool PositionVelocityEstimator<T>::updateFootKinematics(
  const StateEstimate<T> & orientation)
{
  const auto & joint_states = orientation_estimator_.jointStates();

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const LegId leg_id = static_cast<LegId>(index);
    const auto & joints = joint_states[index];
    Mat3<T> jacobian;
    Vec3<T> foot_from_hip;

    // 使用与 LegController 相同的关节顺序和几何公式，避免控制与估计模型不一致。
    computeLegJacobianAndPosition(
      *quadruped_, joints.position, &jacobian, &foot_from_hip, leg_id);

    // 足端相对机身原点的位置 = Hip 安装位置 + 足端相对 Hip 的位置。
    foot_position_body_[index] =
      quadruped_->hipLocation(leg_id) + foot_from_hip;

    // 足端相对机身的速度包含机身转动项 omega x p 和关节运动项 J*qd。
    foot_velocity_body_[index] =
      orientation.angular_velocity_body.cross(foot_position_body_[index]) +
      jacobian * joints.velocity;

    if (!foot_position_body_[index].allFinite() ||
      !foot_velocity_body_[index].allFinite())
    {
      return false;
    }
  }
  return true;
}

// ---------- 第一帧初始化 ----------

template<typename T>
void PositionVelocityEstimator<T>::initializeState(
  const StateEstimate<T> & orientation)
{
  state_.setZero();

  // 水平原点无法仅靠 IMU/关节绝对确定，因此定义启动时机身 x=y=0。
  // 机身高度由接触腿足端相对位置估计，使支撑足初始世界高度接近 0。
  T weighted_height = T(0);
  T total_contact = T(0);
  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const Vec3<T> foot_relative_world =
      orientation.rotation_world_from_body * foot_position_body_[index];
    const T weight = contact_probabilities_[index];
    weighted_height += weight * (-foot_relative_world.z());
    total_contact += weight;
  }
  if (total_contact <= std::numeric_limits<T>::epsilon()) {
    // 尚无接触信息时用模型标称高度作为安全初值。
    state_.z() = quadruped_->nominalBodyHeight();
  } else {
    state_.z() = weighted_height / total_contact;
  }

  // 根据已初始化的机身位置，将四个足端状态放到世界坐标系中。
  const Vec3<T> body_position = state_.template segment<3>(0);
  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const Eigen::Index foot_state = static_cast<Eigen::Index>(6 + 3 * index);
    state_.template segment<3>(foot_state) =
      body_position +
      orientation.rotation_world_from_body * foot_position_body_[index];
  }

  covariance_.setIdentity();
  covariance_ *= parameters_.initial_covariance;
  last_timestamp_ = orientation.timestamp;
  initialized_ = true;
}

// ---------- 卡尔曼滤波主流程 ----------

template<typename T>
bool PositionVelocityEstimator<T>::run()
{
  // 内部姿态估计器负责读取一次 IMU 和四腿反馈，并缓存本周期 JointState。
  if (!orientation_estimator_.run()) {
    result_ = StateEstimate<T>{};
    return false;
  }
  const StateEstimate<T> orientation = orientation_estimator_.result();

  if (!updateFootKinematics(orientation)) {
    result_ = orientation;
    result_.valid = false;
    return false;
  }

  if (!initialized_) {
    initializeState(orientation);
    return updateResult(orientation);
  }

  const T time_step = orientation.timestamp - last_timestamp_;
  if (!std::isfinite(static_cast<double>(time_step))) {
    result_ = orientation;
    result_.valid = false;
    return false;
  }
  if (time_step < T(0)) {
    // 仿真复位或硬件时钟回跳时丢弃旧状态，从当前几何关系重新初始化。
    initialized_ = false;
    initializeState(orientation);
    return updateResult(orientation);
  }
  if (time_step == T(0)) {
    // 同一时间戳重复读取时不进行二次积分，但仍返回当前有效估计。
    return updateResult(orientation);
  }
  if (time_step > parameters_.maximum_time_step) {
    result_ = orientation;
    result_.valid = false;
    return false;
  }

  // A 使用常速度离散模型；B 将补偿重力后的世界线加速度输入速度状态。
  StateMatrix transition = StateMatrix::Identity();
  transition.template block<3, 3>(0, 3) = time_step * Mat3<T>::Identity();
  InputMatrix input = InputMatrix::Zero();
  input.template block<3, 3>(3, 0) = time_step * Mat3<T>::Identity();

  // IMU 加速度计输出比力，转到世界系后还要加上世界重力才能得到线加速度。
  const Vec3<T> gravity_world(T(0), T(0), -parameters_.gravity);
  const Vec3<T> linear_acceleration_world =
    orientation.acceleration_world + gravity_world;

  StateMatrix process_noise = StateMatrix::Zero();
  process_noise.template block<3, 3>(0, 0) =
    parameters_.process_noise_position * (time_step / T(20)) *
    Mat3<T>::Identity();
  process_noise.template block<3, 3>(3, 3) =
    parameters_.process_noise_velocity *
    (time_step * parameters_.gravity / T(20)) * Mat3<T>::Identity();
  process_noise.template block<12, 12>(6, 6) =
    parameters_.process_noise_foot_position * time_step *
    Eigen::Matrix<T, 12, 12>::Identity();

  ObservationCovariance measurement_noise =
    ObservationCovariance::Zero();
  measurement_noise.template block<12, 12>(0, 0) =
    parameters_.sensor_noise_relative_position *
    Eigen::Matrix<T, 12, 12>::Identity();
  measurement_noise.template block<12, 12>(12, 12) =
    parameters_.sensor_noise_relative_velocity *
    Eigen::Matrix<T, 12, 12>::Identity();
  measurement_noise.template block<4, 4>(24, 24) =
    parameters_.sensor_noise_foot_height *
    Eigen::Matrix<T, 4, 4>::Identity();

  ObservationVector observation = ObservationVector::Zero();
  const Vec3<T> predicted_body_position = state_.template segment<3>(0);
  const Vec3<T> predicted_body_velocity = state_.template segment<3>(3);

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const Eigen::Index vector_index = static_cast<Eigen::Index>(3 * index);
    const Eigen::Index foot_state = 6 + vector_index;
    const T trust = contact_probabilities_[index];
    const T noise_scale =
      T(1) + (T(1) - trust) * parameters_.suspect_noise_multiplier;

    const Vec3<T> foot_relative_world =
      orientation.rotation_world_from_body * foot_position_body_[index];
    const Vec3<T> foot_relative_velocity_world =
      orientation.rotation_world_from_body * foot_velocity_body_[index];

    // 位置观测：p_body-p_foot=-p_foot_relative。
    observation.template segment<3>(vector_index) = -foot_relative_world;
    // 支撑时使用零足速约束 v_body=-v_foot_relative；摆动时退回预测速度。
    observation.template segment<3>(12 + vector_index) =
      (T(1) - trust) * predicted_body_velocity +
      trust * (-foot_relative_velocity_world);
    // 支撑足世界高度观测为 0；摆动腿则使用当前预测高度，不强拉向地面。
    observation(24 + static_cast<Eigen::Index>(index)) =
      (T(1) - trust) *
      (predicted_body_position.z() + foot_relative_world.z());

    process_noise.template block<3, 3>(foot_state, foot_state) *= noise_scale;
    measurement_noise.template block<3, 3>(
      12 + vector_index, 12 + vector_index) *= noise_scale;
    measurement_noise(
      24 + static_cast<Eigen::Index>(index),
      24 + static_cast<Eigen::Index>(index)) *= noise_scale;
  }

  // 预测步骤。
  state_ = transition * state_ + input * linear_acceleration_world;
  const StateMatrix predicted_covariance =
    transition * covariance_ * transition.transpose() + process_noise;

  // 观测更新。创新协方差理论上为对称正定矩阵，使用 LDLT 比显式求逆稳定。
  const ObservationVector innovation = observation - observation_matrix_ * state_;
  const ObservationCovariance innovation_covariance =
    observation_matrix_ * predicted_covariance * observation_matrix_.transpose() +
    measurement_noise;
  Eigen::LDLT<ObservationCovariance> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success) {
    result_ = orientation;
    result_.valid = false;
    return false;
  }

  const auto solved_innovation = decomposition.solve(innovation);
  state_ += predicted_covariance * observation_matrix_.transpose() * solved_innovation;
  const auto solved_observation = decomposition.solve(observation_matrix_);
  covariance_ =
    (StateMatrix::Identity() -
    predicted_covariance * observation_matrix_.transpose() * solved_observation) *
    predicted_covariance;
  covariance_ = (covariance_ + covariance_.transpose()) / T(2);

  // 沿用 MIT Cheetah 的水平协方差条件化，防止 x/y 漂移与其他状态强耦合。
  if (covariance_.template block<2, 2>(0, 0).determinant() > T(1e-6)) {
    covariance_.template block<2, 16>(0, 2).setZero();
    covariance_.template block<16, 2>(2, 0).setZero();
    covariance_.template block<2, 2>(0, 0) /= T(10);
  }

  last_timestamp_ = orientation.timestamp;
  return updateResult(orientation);
}

// ---------- 统一状态输出 ----------

template<typename T>
bool PositionVelocityEstimator<T>::updateResult(
  const StateEstimate<T> & orientation)
{
  result_ = orientation;
  result_.position_world = state_.template segment<3>(0);
  result_.velocity_world = state_.template segment<3>(3);
  result_.velocity_body =
    result_.rotation_world_from_body.transpose() * result_.velocity_world;
  result_.valid =
    orientation.valid && state_.allFinite() && covariance_.allFinite() &&
    result_.position_world.allFinite() && result_.velocity_world.allFinite() &&
    result_.velocity_body.allFinite();
  return result_.valid;
}

// 模板实现位于 .cpp，显式生成当前工程使用的两种精度。
template struct PositionVelocityEstimatorParameters<float>;
template struct PositionVelocityEstimatorParameters<double>;
template class PositionVelocityEstimator<float>;
template class PositionVelocityEstimator<double>;
