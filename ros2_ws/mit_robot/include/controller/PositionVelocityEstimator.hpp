/*! @file PositionVelocityEstimator.hpp
 *  @brief 基于 IMU、四腿运动学和接触约束的机身位置/速度估计器。
 *
 *  滤波状态与 MIT Cheetah 3 的 LinearKFPositionVelocityEstimator 一致：
 *  x = [机身世界位置(3), 机身世界速度(3), 四个足端世界位置(12)]。
 *  数据入口已替换成当前工程的 ImuSensor、LegSensor、Quadruped 和 StateEstimate。
 */

#ifndef MYMIT_ROBOT_CONTROLLER_POSITION_VELOCITY_ESTIMATOR_HPP_
#define MYMIT_ROBOT_CONTROLLER_POSITION_VELOCITY_ESTIMATOR_HPP_

#include <array>

#include "controller/OrientationEstimator.hpp"
#include "model/quadruped.hpp"

/** @brief 位置速度卡尔曼滤波器的噪声和时间参数。 */
template<typename T>
struct PositionVelocityEstimatorParameters
{
  T nominal_time_step = T(0.002);              ///< 首次配置使用的控制周期，s。
  T maximum_time_step = T(0.1);                ///< 允许连续积分的最大周期，s。
  T process_noise_position = T(0.02);          ///< IMU 位置过程噪声缩放。
  T process_noise_velocity = T(0.02);          ///< IMU 速度过程噪声缩放。
  T process_noise_foot_position = T(0.002);    ///< 足端世界位置过程噪声缩放。
  T sensor_noise_relative_position = T(0.001); ///< 机身到足端相对位置观测噪声。
  T sensor_noise_relative_velocity = T(0.1);   ///< 支撑足速度约束观测噪声。
  T sensor_noise_foot_height = T(0.001);       ///< 支撑足地面高度观测噪声。
  T initial_covariance = T(100);               ///< 初始状态协方差。
  T gravity = T(9.81);                         ///< 重力加速度绝对值，m/s^2。
  T suspect_noise_multiplier = T(100);         ///< 摆动腿噪声最大放大倍数。

  /** @brief 检查所有参数是否为有限正数。 */
  bool isValid() const;
};

/**
 * @brief MIT Cheetah 风格的 18 状态线性卡尔曼位置速度估计器。
 *
 * 内部 OrientationEstimator 每周期从一个 ImuSensor 和四个 LegSensor 读取完整
 * 传感器帧。IMU 比力经过姿态旋转并补偿重力后用于状态预测；腿关节位置/速度
 * 通过正运动学形成相对足端位置和支撑足零速度观测。
 *
 * @tparam T 滤波数值类型；当前库显式支持 float 和 double。
 */
template<typename T>
class PositionVelocityEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using LegSensors = typename OrientationEstimator<T>::LegSensors;
  using ContactProbabilities = std::array<T, kNumLegs>;

  /**
   * @brief 创建估计器并绑定机器人模型、IMU 和四腿数据源。
   *
   * 估计器不拥有这些对象，它们的生命周期必须覆盖估计器。
   *
   * @param quadruped 提供 Hip 位置和三段腿部几何参数的机器人模型。
   * @param imu 仿真或真实硬件 IMU 数据源。
   * @param legs 按 FR、FL、RR、RL 排列的四腿数据源。
   * @param orientation_mode 仿真真值或 IMU 多源姿态融合模式。
   * @param parameters 卡尔曼滤波噪声和时间参数。
   * @param maximum_sensor_skew IMU 与腿反馈允许的最大时间差，s。
   */
  PositionVelocityEstimator(
    const Quadruped<T> & quadruped, ImuSensor & imu,
    const LegSensors & legs,
    OrientationEstimatorMode orientation_mode =
    OrientationEstimatorMode::IMU_FUSION,
    const PositionVelocityEstimatorParameters<T> & parameters = {},
    T maximum_sensor_skew = T(0.02));

  /**
   * @brief 读取传感器并完成一次预测和观测更新。
   * @return 姿态、腿数据和滤波计算均有效时返回 true。
   */
  bool run();

  /** @brief 清空卡尔曼状态、协方差和姿态融合状态。 */
  void reset();

  /**
   * @brief 设置四腿当前接触概率。
   * @param probabilities 顺序为 FR、FL、RR、RL，每项自动限制到 [0,1]。
   */
  void setContactProbabilities(const ContactProbabilities & probabilities);

  /** @brief 设置单腿接触概率，每项自动限制到 [0,1]。 */
  void setContactProbability(LegId leg_id, T probability);

  /** @brief 切换内部姿态算法；切换时同时重置整个位置速度滤波器。 */
  void setOrientationMode(OrientationEstimatorMode mode);

  /** @brief 返回最近一次完整状态估计结果。 */
  const StateEstimate<T> & result() const noexcept {return result_;}

  /** @brief 返回内部姿态估计器，便于读取四腿缓存和当前模式。 */
  const OrientationEstimator<T> & orientationEstimator() const noexcept
  {
    return orientation_estimator_;
  }

private:
  using StateVector = Eigen::Matrix<T, 18, 1>;
  using ObservationVector = Eigen::Matrix<T, 28, 1>;
  using StateMatrix = Eigen::Matrix<T, 18, 18>;
  using InputMatrix = Eigen::Matrix<T, 18, 3>;
  using ObservationMatrix = Eigen::Matrix<T, 28, 18>;
  using ObservationCovariance = Eigen::Matrix<T, 28, 28>;

  /** @brief 构造固定的 28x18 观测矩阵。 */
  void configureObservationMatrix();

  /** @brief 根据四腿关节反馈计算足端相对机身的位置和速度。 */
  bool updateFootKinematics(const StateEstimate<T> & orientation);

  /** @brief 使用第一帧足端几何关系初始化机身高度和足端世界位置。 */
  void initializeState(const StateEstimate<T> & orientation);

  /** @brief 将滤波状态的位置速度字段写入统一 StateEstimate。 */
  bool updateResult(const StateEstimate<T> & orientation);

  const Quadruped<T> * quadruped_ = nullptr;
  OrientationEstimator<T> orientation_estimator_;
  PositionVelocityEstimatorParameters<T> parameters_;
  ContactProbabilities contact_probabilities_{};

  StateVector state_ = StateVector::Zero();
  StateMatrix covariance_ = StateMatrix::Identity();
  ObservationMatrix observation_matrix_ = ObservationMatrix::Zero();
  StateEstimate<T> result_{};

  std::array<Vec3<T>, kNumLegs> foot_position_body_{};
  std::array<Vec3<T>, kNumLegs> foot_velocity_body_{};
  bool initialized_ = false;
  T last_timestamp_ = T(0);
};

#endif  // MYMIT_ROBOT_CONTROLLER_POSITION_VELOCITY_ESTIMATOR_HPP_
