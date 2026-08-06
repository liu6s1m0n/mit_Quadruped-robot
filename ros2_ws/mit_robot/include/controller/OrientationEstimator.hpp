/*! @file OrientationEstimator.hpp
 *  @brief 使用统一 IMU 与四腿传感器接口的机身姿态估计器。
 *
 *  计算模式保留角速度预测和加速度重力方向修正，并额外融合
 *  IMU 内部已解算的绝对姿态角，以抑制积分漂移。
 *  本文件不依赖 VectorNavData、CheaterState 或 StateEstimatorContainer，而是
 *  直接使用当前工程的 ImuSensor、LegSensor 和 StateEstimate。
 */

#ifndef MYMIT_ROBOT_CONTROLLER_ORIENTATION_ESTIMATOR_HPP_
#define MYMIT_ROBOT_CONTROLLER_ORIENTATION_ESTIMATOR_HPP_

#include <array>
#include <cstdint>

#include "model/robot_types.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

/** @brief 姿态估计器使用的算法模式。 */
enum class OrientationEstimatorMode : std::uint8_t
{
  /** 直接采用仿真 IMU 给出的姿态四元数，供调试和算法对照使用。 */
  SIMULATION_TRUTH = 0,
  /** 融合 IMU 姿态角、角速度和加速度的真实姿态估计。 */
  IMU_FUSION = 1
};

/**
 * @brief 从 IMU 和四腿同步反馈生成机身姿态估计。
 *
 * ImuSensor 是实际姿态信息来源。LegSensor 不参与四元数计算，因为关节编码器
 * 本身无法唯一确定浮动机身姿态；四腿数据用于确认控制周期的整帧反馈有效、
 * 检查传感器时间同步，并缓存给后续足端接触/机身位置估计器使用。
 *
 * @tparam T 估计器内部数值类型；当前库显式支持 float 和 double。
 */
template<typename T>
class OrientationEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief 四条腿的传感器指针，数组顺序固定为 FR、FL、RR、RL。 */
  using LegSensors = std::array<LegSensor *, kNumLegs>;

  /** @brief 四条腿最近一次读取的统一反馈。 */
  using JointStates = std::array<JointState<T>, kNumLegs>;

  /**
   * @brief 创建姿态估计器并绑定 IMU 与四腿数据源。
   *
   * 估计器不拥有这些传感器对象，它们的生命周期必须覆盖估计器。
   *
   * @param imu IMU 数据源，可以是 SimImu 或 HardwareImu。
   * @param legs 四条腿的数据源，可以是 SimLeg 或 HardwareLeg。
   * @param mode 仿真真值模式或 IMU 多源融合模式。
   * @param maximum_sensor_skew IMU 与关节采样允许的最大时间差，单位 s。
   * @param accelerometer_correction_gain 加速度重力方向修正增益，单位 1/s。
   * @param imu_orientation_correction_gain IMU 直接姿态角修正增益，单位 1/s。
   * @throws std::invalid_argument 腿指针为空、时间差或修正增益参数非法。
   */
  OrientationEstimator(
    ImuSensor & imu, const LegSensors & legs,
    OrientationEstimatorMode mode = OrientationEstimatorMode::IMU_FUSION,
    T maximum_sensor_skew = T(0.02),
    T accelerometer_correction_gain = T(2),
    T imu_orientation_correction_gain = T(10));

  /**
   * @brief 读取 IMU 和四腿数据并完成一次姿态估计。
   *
   * 成功后可以通过 result() 获取四元数、旋转矩阵、RPY、机身角速度以及机身/
   * 世界坐标系加速度。任一传感器无效、腿编号错误或时间不同步都会返回 false，
   * 同时把输出标记为无效，避免控制器继续使用旧姿态。
   *
   * @return 本周期所有输入和姿态计算均有效时返回 true。
   */
  bool run();

  /**
   * @brief 清空结果和姿态融合状态。
   */
  void reset();

  /**
   * @brief 切换姿态估计模式并清空旧融合状态。
   * @param mode 新的算法模式。
   */
  void setMode(OrientationEstimatorMode mode);

  /** @brief 返回当前使用的姿态估计算法模式。 */
  OrientationEstimatorMode mode() const noexcept {return mode_;}

  /** @brief 返回最近一次姿态估计结果。 */
  const StateEstimate<T> & result() const noexcept {return result_;}

  /** @brief 返回本周期读取的四条腿反馈，顺序为 FR、FL、RR、RL。 */
  const JointStates & jointStates() const noexcept {return joint_states_;}

  /** @brief 返回最近一次四腿反馈是否全部有效且与 IMU 时间同步。 */
  bool legsValid() const noexcept {return legs_valid_;}

private:
  /** @brief 读取四腿反馈、转换精度并进行腿编号与时间同步检查。 */
  bool readLegs(T imu_timestamp);

  /** @brief 把旋转矩阵转换成 roll、pitch、yaw，单位 rad。 */
  static Vec3<T> rotationMatrixToRpy(const Mat3<T> & rotation);

  /** @brief 校验并转换 IMU 或仿真器给出的姿态四元数。 */
  static bool readImuOrientation(
    const ImuData<float> & imu, Eigen::Quaternion<T> & orientation);

  /** @brief 融合 IMU 直接姿态、角速度与加速度重力方向。 */
  bool computeImuFusionOrientation(
    const ImuData<float> & imu, T timestamp,
    Eigen::Quaternion<T> & orientation);

  /** @brief 根据加速度方向和指定 yaw 构造姿态四元数。 */
  static bool orientationFromAccelerometer(
    const Vec3<T> & acceleration_body, T yaw,
    Eigen::Quaternion<T> & orientation);

  ImuSensor * imu_ = nullptr;      ///< 非拥有指针，指向统一 IMU 数据源。
  LegSensors legs_{};              ///< 非拥有指针，按 FR、FL、RR、RL 保存。
  JointStates joint_states_{};     ///< 最近一次转换后的四腿反馈。
  StateEstimate<T> result_{};      ///< 最近一次姿态估计输出。
  Eigen::Quaternion<T> integrated_orientation_{T(1), T(0), T(0), T(0)};
  OrientationEstimatorMode mode_ = OrientationEstimatorMode::IMU_FUSION;
  bool fusion_initialized_ = false;
  bool legs_valid_ = false;
  T maximum_sensor_skew_ = T(0.02);
  T accelerometer_correction_gain_ = T(2);
  T imu_orientation_correction_gain_ = T(10);
  T last_imu_timestamp_ = T(0);
};

#endif  // MYMIT_ROBOT_CONTROLLER_ORIENTATION_ESTIMATOR_HPP_
