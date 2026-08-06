/*! @file ContactEstimator.hpp
 *  @brief 基于关节力矩反推足端力的接触估计器。
 *
 *  数据直接来自当前工程的 ImuSensor、LegSensor 和 Quadruped：
 *  - 期望支撑力：准静态下每条腿分担 1/4 机身重量，并用 IMU 比力反映动态加减载；
 *  - 实测足端力：根据 tau = J^T * F，用阻尼最小二乘求解足端力；
 *  - 接触概率：实测与期望竖直支撑力的比例经 sigmoid 平滑，并与阈值比较得二值接触。
 *
 *  MIT Cheetah 的 ContactEstimator 接口主要传递步态给出的接触相位；本类保持相同的
 *  四腿接触概率输出思路，但力矩反推属于当前工程在尚未接入步态调度器时的扩展。
 */

#ifndef MYMIT_ROBOT_CONTROLLER_CONTACT_ESTIMATOR_HPP_
#define MYMIT_ROBOT_CONTROLLER_CONTACT_ESTIMATOR_HPP_

#include <array>

#include "controller/leg_controller.hpp"
#include "model/quadruped.hpp"
#include "model/robot_types.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

/** @brief 接触估计器的可调参数。 */
template<typename T>
struct ContactEstimatorParameters
{
  T maximum_sensor_skew = T(0.02);          ///< IMU 与腿反馈允许的最大时间差，s。
  T gravity = T(9.81);                      ///< 重力加速度绝对值，m/s^2。
  T force_ratio_gain = T(8);                ///< 接触概率 sigmoid 的斜率，必须大于零。
  T force_ratio_threshold = T(0.3);         ///< 接触概率 sigmoid 的参考比例。
  T contact_probability_threshold = T(0.5); ///< 二值接触判定阈值。
  T minimum_support_ratio = T(0.2);         ///< 期望支撑力下限占名义支撑力的比例。
  T force_estimation_damping = T(1e-3);     ///< 足端力阻尼最小二乘系数，防止奇异放大。

  /** @brief 检查所有参数是否为有限且物理上合理的值。 */
  bool isValid() const;
};

/**
 * @brief 足端接触估计器。
 *
 * 每个控制周期从 ImuSensor 读取姿态、角速度和比力，从四个 LegSensor 读取
 * 关节位置与力矩，结合 Quadruped 的机身质量，输出四条腿的
 * FootContactState（接触概率、二值接触、法向力）。
 */
template<typename T>
class ContactEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief 四条腿数据源指针，数组顺序固定为 FR、FL、RR、RL。 */
  using LegSensors = std::array<LegSensor *, kNumLegs>;

  /** @brief 四腿接触概率数组，顺序为 FR、FL、RR、RL。 */
  using ContactProbabilities = std::array<T, kNumLegs>;

  /** @brief 四腿接触结果，顺序为 FR、FL、RR、RL。 */
  using FootContacts = std::array<FootContactState<T>, kNumLegs>;

  /**
   * @brief 创建接触估计器并绑定机器人模型、IMU 与四腿数据源。
   *
   * 估计器不拥有这些对象，它们的生命周期必须覆盖估计器。
   *
   * @param quadruped 提供机身质量与腿部几何参数的机器人模型。
   * @param imu 仿真或真实硬件 IMU 数据源。
   * @param legs 按 FR、FL、RR、RL 排列的四腿数据源。
   * @param parameters 接触估计参数。
   * @throws std::invalid_argument 腿指针为空或参数非法。
   */
  ContactEstimator(
    const Quadruped<T> & quadruped, ImuSensor & imu,
    const LegSensors & legs,
    const ContactEstimatorParameters<T> & parameters = {});

  /**
   * @brief 读取 IMU 和四腿数据并完成一次接触估计。
   * @return 全部腿接触计算有效时返回 true；任一输入失效返回 false。
   */
  bool run();

  /** @brief 清空接触结果并把概率置零。 */
  void reset();

  /** @brief 返回四腿接触结果，顺序为 FR、FL、RR、RL。 */
  const FootContacts & footContacts() const noexcept {return contacts_;}

  /** @brief 返回四腿接触概率，顺序为 FR、FL、RR、RL。 */
  const ContactProbabilities & contactProbabilities() const noexcept
  {
    return probabilities_;
  }

  /** @brief 返回本周期四条腿反馈是否全部有效且与 IMU 时间同步。 */
  bool legsValid() const noexcept {return legs_valid_;}

  /** @brief 返回最近一次处理的 IMU 时间戳；IMU 本身无效并复位时返回零。 */
  T timestamp() const noexcept {return timestamp_;}

private:
  /** @brief 读取四腿反馈、转换精度并进行腿编号与时间同步检查。 */
  bool readLegs(T imu_timestamp);

  /** @brief 计算单腿接触：期望/实测支撑力、接触概率、二值接触与法向力。 */
  bool estimateLegContact(
    std::size_t index, const Mat3<T> & rotation_world_from_body,
    const Vec3<T> & acceleration_world);

  /** @brief 数值稳定的 sigmoid 函数。 */
  static T sigmoid(T x);

  /** @brief 清除接触值并将全部输出标记为无效，避免沿用上一帧结果。 */
  void invalidateContacts(T timestamp);

  const Quadruped<T> * quadruped_ = nullptr;  ///< 非拥有指针，提供机身质量与腿几何。
  ImuSensor * imu_ = nullptr;                 ///< 非拥有指针，提供姿态与比力。
  LegSensors legs_{};                         ///< 非拥有指针，提供关节位置与力矩。
  ContactEstimatorParameters<T> parameters_;
  FootContacts contacts_{};                   ///< 最近一次四腿接触结果。
  ContactProbabilities probabilities_{};      ///< 最近一次四腿接触概率。
  std::array<JointState<T>, kNumLegs> joint_states_{};  ///< 最近一次四腿反馈。
  bool legs_valid_ = false;
  T timestamp_ = T(0);
};

#endif  // MYMIT_ROBOT_CONTROLLER_CONTACT_ESTIMATOR_HPP_
