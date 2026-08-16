/*! @file GaitScheduler.hpp
 *  @brief 四足机器人固定步态的相位与接触时序调度器。
 *
 *  本文件不依赖特定用户界面或参数服务。上层只需将参数转换为
 *  GaitSchedulerParameters，步态逻辑因此可同时用于 MuJoCo、ROS 2 和真机。
 */

#ifndef MYMIT_ROBOT_CONTROLLER_GAIT_SCHEDULER_HPP_
#define MYMIT_ROBOT_CONTROLLER_GAIT_SCHEDULER_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>

#include "model/robot_types.hpp"

/** @brief 预定义步态类型。 */
enum class GaitType : std::uint8_t
{
  STAND = 0,
  STAND_CYCLE,
  STATIC_WALK,
  AMBLE,
  TROT_WALK,
  TROT,
  TROT_RUN,
  PACE,
  BOUND,
  ROTARY_GALLOP,
  TRAVERSE_GALLOP,
  PRONK,
  THREE_FOOT,
  CUSTOM,
  TRANSITION_TO_STAND
};

/**
 * @brief 步态参数对预定义或外部步态请求的覆盖方式。
 *
 * 数值保持与原 MIT 用户参数 gait_override 一致。
 */
enum class GaitOverrideMode : std::uint8_t
{
  USE_REQUESTED_GAIT = 0,       ///< 使用 requestGait() 设置的步态。
  USE_PREDEFINED_GAIT = 1,      ///< 使用 parameters.gait_type 的预定义参数。
  OVERRIDE_TIMING = 2,          ///< 选择预定义步态并覆盖周期/支撑比。
  USE_NATURAL_GAIT = 3,         ///< 使用 requestGait() 设置的自然步态。
  OVERRIDE_NATURAL_TIMING = 4   ///< 自然步态另外使用实时周期/支撑比。
};

/** @brief 调度器的运行时参数，可由 ROS 2 参数或用户命令填充。 */
template<typename T>
struct GaitSchedulerParameters
{
  GaitOverrideMode override_mode = GaitOverrideMode::USE_REQUESTED_GAIT;
  GaitType gait_type = GaitType::STAND;
  T gait_period_time = T(0.5);       ///< 覆盖步态周期，s。
  T gait_switching_phase = T(0.5);   ///< 覆盖支撑相占整周期的比例。

  /** @brief 检查枚举、周期和切换相是否可用。 */
  bool isValid() const noexcept;
};

/**
 * @brief 一个控制周期内的步态时序数据。
 *
 * 所有四维向量下标均按 LegId 固定为 FR、FL、RR、RL。
 */
template<typename T>
struct GaitData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GaitData() {zero();}

  /** @brief 恢复为安全站立前的全零状态。 */
  void zero();

  GaitType current_gait = GaitType::STAND;
  GaitType next_gait = GaitType::STAND;
  std::string gait_name;

  T period_time_nominal = T(0);      ///< 整个步态周期，s。
  T initial_phase = T(0);            ///< 全局初始相位，[0, 1)。
  T switching_phase_nominal = T(0); ///< 从支撑切换到摆动的名义相位。
  bool overrideable = false;

  Eigen::Vector4i gait_enabled = Eigen::Vector4i::Zero();

  Vec4<T> period_time = Vec4<T>::Zero(); // 整个步态周期，s。
  Vec4<T> time_stance = Vec4<T>::Zero(); // 支撑时间，s。
  Vec4<T> time_swing = Vec4<T>::Zero();  // 摆动时间，s。
  Vec4<T> time_stance_remaining = Vec4<T>::Zero(); // 剩余支撑时间，s。
  Vec4<T> time_swing_remaining = Vec4<T>::Zero();  // 剩余摆动时间，s。

  Vec4<T> switching_phase = Vec4<T>::Zero(); // 从支撑切换到摆动的名义相位。
  Vec4<T> phase_variable = Vec4<T>::Zero();  // 全局归一化相位，[0, 1)。
  Vec4<T> phase_offset = Vec4<T>::Zero(); // 相位偏移量
  Vec4<T> phase_scale = Vec4<T>::Zero();  // 相位缩放量
  Vec4<T> phase_stance = Vec4<T>::Zero(); // 归一化支撑子相位
  Vec4<T> phase_swing = Vec4<T>::Zero();  // 归一化摆动子相位

  Eigen::Vector4i contact_state_scheduled = Eigen::Vector4i::Zero();// 计划接触状态，1=支撑，0=摆动
  Eigen::Vector4i contact_state_previous = Eigen::Vector4i::Zero();// 记录上周期的接触状态
  Eigen::Vector4i touchdown_scheduled = Eigen::Vector4i::Zero();// 计划触地事件，1=触地，0=未触地
  Eigen::Vector4i liftoff_scheduled = Eigen::Vector4i::Zero();// 计划离地事件，1=离地，0=未离地

  /** @brief 按 LegId 读取本周期计划接触状态。 */
  bool contactScheduled(LegId leg) const noexcept
  {
    return contact_state_scheduled(static_cast<Eigen::Index>(leg)) != 0;
  }

  /** @brief 按 LegId 读取归一化支撑子相位。 */
  T stancePhase(LegId leg) const noexcept
  {
    return phase_stance(static_cast<Eigen::Index>(leg));
  }

  /** @brief 按 LegId 读取归一化摆动子相位。 */
  T swingPhase(LegId leg) const noexcept
  {
    return phase_swing(static_cast<Eigen::Index>(leg));
  }

  /**
   * @brief 返回可直接传给 PositionVelocityEstimator 的计划接触概率。
   *
   * 固定步态调度的输出是二值的：支撑腿为 1，摆动腿为 0。
   */
  std::array<T, kNumLegs> scheduledContactProbabilities() const noexcept
  {
    std::array<T, kNumLegs> probabilities{};
    for (std::size_t index = 0; index < kNumLegs; ++index) {
      probabilities[index] = contact_state_scheduled(
        static_cast<Eigen::Index>(index)) != 0 ? T(1) : T(0);
    }
    return probabilities;
  }

  /** 返回状态估计器使用的平滑接触可信度，降低触地/离地边界的约束权重。 */
  std::array<T, kNumLegs> estimatorContactProbabilities() const noexcept
  {
    std::array<T, kNumLegs> probabilities{};
    if (current_gait == GaitType::STAND || current_gait == GaitType::STAND_CYCLE) {
      probabilities.fill(T(1));
      return probabilities;
    }
    constexpr T transition_fraction = T(0.2);
    for (std::size_t index = 0; index < kNumLegs; ++index) {
      const Eigen::Index foot = static_cast<Eigen::Index>(index);
      if (contact_state_scheduled(foot) == 0) {
        probabilities[index] = T(0);
        continue;
      }
      const T phase = std::clamp(phase_stance(foot), T(0), T(1));
      probabilities[index] = std::clamp(
        std::min(phase / transition_fraction,
        (T(1) - phase) / transition_fraction), T(0), T(1));
    }
    return probabilities;
  }
};

/** @brief 根据固定步态库生成四腿支撑/摆动相位与触地事件。 */
template<typename T>
class GaitScheduler
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 创建调度器并以站立步态初始化。
   * @param parameters 参数快照，后续可用 setParameters() 更新。
   * @param time_step 控制周期，s，必须为有限正数。
   */
  GaitScheduler(
    const GaitSchedulerParameters<T> & parameters, T time_step);

  /** @brief 使用默认参数创建调度器。 */
  explicit GaitScheduler(T time_step);

  /** @brief 重置并创建站立步态。 */
  void initialize();

  /** @brief 推进一个控制周期。 */
  void step();

  /** @brief 更新由上层传入的步态参数快照。 */
  void setParameters(const GaitSchedulerParameters<T> & parameters);

  /** @brief 请求切换预定义步态；在请求/自然模式的下一周期生效。 */
  void requestGait(GaitType gait);

  /** @brief 设置自然步态模式使用的周期和支撑相比例。 */
  void setNaturalTiming(T period_time, T switching_phase);

  /** @brief 根据当前覆盖模式处理步态变更。 */
  void modifyGait();

  /** @brief 从预定义步态库创建 next_gait。 */
  void createGait();

  /** @brief 根据名义参数重算四腿周期、支撑和摆动时间。 */
  void calculateAuxiliaryGaitData();

  /** @brief 打印当前步态与四腿相位。 */
  void printGaitInfo() const;

  /** @brief 返回最近一次参数快照。 */
  const GaitSchedulerParameters<T> & parameters() const noexcept
  {
    return parameters_;
  }

  GaitData<T> gait_data;

  // 保留原有自然步态调节量，供上层 FSM/MPC 实时修改。
  T period_time_natural = T(0.5);
  T switching_phase_natural = T(0.5);
  T swing_time_natural = T(0.25);

private:
  void configureGait(
    const char * name, T period_time, T switching_phase,
    const Eigen::Vector4i & enabled, const Vec4<T> & phase_offset,
    const Vec4<T> & phase_scale, bool overrideable);

  static bool gaitTypeIsValid(GaitType gait) noexcept;
  static T wrapPhase(T phase);

  GaitSchedulerParameters<T> parameters_{};
  T dt_ = T(0);
};

static_assert(kNumLegs == 4, "GaitScheduler requires exactly four legs");

#endif  // MYMIT_ROBOT_CONTROLLER_GAIT_SCHEDULER_HPP_
