/**
 * @file Gait.h
 * @brief 用“周期偏移 + 接触持续时间”描述四条腿的离散步态。
 *
 * contactPhase 为支撑期进度，swingPhase 为摆动期进度，取值均为 0 到 1。
 */
#ifndef MYMIT_ROBOT_MPC_GAIT_H_
#define MYMIT_ROBOT_MPC_GAIT_H_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "model/robot_types.hpp"

namespace mpc
{

class OffsetDurationGait
{
public:
  /**
   * @brief 创建离散步态。
   * @param horizon 一个步态周期包含的离散段数。
   * @param offsets 每条腿相对于周期起点的支撑段偏移。
   * @param durations 每条腿每周期的支撑段数。
   * @param name 步态名称，用于日志和调试。
   */
  OffsetDurationGait(
    std::size_t horizon, const std::array<std::size_t, kNumLegs> & offsets,
    const std::array<std::size_t, kNumLegs> & durations,
    std::string name = "gait");

  /**
   * @brief 根据高速控制周期更新当前步态段及段内连续相位。
   * @param control_iteration 当前控制周期编号。
   * @param iterations_per_segment 每个步态段对应的控制周期数。
   */
  void advance(std::size_t control_iteration, std::size_t iterations_per_segment);
  /** @brief 返回每条腿的支撑期相位，支撑时为 [0,1]，否则为 0。 */
  std::array<float, kNumLegs> contactPhase() const noexcept;
  /** @brief 返回每条腿的摆动期相位，摆动时为 [0,1]，否则为 0。 */
  std::array<float, kNumLegs> swingPhase() const noexcept;
  /**
   * @brief 生成预测窗接触表。
   * @return 按 [预测段][腿号] 排列的数组，1 表示支撑，0 表示摆动。
   */
  const std::vector<int> & contactTable();
  /** @brief 根据单段时间计算指定腿的支撑总时长。 */
  float stanceTime(float segment_time, std::size_t leg) const;
  /** @brief 根据单段时间计算指定腿的摆动总时长。 */
  float swingTime(float segment_time, std::size_t leg) const;
  std::size_t phaseSegment() const noexcept {return phase_segment_;}
  std::size_t horizon() const noexcept {return horizon_;}

private:
  float phaseWithin(std::size_t leg, bool contact) const noexcept;

  std::size_t horizon_;  ///< 一个完整离散步态周期包含的段数，也是接触表周期长度。
  std::array<std::size_t, kNumLegs> offsets_;  ///< 每条腿支撑期相对周期起点的段偏移。
  std::array<std::size_t, kNumLegs> durations_;  ///< 每条腿每周期持续接触的段数。
  std::string name_;  ///< 供日志显示的步态名称。
  std::size_t phase_segment_ = 0;  ///< 当前位于步态周期中的离散段编号。
  float segment_fraction_ = 0.0F;  ///< 当前离散段内部的连续进度，范围 [0,1)。
  std::vector<int> table_;  ///< MPC 预测窗接触表；每项 1 表示支撑，0 表示摆动。
};

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_GAIT_H_
