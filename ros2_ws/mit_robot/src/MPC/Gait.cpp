/**
 * @file Gait.cpp
 * @brief 离散步态相位、接触表和支撑/摆动时长的实现。
 */

#include "MPC/Gait.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

// 步态时序实现：用“起始偏移 + 支撑持续段数”描述每条腿在循环中的接触状态。
namespace mpc
{

/**
 * @brief 根据每条腿的周期偏移和支撑持续时间创建步态。
 * @throws std::invalid_argument 当 horizon、offset 或 duration 非法时抛出。
 */
OffsetDurationGait::OffsetDurationGait(
  std::size_t horizon, const std::array<std::size_t, kNumLegs> & offsets,
  const std::array<std::size_t, kNumLegs> & durations, std::string name)
: horizon_(horizon), offsets_(offsets), durations_(durations),
  name_(std::move(name)), table_(horizon * kNumLegs, 0)
{
  if (horizon_ == 0) {throw std::invalid_argument("gait horizon must be positive");}
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (offsets_[leg] >= horizon_ || durations_[leg] > horizon_) {
      throw std::invalid_argument("gait offset or duration is outside horizon");
    }
  }
}

/**
 * @brief 将控制周期映射到当前步态段和段内连续相位。
 *
 * @f$segment=\lfloor k/N_s\rfloor\bmod H@f$，
 * @f$f=(k\bmod N_s)/N_s@f$。
 */
void OffsetDurationGait::advance(
  std::size_t control_iteration, std::size_t iterations_per_segment)
{
  if (iterations_per_segment == 0) {
    throw std::invalid_argument("iterations per gait segment must be positive");
  }
  // 一个 horizon 被分成若干步态段；segment_fraction_ 表示当前段内的连续进度。
  //phase_segment_表示当前位于步态周期的第几个离散段。
  phase_segment_ = (control_iteration / iterations_per_segment) % horizon_;
  segment_fraction_ = static_cast<float>(control_iteration % iterations_per_segment) /
    static_cast<float>(iterations_per_segment);
}

/**
 * @brief 计算指定腿在支撑期或摆动期内的归一化相位。
 * @param leg 腿编号。
 * @param contact true 表示计算支撑相位，false 表示计算摆动相位。
 * @return 当前阶段内的 [0,1] 相位，不在该阶段时返回 0。
 */
float OffsetDurationGait::phaseWithin(std::size_t leg, bool contact) const noexcept
{
  // 加上 horizon_支撑时间  再取模，可在循环边界处安全计算相对相位，避免无符号数下溢。
  const std::size_t duration =
     contact ? durations_[leg] : horizon_ - durations_[leg];
  if (duration == 0) {return contact ? 1.0F : 0.0F;}
  //5.3 计算相对周期位置
  const std::size_t relative =
  (phase_segment_ + horizon_ - offsets_[leg]) % horizon_;
  //判断当前是否处于支撑期
  const bool active =
  contact ? relative < durations_[leg] : relative >= durations_[leg];
  //判断当前是否处于支撑期
  if (!active) {return 0.0F;}
  //计算已经经过的离散段数
  const std::size_t elapsed =
  contact ? relative : relative - durations_[leg];
  return std::min(1.0F, (static_cast<float>(elapsed) + segment_fraction_) /
    static_cast<float>(duration));
}

/*其中每个元素含义是：
  当前不支撑：0；
  正在支撑：0 到 1；
  支撑阶段刚结束：接近 1。
  注意：它不是布尔值，而是连续相位。*/
/** @brief 汇总四条腿的支撑期相位。 */
std::array<float, kNumLegs> OffsetDurationGait::contactPhase() const noexcept
{
  std::array<float, kNumLegs> result{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {result[leg] = phaseWithin(leg, true);}
  return result;
}
/*其含义是：
  当前不摆动：0；
  正在摆动：0 到 1；
  摆动阶段结束：接近 1。
  在摆腿轨迹生成中，通常会用到这个相位：
*/
/** @brief 汇总四条腿的摆动期相位。 */
std::array<float, kNumLegs> OffsetDurationGait::swingPhase() const noexcept
{
  std::array<float, kNumLegs> result{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {result[leg] = phaseWithin(leg, false);}
  return result;
}

/**
 * @brief 按当前相位重新生成整个预测窗的接触表。
 * @return 内部接触表引用，下一次调用可能被覆盖。
 */
const std::vector<int> & OffsetDurationGait::contactTable()
{
  // 表采用 [预测步][腿号] 的连续布局：1 表示支撑，0 表示摆动。
  for (std::size_t step = 0; step < horizon_; ++step) {
    const std::size_t segment = (phase_segment_ + step) % horizon_;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const std::size_t relative = (segment + horizon_ - offsets_[leg]) % horizon_;
      table_[step * kNumLegs + leg] = relative < durations_[leg] ? 1 : 0;
    }
  }
  return table_;
}

/**
 * @brief 计算指定腿的支撑时长。
 * @throws std::invalid_argument 当腿编号或段时间非法时抛出。
 */
float OffsetDurationGait::stanceTime(float segment_time, std::size_t leg) const
{
  if (leg >= kNumLegs || segment_time <= 0.0F) {
    throw std::invalid_argument("invalid stance time input");
  }
  return segment_time * static_cast<float>(durations_[leg]);
}

/**
 * @brief 计算指定腿的摆动时长。
 * @throws std::invalid_argument 当腿编号或段时间非法时抛出。
 */
float OffsetDurationGait::swingTime(float segment_time, std::size_t leg) const
{
  if (leg >= kNumLegs || segment_time <= 0.0F) {
    throw std::invalid_argument("invalid swing time input");
  }
  return segment_time * static_cast<float>(horizon_ - durations_[leg]);
}

}  // namespace mpc
