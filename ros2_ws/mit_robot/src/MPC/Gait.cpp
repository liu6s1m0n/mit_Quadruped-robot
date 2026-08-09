#include "MPC/Gait.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace mpc
{

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

void OffsetDurationGait::advance(
  std::size_t control_iteration, std::size_t iterations_per_segment)
{
  if (iterations_per_segment == 0) {
    throw std::invalid_argument("iterations per gait segment must be positive");
  }
  phase_segment_ = (control_iteration / iterations_per_segment) % horizon_;
  segment_fraction_ = static_cast<float>(control_iteration % iterations_per_segment) /
    static_cast<float>(iterations_per_segment);
}

float OffsetDurationGait::phaseWithin(std::size_t leg, bool contact) const noexcept
{
  const std::size_t duration = contact ? durations_[leg] : horizon_ - durations_[leg];
  if (duration == 0) {return contact ? 1.0F : 0.0F;}
  const std::size_t relative = (phase_segment_ + horizon_ - offsets_[leg]) % horizon_;
  const bool active = contact ? relative < durations_[leg] : relative >= durations_[leg];
  if (!active) {return 0.0F;}
  const std::size_t elapsed = contact ? relative : relative - durations_[leg];
  return std::min(1.0F, (static_cast<float>(elapsed) + segment_fraction_) /
    static_cast<float>(duration));
}

std::array<float, kNumLegs> OffsetDurationGait::contactPhase() const noexcept
{
  std::array<float, kNumLegs> result{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {result[leg] = phaseWithin(leg, true);}
  return result;
}

std::array<float, kNumLegs> OffsetDurationGait::swingPhase() const noexcept
{
  std::array<float, kNumLegs> result{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {result[leg] = phaseWithin(leg, false);}
  return result;
}

const std::vector<int> & OffsetDurationGait::contactTable()
{
  for (std::size_t step = 0; step < horizon_; ++step) {
    const std::size_t segment = (phase_segment_ + step) % horizon_;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const std::size_t relative = (segment + horizon_ - offsets_[leg]) % horizon_;
      table_[step * kNumLegs + leg] = relative < durations_[leg] ? 1 : 0;
    }
  }
  return table_;
}

float OffsetDurationGait::stanceTime(float segment_time, std::size_t leg) const
{
  if (leg >= kNumLegs || segment_time <= 0.0F) {
    throw std::invalid_argument("invalid stance time input");
  }
  return segment_time * static_cast<float>(durations_[leg]);
}

float OffsetDurationGait::swingTime(float segment_time, std::size_t leg) const
{
  if (leg >= kNumLegs || segment_time <= 0.0F) {
    throw std::invalid_argument("invalid swing time input");
  }
  return segment_time * static_cast<float>(horizon_ - durations_[leg]);
}

}  // namespace mpc
