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
  OffsetDurationGait(
    std::size_t horizon, const std::array<std::size_t, kNumLegs> & offsets,
    const std::array<std::size_t, kNumLegs> & durations,
    std::string name = "gait");

  void advance(std::size_t control_iteration, std::size_t iterations_per_segment);
  std::array<float, kNumLegs> contactPhase() const noexcept;
  std::array<float, kNumLegs> swingPhase() const noexcept;
  const std::vector<int> & contactTable();
  float stanceTime(float segment_time, std::size_t leg) const;
  float swingTime(float segment_time, std::size_t leg) const;
  std::size_t phaseSegment() const noexcept {return phase_segment_;}
  std::size_t horizon() const noexcept {return horizon_;}

private:
  float phaseWithin(std::size_t leg, bool contact) const noexcept;

  std::size_t horizon_;
  std::array<std::size_t, kNumLegs> offsets_;
  std::array<std::size_t, kNumLegs> durations_;
  std::string name_;
  std::size_t phase_segment_ = 0;
  float segment_fraction_ = 0.0F;
  std::vector<int> table_;
};

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_GAIT_H_
