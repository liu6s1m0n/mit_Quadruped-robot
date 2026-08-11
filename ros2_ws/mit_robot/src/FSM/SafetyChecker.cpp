#include "FSM/SafetyChecker.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
template<typename T>
bool clampCommand(
  T & value, T lower, T upper, std::size_t leg, const char * quantity,
  std::size_t coordinate)
{
  if (!std::isfinite(static_cast<double>(value))) {
    std::cerr << "[CONTROL FSM] Safety: non-finite " << quantity
              << " for leg " << leg << ", coordinate " << coordinate << '\n';
    return false;
  }
  const T original = value;
  value = std::clamp(value, lower, upper);
  if (value == original) {return true;}
  std::cerr << "[CONTROL FSM] Safety: clamped " << quantity << " for leg "
            << leg << ", coordinate " << coordinate << " from " << original
            << " to " << value << '\n';
  return false;
}
}  // namespace

template<typename T>
SafetyChecker<T>::SafetyChecker(ControlFSMData<T> * data)
: data_(data)
{
  if (data_ == nullptr || !data_->valid()) {
    throw std::invalid_argument("safety checker requires valid FSM data");
  }
}

template<typename T>
bool SafetyChecker<T>::checkSafeOrientation() const
{
  const auto & estimate = *data_->state_estimate;
  constexpr T maximum_roll_pitch = T(1.4);  // 约 80 度。
  return estimate.valid && estimate.rpy.allFinite() &&
         std::abs(estimate.rpy.x()) <= maximum_roll_pitch &&
         std::abs(estimate.rpy.y()) <= maximum_roll_pitch;
}

template<typename T>
bool SafetyChecker<T>::checkPDesFoot()
{
  bool safe = true;
  constexpr T sine_sixty_degrees = T(0.8660254037844386);

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const LegId leg_id = static_cast<LegId>(leg);
    const T maximum_length = data_->quadruped->leg(leg_id).maximumLegLength();
    const T maximum_lateral_position = maximum_length * sine_sixty_degrees;
    auto & position = data_->leg_controller->commands[leg].foot_position_desired;

    safe = clampCommand(
      position.x(), -maximum_lateral_position, maximum_lateral_position,
      leg, "desired foot position", 0) && safe;
    safe = clampCommand(
      position.y(), -maximum_lateral_position, maximum_lateral_position,
      leg, "desired foot position", 1) && safe;
    safe = clampCommand(
      position.z(), -maximum_length, -maximum_length / T(4),
      leg, "desired foot position", 2) && safe;
  }
  return safe;
}

template<typename T>
bool SafetyChecker<T>::checkForceFeedForward()
{
  bool safe = true;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const LegId leg_id = static_cast<LegId>(leg);
    const auto & model = data_->quadruped->leg(leg_id);
    const auto & torque_limit = model.joints.torque_limit;

    // 由当前机型的力矩和连杆长度推导粗略笛卡尔上限，避免按机器人名称写死常数。
    const T sagittal_limit = std::min(
      torque_limit.y() / model.thigh_link_length,
      torque_limit.z() / model.calf_link_length);
    const T lateral_limit = torque_limit.x() / model.hip_link_length;
    auto & force = data_->leg_controller->commands[leg].force_feedforward;

    safe = clampCommand(
      force.x(), -sagittal_limit, sagittal_limit,
      leg, "feedforward force", 0) && safe;
    safe = clampCommand(
      force.y(), -lateral_limit, lateral_limit,
      leg, "feedforward force", 1) && safe;
    safe = clampCommand(
      force.z(), -sagittal_limit, sagittal_limit,
      leg, "feedforward force", 2) && safe;
  }
  return safe;
}

template class SafetyChecker<float>;
