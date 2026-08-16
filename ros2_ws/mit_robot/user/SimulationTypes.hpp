#ifndef MYMIT_ROBOT_USER_SIMULATION_TYPES_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_TYPES_HPP_

#include <cstddef>
#include <limits>

struct SimulationDiagnosticReport
{
  float maximum_position_error = 0.0F;
  float maximum_velocity_error = 0.0F;
  float maximum_orientation_error = 0.0F;
  float maximum_absolute_pitch = 0.0F;
  float minimum_height = std::numeric_limits<float>::infinity();
  std::size_t calf_collision_frames = 0;
  std::size_t rejected_control_frames = 0;
  std::size_t observed_frames = 0;
  double squared_position_error_sum = 0.0;
  double squared_velocity_error_sum = 0.0;

  float rmsPositionError() const noexcept;
  float rmsVelocityError() const noexcept;
};

struct SimulationBackendConfig
{
  const char * scene_file = nullptr;
  float standing_height = 0.27F;
  float slow_walking_speed = 0.18F;
  float fast_walking_speed = 0.36F;
  float backward_walking_speed = 0.30F;
  float lateral_walking_speed = 0.25F;
  float turning_yaw_rate = 0.35F;
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_TYPES_HPP_
