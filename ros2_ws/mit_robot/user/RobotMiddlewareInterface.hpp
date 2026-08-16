#ifndef MYMIT_ROBOT_USER_ROBOT_MIDDLEWARE_INTERFACE_HPP_
#define MYMIT_ROBOT_USER_ROBOT_MIDDLEWARE_INTERFACE_HPP_

#include <array>
#include <cstddef>
#include <optional>

#include "SimulationTypes.hpp"
#include "model/robot_types.hpp"

struct RosVelocityCommand
{
  float forward = 0.0F;
  float lateral = 0.0F;
  float yaw_rate = 0.0F;
  bool received = false;
  bool active = false;
};

struct RosContactSnapshot
{
  std::array<bool, kNumLegs> contact{};
  std::array<double, kNumLegs> probability{};
  std::array<double, kNumLegs> normal_force{};
  bool valid = false;
};

class RobotMiddlewareInterface
{
public:
  static constexpr std::size_t kJointCount = kNumLegs * kJointsPerLeg;
  virtual ~RobotMiddlewareInterface() = default;

  virtual bool middlewareOk() const noexcept = 0;
  virtual bool takeStandingHeight(float & height) noexcept = 0;
  virtual std::optional<ControlMode> takeControlModeRequest() noexcept = 0;
  virtual RosVelocityCommand velocityCommand() const = 0;
  virtual bool cmdVelActivatesLocomotion() const noexcept = 0;
  virtual void setCurrentMode(ControlMode mode) = 0;
  virtual void publishClock(double simulation_time) = 0;
  virtual void publishState(
    double simulation_time, const StateEstimate<float> & estimate,
    const std::array<double, kJointCount> & joint_position,
    const std::array<double, kJointCount> & joint_velocity,
    const std::array<double, kJointCount> & joint_effort,
    const RosContactSnapshot & contacts,
    const SimulationDiagnosticReport & diagnostics, bool control_valid) = 0;
};

using RunSimulationBackend = int (*)(
  const SimulationBackendConfig *, RobotMiddlewareInterface *, char *, std::size_t);

#endif  // MYMIT_ROBOT_USER_ROBOT_MIDDLEWARE_INTERFACE_HPP_
