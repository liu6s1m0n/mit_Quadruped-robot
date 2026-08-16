#include <algorithm>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>

#include "RobotMiddlewareInterface.hpp"
#include "SimulationBridge.hpp"

extern "C" int mymit_robot_run_simulation(
  const SimulationBackendConfig * config, RobotMiddlewareInterface * middleware,
  char * error_buffer, std::size_t error_buffer_size)
{
  try {
    if (config == nullptr || config->scene_file == nullptr || middleware == nullptr) {
      throw std::invalid_argument("simulation backend received an invalid argument");
    }
    SimulationBridge bridge(config->scene_file, middleware);
    bridge.setStandingHeight(config->standing_height);
    bridge.setSlowWalkingForwardSpeed(config->slow_walking_speed);
    bridge.setFastWalkingForwardSpeed(config->fast_walking_speed);
    bridge.setWalkingBackwardSpeed(config->backward_walking_speed);
    bridge.setWalkingLateralSpeed(config->lateral_walking_speed);
    bridge.setTurningYawRate(config->turning_yaw_rate);
    return bridge.run();
  } catch (const std::exception & error) {
    if (error_buffer != nullptr && error_buffer_size > 0U) {
      std::snprintf(error_buffer, error_buffer_size, "%s", error.what());
    }
    return 1;
  }
}
