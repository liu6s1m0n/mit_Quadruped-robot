#ifndef MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_

#include <atomic>
#include <string>

/** Owns the MuJoCo GUI/physics lifecycle and bridges it to RobotRunner. */
class SimulationBridge
{
public:
  explicit SimulationBridge(std::string scene_path);
  int run();
  void setStandingHeight(float height);
  float standingHeight() const noexcept {return standing_height_.load();}

private:
  std::string scene_path_;
  std::atomic<float> standing_height_{0.27F};
  double standing_height_slider_ = 0.27;
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
