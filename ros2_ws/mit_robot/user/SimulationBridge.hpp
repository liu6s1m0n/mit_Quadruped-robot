/**
 * @file SimulationBridge.hpp
 * @brief 连接 MuJoCo 渲染/物理循环与 RobotRunner 控制循环。
 *
 * 主线程负责 GLFW 和绘制，物理线程负责状态读取、控制计算和 mj_step。
 */
#ifndef MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_

#include <atomic>
#include <string>

/** 管理 MuJoCo GUI 与物理对象生命周期，并将其接到 RobotRunner。 */
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
  int walking_toggle_ = 0;
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
