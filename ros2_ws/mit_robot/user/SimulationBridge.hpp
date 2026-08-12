/**
 * @file SimulationBridge.hpp
 * @brief 连接 MuJoCo 渲染/物理循环与 RobotRunner 控制循环。
 *
 * 主线程负责 GLFW 和绘制，物理线程负责状态读取、控制计算和 mj_step。
 */
#ifndef MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_

#include <array>
#include <atomic>
#include <string>

/** 管理 MuJoCo GUI 与物理对象生命周期，并将其接到 RobotRunner。 */
class SimulationBridge
{
public:
  explicit SimulationBridge(std::string scene_path);
  int run();
  void setStandingHeight(float height);
  /** 设置进入 Locomotion 后使用的固定前进速度，单位 m/s。 */
  void setWalkingForwardSpeed(float speed);
  /** 设置左右平移速度的绝对值，单位 m/s。 */
  void setWalkingLateralSpeed(float speed);
  /** 设置自转角速度的绝对值，单位 rad/s，正方向为逆时针。 */
  void setTurningYawRate(float yaw_rate);
  float standingHeight() const noexcept {return standing_height_.load();}
  float walkingForwardSpeed() const noexcept {return walking_forward_speed_;}
  float walkingLateralSpeed() const noexcept {return walking_lateral_speed_;}
  float turningYawRate() const noexcept {return turning_yaw_rate_;}

private:
  std::string scene_path_;
  std::atomic<float> standing_height_{0.27F};
  float walking_forward_speed_ = 0.32F;
  float walking_lateral_speed_ = 0.25F;
  float turning_yaw_rate_ = 0.35F;
  double standing_height_slider_ = 0.27;
  std::array<int, 5> direction_toggles_{};
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
