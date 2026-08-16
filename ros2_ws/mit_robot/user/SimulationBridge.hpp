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
#include <memory>
#include <string>

class RobotMiddlewareInterface;

/** 管理 MuJoCo GUI 与物理对象生命周期，并将其接到 RobotRunner。 */
class SimulationBridge
{
public:
  explicit SimulationBridge(
    std::string scene_path, RobotMiddlewareInterface * middleware = nullptr);
  int run();
  void setStandingHeight(float height);
  /** 设置低速稳定前进档，单位 m/s。 */
  void setSlowWalkingForwardSpeed(float speed);
  /** 设置快速前进档，单位 m/s。 */
  void setFastWalkingForwardSpeed(float speed);
  /** 设置后退速度的绝对值，单位 m/s。 */
  void setWalkingBackwardSpeed(float speed);
  /** 设置左右平移速度的绝对值，单位 m/s。 */
  void setWalkingLateralSpeed(float speed);
  /** 设置自转角速度的绝对值，单位 rad/s，正方向为逆时针。 */
  void setTurningYawRate(float yaw_rate);
  float standingHeight() const noexcept {return standing_height_.load();}
  float slowWalkingForwardSpeed() const noexcept {return slow_walking_forward_speed_;}
  float fastWalkingForwardSpeed() const noexcept {return fast_walking_forward_speed_;}
  float walkingLateralSpeed() const noexcept {return walking_lateral_speed_;}
  float turningYawRate() const noexcept {return turning_yaw_rate_;}

private:
  std::string scene_path_;
  RobotMiddlewareInterface * middleware_ = nullptr;
  std::atomic<float> standing_height_{0.27F};
  std::atomic<bool> front_jump_requested_{false};
  float slow_walking_forward_speed_ = 0.18F;
  float fast_walking_forward_speed_ = 0.36F;
  float walking_backward_speed_ = 0.30F;
  float walking_lateral_speed_ = 0.25F;
  float turning_yaw_rate_ = 0.35F;
  double standing_height_slider_ = 0.27;
  std::array<int, 6> direction_toggles_{};
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
