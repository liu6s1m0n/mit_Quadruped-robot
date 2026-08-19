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

#include "model/robot_types.hpp"

/** 管理 MuJoCo GUI 与物理对象生命周期，并将其接到 RobotRunner。 */
class SimulationBridge
{
public:
  /** @brief 绑定场景文件及其对应的控制参数机型。 */
  SimulationBridge(std::string scene_path, RobotType robot_type);
  int run();
  void setStandingHeight(float height);
  /** 设置低速稳定前进档，单位 m/s。 */
  void setSlowWalkingForwardSpeed(float speed);
  /** 设置快速前进档，单位 m/s。 */
  void setFastWalkingForwardSpeed(float speed);
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
  std::string scene_path_;  ///< 要加载的 MuJoCo scene.xml 绝对或相对路径。
  RobotType robot_type_;    ///< 与场景匹配的控制器参数型号。
  std::atomic<float> standing_height_{0.27F};  ///< GUI 线程传给物理线程的目标机身高度，m。
  std::atomic<bool> stand_up_requested_{false};  ///< GUI 发出且等待物理线程消费的站起请求。
  std::atomic<bool> front_jump_requested_{false};  ///< GUI 发出且等待物理线程消费的前跳请求。
  float slow_walking_forward_speed_ = 0.18F;  ///< 键盘低速前进档速度，m/s。
  float fast_walking_forward_speed_ = 0.36F;  ///< 键盘快速前进档速度，m/s。
  float walking_backward_speed_ = 0.30F;      ///< 键盘后退档速度绝对值，m/s。
  float walking_lateral_speed_ = 0.25F;       ///< 左右平移档速度绝对值，m/s。
  float turning_yaw_rate_ = 0.35F;            ///< 原地自转角速度绝对值，rad/s。
  double standing_height_slider_ = 0.27;      ///< MuJoCo UI 滑块使用的双精度高度缓存，m。
  std::array<int, 6> direction_toggles_{};     ///< 六个方向按钮的 UI 开关值，不是控制器状态。
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
