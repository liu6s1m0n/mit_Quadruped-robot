/**
 * @file RobotRosNode.hpp
 * @brief ROS 2 API adapter for the MuJoCo control application.
 *
 * This class owns middleware resources only.  RobotRunner remains independent
 * of ROS 2 and keeps the estimator/FSM/MPC/WBC algorithm unchanged.
 */
#ifndef MYMIT_ROBOT_USER_ROBOT_ROS_NODE_HPP_
#define MYMIT_ROBOT_USER_ROBOT_ROS_NODE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "RobotMiddlewareInterface.hpp"
#include "model/robot_types.hpp"
#include "mymit_robot/msg/foot_contact.hpp"
#include "mymit_robot/msg/state_estimate.hpp"
#include "mymit_robot/srv/set_control_mode.hpp"
#include "mymit_robot/srv/set_standing_height.hpp"

class RobotRosNode : public rclcpp::Node, public RobotMiddlewareInterface
{
public:
  explicit RobotRosNode(const std::string & default_scene_file);

  const std::string & sceneFile() const noexcept {return scene_file_;}
  float initialStandingHeight() const noexcept {return initial_standing_height_;}
  float slowWalkingSpeed() const noexcept {return slow_walking_speed_;}
  float fastWalkingSpeed() const noexcept {return fast_walking_speed_;}
  float backwardWalkingSpeed() const noexcept {return backward_walking_speed_;}
  float lateralWalkingSpeed() const noexcept {return lateral_walking_speed_;}
  float turningYawRate() const noexcept {return turning_yaw_rate_;}

  bool middlewareOk() const noexcept override {return rclcpp::ok();}
  bool takeStandingHeight(float & height) noexcept override;
  std::optional<ControlMode> takeControlModeRequest() noexcept override;
  RosVelocityCommand velocityCommand() const override;
  bool cmdVelActivatesLocomotion() const noexcept override
  {
    return cmd_vel_activates_locomotion_;
  }

  void setCurrentMode(ControlMode mode) override;
  void publishClock(double simulation_time) override;
  void publishState(
    double simulation_time, const StateEstimate<float> & estimate,
    const std::array<double, kJointCount> & joint_position,
    const std::array<double, kJointCount> & joint_velocity,
    const std::array<double, kJointCount> & joint_effort,
    const RosContactSnapshot & contacts,
    const SimulationDiagnosticReport & diagnostics, bool control_valid) override;

private:
  static bool validMode(std::uint8_t mode) noexcept;
  static builtin_interfaces::msg::Time simulationStamp(double seconds);
  bool shouldPublish(double simulation_time) noexcept;

  std::string scene_file_;
  std::string odom_frame_;
  std::string base_frame_;
  float initial_standing_height_ = 0.27F;
  float slow_walking_speed_ = 0.18F;
  float fast_walking_speed_ = 0.36F;
  float backward_walking_speed_ = 0.30F;
  float lateral_walking_speed_ = 0.25F;
  float turning_yaw_rate_ = 0.35F;
  double publish_period_ = 0.01;
  double command_timeout_ = 0.5;
  bool cmd_vel_activates_locomotion_ = true;
  double last_publish_time_ = -1.0;

  std::atomic<float> requested_height_{0.27F};
  std::atomic<std::uint64_t> height_sequence_{0};
  std::uint64_t consumed_height_sequence_ = 0;
  std::atomic<int> requested_mode_{-1};
  std::atomic<std::uint8_t> current_mode_{
    static_cast<std::uint8_t>(ControlMode::BalanceStand)};

  mutable std::mutex velocity_mutex_;
  geometry_msgs::msg::Twist velocity_command_;
  std::chrono::steady_clock::time_point velocity_received_at_{};
  bool velocity_received_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Service<mymit_robot::srv::SetStandingHeight>::SharedPtr height_service_;
  rclcpp::Service<mymit_robot::srv::SetControlMode>::SharedPtr mode_service_;
  rclcpp::Publisher<mymit_robot::msg::StateEstimate>::SharedPtr state_publisher_;
  rclcpp::Publisher<mymit_robot::msg::FootContact>::SharedPtr contact_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

#endif  // MYMIT_ROBOT_USER_ROBOT_ROS_NODE_HPP_
