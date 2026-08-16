#include "RobotRosNode.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace
{
constexpr std::array<const char *, RobotRosNode::kJointCount> kJointNames{{
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}};

diagnostic_msgs::msg::KeyValue diagnosticValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

double finiteParameter(rclcpp::Node & node, const char * name, double default_value)
{
  const double value = node.declare_parameter<double>(name, default_value);
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string("parameter '") + name + "' must be finite");
  }
  return value;
}

double rootMeanSquare(double squared_sum, std::size_t samples) noexcept
{
  return samples == 0U ? 0.0 :
         std::sqrt(squared_sum / static_cast<double>(samples));
}
}  // namespace

RobotRosNode::RobotRosNode(const std::string & default_scene_file)
: Node("mymit_robot")
{
  // Declare parameters only after the rclcpp::Node base is fully constructed.
  scene_file_ = declare_parameter<std::string>("scene_file", default_scene_file);
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  initial_standing_height_ = static_cast<float>(
    finiteParameter(*this, "standing_height", 0.27));
  slow_walking_speed_ = static_cast<float>(
    finiteParameter(*this, "slow_walking_speed", 0.18));
  fast_walking_speed_ = static_cast<float>(
    finiteParameter(*this, "fast_walking_speed", 0.36));
  backward_walking_speed_ = static_cast<float>(
    finiteParameter(*this, "backward_walking_speed", 0.30));
  lateral_walking_speed_ = static_cast<float>(
    finiteParameter(*this, "lateral_walking_speed", 0.25));
  turning_yaw_rate_ = static_cast<float>(
    finiteParameter(*this, "turning_yaw_rate", 0.35));
  const double publish_rate = finiteParameter(*this, "publish_rate", 100.0);
  command_timeout_ = finiteParameter(*this, "cmd_vel_timeout", 0.5);
  cmd_vel_activates_locomotion_ =
    declare_parameter<bool>("cmd_vel_activates_locomotion", true);

  if (scene_file_.empty() || odom_frame_.empty() || base_frame_.empty()) {
    throw std::invalid_argument("scene_file and frame names must not be empty");
  }
  if (initial_standing_height_ < 0.18F || initial_standing_height_ > 0.34F) {
    throw std::invalid_argument("standing_height must be within [0.18, 0.34] m");
  }
  if (slow_walking_speed_ <= 0.0F || fast_walking_speed_ <= slow_walking_speed_ ||
    fast_walking_speed_ > 0.6F || backward_walking_speed_ < 0.0F ||
    backward_walking_speed_ > 0.6F || lateral_walking_speed_ < 0.0F ||
    lateral_walking_speed_ > 0.6F || turning_yaw_rate_ < 0.0F ||
    turning_yaw_rate_ > 1.5F)
  {
    throw std::invalid_argument("walking speed parameters are outside supported limits");
  }
  if (publish_rate <= 0.0 || publish_rate > 500.0 || command_timeout_ <= 0.0) {
    throw std::invalid_argument(
            "publish_rate must be in (0, 500] Hz and cmd_vel_timeout must be positive");
  }
  publish_period_ = 1.0 / publish_rate;
  requested_height_.store(initial_standing_height_);
  height_sequence_.store(1);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  state_publisher_ = create_publisher<mymit_robot::msg::StateEstimate>(
    "state_estimate", sensor_qos);
  contact_publisher_ = create_publisher<mymit_robot::msg::FootContact>(
    "foot_contacts", sensor_qos);
  joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", sensor_qos);
  odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", sensor_qos);
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10);
  clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  mode_publisher_ = create_publisher<std_msgs::msg::UInt8>(
    "control_mode", rclcpp::QoS(1).transient_local().reliable());
  transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
      const bool valid = std::isfinite(message->linear.x) &&
      std::isfinite(message->linear.y) && std::isfinite(message->angular.z) &&
      std::abs(message->linear.x) <= 1.0 &&
      std::abs(message->linear.y) <= 1.0 &&
      std::abs(message->angular.z) <= 2.0;
      if (!valid) {
        RCLCPP_WARN(get_logger(), "Rejected non-finite or out-of-range cmd_vel");
        return;
      }
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      velocity_command_ = *message;
      velocity_received_at_ = std::chrono::steady_clock::now();
      velocity_received_ = true;
    });

  height_service_ = create_service<mymit_robot::srv::SetStandingHeight>(
    "set_standing_height",
    [this](
      const std::shared_ptr<mymit_robot::srv::SetStandingHeight::Request> request,
      std::shared_ptr<mymit_robot::srv::SetStandingHeight::Response> response) {
      response->target_height = request->height;
      if (!std::isfinite(request->height) || request->height < 0.18F ||
      request->height > 0.34F)
      {
        response->success = false;
        response->message = "standing height must be within [0.18, 0.34] m";
        return;
      }
      requested_height_.store(request->height);
      height_sequence_.fetch_add(1);
      response->success = true;
      response->message = "standing height command accepted";
    });

  mode_service_ = create_service<mymit_robot::srv::SetControlMode>(
    "set_control_mode",
    [this](
      const std::shared_ptr<mymit_robot::srv::SetControlMode::Request> request,
      std::shared_ptr<mymit_robot::srv::SetControlMode::Response> response) {
      response->current_mode = current_mode_.load();
      if (!validMode(request->mode)) {
        response->success = false;
        response->message = "unknown control mode";
        return;
      }
      requested_mode_.store(static_cast<int>(request->mode));
      response->success = true;
      response->message = request->force ?
      "mode command queued; force does not bypass controller safety checks" :
      "mode command queued";
    });

  setCurrentMode(ControlMode::BalanceStand);
  RCLCPP_INFO(
    get_logger(),
    "ROS 2 control API ready (cmd_vel, state_estimate, joint_states, odom, TF, services)");
}

bool RobotRosNode::validMode(std::uint8_t mode) noexcept
{
  return mode <= static_cast<std::uint8_t>(ControlMode::FrontJump);
}

bool RobotRosNode::takeStandingHeight(float & height) noexcept
{
  const std::uint64_t sequence = height_sequence_.load();
  if (sequence == consumed_height_sequence_) {return false;}
  consumed_height_sequence_ = sequence;
  height = requested_height_.load();
  return true;
}

std::optional<ControlMode> RobotRosNode::takeControlModeRequest() noexcept
{
  const int requested = requested_mode_.exchange(-1);
  if (requested < 0) {return std::nullopt;}
  return static_cast<ControlMode>(requested);
}

RosVelocityCommand RobotRosNode::velocityCommand() const
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  RosVelocityCommand result;
  result.received = velocity_received_;
  if (!velocity_received_) {return result;}
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - velocity_received_at_).count();
  result.active = age <= command_timeout_;
  if (result.active) {
    result.forward = static_cast<float>(velocity_command_.linear.x);
    result.lateral = static_cast<float>(velocity_command_.linear.y);
    result.yaw_rate = static_cast<float>(velocity_command_.angular.z);
  }
  return result;
}

void RobotRosNode::setCurrentMode(ControlMode mode)
{
  const auto value = static_cast<std::uint8_t>(mode);
  const auto previous = current_mode_.exchange(value);
  if (previous == value) {return;}
  std_msgs::msg::UInt8 message;
  message.data = value;
  mode_publisher_->publish(message);
}

builtin_interfaces::msg::Time RobotRosNode::simulationStamp(double seconds)
{
  builtin_interfaces::msg::Time result;
  if (!std::isfinite(seconds) || seconds < 0.0) {return result;}
  const auto nanoseconds = static_cast<std::int64_t>(seconds * 1.0e9);
  result.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return result;
}

void RobotRosNode::publishClock(double simulation_time)
{
  rosgraph_msgs::msg::Clock message;
  message.clock = simulationStamp(simulation_time);
  clock_publisher_->publish(message);
}

bool RobotRosNode::shouldPublish(double simulation_time) noexcept
{
  if (last_publish_time_ < 0.0 || simulation_time < last_publish_time_ ||
    simulation_time - last_publish_time_ + 1.0e-12 >= publish_period_)
  {
    last_publish_time_ = simulation_time;
    return true;
  }
  return false;
}

void RobotRosNode::publishState(
  double simulation_time, const StateEstimate<float> & estimate,
  const std::array<double, kJointCount> & joint_position,
  const std::array<double, kJointCount> & joint_velocity,
  const std::array<double, kJointCount> & joint_effort,
  const RosContactSnapshot & contacts,
  const SimulationDiagnosticReport & diagnostics, bool control_valid)
{
  if (!shouldPublish(simulation_time)) {return;}
  const auto stamp = simulationStamp(simulation_time);
  const auto & q = estimate.orientation_world_from_body;
  const Vec3<float> angular_velocity_world =
    estimate.rotation_world_from_body * estimate.angular_velocity_body;

  mymit_robot::msg::StateEstimate state;
  state.header.stamp = stamp;
  state.header.frame_id = odom_frame_;
  state.body_pose.position.x = estimate.position_world.x();
  state.body_pose.position.y = estimate.position_world.y();
  state.body_pose.position.z = estimate.position_world.z();
  state.body_pose.orientation.w = q.w();
  state.body_pose.orientation.x = q.x();
  state.body_pose.orientation.y = q.y();
  state.body_pose.orientation.z = q.z();
  state.body_twist.linear.x = estimate.velocity_world.x();
  state.body_twist.linear.y = estimate.velocity_world.y();
  state.body_twist.linear.z = estimate.velocity_world.z();
  state.body_twist.angular.x = angular_velocity_world.x();
  state.body_twist.angular.y = angular_velocity_world.y();
  state.body_twist.angular.z = angular_velocity_world.z();
  state.body_angular_velocity.x = estimate.angular_velocity_body.x();
  state.body_angular_velocity.y = estimate.angular_velocity_body.y();
  state.body_angular_velocity.z = estimate.angular_velocity_body.z();
  state.valid = estimate.valid;
  state_publisher_->publish(state);

  sensor_msgs::msg::JointState joints;
  joints.header.stamp = stamp;
  joints.name.assign(kJointNames.begin(), kJointNames.end());
  joints.position.assign(joint_position.begin(), joint_position.end());
  joints.velocity.assign(joint_velocity.begin(), joint_velocity.end());
  joints.effort.assign(joint_effort.begin(), joint_effort.end());
  joint_state_publisher_->publish(joints);

  mymit_robot::msg::FootContact contact_message;
  contact_message.header.stamp = stamp;
  contact_message.header.frame_id = base_frame_;
  contact_message.contact = contacts.contact;
  contact_message.contact_probability = contacts.probability;
  contact_message.normal_force = contacts.normal_force;
  contact_message.valid = contacts.valid;
  contact_publisher_->publish(contact_message);

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = base_frame_;
  odometry.pose.pose = state.body_pose;
  const Vec3<float> velocity_body =
    estimate.rotation_world_from_body.transpose() * estimate.velocity_world;
  odometry.twist.twist.linear.x = velocity_body.x();
  odometry.twist.twist.linear.y = velocity_body.y();
  odometry.twist.twist.linear.z = velocity_body.z();
  odometry.twist.twist.angular.x = estimate.angular_velocity_body.x();
  odometry.twist.twist.angular.y = estimate.angular_velocity_body.y();
  odometry.twist.twist.angular.z = estimate.angular_velocity_body.z();
  odometry_publisher_->publish(odometry);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odometry.header;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = estimate.position_world.x();
  transform.transform.translation.y = estimate.position_world.y();
  transform.transform.translation.z = estimate.position_world.z();
  transform.transform.rotation = state.body_pose.orientation;
  transform_broadcaster_->sendTransform(transform);

  diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
  diagnostic_array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string(": control_pipeline");
  status.hardware_id = "mujoco_go1";
  status.level = control_valid ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = control_valid ? "control output valid" : "control output rejected";
  status.values.push_back(
    diagnosticValue(
      "position_rms_m", std::to_string(
        rootMeanSquare(
          diagnostics.squared_position_error_sum, diagnostics.observed_frames))));
  status.values.push_back(
    diagnosticValue(
      "velocity_rms_mps", std::to_string(
        rootMeanSquare(
          diagnostics.squared_velocity_error_sum, diagnostics.observed_frames))));
  status.values.push_back(
    diagnosticValue(
      "maximum_pitch_rad", std::to_string(diagnostics.maximum_absolute_pitch)));
  status.values.push_back(
    diagnosticValue(
      "minimum_height_m", std::to_string(diagnostics.minimum_height)));
  status.values.push_back(
    diagnosticValue(
      "rejected_frames", std::to_string(diagnostics.rejected_control_frames)));
  diagnostic_array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(diagnostic_array);
}
