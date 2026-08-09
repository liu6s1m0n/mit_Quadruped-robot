#include "RobotRunner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
constexpr std::array<LegId, kNumLegs> kLegOrder{
  LegId::FR, LegId::FL, LegId::RR, LegId::RL};
}

RobotRunner::LegSensorOwners RobotRunner::makeLegSensors(
  const mjModel * model, const mjData * data)
{
  LegSensorOwners sensors;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    sensors[leg] = makeLeg(LegSource::SIMULATOR, kLegOrder[leg], model, data);
  }
  return sensors;
}

RobotRunner::LegSensorPointers RobotRunner::sensorPointers(
  const LegSensorOwners & sensors)
{
  LegSensorPointers pointers{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    pointers[leg] = sensors[leg].get();
  }
  return pointers;
}

RobotRunner::RobotRunner(const mjModel * model, const mjData * data)
: model_(model), data_(data),
  quadruped_(makeQuadruped<float>(RobotType::UNITREE_GO1)),
  leg_controller_(quadruped_),
  imu_(makeImu(ImuSource::SIMULATOR, model, data)),
  leg_sensors_(makeLegSensors(model, data)),
  leg_sensor_pointers_(sensorPointers(leg_sensors_)),
  gait_scheduler_(static_cast<float>(model == nullptr ? 0.0 : model->opt.timestep))
{
  if (model_ == nullptr || data_ == nullptr) {
    throw std::invalid_argument("RobotRunner requires a MuJoCo model and data");
  }

  PositionVelocityEstimatorParameters<float> estimator_parameters;
  estimator_parameters.nominal_time_step = static_cast<float>(model_->opt.timestep);
  estimator_parameters.maximum_time_step = std::max(
    estimator_parameters.maximum_time_step,
    estimator_parameters.nominal_time_step);
  state_estimator_ = std::make_unique<PositionVelocityEstimator<float>>(
    quadruped_, *imu_, leg_sensor_pointers_,
    OrientationEstimatorMode::SIMULATION_TRUTH, estimator_parameters);

  desired_state_.mode = ControlMode::BalanceStand;
  desired_state_.body_position_world.z() = quadruped_.nominalBodyHeight();
  desired_state_.valid = true;
  standing_height_target_ = quadruped_.nominalBodyHeight();
  standing_height_command_ = standing_height_target_;
  control_fsm_ = std::make_unique<ControlFSM<float>>(
    quadruped_, state_estimate_, joint_states_, leg_controller_, gait_scheduler_,
    desired_state_, static_cast<float>(model_->opt.timestep));
  control_fsm_->setUseWbc(true);
  disableCommands();
}

void RobotRunner::setControlMode(ControlMode mode) noexcept
{
  desired_state_.mode = mode;
}

void RobotRunner::setStandingHeight(float height)
{
  if (!std::isfinite(height) || height < minimumStandingHeight() ||
    height > maximumStandingHeight())
  {
    throw std::invalid_argument("standing height must be within [0.18, 0.34] m");
  }
  standing_height_target_ = height;
}

void RobotRunner::reset()
{
  state_estimator_->reset();
  gait_scheduler_.initialize();
  state_estimate_ = StateEstimate<float>{};
  joint_states_ = std::array<JointState<float>, kNumLegs>{};
  desired_state_ = DesiredState<float>{};
  desired_state_.mode = ControlMode::BalanceStand;
  desired_state_.body_position_world.z() = quadruped_.nominalBodyHeight();
  desired_state_.valid = true;
  joint_initialization_started_ = false;
  joint_initialization_start_time_ = 0.0F;
  standing_height_command_initialized_ = false;
  desired_state_initialized_ = false;
  control_fsm_->initialize();
  disableCommands();
}

void RobotRunner::updateStandingHeightCommand()
{
  if (!standing_height_command_initialized_) {
    standing_height_command_ = std::clamp(
      state_estimate_.position_world.z(), minimumStandingHeight(),
      maximumStandingHeight());
    standing_height_command_initialized_ = true;
  }

  const float maximum_step =
    standing_height_rate_limit_ * static_cast<float>(model_->opt.timestep);
  const float error = standing_height_target_ - standing_height_command_;
  standing_height_command_ += std::clamp(error, -maximum_step, maximum_step);
  desired_state_.body_position_world.z() = standing_height_command_;
}

bool RobotRunner::jointInitializationComplete() const noexcept
{
  return joint_initialization_started_ &&
         static_cast<float>(data_->time) - joint_initialization_start_time_ >=
         joint_initialization_duration_;
}

void RobotRunner::prepareJointInitialization()
{
  if (!joint_initialization_started_) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      initial_joint_positions_[leg] = leg_controller_.datas[leg].q;
    }
    joint_initialization_start_time_ = static_cast<float>(data_->time);
    joint_initialization_started_ = true;
  }

  const float elapsed =
    static_cast<float>(data_->time) - joint_initialization_start_time_;
  const float phase = std::clamp(
    elapsed / joint_initialization_duration_, 0.0F, 1.0F);
  const float blend = phase * phase * (3.0F - 2.0F * phase);
  const float blend_rate =
    6.0F * phase * (1.0F - phase) / joint_initialization_duration_;

  leg_controller_.zeroCommand();
  leg_controller_.setEnabled(true);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const Vec3<float> target = quadruped_.leg(kLegOrder[leg]).joints.home_position;
    const Vec3<float> travel = target - initial_joint_positions_[leg];
    auto & command = leg_controller_.commands[leg];
    command.position_desired = initial_joint_positions_[leg] + blend * travel;
    command.velocity_desired = blend_rate * travel;
    command.kp_joint.setConstant(60.0F);
    command.kd_joint.setConstant(3.0F);
  }
}

bool RobotRunner::collectJointCommands()
{
  bool commands_valid = true;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    joint_commands_[leg] = leg_controller_.command(
      kLegOrder[leg], static_cast<float>(data_->time));
    commands_valid = joint_commands_[leg].enabled && commands_valid;
  }
  if (!commands_valid) {disableCommands();}
  return commands_valid;
}

void RobotRunner::setDesiredState(const DesiredState<float> & desired)
{
  if (!desired.body_position_world.allFinite() ||
    !desired.body_velocity_world.allFinite() ||
    !desired.body_acceleration_world.allFinite() || !desired.body_rpy.allFinite() ||
    !desired.body_angular_velocity.allFinite())
  {
    throw std::invalid_argument("desired robot state contains a non-finite value");
  }
  if (desired.mode == ControlMode::BalanceStand) {
    setStandingHeight(desired.body_position_world.z());
  }
  desired_state_ = desired;
}

void RobotRunner::disableCommands() noexcept
{
  leg_controller_.zeroCommand();
  leg_controller_.setEnabled(false);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    joint_commands_[leg] = JointCommand<float>{};
    joint_commands_[leg].leg = kLegOrder[leg];
  }
}

bool RobotRunner::run()
{
  gait_scheduler_.step();
  state_estimator_->setContactProbabilities(
    gait_scheduler_.gait_data.scheduledContactProbabilities());
  if (!state_estimator_->run()) {
    disableCommands();
    return false;
  }

  state_estimate_ = state_estimator_->result();
  joint_states_ = state_estimator_->orientationEstimator().jointStates();
  bool feedback_valid = true;
  for (const auto & state : joint_states_) {
    feedback_valid = leg_controller_.updateData(state) && feedback_valid;
  }
  if (!feedback_valid) {
    disableCommands();
    return false;
  }

  if (!jointInitializationComplete()) {
    prepareJointInitialization();
    return collectJointCommands();
  }

  if (!desired_state_initialized_) {
    desired_state_.body_position_world = state_estimate_.position_world;
    desired_state_.body_rpy = state_estimate_.rpy;
    desired_state_.body_velocity_world.setZero();
    desired_state_.body_acceleration_world.setZero();
    desired_state_.body_angular_velocity.setZero();
    desired_state_.mode = ControlMode::BalanceStand;
    desired_state_.timestamp = state_estimate_.timestamp;
    desired_state_.valid = true;
    desired_state_initialized_ = true;
  }

  updateStandingHeightCommand();

  control_fsm_->runFSM();
  return collectJointCommands();
}
