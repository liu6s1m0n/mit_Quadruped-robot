#ifndef MYMIT_ROBOT_USER_ROBOT_RUNNER_HPP_
#define MYMIT_ROBOT_USER_ROBOT_RUNNER_HPP_

#include <array>
#include <memory>

#include <mujoco/mujoco.h>

#include "FSM/ControlFSM.h"
#include "controller/PositionVelocityEstimator.hpp"
#include "controller/leg_controller.hpp"
#include "model/quadruped.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

/** Owns and executes the complete estimator/FSM/MPC/WBC control pipeline. */
class RobotRunner
{
public:
  RobotRunner(const mjModel * model, const mjData * data);
  ~RobotRunner() = default;

  RobotRunner(const RobotRunner &) = delete;
  RobotRunner & operator=(const RobotRunner &) = delete;

  bool run();
  void reset();
  void setControlMode(ControlMode mode) noexcept;
  void setStandingHeight(float height);
  void setDesiredState(const DesiredState<float> & desired);

  static constexpr float minimumStandingHeight() noexcept {return 0.18F;}
  static constexpr float maximumStandingHeight() noexcept {return 0.34F;}
  float standingHeightTarget() const noexcept {return standing_height_target_;}

  const std::array<JointCommand<float>, kNumLegs> & jointCommands() const noexcept
  {
    return joint_commands_;
  }
  const StateEstimate<float> & stateEstimate() const noexcept {return state_estimate_;}
  const Quadruped<float> & quadruped() const noexcept {return quadruped_;}

private:
  using LegSensorOwners = std::array<std::unique_ptr<LegSensor>, kNumLegs>;
  using LegSensorPointers = std::array<LegSensor *, kNumLegs>;

  static LegSensorOwners makeLegSensors(const mjModel * model, const mjData * data);
  static LegSensorPointers sensorPointers(const LegSensorOwners & sensors);
  void prepareJointInitialization();
  void updateStandingHeightCommand();
  bool jointInitializationComplete() const noexcept;
  bool collectJointCommands();
  void disableCommands() noexcept;

  const mjModel * model_ = nullptr;
  const mjData * data_ = nullptr;
  Quadruped<float> quadruped_;
  LegController<float> leg_controller_;
  std::unique_ptr<ImuSensor> imu_;
  LegSensorOwners leg_sensors_;
  LegSensorPointers leg_sensor_pointers_{};
  std::unique_ptr<PositionVelocityEstimator<float>> state_estimator_;
  GaitScheduler<float> gait_scheduler_;
  StateEstimate<float> state_estimate_;
  std::array<JointState<float>, kNumLegs> joint_states_{};
  DesiredState<float> desired_state_;
  std::unique_ptr<ControlFSM<float>> control_fsm_;
  std::array<JointCommand<float>, kNumLegs> joint_commands_{};
  std::array<Vec3<float>, kNumLegs> initial_joint_positions_{};
  float joint_initialization_start_time_ = 0.0F;
  float joint_initialization_duration_ = 0.4F;
  float standing_height_target_ = 0.27F;
  float standing_height_command_ = 0.27F;
  float standing_height_rate_limit_ = 0.08F;
  bool joint_initialization_started_ = false;
  bool standing_height_command_initialized_ = false;
  bool desired_state_initialized_ = false;
};

#endif  // MYMIT_ROBOT_USER_ROBOT_RUNNER_HPP_
