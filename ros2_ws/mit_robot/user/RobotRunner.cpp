// 本文件按“估计 -> 安全初始化 -> 期望状态 -> FSM/WBC -> 关节命令”的顺序执行。
#include "RobotRunner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
constexpr std::array<LegId, kNumLegs> kLegOrder{
  LegId::FR, LegId::FL, LegId::RR, LegId::RL};
}

/*使用仿真器数据创建指定腿的传感器。
  这里传入：数据源：SIMULATOR；
  腿编号；MuJoCo 模型；MuJoCo 数据。
  以后如果切换真实硬件，只需要替换传感器创建方式，上层控制器不用修改。*/
RobotRunner::LegSensorOwners RobotRunner::makeLegSensors(
  const mjModel * model, const mjData * data)
{
  LegSensorOwners sensors;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    sensors[leg] = makeLeg(LegSource::SIMULATOR, kLegOrder[leg], model, data);
  }
  return sensors;
}

/*将裸指针数组传给状态估计器。
  传感器的生命周期仍然由 leg_sensors_ 管理。*/
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
  /*Unitree Go1 机器人模型；*/
  quadruped_(makeQuadruped<float>(RobotType::UNITREE_GO1)),
  /*腿部控制器*/
  leg_controller_(quadruped_),
  /*IMU 传感器*/
  imu_(makeImu(ImuSource::SIMULATOR, model, data)),
  /*四条腿传感器*/
  leg_sensors_(makeLegSensors(model, data)),
  /*腿部传感器指针*/
  leg_sensor_pointers_(sensorPointers(leg_sensors_)),
  /*步态调度器*/
  gait_scheduler_(static_cast<float>(model == nullptr ? 0.0 : model->opt.timestep))
{
  if (model_ == nullptr || data_ == nullptr) {
    throw std::invalid_argument("RobotRunner requires a MuJoCo model and data");
  }

  // 仿真时间步是所有滤波器和轨迹推进的共同时间基准。
  /*创建状态估计器参数*/
  PositionVelocityEstimatorParameters<float> estimator_parameters;
  /*把 MuJoCo 时间步作为状态估计器的标准时间步*/
  estimator_parameters.nominal_time_step = static_cast<float>(model_->opt.timestep);
  /*确保最大时间步不会小于实际仿真时间步。
    这可以防止：仿真步长比默认值大；估计器误判时间间隔；速度积分和滤波器参数异常。*/
  estimator_parameters.maximum_time_step = std::max(
    estimator_parameters.maximum_time_step,
    estimator_parameters.nominal_time_step);
  /*软件仿真模式：传感器数据由 MuJoCo 提供，姿态直接采用 MuJoCo
    输出的机身四元数真值，不经过真机使用的 IMU 融合链路。*/
  state_estimator_ = std::make_unique<PositionVelocityEstimator<float>>(
    quadruped_, *imu_, leg_sensor_pointers_,
    OrientationEstimatorMode::SIMULATION_TRUTH, estimator_parameters);

  // 默认进入平衡站立；真正输出命令前先经过 0.4 秒平滑关节初始化。
  desired_state_.mode = ControlMode::BalanceStand;
  /*默认机身高度设为机器人模型中的名义高度*/
  desired_state_.body_position_world.z() = quadruped_.nominalBodyHeight();
  /*标记期望状态有效*/
  desired_state_.valid = true;
  /*目标高度和当前命令高度都设为名义高度*/
  standing_height_target_ = quadruped_.nominalBodyHeight();
  standing_height_command_ = standing_height_target_;
  /*将所有共享对象传给总状态机。
    注意这些对象都是 RobotRunner 的成员：因此 RobotRunner 必须比 ControlFSM 活得更久*/
  control_fsm_ = std::make_unique<ControlFSM<float>>(
    quadruped_, state_estimate_, joint_states_, leg_controller_, gait_scheduler_,
    desired_state_, static_cast<float>(model_->opt.timestep));
  /*强制启用 WBC。*/
  control_fsm_->setUseWbc(true);
  setWalkingForwardSpeed(defaultWalkingForwardSpeed());
  /*初始化完成后，清零并关闭腿部命令。
    即使 FSM 默认是 BalanceStand，也不会在第一次传感器更新前直接输出力矩。*/
  disableCommands();
}

/*设置目标控制模式*/
void RobotRunner::setControlMode(ControlMode mode) noexcept
{
  if (desired_state_.mode == mode) {return;}
  if (state_estimate_.valid) {
    // 切换站立/行走时只继承不可观测的水平原点和航向。roll/pitch 必须保持
    // 水平目标；若把切换瞬间的倾斜锁存下来，WBC 会主动维持后仰姿态。
    desired_state_.body_position_world.x() = state_estimate_.position_world.x();
    desired_state_.body_position_world.y() = state_estimate_.position_world.y();
    desired_state_.body_rpy << 0.0F, 0.0F, state_estimate_.rpy.z();
  }
  desired_state_.body_velocity_world.setZero();
  desired_state_.body_acceleration_world.setZero();
  desired_state_.body_angular_velocity.setZero();
  desired_state_.mode = mode;
}

ControlMode RobotRunner::currentControlMode() const noexcept
{
  switch (control_fsm_->currentStateName()) {
    case FSM_StateName::PASSIVE: return ControlMode::Passive;
    case FSM_StateName::JOINT_PD: return ControlMode::JointPd;
    case FSM_StateName::BALANCE_STAND: return ControlMode::BalanceStand;
    case FSM_StateName::LOCOMOTION: return ControlMode::Locomotion;
    case FSM_StateName::STAND_UP: return ControlMode::StandUp;
    case FSM_StateName::RECOVERY_STAND: return ControlMode::RecoveryStand;
    case FSM_StateName::FRONT_JUMP: return ControlMode::FrontJump;
    case FSM_StateName::INVALID: return desired_state_.mode;
  }
  return desired_state_.mode;
}

bool RobotRunner::requestFrontJump() noexcept
{
  constexpr float maximum_tilt = 0.15F;
  constexpr float maximum_linear_speed = 0.12F;
  constexpr float maximum_angular_speed = 0.35F;
  if (!jointInitializationComplete() || !state_estimate_.valid ||
    control_fsm_->currentStateName() != FSM_StateName::BALANCE_STAND ||
    std::abs(state_estimate_.rpy.x()) > maximum_tilt ||
    std::abs(state_estimate_.rpy.y()) > maximum_tilt ||
    state_estimate_.velocity_world.norm() > maximum_linear_speed ||
    state_estimate_.angular_velocity_body.norm() > maximum_angular_speed)
  {
    return false;
  }
  desired_state_.body_velocity_world.setZero();
  desired_state_.body_acceleration_world.setZero();
  desired_state_.body_angular_velocity.setZero();
  desired_state_.mode = ControlMode::FrontJump;
  return true;
}

/*把速度传递给 FSM 内部的 Locomotion*/
void RobotRunner::setWalkingForwardSpeed(float speed)
{
  control_fsm_->setLocomotionForwardVelocity(speed);
}

void RobotRunner::setLocomotionVelocityCommand(
  float forward_velocity, float lateral_velocity, float yaw_rate)
{
  control_fsm_->setLocomotionVelocityCommand(
    forward_velocity, lateral_velocity, yaw_rate);
}

/*设置站立高度*/
void RobotRunner::setStandingHeight(float height)
{
  /*不能是 NaN；不能是 Inf；
    不能低于 0.18 m；不能高于 0.34 m。*/
  if (!std::isfinite(height) || height < minimumStandingHeight() ||
    height > maximumStandingHeight())
  {
    throw std::invalid_argument("standing height must be within [0.18, 0.34] m");
  }
  standing_height_target_ = height;
}


void RobotRunner::reset()
{
  // 只清除算法内部状态。MuJoCo 已经处理了 qpos/qvel，本控制器不“扶正”机器人。
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
  // 首帧从当前实测高度起步，避免 Reset 后目标高度突跳。
  if (!standing_height_command_initialized_) {
    standing_height_command_ = std::clamp(
      state_estimate_.position_world.z(), minimumStandingHeight(),
      maximumStandingHeight());
    standing_height_command_initialized_ = true;
  }

  // 速率限制换算为“每个控制周期允许变化的最大高度”。
  // 如果时间步为 0.001 s，则每个控制周期最多改变0. × 0.001 = 0.00008 m：
  const float maximum_step =
    standing_height_rate_limit_ * static_cast<float>(model_->opt.timestep);
  //计算目标高度和当前高度之间的误差
  const float error = standing_height_target_ - standing_height_command_;
  standing_height_command_ += std::clamp(error, -maximum_step, maximum_step);
  //将平滑后的高度写回 FSM 使用的期望状态
  desired_state_.body_position_world.z() = standing_height_command_;
}

/*关节初始化完成必须满足：
  初始化已经开始；当前仿真时间减去开始时间大于等于 0.4 秒。
  如果还没有开始，即使时间足够长，也返回 false。*/
bool RobotRunner::jointInitializationComplete() const noexcept
{
  return joint_initialization_started_ &&
         static_cast<float>(data_->time) - joint_initialization_start_time_ >=
         joint_initialization_duration_;
}

/*这个函数负责启动时平滑进入默认姿态。*/
void RobotRunner::prepareJointInitialization()
{
  if (!joint_initialization_started_) {
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      //锁存四条腿当前实际关节角，作为轨迹起点
      initial_joint_positions_[leg] = leg_controller_.datas[leg].q;
    }
    //记录初始化起始时间
    joint_initialization_start_time_ = static_cast<float>(data_->time);
    joint_initialization_started_ = true;
  }
  //计算已经经过的时间。
  const float elapsed =
    static_cast<float>(data_->time) - joint_initialization_start_time_;
  //将准备时间化为[0, 1]区间的相位，超过 1.0 的部分会被 clamp 掉。
  const float phase = std::clamp(
    elapsed / joint_initialization_duration_, 0.0F, 1.0F);
  // 三次 smoothstep 的起止速度均为零，比线性插值更不容易产生冲击。
  /*它的特点是：起点速度为零；终点速度为零；比线性插值冲击更小。*/
  const float blend = phase * phase * (3.0F - 2.0F * phase);
  /*计算 smoothstep 的时间导数，作为期望关节速度。上一步的导数形式*/
  const float blend_rate =
    6.0F * phase * (1.0F - phase) / joint_initialization_duration_;
  /*先清除所有旧控制模式命令，再开启腿部输出。*/
  leg_controller_.zeroCommand();
  leg_controller_.setEnabled(true);

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    /*target,travel这两个是啥玩意*/
    const Vec3<float> target = quadruped_.leg(kLegOrder[leg]).joints.home_position;
    const Vec3<float> travel = target - initial_joint_positions_[leg];
    auto & command = leg_controller_.commands[leg];
    command.position_desired = initial_joint_positions_[leg] + blend * travel;
    command.velocity_desired = blend_rate * travel;
    // 初始化阶段恢复经过验证的 PD；行走提速只在 Locomotion 摆动腿生效，
    // 避免把更快的启动过程误认为电机行走速度提升。
    command.kp_joint.setConstant(60.0F);
    command.kd_joint.setConstant(3.0F);
  }
}

//从 LegController 生成四条腿最终输出命令。
bool RobotRunner::collectJointCommands()
{
  bool commands_valid = true;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    /*调用 LegController::command()：
      读取当前腿反馈；计算笛卡尔足端力；使用 JᵀF 转换为关节力矩；
      限制前馈力矩；生成 JointCommand。*/
    joint_commands_[leg] = leg_controller_.command(
      kLegOrder[leg], static_cast<float>(data_->time));
    commands_valid = joint_commands_[leg].enabled && commands_valid;
  }
  if (!commands_valid) {disableCommands();}
  return commands_valid;
}

/*设置完整的目标状态。*/
void RobotRunner::setDesiredState(const DesiredState<float> & desired)
{
  /*检查所有主要目标量是否有限*/
  if (!desired.body_position_world.allFinite() ||
    !desired.body_velocity_world.allFinite() ||
    !desired.body_acceleration_world.allFinite() || !desired.body_rpy.allFinite() ||
    !desired.body_angular_velocity.allFinite())
  {/*发现 NaN 或 Inf 时拒绝整个目标状态*/
    throw std::invalid_argument("desired robot state contains a non-finite value");
  }
  /*如果目标模式是 BalanceStand，则同步更新目标站立高度。
                     这会触发 0.18～0.34 m 范围检查*/
  if (desired.mode == ControlMode::BalanceStand) {
    setStandingHeight(desired.body_position_world.z());
  }
  desired_state_ = desired;
}

/*统一关闭全部腿部命令。*/
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
  // 1. 推进一步态并把“预计接触概率”交给状态估计器。
  //让步态调度器前进一步。它会更新：当前 gait phase；
  //每条腿的接触概率；摆动/支撑状态。
  gait_scheduler_.step();
  state_estimator_->setContactProbabilities(
    gait_scheduler_.gait_data.estimatorContactProbabilities());
  /*运行状态估计器。如果失败：*/
  if (!state_estimator_->run()) {
    disableCommands();
    return false;
  }

  // 2. 将估计状态同步给腿控制器；任一腿反馈无效就关闭全部输出。
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

  // 3. 刚启动或 Reset 后先把关节平滑带到名义姿态，再启用全身控制。
  if (!jointInitializationComplete()) {
    prepareJointInitialization();
    return collectJointCommands();
  }

  // 4. 首次进入闭环时从当前水平位置和航向建立参考。初始化期间产生的
  // roll/pitch 是需要消除的扰动，不能锁存成后续站立和行走的目标姿态。
  if (!desired_state_initialized_) {
    desired_state_.body_position_world = state_estimate_.position_world;
    desired_state_.body_rpy << 0.0F, 0.0F, state_estimate_.rpy.z();
    desired_state_.body_velocity_world.setZero();
    desired_state_.body_acceleration_world.setZero();
    desired_state_.body_angular_velocity.setZero();
    desired_state_.timestamp = state_estimate_.timestamp;
    desired_state_.valid = true;
    desired_state_initialized_ = true;
  }

  // 5. 更新高度目标，最后由 FSM 选择站立或行走控制器并生成命令。
  updateStandingHeightCommand();

  control_fsm_->runFSM();
  return collectJointCommands();
}
