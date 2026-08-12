// 行走状态的数据流：状态估计 -> MPC 接触力规划 -> WBC 关节力矩。
#include "FSM/FSM_State_Locomotion.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "model/floating_base_model_factory.hpp"
#include "Utilities/orientation_tools.h"

template<typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(
  ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(control_fsm_data, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  //检查控制系统数据是否有效。
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("locomotion state requires valid FSM data");
  }
  // MPC 的 10 段 TROT 必须与 GaitScheduler 的 0.5 s 周期一致，因此每段为
  // 50 ms   参数round(T(0.05)。两套接触时序若周期不同，状态估计器会逐渐把摆动脚误当支撑脚。
  //这里计算 MPC 的时间间隔。
  const std::size_t mpc_interval = static_cast<std::size_t>(std::max(
      T(1), std::round(T(0.05) / control_fsm_data->control_time_step)));
  //创建 MPC 控制器。传入:机器人模型；控制周期；MPC 步态时间间隔。  
  mpc_ = std::make_unique<mpc::ConvexMPCLocomotion<T>>(
    *control_fsm_data->quadruped, control_fsm_data->control_time_step,
    mpc_interval);
  //创建 WBC。buildFloatingBaseModel() 会建立一个带浮动基座的机器人动力学模型。
  //浮动基座表示机身不是固定在地面上，而是有：3 个平移自由度 + 3 个旋转自由度
  wbc_ctrl_ = std::make_unique<LocomotionCtrl<T>>(
    model::makeFloatingBaseModel(*control_fsm_data->quadruped));
  // 默认姿态环 Kp=50、Kd=1 在对角支撑切换时阻尼不足，前进地面力作用于
  // 质心下方后产生的俯仰无法及时衰减。提高 roll/pitch 恢复力和角速度阻尼，
  // yaw 保持较温和，避免改变航向响应。
  wbc_ctrl_->setBodyOrientationGains(
    Vec3<T>(T(100), T(100), T(50)),
    Vec3<T>(T(10), T(10), T(3)));
  // 行走关节 PD 按电机职责分别设置：Hip 保持Kp=30，避免四腿向机身
  // 内侧快速收缩；thigh/calf使用Kp=42跟随高速摆动轨迹。
  // Kd=[4,4.5,4.5]提供适量阻尼；继续加硬会让落脚过冲并造成小腿擦地。
  wbc_ctrl_->setJointGains(
    Vec3<T>(T(30), T(42), T(42)),
    Vec3<T>(T(4), T(4.5), T(4.5)));
  //安全检查
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
}

/*仅在第一次启动，后续不会启动*/
template<typename T>
void FSM_State_Locomotion<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  /*初始化 MPC 内部状态。包括：MPC 迭代器；步态相位；
    目标位置；约束和缓存。*/
  mpc_->initialize();
  resetSwingTrajectories();
  mpc_->setGait(GaitType::TROT);
  /*同时通知步态调度器使用 TROT。这里必须同步两个系统：
    MPC 的接触预测；GaitScheduler 的实际接触逻辑。
    如果两者不一致，就可能出现：MPC 认为后腿支撑；
    GaitScheduler 认为后腿摆动；WBC 约束错误；足端轨迹不断漂移或打滑。*/
  // 当前行走使用 60% 支撑率的 TROT_WALK：0.5 s 周期内支撑 0.30 s、
  // 摆动 0.20 s；与 MPC 接触表保持一致，避免状态估计接触约束错相。
  this->_data->gait_scheduler->requestGait(GaitType::TROT_WALK);
}

/*每个控制周期直接执行一次完整行走控制。*/
template<typename T>
void FSM_State_Locomotion<T>::run()
{
  LocomotionControlStep();
}
/*将用户要求的前进速度传给 MPC。
  MPC 会利用该速度计算足端落点。*/
template<typename T>
void FSM_State_Locomotion<T>::setForwardVelocity(T velocity)
{
  mpc_->setForwardVelocity(velocity);
}

template<typename T>
void FSM_State_Locomotion<T>::setVelocityCommand(
  T forward_velocity, T lateral_velocity, T yaw_rate)
{
  mpc_->setVelocityCommand(forward_velocity, lateral_velocity, yaw_rate);
}

/*checkTransition()每次检查状态切换时，迭代计数加一。*/
template<typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition()
{
  ++iteration_;
  if (!locomotionSafe()) {
    this->nextStateName = FSM_StateName::BALANCE_STAND;
    this->transitionData.done = false;
    this->transitionDuration = T(0);
    return this->nextStateName;
  }

  switch (this->_data->desired_state->mode) {
    case ControlMode::Locomotion:
      break;
    case ControlMode::BalanceStand:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      this->transitionDuration = T(0);
      break;
    case ControlMode::Passive:
      this->nextStateName = FSM_StateName::PASSIVE;
      this->transitionDuration = T(0);
      break;
    case ControlMode::JointPd:
      this->nextStateName = FSM_StateName::JOINT_PD;
      this->transitionDuration = T(0);
      break;
    case ControlMode::StandUp:
      this->nextStateName = FSM_StateName::STAND_UP;
      this->transitionDuration = T(0);
      break;
    case ControlMode::RecoveryStand:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = T(0);
      break;
  }
  return this->nextStateName;
}
/*执行状态切换处理 如果要切换到 BalanceStand，再执行一次控制周期。
  作用是：在切换瞬间仍然输出有效关节命令；避免切换帧出现零命令；
  让站立状态接收到连续的姿态和足端状态。*/
template<typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition()
{
  if (this->nextStateName == FSM_StateName::BALANCE_STAND) {
    LocomotionControlStep();
  }
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->turnOffAllSafetyChecks();
  }
  this->transitionData.done = true;
  return this->transitionData;
}

/*  检查机器人是否适合继续走。读取状态估计器结果：
    机身位置；机身速度；姿态；角速度。*/
template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() const
{
  // 行走安全条件比普通 FSM 姿态检查更严格，并同时限制足端位置与速度。
  const StateEstimate<T> & estimate = *this->_data->state_estimate;
  /* 参数四： 俯仰角 翻滚的安全上限了*/
  constexpr T max_roll_degrees = T(40);
  constexpr T max_pitch_degrees = T(40);
  if (!estimate.valid ||
    std::abs(estimate.rpy.x()) > ori::deg2rad(max_roll_degrees) ||
    std::abs(estimate.rpy.y()) > ori::deg2rad(max_pitch_degrees))
  {
    return false;
  }
  
  /*abs(data.p[1]) > 0.18 足端横向偏移不能超过 18 cm。
    data.v.norm() > 9  足端速度不能超过 9 m/s。如果超过，通常表示：*/
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & data = this->_data->leg_controller->datas[leg];
    if (!data.valid || data.p.z() > T(0) || std::abs(data.p.y()) > T(0.18) ||
      data.v.norm() > T(9))
    {
      return false;
    }
  }
  return true;
}

template<typename T>
std::array<Vec3<T>, kNumLegs>
FSM_State_Locomotion<T>::footPositionsWorld() const
{
  std::array<Vec3<T>, kNumLegs> positions{};
  /*读取机身世界位置和姿态。*/
  const auto & estimate = *this->_data->state_estimate;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    /*遍历四条腿，并将整数转换成 LegId 枚举。*/
    const LegId leg_id = static_cast<LegId>(leg);
    // 腿传感器给出足端相对髋的位置：先平移到机身系，再旋转和平移到世界系。
    const Vec3<T> foot_body = this->_data->quadruped->hipLocation(leg_id) +
     this->_data->leg_controller->datas[leg].p;
     /*足端相对机身位置=髋关节相对机身位置+足端相对髋关节位置*/
    positions[leg] = estimate.position_world +
      estimate.rotation_world_from_body * foot_body;
  }
  return positions;
}

/* 将四条腿全部标记为没有开始摆动。清除每条腿轨迹内部的：
   起点；终点；高度；时间；相位。*/
template<typename T>
void FSM_State_Locomotion<T>::resetSwingTrajectories() noexcept
{
  swing_active_.fill(false);
  for (auto & trajectory : swing_trajectories_) {
    trajectory.reset();
  }
}

template<typename T>
void FSM_State_Locomotion<T>::startSwingTrajectory(
  std::size_t leg, const Vec3<T> & initial_position,
  const mpc::LocomotionResult<T> & locomotion_result)
{
  /*先清除旧轨迹，再锁存当前足端位置作为起点。关键点是：
    initial_position只在离地瞬间记录一次。
    不能每帧用新的实测足端位置覆盖它，否则起点会不断移动。*/
  auto & trajectory = swing_trajectories_.at(leg);
  trajectory.reset();
  trajectory.setInitialPosition(initial_position);

  // 落脚点必须相对当前机身/髋部规划，不能在上一次世界足点上反复累加步长；
  // 否则机身稍有打滑，脚就会越走越远并最终把腿拉直。
  const auto & estimate = *this->_data->state_estimate;
  const LegId leg_id = static_cast<LegId>(leg);
  const T yaw = locomotion_result.command.body_rpy.z();
  Eigen::Matrix<T, 2, 2> yaw_rotation;
  yaw_rotation << std::cos(yaw), -std::sin(yaw),
    std::sin(yaw), std::cos(yaw);
  // GO1 的 Hip 安装点并不是名义落脚点。q_abad=0 时，8 cm 的 Hip 横向
  // 连杆仍会把左右足分别向机身外侧推出；若只使用 hipLocation，落脚宽度会
  // 从约 25.35 cm 缩到 9.35 cm，四条腿便会在每次摆动时明显向内收。
  Vec3<T> nominal_foot_from_hip = Vec3<T>::Zero();
  const auto & leg_model = this->_data->quadruped->leg(leg_id);
  computeLegJacobianAndPosition(
    *this->_data->quadruped, leg_model.joints.home_position,
    static_cast<Mat3<T> *>(nullptr), &nominal_foot_from_hip, leg_id);
  const Vec2<T> nominal_foot_body =
    (this->_data->quadruped->hipLocation(leg_id) + nominal_foot_from_hip)
    .template head<2>();
  const Vec2<T> nominal_foot_world =
    estimate.position_world.template head<2>() + yaw_rotation * nominal_foot_body;
  const Vec2<T> desired_velocity =
    locomotion_result.command.body_velocity_world.template head<2>();
  const Vec2<T> measured_velocity =
    estimate.velocity_world.template head<2>();
  const Vec2<T> velocity_error =
    measured_velocity - desired_velocity;
  // 从离地到下一次着地，机身还会在整个摆动期内继续前进；着地后再以前后
  // 对称方式跨过半个支撑期。漏掉 swing_time 会让每一步少前移 v*T_swing，
  // 后腿因而逐周期落后于机身并最终拉直。
  const T placement_time = locomotion_result.swing_time[leg] +
    T(0.5) * locomotion_result.stance_time[leg];
  Vec2<T> step = placement_time * measured_velocity + T(0.08) * velocity_error;
  if (step.norm() > maximum_step_length_) {
    step *= maximum_step_length_ / step.norm();
  }
  Vec3<T> landing_position = initial_position;
  // 前进/横移补偿叠加在名义支撑点上，因此速度控制不会侵蚀支撑宽度。
  landing_position.template head<2>() = nominal_foot_world + step;
  // z 始终使用离地瞬间锁存的地面高度，不能跟随摆动中的实测足高漂移。
  landing_position.z() = initial_position.z();
  trajectory.setFinalPosition(landing_position);
  trajectory.setHeight(swing_height_);
  swing_active_[leg] = true;
}

template<typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep()
{
  // MPC 输入必须使用统一的世界坐标系，否则反作用力方向会与 WBC 不一致。
  const auto feet_world = footPositionsWorld();
  /*MPC 输入：当前状态估计；用户期望状态；当前四个足端的世界坐标。
    MPC 输出 LocomotionResult，包括：command:机身期望状态；
    reaction_forces_world：各腿期望地面反作用力；
    contact_state：当前是否接触；,swing_phase：摆动进度，通常范围为 [0, 1]
    ,swing_time：摆动总时间,stance_time,valid：支撑总时间。*/
  const auto result = mpc_->run(
    *this->_data->state_estimate, *this->_data->desired_state, feet_world);
  if (!result.valid) {
    this->_data->leg_controller->zeroCommand();
    this->_data->leg_controller->setEnabled(false);
    return;
  }
  //设置 WBC 身体目标
  const auto & desired = result.command;
  wbc_data_.pBody_des = desired.body_position_world;
  wbc_data_.vBody_des = desired.body_velocity_world;
  wbc_data_.aBody_des = desired.body_acceleration_world;
  wbc_data_.pBody_RPY_des = desired.body_rpy;
  wbc_data_.vBody_Ori_des = desired.body_angular_velocity;
  //逐腿处理支撑或摆动逻辑。
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    //将 MPC 计算出的世界坐标系反作用力传给 WBC。
    wbc_data_.Fr_des[leg] = result.reaction_forces_world[leg];
    //将布尔接触状态转成浮点数：
    wbc_data_.contact_state[leg] = result.contact_state[leg] ? T(1) : T(0);

    if (result.contact_state[leg]) {
      // 支撑时足端位置任务不会加入 WBC；同步当前值仅用于诊断，并清除上一摆动态。
      swing_active_[leg] = false;
      wbc_data_.pFoot_des[leg] = feet_world[leg];
      wbc_data_.vFoot_des[leg].setZero();
      wbc_data_.aFoot_des[leg].setZero();
      continue;
    }

    // 只在离地沿锁存起点和落脚点。后续周期即使实测足端受到扰动，目标轨迹
    // 也只由锁存数据和 swing_phase 推进，不会把扰动累积成新的目标高度。
    if (!swing_active_[leg]) {
      startSwingTrajectory(leg, feet_world[leg], result);
    }
    auto & trajectory = swing_trajectories_[leg];
    /*Bezier 轨迹会生成平滑的：足端位置；足端速度；足端加速度。*/
    trajectory.computeSwingTrajectoryBezier(
      result.swing_phase[leg], result.swing_time[leg]);
    wbc_data_.pFoot_des[leg] = trajectory.getPosition();
    wbc_data_.vFoot_des[leg] = trajectory.getVelocity();
    wbc_data_.aFoot_des[leg] = trajectory.getAcceleration();
  }

  if (this->_data->use_wbc) {
    // WBC和最终关节PD均保持500 Hz。测试确认WBC降频会使支撑力和足端约束
    // 滞后并造成小腿擦地，因此计算削减只放在20 Hz的MPC内部。
    const bool wbc_valid = wbc_ctrl_->runAndApply(
      &wbc_data_, *this->_data->state_estimate, *this->_data->joint_states,
      *this->_data->leg_controller);
    if (wbc_valid) {
      // 直接提高摆动腿电机期望关节转速；支撑腿保持KinWBC原始速度。
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        if (result.contact_state[leg]) {continue;}
        auto & velocity = this->_data->leg_controller->commands[leg].velocity_desired;
        velocity = velocity.cwiseProduct(swing_joint_velocity_scale_);
        const auto leg_id = static_cast<LegId>(leg);
        const Vec3<T> & limit = this->_data->quadruped->leg(leg_id).joints.velocity_limit;
        velocity = velocity.cwiseMin(limit).cwiseMax(-limit);
      }
    }
  } else {
    // 关闭 WBC 时的降级路径只发送足端前馈力，主要用于调试算法分层。
    auto & controller = *this->_data->leg_controller;
    controller.zeroCommand();
    controller.setEnabled(true);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      controller.commands[leg].force_feedforward =
        this->_data->state_estimate->rotation_world_from_body.transpose() *
        result.reaction_forces_world[leg];
    }
  }
}

template<typename T>
void FSM_State_Locomotion<T>::onExit()
{
  iteration_ = 0;
  resetSwingTrajectories();
}

template class FSM_State_Locomotion<float>;
