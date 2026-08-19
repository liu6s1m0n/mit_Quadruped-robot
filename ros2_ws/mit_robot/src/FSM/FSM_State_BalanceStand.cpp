// 平衡站立状态：四足作为接触约束，机身位置与姿态作为高优先级任务。
#include "FSM/FSM_State_BalanceStand.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "model/floating_base_model_factory.hpp"

template<typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(
  ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(
    control_fsm_data, FSM_StateName::BALANCE_STAND, "BALANCE_STAND")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("balance-stand state requires valid FSM data");
  }
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
  //创建WBC
  wbc_ctrl_ = std::make_unique<LocomotionCtrl<T>>(
    model::makeFloatingBaseModel(*control_fsm_data->quadruped));
  const auto & parameters = *control_fsm_data->control_parameters;
  if (control_fsm_data->quadruped->robotType() == RobotType::UNITREE_GO1) {
    // GO1 继续采用原来经过回归验证的初始化顺序，避免无意义地重写
    // WBC 默认参数后改变求解器的数值轨迹。
    wbc_ctrl_->setFloatingBaseWeight(T(1000));
  } else {
    wbc_ctrl_->setBodyPositionGains(
      parameters.balance_body_position_kp, parameters.balance_body_position_kd);
    wbc_ctrl_->setBodyOrientationGains(
      parameters.balance_body_orientation_kp,
      parameters.balance_body_orientation_kd);
    wbc_ctrl_->setJointGains(
      parameters.balance_joint_kp, parameters.balance_joint_kd);
    wbc_ctrl_->setFloatingBaseWeight(parameters.balance_floating_base_weight);
    wbc_ctrl_->setReactionForceWeight(parameters.balance_reaction_force_weight);
    wbc_ctrl_->setMaxNormalForce(parameters.maximum_normal_force);
  }
}

/*仅在第一次启动，后续不会启动*/
template<typename T>
void FSM_State_BalanceStand<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  //请求站立步态
  this->_data->gait_scheduler->requestGait(GaitType::STAND);

  // 进入状态时锁定当前水平位置和姿态，避免突然跳到世界原点。
  //这样子就到导致了步行时刻突然切换站立直接翻到
  initial_body_position_ = this->_data->state_estimate->position_world;
  //如果估计的机身高度低于20厘米，就认为高度估计不可靠，强行设置为30厘米。
  if (this->_data->quadruped->robotType() == RobotType::UNITREE_GO1) {
    if (initial_body_position_.z() < T(0.2)) {initial_body_position_.z() = T(0.3);}
  } else if (!this->_data->control_parameters->start_in_prone_home &&
    initial_body_position_.z() <
    this->_data->control_parameters->minimum_standing_height)
  {
    initial_body_position_.z() = this->_data->quadruped->nominalBodyHeight();
  }
  //后续高度限制以这个高度为起点
  last_height_command_ = initial_body_position_.z();
  // 世界航向沿用进入状态时的实测值，但站立目标始终保持机身水平。不能把
  // 切换瞬间的 roll/pitch 锁存为目标，否则已有倾斜会被 WBC 永久维持。
  initial_body_rpy_ << T(0), T(0), this->_data->state_estimate->rpy.z();
  body_weight_ = this->_data->quadruped->robotType() == RobotType::UNITREE_GO1 ?
    this->_data->quadruped->bodyInertia().mass * T(9.81) :
    this->_data->control_parameters->standing_supported_mass * T(9.81);
}

template<typename T>
void FSM_State_BalanceStand<T>::run()
{
  BalanceStandStep();
}

//检查是否切换状态
template<typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition()
{
  /*每次检查状态切换时，运行计数增加1。
    注意：它发生在 run() 之前还是之后，由 ControlFSM::runFSM() 的调用顺序决定。
    当前控制框架中，先检查切换，如果不切换才执行当前状态的 run()。*/
  ++iteration_;
  switch (this->_data->desired_state->mode) {
    case ControlMode::BalanceStand:
      break;
    case ControlMode::Locomotion:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      this->transitionDuration = T(0);
      this->_data->gait_scheduler->requestGait(GaitType::TROT_WALK);
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
    case ControlMode::FrontJump:
      this->nextStateName = FSM_StateName::FRONT_JUMP;
      this->transitionDuration = T(0);
      break;
  }
  return this->nextStateName;
}

/*transition：执行状态切换 在真正离开站立状态前，
再输出一帧有效的站立控制，避免切换瞬间没有关节命令。*/
template<typename T>
TransitionData<T> FSM_State_BalanceStand<T>::transition()
{
  /*在真正离开站立状态前，再输出一帧有效的站立控制，避免切换瞬间没有关节命令。*/
  if (this->nextStateName == FSM_StateName::LOCOMOTION) {BalanceStandStep();}
  /*进入Passive时关闭当前状态的安全检查.原因是Passive
    本身就是一种停止主动控制的状态，继续执行普通状态的足端和力矩检查没有意义。*/
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->turnOffAllSafetyChecks();
  }
  this->transitionData.done = true;
  return this->transitionData;
}

//onExit：退出状态
template<typename T>
void FSM_State_BalanceStand<T>::onExit()
{
  iteration_ = 0;
}
/*BalanceStandStep：真正的站立控制
            这是整个文件最重要的函数。*/
template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep()
{
  // 如果外部期望无效，就继续保持进入状态时记录的位姿。
  wbc_data_.pBody_des = initial_body_position_;
  wbc_data_.vBody_des.setZero();
  wbc_data_.aBody_des.setZero();
  wbc_data_.pBody_RPY_des = initial_body_rpy_;
  wbc_data_.vBody_Ori_des.setZero();
  
  //读取外部期望状态
  const DesiredState<T> & desired = *this->_data->desired_state;
  if (desired.valid) {
    wbc_data_.pBody_des = desired.body_position_world;
    wbc_data_.pBody_RPY_des = desired.body_rpy;
    wbc_data_.vBody_des = desired.body_velocity_world;
    wbc_data_.aBody_des = desired.body_acceleration_world;
    wbc_data_.vBody_Ori_des = desired.body_angular_velocity;
  }
  // 额外限制单周期下降量，防止高度滑块快速下拉造成腿部瞬时折叠。
  // 参数2 高度变化率
  const T maximum_height_step =
    this->_data->control_parameters->balance_height_step;
  if (last_height_command_ - wbc_data_.pBody_des.z() > maximum_height_step) {
    wbc_data_.pBody_des.z() = last_height_command_ - maximum_height_step;
  }
  if (wbc_data_.pBody_des.z() - last_height_command_ > maximum_height_step) {
      wbc_data_.pBody_des.z() = last_height_command_ + maximum_height_step;
  }
  last_height_command_ = wbc_data_.pBody_des.z();

  // 静态站立时先把体重平均分配给四只脚，WBIC 会在动力学约束下修正它。
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    wbc_data_.pFoot_des[leg].setZero();
    wbc_data_.vFoot_des[leg].setZero();
    wbc_data_.aFoot_des[leg].setZero();
    wbc_data_.Fr_des[leg] = Vec3<T>(T(0), T(0), body_weight_ / T(4));
    //标记为接触腿
    wbc_data_.contact_state[leg] = T(1);
  }
  //调用WBC
  /*&wbc_data_ 机身想去哪里；机身想保持什么姿态；四只脚是否接触；期望反力是多少。*/
  /*&state_estimate，包括：当前机身位置；当前机身姿态；当前机身速度；当前角速度。*/
  /*&joint_states,  12个关机的位置和速度*/
  /*position_desired  velocity_desired ,torque_feedforward
                                       kp_joint kd_joint */
  const bool wbc_valid = wbc_ctrl_->runAndApply(
    &wbc_data_, *this->_data->state_estimate, *this->_data->joint_states,
    *this->_data->leg_controller);

  if (!wbc_valid) {return;}

  // 机身任务和四足接触不能唯一确定 12 个关节角，KinWBC 仍存在姿态零空间。
  // 因此根据目标高度构造对称腿姿，并用较弱关节阻抗抑制零空间漂移；
  // WBIC 计算的全身前馈力矩和地面反力仍然保留。
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    //遍历四条腿
    const LegId leg_id = static_cast<LegId>(leg);
    //该腿的最终关节命令。
    auto & command = this->_data->leg_controller->commands[leg];
    //也就是该腿的模型参数，包括：关节上下限；home姿态；扭矩上限；连杆长度。
    const auto & leg_model = this->_data->quadruped->leg(leg_id);
    //GO1默认home姿态大致是：髋外展角；大腿俯仰角；小腿/膝角。
    const Vec3<T> home = leg_model.joints.home_position;
    //GO1名义机身高度当前是：
    command.position_desired = home;
    if (this->_data->control_parameters->use_go1_height_joint_mapping) {
      const T nominal_height = this->_data->quadruped->nominalBodyHeight();
      const T height_ratio = wbc_data_.pBody_des.z() / nominal_height;
      const T cosine = std::clamp(
        std::cos(home.y()) * height_ratio, T(0), T(1));
      const T thigh_angle = std::acos(cosine);
      command.position_desired << home.x(), thigh_angle, -T(2) * thigh_angle;
    }
    command.position_desired = command.position_desired.cwiseMax(
      leg_model.joints.lower_limit).cwiseMin(leg_model.joints.upper_limit);
    command.velocity_desired.setZero();
    if (this->_data->quadruped->robotType() == RobotType::UNITREE_GO1) {
      command.kp_joint.setConstant(T(20));
      command.kd_joint.setConstant(T(2));
    } else {
      command.kp_joint = this->_data->control_parameters->balance_joint_kp;
      command.kd_joint = this->_data->control_parameters->balance_joint_kd;
    }
  }
}

template class FSM_State_BalanceStand<float>;
