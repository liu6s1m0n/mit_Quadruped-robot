// FSM 调度实现：安全检查优先于状态运行，任何无效状态或命令都会回到被动模式。
#include "FSM/ControlFSM.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
//将用户控制模式转换成 FSM 状态名称。
FSM_StateName stateForMode(ControlMode mode) noexcept
{
  switch (mode) {
    case ControlMode::Passive: return FSM_StateName::PASSIVE;
    case ControlMode::JointPd: return FSM_StateName::JOINT_PD;
    case ControlMode::BalanceStand: return FSM_StateName::BALANCE_STAND;
    case ControlMode::Locomotion: return FSM_StateName::LOCOMOTION;
    case ControlMode::StandUp: return FSM_StateName::STAND_UP;
    case ControlMode::RecoveryStand: return FSM_StateName::RECOVERY_STAND;
  }
  return FSM_StateName::INVALID;
}
//2. JointPdState这是 ControlFSM.cpp 内部定义的一个简单状态。
template<typename T>
class JointPdState final : public FSM_State<T>
{
public:
  explicit JointPdState(ControlFSMData<T> * data)
  : FSM_State<T>(data, FSM_StateName::JOINT_PD, "JOINT_PD") {}

  void onEnter() override
  {
    this->nextStateName = this->stateName;
    this->transitionData.zero();
  }

  void run() override
  {
    // 简单关节 PD 状态将四条腿保持在模型定义的默认姿态。
    auto & controller = *this->_data->leg_controller;
    controller.zeroCommand();
    controller.setEnabled(true);
    //遍历四条腿
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const LegId leg_id = static_cast<LegId>(leg);
      controller.commands[leg].position_desired =
        this->_data->quadruped->leg(leg_id).joints.home_position;
      controller.commands[leg].kp_joint.setConstant(T(20));
      controller.commands[leg].kd_joint.setConstant(T(2));
    }
  }
  
  /*读取用户选择的控制模式，并转换成下一个 FSM 状态。*/
  FSM_StateName checkTransition() override
  {
    this->nextStateName = stateForMode(this->_data->desired_state->mode);
    this->transitionDuration = T(0);
    return this->nextStateName;
  }
  
  /*切换时再执行一次 Joint PD 控制。作用是避免切换过程中出现无效命令。*/
  TransitionData<T> transition() override
  {
    run();
    this->transitionData.done = true;
    return this->transitionData;
  }

  void onExit() override {}
};

}  // namespace

//创建各个状态
template<typename T>
ControlFSM<T>::ControlFSM(
  const Quadruped<T> & quadruped, StateEstimate<T> & state_estimate,
  const std::array<JointState<T>, kNumLegs> & joint_states,
  LegController<T> & leg_controller, GaitScheduler<T> & gait_scheduler,
  DesiredState<T> & desired_state, T control_time_step)
{
  data.quadruped = &quadruped;
  data.state_estimate = &state_estimate;
  data.joint_states = &joint_states;
  data.leg_controller = &leg_controller;
  data.gait_scheduler = &gait_scheduler;
  data.desired_state = &desired_state;
  data.control_time_step = control_time_step;
  if (!data.valid()) {throw std::invalid_argument("invalid ControlFSM dependencies");}

  statesList.passive = std::make_unique<FSM_State_Passive<T>>(&data);
  statesList.joint_pd = std::make_unique<JointPdState<T>>(&data);
  statesList.stand_up = std::make_unique<FSM_State_StandUp<T>>(&data);
  statesList.recovery_stand = std::make_unique<FSM_State_RecoveryStand<T>>(&data);
  statesList.balance_stand = std::make_unique<FSM_State_BalanceStand<T>>(&data);
  statesList.locomotion = std::make_unique<FSM_State_Locomotion<T>>(&data);
  safety_checker_ = std::make_unique<SafetyChecker<T>>(&data);
  initialize();
}

//FSM 默认从 Passive 开始。调用 Passive 的进入函数，默认下一个状态也为 Passive。
template<typename T>
void ControlFSM<T>::initialize()
{
  currentState = statesList.passive.get();
  currentState->onEnter();
  nextState = currentState;
  nextStateName = currentState->stateName;
  operating_mode_ = FSM_OperatingMode::NORMAL;
}

//调度器的执行逻辑
template<typename T>
void ControlFSM<T>::runFSM()
{
  // 前置检查针对输入状态；失败时立即进入 ESTOP，并输出被动命令。
  operating_mode_ = safetyPreCheck();
  //检查失败,立即进入紧急停止流程
  if (operating_mode_ == FSM_OperatingMode::ESTOP) {
    //如果当前不是 Passive，先调用当前状态的 onExit()。
    if (currentState != statesList.passive.get()) {currentState->onExit();}
    currentState = statesList.passive.get();
    //切换到 Passive，并执行其进入逻辑。
    currentState->onEnter();
    //清除原来准备切换的目标状态。
    nextState = currentState;
    nextStateName = currentState->stateName;
    currentState->run();
    printInfo(0);
    ++iteration_;
    return;
  }

  if (operating_mode_ == FSM_OperatingMode::NORMAL) {
    // 正常模式先询问当前状态是否需要切换；不切换才执行本状态控制。
    nextStateName = currentState->checkTransition();
    if (nextStateName != currentState->stateName) {
      nextState = getNextState(nextStateName);
      if (nextState == nullptr) {
        operating_mode_ = FSM_OperatingMode::ESTOP;
      } else {
        operating_mode_ = FSM_OperatingMode::TRANSITIONING;
      }
    } else {
      currentState->run();
    }
  }

  if (operating_mode_ == FSM_OperatingMode::TRANSITIONING) {
    // transition() 由“旧状态”执行，完成后才调用旧状态 onExit 和新状态 onEnter。
    transitionData = currentState->transition();
    safetyPostCheck();
    if (transitionData.done) {
      currentState->onExit();
      currentState = nextState;
      currentState->onEnter();
      operating_mode_ = FSM_OperatingMode::NORMAL;
    }
  } else {
    safetyPostCheck();
  }
  printInfo(0);
  ++iteration_;
}

//执行控制前安全检查。
template<typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck()
{
  // 如果状态估计：无效；姿态包含 NaN；姿态包含 Inf；直接急停。
  if (!data.state_estimate->valid || !data.state_estimate->rpy.allFinite()) {
    return FSM_OperatingMode::ESTOP;
  }
  /*如果当前状态要求检查姿态，则调用安全检查器。不是所有状态都一定检查安全姿态。
    例如 Passive 状态可能不需要继续判断姿态，因为它本来就不输出主动支撑力矩。*/
  if (currentState->checkSafeOrientation &&
    !safety_checker_->checkSafeOrientation())
  {
    return FSM_OperatingMode::ESTOP;
  }
  return operating_mode_;
}

/*检查状态输出的腿部命令。*/
template<typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck()
{
  // 后置检查保护执行器：只要任一条腿出现 NaN/Inf，就关闭所有腿而不是部分输出。
  for (const auto & command : data.leg_controller->commands) {
    if (!command.position_desired.allFinite() ||
      !command.velocity_desired.allFinite() ||
      !command.torque_feedforward.allFinite() ||
      !command.force_feedforward.allFinite() ||
      !command.foot_position_desired.allFinite() ||
      !command.foot_velocity_desired.allFinite() ||
      !command.kp_joint.allFinite() || !command.kd_joint.allFinite() ||
      !command.kp_cartesian.allFinite() || !command.kd_cartesian.allFinite())
    {
      data.leg_controller->zeroCommand();
      data.leg_controller->setEnabled(false);
      operating_mode_ = FSM_OperatingMode::ESTOP;
      break;
    }
  }
  if (operating_mode_ != FSM_OperatingMode::ESTOP) {
    /*检查足端期望位置。Locomotion 中之前关闭了 checkPDesFoot，
    因为 Locomotion 自己管理世界坐标摆动足目标。*/
    if (currentState->checkPDesFoot) {safety_checker_->checkPDesFoot();}
    if (currentState->checkForceFeedForward) {
      safety_checker_->checkForceFeedForward();
    }
  }
  return operating_mode_;
}

template<typename T>
FSM_State<T> * ControlFSM<T>::getNextState(FSM_StateName state_name) noexcept
{
  switch (state_name) {
    case FSM_StateName::PASSIVE: return statesList.passive.get();
    case FSM_StateName::JOINT_PD: return statesList.joint_pd.get();
    case FSM_StateName::STAND_UP: return statesList.stand_up.get();
    case FSM_StateName::RECOVERY_STAND: return statesList.recovery_stand.get();
    case FSM_StateName::BALANCE_STAND: return statesList.balance_stand.get();
    case FSM_StateName::LOCOMOTION: return statesList.locomotion.get();
    case FSM_StateName::INVALID: return nullptr;
  }
  return nullptr;
}

template<typename T>
FSM_StateName ControlFSM<T>::currentStateName() const noexcept
{
  return currentState == nullptr ? FSM_StateName::INVALID : currentState->stateName;
}

template<typename T>
void ControlFSM<T>::setLocomotionForwardVelocity(T velocity)
{
  statesList.locomotion->setForwardVelocity(velocity);
}

template<typename T>
void ControlFSM<T>::printInfo(int option)
{
  if (option == 0 && ++print_iteration_ < print_num_) {return;}
  print_iteration_ = 0;
  std::cout << "[CONTROL FSM] iteration " << iteration_ << ", state "
            << (currentState == nullptr ? "INVALID" : currentState->stateString)
            << ", gait " << data.gait_scheduler->gait_data.gait_name << '\n';
}

template class ControlFSM<float>;
