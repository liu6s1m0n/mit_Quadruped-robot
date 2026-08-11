#include "WBC/WBC_Ctrl/WBC_Ctrl.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

// WBC 通用执行框架：更新动力学模型，组织任务/接触，依次运行 KinWBC 和 WBIC，
// 最后把关节位置、速度和前馈力矩写入腿部控制器。
template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model)
: model_(std::move(model)),
  kin_wbc_(std::make_unique<KinWBC<T>>(6 + kNumJoints)),
  wbic_(std::make_unique<WBIC<T>>(6 + kNumJoints, &contacts_, &tasks_)),
  wbic_data_(6, 0)
{
  state_.q = DVec<T>::Zero(kNumJoints);
  state_.qd = DVec<T>::Zero(kNumJoints);
  invalidate();
}

template<typename T>
void WBC_Ctrl<T>::setFloatingBaseWeight(T weight)
{
  if (!std::isfinite(static_cast<double>(weight)) || weight <= T(0)) {
    throw std::invalid_argument("floating-base weight must be finite and positive");
  }
  floating_base_weight_ = weight;
}

template<typename T>
void WBC_Ctrl<T>::setReactionForceWeight(T weight)
{
  if (!std::isfinite(static_cast<double>(weight)) || weight <= T(0)) {
    throw std::invalid_argument("reaction-force weight must be finite and positive");
  }
  reaction_force_weight_ = weight;
}

template<typename T>
void WBC_Ctrl<T>::setJointGains(const Vec3<T> & kp, const Vec3<T> & kd)
{
  if (!kp.allFinite() || !kd.allFinite() || (kp.array() < T(0)).any() ||
    (kd.array() < T(0)).any())
  {
    throw std::invalid_argument("joint gains must be finite and non-negative");
  }
  kp_joint_ = kp;
  kd_joint_ = kd;
}

template<typename T>
void WBC_Ctrl<T>::addTask(Task<T> & task)
{
  tasks_.push_back(&task);
}

template<typename T>
void WBC_Ctrl<T>::addContact(ContactSpec<T> & contact)
{
  contacts_.push_back(&contact);
}

template<typename T>
void WBC_Ctrl<T>::invalidate() noexcept
{
  result_ = Result{};
}

template<typename T>
bool WBC_Ctrl<T>::updateModel(
  const StateEstimate<T> & estimate,
  const std::array<JointState<T>, kNumLegs> & joint_states)
{
  try {
    // 同一份估计状态同时用于质量矩阵、重力、科氏力和接触雅可比，保证时刻一致。
    state_ = model::makeFloatingBaseState(estimate, joint_states);
    model_.setState(state_);
    model_.contactJacobians();
    model_.massMatrix();
    model_.generalizedGravityForce();
    model_.generalizedCoriolisForce();
    mass_matrix_ = model_.getMassMatrix();
    gravity_ = model_.getGravityForce();
    coriolis_ = model_.getCoriolisForce();
    if (!mass_matrix_.allFinite() || !gravity_.allFinite() || !coriolis_.allFinite()) {
      return false;
    }
    mass_matrix_inverse_ = mass_matrix_.inverse();
    if (!mass_matrix_inverse_.allFinite()) {return false;}
  } catch (const std::exception &) {
    return false;
  }
  estimate_ = estimate;
  joint_states_ = joint_states;
  return true;
}

template<typename T>
bool WBC_Ctrl<T>::compute()
{
  for (const auto * task : tasks_) {
    if (task == nullptr || !task->IsTaskSet()) {return false;}
  }

  std::size_t reaction_force_dimension = 0;
  for (const auto * contact : contacts_) {
    if (contact == nullptr) {return false;}
    reaction_force_dimension += contact->getDim();
  }
  wbic_data_ = WBIC_ExtraData<T>(6, reaction_force_dimension);
  wbic_data_._W_floating.setConstant(floating_base_weight_);
  wbic_data_._W_rf.setConstant(reaction_force_weight_);

  // KinWBC 先产生可实现的关节运动参考，不直接考虑所需力矩大小。
  if (!kin_wbc_->findConfiguration(
      state_.q, tasks_, contacts_, result_.joint_position, result_.joint_velocity))
  {
    return false;
  }
  wbic_->UpdateSetting(
    mass_matrix_, mass_matrix_inverse_, coriolis_, gravity_);
  // WBIC 再加入完整动力学和接触力约束，生成关节前馈力矩。
  if (!wbic_->makeTorque(result_.joint_torque, wbic_data_)) {return false;}

  result_.generalized_acceleration = wbic_data_._qddot;
  result_.reaction_force = wbic_data_._Fr;
  result_.valid = result_.joint_position.allFinite() &&
    result_.joint_velocity.allFinite() && result_.joint_torque.allFinite();
  return result_.valid;
}

template<typename T>
bool WBC_Ctrl<T>::run(
  const void * input, const StateEstimate<T> & estimate,
  const std::array<JointState<T>, kNumLegs> & joint_states)
{
  // 任务和接触会随步态逐周期变化，必须清空后由派生控制器重新填写。
  ++iteration_;
  invalidate();
  tasks_.clear();
  contacts_.clear();
  if (!updateModel(estimate, joint_states)) {return false;}
  if (!prepareTasksAndContacts(input)) {return false;}
  return compute();
}

template<typename T>
void WBC_Ctrl<T>::applyResult(LegController<T> & leg_controller) const
{
  // 先清零并按结果有效性设置使能，避免失败时沿用上一周期命令。
  leg_controller.zeroCommand();
  leg_controller.setEnabled(result_.valid);
  if (!result_.valid) {return;}

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = leg_controller.commands[leg];
    const Eigen::Index offset = static_cast<Eigen::Index>(leg * kJointsPerLeg);
    command.position_desired = result_.joint_position.segment(offset, kJointsPerLeg);
    command.velocity_desired = result_.joint_velocity.segment(offset, kJointsPerLeg);
    command.torque_feedforward = result_.joint_torque.segment(offset, kJointsPerLeg);
    // 最终关节命令 = 前馈力矩 + 对 WBC 运动参考的关节 PD 修正。
    command.kp_joint = kp_joint_;
    command.kd_joint = kd_joint_;
  }
}

template<typename T>
bool WBC_Ctrl<T>::runAndApply(
  const void * input, const StateEstimate<T> & estimate,
  const std::array<JointState<T>, kNumLegs> & joint_states,
  LegController<T> & leg_controller)
{
  const bool success = run(input, estimate, joint_states);
  applyResult(leg_controller);
  return success;
}

template class WBC_Ctrl<float>;
