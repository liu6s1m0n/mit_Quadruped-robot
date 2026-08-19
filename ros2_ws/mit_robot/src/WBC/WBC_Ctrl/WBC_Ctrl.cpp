#include "WBC/WBC_Ctrl/WBC_Ctrl.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

// WBC 通用执行框架：更新动力学模型，组织任务/接触，依次运行 KinWBC 和 WBIC，
// 最后把关节位置、速度和前馈力矩写入腿部控制器。
/** @brief 接管浮动基模型，并创建运动学和动力学求解器。 */
template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model)
: model_(std::move(model)),
  kin_wbc_(std::make_unique<KinWBC<T>>(6 + kNumJoints)),
  wbic_(std::make_unique<WBIC<T>>(6 + kNumJoints, &contacts_, &tasks_)),
  wbic_data_(6, 0)
{
  if (model_.getJointPositionOffsets().size() == 0) {
    model_.setJointPositionOffsets(DVec<T>::Zero(kNumJoints));
  }
  state_.q = DVec<T>::Zero(kNumJoints);
  state_.qd = DVec<T>::Zero(kNumJoints);
  invalidate();
}

/** @brief 设置浮动基加速度修正的 WBIC 代价权重，必须为有限正数。 */
template<typename T>
void WBC_Ctrl<T>::setFloatingBaseWeight(T weight)
{
  if (!std::isfinite(static_cast<double>(weight)) || weight <= T(0)) {
    throw std::invalid_argument("floating-base weight must be finite and positive");
  }
  floating_base_weight_ = weight;
}

/** @brief 设置接触反力修正的 WBIC 代价权重，必须为有限正数。 */
template<typename T>
void WBC_Ctrl<T>::setReactionForceWeight(T weight)
{
  if (!std::isfinite(static_cast<double>(weight)) || weight <= T(0)) {
    throw std::invalid_argument("reaction-force weight must be finite and positive");
  }
  reaction_force_weight_ = weight;
}

/**
 * @brief 设置最终关节 PD 修正增益。
 * @param kp 三个关节的位置比例增益。
 * @param kd 三个关节的速度微分增益。
 */
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

/** @brief 按调用顺序注册一个任务，任务对象由派生控制器持有。 */
template<typename T>
void WBC_Ctrl<T>::addTask(Task<T> & task)
{
  tasks_.push_back(&task);
}

/** @brief 注册一个支撑接触约束，接触对象由派生控制器持有。 */
template<typename T>
void WBC_Ctrl<T>::addContact(ContactSpec<T> & contact)
{
  contacts_.push_back(&contact);
}

/** @brief 将上一次求解结果标记为无效并清零。 */
template<typename T>
void WBC_Ctrl<T>::invalidate() noexcept
{
  result_ = Result{};
}

/**
 * @brief 用状态估计和关节测量刷新动力学模型。
 * @return 质量矩阵及其逆、重力和科氏力均有效时返回 true。
 */
template<typename T>
bool WBC_Ctrl<T>::updateModel(
  const StateEstimate<T> & estimate,
  const std::array<JointState<T>, kNumLegs> & joint_states)
{
  try {
    // 同一份估计状态同时用于质量矩阵、重力、科氏力和接触雅可比，保证时刻一致。
    state_ = model::makeFloatingBaseState(estimate, joint_states);
    // 动力学树使用机械关节角；传感器和执行器接口使用现场校零后的电机读数。
    state_.q += model_.getJointPositionOffsets();
    model_.setState(state_); //设置到模型
    model_.contactJacobians(); //更新接触雅可比
    model_.massMatrix(); //更新质量矩阵
    model_.generalizedGravityForce(); //更新广义重力力
    model_.generalizedCoriolisForce(); //更新广义科氏力

    mass_matrix_ = model_.getMassMatrix(); //获取质量矩阵
    gravity_ = model_.getGravityForce(); //获取广义重力力
    coriolis_ = model_.getCoriolisForce(); //获取广义科氏力
    if (!mass_matrix_.allFinite() || !gravity_.allFinite() || !coriolis_.allFinite()) {
      return false;
    }
    /*它会被 KinWBC 或 WBIC 的加权伪逆使用*/
    mass_matrix_inverse_ = mass_matrix_.inverse();
    if (!mass_matrix_inverse_.allFinite()) {return false;}
  } catch (const std::exception &) {
    return false;
  }
  estimate_ = estimate;
  joint_states_ = joint_states;
  return true;
}

/**
 * @brief 调用 KinWBC 生成运动参考，再调用 WBIC 生成力矩和反力。
 * @return 所有任务/接触有效且两个求解器成功时返回 true。
 */
template<typename T>
bool WBC_Ctrl<T>::compute()
{
  for (const auto * task : tasks_) {
    if (task == nullptr || !task->IsTaskSet()) {return false;}
  }
  //所有支撑接触反力的总维度
  std::size_t reaction_force_dimension = 0;
  for (const auto * contact : contacts_) {
    if (contact == nullptr) {return false;}
    reaction_force_dimension += contact->getDim();
  }
  /*浮动基座标+接触反力维度*/
  wbic_data_ = WBIC_ExtraData<T>(6, reaction_force_dimension);
  /*设置代价函数*/
  wbic_data_._W_floating.setConstant(floating_base_weight_);
  wbic_data_._W_rf.setConstant(reaction_force_weight_);

  // KinWBC 先产生可实现的关节运动参考，不直接考虑所需力矩大小。
  /*重点函数，但是我看不懂*/
  if (!kin_wbc_->findConfiguration(
      state_.q, tasks_, contacts_, result_.joint_position, result_.joint_velocity))
  {
    return false;
  }
  /*更新 WBIC 动力学参数*/
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

/**
 * @brief 执行一个完整 WBC 控制周期。
 * @param input 派生类输入，由 prepareTasksAndContacts() 解释。
 * @param estimate 当前状态估计。
 * @param joint_states 四条腿关节测量。
 * @return 本周期求解成功时返回 true。
 */
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

/**
 * @brief 将有效 WBC 结果转换为四条腿关节命令。
 * @param leg_controller 接收命令的腿控制器；无效结果会关闭其输出。
 */
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
    command.position_desired =
      result_.joint_position.segment(offset, kJointsPerLeg) -
      model_.getJointPositionOffsets().segment(offset, kJointsPerLeg);
    command.velocity_desired = result_.joint_velocity.segment(offset, kJointsPerLeg);
    command.torque_feedforward = result_.joint_torque.segment(offset, kJointsPerLeg);
    // 最终关节命令 = 前馈力矩 + 对 WBC 运动参考的关节 PD 修正。
    command.kp_joint = kp_joint_;
    command.kd_joint = kd_joint_;
  }
}

/** @brief 执行 WBC 并立即将结果应用到腿控制器。 */
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
/**
 * @brief 设置 WBIC 中浮动基加速度修正的代价权重。
 * @param weight 必须为有限正数。
 * @throws std::invalid_argument weight 无效时抛出。
 */
/**
 * @brief 设置 WBIC 中接触反力修正的代价权重。
 * @param weight 必须为有限正数。
 * @throws std::invalid_argument weight 无效时抛出。
 */
/**
 * @brief 设置最终输出关节 PD 修正的比例和微分增益。
 * @param kp 三个关节的位置增益。
 * @param kd 三个关节的速度增益。
 * @throws std::invalid_argument 增益非有限或为负时抛出。
 */
/** @brief 将任务指针加入本周期任务列表，保持调用顺序作为优先级。 */
/** @brief 将接触约束指针加入本周期接触列表。 */
/** @brief 清空上一次求解结果，并将 valid 置为 false。 */
/**
 * @brief 用状态估计和关节测量更新浮动基动力学模型。
 * @return 质量矩阵、重力和科氏力均成功计算且为有限数时返回 true。
 */
/**
 * @brief 依次执行 KinWBC 和 WBIC，生成关节参考、前馈力矩和接触反力。
 * @return 所有任务已设置且两个求解器均成功时返回 true。
 */
/**
 * @brief 执行一个完整控制周期。
 * @param input 由派生控制器解释的任务/步态输入。
 * @return 模型更新、任务准备和 WBC 求解均成功时返回 true。
 */
/**
 * @brief 将最近一次有效 WBC 结果转换为四条腿的命令。
 * @param leg_controller 接收关节位置、速度、力矩和 PD 增益的腿控制器。
 */
/**
 * @brief 执行 WBC 并立即把结果应用到腿控制器。
 * @return WBC 成功时返回 true；失败时腿部输出保持关闭。
 */
