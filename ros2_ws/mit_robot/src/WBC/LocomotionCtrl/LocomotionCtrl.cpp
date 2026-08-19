#include "WBC/LocomotionCtrl/LocomotionCtrl.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "Utilities/orientation_tools.h"

// 四足运动 WBC 适配层：机身姿态和位置始终作为任务；每条腿根据接触状态，
// 在“支撑接触约束”与“摆动足位置任务”之间二选一。
/**
 * @brief 初始化一周期运动控制输入。
 *
 * Eigen 成员已经在声明处清零；这里额外清零 C 数组形式的四腿输入，保持
 * 与旧版调用方的内存布局兼容。
 */
template<typename T>
LocomotionCtrlData<T>::LocomotionCtrlData() noexcept
{
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    pFoot_des[leg].setZero();
    vFoot_des[leg].setZero();
    aFoot_des[leg].setZero();
    Fr_des[leg].setZero();
  }
}

/**
 * @brief 检查机身、足端轨迹、反力和接触状态输入。
 * @return 所有元素均为有限数时返回 true。
 */
template<typename T>
bool LocomotionCtrlData<T>::allFinite() const noexcept
{
  if (!pBody_des.allFinite() || !vBody_des.allFinite() ||
    !aBody_des.allFinite() || !pBody_RPY_des.allFinite() ||
    !vBody_Ori_des.allFinite() || !contact_state.allFinite())
  {
    return false;
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (!pFoot_des[leg].allFinite() || !vFoot_des[leg].allFinite() ||
      !aFoot_des[leg].allFinite() || !Fr_des[leg].allFinite())
    {
      return false;
    }
  }
  return true;
}

/**
 * @brief 构造运动 WBC 的机身任务、足端任务和接触约束。
 * @param model 要控制的浮动基机器人模型，按值传入并移动保存。
 * @throws std::invalid_argument 足端数量不是四个时抛出。
 */
template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(FloatingBaseModel<T> model)
: WBC_Ctrl<T>(std::move(model))
{
  const auto & foot_indices = this->model().getFootIndices();  // 模型中四个足端刚体的索引。
  if (foot_indices.size() != kNumLegs) {
    throw std::invalid_argument("locomotion controller requires exactly four foot contacts");
  }

  body_position_task_ = std::make_unique<BodyPosTask<T>>(&this->model());
  body_orientation_task_ = std::make_unique<BodyOriTask<T>>(this->model());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_contact_indices_[leg] = static_cast<std::size_t>(foot_indices[leg]);
    foot_contacts_[leg] =
      std::make_unique<SingleContact<T>>(this->model(), foot_contact_indices_[leg]);
    foot_tasks_[leg] =
      std::make_unique<LinkPosTask<T>>(this->model(), foot_contact_indices_[leg]);
  }
}

/**
 * @brief 设置机身位置任务的比例/微分增益。
 * @param kp 三个方向的位置比例增益。
 * @param kd 三个方向的速度微分增益。
 * @throws std::invalid_argument 增益包含非有限数或负数时抛出。
 */
template<typename T>
void LocomotionCtrl<T>::setBodyPositionGains(const Vec3<T> & kp, const Vec3<T> & kd)
{
  if (!kp.allFinite() || !kd.allFinite() || (kp.array() < T(0)).any() ||
    (kd.array() < T(0)).any())
  {
    throw std::invalid_argument("body position gains must be finite and non-negative");
  }
  body_position_task_->_Kp = kp;
  body_position_task_->_Kd = kd;
}

/**
 * @brief 设置机身姿态任务的比例/微分增益。
 * @param kp roll、pitch、yaw 三个方向的比例增益。
 * @param kd roll、pitch、yaw 三个方向的微分增益。
 */
template<typename T>
void LocomotionCtrl<T>::setBodyOrientationGains(
  const Vec3<T> & kp, const Vec3<T> & kd)
{
  body_orientation_task_->setProportionalGain(kp);
  body_orientation_task_->setDerivativeGain(kd);
}

/**
 * @brief 设置四条摆动腿足端位置任务的比例/微分增益。
 * @param kp 足端 x/y/z 位置比例增益。
 * @param kd 足端 x/y/z 速度微分增益。
 */
template<typename T>
void LocomotionCtrl<T>::setFootPositionGains(const Vec3<T> & kp, const Vec3<T> & kd)
{
  for (auto & task : foot_tasks_) {
    task->setProportionalGain(kp);
    task->setDerivativeGain(kd);
  }
}

/**
 * @brief 设置所有支撑接触约束的最大法向力。
 * @param max_fz 最大法向力，单位 N。
 * @throws std::invalid_argument max_fz 不是有限正数时抛出。
 */
template<typename T>
void LocomotionCtrl<T>::setMaxNormalForce(T max_fz)
{
  if (!std::isfinite(static_cast<double>(max_fz)) || max_fz <= T(0)) {
    throw std::invalid_argument("maximum normal force must be finite and positive");
  }
  for (auto & contact : foot_contacts_) {contact->setMaxFz(max_fz);}
}

/**
 * @brief 根据上层输入建立本周期的 WBC 任务列表和接触列表。
 * @param input 指向 LocomotionCtrlData<T> 的非空指针。
 * @return 输入和各任务约束均有效时返回 true，否则返回 false。
 */
template<typename T>
bool LocomotionCtrl<T>::prepareTasksAndContacts(const void * input)
{
  if (input == nullptr) {return false;}
  const auto & input_data = *static_cast<const LocomotionCtrlData<T> *>(input);  // 上层 MPC/WBC 输入快照。
  if (!input_data.allFinite()) {return false;}
  active_contact_state_ = input_data.contact_state;

  // 外部输入使用易读的 RPY，姿态任务内部使用四元数以避免直接做欧拉角差。
  const Quat<T> quaternion = ori::rpyToQuat(input_data.pBody_RPY_des);  // RPY 转四元数，避免直接相减欧拉角。
  const Eigen::Quaternion<T> desired_orientation(  // 姿态任务使用的期望四元数。
    quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  if (!body_orientation_task_->update(
      desired_orientation, input_data.vBody_Ori_des, Vec3<T>::Zero()))
  {
    return false;
  }

  DVec<T> body_velocity = input_data.vBody_des;       // 机身位置任务的期望线速度。
  DVec<T> body_acceleration = input_data.aBody_des;   // 机身位置任务的期望线加速度。
  if (!body_position_task_->UpdateTask(
      &input_data.pBody_des, body_velocity, body_acceleration))
  {
    return false;
  }
  this->addTask(*body_orientation_task_);
  this->addTask(*body_position_task_);

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (input_data.contact_state[leg] > T(0)) {
      // 支撑腿：固定足端并跟踪上游（通常为 MPC）给出的地面反作用力。
      DVec<T> desired_force = input_data.Fr_des[leg];  // 当前支撑腿期望承受的地面反力。
      foot_contacts_[leg]->setRFDesired(desired_force);
      if (!foot_contacts_[leg]->UpdateContactSpec()) {return false;}
      this->addContact(*foot_contacts_[leg]);
    } else {
      // 摆动腿：不施加接触力，改为跟踪足端位置、速度和加速度轨迹。
      if (!foot_tasks_[leg]->update(
          input_data.pFoot_des[leg], input_data.vFoot_des[leg],
          input_data.aFoot_des[leg]))
      {
        return false;
      }
      this->addTask(*foot_tasks_[leg]);
    }
  }
  return true;
}

/**
 * @brief 将 WBIC 紧凑排列的支撑腿反力还原为四腿数组。
 * @return 按 FR、FL、RR、RL 排列的世界坐标系反力，单位 N。
 */
template<typename T>
std::array<Vec3<T>, kNumLegs> LocomotionCtrl<T>::reactionForces() const
{
  std::array<Vec3<T>, kNumLegs> forces{};  // 固定按 FR/FL/RR/RL 排列的四腿反力输出。
  for (auto & force : forces) {force.setZero();}
  if (!this->result().valid) {return forces;}

  Eigen::Index offset = 0;  // WBIC 紧凑反力向量中当前支撑腿的起始索引。
  // WBIC 只紧凑存储当前支撑腿的反力，这里按 contact_state 恢复为固定四腿数组。
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (active_contact_state_[leg] > T(0)) {
      if (offset + 3 > this->result().reaction_force.size()) {
        for (auto & force : forces) {force.setZero();}
        return forces;
      }
      forces[leg] = this->result().reaction_force.template segment<3>(offset);
      offset += 3;
    }
  }
  return forces;
}

template struct LocomotionCtrlData<float>;
template struct LocomotionCtrlData<double>;
template class LocomotionCtrl<float>;
