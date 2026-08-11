#include "WBC/KinWBC.hpp"
#include "pseudoInverse.h"

#include <stdexcept>

// 运动学 WBC：在满足接触约束的零空间内，按列表顺序逐级完成各个运动任务。
// 它只求关节位置/速度参考；动力学一致的力矩由 WBIC 另行计算。
template<typename T>
KinWBC<T>::KinWBC(std::size_t num_qdot)
: threshold_(0.001), num_qdot_(num_qdot),
  num_act_joint_(num_qdot >= 6 ? num_qdot - 6 : 0)
{
  if (num_qdot < 6) {
    throw std::invalid_argument("KinWBC requires at least six floating-base velocities");
  }
  I_mtx = DMat<T>::Identity(num_qdot_, num_qdot_);
}

template<typename T>
bool KinWBC<T>::FindConfiguration(
  const DVec<T> & curr_config, const std::vector<Task<T> *> & task_list,
  const std::vector<ContactSpec<T> *> & contact_list, DVec<T> & jpos_cmd,
  DVec<T> & jvel_cmd)
{

  const auto actuated = static_cast<Eigen::Index>(num_act_joint_);
  const auto generalized = static_cast<Eigen::Index>(num_qdot_);
  DVec<T> current_joint_position;
  if (curr_config.size() == actuated) {
    current_joint_position = curr_config;
  } else if (curr_config.size() == generalized) {
    current_joint_position = curr_config.tail(actuated);
  } else {
    return false;
  }
  if (!current_joint_position.allFinite()) {return false;}

  jpos_cmd = current_joint_position;
  jvel_cmd = DVec<T>::Zero(actuated);

  // Nc 是接触雅可比的零空间投影。投影后的运动不会破坏“支撑脚固定”约束。
  DMat<T> Nc = I_mtx;
  if (!contact_list.empty()) {
    Eigen::Index contact_rows = 0;
    std::vector<DMat<T>> jacobians;
    jacobians.reserve(contact_list.size());
    for (ContactSpec<T> * contact : contact_list) {
      if (contact == nullptr) {return false;}
      DMat<T> jacobian;
      contact->getContactJacobian(jacobian);
      if (jacobian.cols() != generalized || !jacobian.allFinite()) {return false;}
      contact_rows += jacobian.rows();
      jacobians.push_back(std::move(jacobian));
    }
    DMat<T> Jc(contact_rows, generalized);
    Eigen::Index row = 0;
    for (const auto & jacobian : jacobians) {
      Jc.middleRows(row, jacobian.rows()) = jacobian;
      row += jacobian.rows();
    }
    _BuildProjectionMatrix(Jc, Nc);
  }

  if (task_list.empty()) {return true;}

  // 第一个任务优先级最高，先在接触零空间中求满足它的最小范数解。
  DVec<T> delta_q, qdot;
  DMat<T> Jt, JtPre, JtPre_pinv, N_nx, N_pre;

  Task<T> * task = task_list[0];
  if (task == nullptr) {return false;}
  task->getTaskJacobian(Jt);
  if (Jt.cols() != generalized || task->getPosError().size() != Jt.rows() ||
    task->getDesVel().size() != Jt.rows() || !Jt.allFinite() ||
    !task->getPosError().allFinite() || !task->getDesVel().allFinite())
  {
    return false;
  }
  JtPre = Jt * Nc;
  _PseudoInverse(JtPre, JtPre_pinv);

  delta_q = JtPre_pinv * (task->getPosError());
  qdot = JtPre_pinv * (task->getDesVel());

  DVec<T> prev_delta_q = delta_q;
  DVec<T> prev_qdot = qdot;

  _BuildProjectionMatrix(JtPre, N_nx);
  N_pre = Nc * N_nx;

  for (size_t i(1); i < task_list.size(); ++i) {
    task = task_list[i];
    if (task == nullptr) {return false;}

    task->getTaskJacobian(Jt);
    if (Jt.cols() != generalized || task->getPosError().size() != Jt.rows() ||
      task->getDesVel().size() != Jt.rows() || !Jt.allFinite() ||
      !task->getPosError().allFinite() || !task->getDesVel().allFinite())
    {
      return false;
    }
    JtPre = Jt * N_pre;

    // 后续任务只利用高优先级任务留下的自由度，并修正前级解的剩余误差。
    _PseudoInverse(JtPre, JtPre_pinv);
    delta_q =
      prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
    qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);

    // 更新累计零空间，供下一个更低优先级任务使用。
    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre *= N_nx;
    prev_delta_q = delta_q;
    prev_qdot = qdot;
  }
  // 浮动基座的前 6 个自由度不可直接驱动，因此只输出末尾的关节部分。
  jpos_cmd = current_joint_position + delta_q.tail(actuated);
  jvel_cmd = qdot.tail(actuated);
  if (!jpos_cmd.allFinite() || !jvel_cmd.allFinite()) {return false;}
  return true;
}

template<typename T>
void KinWBC<T>::_BuildProjectionMatrix(const DMat<T> & J, DMat<T> & N)
{
  // N = I - J#J；任意经过 N 的速度都位于 J 的零空间内。
  DMat<T> J_pinv;
  _PseudoInverse(J, J_pinv);
  N = I_mtx - J_pinv * J;
}

template<typename T>
void KinWBC<T>::_PseudoInverse(const DMat<T> & J, DMat<T> & Jinv)
{
  pseudoInverse(J, threshold_, Jinv);
}

template class KinWBC<float>;
template class KinWBC<double>;
