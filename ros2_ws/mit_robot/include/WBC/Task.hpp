/**
 * @file Task.hpp
 * @brief WBC 任务的抽象基类，统一保存任务雅可比、Jdot*qdot 和期望任务加速度。
 */
#ifndef WBC_TASK
#define WBC_TASK

#include "Utilities/cppTypes.h"

#define TK Task<T>

template<typename T>
class Task
{
public:
  Task(size_t dim)
  : b_set_task_(false),
    dim_task_(dim),
    op_cmd_(dim),
    pos_err_(dim),
    vel_des_(dim),
    acc_des_(dim) {}

  virtual ~Task() = default;

  void getCommand(DVec<T> & op_cmd) const {op_cmd = op_cmd_;}
  void getTaskJacobian(DMat<T> & Jt) const {Jt = Jt_;}
  void getTaskJacobianDotQdot(DVec<T> & JtDotQdot) const {JtDotQdot = JtDotQdot_;}

  bool UpdateTask(
    const void * pos_des, const DVec<T> & vel_des,
    const DVec<T> & acc_des)
  {
    if (vel_des.size() != static_cast<Eigen::Index>(dim_task_) ||
      acc_des.size() != static_cast<Eigen::Index>(dim_task_) ||
      !vel_des.allFinite() || !acc_des.allFinite()) {return false;}
    b_set_task_ = _UpdateTaskJacobian() && _UpdateTaskJDotQdot() &&
      _UpdateCommand(pos_des, vel_des, acc_des) && _AdditionalUpdate();
    return b_set_task_;
  }

  bool IsTaskSet() const {return b_set_task_;}
  size_t getDim() const {return dim_task_;}
  void UnsetTask() {b_set_task_ = false;}

  const DVec<T> & getPosError() const {return pos_err_;}
  const DVec<T> & getDesVel() const {return vel_des_;}
  const DVec<T> & getDesAcc() const {return acc_des_;}

protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(
    const void * pos_des, const DVec<T> & vel_des,
    const DVec<T> & acc_des) = 0;
  // Update Jt_
  virtual bool _UpdateTaskJacobian() = 0;
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot() = 0;
  // Additional Update (defined in child classes)
  virtual bool _AdditionalUpdate() = 0;

  bool b_set_task_;   ///< 本周期任务数据是否已经成功更新。
  size_t dim_task_;   ///< 任务空间维数，例如位置任务为 3。

  DVec<T> op_cmd_;      ///< 最终操作空间加速度命令，供 KinWBC/WBIC 使用。
  DVec<T> JtDotQdot_;   ///< 任务雅可比变化造成的偏置加速度 Jdot*qdot。
  DMat<T> Jt_;          ///< 任务雅可比，将广义速度映射到任务空间速度。

  DVec<T> pos_err_;  ///< 当前任务位置误差，定义为期望值减实际值。
  DVec<T> vel_des_;  ///< 任务空间期望速度。
  DVec<T> acc_des_;  ///< 任务空间前馈期望加速度。
};

#endif
