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

  bool b_set_task_;
  size_t dim_task_;

  DVec<T> op_cmd_;
  DVec<T> JtDotQdot_;
  DMat<T> Jt_;

  DVec<T> pos_err_;
  DVec<T> vel_des_;
  DVec<T> acc_des_;
};

#endif
