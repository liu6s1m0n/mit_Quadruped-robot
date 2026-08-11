/**
 * @file BodyPosTask.hpp
 * @brief 机身质心位置任务，使用位置/速度 PD 加前馈加速度生成任务命令。
 */
#ifndef MYMIT_ROBOT_WBC_CTRL_BODY_POS_TASK_HPP_
#define MYMIT_ROBOT_WBC_CTRL_BODY_POS_TASK_HPP_

#include "WBC/FloatingBaseModel.h"
#include "WBC/Task.hpp"

template<typename T>
class BodyPosTask : public Task<T>
{
public:
  explicit BodyPosTask(const FloatingBaseModel<T> * model);
  explicit BodyPosTask(const FloatingBaseModel<T> & model) : BodyPosTask(&model) {}
  ~BodyPosTask() override = default;

  DVec<T> _Kp_kin;
  DVec<T> _Kp;
  DVec<T> _Kd;

protected:
  bool _UpdateCommand(
    const void * position_desired, const DVec<T> & velocity_desired,
    const DVec<T> & acceleration_desired) override;
  bool _UpdateTaskJacobian() override;
  bool _UpdateTaskJDotQdot() override;
  bool _AdditionalUpdate() override {return true;}

  const FloatingBaseModel<T> * _robot_sys;
};

extern template class BodyPosTask<float>;
extern template class BodyPosTask<double>;

#endif  // MYMIT_ROBOT_WBC_CTRL_BODY_POS_TASK_HPP_
