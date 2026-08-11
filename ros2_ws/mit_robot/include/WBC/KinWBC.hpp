/**
 * @file KinWBC.hpp
 * @brief 运动学层全身控制器，按任务优先级求期望关节位置和速度。
 *
 * 它使用零空间投影避免低优先级任务破坏高优先级任务，不处理动力学力矩。
 */
#ifndef KINEMATICS_WHOLE_BODY_CONTROL
#define KINEMATICS_WHOLE_BODY_CONTROL

#include "WBC/ContactSpec.hpp"
#include "WBC/Task.hpp"
#include <cstddef>
#include <vector>

template<typename T>
class KinWBC
{
public:
  explicit KinWBC(std::size_t num_qdot);
  ~KinWBC() = default;

  bool FindConfiguration(
    const DVec<T> & curr_config,
    const std::vector<Task<T> *> & task_list,
    const std::vector<ContactSpec<T> *> & contact_list,
    DVec<T> & jpos_cmd, DVec<T> & jvel_cmd);

  bool findConfiguration(
    const DVec<T> & current_joint_or_generalized_position,
    const std::vector<Task<T> *> & task_list,
    const std::vector<ContactSpec<T> *> & contact_list,
    DVec<T> & joint_position_command,
    DVec<T> & joint_velocity_command)
  {
    return FindConfiguration(
      current_joint_or_generalized_position, task_list,
      contact_list, joint_position_command,
      joint_velocity_command);
  }

  DMat<T> Ainv_;

private:
  void _PseudoInverse(const DMat<T> & J, DMat<T> & Jinv);
  void _BuildProjectionMatrix(const DMat<T> & J, DMat<T> & N);

  double threshold_;
  std::size_t num_qdot_;
  std::size_t num_act_joint_;
  DMat<T> I_mtx;
};
#endif
