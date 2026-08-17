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
/**
 * @brief 运动学全身控制器。
 *
 * 按任务优先级递归求解广义位置增量和广义速度：高优先级任务先求解，
 * 低优先级任务只能使用前级任务留下的零空间。接触约束通过接触雅可比
 * 的零空间投影实现。伪逆采用阈值截断，避免奇异构型导致数值爆炸。
 *
 * 数学形式为 @f$N=I-J^{\#}J@f$。若 @f$\dot q=N\dot q_0@f$，则
 * @f$J\dot q=0@f$，因此不会破坏已满足的约束。
 *
 * @tparam T 矩阵和向量的标量类型。
 */
class KinWBC
{
public:
  /**
   * @brief 创建运动学全身控制器。
   * @param num_qdot 广义速度维数，浮动基座模型至少需要 6。
   * @throws std::invalid_argument 当 num_qdot 小于 6 时抛出。
   */
  explicit KinWBC(std::size_t num_qdot);
  /** @brief 析构函数。 */
  ~KinWBC() = default;

  /**
   * @brief 根据当前关节状态、任务和接触约束求解关节命令。
   *
   * 输入位置既可以是仅关节位置，也可以是包含浮动基座位置的广义位置；
   * 输出只包含可驱动关节的位置和速度。
   *
   * @param curr_config 当前关节位置或完整广义位置。
   * @param task_list 按优先级从高到低排列的任务列表。
   * @param contact_list 当前接触约束列表。
   * @param jpos_cmd 输出关节位置命令。
   * @param jvel_cmd 输出关节速度命令。
   * @return 输入有效且求解结果有限时返回 true，否则返回 false。
   */
  bool FindConfiguration(
    const DVec<T> & curr_config,
    const std::vector<Task<T> *> & task_list,
    const std::vector<ContactSpec<T> *> & contact_list,
    DVec<T> & jpos_cmd, DVec<T> & jvel_cmd);

  /** @brief FindConfiguration() 的小写兼容接口。 */
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

  /** @brief 预留的加权伪逆矩阵，当前运动学求解使用内部阈值伪逆。 */
  DMat<T> Ainv_;

private:
  /** @brief 计算矩阵的阈值截断伪逆 @f$J^{\#}@f$。 */
  void _PseudoInverse(const DMat<T> & J, DMat<T> & Jinv);
  /** @brief 根据 @f$N=I-J^{\#}J@f$ 构造零空间投影矩阵。 */
  void _BuildProjectionMatrix(const DMat<T> & J, DMat<T> & N);

  /** @brief 奇异值截断阈值。 */
  double threshold_;
  /** @brief 广义速度维数。 */
  std::size_t num_qdot_;
  /** @brief 可驱动关节维数，等于 num_qdot_ 减去浮动基座 6 维。 */
  std::size_t num_act_joint_;
  /** @brief 广义空间单位矩阵。 */
  DMat<T> I_mtx;
};
#endif
