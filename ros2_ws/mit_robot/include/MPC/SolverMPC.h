/**
 * @file SolverMPC.h
 * @brief 有限时域接触力优化器及其参数、结果结构。
 *
 * 求解变量是每个预测时刻四只脚的三维地面反力，约束包括接触开关、法向力范围和摩擦锥。
 */
#ifndef MYMIT_ROBOT_MPC_SOLVER_MPC_H_
#define MYMIT_ROBOT_MPC_SOLVER_MPC_H_

#include <array>
#include <cstddef>
#include <vector>

#include "MPC/RobotState.h"

namespace mpc
{

template<typename T>
struct SolverSettings
{
  //参数预测时间  horizon是次数 time_step则是每一步的时间 总计算时间是相乘
  //原来是10次 ，先改成8次
  std::size_t horizon = 10;  ///< 预测窗包含的离散时刻数。
  T time_step = T(0.03);     ///< 相邻预测时刻的时间间隔，s。

  T friction_coefficient = T(0.4);  ///< 线性摩擦锥使用的地面摩擦系数 mu。
  T minimum_normal_force = T(0);    ///< 支撑脚允许的最小法向力，N。
  T maximum_normal_force = T(120);  ///< 单只支撑脚允许的最大法向力，N。
  //惩罚参数值
  T force_regularization = T(1e-5);  ///< 抑制过大或剧烈变化接触力的正则权重。
  // 迭代上限温和提高到75；MPC以25 Hz求解，不影响500 Hz WBC/电机闭环。
  std::size_t maximum_iterations = 80;  ///< 迭代优化器单次求解的最大迭代次数。
  //收敛容差 接触精度，依靠迭代法所以降低精度也可以减少计算
  T convergence_tolerance = T(1e-5);  ///< 判断迭代收敛的残差阈值。

  /** @brief 检查预测时域、力约束和迭代参数是否有效。 */
  bool isValid() const noexcept;
};

template<typename T>
struct SolverResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::array<Vec3<T>, kNumLegs> reaction_forces_world{};  ///< 当前时刻四脚世界系地面反力，N。
  std::vector<T> force_trajectory;  ///< 整个预测窗的最优接触力序列，按时刻和腿拼接。
  std::size_t iterations = 0;  ///< 本次优化实际执行的迭代次数。
  T residual = T(0);           ///< 终止时的最优性/约束综合残差。
  bool converged = false;      ///< 是否在迭代上限前满足收敛阈值。
  bool valid = false;          ///< 输出维度、数值和约束是否均可用于控制。
};

template<typename T>
class SolverMPC
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 创建有限时域接触力求解器。
   * @param quadruped 四足机器人模型，提供质量和机身惯量。
   * @param settings 预测时域、摩擦锥和迭代求解参数。
   */
  explicit SolverMPC(
    const Quadruped<T> & quadruped,
    const SolverSettings<T> & settings = SolverSettings<T>{});

  /**
   * @brief 求解预测窗内的接触反力。
   *
   * 优化变量为每个预测段、每条腿的三维地面反力；当前控制周期只返回
   * 最优序列的第一段，完整序列保存在 force_trajectory 中。
   *
   * @param state 当前 MPC 状态。
   * @param trajectory 预测窗内的期望机身状态。
   * @param contact_table 预测窗接触表。
   * @return 接触反力、收敛状态和有效性标志。
   */
  SolverResult<T> solve(
    const RobotState<T> & state, const std::vector<DesiredState<T>> & trajectory,
    const std::vector<int> & contact_table) const;

  const SolverSettings<T> & settings() const noexcept {return settings_;}

private:
  DVec<T> desiredVector(const DesiredState<T> & desired) const;
  void projectForce(Vec3<T> & force, bool contact) const noexcept;

  const Quadruped<T> * quadruped_;  ///< 非拥有指针，提供质量和足端几何参数。
  SolverSettings<T> settings_;      ///< 本求解器固定使用的预测和约束参数副本。
};

extern template struct SolverSettings<float>;
extern template struct SolverSettings<double>;
extern template struct SolverResult<float>;
extern template struct SolverResult<double>;
extern template class SolverMPC<float>;
extern template class SolverMPC<double>;

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_SOLVER_MPC_H_
