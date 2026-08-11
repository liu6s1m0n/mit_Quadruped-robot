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
  std::size_t horizon = 10;
  T time_step = T(0.03);
  T friction_coefficient = T(0.4);
  T minimum_normal_force = T(0);
  T maximum_normal_force = T(120);
  T force_regularization = T(1e-5);
  std::size_t maximum_iterations = 250;
  T convergence_tolerance = T(1e-5);

  bool isValid() const noexcept;
};

template<typename T>
struct SolverResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::array<Vec3<T>, kNumLegs> reaction_forces_world{};
  std::vector<T> force_trajectory;
  std::size_t iterations = 0;
  T residual = T(0);
  bool converged = false;
  bool valid = false;
};

template<typename T>
class SolverMPC
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SolverMPC(
    const Quadruped<T> & quadruped,
    const SolverSettings<T> & settings = SolverSettings<T>{});

  SolverResult<T> solve(
    const RobotState<T> & state, const std::vector<DesiredState<T>> & trajectory,
    const std::vector<int> & contact_table) const;

  const SolverSettings<T> & settings() const noexcept {return settings_;}

private:
  DVec<T> desiredVector(const DesiredState<T> & desired) const;
  void projectForce(Vec3<T> & force, bool contact) const noexcept;

  const Quadruped<T> * quadruped_;
  SolverSettings<T> settings_;
};

extern template struct SolverSettings<float>;
extern template struct SolverSettings<double>;
extern template struct SolverResult<float>;
extern template struct SolverResult<double>;
extern template class SolverMPC<float>;
extern template class SolverMPC<double>;

}  // namespace mpc

#endif  // MYMIT_ROBOT_MPC_SOLVER_MPC_H_
