#include "MPC/SolverMPC.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mpc
{
namespace
{

template<typename T>
Mat3<T> skew(const Vec3<T> & value)
{
  Mat3<T> result;
  result << T(0), -value.z(), value.y(), value.z(), T(0), -value.x(),
    -value.y(), value.x(), T(0);
  return result;
}

}  // namespace

template<typename T>
bool SolverSettings<T>::isValid() const noexcept
{
  return horizon > 0 && std::isfinite(static_cast<double>(time_step)) &&
    time_step > T(0) && std::isfinite(static_cast<double>(friction_coefficient)) &&
    friction_coefficient > T(0) && minimum_normal_force >= T(0) &&
    maximum_normal_force >= minimum_normal_force && force_regularization > T(0) &&
    maximum_iterations > 0 && convergence_tolerance > T(0);
}

template<typename T>
SolverMPC<T>::SolverMPC(
  const Quadruped<T> & quadruped, const SolverSettings<T> & settings)
: quadruped_(&quadruped), settings_(settings)
{
  if (!settings_.isValid()) {throw std::invalid_argument("invalid MPC settings");}
}

template<typename T>
DVec<T> SolverMPC<T>::desiredVector(const DesiredState<T> & desired) const
{
  DVec<T> result(12);
  result << desired.body_rpy, desired.body_position_world,
    desired.body_angular_velocity, desired.body_velocity_world;
  return result;
}

template<typename T>
void SolverMPC<T>::projectForce(Vec3<T> & force, bool contact) const noexcept
{
  if (!contact) {force.setZero(); return;}
  force.z() = std::clamp(
    force.z(), settings_.minimum_normal_force, settings_.maximum_normal_force);
  const T tangential_limit = settings_.friction_coefficient * force.z();
  const T tangential_norm = force.template head<2>().norm();
  if (tangential_norm > tangential_limit && tangential_norm > T(0)) {
    force.template head<2>() *= tangential_limit / tangential_norm;
  }
}

template<typename T>
SolverResult<T> SolverMPC<T>::solve(
  const RobotState<T> & state, const std::vector<DesiredState<T>> & trajectory,
  const std::vector<int> & contact_table) const
{
  SolverResult<T> result;
  const std::size_t horizon = settings_.horizon;
  if (!state.isValid() || trajectory.size() != horizon ||
    contact_table.size() != horizon * kNumLegs)
  {
    return result;
  }
  for (const auto & desired : trajectory) {
    if (!desired.valid || !desired.body_position_world.allFinite() ||
      !desired.body_velocity_world.allFinite() || !desired.body_rpy.allFinite() ||
      !desired.body_angular_velocity.allFinite()) {return result;}
  }

  constexpr Eigen::Index state_dimension = 12;
  const Eigen::Index input_dimension = static_cast<Eigen::Index>(horizon * 12);
  DMat<T> transition = DMat<T>::Identity(state_dimension, state_dimension);
  transition.block(0, 6, 3, 3) = settings_.time_step * Mat3<T>::Identity();
  transition.block(3, 9, 3, 3) = settings_.time_step * Mat3<T>::Identity();

  const T mass = quadruped_->totalMass();
  const Mat3<T> inertia_world = state.rotation_world_from_body *
    quadruped_->bodyInertia().inertia_com * state.rotation_world_from_body.transpose();
  if (!std::isfinite(static_cast<double>(mass)) || mass <= T(0) ||
    std::abs(inertia_world.determinant()) <= std::numeric_limits<T>::epsilon())
  {
    return result;
  }
  const Mat3<T> inertia_inverse = inertia_world.inverse();

  DVec<T> initial(state_dimension);
  initial << state.rpy, state.position_world, state.angular_velocity_world,
    state.velocity_world;
  DVec<T> free_state = initial;
  DMat<T> sensitivity = DMat<T>::Zero(state_dimension, input_dimension);
  DMat<T> prediction = DMat<T>::Zero(
    static_cast<Eigen::Index>(horizon) * state_dimension, input_dimension);
  DVec<T> free_prediction(static_cast<Eigen::Index>(horizon) * state_dimension);
  DVec<T> desired_prediction(static_cast<Eigen::Index>(horizon) * state_dimension);

  const Vec3<T> gravity(T(0), T(0), T(-9.81));
  for (std::size_t step = 0; step < horizon; ++step) {
    free_state = transition * free_state;
    free_state.template segment<3>(9) += settings_.time_step * gravity;
    sensitivity = transition * sensitivity;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      DMat<T> input = DMat<T>::Zero(state_dimension, 3);
      const Vec3<T> lever = state.foot_position_world[leg] - state.position_world;
      input.block(6, 0, 3, 3) =
        settings_.time_step * inertia_inverse * skew(lever);
      input.block(9, 0, 3, 3) =
        settings_.time_step * Mat3<T>::Identity() / mass;
      sensitivity.block(0, static_cast<Eigen::Index>(step * 12 + leg * 3), 12, 3) = input;
    }
    const Eigen::Index row = static_cast<Eigen::Index>(step) * state_dimension;
    prediction.block(row, 0, state_dimension, input_dimension) = sensitivity;
    free_prediction.segment(row, state_dimension) = free_state;
    desired_prediction.segment(row, state_dimension) = desiredVector(trajectory[step]);
  }

  DVec<T> weights(state_dimension);
  weights << T(20), T(20), T(10), T(2), T(2), T(50),
    T(0.2), T(0.2), T(0.2), T(1), T(1), T(2);
  DVec<T> repeated_weights(static_cast<Eigen::Index>(horizon) * state_dimension);
  for (std::size_t step = 0; step < horizon; ++step) {
    repeated_weights.segment(static_cast<Eigen::Index>(step) * state_dimension, state_dimension) =
      weights;
  }
  const DMat<T> weighted_prediction = repeated_weights.asDiagonal() * prediction;
  const DVec<T> weighted_error = repeated_weights.asDiagonal() *
    (free_prediction - desired_prediction);
  DMat<T> hessian = weighted_prediction.transpose() * weighted_prediction;
  hessian.diagonal().array() += settings_.force_regularization;
  const DVec<T> gradient_offset = weighted_prediction.transpose() * weighted_error;

  const T lipschitz = std::max(
    hessian.template selfadjointView<Eigen::Upper>().eigenvalues().maxCoeff(), T(1e-6));
  const T step_size = T(1) / lipschitz;
  DVec<T> forces = DVec<T>::Zero(input_dimension);
  for (std::size_t step = 0; step < horizon; ++step) {
    std::size_t contacts = 0;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      contacts += contact_table[step * kNumLegs + leg] != 0 ? 1U : 0U;
    }
    if (contacts > 0) {
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        if (contact_table[step * kNumLegs + leg] != 0) {
          forces[static_cast<Eigen::Index>(step * 12 + leg * 3 + 2)] =
            mass * T(9.81) / static_cast<T>(contacts);
        }
      }
    }
  }

  for (std::size_t iteration = 0; iteration < settings_.maximum_iterations; ++iteration) {
    const DVec<T> previous = forces;
    forces -= step_size * (hessian * forces + gradient_offset);
    for (std::size_t step = 0; step < horizon; ++step) {
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        Vec3<T> force = forces.template segment<3>(
          static_cast<Eigen::Index>(step * 12 + leg * 3));
        projectForce(force, contact_table[step * kNumLegs + leg] != 0);
        forces.template segment<3>(static_cast<Eigen::Index>(step * 12 + leg * 3)) = force;
      }
    }
    result.residual = (forces - previous).template lpNorm<Eigen::Infinity>();
    result.iterations = iteration + 1;
    if (result.residual <= settings_.convergence_tolerance) {
      result.converged = true;
      break;
    }
  }

  if (!forces.allFinite()) {return SolverResult<T>{};}
  result.force_trajectory.assign(forces.data(), forces.data() + forces.size());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    result.reaction_forces_world[leg] =
      forces.template segment<3>(static_cast<Eigen::Index>(leg * 3));
  }
  result.valid = true;
  return result;
}

template struct SolverSettings<float>;
template struct SolverSettings<double>;
template struct SolverResult<float>;
template struct SolverResult<double>;
template class SolverMPC<float>;
template class SolverMPC<double>;

}  // namespace mpc
