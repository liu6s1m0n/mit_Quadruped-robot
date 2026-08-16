#include "SimulationDiagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

namespace
{
constexpr std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};
}

float SimulationDiagnosticReport::rmsPositionError() const noexcept
{
  return observed_frames == 0 ? 0.0F : static_cast<float>(
    std::sqrt(squared_position_error_sum / static_cast<double>(observed_frames)));
}

float SimulationDiagnosticReport::rmsVelocityError() const noexcept
{
  return observed_frames == 0 ? 0.0F : static_cast<float>(
    std::sqrt(squared_velocity_error_sum / static_cast<double>(observed_frames)));
}

int SimulationDiagnostics::requireObject(
  const mjModel * model, int type, const char * name)
{
  const int id = mj_name2id(model, type, name);
  if (id < 0) {throw std::runtime_error(std::string("MuJoCo object not found: ") + name);}
  return id;
}

SimulationDiagnostics::SimulationDiagnostics(const mjModel * model)
: model_(model)
{
  if (model_ == nullptr) {
    throw std::invalid_argument("SimulationDiagnostics requires a MuJoCo model");
  }
  trunk_body_ = requireObject(model_, mjOBJ_BODY, "trunk");
  floor_geom_ = requireObject(model_, mjOBJ_GEOM, "floor");
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_geoms_[leg] = requireObject(model_, mjOBJ_GEOM, kLegNames[leg]);
    const std::string calf_name = std::string(kLegNames[leg]) + "_calf";
    calf_bodies_[leg] = requireObject(model_, mjOBJ_BODY, calf_name.c_str());
  }
  reset();
}

void SimulationDiagnostics::reset() noexcept
{
  position_offset_.setZero();
  position_offset_initialized_ = false;
  report_ = SimulationDiagnosticReport{};
}

Vec3<float> SimulationDiagnostics::bodyPosition(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  return Eigen::Map<const Vec3<double>>(data->xpos + 3 * trunk_body_).cast<float>();
}

Vec3<float> SimulationDiagnostics::bodyLinearVelocity(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  mjtNum velocity[6]{};
  mj_objectVelocity(model_, data, mjOBJ_BODY, trunk_body_, velocity, 0);
  return Eigen::Map<const Vec3<double>>(velocity + 3).cast<float>();
}

float SimulationDiagnostics::bodyYaw(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  const mjtNum * rotation = data->xmat + 9 * trunk_body_;
  return static_cast<float>(std::atan2(rotation[3], rotation[0]));
}

float SimulationDiagnostics::orientationError(
  const mjData * data, const Eigen::Quaternionf & estimate) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  const mjtNum * value = data->xquat + 4 * trunk_body_;
  const Eigen::Quaternionf truth(
    static_cast<float>(value[0]), static_cast<float>(value[1]),
    static_cast<float>(value[2]), static_cast<float>(value[3]));
  return Eigen::AngleAxisf(truth.conjugate() * estimate).angle();
}

bool SimulationDiagnostics::hasCalfCollision(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  for (int index = 0; index < data->ncon; ++index) {
    const mjContact & contact = data->contact[index];
    const int other = contact.geom1 == floor_geom_ ? contact.geom2 :
      (contact.geom2 == floor_geom_ ? contact.geom1 : -1);
    if (other < 0 ||
      std::find(foot_geoms_.begin(), foot_geoms_.end(), other) != foot_geoms_.end())
    {
      continue;
    }
    const int body = model_->geom_bodyid[other];
    if (std::find(calf_bodies_.begin(), calf_bodies_.end(), body) !=
      calf_bodies_.end())
    {
      return true;
    }
  }
  return false;
}

void SimulationDiagnostics::observe(
  const mjData * data, const StateEstimate<float> & estimate,
  bool control_valid)
{
  if (!estimate.valid) {
    ++report_.rejected_control_frames;
    return;
  }
  const Vec3<float> truth_position = bodyPosition(data);
  if (!position_offset_initialized_) {
    // 足端里程计无法观测绝对水平原点，因此仅校准一次x/y常量偏移。
    position_offset_ = truth_position - estimate.position_world;
    position_offset_.z() = 0.0F;
    position_offset_initialized_ = true;
  }
  const float position_error =
    (estimate.position_world + position_offset_ - truth_position).norm();
  const float velocity_error =
    (estimate.velocity_world - bodyLinearVelocity(data)).norm();
  report_.maximum_position_error =
    std::max(report_.maximum_position_error, position_error);
  report_.maximum_velocity_error =
    std::max(report_.maximum_velocity_error, velocity_error);
  report_.maximum_orientation_error = std::max(
    report_.maximum_orientation_error,
    orientationError(data, estimate.orientation_world_from_body));
  report_.maximum_absolute_pitch =
    std::max(report_.maximum_absolute_pitch, std::abs(estimate.rpy.y()));
  report_.minimum_height = std::min(report_.minimum_height, estimate.position_world.z());
  report_.squared_position_error_sum += position_error * position_error;
  report_.squared_velocity_error_sum += velocity_error * velocity_error;
  report_.calf_collision_frames += hasCalfCollision(data) ? 1U : 0U;
  report_.rejected_control_frames += control_valid ? 0U : 1U;
  ++report_.observed_frames;
}
