// ============================================================
// 四足步态时序调度器
// 保留原 MIT 固定步态库及覆盖模式，使用当前工程的统一腿序与参数接口。
// ============================================================

#include "controller/GaitScheduler.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

bool gaitTypeValueIsValid(GaitType gait) noexcept
{
  const auto value = static_cast<std::uint8_t>(gait);
  return value <= static_cast<std::uint8_t>(GaitType::TRANSITION_TO_STAND);
}

bool overrideModeIsValid(GaitOverrideMode mode) noexcept
{
  const auto value = static_cast<std::uint8_t>(mode);
  return value <=
         static_cast<std::uint8_t>(GaitOverrideMode::OVERRIDE_NATURAL_TIMING);
}

template<typename T>
bool timingIsValid(T period_time, T switching_phase) noexcept
{
  return std::isfinite(static_cast<double>(period_time)) && period_time > T(0) &&
         std::isfinite(static_cast<double>(switching_phase)) &&
         switching_phase >= T(0) && switching_phase <= T(1);
}

}  // namespace

// ---------- 参数和数据复位 ----------

template<typename T>
bool GaitSchedulerParameters<T>::isValid() const noexcept
{
  return overrideModeIsValid(override_mode) && gaitTypeValueIsValid(gait_type) &&
         timingIsValid(gait_period_time, gait_switching_phase);
}

template<typename T>
void GaitData<T>::zero()
{
  current_gait = GaitType::STAND;
  next_gait = GaitType::STAND;
  gait_name.clear();

  period_time_nominal = T(0);
  initial_phase = T(0);
  switching_phase_nominal = T(0);
  overrideable = false;

  gait_enabled.setZero();
  period_time.setZero();
  time_stance.setZero();
  time_swing.setZero();
  time_stance_remaining.setZero();
  time_swing_remaining.setZero();
  switching_phase.setZero();
  phase_variable.setZero();
  phase_offset.setZero();
  phase_scale.setZero();
  phase_stance.setZero();
  phase_swing.setZero();
  contact_state_scheduled.setZero();
  contact_state_previous.setZero();
  touchdown_scheduled.setZero();
  liftoff_scheduled.setZero();
}

// ---------- 生命周期和对外配置 ----------

template<typename T>
GaitScheduler<T>::GaitScheduler(
  const GaitSchedulerParameters<T> & parameters, T time_step)
: parameters_(parameters), dt_(time_step)
{
  if (!parameters_.isValid()) {
    throw std::invalid_argument("invalid gait scheduler parameters");
  }
  if (!std::isfinite(static_cast<double>(dt_)) || dt_ <= T(0)) {
    throw std::invalid_argument("gait scheduler time step must be finite and positive");
  }
  initialize();
}

template<typename T>
GaitScheduler<T>::GaitScheduler(T time_step)
: GaitScheduler(GaitSchedulerParameters<T>{}, time_step)
{
}

template<typename T>
void GaitScheduler<T>::initialize()
{
  gait_data.zero();
  gait_data.next_gait = GaitType::STAND;
  createGait();
  period_time_natural = gait_data.period_time_nominal;
  switching_phase_natural = gait_data.switching_phase_nominal;
  swing_time_natural = gait_data.time_swing.minCoeff();
}

template<typename T>
void GaitScheduler<T>::setParameters(
  const GaitSchedulerParameters<T> & parameters)
{
  if (!parameters.isValid()) {
    throw std::invalid_argument("invalid gait scheduler parameters");
  }
  parameters_ = parameters;
}

template<typename T>
void GaitScheduler<T>::requestGait(GaitType gait)
{
  if (!gaitTypeIsValid(gait)) {
    throw std::invalid_argument("invalid gait type");
  }
  gait_data.next_gait = gait;
}

template<typename T>
void GaitScheduler<T>::setNaturalTiming(T period_time, T switching_phase)
{
  if (!timingIsValid(period_time, switching_phase)) {
    throw std::invalid_argument("invalid natural gait timing");
  }
  period_time_natural = period_time;
  switching_phase_natural = switching_phase;
  swing_time_natural = period_time * (T(1) - switching_phase);
}

template<typename T>
bool GaitScheduler<T>::gaitTypeIsValid(GaitType gait) noexcept
{
  return gaitTypeValueIsValid(gait);
}

template<typename T>
T GaitScheduler<T>::wrapPhase(T phase)
{
  T wrapped = std::fmod(phase, T(1));
  if (wrapped < T(0)) {
    wrapped += T(1);
  }
  return wrapped;
}

// ---------- 周期推进与事件生成 ----------

template<typename T>
void GaitScheduler<T>::step()
{
  modifyGait();

  if (gait_data.current_gait != GaitType::STAND) {
    gait_data.initial_phase = wrapPhase(
      gait_data.initial_phase + dt_ / gait_data.period_time_nominal);
  }

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const Eigen::Index foot = static_cast<Eigen::Index>(index);
    gait_data.contact_state_previous(foot) =
      gait_data.contact_state_scheduled(foot);
    gait_data.touchdown_scheduled(foot) = 0;
    gait_data.liftoff_scheduled(foot) = 0;

    if (gait_data.gait_enabled(foot) == 0) {
      gait_data.phase_variable(foot) = T(0);
      gait_data.phase_stance(foot) = T(0);
      gait_data.phase_swing(foot) = T(0);
      gait_data.time_stance_remaining(foot) = T(0);
      gait_data.time_swing_remaining(foot) = T(0);
      gait_data.contact_state_scheduled(foot) = 0;
      continue;
    }

    const T phase_increment = gait_data.current_gait == GaitType::STAND ?
      T(0) : gait_data.phase_scale(foot) * dt_ / gait_data.period_time_nominal;
    gait_data.phase_variable(foot) = wrapPhase(
      gait_data.phase_variable(foot) + phase_increment);

    const T phase = gait_data.phase_variable(foot);
    const T switch_phase = gait_data.switching_phase(foot);
    const bool in_stance =
      switch_phase >= T(1) || (switch_phase > T(0) && phase <= switch_phase);

    if (in_stance) {
      gait_data.contact_state_scheduled(foot) = 1;
      gait_data.phase_stance(foot) = std::clamp(
        phase / switch_phase, T(0), T(1));
      gait_data.phase_swing(foot) = T(0);
      gait_data.time_stance_remaining(foot) =
        gait_data.period_time(foot) * (switch_phase - phase);
      gait_data.time_swing_remaining(foot) = T(0);
      gait_data.touchdown_scheduled(foot) =
        gait_data.contact_state_previous(foot) == 0 ? 1 : 0;
    } else {
      gait_data.contact_state_scheduled(foot) = 0;
      gait_data.phase_stance(foot) = T(1);
      const T swing_fraction = T(1) - switch_phase;
      gait_data.phase_swing(foot) = swing_fraction > T(0) ?
        std::clamp((phase - switch_phase) / swing_fraction, T(0), T(1)) : T(0);
      gait_data.time_stance_remaining(foot) = T(0);
      gait_data.time_swing_remaining(foot) =
        gait_data.period_time(foot) * (T(1) - phase);
      gait_data.liftoff_scheduled(foot) =
        gait_data.contact_state_previous(foot) == 1 ? 1 : 0;
    }
  }
}

// ---------- 步态选择和覆盖 ----------

template<typename T>
void GaitScheduler<T>::modifyGait()
{
  const auto select_parameter_gait = [this]() {
      if (gait_data.current_gait != parameters_.gait_type) {
        gait_data.next_gait = parameters_.gait_type;
        createGait();
      }
    };

  switch (parameters_.override_mode) {
    case GaitOverrideMode::USE_REQUESTED_GAIT:
    case GaitOverrideMode::USE_NATURAL_GAIT:
      if (gait_data.current_gait != gait_data.next_gait) {
        createGait();
      }
      break;

    case GaitOverrideMode::USE_PREDEFINED_GAIT:
      select_parameter_gait();
      break;

    case GaitOverrideMode::OVERRIDE_TIMING:
      select_parameter_gait();
      if (gait_data.overrideable &&
        (std::abs(gait_data.period_time_nominal - parameters_.gait_period_time) >
        T(0.0001) ||
        std::abs(
          gait_data.switching_phase_nominal -
          parameters_.gait_switching_phase) > T(0.0001)))
      {
        gait_data.period_time_nominal = parameters_.gait_period_time;
        gait_data.switching_phase_nominal = parameters_.gait_switching_phase;
        calculateAuxiliaryGaitData();
      }
      break;

    case GaitOverrideMode::OVERRIDE_NATURAL_TIMING:
      if (gait_data.current_gait != gait_data.next_gait) {
        createGait();
        period_time_natural = gait_data.period_time_nominal;
        switching_phase_natural = gait_data.switching_phase_nominal;
        swing_time_natural =
          period_time_natural * (T(1) - switching_phase_natural);
      } else {
        gait_data.period_time_nominal = period_time_natural;
        gait_data.switching_phase_nominal = switching_phase_natural;
        calculateAuxiliaryGaitData();
      }
      break;
  }
}

template<typename T>
void GaitScheduler<T>::configureGait(
  const char * name, T period_time, T switching_phase,
  const Eigen::Vector4i & enabled, const Vec4<T> & phase_offset,
  const Vec4<T> & phase_scale, bool can_override)
{
  gait_data.gait_name = name;
  gait_data.gait_enabled = enabled;
  gait_data.period_time_nominal = period_time;
  gait_data.initial_phase = T(0);
  gait_data.switching_phase_nominal = switching_phase;
  gait_data.phase_offset = phase_offset;
  gait_data.phase_scale = phase_scale;
  gait_data.overrideable = can_override;
}

template<typename T>
void GaitScheduler<T>::createGait()
{
  if (!gaitTypeIsValid(gait_data.next_gait)) {
    throw std::invalid_argument("invalid requested gait type");
  }

  const Eigen::Vector4i all_legs = Eigen::Vector4i::Ones();
  const Vec4<T> unit_scale = Vec4<T>::Ones();

  switch (gait_data.next_gait) {
    case GaitType::STAND:
      configureGait(
        "STAND", T(10), T(1), all_legs,
        Vec4<T>(T(0.5), T(0.5), T(0.5), T(0.5)), unit_scale, false);
      break;

    case GaitType::STAND_CYCLE:
      configureGait(
        "STAND_CYCLE", T(1), T(1), all_legs,
        Vec4<T>(T(0.5), T(0.5), T(0.5), T(0.5)), unit_scale, false);
      break;

    case GaitType::STATIC_WALK:
      configureGait(
        "STATIC_WALK", T(1.25), T(0.8), all_legs,
        Vec4<T>(T(0.25), T(0), T(0.75), T(0.5)), unit_scale, true);
      break;

    case GaitType::AMBLE:
      configureGait(
        "AMBLE", T(0.5), T(0.625), all_legs,
        Vec4<T>(T(0), T(0.5), T(0.25), T(0.75)), unit_scale, true);
      break;
    
    /*参数含义：0.5 是完整步态周期（秒），0.6 是支撑相占比。
      当前支撑时间为 0.30 秒、摆动时间为 0.20 秒。这里保持原接触时序，
      使 GaitScheduler、状态估计器和 20 Hz MPC 接触表继续严格同步；
      抬脚响应改由 swing_height_ 和行走关节 PD 提升。*/
    case GaitType::TROT_WALK:
      configureGait(
        "TROT_WALK", T(0.5), T(0.6), all_legs,
        Vec4<T>(T(0), T(0.5), T(0.5), T(0)), unit_scale, true);
      break;

    case GaitType::TROT:
      configureGait(
        "TROT", T(0.5), T(0.5), all_legs,
        Vec4<T>(T(0), T(0.5), T(0.5), T(0)), unit_scale, true);
      break;

    case GaitType::TROT_RUN:
      configureGait(
        "TROT_RUN", T(0.4), T(0.4), all_legs,
        Vec4<T>(T(0), T(0.5), T(0.5), T(0)), unit_scale, true);
      break;

    case GaitType::PACE:
      configureGait(
        "PACE", T(0.35), T(0.5), all_legs,
        Vec4<T>(T(0), T(0.5), T(0), T(0.5)), unit_scale, true);
      gait_data.initial_phase = T(0.25);
      break;

    case GaitType::BOUND:
      configureGait(
        "BOUND", T(0.4), T(0.4), all_legs,
        Vec4<T>(T(0), T(0), T(0.5), T(0.5)), unit_scale, true);
      break;

    case GaitType::ROTARY_GALLOP:
      configureGait(
        "ROTARY_GALLOP", T(0.4), T(0.2), all_legs,
        Vec4<T>(T(0), T(0.8571), T(0.3571), T(0.5)), unit_scale, true);
      break;

    case GaitType::TRAVERSE_GALLOP:
      configureGait(
        "TRAVERSE_GALLOP", T(0.5), T(0.2), all_legs,
        Vec4<T>(T(0), T(0.8571), T(0.3571), T(0.5)), unit_scale, true);
      break;

    case GaitType::PRONK:
      configureGait(
        "PRONK", T(0.5), T(0.5), all_legs, Vec4<T>::Zero(), unit_scale, true);
      break;

    case GaitType::THREE_FOOT:
      configureGait(
        "THREE_FOOT", T(0.4), T(0.666),
        Eigen::Vector4i(0, 1, 1, 1),
        Vec4<T>(T(0), T(0.666), T(0), T(0.333)),
        Vec4<T>(T(0), T(1), T(1), T(1)), true);
      break;

    case GaitType::CUSTOM:
      // 保留当前时序数据，供上层在 requestGait(CUSTOM) 前填充自定义参数。
      gait_data.gait_name = "CUSTOM";
      break;

    case GaitType::TRANSITION_TO_STAND:
      {
        const T old_period = gait_data.period_time_nominal;
        gait_data.gait_name = "TRANSITION_TO_STAND";
        gait_data.gait_enabled = all_legs;
        gait_data.period_time_nominal = T(3) * old_period;
        gait_data.initial_phase = T(0);
        gait_data.switching_phase_nominal =
          (gait_data.period_time_nominal +
          old_period * (gait_data.switching_phase_nominal - T(1))) /
          gait_data.period_time_nominal;
        for (std::size_t index = 0; index < kNumLegs; ++index) {
          const Eigen::Index foot = static_cast<Eigen::Index>(index);
          gait_data.phase_offset(foot) =
            (gait_data.period_time_nominal +
            old_period * (gait_data.phase_variable(foot) - T(1))) /
            gait_data.period_time_nominal;
        }
        gait_data.phase_scale = unit_scale;
        gait_data.overrideable = false;
        break;
      }
  }

  gait_data.current_gait = gait_data.next_gait;
  calculateAuxiliaryGaitData();
}

// ---------- 辅助时序计算 ----------

template<typename T>
void GaitScheduler<T>::calculateAuxiliaryGaitData()
{
  if (!timingIsValid(
      gait_data.period_time_nominal, gait_data.switching_phase_nominal))
  {
    throw std::invalid_argument("invalid active gait timing");
  }

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const Eigen::Index foot = static_cast<Eigen::Index>(index);
    if (gait_data.gait_enabled(foot) != 0) {
      const T scale = gait_data.phase_scale(foot);
      if (!std::isfinite(static_cast<double>(scale)) || scale <= T(0)) {
        throw std::invalid_argument("enabled gait leg must have positive phase scale");
      }
      gait_data.period_time(foot) = gait_data.period_time_nominal / scale;
      gait_data.switching_phase(foot) = gait_data.switching_phase_nominal;
      gait_data.phase_variable(foot) = wrapPhase(
        gait_data.initial_phase + gait_data.phase_offset(foot));
      gait_data.time_stance(foot) =
        gait_data.period_time(foot) * gait_data.switching_phase(foot);
      gait_data.time_swing(foot) =
        gait_data.period_time(foot) * (T(1) - gait_data.switching_phase(foot));
    } else {
      gait_data.period_time(foot) = T(0);
      gait_data.switching_phase(foot) = T(0);
      gait_data.phase_variable(foot) = T(0);
      gait_data.time_stance(foot) = T(0);
      gait_data.time_swing(foot) = T(0);
    }
    gait_data.time_stance_remaining(foot) = T(0);
    gait_data.time_swing_remaining(foot) = T(0);
    gait_data.phase_stance(foot) = T(0);
    gait_data.phase_swing(foot) = T(0);
  }
}

// ---------- 调试输出 ----------

template<typename T>
void GaitScheduler<T>::printGaitInfo() const
{
  std::cout << "[GAIT] " << gait_data.gait_name << '\n';
  std::cout << "  enabled: " << gait_data.gait_enabled.transpose() << '\n';
  std::cout << "  period(s): " << gait_data.period_time.transpose() << '\n';
  std::cout << "  contact: " << gait_data.contact_state_scheduled.transpose() << '\n';
  std::cout << "  phase: " << gait_data.phase_variable.transpose() << '\n';
  std::cout << "  stance remaining(s): " <<
    gait_data.time_stance_remaining.transpose() << '\n';
  std::cout << "  swing remaining(s): " <<
    gait_data.time_swing_remaining.transpose() << std::endl;
}

template struct GaitSchedulerParameters<float>;
template struct GaitSchedulerParameters<double>;
template struct GaitData<float>;
template struct GaitData<double>;
template class GaitScheduler<float>;
template class GaitScheduler<double>;
