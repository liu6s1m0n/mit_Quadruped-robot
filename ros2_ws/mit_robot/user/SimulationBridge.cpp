// MuJoCo 适配层：负责原生 UI、模型地址映射、控制力矩写入和物理线程。
#include "SimulationBridge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <glfw_adapter.h>
#include <mujoco/mujoco.h>
#include <simulate.h>

#include <dlfcn.h>
#include "RobotRunner.hpp"
#include "RobotMiddlewareInterface.hpp"
#include "SimulationDiagnostics.hpp"

namespace mj = ::mujoco;

namespace
{

using MjuiAddFunction = void (*)(mjUI *, const mjuiDef *);
using MjuiEventFunction = mjuiItem * (*)(mjUI *, mjuiState *, const mjrContext *);
double * g_standing_height_slider = nullptr;
std::array<int, 6> * g_direction_toggles = nullptr;
std::atomic<bool> * g_front_jump_requested = nullptr;

enum DirectionIndex : std::size_t
{
  kForwardSlow = 0,
  kForwardFast = 1,
  kBackward = 2,
  kLeft = 3,
  kRight = 4,
  kRotate = 5
};

MjuiAddFunction originalMjuiAdd()
{
  // 取得 MuJoCo 原始函数，避免下面的同名扩展再次调用自身而递归。
  static const MjuiAddFunction function = []() {
      void * symbol = ::dlsym(RTLD_NEXT, "mjui_add");
      MjuiAddFunction result = nullptr;
      static_assert(sizeof(result) == sizeof(symbol));
      std::memcpy(&result, &symbol, sizeof(result));
      return result;
    }();
  return function;
}

MjuiEventFunction originalMjuiEvent()
{
  static const MjuiEventFunction function = []() {
      void * symbol = ::dlsym(RTLD_NEXT, "mjui_event");
      MjuiEventFunction result = nullptr;
      static_assert(sizeof(result) == sizeof(symbol));
      std::memcpy(&result, &symbol, sizeof(result));
      return result;
    }();
  return function;
}

}  // namespace

// Simulate 在 RenderLoop 内部创建 UI。这里先调用原函数建立标准 Simulation 区域，
// 再在同一渲染线程追加高度滑块，避免跨线程调用 OpenGL UI 接口。
extern "C" void mjui_add(mjUI * ui, const mjuiDef * definition)
{
  const MjuiAddFunction add = originalMjuiAdd();
  if (add == nullptr) {
    std::fprintf(stderr, "unable to resolve MuJoCo mjui_add\n");
    std::abort();
  }
  add(ui, definition);

  if (g_standing_height_slider == nullptr || g_direction_toggles == nullptr ||
    definition == nullptr ||
    definition[0].type != mjITEM_SECTION ||
    std::strcmp(definition[0].name, "Simulation") != 0)
  {
    return;
  }

  const mjuiDef height_controls[] = {
    {mjITEM_SEPARATOR, "Robot controller", 1, nullptr, "", 0},
    {
      mjITEM_SLIDERNUM, "Height (m)", 2,
      g_standing_height_slider, "0.18 0.34", 0
    },
    {mjITEM_CHECKINT, "Forward slow", 2, &(*g_direction_toggles)[kForwardSlow], "", 0},
    {mjITEM_CHECKINT, "Forward fast", 2, &(*g_direction_toggles)[kForwardFast], "", 0},
    {mjITEM_CHECKINT, "Backward", 2, &(*g_direction_toggles)[kBackward], "", 0},
    {mjITEM_CHECKINT, "Left", 2, &(*g_direction_toggles)[kLeft], "", 0},
    {mjITEM_CHECKINT, "Right", 2, &(*g_direction_toggles)[kRight], "", 0},
    {mjITEM_CHECKINT, "Rotate CCW", 2, &(*g_direction_toggles)[kRotate], "", 0},
    {mjITEM_BUTTON, "Jump forward", 2, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}
  };
  add(ui, height_controls);
}

// Simulate consumes button events internally, so intercept the returned item
// and hand a lock-free one-shot request to the physics thread.
extern "C" mjuiItem * mjui_event(
  mjUI * ui, mjuiState * state, const mjrContext * context)
{
  const MjuiEventFunction event = originalMjuiEvent();
  if (event == nullptr) {
    std::fprintf(stderr, "unable to resolve MuJoCo mjui_event\n");
    std::abort();
  }
  mjuiItem * changed = event(ui, state, context);
  if (changed != nullptr && changed->type == mjITEM_BUTTON &&
    std::strcmp(changed->name, "Jump forward") == 0 &&
    g_front_jump_requested != nullptr)
  {
    g_front_jump_requested->store(true);
  }
  return changed;
}

namespace
{

constexpr std::array<LegId, kNumLegs> kLegOrder{
  LegId::FR, LegId::FL, LegId::RR, LegId::RL};
constexpr std::array<std::array<const char *, kJointsPerLeg>, kNumLegs>
kJointNames{{
  {{"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}},
  {{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}},
  {{"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}},
  {{"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}}
}};
constexpr std::array<std::array<const char *, kJointsPerLeg>, kNumLegs>
kActuatorNames{{
  {{"FR_hip", "FR_thigh", "FR_calf"}},
  {{"FL_hip", "FL_thigh", "FL_calf"}},
  {{"RR_hip", "RR_thigh", "RR_calf"}},
  {{"RL_hip", "RL_thigh", "RL_calf"}}
}};

struct JointAddress
{
  int qpos = -1;
  int dof = -1;
  int actuator = -1;
};

using JointAddresses =
  std::array<std::array<JointAddress, kJointsPerLeg>, kNumLegs>;

JointAddresses findJointAddresses(const mjModel * model)
{
  // 不能假定 XML 中关节编号连续，因此按名字查询 qpos、qvel 和 actuator 地址。
  JointAddresses result{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int joint_id = mj_name2id(model, mjOBJ_JOINT, kJointNames[leg][joint]);
      const int actuator_id =
        mj_name2id(model, mjOBJ_ACTUATOR, kActuatorNames[leg][joint]);
      if (joint_id < 0 || actuator_id < 0) {
        throw std::runtime_error("MuJoCo joint or actuator mapping is incomplete");
      }
      result[leg][joint] = JointAddress{
        model->jnt_qposadr[joint_id], model->jnt_dofadr[joint_id], actuator_id};
    }
  }
  return result;
}

double clampActuatorForce(const mjModel * model, int actuator, double force)
{
  if (model->actuator_forcelimited[actuator] == 0) {return force;}
  const double minimum = model->actuator_forcerange[2 * actuator];
  const double maximum = model->actuator_forcerange[2 * actuator + 1];
  return std::clamp(force, minimum, maximum);
}

float synchronizeStandingHeight(
  double & slider_height, float & previous_slider_height,
  std::atomic<float> & requested_height)
{
  // 滑块变化时以 GUI 为准；否则把 ROS 服务写入的原子目标同步回滑块。
  const float slider_value = std::clamp(
    static_cast<float>(slider_height),
    RobotRunner::minimumStandingHeight(),
    RobotRunner::maximumStandingHeight());
  if (std::abs(slider_value - previous_slider_height) > 1.0e-6F) {
    requested_height.store(slider_value);
    slider_height = slider_value;
  } else {
    const float external_value = requested_height.load();
    if (std::abs(external_value - previous_slider_height) > 1.0e-6F) {
      slider_height = external_value;
    }
  }

  previous_slider_height = static_cast<float>(slider_height);
  return previous_slider_height;
}

RosContactSnapshot readContacts(const mjModel * model, const mjData * data)
{
  RosContactSnapshot result;
  if (model == nullptr || data == nullptr) {return result;}
  const int floor = mj_name2id(model, mjOBJ_GEOM, "floor");
  if (floor < 0) {return result;}
  std::array<int, kNumLegs> feet{};
  constexpr std::array<const char *, kNumLegs> names{{"FR", "FL", "RR", "RL"}};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    feet[leg] = mj_name2id(model, mjOBJ_GEOM, names[leg]);
    if (feet[leg] < 0) {return result;}
  }
  for (int index = 0; index < data->ncon; ++index) {
    const mjContact & contact = data->contact[index];
    const int other = contact.geom1 == floor ? contact.geom2 :
      (contact.geom2 == floor ? contact.geom1 : -1);
    if (other < 0) {continue;}
    const auto iterator = std::find(feet.begin(), feet.end(), other);
    if (iterator == feet.end()) {continue;}
    const std::size_t leg = static_cast<std::size_t>(iterator - feet.begin());
    mjtNum force[6]{};
    mj_contactForce(model, data, index, force);
    result.contact[leg] = true;
    result.probability[leg] = 1.0;
    result.normal_force[leg] += std::abs(static_cast<double>(force[0]));
  }
  result.valid = true;
  return result;
}

void readJointState(
  const JointAddresses & addresses, const mjData * data,
  std::array<double, RobotMiddlewareInterface::kJointCount> & position,
  std::array<double, RobotMiddlewareInterface::kJointCount> & velocity,
  std::array<double, RobotMiddlewareInterface::kJointCount> & effort)
{
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::size_t index = leg * kJointsPerLeg + joint;
      const JointAddress & address = addresses[leg][joint];
      position[index] = data->qpos[address.qpos];
      velocity[index] = data->qvel[address.dof];
      effort[index] = data->qfrc_actuator[address.dof];
    }
  }
}

void writeCommands(
  const RobotRunner & runner, const JointAddresses & addresses,
  const mjModel * model, mjData * data)
{
  // qfrc_applied 每帧都必须清零，否则上一帧力矩会继续叠加。
  mju_zero(data->qfrc_applied, model->nv);
  const auto & commands = runner.jointCommands();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = commands[leg];
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto & address = addresses[leg][joint];
      // MJCF 自带位置伺服器在这里被置为“保持当前位置”，避免它和 WBC 重复控制。
      // 完整命令 = WBC 前馈力矩 + 关节位置 PD + 关节速度 PD。
      data->ctrl[address.actuator] = data->qpos[address.qpos];
      if (!command.enabled) {continue;}
      const Eigen::Index index = static_cast<Eigen::Index>(joint);
      const double torque = command.torque_feedforward[index] +
        command.kp[index] * (command.position_desired[index] - data->qpos[address.qpos]) +
        command.kd[index] * (command.velocity_desired[index] - data->qvel[address.dof]);
      data->qfrc_applied[address.dof] =
        clampActuatorForce(model, address.actuator, torque);
    }
  }
}

void runPhysics(
  mj::Simulate & simulation, const std::string & scene_path,
  std::atomic<float> & standing_height, double & standing_height_slider,
  std::atomic<bool> & front_jump_requested,
  std::array<int, 6> & direction_toggles, float slow_walking_forward_speed,
  float fast_walking_forward_speed, float walking_backward_speed,
  float walking_lateral_speed, float turning_yaw_rate,
  RobotMiddlewareInterface * middleware,
  std::exception_ptr & failure)
{
  // 该函数运行在物理线程；渲染线程通过 simulation.mtx 与它共享 mjData。
  mjModel * model = nullptr;
  mjData * data = nullptr;
  try {
    char error[1024]{};
    model = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
    if (model == nullptr) {
      throw std::runtime_error(std::string("unable to load MuJoCo scene: ") + error);
    }
    data = mj_makeData(model);
    if (data == nullptr) {throw std::runtime_error("unable to create MuJoCo data");}
    const int home_keyframe = mj_name2id(model, mjOBJ_KEY, "home");
    if (home_keyframe < 0) {
      throw std::runtime_error("MuJoCo model is missing the required home keyframe");
    }
    // mj_makeData 使用模型默认 qpos（腿关节为零），并不是 GO1 的站立姿态。
    // 控制器启动前先加载 home，避免机器人在关节初始化阶段从高处坠落并后仰。
    mj_resetDataKeyframe(model, data, home_keyframe);
    mj_forward(model, data);
    const JointAddresses addresses = findJointAddresses(model);
    RobotRunner runner(model, data);
    SimulationDiagnostics diagnostics(model);

    simulation.Load(model, data, scene_path.c_str());
    std::printf(
      "MuJoCo GO1 control started: dt=%.4f s, slow/fast=%.2f/%.2f m/s, "
      "backward=%.2f m/s, lateral=%.2f m/s, turning=%.2f rad/s, "
      "default mode=BalanceStand/WBC\n",
      model->opt.timestep, slow_walking_forward_speed, fast_walking_forward_speed,
      walking_backward_speed, walking_lateral_speed, turning_yaw_rate);

    bool controller_ready = false;
    std::size_t consecutive_failures = 0;
    double previous_simulation_time = data->time;
    float previous_slider_height = standing_height.load();
    std::array<int, 6> previous_direction_toggles{};
    int previous_direction = -2;
    bool ros_velocity_was_active = false;
    while (!simulation.exitrequest.load()) {
      if (middleware != nullptr && !middleware->middlewareOk()) {
        simulation.exitrequest.store(1);
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      std::unique_lock<std::recursive_mutex> lock(simulation.mtx);
      // MuJoCo Reset 会让仿真时间回退，据此重置估计器和 FSM 内部历史。
      if (data->time + 0.5 * model->opt.timestep < previous_simulation_time) {
        // UI Reset 默认会回到零关节角；统一恢复为与控制器参数一致的 home 站姿。
        mj_resetDataKeyframe(model, data, home_keyframe);
        mj_forward(model, data);
        runner.reset();
        diagnostics.reset();
        controller_ready = false;
        consecutive_failures = 0;
        previous_direction = -2;
        std::printf(
          "MuJoCo reset detected: robot and controller restored to home posture\n");
      }
      previous_simulation_time = data->time;
      if (middleware != nullptr) {
        float requested_height = 0.0F;
        if (middleware->takeStandingHeight(requested_height)) {
          standing_height.store(requested_height);
        }
      }
      const float commanded_height = synchronizeStandingHeight(
        standing_height_slider, previous_slider_height, standing_height);
      // 六个运动开关互斥：新打开的开关取得控制权，关闭当前开关则回到站立。
      int requested_direction = -1;
      for (std::size_t index = 0; index < direction_toggles.size(); ++index) {
        direction_toggles[index] = direction_toggles[index] == 0 ? 0 : 1;
        if (direction_toggles[index] != 0 &&
          previous_direction_toggles[index] == 0)
        {
          requested_direction = static_cast<int>(index);
        }
      }
      if (requested_direction < 0) {
        for (std::size_t index = 0; index < direction_toggles.size(); ++index) {
          if (direction_toggles[index] != 0) {
            requested_direction = static_cast<int>(index);
            break;
          }
        }
      }
      if (requested_direction >= 0) {
        for (std::size_t index = 0; index < direction_toggles.size(); ++index) {
          direction_toggles[index] =
            static_cast<int>(index) == requested_direction ? 1 : 0;
        }
      }
      previous_direction_toggles = direction_toggles;

      const RosVelocityCommand ros_velocity = middleware == nullptr ?
        RosVelocityCommand{} : middleware->velocityCommand();
      if (ros_velocity.active) {
        runner.setLocomotionVelocityCommand(
          ros_velocity.forward, ros_velocity.lateral, ros_velocity.yaw_rate);
        if (middleware->cmdVelActivatesLocomotion() && !ros_velocity_was_active) {
          runner.setControlMode(ControlMode::Locomotion);
        }
        ros_velocity_was_active = true;
      } else if (ros_velocity_was_active) {
        runner.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
        if (middleware->cmdVelActivatesLocomotion()) {
          runner.setControlMode(ControlMode::BalanceStand);
        }
        ros_velocity_was_active = false;
        previous_direction = -2;
      }

      // A fresh ROS velocity command has priority over native UI direction toggles.
      if (!ros_velocity_was_active && requested_direction != previous_direction) {
        float forward_velocity = 0.0F;
        float lateral_velocity = 0.0F;
        float yaw_rate = 0.0F;
        const char * direction_name = "Stand";
        switch (requested_direction) {
          case kForwardSlow:
            forward_velocity = slow_walking_forward_speed;
            direction_name = "Forward slow";
            break;
          case kForwardFast:
            forward_velocity = fast_walking_forward_speed;
            direction_name = "Forward fast";
            break;
          case kBackward:
            forward_velocity = -walking_backward_speed;
            direction_name = "Backward";
            break;
          case kLeft:
            lateral_velocity = walking_lateral_speed;
            direction_name = "Left";
            break;
          case kRight:
            lateral_velocity = -walking_lateral_speed;
            direction_name = "Right";
            break;
          case kRotate:
            yaw_rate = turning_yaw_rate;
            direction_name = "Rotate CCW";
            break;
          default:
            break;
        }
        runner.setLocomotionVelocityCommand(
          forward_velocity, lateral_velocity, yaw_rate);
        runner.setControlMode(
          requested_direction < 0 ?
          ControlMode::BalanceStand : ControlMode::Locomotion);
        // 每次切换方向重新开始一段统计，方便直接观察该命令下的估计误差、
        // 俯仰和小腿碰地情况，而不是被上一方向的累计峰值污染。
        diagnostics.reset();
        std::printf("Robot direction set to %s\n", direction_name);
        previous_direction = requested_direction;
      }
      if (middleware != nullptr) {
        const std::optional<ControlMode> requested_mode =
          middleware->takeControlModeRequest();
        if (requested_mode.has_value()) {
          if (*requested_mode == ControlMode::FrontJump) {
            front_jump_requested.store(true);
          } else {
            runner.setControlMode(*requested_mode);
          }
        }
      }
      if (simulation.run) {
        if (front_jump_requested.exchange(false)) {
          if (runner.requestFrontJump()) {
            for (int & toggle : direction_toggles) {
              toggle = 0;
            }
            previous_direction_toggles = direction_toggles;
            previous_direction = -1;
            diagnostics.reset();
            std::printf("Front jump accepted\n");
          } else {
            std::printf("Front jump ignored: robot must be stable in BalanceStand\n");
          }
        }
        // 控制计算使用当前传感器状态，随后写力矩，最后推进一个物理时间步。
        runner.setStandingHeight(commanded_height);
        const bool control_valid = runner.run();
        if (control_valid) {
          consecutive_failures = 0;
          if (!controller_ready) {
            controller_ready = true;
            std::printf("RobotRunner control pipeline is producing valid commands\n");
          }
        } else if (++consecutive_failures % 500 == 0) {
          std::fprintf(
            stderr, "RobotRunner has rejected %zu consecutive control frames\n",
            consecutive_failures);
        }
        diagnostics.observe(data, runner.stateEstimate(), control_valid);
        const auto & diagnostic_report = diagnostics.report();
        if (middleware != nullptr) {
          std::array<double, RobotMiddlewareInterface::kJointCount> joint_position{};
          std::array<double, RobotMiddlewareInterface::kJointCount> joint_velocity{};
          std::array<double, RobotMiddlewareInterface::kJointCount> joint_effort{};
          readJointState(
            addresses, data, joint_position, joint_velocity, joint_effort);
          middleware->setCurrentMode(runner.currentControlMode());
          middleware->publishClock(data->time);
          middleware->publishState(
            data->time, runner.stateEstimate(), joint_position, joint_velocity,
            joint_effort, readContacts(model, data), diagnostic_report,
            control_valid);
        }
        if (diagnostic_report.observed_frames != 0 &&
          diagnostic_report.observed_frames % 500 == 0)
        {
          // 500 Hz下每秒输出一次正式运行诊断。这里只报告，不改变控制状态。
          std::printf(
            "Runtime diagnostics: pos_rms=%.4f m, vel_rms=%.4f m/s, "
            "pitch_max=%.3f rad, height_min=%.3f m, calf_contacts=%zu, "
            "rejected=%zu\n",
            diagnostic_report.rmsPositionError(),
            diagnostic_report.rmsVelocityError(),
            diagnostic_report.maximum_absolute_pitch,
            diagnostic_report.minimum_height,
            diagnostic_report.calf_collision_frames,
            diagnostic_report.rejected_control_frames);
        }
        writeCommands(runner, addresses, model, data);
        mj_step(model, data);
      } else {
        // 暂停时只刷新运动学量，不推进时间，也不运行控制器。
        mj_forward(model, data);
      }
    }
  } catch (...) {
    failure = std::current_exception();
    simulation.exitrequest.store(1);
  }

  if (data != nullptr) {mj_deleteData(data);}
  if (model != nullptr) {mj_deleteModel(model);}
}

}  // namespace

SimulationBridge::SimulationBridge(
  std::string scene_path, RobotMiddlewareInterface * middleware)
: scene_path_(std::move(scene_path)), middleware_(middleware)
{
  if (scene_path_.empty()) {throw std::invalid_argument("scene path must not be empty");}
}

void SimulationBridge::setStandingHeight(float height)
{
  if (!std::isfinite(height) || height < RobotRunner::minimumStandingHeight() ||
    height > RobotRunner::maximumStandingHeight())
  {
    throw std::invalid_argument("standing height must be within [0.18, 0.34] m");
  }
  standing_height_.store(height);
}

void SimulationBridge::setWalkingBackwardSpeed(float speed)
{
  if (!std::isfinite(speed) || speed < 0.0F || speed > 0.6F) {
    throw std::invalid_argument("backward speed must be within [0, 0.6] m/s");
  }
  walking_backward_speed_ = speed;
}

void SimulationBridge::setSlowWalkingForwardSpeed(float speed)
{
  if (!std::isfinite(speed) || speed <= 0.0F || speed > 0.6F) {
    throw std::invalid_argument("slow walking speed must be within (0, 0.6] m/s");
  }
  slow_walking_forward_speed_ = speed;
}

void SimulationBridge::setFastWalkingForwardSpeed(float speed)
{
  if (!std::isfinite(speed) || speed <= 0.0F || speed > 0.6F) {
    throw std::invalid_argument("fast walking speed must be within (0, 0.6] m/s");
  }
  fast_walking_forward_speed_ = speed;
}

void SimulationBridge::setWalkingLateralSpeed(float speed)
{
  if (!std::isfinite(speed) || speed < 0.0F || speed > 0.6F) {
    throw std::invalid_argument("lateral speed must be within [0, 0.6] m/s");
  }
  walking_lateral_speed_ = speed;
}

void SimulationBridge::setTurningYawRate(float yaw_rate)
{
  if (!std::isfinite(yaw_rate) || yaw_rate < 0.0F || yaw_rate > 1.5F) {
    throw std::invalid_argument("turning yaw rate must be within [0, 1.5] rad/s");
  }
  turning_yaw_rate_ = yaw_rate;
}

int SimulationBridge::run()
{
  if (mjVERSION_HEADER != mj_version()) {
    throw std::runtime_error("MuJoCo headers and runtime library versions differ");
  }
  if (slow_walking_forward_speed_ >= fast_walking_forward_speed_) {
    throw std::runtime_error("slow walking speed must be lower than fast walking speed");
  }

  mjvCamera camera;
  mjv_defaultCamera(&camera);
  mjvOption options;
  mjv_defaultOption(&options);
  mjvPerturb perturbation;
  mjv_defaultPerturb(&perturbation);
  auto glfw_adapter = std::make_unique<mj::GlfwAdapter>();
  g_standing_height_slider = &standing_height_slider_;
  g_direction_toggles = &direction_toggles_;
  g_front_jump_requested = &front_jump_requested_;
  auto simulation = std::make_unique<mj::Simulate>(
    std::move(glfw_adapter), &camera, &options, &perturbation, false);
  simulation->run = true;

  std::exception_ptr failure;
  std::thread physics(
    runPhysics, std::ref(*simulation), std::cref(scene_path_),
    std::ref(standing_height_), std::ref(standing_height_slider_),
    std::ref(front_jump_requested_),
    std::ref(direction_toggles_), slow_walking_forward_speed_,
    fast_walking_forward_speed_, walking_backward_speed_, walking_lateral_speed_,
    turning_yaw_rate_, middleware_, std::ref(failure));
  simulation->RenderLoop();
  physics.join();
  g_standing_height_slider = nullptr;
  g_direction_toggles = nullptr;
  g_front_jump_requested = nullptr;
  if (failure != nullptr) {std::rethrow_exception(failure);}
  return 0;
}
