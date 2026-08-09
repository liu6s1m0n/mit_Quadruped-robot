#include "SimulationBridge.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
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
#include <sys/socket.h>
#include <unistd.h>

#include "RobotRunner.hpp"
#include "StandingHeightIpc.hpp"

namespace mj = ::mujoco;

namespace
{

using MjuiAddFunction = void (*)(mjUI *, const mjuiDef *);
double * g_standing_height_slider = nullptr;

MjuiAddFunction originalMjuiAdd()
{
  static const MjuiAddFunction function = []() {
      void * symbol = ::dlsym(RTLD_NEXT, "mjui_add");
      MjuiAddFunction result = nullptr;
      static_assert(sizeof(result) == sizeof(symbol));
      std::memcpy(&result, &symbol, sizeof(result));
      return result;
    }();
  return function;
}

}  // namespace

// Simulate constructs its UI inside RenderLoop, so append the project control
// while its standard Simulation section is being built on the render thread.
extern "C" void mjui_add(mjUI * ui, const mjuiDef * definition)
{
  const MjuiAddFunction add = originalMjuiAdd();
  if (add == nullptr) {
    std::fprintf(stderr, "unable to resolve MuJoCo mjui_add\n");
    std::abort();
  }
  add(ui, definition);

  if (g_standing_height_slider == nullptr || definition == nullptr ||
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
    {mjITEM_END, "", 0, nullptr, "", 0}
  };
  add(ui, height_controls);
}

namespace
{

class StandingHeightReceiver
{
public:
  StandingHeightReceiver()
  : descriptor_(::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0))
  {
    if (descriptor_ < 0) {
      throw std::runtime_error(
              std::string("unable to create height-command receiver: ") +
              std::strerror(errno));
    }
    const sockaddr_un address = standing_height_ipc::socketAddress();
    if (::bind(
        descriptor_, reinterpret_cast<const sockaddr *>(&address),
        standing_height_ipc::socketAddressLength()) < 0)
    {
      const std::string message = std::string("unable to bind height-command receiver: ") +
        std::strerror(errno);
      ::close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error(message);
    }
  }

  ~StandingHeightReceiver()
  {
    if (descriptor_ >= 0) {::close(descriptor_);}
  }

  StandingHeightReceiver(const StandingHeightReceiver &) = delete;
  StandingHeightReceiver & operator=(const StandingHeightReceiver &) = delete;

  void receive(std::atomic<float> & standing_height) const
  {
    standing_height_ipc::Command command;
    while (true) {
      const ssize_t received = ::recv(descriptor_, &command, sizeof(command), 0);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {return;}
        if (errno == EINTR) {continue;}
        throw std::runtime_error(
                std::string("unable to receive height command: ") +
                std::strerror(errno));
      }
      if (received != static_cast<ssize_t>(sizeof(command)) ||
        command.magic != standing_height_ipc::kCommandMagic ||
        !std::isfinite(command.height) ||
        command.height < standing_height_ipc::kMinimumHeight ||
        command.height > standing_height_ipc::kMaximumHeight)
      {
        std::fprintf(stderr, "ignored an invalid standing-height command\n");
        continue;
      }
      standing_height.store(command.height);
      std::printf("Standing-height target set to %.3f m\n", command.height);
    }
  }

private:
  int descriptor_ = -1;
};

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
  const float slider_value = std::clamp(
    static_cast<float>(slider_height),
    standing_height_ipc::kMinimumHeight,
    standing_height_ipc::kMaximumHeight);
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

void writeCommands(
  const RobotRunner & runner, const JointAddresses & addresses,
  const mjModel * model, mjData * data)
{
  mju_zero(data->qfrc_applied, model->nv);
  const auto & commands = runner.jointCommands();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = commands[leg];
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto & address = addresses[leg][joint];
      // Neutralize the MJCF position servo; the complete WBC torque, including
      // its joint-space PD terms, is applied directly as generalized force.
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
  std::exception_ptr & failure)
{
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
    mj_forward(model, data);
    const JointAddresses addresses = findJointAddresses(model);
    RobotRunner runner(model, data);
    StandingHeightReceiver height_receiver;

    simulation.Load(model, data, scene_path.c_str());
    std::printf(
      "MuJoCo GO1 control started: dt=%.4f s, default mode=BalanceStand/WBC\n",
      model->opt.timestep);

    bool controller_ready = false;
    std::size_t consecutive_failures = 0;
    double previous_simulation_time = data->time;
    float previous_slider_height = standing_height.load();
    while (!simulation.exitrequest.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      std::unique_lock<std::recursive_mutex> lock(simulation.mtx);
      if (data->time + 0.5 * model->opt.timestep < previous_simulation_time) {
        runner.reset();
        controller_ready = false;
        consecutive_failures = 0;
        std::printf(
          "MuJoCo reset detected: controller reset; physical state left untouched\n");
      }
      previous_simulation_time = data->time;
      height_receiver.receive(standing_height);
      const float commanded_height = synchronizeStandingHeight(
        standing_height_slider, previous_slider_height, standing_height);
      if (simulation.run) {
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
        writeCommands(runner, addresses, model, data);
        mj_step(model, data);
      } else {
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

SimulationBridge::SimulationBridge(std::string scene_path)
: scene_path_(std::move(scene_path))
{
  if (scene_path_.empty()) {throw std::invalid_argument("scene path must not be empty");}
}

void SimulationBridge::setStandingHeight(float height)
{
  if (!std::isfinite(height) || height < standing_height_ipc::kMinimumHeight ||
    height > standing_height_ipc::kMaximumHeight)
  {
    throw std::invalid_argument("standing height must be within [0.18, 0.34] m");
  }
  standing_height_.store(height);
}

int SimulationBridge::run()
{
  if (mjVERSION_HEADER != mj_version()) {
    throw std::runtime_error("MuJoCo headers and runtime library versions differ");
  }

  mjvCamera camera;
  mjv_defaultCamera(&camera);
  mjvOption options;
  mjv_defaultOption(&options);
  mjvPerturb perturbation;
  mjv_defaultPerturb(&perturbation);
  auto glfw_adapter = std::make_unique<mj::GlfwAdapter>();
  g_standing_height_slider = &standing_height_slider_;
  auto simulation = std::make_unique<mj::Simulate>(
    std::move(glfw_adapter), &camera, &options, &perturbation, false);
  simulation->run = true;

  std::exception_ptr failure;
  std::thread physics(
    runPhysics, std::ref(*simulation), std::cref(scene_path_),
    std::ref(standing_height_), std::ref(standing_height_slider_),
    std::ref(failure));
  simulation->RenderLoop();
  physics.join();
  if (failure != nullptr) {std::rethrow_exception(failure);}
  return 0;
}
