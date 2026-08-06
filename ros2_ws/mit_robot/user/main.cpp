// ============================================================
// 用户示例程序（GUI 版）
// 打开 MuJoCo 内置窗口显示 Unitree GO1，通过现有 SimLeg、LegController
// 和 JointCommand 接口控制机器人保持站立，同时实时打印 IMU 数据。
//
// 注意：本程序链接 /usr/local 完整版 MuJoCo(3.11)，复用其 Simulate
//       图形界面（libsimulate.a + GLFW）；因此不再链接 mymit_robot_model，
//       而是直接编译 src/sensor/imu.cpp，避免同时加载两套 libmujoco。
// ============================================================

#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <simulate.h>
#include <glfw_adapter.h>

#include "controller/leg_controller.hpp"
#include "model/quadruped.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

namespace mj = ::mujoco;

namespace
{

// 由物理线程持有，供仿真与读取共用
mjModel * g_model = nullptr;
mjData * g_data = nullptr;

// 四条腿在控制器数组、MJCF actuator 和打印输出中统一使用这个顺序。
constexpr std::array<LegId, kNumLegs> kLegOrder{
  LegId::FR, LegId::FL, LegId::RR, LegId::RL};

// 每条腿三个位置执行器的 MuJoCo 名称，关节顺序固定为 Hip、thigh、calf。
const std::array<std::array<const char *, kJointsPerLeg>, kNumLegs> kActuatorNames{{
  {{"FR_hip", "FR_thigh", "FR_calf"}},
  {{"FL_hip", "FL_thigh", "FL_calf"}},
  {{"RR_hip", "RR_thigh", "RR_calf"}},
  {{"RL_hip", "RL_thigh", "RL_calf"}}
}};

using LegSensorArray = std::array<std::unique_ptr<LegSensor>, kNumLegs>;
using ActuatorAddressArray =
  std::array<std::array<int, kJointsPerLeg>, kNumLegs>;

// 构造四条仿真腿的数据源。每个 SimLeg 在构造时查找并缓存本腿三个关节地址。
LegSensorArray makeSimLegs(const mjModel * model, const mjData * data)
{
  LegSensorArray legs;
  for (std::size_t index = 0; index < kNumLegs; ++index) {
    legs[index] = makeLeg(LegSource::SIMULATOR, kLegOrder[index], model, data);
  }
  return legs;
}

// 按名称查找 12 个 actuator，并缓存它们在 mjData::ctrl 中的地址。
// 不直接假定 actuator ID 就是 ctrl 下标，以便以后模型中加入其他执行器。
ActuatorAddressArray findActuatorAddresses(const mjModel * model)
{
  ActuatorAddressArray addresses{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const char * name = kActuatorNames[leg][joint];
      const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, name);
      if (actuator_id < 0) {
        throw std::invalid_argument("MuJoCo actuator not found: " + std::string(name));
      }

      const int control_address = model->actuator_ctrladr[actuator_id];
      if (control_address < 0 || control_address >= model->nu) {
        throw std::invalid_argument(
                "MuJoCo actuator has invalid ctrl address: " + std::string(name));
      }
      addresses[leg][joint] = control_address;
    }
  }
  return addresses;
}

// 设置四条腿的站立目标。home_position 来自当前 Quadruped 机型参数，
// GO1 对应 [Hip, thigh, calf] = [0, 0.9, -1.8] rad。
void configureStandingCommand(LegController<float> & controller)
{
  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const LegId leg_id = kLegOrder[index];
    auto & command = controller.commands[index];
    command.zero();
    command.position_desired =
      controller._quadruped.leg(leg_id).joints.home_position;

    // 当前 MJCF 使用 <position kp="100"> actuator，位置闭环增益由 MuJoCo
    // actuator 自身提供；这里不再额外叠加关节 PD，避免重复计算控制力矩。
    command.kp_joint.setZero();
    command.kd_joint.setZero();
  }
  controller.setEnabled(true);
}

// 将统一 JointCommand 发送到当前 MJCF 的位置执行器。
// 只有 enabled=true 的有效命令才会覆盖 ctrl；无效时保留上一帧目标位置。
void writePositionCommand(
  const JointCommand<float> & command, std::size_t leg_index,
  const ActuatorAddressArray & addresses, mjData * data)
{
  if (!command.enabled || data == nullptr || data->ctrl == nullptr) {
    return;
  }

  for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
    data->ctrl[addresses[leg_index][joint]] =
      command.position_desired[static_cast<Eigen::Index>(joint)];
  }
}

// 完成一个站立控制周期：读取四腿反馈、更新运动学、生成并发送四腿命令。
// 任一条腿反馈无效时关闭本周期全部输出，避免只控制部分腿造成机器人失稳。
bool updateStandingControl(
  LegController<float> & controller, LegSensorArray & legs,
  const ActuatorAddressArray & addresses)
{
  bool all_legs_valid = true;
  for (auto & leg : legs) {
    all_legs_valid = controller.updateData(*leg) && all_legs_valid;
  }
  controller.setEnabled(all_legs_valid);

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const JointCommand<float> command =
      controller.command(kLegOrder[index], static_cast<float>(g_data->time));
    writePositionCommand(command, index, addresses, g_data);
  }
  return all_legs_valid;
}

// 将 MuJoCo 传感器类型枚举转换为可读字符串。
const char * sensorTypeName(int type)
{
  switch (type) {
    case mjSENS_FRAMEQUAT: return "FRAMEQUAT(四元数)";
    case mjSENS_GYRO: return "GYRO(陀螺仪)";
    case mjSENS_ACCELEROMETER: return "ACCELEROMETER(加速度计)";
    default: return "(未知类型)";
  }
}

// 打印某个 IMU 传感器的定义（名称、类型、维度、sensordata 起始地址）。
void printSensor(const mjModel * model, const char * name)
{
  const int id = mj_name2id(model, mjOBJ_SENSOR, name);
  if (id < 0) {
    std::printf("  [%s]：未找到\n", name);
    return;
  }
  std::printf(
    "  [%s]  id=%d  类型=%s  维度=%d  sensordata 起始地址=%d\n",
    name, id, sensorTypeName(model->sensor_type[id]),
    model->sensor_dim[id], model->sensor_adr[id]);
}

// 启动时一次性打印模型概览与传感器定义。
void printModelInfo(const mjModel * model)
{
  std::printf("==================== MuJoCo 模型概览 ====================\n");
  std::printf("模型文件：%s\n", MYMIT_ROBOT_SCENE_PATH);
  std::printf("模型名称：%s\n", model->names);
  std::printf(
    "自由度：nq=%d（位置）  nv=%d（速度）\n",
    static_cast<int>(model->nq), static_cast<int>(model->nv));
  std::printf(
    "数量：body=%d  joint=%d  actuator=%d  geom=%d  site=%d  sensor=%d\n",
    static_cast<int>(model->nbody), static_cast<int>(model->njnt),
    static_cast<int>(model->nu),
    static_cast<int>(model->ngeom), static_cast<int>(model->nsite),
    static_cast<int>(model->nsensor));
  std::printf("仿真步长：dt=%g s\n\n", model->opt.timestep);

  std::printf("---------------- IMU 传感器参数 ----------------\n");
  printSensor(model, "imu_orientation");
  printSensor(model, "imu_angular_velocity");
  printSensor(model, "imu_linear_acceleration");
  std::printf("\n站立控制：目标关节角 [0, 0.9, -1.8] rad\n");
  std::printf("操作提示：空格=暂停/运行  Esc/关闭窗口=退出\n\n");
}

// 物理线程主循环：站立控制 + 仿真。
void physicsLoop(
  mj::Simulate & sim, ImuSensor & imu,
  LegController<float> & controller, LegSensorArray & legs,
  const ActuatorAddressArray & actuator_addresses)
{
  while (!sim.exitrequest.load()) {
    // 让出 CPU，避免忙等
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    std::unique_lock<std::recursive_mutex> lock(sim.mtx);
    if (g_model == nullptr || g_data == nullptr) {
      continue;
    }

    if (sim.run) {
      // mj_step 前先读取上一时刻的四腿状态，并把新站立目标写入 actuator。
      updateStandingControl(controller, legs, actuator_addresses);

      // 仿真前进一个物理步长
      mj_step(g_model, g_data);

      // 读取并保存 IMU 数据（不再打印，供控制循环后续使用）
      imu.read();
    } else {
      // 暂停时也要更新渲染，供拖动、调节滑块等操作刷新画面
      mj_forward(g_model, g_data);
    }
  }
}

}  // namespace

// 物理线程入口：加载模型 → 通知 GUI → 打印模型信息 → 进入仿真循环。
void physicsThread(mj::Simulate & sim)
{
  char error[1024]{};
  g_model = mj_loadXML(MYMIT_ROBOT_SCENE_PATH, nullptr, error, sizeof(error));
  if (g_model == nullptr) {
    std::fprintf(stderr, "无法加载模型：%s\n", error);
    sim.exitrequest.store(1);
    return;
  }
  g_data = mj_makeData(g_model);
  if (g_data == nullptr) {
    std::fprintf(stderr, "无法创建 MuJoCo 仿真数据\n");
    mj_deleteModel(g_model);
    g_model = nullptr;
    sim.exitrequest.store(1);
    return;
  }

  // 从 MJCF 已有的 home keyframe 启动：机身高度 0.27 m，四腿关节角均为
  // [0, 0.9, -1.8]。这只设置初始状态，后续仍由 LegController 持续控制。
  const int home_keyframe = mj_name2id(g_model, mjOBJ_KEY, "home");
  if (home_keyframe < 0) {
    std::fprintf(stderr, "模型中未找到 home keyframe，无法初始化站立姿态\n");
    mj_deleteData(g_data);
    mj_deleteModel(g_model);
    g_data = nullptr;
    g_model = nullptr;
    sim.exitrequest.store(1);
    return;
  }
  mj_resetDataKeyframe(g_model, g_data, home_keyframe);
  mj_forward(g_model, g_data);

  // 通知 Simulate UI 显示新模型
  sim.Load(g_model, g_data, MYMIT_ROBOT_SCENE_PATH);

  // 打印模型信息与传感器定义
  printModelInfo(g_model);

  // 通过工厂选择数据来源：这里使用仿真器（MuJoCo）
  // 要切换为真实硬件时，把 ImuSource::SIMULATOR 改为 ImuSource::HARDWARE
  // （硬件当前为占位实现，读取到的数据 valid == false）。
  try {
    auto imu = makeImu(ImuSource::SIMULATOR, g_model, g_data);
    auto legs = makeSimLegs(g_model, g_data);
    const auto actuator_addresses = findActuatorAddresses(g_model);
    const auto quadruped = makeQuadruped<float>(RobotType::UNITREE_GO1);
    LegController<float> controller(quadruped);
    configureStandingCommand(controller);
    physicsLoop(sim, *imu, controller, legs, actuator_addresses);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "站立控制初始化失败：%s\n", error.what());
    sim.exitrequest.store(1);
  }

  mj_deleteData(g_data);
  mj_deleteModel(g_model);
  g_model = nullptr;
  g_data = nullptr;
}

int main()
{
  // 版本检查
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // 初始化相机、渲染选项、扰动状态
  mjvCamera cam;
  mjv_defaultCamera(&cam);
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // 创建 Simulate 对象（封装整个 MuJoCo GUI）
  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(), &cam, &opt, &pert, false);

  // 物理线程：加载模型 + 仿真 + 实时打印 IMU 参数
  std::thread physics(physicsThread, std::ref(*sim));

  // 渲染事件循环（阻塞，直到关闭窗口）
  sim->RenderLoop();
  physics.join();

  return 0;
}
