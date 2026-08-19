// 正式仿真入口：这里配置运行参数，SimulationBridge 再将其送入
// RobotRunner -> ControlFSM -> MPC/WBC -> LegController 控制链。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>

#include "SimulationBridge.hpp"

namespace
{
// 慢档用于崎岖地形稳定性测试；快档只在原 0.30 m/s 基础上适度提速。
constexpr float kSlowWalkingForwardSpeed = 0.18F;
constexpr float kFastWalkingForwardSpeed = 0.36F;
constexpr float kGo1WalkingLateralSpeed = 0.25F;
constexpr float kDm1WalkingLateralSpeed = 0.20F;
constexpr float kTurningYawRate = 0.30F;

struct RobotSelection
{
  RobotType type;             ///< 要加载的机器人型号。
  const char * scene_path;    ///< 该型号对应的 MuJoCo 场景。
  float standing_height;      ///< 启动后的默认站立高度，m。
  float lateral_speed;        ///< 该机型经过接触回归验证的默认横移速度，m/s。
  const char * render_gpu;    ///< OpenGL 渲染设备策略：nvidia 或 auto。
};

RobotSelection selectRobot(int argc, char ** argv)
{
  // launch_ros 会在自定义参数后追加“--ros-args -r ...”。这里只解析本程序的
  // --robot/--render-gpu 选项，遇到 --ros-args 后把剩余参数完整留给 ROS 2。
  const char * robot_name = "go1";
  const char * render_gpu = "nvidia";
  for (int index = 1; index < argc; ++index) {
    if (std::strcmp(argv[index], "--ros-args") == 0) {break;}
    if (index + 1 >= argc) {
      throw std::invalid_argument(
        "usage: mymit_robot_user [--robot go1|dm1] "
        "[--render-gpu nvidia|auto]");
    }
    if (std::strcmp(argv[index], "--robot") == 0) {
      robot_name = argv[++index];
    } else if (std::strcmp(argv[index], "--render-gpu") == 0) {
      render_gpu = argv[++index];
    } else {
      throw std::invalid_argument(
        "usage: mymit_robot_user [--robot go1|dm1] "
        "[--render-gpu nvidia|auto]");
    }
  }

  if (std::strcmp(render_gpu, "nvidia") != 0 &&
    std::strcmp(render_gpu, "auto") != 0)
  {
    throw std::invalid_argument("render GPU must be nvidia or auto");
  }
  if (std::strcmp(robot_name, "go1") == 0) {
    return {
      RobotType::UNITREE_GO1, MYMIT_ROBOT_GO1_SCENE_PATH, 0.27F,
      kGo1WalkingLateralSpeed, render_gpu};
  }
  if (std::strcmp(robot_name, "dm1") == 0) {
    return {
      RobotType::DM_BOT1, MYMIT_ROBOT_DM1_SCENE_PATH, 0.39F,
      kDm1WalkingLateralSpeed, render_gpu};
  }
  throw std::invalid_argument(
    "usage: mymit_robot_user [--robot go1|dm1] "
    "[--render-gpu nvidia|auto]");
}

/** @brief 在混合显卡笔记本上让 GLFW/OpenGL 默认创建 NVIDIA 上下文。 */
void configureGpuRendering(const char * render_gpu)
{
  if (std::strcmp(render_gpu, "auto") == 0) {return;}
  // 这些变量必须在 GlfwAdapter 创建 OpenGL 上下文之前设置。第三个参数为 1，
  // 确保 launch 的默认 nvidia 策略不会被桌面会话中的旧变量意外覆盖。
  if (setenv("__NV_PRIME_RENDER_OFFLOAD", "1", 1) != 0 ||
    setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", 1) != 0 ||
    setenv("__VK_LAYER_NV_optimus", "NVIDIA_only", 1) != 0)
  {
    throw std::runtime_error("failed to configure NVIDIA PRIME render offload");
  }
}
}

int main(int argc, char ** argv)
{
  try {
    const RobotSelection robot = selectRobot(argc, argv);
    configureGpuRendering(robot.render_gpu);
    SimulationBridge bridge(robot.scene_path, robot.type);
    bridge.setStandingHeight(robot.standing_height);
    bridge.setSlowWalkingForwardSpeed(kSlowWalkingForwardSpeed);
    bridge.setFastWalkingForwardSpeed(kFastWalkingForwardSpeed);
    bridge.setWalkingLateralSpeed(robot.lateral_speed);
    bridge.setTurningYawRate(kTurningYawRate);
    return bridge.run();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "simulation failed: %s\n", error.what());
    return 1;
  }
}
