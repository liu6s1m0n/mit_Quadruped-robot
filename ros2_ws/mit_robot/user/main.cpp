// 正式仿真入口：这里配置运行参数，SimulationBridge 再将其送入
// RobotRunner -> ControlFSM -> MPC/WBC -> LegController 控制链。
#include <cstdio>
#include <exception>

#include "SimulationBridge.hpp"

namespace
{
constexpr float kStandingHeight = 0.27F;
// 正值向前、负值向后。当前为兼顾稳定性和估计精度的 0.32 m/s。
//速度参数 重要！！！
constexpr float kWalkingForwardSpeed = 0.40F;
}

int main()
{
  try {
    SimulationBridge bridge(MYMIT_ROBOT_SCENE_PATH);
    bridge.setStandingHeight(kStandingHeight);
    bridge.setWalkingForwardSpeed(kWalkingForwardSpeed);
    return bridge.run();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "simulation failed: %s\n", error.what());
    return 1;
  }
}
