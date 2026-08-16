// 正式仿真入口：这里配置运行参数，SimulationBridge 再将其送入
// RobotRunner -> ControlFSM -> MPC/WBC -> LegController 控制链。
#include <cstdio>
#include <exception>

#include "SimulationBridge.hpp"

namespace
{
constexpr float kStandingHeight = 0.27F;
// 慢档用于崎岖地形稳定性测试；快档只在原 0.30 m/s 基础上适度提速。
constexpr float kSlowWalkingForwardSpeed = 0.18F;
constexpr float kFastWalkingForwardSpeed = 0.36F;
constexpr float kWalkingLateralSpeed = 0.25F;
constexpr float kTurningYawRate = 0.30F;
}

int main()
{
  try {
    SimulationBridge bridge(MYMIT_ROBOT_SCENE_PATH);
    bridge.setStandingHeight(kStandingHeight);
    bridge.setSlowWalkingForwardSpeed(kSlowWalkingForwardSpeed);
    bridge.setFastWalkingForwardSpeed(kFastWalkingForwardSpeed);
    bridge.setWalkingLateralSpeed(kWalkingLateralSpeed);
    bridge.setTurningYawRate(kTurningYawRate);
    return bridge.run();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "simulation failed: %s\n", error.what());
    return 1;
  }
}
