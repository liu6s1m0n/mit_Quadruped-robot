// main 只负责创建桥接器；模型加载、线程和控制算法全部封装在 SimulationBridge 中。
#include <cstdio>
#include <exception>

#include "SimulationBridge.hpp"

int main()
{
  try {
    SimulationBridge bridge(MYMIT_ROBOT_SCENE_PATH);
    return bridge.run();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "simulation failed: %s\n", error.what());
    return 1;
  }
}
