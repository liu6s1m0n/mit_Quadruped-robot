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
