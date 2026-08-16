// ROS 2 正式入口。RobotRosNode 只负责中间件适配，SimulationBridge 再将命令
// 送入 RobotRunner -> ControlFSM -> MPC/WBC -> LegController 控制链。
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <dlfcn.h>

#include "RobotMiddlewareInterface.hpp"
#include "RobotRosNode.hpp"

namespace
{
struct DynamicLibraryCloser
{
  void operator()(void * handle) const noexcept
  {
    if (handle != nullptr) {::dlclose(handle);}
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    const std::string default_scene =
      ament_index_cpp::get_package_share_directory("mymit_robot") +
      "/unitree_go1/scene.xml";
    auto node = std::make_shared<RobotRosNode>(default_scene);

    // MuJoCo bundles third-party XML symbols also used by ROS middleware. Load
    // the backend locally with deep binding so neither library interposes on
    // the other; the C++ control algorithms stay entirely inside the backend.
    const std::string backend_path =
      ament_index_cpp::get_package_prefix("mymit_robot") +
      "/lib/libmymit_robot_simulation_backend.so";
    void * backend_handle = ::dlopen(
      backend_path.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
    if (backend_handle == nullptr) {
      throw std::runtime_error(
              "unable to load simulation backend: " + std::string(::dlerror()));
    }
    std::unique_ptr<void, DynamicLibraryCloser> backend(backend_handle);
    void * symbol = ::dlsym(backend.get(), "mymit_robot_run_simulation");
    if (symbol == nullptr) {
      throw std::runtime_error(
              "simulation backend entry point is missing: " + std::string(::dlerror()));
    }
    RunSimulationBackend run_backend = nullptr;
    static_assert(sizeof(run_backend) == sizeof(symbol));
    std::memcpy(&run_backend, &symbol, sizeof(run_backend));

    const SimulationBackendConfig config{
      node->sceneFile().c_str(), node->initialStandingHeight(),
      node->slowWalkingSpeed(), node->fastWalkingSpeed(),
      node->backwardWalkingSpeed(), node->lateralWalkingSpeed(),
      node->turningYawRate()};

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() {executor.spin();});
    try {
      char backend_error[1024]{};
      result = run_backend(&config, node.get(), backend_error, sizeof(backend_error));
      if (result != 0 && backend_error[0] != '\0') {
        throw std::runtime_error(backend_error);
      }
    } catch (...) {
      rclcpp::shutdown();
      executor_thread.join();
      throw;
    }
    rclcpp::shutdown();
    executor_thread.join();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "ROS 2 simulation node failed: %s\n", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
