#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include "mymit_robot/srv/set_standing_height.hpp"
#include "StandingHeightIpc.hpp"

namespace
{

class SocketHandle
{
public:
  SocketHandle()
  : descriptor_(::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0))
  {
    if (descriptor_ < 0) {
      throw std::runtime_error(
              std::string("unable to create height-command socket: ") +
              std::strerror(errno));
    }
  }

  ~SocketHandle()
  {
    if (descriptor_ >= 0) {::close(descriptor_);}
  }

  SocketHandle(const SocketHandle &) = delete;
  SocketHandle & operator=(const SocketHandle &) = delete;

  int get() const noexcept {return descriptor_;}

private:
  int descriptor_ = -1;
};

class StandingHeightService : public rclcpp::Node
{
public:
  StandingHeightService()
  : Node("mymit_robot_standing_height_service")
  {
    service_ = create_service<mymit_robot::srv::SetStandingHeight>(
      "set_standing_height",
      [this](
        const std::shared_ptr<mymit_robot::srv::SetStandingHeight::Request> request,
        std::shared_ptr<mymit_robot::srv::SetStandingHeight::Response> response)
      {
        handleRequest(request->height, *response);
      });
    RCLCPP_INFO(
      get_logger(), "Standing-height service ready: accepted range [%.2f, %.2f] m",
      standing_height_ipc::kMinimumHeight,
      standing_height_ipc::kMaximumHeight);
  }

private:
  void handleRequest(
    float height, mymit_robot::srv::SetStandingHeight::Response & response)
  {
    response.target_height = height;
    if (!std::isfinite(height) || height < standing_height_ipc::kMinimumHeight ||
      height > standing_height_ipc::kMaximumHeight)
    {
      response.success = false;
      response.message = "standing height must be within [0.18, 0.34] m";
      return;
    }

    const standing_height_ipc::Command command{
      standing_height_ipc::kCommandMagic, height};
    const sockaddr_un address = standing_height_ipc::socketAddress();
    const ssize_t sent = ::sendto(
      socket_.get(), &command, sizeof(command), 0,
      reinterpret_cast<const sockaddr *>(&address),
      standing_height_ipc::socketAddressLength());
    if (sent != static_cast<ssize_t>(sizeof(command))) {
      response.success = false;
      response.message = std::string("simulation is not receiving commands: ") +
        std::strerror(errno);
      return;
    }

    response.success = true;
    response.message = "standing height command accepted";
  }

  SocketHandle socket_;
  rclcpp::Service<mymit_robot::srv::SetStandingHeight>::SharedPtr service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<StandingHeightService>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "standing-height service failed: %s\n", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
