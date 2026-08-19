/**
 * @file StandingHeightIpc.hpp
 * @brief MuJoCo 进程与 ROS 2 高度服务进程之间的轻量本地通信协议。
 *
 * 两个进程分离是为了避免本机 MuJoCo 3.11 与 rclcpp 的 C++ 符号冲突。
 */
#ifndef MYMIT_ROBOT_USER_STANDING_HEIGHT_IPC_HPP_
#define MYMIT_ROBOT_USER_STANDING_HEIGHT_IPC_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <sys/socket.h>
#include <sys/un.h>

namespace standing_height_ipc
{

inline constexpr float kMinimumHeight = 0.18F;
// 协议范围覆盖两种机型；每个 RobotRunner 还会按自己的 profile 再次限幅。
inline constexpr float kMaximumHeight = 0.42F;
inline constexpr float kDefaultHeight = 0.27F;
inline constexpr std::uint32_t kCommandMagic = 0x4D485447U;
inline constexpr char kSocketName[] = "mymit_robot_standing_height";

struct Command
{
  std::uint32_t magic = kCommandMagic;
  float height = kDefaultHeight;
};

inline sockaddr_un socketAddress() noexcept
{
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  // Linux 抽象域 socket 不创建磁盘文件，仿真退出后由内核自动清理，
  // 因此不会残留 /tmp 文件影响下一次启动。
  address.sun_path[0] = '\0';
  std::memcpy(address.sun_path + 1, kSocketName, sizeof(kSocketName) - 1);
  return address;
}

inline constexpr socklen_t socketAddressLength() noexcept
{
  return static_cast<socklen_t>(
    offsetof(sockaddr_un, sun_path) + 1 + sizeof(kSocketName) - 1);
}

}  // namespace standing_height_ipc

#endif  // MYMIT_ROBOT_USER_STANDING_HEIGHT_IPC_HPP_
