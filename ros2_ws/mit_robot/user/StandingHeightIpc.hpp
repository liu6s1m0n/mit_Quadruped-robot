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
inline constexpr float kMaximumHeight = 0.34F;
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
  // An abstract-domain socket is removed automatically when the simulator
  // exits and cannot collide with stale files in /tmp.
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
