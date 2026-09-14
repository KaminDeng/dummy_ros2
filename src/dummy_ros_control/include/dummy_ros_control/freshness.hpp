#pragma once
#include <cstdint>

namespace dummy_ros_control {
inline bool ticket_valid(std::uint64_t issued_at, std::uint64_t now,
                         std::uint64_t window) noexcept {
  if (window == 0U || issued_at > UINT64_MAX - window) return false;
  return now >= issued_at && now < issued_at + window;
}
}  // namespace dummy_ros_control
