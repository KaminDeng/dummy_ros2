/**
 * @file freshness.hpp
 * @brief 提供不假定墙钟的半开时间窗判断。
 * issued_at/now/window必须在同一时钟域且单位一致；函数只做数值边界检查，不负责源设备时钟与宿主steady时钟的对齐。
 */
#pragma once
#include <cstdint>

namespace dummy_ros_control {
/**
 * @brief 判断当前时刻是否位于票据有效半开区间。
 * @param issued_at 签发时刻。
 * @param now 同域当前时刻。
 * @param window 同单位有效时长，必须非零。
 * @return bool：issued_at≤now<issued_at+window，且加法不溢出。
 */
inline bool ticket_valid(std::uint64_t issued_at, std::uint64_t now,
                         std::uint64_t window) noexcept {
  if (window == 0U || issued_at > UINT64_MAX - window) return false;
  return now >= issued_at && now < issued_at + window;
}
}  // namespace dummy_ros_control
