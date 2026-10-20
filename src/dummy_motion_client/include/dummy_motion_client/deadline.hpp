/**
 * @file deadline.hpp
 * @brief 无符号微秒截止时间的饱和加法，避免大预算回绕为已超时。
 */
#pragma once
#include <cstdint>
#include <limits>
namespace dummy_motion_client {
/**
 * @brief 计算 min(now+budget, UINT64_MAX)，保持同一单调时钟域。
 * @param now 当前时间，单位微秒。
 * @param budget 相对预算，单位微秒；零预算返回 now。
 * @return 饱和绝对截止时间；饱和不代表获得无限运行许可，调用方仍需自己的状态/权限门禁。
 */
constexpr std::uint64_t saturating_deadline(std::uint64_t now,std::uint64_t budget) noexcept {
  const auto maximum=std::numeric_limits<std::uint64_t>::max();
  return budget>maximum-now ? maximum : now+budget;
}
}  // namespace dummy_motion_client
