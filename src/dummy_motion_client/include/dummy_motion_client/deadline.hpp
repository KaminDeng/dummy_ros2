#pragma once
#include <cstdint>
#include <limits>
namespace dummy_motion_client {
constexpr std::uint64_t saturating_deadline(std::uint64_t now,std::uint64_t budget) noexcept {
  const auto maximum=std::numeric_limits<std::uint64_t>::max();
  return budget>maximum-now ? maximum : now+budget;
}
}  // namespace dummy_motion_client
