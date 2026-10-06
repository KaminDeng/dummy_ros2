#pragma once

#include <array>
#include <cstdint>

namespace dummy_motion_client {
struct PermitBinding {
  std::uint64_t boot_epoch = 0;
  std::uint32_t reference_epoch = 0;
  std::uint32_t profile_id = 0;
  std::array<std::uint8_t, 32> firmware_sha256{};
  std::array<std::uint8_t, 32> contract_sha256{};
  std::array<std::uint8_t, 32> model_sha256{};
  std::array<std::uint8_t, 32> profile_sha256{};
};

bool trusted_bench_permit_digest(const PermitBinding &binding,
                                 const std::array<std::uint8_t, 16> &client,
                                 std::uint64_t session_id, std::uint32_t request_seq,
                                 std::array<std::uint8_t, 32> *output) noexcept;
}  // namespace dummy_motion_client
