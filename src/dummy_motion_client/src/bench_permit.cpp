#include "dummy_motion_client/bench_permit.hpp"

#include <algorithm>
#include <cstring>
#include <openssl/sha.h>

namespace dummy_motion_client {
namespace {
template <typename Integer>
void append_le(std::uint8_t *output, Integer value) noexcept {
  for (std::size_t index = 0; index < sizeof(Integer); ++index)
    output[index] = static_cast<std::uint8_t>(value >> (index * 8U));
}
bool nonzero(const std::array<std::uint8_t, 32> &value) noexcept {
  return std::any_of(value.begin(), value.end(), [](auto byte) { return byte != 0U; });
}
}  // namespace

bool trusted_bench_permit_digest(const PermitBinding &binding,
                                 const std::array<std::uint8_t, 16> &client,
                                 std::uint64_t session_id, std::uint32_t request_seq,
                                 std::array<std::uint8_t, 32> *output) noexcept {
  if (output == nullptr || binding.boot_epoch == 0U || binding.reference_epoch == 0U ||
      binding.profile_id == 0U || session_id == 0U || request_seq == 0U ||
      !nonzero(binding.firmware_sha256) || !nonzero(binding.contract_sha256) ||
      !nonzero(binding.model_sha256) || !nonzero(binding.profile_sha256)) return false;
  constexpr char domain[] = "DUMMY-BENCH-PERMIT-V1";
  std::array<std::uint8_t, 193> bytes{};
  std::size_t offset = sizeof(domain) - 1U;
  std::memcpy(bytes.data(), domain, offset);
  append_le(bytes.data() + offset, binding.boot_epoch); offset += 8U;
  append_le(bytes.data() + offset, session_id); offset += 8U;
  std::copy(client.begin(), client.end(), bytes.begin() + offset); offset += client.size();
  append_le(bytes.data() + offset, request_seq); offset += 4U;
  append_le(bytes.data() + offset, binding.reference_epoch); offset += 4U;
  append_le(bytes.data() + offset, binding.profile_id); offset += 4U;
  for (const auto *hash : {&binding.firmware_sha256, &binding.contract_sha256,
                           &binding.model_sha256, &binding.profile_sha256}) {
    std::copy(hash->begin(), hash->end(), bytes.begin() + offset);
    offset += hash->size();
  }
  return offset == bytes.size() && SHA256(bytes.data(), bytes.size(), output->data()) != nullptr;
}
}  // namespace dummy_motion_client
