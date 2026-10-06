#include "dummy_motion_client/bench_permit.hpp"

#include <cassert>
#include <string>

int main() {
  using namespace dummy_motion_client;
  PermitBinding binding{};
  binding.boot_epoch = 7U;
  binding.reference_epoch = 3U;
  binding.profile_id = 101U;
  binding.firmware_sha256[0] = 1U;
  binding.contract_sha256[0] = 2U;
  binding.model_sha256[0] = 3U;
  binding.profile_sha256[0] = 4U;
  std::array<std::uint8_t, 16> client{};
  client[0] = 0x55U;
  std::array<std::uint8_t, 32> digest{};
  assert(trusted_bench_permit_digest(binding, client, 9U, 11U, &digest));
  constexpr char hex[] = "0123456789abcdef";
  std::string actual;
  for (auto byte : digest) { actual += hex[byte >> 4U]; actual += hex[byte & 15U]; }
  assert(actual == "c43173a8461f3048b9cf82b6b5c6443f9e3861319cde7e70c2ce78032ee3b151");
  assert(!trusted_bench_permit_digest(binding, client, 9U, 0U, &digest));
  binding.firmware_sha256 = {};
  assert(!trusted_bench_permit_digest(binding, client, 9U, 11U, &digest));
}
