#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "dummy_motion_client/action_gate.hpp"

namespace dummy_motion_client {
struct DeviceProfile {
  JointProfile joints;
  std::array<std::uint8_t, 32U> contract_hash{};
  std::array<std::uint8_t, 32U> model_hash{};
  std::array<std::uint8_t, 32U> profile_hash{};
};

// Loads a manifest-pinned CI bundle. Any missing field, digest drift, or axis/profile mismatch throws.
DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask);
}  // namespace dummy_motion_client
