#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "dummy_motion_client/action_gate.hpp"

namespace dummy_motion_client {
struct DeviceBundleIdentity {
  std::array<std::uint8_t, 32U> bundle_hash{};
  std::array<std::uint8_t, 32U> contract_hash{};
  std::array<std::uint8_t, 32U> model_hash{};
  std::array<std::uint8_t, 32U> candidate_profile_payload_hash{};
  std::array<std::uint8_t, 32U> profile_hash{};
  std::array<std::uint8_t, 32U> policy_hash{};
  std::array<std::uint8_t, 32U> firmware_hash{};
  bool firmware_bound = false;
};

struct DeviceStatusIdentity {
  std::array<std::uint8_t, 32U> contract_hash{};
  std::array<std::uint8_t, 32U> model_hash{};
  std::array<std::uint8_t, 32U> profile_hash{};
  std::array<std::uint8_t, 32U> policy_hash{};
  std::array<std::uint8_t, 32U> firmware_hash{};
};

struct DeviceProfile : DeviceBundleIdentity {
  JointProfile joints;
  std::uint64_t motion_permit_max_us = 60000000U;
  bool bench_only = false;
  bool software_only_development = false;
  bool functional_development_basis = false;
};

std::uint64_t validated_motion_permit_limit(std::uint64_t goal_us,
    std::uint64_t trial_us, std::uint64_t stop_us,
    std::uint64_t completion_us, std::uint64_t hold_us,
    std::optional<std::uint64_t> explicit_permit_us);

struct NoLoadDevActivationRequest {
  std::string activation_path;
  // Must be pinned independently of activation_path; this loader never derives authority from the file itself.
  std::string expected_activation_sha256;
  std::string source_bundle_root;
  std::string prebuild_bundle_manifest_path;
  std::string compiled_policy_manifest_path;
  std::string firmware_image_path;
  std::string host_executable_path;
  std::string map_file_path;
  std::string resource_report_path;
  std::string qualification_report_path;
  std::string device_id;
  std::string usb_serial;
  std::string endpoint;
  std::string d_regression_report_path;
  std::string d_regression_provenance_path;
  std::string hil_report_path;
  std::string hil_raw_index_path;
  std::string hil_evidence_root;
  std::string elf_path;
  std::string resource_base_root;
};

// Inspects payload identity without granting runtime authority.
DeviceBundleIdentity inspect_device_bundle_identity(const std::string &bundle_root,
                                                    const std::string &profile_name);
bool status_matches_bundle_identity(const DeviceBundleIdentity &expected,
                                    const DeviceStatusIdentity &actual) noexcept;

// Loads a manifest-pinned CI bundle. Any missing field, digest drift, or axis/profile mismatch throws.
DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask);
DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask,
                                  const NoLoadDevActivationRequest &activation);
}  // namespace dummy_motion_client
