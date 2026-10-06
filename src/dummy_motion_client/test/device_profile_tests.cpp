#include <cassert>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "dummy_motion_client/device_profile.hpp"

namespace dummy_motion_client::detail {
YAML::Node load_pinned_yaml(const std::filesystem::path &path, const std::string &expected_sha256,
                            const std::function<void()> &after_snapshot);
}

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  using dummy_motion_client::validated_motion_permit_limit;
  assert(validated_motion_permit_limit(10000000U,660000000U,1500000U,3000000U,
                                       5000000U,std::nullopt)==60000000U);
  assert(validated_motion_permit_limit(630000000U,660000000U,1500000U,3000000U,
                                       5000000U,650000000U)==650000000U);
  for(const auto request: {std::optional<std::uint64_t>{},
                           std::optional<std::uint64_t>{639499999U},
                           std::optional<std::uint64_t>{660000001U}}) {
    bool denied=false;
    try { (void)validated_motion_permit_limit(630000000U,660000000U,1500000U,
                                               3000000U,5000000U,request); }
    catch(const std::invalid_argument &) { denied=true; }
    assert(denied);
  }
  const auto profile = dummy_motion_client::load_device_profile(argv[1], "sil_6dof", 1U, 63U);
  assert(profile.joints.joints.size() == 6U);
  assert(profile.joints.joints.front() == "Joint1");
  assert(profile.joints.lower.at(2) == -0.3);
  assert(profile.joints.upper.at(2) == 0.3);
  const auto frozen = dummy_motion_client::inspect_device_bundle_identity(argv[2], "sil_j1");
  assert(frozen.contract_hash != profile.contract_hash);
  bool foreign_sil_rejected = false;
  try { (void)dummy_motion_client::load_device_profile(argv[2], "sil_j1", 2U, 1U); }
  catch (const std::invalid_argument &) { foreign_sil_rejected = true; }
  assert(foreign_sil_rejected);
  assert(profile.joints.velocity.at(0) == 0.25);
  assert(profile.joints.acceleration.at(0) == 0.5);
  assert(profile.joints.path_tolerance.at(0) == 0.15);
  assert(profile.joints.goal_tolerance.at(0) == 0.001);
  assert(profile.candidate_profile_payload_hash == profile.profile_hash);
  assert(!profile.firmware_bound);
  dummy_motion_client::DeviceStatusIdentity status{};
  status.contract_hash = profile.contract_hash;
  status.model_hash = profile.model_hash;
  status.profile_hash = profile.profile_hash;
  status.policy_hash = profile.policy_hash;
  status.firmware_hash[0] = 1U;
  assert(dummy_motion_client::status_matches_bundle_identity(profile, status));
  status.firmware_hash.fill(0U);
  assert(!dummy_motion_client::status_matches_bundle_identity(profile, status));
  bool unqualified_bench = false;
  try { (void)dummy_motion_client::load_device_profile(argv[1], "f405_6dof_bench", 201U, 63U); }
  catch (const std::invalid_argument &) { unqualified_bench = true; }
  assert(unqualified_bench);
  bool wrong_bench = false;
  try { (void)dummy_motion_client::load_device_profile(argv[1], "f405_6dof_bench", 201U, 1U); }
  catch (const std::invalid_argument &) { wrong_bench = true; }
  assert(wrong_bench);
  bool rejected = false;
  try { (void)dummy_motion_client::load_device_profile(argv[1], "sil_6dof", 1U, 1U); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  const auto copy = std::filesystem::temp_directory_path() / ("device_profile_" + std::to_string(getpid()));
  std::filesystem::remove_all(copy);
  std::filesystem::copy(argv[1], copy, std::filesystem::copy_options::recursive);
  { std::ofstream drift(copy / "profiles/sil_6dof.yaml", std::ios::app); drift << "\n# payload drift\n"; }
  rejected = false;
  try { (void)dummy_motion_client::load_device_profile(copy.string(), "sil_6dof", 1U, 63U); }
  catch (const std::invalid_argument &) { rejected = true; }
  std::filesystem::remove_all(copy);
  assert(rejected);
  const auto activation = std::filesystem::temp_directory_path() /
                          ("device_profile_activation_" + std::to_string(getpid()) + ".yaml");
  {
    std::ofstream output(activation);
    output << "scope: NO_LOAD_DEV\n";
  }
  const auto pinned = dummy_motion_client::detail::load_pinned_yaml(
      activation, "c8fdca54aef747dab564237233407b0cae847743a1e70df1c34774176236dc9b",
      [&activation] {
        std::ofstream replacement(activation, std::ios::trunc);
        replacement << "scope: FORMAL\n";
      });
  assert(pinned["scope"].as<std::string>() == "NO_LOAD_DEV");
  std::filesystem::remove(activation);
  std::cout << "{\"device_profile\":true,\"six_axis\":true,\"mask_drift_rejected\":true,\"payload_drift_rejected\":true}\n";
}
