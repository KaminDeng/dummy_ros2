#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

#include "dummy_motion_client/device_profile.hpp"

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const auto profile = dummy_motion_client::load_device_profile(argv[1], "sil_6dof", 1U, 63U);
  assert(profile.joints.joints.size() == 6U);
  assert(profile.joints.joints.front() == "Joint1");
  assert(profile.joints.lower.at(2) > 0.6);
  assert(profile.joints.velocity.at(0) == 0.25);
  assert(profile.joints.acceleration.at(0) == 0.5);
  assert(profile.joints.path_tolerance.at(0) == 0.15);
  assert(profile.joints.goal_tolerance.at(0) == 0.001);
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
  std::cout << "{\"device_profile\":true,\"six_axis\":true,\"mask_drift_rejected\":true,\"payload_drift_rejected\":true}\n";
}
