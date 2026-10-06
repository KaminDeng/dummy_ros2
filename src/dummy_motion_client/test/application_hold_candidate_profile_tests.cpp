#include <cassert>
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include "dummy_motion_client/device_profile.hpp"

namespace {
std::array<std::uint8_t, 32U> digest(const std::string &hex) {
  assert(hex.size() == 64U);
  std::array<std::uint8_t, 32U> value{};
  for (std::size_t index = 0; index < value.size(); ++index)
    value[index] = static_cast<std::uint8_t>(std::stoul(hex.substr(index * 2U, 2U), nullptr, 16));
  return value;
}

std::string file_sha256(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::string bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  std::array<unsigned char, SHA256_DIGEST_LENGTH> hash{};
  SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), hash.data());
  std::ostringstream output;
  for (const auto byte : hash) output << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
  return output.str();
}
}  // namespace

int main(int argc, char **argv) {
  if (argc != 2 && argc != 3 && argc != 4) return 2;
  assert(std::filesystem::is_regular_file(std::filesystem::path(argv[1]) / "manifest.json"));
  const auto manifest = YAML::LoadFile((std::filesystem::path(argv[1]) / "manifest.json").string());
  const auto identity = dummy_motion_client::inspect_device_bundle_identity(
    argv[1], "f405_6dof_bench");
  assert(identity.contract_hash != identity.bundle_hash);
  assert(identity.contract_hash == digest(manifest["source_contract_sha256"].as<std::string>()));
  assert(identity.bundle_hash == digest(manifest["payload_sorted_sha256"].as<std::string>()));
  assert(identity.firmware_hash == digest(manifest["firmware_image_sha256"].as<std::string>()));
  assert(identity.policy_hash == digest(manifest["compiled_policy_sha256"].as<std::string>()));
  assert(identity.firmware_bound);
  assert(std::any_of(identity.firmware_hash.begin(), identity.firmware_hash.end(),
                     [](auto byte) { return byte != 0U; }));

  dummy_motion_client::DeviceStatusIdentity status{};
  status.contract_hash = identity.contract_hash;
  status.model_hash = identity.model_hash;
  status.profile_hash = identity.profile_hash;
  status.policy_hash = identity.policy_hash;
  status.firmware_hash = identity.firmware_hash;
  assert(dummy_motion_client::status_matches_bundle_identity(identity, status));
  status.firmware_hash[0] ^= 1U;
  assert(!dummy_motion_client::status_matches_bundle_identity(identity, status));
  status.firmware_hash.fill(0U);
  assert(!dummy_motion_client::status_matches_bundle_identity(identity, status));
  status.firmware_hash = identity.firmware_hash;
  status.contract_hash = identity.bundle_hash;
  assert(!dummy_motion_client::status_matches_bundle_identity(identity, status));
  status.contract_hash = identity.contract_hash;
  status.policy_hash[0] ^= 1U;
  assert(!dummy_motion_client::status_matches_bundle_identity(identity, status));
  status.policy_hash = identity.policy_hash;
  auto missing_firmware = identity;
  missing_firmware.firmware_hash.fill(0U);
  assert(!dummy_motion_client::status_matches_bundle_identity(missing_firmware, status));

  std::string reason;
  try {
    (void)dummy_motion_client::load_device_profile(argv[1], "f405_6dof_bench", 201U, 63U);
  } catch (const std::invalid_argument &error) {
    reason = error.what();
  }
  assert(reason == "no-load development activation required");

  const auto flipped_root = std::filesystem::temp_directory_path() /
                            ("application_hold_flipped_" + std::to_string(getpid()));
  std::filesystem::copy(argv[1], flipped_root, std::filesystem::copy_options::recursive);
  const auto flipped_manifest = flipped_root / "manifest.json";
  auto flipped = YAML::LoadFile(flipped_manifest.string());
  flipped["runtime_enabled"] = true;
  { std::ofstream output(flipped_manifest); output << flipped; }
  reason.clear();
  try { (void)dummy_motion_client::load_device_profile(flipped_root.string(), "f405_6dof_bench", 201U, 63U); }
  catch (const std::invalid_argument &error) { reason = error.what(); }
  std::filesystem::remove_all(flipped_root);
  assert(reason == "bench profile authority mismatch");

  const auto activation_root = std::filesystem::temp_directory_path() /
                               ("application_hold_activation_" + std::to_string(getpid()));
  std::filesystem::create_directories(activation_root);
  const auto candidate_root = activation_root / "candidate";
  std::filesystem::copy(argv[1], candidate_root, std::filesystem::copy_options::recursive);
  const auto source_root = argc == 3 ? std::filesystem::path(argv[2]) :
                           std::filesystem::path(argv[1]).parent_path() / "dummy_robot_v2";
  const auto image = activation_root / "firmware.bin";
  const auto host = activation_root / "host.bin";
  const auto elf = activation_root / "firmware.elf";
  const auto map_file = activation_root / "firmware.map";
  const auto resource = activation_root / "resource.json";
  const auto report = activation_root / "report.json";
  const auto prebuild_path = activation_root / "prebuild.json";
  const auto compiled_path = activation_root / "compiled.json";
  const auto activation_path = activation_root / "activation.yaml";
  { std::ofstream output(image); output << "firmware fixture"; }
  { std::ofstream output(host); output << "host fixture"; }
  { std::ofstream output(elf); output << "ELF fixture"; }
  { std::ofstream output(map_file); output << "map fixture"; }
  YAML::Node resource_data;
  resource_data["result"] = "FAIL";
  resource_data["measurement_provenance"] = "NOT_ASSESSED";
  resource_data["measurement_source"] = "static_link_only";
  resource_data["release_qualified"] = false;
  resource_data["elf"] = elf.string();
  { std::ofstream output(resource); output << resource_data; }
  { std::ofstream output(report); output << "{\"qualification_scope\":\"SOFTWARE_ONLY\",\"result\":\"PASS\"}"; }
  const auto source = YAML::LoadFile((source_root / "manifest.json").string());
  const auto candidate_path = candidate_root / "manifest.json";
  auto candidate = YAML::LoadFile(candidate_path.string());
  candidate["firmware_image_sha256"] = file_sha256(image);
  { std::ofstream output(candidate_path); output << candidate; }

  YAML::Node prebuild;
  prebuild["format"] = "dummy_robot_bundle_v1";
  prebuild["profile_id"] = 201;
  prebuild["configured_mask"] = 63;
  prebuild["contract_sha256"] = candidate["source_contract_sha256"].as<std::string>();
  prebuild["model_sha256"] = std::string(64U, '4');
  prebuild["profile_sha256"] = std::string(64U, '1');
  prebuild["compiled_policy_sha256"] = std::string(64U, '2');
  prebuild["bundle_identity"]["build_config_sha256"] = std::string(64U, '3');
  prebuild["bundle_identity"]["source_sha256"] = std::string(64U, '6');
  { std::ofstream output(prebuild_path); output << prebuild; }

  YAML::Node compiled;
  compiled["format"] = "dummy_compiled_policy_manifest_v1";
  compiled["policy_sha256"] = prebuild["compiled_policy_sha256"].as<std::string>();
  compiled["firmware_image_sha256"] = file_sha256(image);
  compiled["runtime_enabled"] = true;
  auto post_build = compiled["post_build_bindings"];
  post_build["bundle_manifest_sha256"] = file_sha256(prebuild_path);
  post_build["firmware_sha256"] = file_sha256(image);
  post_build["host_sha256"] = file_sha256(host);
  post_build["map_sha256"] = file_sha256(map_file);
  post_build["resource_report_sha256"] = file_sha256(resource);
  post_build["qualification_report_sha256"] = file_sha256(report);
  { std::ofstream output(compiled_path); output << compiled; }

  YAML::Node activation;
  activation["schema"] = "dummy_no_load_dev_activation_v1";
  activation["scope"] = "NO_LOAD_DEV";
  activation["qualification"] = "UNQUALIFIED";
  activation["candidate_manifest_sha256"] = file_sha256(candidate_path);
  activation["candidate_payload_sorted_sha256"] = candidate["payload_sorted_sha256"].as<std::string>();
  activation["source_wire_manifest_sha256"] = file_sha256(source_root / "manifest.json");
  activation["source_contract_sha256"] = candidate["source_contract_sha256"].as<std::string>();
  activation["model_sha256"] = candidate["model_sha256"].as<std::string>();
  activation["profile_sha256"] = candidate["payloads"][2]["sha256"].as<std::string>();
  activation["compiled_policy_sha256"] = candidate["compiled_policy_sha256"].as<std::string>();
  activation["firmware_sha256"] = file_sha256(image);
  activation["prebuild_bundle_manifest_sha256"] = file_sha256(prebuild_path);
  activation["prebuild_model_sha256"] = prebuild["model_sha256"].as<std::string>();
  activation["prebuild_profile_sha256"] = prebuild["profile_sha256"].as<std::string>();
  activation["prebuild_compiled_policy_sha256"] = prebuild["compiled_policy_sha256"].as<std::string>();
  activation["build_config_sha256"] = prebuild["bundle_identity"]["build_config_sha256"].as<std::string>();
  activation["host_sha256"] = file_sha256(host);
  activation["map_sha256"] = file_sha256(map_file);
  activation["resource_report_sha256"] = file_sha256(resource);
  activation["qualification_report_sha256"] = file_sha256(report);
  activation["device_id"] = "0123456789abcdef0123456789abcdef";
  activation["usb_vid"] = 0x1209;
  activation["usb_pid"] = 0x7461;
  activation["usb_interface"] = 0;
  activation["usb_serial"] = "DEMO00000001";
  activation["endpoint"] = "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200";
  auto write_activation = [&] {
    std::ofstream output(activation_path);
    output << activation;
    output.close();
    return file_sha256(activation_path);
  };
  const auto activation_hash = write_activation();
  dummy_motion_client::NoLoadDevActivationRequest request{
    activation_path.string(), activation_hash, source_root.string(), prebuild_path.string(),
    compiled_path.string(), image.string(), host.string(), map_file.string(), resource.string(), report.string(),
    "0123456789abcdef0123456789abcdef", "DEMO00000001",
    "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200"};
  const auto activated = dummy_motion_client::load_device_profile(
    candidate_root.string(), "f405_6dof_bench", 201U, 63U, request);
  assert(activated.bench_only);
  assert(activated.software_only_development);
  assert(activated.motion_permit_max_us == 60000000U);
  assert(activated.joints.joints.size() == 6U);
  for (std::size_t axis = 0U; axis < activated.joints.joints.size(); ++axis) {
    assert(activated.joints.lower[axis] == -0.6);
    assert(activated.joints.upper[axis] == 0.6);
  }
  dummy_motion_client::Trajectory zero_goal;
  zero_goal.joints = activated.joints.joints;
  zero_goal.points.push_back({std::vector<double>(6U, 0.0), {}, {}, 1.0});
  dummy_motion_client::Position zero_start{};
  assert(dummy_motion_client::validate_trajectory(zero_goal, activated.joints, &zero_start));
  assert(activated.candidate_profile_payload_hash == activated.profile_hash);
  assert(activated.contract_hash == digest(source["payload_sorted_sha256"].as<std::string>()));

  const auto reject_request = [&](const dummy_motion_client::NoLoadDevActivationRequest &attempt) {
    try {
      (void)dummy_motion_client::load_device_profile(
        candidate_root.string(), "f405_6dof_bench", 201U, 63U, attempt);
      return false;
    } catch (const std::invalid_argument &) {
      return true;
    }
  };
  auto unbacked_functional = YAML::Clone(activation);
  unbacked_functional["basis"] = "NO_LOAD_DEV_FUNCTIONAL";
  { std::ofstream output(activation_path); output << unbacked_functional; }
  request.expected_activation_sha256 = file_sha256(activation_path);
  assert(reject_request(request));
  request.expected_activation_sha256 = write_activation();
  compiled["runtime_enabled"] = false;
  { std::ofstream output(compiled_path); output << compiled; }
  assert(reject_request(request));
  compiled["runtime_enabled"] = true;
  { std::ofstream output(compiled_path); output << compiled; }
  prebuild["configured_mask"] = 1;
  { std::ofstream output(prebuild_path); output << prebuild; }
  post_build["bundle_manifest_sha256"] = file_sha256(prebuild_path);
  { std::ofstream output(compiled_path); output << compiled; }
  activation["prebuild_bundle_manifest_sha256"] = file_sha256(prebuild_path);
  request.expected_activation_sha256 = write_activation();
  assert(reject_request(request));
  prebuild["configured_mask"] = 63;
  { std::ofstream output(prebuild_path); output << prebuild; }
  post_build["bundle_manifest_sha256"] = file_sha256(prebuild_path);
  { std::ofstream output(compiled_path); output << compiled; }
  activation["prebuild_bundle_manifest_sha256"] = file_sha256(prebuild_path);
  request.expected_activation_sha256 = write_activation();
  auto toggled_candidate = YAML::Clone(candidate);
  toggled_candidate["runtime_enabled"] = true;
  { std::ofstream output(candidate_path); output << toggled_candidate; }
  assert(reject_request(request));
  { std::ofstream output(candidate_path); output << candidate; }
  { std::ofstream output(report); output << "{\"qualification_scope\":\"APPLICATION_HOLD\",\"result\":\"PASS\"}"; }
  post_build["qualification_report_sha256"] = file_sha256(report);
  { std::ofstream output(compiled_path); output << compiled; }
  activation["qualification_report_sha256"] = file_sha256(report);
  request.expected_activation_sha256 = write_activation();
  assert(reject_request(request));
  { std::ofstream output(report); output << "{\"qualification_scope\":\"SOFTWARE_ONLY\",\"result\":\"PASS\"}"; }
  post_build["qualification_report_sha256"] = file_sha256(report);
  { std::ofstream output(compiled_path); output << compiled; }
  activation["qualification_report_sha256"] = file_sha256(report);
  request.expected_activation_sha256 = write_activation();
  for (const auto &change : std::array<std::pair<const char *, const char *>, 21U>{{
           {"scope", "RELEASE_APPROVED"}, {"qualification", "QUALIFIED"},
           {"candidate_manifest_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"candidate_payload_sorted_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"source_wire_manifest_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"source_contract_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"model_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"profile_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"compiled_policy_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"firmware_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"prebuild_bundle_manifest_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"prebuild_model_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"prebuild_profile_sha256", "5555555555555555555555555555555555555555555555555555555555555555"},
           {"prebuild_compiled_policy_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"build_config_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"host_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"map_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"resource_report_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"qualification_report_sha256", "1111111111111111111111111111111111111111111111111111111111111111"},
           {"device_id", "11111111111111111111111111111111"},
           {"endpoint", "serial//dev/ttyACM1#baudrate=115200"}}}) {
    auto changed = YAML::Clone(activation);
    changed[change.first] = change.second;
    { std::ofstream output(activation_path); output << changed; }
    request.expected_activation_sha256 = file_sha256(activation_path);
    assert(reject_request(request));
  }
  request.expected_activation_sha256 = write_activation();
  request.device_id = "11111111111111111111111111111111";
  assert(reject_request(request));
  request.device_id = "0123456789abcdef0123456789abcdef";
  request.usb_serial = "OTHER";
  assert(reject_request(request));
  request.usb_serial = "DEMO00000001";
  activation["usb_pid"] = 0x1366;
  request.expected_activation_sha256 = write_activation();
  assert(reject_request(request));
  activation["usb_pid"] = 0x7461;

  request.expected_activation_sha256 = std::string(64U, '0');
  assert(reject_request(request));
  request.expected_activation_sha256 = activation_hash;
  activation["firmware_sha256"] = std::string(64U, '1');
  request.expected_activation_sha256 = write_activation();
  assert(reject_request(request));

  const auto functional_root = activation_root / "functional";
  const auto evidence_root = functional_root / "evidence";
  std::filesystem::create_directories(evidence_root);
  const auto junit_path = functional_root / "d253.xml";
  const auto provenance_path = functional_root / "provenance.json";
  const auto hil_path = functional_root / "hil.json";
  const auto raw_index_path = functional_root / "hil_raw_index.json";
  const auto functional_compiled_path = functional_root / "compiled.json";
  const auto timestamp = "2026-09-30T20:08:40";
  {
    std::ofstream output(junit_path);
    output << "<testsuite tests=\"253\" failures=\"0\" skipped=\"0\" timestamp=\"" << timestamp << "\">";
    for (int test = 1; test <= 253; ++test)
      output << "<testcase name=\"case" << test << "\" classname=\"dummy\" status=\"run\"/>";
    output << "</testsuite>";
  }
  const auto build_dir = (functional_root / "build").string();
  YAML::Node provenance;
  provenance["schema"] = "dummy_d_ctest_provenance_v1";
  provenance["generated_at_utc"] = "2026-09-30T20:09:00Z";
  provenance["junit_timestamp"] = timestamp;
  provenance["build_dir"] = build_dir;
  provenance["rerun_argv"].push_back("ctest");
  provenance["rerun_argv"].push_back("--test-dir");
  provenance["rerun_argv"].push_back(build_dir);
  provenance["rerun_argv"].push_back("--output-junit");
  provenance["rerun_argv"].push_back(junit_path.string());
  provenance["source_sha256"] = prebuild["bundle_identity"]["source_sha256"].as<std::string>();
  provenance["source_hash_kind"] = "PREBUILD_INPUTS";
  provenance["source_state"] = "DIRTY_UNQUALIFIED";
  provenance["firmware_sha256"] = file_sha256(image);
  provenance["host_sha256"] = file_sha256(host);
  provenance["contract_sha256"] = candidate["source_contract_sha256"].as<std::string>();
  provenance["model_sha256"] = candidate["model_sha256"].as<std::string>();
  provenance["compiled_policy_sha256"] = candidate["compiled_policy_sha256"].as<std::string>();
  provenance["build_config_sha256"] = prebuild["bundle_identity"]["build_config_sha256"].as<std::string>();
  provenance["junit_sha256"] = file_sha256(junit_path);
  provenance["test_count"] = 253;
  { std::ofstream output(provenance_path); output << provenance; }

  YAML::Node hil;
  hil["format"] = "dummy_v2_hil_v2";
  hil["qualification_scope"] = "APPLICATION_HOLD";
  hil["application_hold"] = true;
  hil["result"] = "PASS";
  hil["requested_cycles"] = 50;
  hil["executed_cycles"] = 50;
  hil["passed_cycles"] = 50;
  hil["firmware_sha256"] = file_sha256(image);
  hil["cli_sha256"] = file_sha256(host);
  hil["contract_sha256"] = candidate["source_contract_sha256"].as<std::string>();
  hil["model_sha256"] = candidate["model_sha256"].as<std::string>();
  hil["profile_sha256"] = candidate["compiled_policy_sha256"].as<std::string>();
  hil["profile_id"] = 201;
  hil["configured_mask"] = 63;
  hil["endpoint"] = activation["endpoint"].as<std::string>();
  const std::string boot = "39be60bb526bb02b";
  hil["boot_epochs"].push_back(boot);
  YAML::Node raw_index;
  raw_index["schema"] = "dummy_no_load_dev_functional_raw_index_v1";
  for (int cycle = 1; cycle <= 50; ++cycle) {
    std::ostringstream directory_name;
    directory_name << "cycle-" << std::setw(3) << std::setfill('0') << cycle;
    const auto relative_dir = std::filesystem::path(directory_name.str());
    const auto cycle_dir = evidence_root / relative_dir;
    std::filesystem::create_directories(cycle_dir);
    const auto motion_run = std::to_string(1000 + cycle);
    const auto release_run = std::to_string(2000 + cycle);
    YAML::Node cycle_index;
    YAML::Node cycle_binding;
    cycle_binding["cycle"] = cycle;
    cycle_binding["run_id"] = release_run;
    cycle_binding["boot_epoch"] = boot;
    cycle_binding["index_file"] = (relative_dir / "index.json").string();
    for (int page = 0; page < 2; ++page) {
      const auto name = page == 0 ? "page-0000.bin" : "page-0001.bin";
      const auto page_path = cycle_dir / name;
      { std::ofstream output(page_path); output << "raw-" << cycle << "-" << page; }
      const auto run_id = page == 0 ? motion_run : release_run;
      YAML::Node indexed_page;
      indexed_page["file"] = name;
      indexed_page["raw_sha256"] = file_sha256(page_path);
      indexed_page["run_id"] = std::stoull(run_id);
      indexed_page["evidence_seq"] = page;
      indexed_page["attempt_kind"] = page == 0 ? 1 : 3;
      indexed_page["release_committed"] = page == 1;
      indexed_page["expected_mask"] = page == 0 ? 63 : 0;
      indexed_page["sent_mask"] = page == 0 ? 63 : 0;
      indexed_page["failed_mask"] = 0;
      indexed_page["final_state"] = page == 1 ? 8 : 0;
      cycle_index.push_back(indexed_page);
      YAML::Node bound_page;
      bound_page["file"] = (relative_dir / name).string();
      bound_page["raw_sha256"] = file_sha256(page_path);
      bound_page["run_id"] = run_id;
      bound_page["evidence_seq"] = page;
      cycle_binding["pages"].push_back(bound_page);
    }
    const auto index_path = cycle_dir / "index.json";
    { std::ofstream output(index_path); output << cycle_index; }
    cycle_binding["index_sha256"] = file_sha256(index_path);
    raw_index["cycles"].push_back(cycle_binding);
    YAML::Node hil_cycle;
    hil_cycle["cycle"] = cycle;
    hil_cycle["passed"] = true;
    hil_cycle["observed"]["run_id"] = release_run;
    hil_cycle["observed"]["evidence_run_id"] = release_run;
    hil["cycles"].push_back(hil_cycle);
  }
  { std::ofstream output(raw_index_path); output << raw_index; }
  { std::ofstream output(hil_path); output << hil; }
  auto functional_compiled = YAML::Clone(compiled);
  functional_compiled.remove("post_build_bindings");
  functional_compiled["policy_sha256"] = candidate["compiled_policy_sha256"].as<std::string>();
  { std::ofstream output(functional_compiled_path); output << functional_compiled; }
  auto functional_activation = YAML::Clone(activation);
  functional_activation["firmware_sha256"] = file_sha256(image);
  functional_activation["basis"] = "NO_LOAD_DEV_FUNCTIONAL";
  functional_activation.remove("qualification_report_sha256");
  functional_activation["d_regression_report_sha256"] = file_sha256(junit_path);
  functional_activation["d_regression_provenance_sha256"] = file_sha256(provenance_path);
  functional_activation["hil_report_sha256"] = file_sha256(hil_path);
  functional_activation["hil_raw_index_sha256"] = file_sha256(raw_index_path);
  functional_activation["elf_sha256"] = file_sha256(elf);
  functional_activation["live_profile_sha256"] = candidate["compiled_policy_sha256"].as<std::string>();
  { std::ofstream output(activation_path); output << functional_activation; }
  auto functional_request = request;
  functional_request.compiled_policy_manifest_path = functional_compiled_path.string();
  functional_request.qualification_report_path.clear();
  functional_request.d_regression_report_path = junit_path.string();
  functional_request.d_regression_provenance_path = provenance_path.string();
  functional_request.hil_report_path = hil_path.string();
  functional_request.hil_raw_index_path = raw_index_path.string();
  functional_request.hil_evidence_root = evidence_root.string();
  functional_request.elf_path = elf.string();
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  const auto functional_profile = dummy_motion_client::load_device_profile(
    candidate_root.string(), "f405_6dof_bench", 201U, 63U, functional_request);
  assert(functional_profile.functional_development_basis);
  assert(!functional_profile.software_only_development);
  assert(functional_profile.joints.joints[2] == "Joint3");
  assert(functional_profile.joints.lower[2] == -0.6);
  assert(functional_profile.joints.upper[2] == 0.6);
  assert(functional_profile.candidate_profile_payload_hash ==
         digest(candidate["payloads"][2]["sha256"].as<std::string>()));
  assert(functional_profile.profile_hash == digest(candidate["compiled_policy_sha256"].as<std::string>()));
  assert(functional_profile.profile_hash != digest(candidate["payloads"][2]["sha256"].as<std::string>()));
  dummy_motion_client::DeviceStatusIdentity live_status{};
  live_status.contract_hash = functional_profile.contract_hash;
  live_status.model_hash = functional_profile.model_hash;
  live_status.profile_hash = functional_profile.profile_hash;
  live_status.policy_hash = functional_profile.policy_hash;
  live_status.firmware_hash = functional_profile.firmware_hash;
  assert(dummy_motion_client::status_matches_bundle_identity(functional_profile, live_status));
  live_status.profile_hash = digest(candidate["payloads"][2]["sha256"].as<std::string>());
  assert(!dummy_motion_client::status_matches_bundle_identity(functional_profile, live_status));
  auto missing_live_profile = YAML::Clone(functional_activation);
  missing_live_profile.remove("live_profile_sha256");
  { std::ofstream output(activation_path); output << missing_live_profile; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  assert(reject_request(functional_request));
  functional_activation["live_profile_sha256"] = std::string(64U, '1');
  { std::ofstream output(activation_path); output << functional_activation; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  assert(reject_request(functional_request));
  functional_activation["live_profile_sha256"] = candidate["compiled_policy_sha256"].as<std::string>();
  { std::ofstream output(activation_path); output << functional_activation; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  const auto first_index_path = evidence_root / "cycle-001/index.json";
  auto bad_sent_index = YAML::LoadFile(first_index_path.string());
  bad_sent_index[0]["sent_mask"] = 0;
  { std::ofstream output(first_index_path); output << bad_sent_index; }
  auto bad_sent_raw_index = YAML::Clone(raw_index);
  bad_sent_raw_index["cycles"][0]["index_sha256"] = file_sha256(first_index_path);
  { std::ofstream output(raw_index_path); output << bad_sent_raw_index; }
  functional_activation["hil_raw_index_sha256"] = file_sha256(raw_index_path);
  { std::ofstream output(activation_path); output << functional_activation; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  assert(reject_request(functional_request));
  auto good_first_index = YAML::LoadFile(first_index_path.string());
  good_first_index[0]["sent_mask"] = 63;
  { std::ofstream output(first_index_path); output << good_first_index; }
  { std::ofstream output(raw_index_path); output << raw_index; }
  functional_activation["hil_raw_index_sha256"] = file_sha256(raw_index_path);
  { std::ofstream output(activation_path); output << functional_activation; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  auto false_resource = YAML::Clone(resource_data);
  false_resource["result"] = "PASS";
  { std::ofstream output(resource); output << false_resource; }
  functional_activation["resource_report_sha256"] = file_sha256(resource);
  { std::ofstream output(activation_path); output << functional_activation; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  functional_activation["elf_sha256"] = std::string(64U, '1');
  { std::ofstream output(activation_path); output << functional_activation; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  assert(reject_request(functional_request));
  functional_activation["elf_sha256"] = file_sha256(elf);
  { std::ofstream output(activation_path); output << functional_activation; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  assert(reject_request(functional_request));
  { std::ofstream output(resource); output << resource_data; }
  functional_activation["resource_report_sha256"] = file_sha256(resource);
  { std::ofstream output(activation_path); output << functional_activation; }
  functional_request.expected_activation_sha256 = file_sha256(activation_path);
  const auto first_page = evidence_root / "cycle-001/page-0000.bin";
  { std::ofstream output(first_page, std::ios::app); output << "tampered"; }
  assert(reject_request(functional_request));
  { std::ofstream output(first_page); output << "raw-1-0"; }

  auto write_functional_activation = [&] {
    std::ofstream output(activation_path);
    output << functional_activation;
    output.close();
    functional_request.expected_activation_sha256 = file_sha256(activation_path);
  };
  auto short_index = YAML::Clone(raw_index);
  short_index["cycles"].remove(49U);
  { std::ofstream output(raw_index_path); output << short_index; }
  functional_activation["hil_raw_index_sha256"] = file_sha256(raw_index_path);
  write_functional_activation();
  assert(reject_request(functional_request));
  { std::ofstream output(raw_index_path); output << raw_index; }
  functional_activation["hil_raw_index_sha256"] = file_sha256(raw_index_path);

  auto short_hil = YAML::Clone(hil);
  short_hil["passed_cycles"] = 49;
  { std::ofstream output(hil_path); output << short_hil; }
  functional_activation["hil_report_sha256"] = file_sha256(hil_path);
  write_functional_activation();
  assert(reject_request(functional_request));
  { std::ofstream output(hil_path); output << hil; }
  functional_activation["hil_report_sha256"] = file_sha256(hil_path);

  auto short_provenance = YAML::Clone(provenance);
  short_provenance["test_count"] = 251;
  { std::ofstream output(provenance_path); output << short_provenance; }
  functional_activation["d_regression_provenance_sha256"] = file_sha256(provenance_path);
  write_functional_activation();
  assert(reject_request(functional_request));
  { std::ofstream output(provenance_path); output << provenance; }
  std::ifstream junit_input(junit_path);
  std::string junit_xml((std::istreambuf_iterator<char>(junit_input)), std::istreambuf_iterator<char>());
  const auto failure_field = junit_xml.find("failures=\"0\"");
  assert(failure_field != std::string::npos);
  junit_xml.replace(failure_field, std::string("failures=\"0\"").size(), "failures=\"1\"");
  { std::ofstream output(junit_path); output << junit_xml; }
  provenance["junit_sha256"] = file_sha256(junit_path);
  { std::ofstream output(provenance_path); output << provenance; }
  functional_activation["d_regression_report_sha256"] = file_sha256(junit_path);
  functional_activation["d_regression_provenance_sha256"] = file_sha256(provenance_path);
  write_functional_activation();
  assert(reject_request(functional_request));
  std::filesystem::remove_all(activation_root);

  const auto copy = std::filesystem::temp_directory_path() /
                    ("application_hold_candidate_" + std::to_string(getpid()));
  std::filesystem::copy(argv[1], copy, std::filesystem::copy_options::recursive);
  {
    std::ofstream payload(copy / "policy/application_hold_no_load.json", std::ios::app);
    payload << "\n";
  }
  reason.clear();
  try {
    (void)dummy_motion_client::load_device_profile(copy.string(), "f405_6dof_bench", 201U, 63U);
  } catch (const std::invalid_argument &error) {
    reason = error.what();
  }
  std::filesystem::remove_all(copy);
  assert(reason == "bundle payload hash mismatch");

  const auto linked_copy = std::filesystem::temp_directory_path() /
                           ("application_hold_link_" + std::to_string(getpid()));
  const auto outside_model = std::filesystem::temp_directory_path() /
                             ("application_hold_model_" + std::to_string(getpid()) + ".yaml");
  std::filesystem::copy(argv[1], linked_copy, std::filesystem::copy_options::recursive);
  std::filesystem::rename(linked_copy / "model/model.yaml", outside_model);
  std::filesystem::create_symlink(outside_model, linked_copy / "model/model.yaml");
  reason.clear();
  try {
    (void)dummy_motion_client::inspect_device_bundle_identity(linked_copy.string(), "f405_6dof_bench");
  } catch (const std::invalid_argument &error) {
    reason = error.what();
  }
  std::filesystem::remove_all(linked_copy);
  std::filesystem::remove(outside_model);
  assert(reason == "bundle payload hash mismatch");

  const auto linked_directory = std::filesystem::temp_directory_path() /
                                ("application_hold_dir_link_" + std::to_string(getpid()));
  const auto outside_directory = std::filesystem::temp_directory_path() /
                                 ("application_hold_model_dir_" + std::to_string(getpid()));
  std::filesystem::copy(argv[1], linked_directory, std::filesystem::copy_options::recursive);
  std::filesystem::rename(linked_directory / "model", outside_directory);
  std::filesystem::create_directory_symlink(outside_directory, linked_directory / "model");
  reason.clear();
  try {
    (void)dummy_motion_client::inspect_device_bundle_identity(linked_directory.string(), "f405_6dof_bench");
  } catch (const std::invalid_argument &error) {
    reason = error.what();
  }
  std::filesystem::remove_all(linked_directory);
  std::filesystem::remove_all(outside_directory);
  assert(reason == "bundle payload hash mismatch");

  if (argc == 4) {
    const std::filesystem::path supplied(argv[2]);
    const auto fixture = std::filesystem::is_directory(supplied) ? supplied : supplied.parent_path();
    const auto real_activation_path = std::filesystem::is_directory(supplied) ?
      fixture / "activation.json" : supplied;
    const auto real_activation = YAML::LoadFile(real_activation_path.string());
    auto real_candidate_root = fixture / "candidate";
    dummy_motion_client::NoLoadDevActivationRequest real_request{
      real_activation_path.string(), file_sha256(real_activation_path), argv[3],
      (fixture / "bundle/manifest.json").string(), (fixture / "compiled-manifest.json").string(),
      (fixture / "firmware.bin").string(), (fixture / "host.bin").string(),
      (fixture / "firmware.map").string(), (fixture / "resource.json").string(),
      (fixture / "report.json").string(), real_activation["device_id"].as<std::string>(),
      real_activation["usb_serial"].as<std::string>(), real_activation["endpoint"].as<std::string>()};
    const auto real_basis = real_activation["basis"];
    const bool functional = real_basis && real_basis.as<std::string>() == "NO_LOAD_DEV_FUNCTIONAL";
    if (functional) {
      real_request.qualification_report_path.clear();
      real_request.d_regression_report_path = (fixture / "d-regression.xml").string();
      real_request.d_regression_provenance_path = (fixture / "d-provenance.json").string();
      real_request.hil_report_path = (fixture / "hil-report.json").string();
      real_request.hil_raw_index_path = (fixture / "hil-raw-index.json").string();
      real_request.hil_evidence_root = (fixture / "hil-evidence").string();
      real_request.elf_path = (fixture / "firmware.elf").string();
      if (std::filesystem::is_regular_file(fixture / "prebuild-bundle/manifest.json")) {
        const auto dpsdk = std::filesystem::path(argv[3]).parent_path().parent_path();
        const auto firmware = dpsdk / "build_dummy/qualification/p7-app-hold-compile";
        const auto reports = dpsdk / "build_dummy/reports";
        real_request.prebuild_bundle_manifest_path = (fixture / "prebuild-bundle/manifest.json").string();
        real_request.compiled_policy_manifest_path =
          (reports / "p7-app-compiled-manifest-20260930.json").string();
        real_request.firmware_image_path = (firmware / "dpsdk_firmware.bin").string();
        real_request.elf_path = (firmware / "dpsdk_firmware").string();
        real_request.map_file_path = (firmware / "dpsdk_firmware.map").string();
        real_request.host_executable_path = (dpsdk / "build_dummy/qualification/p7-app-hold-host-permit/"
                                                "applications/dummy_ref_control/dummy_joint_cli").string();
        real_request.resource_report_path = (reports / "p7-app-resource-static-20260930.json").string();
        real_request.resource_base_root = dpsdk.string();
        real_request.d_regression_report_path = (reports / "p7-app-v2-tests-252-20260930.xml").string();
        real_request.hil_report_path = (reports / "p7-app-hold-50cycle-20260930-03.json").string();
        real_request.hil_evidence_root =
          (reports / "p7-app-hold-50cycle-20260930-03-evidence").string();
      } else if (real_activation_path.filename().string().rfind("no-load-dev-activation-", 0U) == 0U) {
        const auto dpsdk = std::filesystem::path(argv[3]).parent_path().parent_path();
        const auto reports = dpsdk / "build_dummy/reports";
        const auto firmware = dpsdk / "build_dummy/qualification/p7-app-hold-compile";
        const auto activation_name = real_activation_path.filename().string();
        const std::string variant = activation_name.find("-v4.json") != std::string::npos ? "-v4" :
                                    activation_name.find("-v3.json") != std::string::npos ? "-v3" :
                                    activation_name.find("-v2.json") != std::string::npos ? "-v2" : "";
        real_candidate_root = reports / "no-load-dev-candidate-7468-20260930";
        real_request.prebuild_bundle_manifest_path =
          (reports / "no-load-dev-prebuild-7468-20260930/manifest.json").string();
        real_request.compiled_policy_manifest_path =
          (reports / "p7-app-compiled-manifest-20260930.json").string();
        real_request.firmware_image_path = (firmware / "dpsdk_firmware.bin").string();
        real_request.elf_path = (firmware / "dpsdk_firmware").string();
        real_request.map_file_path = (firmware / "dpsdk_firmware.map").string();
        real_request.host_executable_path = (dpsdk / "build_dummy/qualification/p7-app-hold-host-permit/"
                                                "applications/dummy_ref_control/dummy_joint_cli").string();
        real_request.resource_report_path = (reports / "p7-app-resource-static-20260930.json").string();
        real_request.resource_base_root = dpsdk.string();
        real_request.d_regression_report_path = (reports / "p7-app-v2-tests-252-20260930.xml").string();
        real_request.d_regression_provenance_path =
          (reports / ("no-load-dev-d-provenance-7468-f077-20260930" + variant + ".json")).string();
        real_request.hil_report_path = (reports / "p7-app-hold-50cycle-20260930-03.json").string();
        real_request.hil_raw_index_path =
          (reports / ("no-load-dev-functional-raw-index-7468-f077-20260930" + variant + ".json")).string();
        real_request.hil_evidence_root =
          (reports / "p7-app-hold-50cycle-20260930-03-evidence").string();
      }
    }
    const auto real_profile = dummy_motion_client::load_device_profile(
      real_candidate_root.string(), "f405_6dof_bench", 201U, 63U, real_request);
    assert(real_profile.bench_only);
    assert(real_profile.functional_development_basis == functional);
    assert(real_profile.software_only_development != functional);
  }
}
