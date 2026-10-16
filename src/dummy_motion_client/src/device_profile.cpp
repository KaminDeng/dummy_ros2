#include "dummy_motion_client/device_profile.hpp"
#include "dp_dummy_robot_v2.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <openssl/evp.h>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

namespace dummy_motion_client {
namespace detail {
YAML::Node load_pinned_yaml(const std::filesystem::path &path, const std::string &expected_sha256,
                            const std::function<void()> &after_snapshot);
}
namespace {
namespace fs = std::filesystem;

[[noreturn]] void invalid(const char *message) { throw std::invalid_argument(message); }

std::string sha256_file(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) invalid("bundle payload missing");
  EVP_MD_CTX *context = EVP_MD_CTX_new();
  if (context == nullptr) invalid("sha256 init failed");
  const auto release = [&context] { EVP_MD_CTX_free(context); };
  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) { release(); invalid("sha256 init failed"); }
  std::array<char, 4096U> bytes{};
  while (input.good()) {
    input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (input.gcount() > 0 && EVP_DigestUpdate(context, bytes.data(), static_cast<std::size_t>(input.gcount())) != 1) {
      release(); invalid("sha256 update failed");
    }
  }
  if (!input.eof()) { release(); invalid("bundle payload unreadable"); }
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_size) != 1 || digest_size != digest.size()) {
    release(); invalid("sha256 final failed");
  }
  release();
  static constexpr char hex[] = "0123456789abcdef";
  std::string result(64U, '0');
  for (std::size_t i = 0; i < digest.size(); ++i) {
    result[i * 2U] = hex[digest[i] >> 4U];
    result[i * 2U + 1U] = hex[digest[i] & 15U];
  }
  return result;
}

std::array<std::uint8_t, 32U> parse_sha256(const std::string &text) {
  if (text.size() != 64U) invalid("invalid sha256");
  const auto nibble = [](char value) -> int {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    return -1;
  };
  std::array<std::uint8_t, 32U> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    const int high = nibble(text[i * 2U]);
    const int low = nibble(text[i * 2U + 1U]);
    if (high < 0 || low < 0) invalid("invalid sha256");
    result[i] = static_cast<std::uint8_t>((high << 4U) | low);
  }
  return result;
}

std::string sha256_text(const std::string &text) {
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  if (SHA256(reinterpret_cast<const unsigned char *>(text.data()), text.size(), digest.data()) == nullptr)
    invalid("sha256 text failed");
  static constexpr char hex[] = "0123456789abcdef";
  std::string result(64U, '0');
  for (std::size_t i = 0; i < digest.size(); ++i) {
    result[i * 2U] = hex[digest[i] >> 4U];
    result[i * 2U + 1U] = hex[digest[i] & 15U];
  }
  return result;
}

std::string read_file(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) invalid("bundle payload missing");
  const std::string bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  if (input.bad()) invalid("bundle payload unreadable");
  return bytes;
}

YAML::Node load_yaml_snapshot(const fs::path &path) { return YAML::Load(read_file(path)); }

bool safe_profile_name(const std::string &name) {
  return !name.empty() && name.size() <= 64U && std::all_of(name.begin(), name.end(), [](unsigned char value) {
    return std::isalnum(value) || value == '_';
  });
}

const YAML::Node require(const YAML::Node &node, const char *field) {
  const YAML::Node value = node[field];
  if (!value) invalid("required bundle field missing");
  return value;
}

std::uint8_t axis_bit(const std::string &name) {
  if (name.size() != 6U || name.substr(0U, 5U) != "Joint" || name[5] < '1' || name[5] > '6')
    invalid("invalid active axis");
  return static_cast<std::uint8_t>(1U << static_cast<unsigned>(name[5] - '1'));
}

double finite_positive(const YAML::Node &node) {
  const double value = node.as<double>();
  if (!std::isfinite(value) || value <= 0.0) invalid("invalid profile limit");
  return value;
}

struct VerifiedBundle {
  fs::path root;
  YAML::Node manifest;
  std::string manifest_sha256;
  std::unordered_map<std::string, std::string> payloads;
  std::string aggregate;
};

bool regular_unlinked_payload(const fs::path &root, const fs::path &relative) {
  fs::path current = root;
  for (const auto &part : relative) {
    current /= part;
    std::error_code error;
    const auto status = fs::symlink_status(current, error);
    if (error || fs::is_symlink(status)) return false;
  }
  return fs::is_regular_file(current);
}

VerifiedBundle verify_bundle(const std::string &bundle_root) {
    const fs::path root = fs::weakly_canonical(fs::path(bundle_root));
    if (!fs::is_directory(root)) invalid("bundle root unavailable");
    const fs::path manifest_path = root / "manifest.json";
    const std::string manifest_bytes = read_file(manifest_path);
    const YAML::Node manifest = YAML::Load(manifest_bytes);
    const std::string expected_release = "dummy-wire-v" +
        std::to_string(dp::protocol::dummy_robot_v2::kSchemaMajor) + "." +
        std::to_string(dp::protocol::dummy_robot_v2::kSchemaMinor);
    if (require(manifest, "format").as<std::string>() != "dummy_wire_release_v2" ||
        require(manifest, "release_id").as<std::string>() != expected_release) invalid("unsupported bundle release");
    const YAML::Node payloads = require(manifest, "payloads");
    if (!payloads.IsSequence() || payloads.size() == 0U) invalid("invalid bundle payload list");
    std::unordered_map<std::string, std::string> verified;
    std::string listing, previous;
    for (const YAML::Node &payload : payloads) {
      const std::string relative = require(payload, "path").as<std::string>();
      const std::string expected = require(payload, "sha256").as<std::string>();
      const fs::path path(relative);
      if (path.is_absolute() || relative.find("..") != std::string::npos || !regular_unlinked_payload(root, path) ||
          sha256_file(root / path) != expected || !verified.emplace(relative, expected).second ||
          (!previous.empty() && previous >= relative)) invalid("bundle payload hash mismatch");
      if (!listing.empty()) listing.push_back('\n');
      listing += relative + ":" + expected;
      previous = relative;
    }
    const std::string aggregate = require(manifest, "payload_sorted_sha256").as<std::string>();
    if (aggregate != sha256_text(listing)) invalid("bundle aggregate hash mismatch");
    return {root, manifest, sha256_text(manifest_bytes), std::move(verified), aggregate};
}

DeviceBundleIdentity bundle_identity(const VerifiedBundle &bundle, const std::string &profile_name) {
    if (!safe_profile_name(profile_name)) invalid("invalid requested device profile");
    const std::string model_key = "model/model.yaml";
    const std::string profile_key = "profiles/" + profile_name + ".yaml";
    if (bundle.payloads.find(model_key) == bundle.payloads.end() ||
        bundle.payloads.find(profile_key) == bundle.payloads.end()) invalid("unmanifested model or profile");
    DeviceBundleIdentity output;
    output.bundle_hash = parse_sha256(bundle.aggregate);
    output.contract_hash = output.bundle_hash;
    output.model_hash = parse_sha256(bundle.payloads.at(model_key));
    output.profile_hash = parse_sha256(bundle.payloads.at(profile_key));
    output.candidate_profile_payload_hash = output.profile_hash;
    output.policy_hash = output.profile_hash;
    if (profile_name == "f405_6dof_bench") {
      output.firmware_bound = true;
      output.contract_hash = parse_sha256(require(bundle.manifest, "source_contract_sha256").as<std::string>());
      output.policy_hash = parse_sha256(require(bundle.manifest, "compiled_policy_sha256").as<std::string>());
      output.firmware_hash = parse_sha256(require(bundle.manifest, "firmware_image_sha256").as<std::string>());
      if (std::all_of(output.contract_hash.begin(), output.contract_hash.end(), [](auto byte) { return byte == 0U; }) ||
          std::all_of(output.firmware_hash.begin(), output.firmware_hash.end(), [](auto byte) { return byte == 0U; }))
        invalid("invalid bundle identity");
    }
    return output;
}

std::string xml_attribute(const std::string &tag, const char *name) {
  std::smatch match;
  if (!std::regex_search(tag, match, std::regex(std::string("\\b") + name + "=\"([^\"]*)\"")))
    invalid("D252 JUnit attribute missing");
  return match[1].str();
}

std::pair<std::string, unsigned> verify_d_regression_junit(
    const fs::path &path, const std::string &expected_sha256) {
  const std::string xml = read_file(path);
  if (parse_sha256(sha256_text(xml)) != parse_sha256(expected_sha256))
    invalid("D regression JUnit SHA mismatch");
  std::smatch suite;
  if (!std::regex_search(xml, suite, std::regex(R"(<testsuite\b[^>]*>)")))
    invalid("D regression JUnit suite missing");
  const std::string count_text = xml_attribute(suite.str(), "tests");
  unsigned test_count = 0U;
  const auto count = std::from_chars(count_text.data(), count_text.data() + count_text.size(), test_count);
  if (count.ec != std::errc{} || count.ptr != count_text.data() + count_text.size() ||
      test_count < 252U ||
      xml_attribute(suite.str(), "failures") != "0" ||
      xml_attribute(suite.str(), "skipped") != "0" ||
      std::regex_search(xml, std::regex(R"(<(failure|error|skipped)\b)")))
    invalid("D regression JUnit did not pass");
  std::unordered_set<std::string> cases;
  const std::regex test_case(R"(<testcase\b[^>]*>)");
  for (std::sregex_iterator item(xml.begin(), xml.end(), test_case), end; item != end; ++item) {
    const std::string tag = item->str();
    if (xml_attribute(tag, "status") != "run" ||
        !cases.insert(xml_attribute(tag, "name")).second)
      invalid("D regression JUnit testcase invalid");
  }
  if (cases.size() != test_count) invalid("D regression JUnit testcase count invalid");
  return {xml_attribute(suite.str(), "timestamp"), test_count};
}

void verify_d_regression_provenance(const YAML::Node &provenance, const YAML::Node &activation,
                            const YAML::Node &prebuild, const DeviceBundleIdentity &identity,
                            const NoLoadDevActivationRequest &request,
                            const std::string &junit_timestamp, unsigned test_count) {
  const YAML::Node build_identity = require(prebuild, "bundle_identity");
  const YAML::Node rerun = require(provenance, "rerun_argv");
  const fs::path build_dir(require(provenance, "build_dir").as<std::string>());
  if (require(provenance, "schema").as<std::string>() != "dummy_d_ctest_provenance_v1" ||
      !std::regex_match(require(provenance, "generated_at_utc").as<std::string>(),
                        std::regex(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d+)?Z)")) ||
      require(provenance, "junit_timestamp").as<std::string>() != junit_timestamp ||
      !build_dir.is_absolute() || !rerun.IsSequence() || rerun.size() != 5U ||
      rerun[0].as<std::string>() != "ctest" || rerun[1].as<std::string>() != "--test-dir" ||
      rerun[2].as<std::string>() != build_dir.string() ||
      rerun[3].as<std::string>() != "--output-junit" ||
      rerun[4].as<std::string>() != fs::absolute(request.d_regression_report_path).string() ||
      require(provenance, "source_hash_kind").as<std::string>() != "PREBUILD_INPUTS" ||
      require(provenance, "source_state").as<std::string>() != "DIRTY_UNQUALIFIED" ||
      require(provenance, "source_sha256").as<std::string>() !=
          require(build_identity, "source_sha256").as<std::string>() ||
      require(provenance, "firmware_sha256").as<std::string>() !=
          require(activation, "firmware_sha256").as<std::string>() ||
      require(provenance, "host_sha256").as<std::string>() !=
          require(activation, "host_sha256").as<std::string>() ||
      parse_sha256(require(provenance, "contract_sha256").as<std::string>()) != identity.contract_hash ||
      parse_sha256(require(provenance, "model_sha256").as<std::string>()) != identity.model_hash ||
      parse_sha256(require(provenance, "compiled_policy_sha256").as<std::string>()) != identity.policy_hash ||
      require(provenance, "build_config_sha256").as<std::string>() !=
          require(activation, "build_config_sha256").as<std::string>() ||
      require(provenance, "junit_sha256").as<std::string>() !=
          require(activation, "d_regression_report_sha256").as<std::string>() ||
      require(provenance, "test_count").as<unsigned>() != test_count)
    invalid("D regression provenance binding mismatch");
}

fs::path bounded_evidence_file(const fs::path &root, const std::string &relative) {
  const fs::path path(relative);
  if (relative.empty() || path.is_absolute() ||
      std::find(path.begin(), path.end(), fs::path("..")) != path.end() ||
      !regular_unlinked_payload(root, path)) invalid("HIL evidence path invalid");
  return root / path;
}

void verify_hil_pages_count(const YAML::Node &hil, const YAML::Node &raw_index,
                      const NoLoadDevActivationRequest &request, std::size_t expected_cycles) {
  const YAML::Node cycles = require(raw_index, "cycles");
  const YAML::Node hil_cycles = require(hil, "cycles");
  const YAML::Node boots = require(hil, "boot_epochs");
  if (require(raw_index, "schema").as<std::string>() != "dummy_no_load_dev_functional_raw_index_v1" ||
      !cycles.IsSequence() || cycles.size() != expected_cycles || !hil_cycles.IsSequence() ||
      hil_cycles.size() != expected_cycles || !boots.IsSequence() || boots.size() != 1U)
    invalid("HIL raw index shape invalid");
  const std::string boot = boots[0].as<std::string>();
  if (!std::regex_match(boot, std::regex("[0-9a-f]{16}"))) invalid("HIL boot identity invalid");
  const fs::path root = fs::weakly_canonical(request.hil_evidence_root);
  if (!fs::is_directory(root)) invalid("HIL evidence root unavailable");
  std::unordered_set<std::string> seen_pages;
  for (std::size_t slot = 0; slot < expected_cycles; ++slot) {
    const YAML::Node cycle = cycles[slot], summary = hil_cycles[slot];
    const auto number = static_cast<unsigned>(slot + 1U);
    std::ostringstream name;
    name << "cycle-" << std::setw(3) << std::setfill('0') << number;
    const fs::path directory(name.str());
    const std::string index_file = (directory / "index.json").string();
    const std::string release_run = require(cycle, "run_id").as<std::string>();
    if (require(cycle, "cycle").as<unsigned>() != number ||
        require(cycle, "boot_epoch").as<std::string>() != boot ||
        require(cycle, "index_file").as<std::string>() != index_file ||
        require(summary, "cycle").as<unsigned>() != number || !require(summary, "passed").as<bool>() ||
        require(require(summary, "observed"), "run_id").as<std::string>() != release_run ||
        require(require(summary, "observed"), "evidence_run_id").as<std::string>() != release_run)
      invalid("HIL cycle identity mismatch");
    const fs::path index_path = bounded_evidence_file(root, index_file);
    const YAML::Node original = detail::load_pinned_yaml(
        index_path, require(cycle, "index_sha256").as<std::string>(), {});
    const YAML::Node pages = require(cycle, "pages");
    if (!original.IsSequence() || !pages.IsSequence() || pages.size() != original.size() ||
        pages.size() < 2U) invalid("HIL page coverage incomplete");
    bool motion = false, release = false;
    for (std::size_t page = 0; page < pages.size(); ++page) {
      const YAML::Node bound = pages[page], entry = original[page];
      const std::string relative = require(bound, "file").as<std::string>();
      const fs::path page_path = bounded_evidence_file(root, relative);
      const std::string run_id = require(bound, "run_id").as<std::string>();
      const unsigned sequence = require(bound, "evidence_seq").as<unsigned>();
      if (fs::path(relative).parent_path() != directory ||
          !std::regex_match(page_path.filename().string(), std::regex(R"(page-\d{4}\.bin)")) ||
          !seen_pages.insert(relative).second ||
          require(entry, "file").as<std::string>() != page_path.filename().string() ||
          std::to_string(require(entry, "run_id").as<std::uint64_t>()) != run_id ||
          require(entry, "evidence_seq").as<unsigned>() != sequence ||
          parse_sha256(require(bound, "raw_sha256").as<std::string>()) !=
              parse_sha256(require(entry, "raw_sha256").as<std::string>()) ||
          parse_sha256(require(bound, "raw_sha256").as<std::string>()) !=
              parse_sha256(sha256_file(page_path))) invalid("HIL page identity or SHA mismatch");
      motion |= require(entry, "attempt_kind").as<unsigned>() == 1U &&
                require(entry, "expected_mask").as<unsigned>() == 63U &&
                require(entry, "sent_mask").as<unsigned>() == 63U &&
                require(entry, "failed_mask").as<unsigned>() == 0U;
      release |= require(entry, "attempt_kind").as<unsigned>() == 3U &&
                 require(entry, "release_committed").as<bool>() &&
                 require(entry, "final_state").as<unsigned>() == 8U && run_id == release_run;
    }
    if (!motion || !release) invalid("HIL terminal evidence missing");
    for (const auto &file : fs::directory_iterator(root / directory)) {
      const std::string relative = (directory / file.path().filename()).string();
      if (file.path().filename().string().rfind("page-", 0U) == 0U &&
          seen_pages.find(relative) == seen_pages.end()) invalid("HIL unindexed page present");
    }
  }
}

void verify_hil_pages(const YAML::Node &hil, const YAML::Node &raw_index,
                      const NoLoadDevActivationRequest &request) {
  verify_hil_pages_count(hil, raw_index, request, 50U);
}

void verify_functional_basis(const YAML::Node &activation, const DeviceBundleIdentity &identity,
                             const NoLoadDevActivationRequest &request, const YAML::Node &prebuild) {
  const auto bound_file = [&activation](const char *field, const fs::path &path) {
    return fs::is_regular_file(path) && !fs::is_symlink(path) &&
           parse_sha256(require(activation, field).as<std::string>()) == parse_sha256(sha256_file(path));
  };
  if (!bound_file("d_regression_report_sha256", request.d_regression_report_path) ||
      !bound_file("d_regression_provenance_sha256", request.d_regression_provenance_path) ||
      !bound_file("hil_report_sha256", request.hil_report_path) ||
      !bound_file("hil_raw_index_sha256", request.hil_raw_index_path) ||
      !bound_file("elf_sha256", request.elf_path) ||
      activation["qualification_report_sha256"])
    invalid("functional development evidence binding missing");
  const YAML::Node resource = detail::load_pinned_yaml(
      request.resource_report_path, require(activation, "resource_report_sha256").as<std::string>(), {});
  fs::path reported_elf(require(resource, "elf").as<std::string>());
  if (!reported_elf.is_absolute()) {
    if (request.resource_base_root.empty()) invalid("resource ELF base root missing");
    reported_elf = fs::path(request.resource_base_root) / reported_elf;
  }
  if (require(resource, "result").as<std::string>() != "FAIL" ||
      require(resource, "measurement_provenance").as<std::string>() != "NOT_ASSESSED" ||
      require(resource, "measurement_source").as<std::string>() != "static_link_only" ||
      require(resource, "release_qualified").as<bool>() ||
      fs::weakly_canonical(reported_elf) != fs::weakly_canonical(request.elf_path))
    invalid("functional resource report is not static-only");
  const auto [timestamp, test_count] = verify_d_regression_junit(
      request.d_regression_report_path,
      require(activation, "d_regression_report_sha256").as<std::string>());
  const YAML::Node provenance = detail::load_pinned_yaml(
      request.d_regression_provenance_path,
      require(activation, "d_regression_provenance_sha256").as<std::string>(), {});
  verify_d_regression_provenance(
      provenance, activation, prebuild, identity, request, timestamp, test_count);
  const YAML::Node hil = detail::load_pinned_yaml(
      request.hil_report_path, require(activation, "hil_report_sha256").as<std::string>(), {});
  if (require(hil, "format").as<std::string>() != "dummy_v2_hil_v2" ||
      require(hil, "qualification_scope").as<std::string>() != "APPLICATION_HOLD" ||
      !require(hil, "application_hold").as<bool>() ||
      require(hil, "result").as<std::string>() != "PASS" ||
      require(hil, "requested_cycles").as<unsigned>() != 50U ||
      require(hil, "executed_cycles").as<unsigned>() != 50U ||
      require(hil, "passed_cycles").as<unsigned>() != 50U ||
      parse_sha256(require(hil, "firmware_sha256").as<std::string>()) != identity.firmware_hash ||
      parse_sha256(require(hil, "cli_sha256").as<std::string>()) !=
          parse_sha256(require(activation, "host_sha256").as<std::string>()) ||
      parse_sha256(require(hil, "contract_sha256").as<std::string>()) != identity.contract_hash ||
      parse_sha256(require(hil, "model_sha256").as<std::string>()) != identity.model_hash ||
      parse_sha256(require(hil, "profile_sha256").as<std::string>()) != identity.policy_hash ||
      require(hil, "profile_id").as<unsigned>() != 201U ||
      require(hil, "configured_mask").as<unsigned>() != 63U)
    invalid("functional HIL report mismatch");
  verify_hil_pages(hil, detail::load_pinned_yaml(
      request.hil_raw_index_path, require(activation, "hil_raw_index_sha256").as<std::string>(), {}), request);
}

void verify_limited_functional_basis(const YAML::Node &activation, const DeviceBundleIdentity &identity,
                             const NoLoadDevActivationRequest &request, const YAML::Node &prebuild) {
  const auto limit_text = require(activation, "goal_limit").as<std::string>();
  const auto operator_text = require(activation, "operator_powered_fixed_no_load").as<std::string>();
  const fs::path timeline(require(activation, "goal_timeline_path").as<std::string>());
  if (require(activation, "goal_limit").Tag() != "?" ||
      require(activation, "operator_powered_fixed_no_load").Tag() != "?" ||
      (limit_text != "1" && limit_text != "2") || operator_text != "true" ||
      !timeline.is_absolute() || timeline.filename().empty() || fs::exists(timeline) ||
      fs::is_symlink(timeline)) invalid("limited functional budget or invocation invalid");
  const auto bound_file = [&activation](const char *field, const fs::path &path) {
    return fs::is_regular_file(path) && !fs::is_symlink(path) &&
           parse_sha256(require(activation, field).as<std::string>()) == parse_sha256(sha256_file(path));
  };
  if (!bound_file("d_regression_report_sha256", request.d_regression_report_path) ||
      !bound_file("d_regression_provenance_sha256", request.d_regression_provenance_path) ||
      !bound_file("hil_report_sha256", request.hil_report_path) ||
      !bound_file("hil_raw_index_sha256", request.hil_raw_index_path) ||
      !bound_file("elf_sha256", request.elf_path) ||
      activation["qualification_report_sha256"])
    invalid("functional development evidence binding missing");
  const YAML::Node resource = detail::load_pinned_yaml(
      request.resource_report_path, require(activation, "resource_report_sha256").as<std::string>(), {});
  fs::path reported_elf(require(resource, "elf").as<std::string>());
  if (!reported_elf.is_absolute()) {
    if (request.resource_base_root.empty()) invalid("resource ELF base root missing");
    reported_elf = fs::path(request.resource_base_root) / reported_elf;
  }
  if (require(resource, "result").as<std::string>() != "FAIL" ||
      require(resource, "measurement_provenance").as<std::string>() != "NOT_ASSESSED" ||
      require(resource, "measurement_source").as<std::string>() != "static_link_only" ||
      require(resource, "release_qualified").as<bool>() ||
      fs::weakly_canonical(reported_elf) != fs::weakly_canonical(request.elf_path))
    invalid("functional resource report is not static-only");
  const auto [timestamp, test_count] = verify_d_regression_junit(
      request.d_regression_report_path,
      require(activation, "d_regression_report_sha256").as<std::string>());
  const YAML::Node provenance = detail::load_pinned_yaml(
      request.d_regression_provenance_path,
      require(activation, "d_regression_provenance_sha256").as<std::string>(), {});
  verify_d_regression_provenance(
      provenance, activation, prebuild, identity, request, timestamp, test_count);
  const YAML::Node hil = detail::load_pinned_yaml(
      request.hil_report_path, require(activation, "hil_report_sha256").as<std::string>(), {});
  for (const char *field : {"requested_cycles", "executed_cycles", "passed_cycles"}) {
    if (require(hil, field).Tag() != "?" || require(hil, field).as<std::string>() != "1")
      invalid("limited Native APP count must be integer one");
  }
  if (require(hil, "format").as<std::string>() != "dummy_v2_hil_v2" ||
      require(hil, "qualification_scope").as<std::string>() != "APPLICATION_HOLD" ||
      !require(hil, "application_hold").as<bool>() ||
      require(hil, "result").as<std::string>() != "PASS" ||
      require(hil, "requested_cycles").as<unsigned>() != 1U ||
      require(hil, "executed_cycles").as<unsigned>() != 1U ||
      require(hil, "passed_cycles").as<unsigned>() != 1U ||
      parse_sha256(require(hil, "firmware_sha256").as<std::string>()) != identity.firmware_hash ||
      parse_sha256(require(hil, "cli_sha256").as<std::string>()) !=
          parse_sha256(require(activation, "host_sha256").as<std::string>()) ||
      parse_sha256(require(hil, "contract_sha256").as<std::string>()) != identity.contract_hash ||
      parse_sha256(require(hil, "model_sha256").as<std::string>()) != identity.model_hash ||
      parse_sha256(require(hil, "profile_sha256").as<std::string>()) != identity.policy_hash ||
      require(hil, "profile_id").as<unsigned>() != 201U ||
      require(hil, "configured_mask").as<unsigned>() != 63U)
    invalid("functional HIL report mismatch");
  verify_hil_pages_count(hil, detail::load_pinned_yaml(
      request.hil_raw_index_path, require(activation, "hil_raw_index_sha256").as<std::string>(), {}), request, 1U);
}

struct NoLoadActivationIdentity {
  bool functional_basis;
  NoLoadBasis basis;
  std::array<std::uint8_t, 32U> live_profile_hash;
};

NoLoadActivationIdentity verify_no_load_activation(const VerifiedBundle &candidate,
                                                   const DeviceBundleIdentity &identity,
                                                   const NoLoadDevActivationRequest &request) {
  const fs::path activation_path(request.activation_path);
  if (!fs::is_regular_file(activation_path) || fs::is_symlink(activation_path))
    invalid("activation file hash mismatch");
  const YAML::Node activation = detail::load_pinned_yaml(
      activation_path, request.expected_activation_sha256, {});
  if (require(activation, "schema").as<std::string>() != "dummy_no_load_dev_activation_v1" ||
      require(activation, "scope").as<std::string>() != "NO_LOAD_DEV" ||
      require(activation, "qualification").as<std::string>() != "UNQUALIFIED")
    invalid("invalid no-load development scope");
  const YAML::Node basis = activation["basis"];
  const bool limited_basis = basis && basis.as<std::string>() == "NO_LOAD_DEV_FUNCTIONAL_LIMITED";
  const bool functional_basis = limited_basis || (basis && basis.as<std::string>() == "NO_LOAD_DEV_FUNCTIONAL");
  if (basis && !functional_basis && basis.as<std::string>() != "SOFTWARE_ONLY")
    invalid("unverified no-load development basis");
  const auto equal_hash = [&activation](const char *field, const std::array<std::uint8_t, 32U> &expected) {
    const auto actual = parse_sha256(require(activation, field).as<std::string>());
    return std::any_of(expected.begin(), expected.end(), [](auto byte) { return byte != 0U; }) &&
           actual == expected;
  };
  const auto equal_file_hash = [&equal_hash](const char *field, const fs::path &path) {
    return fs::is_regular_file(path) && !fs::is_symlink(path) &&
           equal_hash(field, parse_sha256(sha256_file(path)));
  };
  const auto source = verify_bundle(request.source_bundle_root);
  const fs::path prebuild_path(request.prebuild_bundle_manifest_path);
  const fs::path compiled_path(request.compiled_policy_manifest_path);
  if (!fs::is_regular_file(prebuild_path) || fs::is_symlink(prebuild_path) ||
      !fs::is_regular_file(compiled_path) || fs::is_symlink(compiled_path))
    invalid("no-load development binding missing");
  const std::string prebuild_sha256 = require(activation, "prebuild_bundle_manifest_sha256").as<std::string>();
  const YAML::Node prebuild = detail::load_pinned_yaml(prebuild_path, prebuild_sha256, {});
  const YAML::Node compiled = load_yaml_snapshot(compiled_path);
  const auto source_model = source.payloads.find("model/model.yaml");
  if (!equal_hash("candidate_manifest_sha256", parse_sha256(candidate.manifest_sha256)) ||
      !equal_hash("candidate_payload_sorted_sha256", identity.bundle_hash) ||
      !equal_hash("source_wire_manifest_sha256", parse_sha256(source.manifest_sha256)) ||
      !equal_hash("source_contract_sha256", identity.contract_hash) ||
      identity.contract_hash != parse_sha256(source.aggregate) ||
      source_model == source.payloads.end() ||
      identity.model_hash != parse_sha256(source_model->second) ||
      !equal_hash("model_sha256", identity.model_hash) ||
      !equal_hash("profile_sha256", identity.candidate_profile_payload_hash) ||
      !equal_hash("compiled_policy_sha256", identity.policy_hash) ||
      !equal_hash("firmware_sha256", identity.firmware_hash) ||
      !equal_hash("prebuild_bundle_manifest_sha256", parse_sha256(prebuild_sha256)) ||
      require(prebuild, "format").as<std::string>() != "dummy_robot_bundle_v1" ||
      require(prebuild, "profile_id").as<std::uint32_t>() != 201U ||
      require(prebuild, "configured_mask").as<std::uint32_t>() != 63U ||
      parse_sha256(require(prebuild, "contract_sha256").as<std::string>()) != identity.contract_hash ||
      !equal_hash("prebuild_model_sha256", parse_sha256(require(prebuild, "model_sha256").as<std::string>())) ||
      !equal_hash("prebuild_profile_sha256", parse_sha256(require(prebuild, "profile_sha256").as<std::string>())) ||
      !equal_hash("prebuild_compiled_policy_sha256",
                  parse_sha256(require(prebuild, "compiled_policy_sha256").as<std::string>())) ||
      !equal_hash("build_config_sha256",
                  parse_sha256(require(require(prebuild, "bundle_identity"), "build_config_sha256").as<std::string>())) ||
      require(compiled, "format").as<std::string>() != "dummy_compiled_policy_manifest_v1" ||
      !require(compiled, "runtime_enabled").as<bool>() ||
      (functional_basis ?
           parse_sha256(require(compiled, "policy_sha256").as<std::string>()) != identity.policy_hash :
           !equal_hash("prebuild_compiled_policy_sha256",
                       parse_sha256(require(compiled, "policy_sha256").as<std::string>()))) ||
      parse_sha256(require(compiled, "firmware_image_sha256").as<std::string>()) != identity.firmware_hash ||
      !equal_file_hash("firmware_sha256", request.firmware_image_path) ||
      !equal_file_hash("host_sha256", request.host_executable_path) ||
      !equal_file_hash("map_sha256", request.map_file_path) ||
      !equal_file_hash("resource_report_sha256", request.resource_report_path))
    invalid("no-load development binding mismatch");
  if (functional_basis) {
    if (!equal_hash("live_profile_sha256", identity.policy_hash))
      invalid("functional live profile identity mismatch");
    if (compiled["post_build_bindings"]) invalid("functional basis cannot claim formal post-build bindings");
    if (limited_basis) verify_limited_functional_basis(activation, identity, request, prebuild);
    else verify_functional_basis(activation, identity, request, prebuild);
  } else {
    const YAML::Node post_build = require(compiled, "post_build_bindings");
    if (!equal_file_hash("qualification_report_sha256", request.qualification_report_path) ||
        parse_sha256(require(post_build, "bundle_manifest_sha256").as<std::string>()) !=
            parse_sha256(prebuild_sha256) ||
        parse_sha256(require(post_build, "firmware_sha256").as<std::string>()) != identity.firmware_hash ||
        !equal_hash("host_sha256", parse_sha256(require(post_build, "host_sha256").as<std::string>())) ||
        !equal_hash("map_sha256", parse_sha256(require(post_build, "map_sha256").as<std::string>())) ||
        !equal_hash("resource_report_sha256",
                    parse_sha256(require(post_build, "resource_report_sha256").as<std::string>())) ||
        !equal_hash("qualification_report_sha256",
                    parse_sha256(require(post_build, "qualification_report_sha256").as<std::string>())))
      invalid("software-only post-build binding mismatch");
    const YAML::Node report = detail::load_pinned_yaml(
        request.qualification_report_path,
        require(activation, "qualification_report_sha256").as<std::string>(), {});
    if (require(report, "qualification_scope").as<std::string>() != "SOFTWARE_ONLY" ||
        require(report, "result").as<std::string>() != "PASS")
      invalid("software-only qualification report required");
  }
  static const std::regex id_pattern("[0-9a-f]{32}");
  static const std::regex serial_pattern("[A-Za-z0-9]+");
  if (!std::regex_match(request.device_id, id_pattern) ||
      request.device_id == std::string(32U, '0') ||
      !std::regex_match(request.usb_serial, serial_pattern) ||
      require(activation, "device_id").as<std::string>() != request.device_id ||
      require(activation, "usb_vid").as<unsigned>() != 0x1209U ||
      require(activation, "usb_pid").as<unsigned>() != 0x7461U ||
      require(activation, "usb_interface").as<unsigned>() != 0U ||
      require(activation, "usb_serial").as<std::string>() != request.usb_serial ||
      request.endpoint != "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_" +
                          request.usb_serial + "-if00#baudrate=115200" ||
      require(activation, "endpoint").as<std::string>() != request.endpoint)
    invalid("no-load development device identity mismatch");
  return {functional_basis, limited_basis ? NoLoadBasis::kFunctionalLimited :
          (functional_basis ? NoLoadBasis::kFunctional50 : NoLoadBasis::kSoftwareOnly),
          functional_basis ? parse_sha256(require(activation, "live_profile_sha256").as<std::string>()) :
                             identity.profile_hash};
}
}  // namespace

namespace detail {
YAML::Node load_pinned_yaml(const fs::path &path, const std::string &expected_sha256,
                            const std::function<void()> &after_snapshot) {
  const std::string bytes = read_file(path);
  if (parse_sha256(sha256_text(bytes)) != parse_sha256(expected_sha256))
    invalid("bundle payload hash mismatch");
  if (after_snapshot) after_snapshot();
  return YAML::Load(bytes);
}
}  // namespace detail

DeviceBundleIdentity inspect_device_bundle_identity(const std::string &bundle_root,
                                                    const std::string &profile_name) {
  try {
    const auto bundle = verify_bundle(bundle_root);
    return bundle_identity(bundle, profile_name);
  } catch (const std::invalid_argument &) {
    throw;
  } catch (const std::exception &) {
    invalid("invalid device profile bundle");
  }
}

bool status_matches_bundle_identity(const DeviceBundleIdentity &expected,
                                    const DeviceStatusIdentity &actual) noexcept {
  const bool firmware_present = std::any_of(actual.firmware_hash.begin(), actual.firmware_hash.end(),
                                            [](auto byte) { return byte != 0U; });
  return expected.contract_hash == actual.contract_hash && expected.model_hash == actual.model_hash &&
         expected.profile_hash == actual.profile_hash && expected.policy_hash == actual.policy_hash &&
         firmware_present && (!expected.firmware_bound || expected.firmware_hash == actual.firmware_hash);
}

std::uint64_t validated_motion_permit_limit(std::uint64_t goal_us,
    std::uint64_t trial_us, std::uint64_t stop_us,
    std::uint64_t completion_us, std::uint64_t hold_us,
    std::optional<std::uint64_t> explicit_permit_us) {
  constexpr std::uint64_t kMaximumBudgetUs=3600000000ULL;
  if(goal_us==0U || trial_us==0U || stop_us==0U || completion_us==0U ||
     hold_us==0U || trial_us>kMaximumBudgetUs || goal_us>trial_us ||
     stop_us>trial_us-goal_us ||
     completion_us>trial_us-goal_us-stop_us ||
     hold_us>trial_us-goal_us-stop_us-completion_us)
    invalid("invalid bounded motion permit policy");
  if(!explicit_permit_us && goal_us>=600000000U)
    invalid("long goal requires explicit motion permit budget");
  const auto permit=explicit_permit_us.value_or(std::min<std::uint64_t>(60000000U,trial_us));
  if(permit==0U || permit>trial_us ||
     permit<goal_us+stop_us+completion_us+hold_us)
    invalid("motion permit does not cover bounded goal and cleanup");
  return permit;
}

DeviceProfile load_device_profile_impl(const std::string &bundle_root, const std::string &profile_name,
                                       std::uint32_t expected_id, std::uint8_t expected_mask,
                                       const NoLoadDevActivationRequest *activation) {
  try {
    if (!safe_profile_name(profile_name) || expected_id == 0U || (expected_mask != 1U && expected_mask != 63U))
      invalid("invalid requested device profile");
    const auto bundle = verify_bundle(bundle_root);
    if ((profile_name == "sil_j1" || profile_name == "sil_6dof") &&
        bundle.aggregate != "0afb58ddfe9ce339c75da3c527b31abe875a261b3ac270e89d445ed96eda497b")
      invalid("untrusted SIL fixture bundle");
    const auto identity = bundle_identity(bundle, profile_name);
    DeviceProfile output;
    static_cast<DeviceBundleIdentity &>(output) = identity;
    const std::string model_key = "model/model.yaml";
    const std::string profile_key = "profiles/" + profile_name + ".yaml";
    const YAML::Node model = detail::load_pinned_yaml(
        bundle.root / model_key, bundle.payloads.at(model_key), {});
    const YAML::Node profile = detail::load_pinned_yaml(
        bundle.root / profile_key, bundle.payloads.at(profile_key), {});
    const bool bench_only = profile_name == "f405_6dof_bench";
    if (require(profile, "profile_name").as<std::string>() != profile_name ||
        require(profile, "profile_id").as<std::uint32_t>() != expected_id ||
        !require(profile, "uses_dpsdk_device").as<bool>() ||
        require(profile, "motion_permitted_on_real_hardware").as<bool>()) invalid("profile authority mismatch");
    if (bench_only) {
      const YAML::Node capability = require(bundle.manifest, "capability");
      if (expected_id != 201U || expected_mask != 63U || require(profile, "test_only").as<bool>() ||
          !require(profile, "motion_supported").as<bool>() ||
          require(profile, "reference_kind").as<std::string>() != "BENCH_SESSION" ||
          require(bundle.manifest, "runtime_enabled").as<bool>() ||
          require(capability, "profile_id").as<std::uint32_t>() != expected_id ||
          require(capability, "configured_mask").as<std::uint32_t>() != expected_mask ||
          require(capability, "motion_permitted_on_real_hardware").as<bool>())
        invalid("bench profile authority mismatch");
      if (require(bundle.manifest, "artifact_kind").as<std::string>() != "application_hold_no_load_candidate" ||
          require(profile, "no_load_bench").as<bool>() != true ||
          require(profile, "gravity_loaded").as<bool>() != false ||
          require(profile, "deployment_stop_policy").as<std::string>() != "APPLICATION_HOLD" ||
          require(profile, "safe_terminal_action").as<std::string>() != "DisableAndReleaseNoLoadBench" ||
          activation == nullptr) invalid("no-load development activation required");
      const auto activated = verify_no_load_activation(bundle, identity, *activation);
      output.functional_development_basis = activated.functional_basis;
      output.no_load_basis = activated.basis;
      if (activated.basis == NoLoadBasis::kFunctionalLimited) {
        const auto limited = detail::load_pinned_yaml(
            activation->activation_path, activation->expected_activation_sha256, {});
        output.goal_limit = require(limited, "goal_limit").as<unsigned>();
        output.limited_goal_timeline_path = require(limited, "goal_timeline_path").as<std::string>();
        const auto hil = detail::load_pinned_yaml(activation->hil_report_path,
            require(limited, "hil_report_sha256").as<std::string>(), {});
        const auto boot_text = require(hil, "boot_epochs")[0].as<std::string>();
        const auto parsed = std::from_chars(boot_text.data(), boot_text.data()+boot_text.size(),
                                            output.limited_boot_epoch, 16);
        if(parsed.ec!=std::errc{} || parsed.ptr!=boot_text.data()+boot_text.size() ||
           output.limited_boot_epoch==0U) invalid("limited functional boot invalid");
      }
      output.profile_hash = activated.live_profile_hash;
      output.bench_only = true;
      output.software_only_development = !output.functional_development_basis;
    } else if (!require(profile, "test_only").as<bool>()) invalid("profile authority mismatch");
    const YAML::Node active = require(profile, "active_axes");
    const YAML::Node model_joints = require(model, "joints");
    if (!active.IsSequence() || !model_joints.IsSequence() || (active.size() != 1U && active.size() != 6U)) invalid("invalid active axes");
    const bool relative_sil = profile_name == "sil_6dof";
    YAML::Node bench_axes;
    if (bench_only) {
      const std::string policy_key = "policy/application_hold_no_load.json";
      if (require(profile, "source_policy").as<std::string>() != policy_key ||
          bundle.payloads.find(policy_key) == bundle.payloads.end())
        invalid("bench policy payload missing");
      const YAML::Node policy = detail::load_pinned_yaml(
          bundle.root / policy_key, bundle.payloads.at(policy_key), {});
      const YAML::Node timing = require(policy, "timing");
      std::optional<std::uint64_t> explicit_permit;
      if(timing["motion_permit_us"])
        explicit_permit=timing["motion_permit_us"].as<std::uint64_t>();
      output.motion_permit_max_us=validated_motion_permit_limit(
          require(timing,"max_goal_duration_us").as<std::uint64_t>(),
          require(timing,"trial_max_duration_us").as<std::uint64_t>(),
          require(timing,"stop_max_duration_us").as<std::uint64_t>(),
          require(timing,"completion_max_duration_us").as<std::uint64_t>(),
          require(timing,"local_hold_budget_us").as<std::uint64_t>(),
          explicit_permit);
      const YAML::Node policy_active = require(policy, "active_axes");
      bench_axes = require(policy, "axes");
      if (require(policy, "reference_kind").as<std::string>() != "BENCH_SESSION" ||
          require(policy, "model_sha256").as<std::string>() != bundle.payloads.at(model_key) ||
          !policy_active.IsSequence() || policy_active.size() != active.size() ||
          !bench_axes.IsSequence() || bench_axes.size() != active.size())
        invalid("bench policy reference frame mismatch");
      for (std::size_t axis = 0U; axis < active.size(); ++axis)
        if (policy_active[axis].as<std::string>() != active[axis].as<std::string>())
          invalid("bench policy axis order mismatch");
    }
    double trial_lower = 0.0, trial_upper = 0.0;
    if (relative_sil) {
      if (require(profile, "reference_kind").as<std::string>() != "BENCH_SESSION" ||
          !require(profile, "test_only").as<bool>()) invalid("SIL reference frame mismatch");
      const YAML::Node range = require(profile, "bench_position_range_rad");
      const double travel = finite_positive(require(require(profile, "simulation"), "max_trial_travel_rad"));
      if (!range.IsSequence() || range.size() != 2U) invalid("invalid SIL trial range");
      trial_lower = range[0U].as<double>();
      trial_upper = range[1U].as<double>();
      if (!std::isfinite(trial_lower) || !std::isfinite(trial_upper) || trial_lower >= 0.0 ||
          trial_upper <= 0.0 || trial_lower < -travel || trial_upper > travel)
        invalid("SIL trial range exceeds Core travel");
    }
    std::unordered_map<std::string, YAML::Node> limits;
    for (const YAML::Node &joint : model_joints) {
      const std::string name = require(joint, "name").as<std::string>();
      if (!limits.emplace(name, require(joint, "model_limits_rad")).second) invalid("duplicate model joint");
    }
    std::uint8_t mask = 0U;
    for (std::size_t axis_index = 0U; axis_index < active.size(); ++axis_index) {
      const YAML::Node axis = active[axis_index];
      const std::string name = axis.as<std::string>();
      const std::uint8_t bit = axis_bit(name);
      if ((mask & bit) != 0U || limits.find(name) == limits.end()) invalid("profile/model axis mismatch");
      const YAML::Node range = limits.at(name);
      if (!range.IsSequence() || range.size() != 2U) invalid("invalid model limits");
      const double mechanical_lower = range[0U].as<double>(), mechanical_upper = range[1U].as<double>();
      if (!std::isfinite(mechanical_lower) || !std::isfinite(mechanical_upper) ||
          mechanical_lower >= mechanical_upper) invalid("invalid model limits");
      double lower = relative_sil ? trial_lower : mechanical_lower;
      double upper = relative_sil ? trial_upper : mechanical_upper;
      if (bench_only) {
        lower = require(bench_axes[axis_index], "trial_min_rad").as<double>();
        upper = require(bench_axes[axis_index], "trial_max_rad").as<double>();
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= 0.0 ||
            upper <= 0.0) invalid("invalid bench policy trial limits");
      }
      output.joints.joints.push_back(name); output.joints.lower.push_back(lower); output.joints.upper.push_back(upper); mask |= bit;
    }
    if (mask != expected_mask) invalid("configured mask drift");
    const YAML::Node simulation = require(profile, "simulation");
    const double velocity = finite_positive(require(simulation, "velocity_limit_rad_s"));
    const double acceleration = finite_positive(require(simulation, "acceleration_limit_rad_s2"));
    const double path_tolerance = finite_positive(require(simulation, "max_tracking_error_rad"));
    const double goal_tolerance = finite_positive(require(simulation, "hold_position_tolerance_rad"));
    output.joints.velocity.assign(active.size(), velocity);
    output.joints.acceleration.assign(active.size(), acceleration);
    output.joints.path_tolerance.assign(active.size(), path_tolerance);
    output.joints.goal_tolerance.assign(active.size(), goal_tolerance);
    return output;
  } catch (const std::invalid_argument &) {
    throw;
  } catch (const std::exception &) {
    invalid("invalid device profile bundle");
  }
}

DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask) {
  return load_device_profile_impl(bundle_root, profile_name, expected_id, expected_mask, nullptr);
}

DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask,
                                  const NoLoadDevActivationRequest &activation) {
  return load_device_profile_impl(bundle_root, profile_name, expected_id, expected_mask, &activation);
}
}  // namespace dummy_motion_client
