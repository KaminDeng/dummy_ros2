#include "dummy_motion_client/device_profile.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

#include <openssl/evp.h>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

namespace dummy_motion_client {
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
}  // namespace

DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask) {
  try {
    if (!safe_profile_name(profile_name) || expected_id == 0U || (expected_mask != 1U && expected_mask != 63U))
      invalid("invalid requested device profile");
    const fs::path root = fs::weakly_canonical(fs::path(bundle_root));
    if (!fs::is_directory(root)) invalid("bundle root unavailable");
    const fs::path manifest_path = root / "manifest.json";
    const YAML::Node manifest = YAML::LoadFile(manifest_path.string());
    if (require(manifest, "format").as<std::string>() != "dummy_wire_release_v1" ||
        require(manifest, "release_id").as<std::string>() != "dummy-wire-v2") invalid("unsupported bundle release");
    const YAML::Node payloads = require(manifest, "payloads");
    if (!payloads.IsSequence() || payloads.size() == 0U) invalid("invalid bundle payload list");
    std::unordered_map<std::string, std::string> verified;
    std::string listing, previous;
    for (const YAML::Node &payload : payloads) {
      const std::string relative = require(payload, "path").as<std::string>();
      const std::string expected = require(payload, "sha256").as<std::string>();
      const fs::path path(relative);
      if (path.is_absolute() || relative.find("..") != std::string::npos || !fs::is_regular_file(root / path) ||
          sha256_file(root / path) != expected || !verified.emplace(relative, expected).second ||
          (!previous.empty() && previous >= relative)) invalid("bundle payload hash mismatch");
      if (!listing.empty()) listing.push_back('\n');
      listing += relative + ":" + expected;
      previous = relative;
    }
    static constexpr std::string_view kC05Contract = "39c460051675e0defe3529cb6c6e5ec518327576a3c8ed1607522a612daebaf1";
    const std::string aggregate = require(manifest, "payload_sorted_sha256").as<std::string>();
    if (aggregate != sha256_text(listing) || aggregate != kC05Contract) invalid("bundle aggregate hash mismatch");
    const std::string model_key = "model/model.yaml";
    const std::string profile_key = "profiles/" + profile_name + ".yaml";
    if (verified.find(model_key) == verified.end() || verified.find(profile_key) == verified.end()) invalid("unmanifested model or profile");
    DeviceProfile output;
    output.contract_hash = parse_sha256(aggregate);
    output.model_hash = parse_sha256(verified.at(model_key));
    output.profile_hash = parse_sha256(verified.at(profile_key));
    const YAML::Node model = YAML::LoadFile((root / model_key).string());
    const YAML::Node profile = YAML::LoadFile((root / profile_key).string());
    if (require(profile, "profile_name").as<std::string>() != profile_name ||
        require(profile, "profile_id").as<std::uint32_t>() != expected_id ||
        !require(profile, "uses_dpsdk_device").as<bool>() || !require(profile, "test_only").as<bool>() ||
        require(profile, "motion_permitted_on_real_hardware").as<bool>()) invalid("profile authority mismatch");
    const YAML::Node active = require(profile, "active_axes");
    const YAML::Node model_joints = require(model, "joints");
    if (!active.IsSequence() || !model_joints.IsSequence() || (active.size() != 1U && active.size() != 6U)) invalid("invalid active axes");
    std::unordered_map<std::string, YAML::Node> limits;
    for (const YAML::Node &joint : model_joints) {
      const std::string name = require(joint, "name").as<std::string>();
      if (!limits.emplace(name, require(joint, "model_limits_rad")).second) invalid("duplicate model joint");
    }
    std::uint8_t mask = 0U;
    for (const YAML::Node &axis : active) {
      const std::string name = axis.as<std::string>();
      const std::uint8_t bit = axis_bit(name);
      if ((mask & bit) != 0U || limits.find(name) == limits.end()) invalid("profile/model axis mismatch");
      const YAML::Node range = limits.at(name);
      if (!range.IsSequence() || range.size() != 2U) invalid("invalid model limits");
      const double lower = range[0U].as<double>(), upper = range[1U].as<double>();
      if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) invalid("invalid model limits");
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
}  // namespace dummy_motion_client
