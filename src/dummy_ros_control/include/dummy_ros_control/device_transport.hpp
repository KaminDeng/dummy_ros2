#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "dummy_ros_control/command_producer.hpp"

namespace dummy_ros_control {
class GuardedZenbeddedSystem;
struct DeviceManagementChannel;

struct TransportParameters {
  std::string endpoint;
  std::array<std::uint8_t, 16> device_id{};
  std::array<std::uint8_t, 16> client_instance_id{};
  std::uint32_t profile_id = 0U;
  std::uint8_t configured_mask = 0U;
  std::uint64_t max_state_age_us = 100000U;
  std::uint64_t state_publish_period_us = 10000U;
  std::array<std::uint8_t, 32> contract_hash{};
  std::array<std::uint8_t, 32> model_hash{};
  std::array<std::uint8_t, 32> profile_hash{};
  std::array<std::uint8_t, 32> firmware_hash{};
  std::array<std::uint8_t, 32> policy_hash{};
  std::array<double, 6> position_min{};
  std::array<double, 6> position_max{};
  std::array<double, 6> velocity_max{};
  std::array<double, 6> acceleration_max{};
  std::uint64_t control_period_us = 0U;
};

// The current v2 status cannot carry policy, limits, or timebase evidence.
// These fields remain explicit so bootstrap fails closed until the wire format
// can prove the complete position-only contract.
struct PositionLimit {
  double lower = 0.0;
  double upper = 0.0;
};

constexpr std::uint64_t kPositionOnlyCapability = 1ULL << 2U;

struct PositionOnlyCapability {
  std::array<std::uint8_t, 32> contract_hash{};
  std::array<std::uint8_t, 32> model_hash{};
  std::array<std::uint8_t, 32> profile_hash{};
  std::array<std::uint8_t, 32> firmware_hash{};
  std::array<std::uint8_t, 32> policy_hash{};
  std::array<PositionLimit, 6> limits{};
  std::array<double, 6> velocity_max{};
  std::array<double, 6> acceleration_max{};
  std::uint64_t capabilities = 0U;
  std::uint64_t control_timebase_us = 0U;
  std::uint64_t state_publish_period_us = 0U;
  std::uint8_t configured_mask = 0U;
  bool has_policy_hash = true;
  bool has_limits = true;
  bool has_control_timebase = true;
};

inline bool position_only_capability_matches(const PositionOnlyCapability &expected,
                                             const PositionOnlyCapability &observed) noexcept {
  if (!(expected.has_policy_hash && expected.has_limits && expected.has_control_timebase &&
        observed.has_policy_hash && observed.has_limits && observed.has_control_timebase &&
        expected.configured_mask == 0x3fU && observed.configured_mask == expected.configured_mask &&
        expected.capabilities == kPositionOnlyCapability &&
        (observed.capabilities & expected.capabilities) == expected.capabilities &&
        expected.control_timebase_us != 0U && expected.state_publish_period_us != 0U &&
        observed.control_timebase_us == expected.control_timebase_us &&
        observed.state_publish_period_us == expected.state_publish_period_us &&
        expected.contract_hash == observed.contract_hash && expected.model_hash == observed.model_hash &&
        expected.profile_hash == observed.profile_hash && expected.firmware_hash == observed.firmware_hash &&
        expected.policy_hash == observed.policy_hash)) return false;
  for (std::size_t axis = 0U; axis < expected.limits.size(); ++axis) {
    if (expected.limits[axis].lower != observed.limits[axis].lower ||
        expected.limits[axis].upper != observed.limits[axis].upper ||
        !std::isfinite(expected.limits[axis].lower) || !std::isfinite(expected.limits[axis].upper) ||
        expected.limits[axis].lower >= expected.limits[axis].upper ||
        !std::isfinite(expected.velocity_max[axis]) || expected.velocity_max[axis] <= 0.0 ||
        !std::isfinite(expected.acceleration_max[axis]) || expected.acceleration_max[axis] <= 0.0 ||
        observed.velocity_max[axis] != expected.velocity_max[axis] ||
        observed.acceleration_max[axis] != expected.acceleration_max[axis]) return false;
  }
  return true;
}

inline bool position_only_positions_valid(const std::array<double, 6> &position,
                                          const PositionOnlyCapability &capability) noexcept {
  if (!capability.has_limits || capability.configured_mask != 0x3fU) return false;
  for (std::size_t axis = 0U; axis < position.size(); ++axis) {
    const auto &limit = capability.limits[axis];
    if (!std::isfinite(position[axis]) || !std::isfinite(limit.lower) ||
        !std::isfinite(limit.upper) || limit.lower > limit.upper ||
        position[axis] < limit.lower || position[axis] > limit.upper) return false;
  }
  return true;
}

enum class HeartbeatAction : std::uint8_t { kNone, kWarn, kStop };

class TransportHeartbeat {
 public:
  explicit TransportHeartbeat(std::uint64_t period_us) noexcept : period_us_(period_us) {}
  void observe_state(std::uint64_t now_us) noexcept { last_state_us_ = now_us; }
  HeartbeatAction poll(std::uint64_t now_us) const noexcept {
    if (last_state_us_ == 0U || now_us <= last_state_us_ || period_us_ == 0U) return HeartbeatAction::kNone;
    const auto elapsed = now_us - last_state_us_;
    if (elapsed >= period_us_ * 5U) return HeartbeatAction::kStop;
    if (elapsed >= period_us_ * 2U) return HeartbeatAction::kWarn;
    return HeartbeatAction::kNone;
  }

 private:
  std::uint64_t period_us_ = 0U;
  std::uint64_t last_state_us_ = 0U;
};

// At most one message may be awaiting firmware commitment and one newer
// message may wait behind it. Transport acceptance is deliberately not an ack.
class LatestOnlyCommandStream {
 public:
  void offer(const CommandGenerationRecord &record) noexcept { pending_ = record; }
  std::optional<CommandGenerationRecord> begin_send() noexcept {
    if (in_flight_.has_value()) return std::nullopt;
    if (!pending_.has_value()) return std::nullopt;
    in_flight_ = pending_;
    pending_.reset();
    return in_flight_;
  }
  std::optional<CommandGenerationRecord> in_flight_record() const noexcept { return in_flight_; }
  void discard_in_flight() noexcept { in_flight_.reset(); }
  // A reconnect starts a new capability/session/permit/qualification cycle;
  // command intent from the previous link is never resumed.
  void close_session() noexcept {
    in_flight_.reset();
    pending_.reset();
  }
  bool acknowledge_committed(std::uint32_t committed_sequence) noexcept {
    if (!in_flight_.has_value() ||
        wire_producer_sequence(in_flight_->generation) != committed_sequence) return false;
    in_flight_.reset();
    return true;
  }
  bool has_in_flight() const noexcept { return in_flight_.has_value(); }
  bool has_pending() const noexcept { return pending_.has_value(); }
  std::uint64_t pending_generation() const noexcept {
    return pending_.has_value() ? pending_->generation : 0U;
  }

 private:
  std::optional<CommandGenerationRecord> in_flight_;
  std::optional<CommandGenerationRecord> pending_;
};

bool parse_transport_parameters(
    const std::unordered_map<std::string, std::string> &values,
    std::size_t joint_count, TransportParameters *output);

struct TransportStatistics {
  std::uint64_t state_messages = 0U;
  std::uint64_t status_messages = 0U;
  std::uint64_t rejected_states = 0U;
  std::uint64_t commands_published = 0U;
  std::uint64_t rejected_commands = 0U;
  std::uint64_t invalidations = 0U;
  std::uint64_t stop_requests_sent = 0U;
  std::uint64_t stop_request_failures = 0U;
  std::uint64_t heartbeat_warnings = 0U;
  std::uint64_t heartbeat_stops = 0U;
  std::uint64_t bootstrap_rejections = 0U;
  bool ready = false;
  bool initialization_failed = false;
};

// Context, node, publishers, subscriptions and executor live exclusively on
// the worker thread. start/stop are non-RT lifecycle operations.
class DeviceTransport final {
 public:
  DeviceTransport(GuardedZenbeddedSystem &system, TransportParameters parameters);
  ~DeviceTransport();
  DeviceTransport(const DeviceTransport &) = delete;
  DeviceTransport &operator=(const DeviceTransport &) = delete;
  bool start();
  void stop();
  TransportStatistics statistics() const noexcept;
  static std::shared_ptr<DeviceManagementChannel> management_channel(const TransportParameters &) noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
std::uint64_t transport_steady_now_us() noexcept;
inline std::uint64_t saturating_state_deadline(std::uint64_t produced_at_us,
                                              std::uint64_t max_age_us) noexcept {
  return produced_at_us > UINT64_MAX - max_age_us ? UINT64_MAX : produced_at_us + max_age_us;
}
}  // namespace dummy_ros_control
