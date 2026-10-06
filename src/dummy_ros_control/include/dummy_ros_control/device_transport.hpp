#pragma once

#include <algorithm>
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
  // Zero retains the transport's legacy 5x state-period Stop threshold.
  std::uint64_t heartbeat_stop_budget_us = 0U;
  // Zero uses max_state_age_us until an observed host ACK budget is configured.
  std::uint64_t group_sent_ack_timeout_us = 0U;
  std::uint64_t stop_confirmation_timeout_us = 0U;
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
  std::string raw_evidence_path;
};

struct HistoricalIdleBaselineContext {
  std::uint64_t current_boot = 0U, historical_run_boot = 0U,
                historical_run_id = 0U,
                last_status_run_id = 0U, pages_written = 0U,
                terminal_written_run_id = 0U;
  std::array<std::uint8_t, 32> historical_chain{};
};

template <typename Status>
bool historical_released_idle_baseline(const Status &value,
                                       const HistoricalIdleBaselineContext &context) noexcept {
  const bool released_attempt =
      (value.attempt_kind() == 1U && value.parent_run_id() == 0U &&
       value.final_state() == 2U) ||
      (value.attempt_kind() == 3U && value.run_id() > 1U &&
       value.parent_run_id() == value.run_id()-1U && value.final_state() == 8U);
  const bool clean_idle = value.boot_epoch() != 0U && value.run_id() != 0U &&
      value.session_id() == 0U && value.reference_epoch() == 0U &&
      value.core_state() == 1U && value.release_committed() &&
      !value.real_motion_permitted() && value.permit_remaining_us() == 0U &&
      !value.evidence_lost();
  const bool cold_start = context.current_boot == 0U &&
      context.historical_run_boot == 0U && context.historical_run_id == 0U &&
      context.last_status_run_id == 0U &&
      context.pages_written == 0U && context.terminal_written_run_id == 0U;
  const bool same_historical_run = context.historical_run_id != 0U &&
      context.historical_run_boot == value.boot_epoch() &&
      context.historical_run_id == value.run_id() &&
      std::equal(context.historical_chain.begin(), context.historical_chain.end(),
                 std::begin(value.evidence_chain_sha256()));
  return released_attempt && clean_idle && context.pages_written == 0U &&
      context.terminal_written_run_id == 0U && (cold_start || same_historical_run);
}

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
        (expected.configured_mask == 0x01U || expected.configured_mask == 0x3fU) &&
        observed.configured_mask == expected.configured_mask &&
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
  void reset() noexcept { last_state_us_ = 0U; }
  HeartbeatAction poll(std::uint64_t now_us) const noexcept {
    if (last_state_us_ == 0U || now_us <= last_state_us_ || period_us_ == 0U) return HeartbeatAction::kNone;
    const auto elapsed = now_us - last_state_us_;
    if (elapsed >= period_us_ * 5U) return HeartbeatAction::kStop;
    if (elapsed >= period_us_ * 2U) return HeartbeatAction::kWarn;
    return HeartbeatAction::kNone;
  }
  HeartbeatAction poll(std::uint64_t now_us, std::uint64_t max_state_age_us) const noexcept {
    if (last_state_us_ == 0U || now_us <= last_state_us_ ||
        period_us_ == 0U || max_state_age_us == 0U) return HeartbeatAction::kNone;
    const auto elapsed = now_us - last_state_us_;
    if (elapsed > max_state_age_us) return HeartbeatAction::kStop;
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

class GroupSentAck {
 public:
  void begin(const CommandGenerationRecord &record, std::uint64_t sent_at_us,
             std::uint64_t timeout_us, std::uint32_t reject_count = 0U,
             std::uint32_t committed_before = 0U) noexcept {
    identity_ = record.identity;
    sequence_ = wire_producer_sequence(record.generation);
    reject_count_ = reject_count;
    committed_before_ = committed_before;
    deadline_us_ = sent_at_us > UINT64_MAX - timeout_us ? UINT64_MAX : sent_at_us + timeout_us;
    pending_ = true;
  }
  bool command_rejected(std::uint64_t boot, std::uint64_t session,
                        std::uint32_t reference_epoch, std::uint32_t motion_epoch,
                        std::uint32_t committed_sequence, std::uint32_t reject_count,
                        std::uint64_t observed_at_us) const noexcept {
    return pending_ && observed_at_us <= deadline_us_ &&
           boot == identity_.boot && session == identity_.session &&
           reference_epoch == identity_.reference_epoch &&
           motion_epoch == identity_.motion_epoch &&
           committed_sequence == committed_before_ && reject_count > reject_count_;
  }
  bool observe(std::uint64_t boot, std::uint64_t session, std::uint32_t reference_epoch,
               std::uint32_t motion_epoch, std::uint32_t committed_sequence,
               std::uint8_t /* can_sent_mask */, std::uint64_t observed_at_us) noexcept {
    if (!pending_ || observed_at_us > deadline_us_ ||
        boot != identity_.boot || session != identity_.session ||
        reference_epoch != identity_.reference_epoch || motion_epoch != identity_.motion_epoch ||
        committed_sequence != sequence_) return false;
    reset();
    return true;
  }
  bool overdue(std::uint64_t now_us) const noexcept {
    return pending_ && now_us > deadline_us_;
  }
  bool pending() const noexcept { return pending_; }
  std::uint32_t sequence() const noexcept { return sequence_; }
  void reset() noexcept { pending_ = false; deadline_us_ = 0U; }

 private:
  CommandIdentity identity_{};
  std::uint32_t sequence_ = 0U;
  std::uint64_t deadline_us_ = 0U;
  std::uint32_t reject_count_ = 0U, committed_before_ = 0U;
  bool pending_ = false;
};

struct TransportStopCauses {
  bool state_stale = false;
  bool ack_timed_out = false;
  bool ack_unconfirmed_on_state_stale_stop = false;
};

inline TransportStopCauses classify_transport_stop_causes(
    const GroupSentAck &ack, std::uint64_t now_us, std::uint64_t last_state_us,
    std::uint64_t max_state_age_us) noexcept {
  const bool state_stale = last_state_us != 0U && now_us > last_state_us &&
      now_us - last_state_us > max_state_age_us;
  const bool ack_timed_out = ack.overdue(now_us);
  return {state_stale, ack_timed_out,
          state_stale && ack.pending() && !ack_timed_out};
}

bool parse_transport_parameters(
    const std::unordered_map<std::string, std::string> &values,
    std::size_t joint_count, TransportParameters *output);

struct TransportStatistics {
  std::uint64_t state_callbacks = 0U;
  std::uint64_t state_messages = 0U;
  std::uint64_t status_messages = 0U;
  std::uint64_t stale_run_status_rejections = 0U;
  std::uint64_t rejected_states = 0U;
  std::uint64_t state_decode_rejections = 0U;
  std::uint64_t state_identity_rejections = 0U;
  std::uint64_t state_authority_rejections = 0U;
  std::uint64_t state_sequence_rejections = 0U;
  std::uint64_t state_ingest_rejections = 0U;
  std::uint64_t stop_latched_state_rejections = 0U;
  std::uint64_t commands_published = 0U;
  std::uint64_t rejected_commands = 0U;
  std::uint64_t invalidations = 0U;
  std::uint64_t stop_requests_sent = 0U;
  std::uint64_t stop_request_failures = 0U;
  std::uint64_t stop_requests_confirmed = 0U;
  std::uint64_t unconfirmed_stop_requests = 0U;
  std::uint64_t heartbeat_warnings = 0U;
  std::uint64_t heartbeat_stops = 0U;
  std::uint64_t group_sent_ack_timeouts = 0U;
  std::uint64_t group_sent_ack_misses = 0U;
  std::uint64_t state_stale_stops = 0U;
  std::uint64_t raw_idle_reobservations = 0U;
  // Stop began with an unconfirmed ACK before its timeout; CAN SENT is unknown.
  std::uint64_t ack_unconfirmed_on_state_stale_stop = 0U;
  std::uint64_t bootstrap_rejections = 0U;
  std::uint64_t stale_payload_rejections = 0U;
  std::uint64_t clock_skew_rejections = 0U;
  std::uint64_t state_heartbeat_gap_stops = 0U;
  std::uint64_t last_state_gap_us = 0U;
  std::uint64_t raw_evidence_written = 0U;
  std::uint64_t raw_evidence_pages_written = 0U;
  std::uint64_t raw_evidence_last_terminal_written_run = 0U;
  std::uint8_t raw_evidence_failure = 0U;
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
