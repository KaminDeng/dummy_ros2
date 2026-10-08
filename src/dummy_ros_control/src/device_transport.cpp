#include "dummy_ros_control/device_transport.hpp"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cstring>
#include <optional>
#include <random>
#include <sstream>
#include <thread>

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/state_validator.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"
#include "dummy_ros_control/management_admission_retry.hpp"

namespace dummy_ros_control {
namespace {
namespace wire = dp::protocol::dummy_robot_v2;
using State = wire::DummyJointStateV2;
using Status = wire::DummyDeviceStatusV2;
using Evidence = wire::DummyAttemptEvidenceV2;
using Command = wire::DummyJointCommandV2;
using Management = dp::protocol::services::DummyManageV2RawFixed;
constexpr std::uint16_t kWireEnablingHold = 3U;
constexpr std::uint16_t kWireHold = 4U;
constexpr std::uint16_t kWireDisabling = 7U;
constexpr std::uint16_t kWireDisableAndRelease = 10U;
constexpr std::uint16_t kWireGetOperation = 12U;
constexpr std::uint16_t kWirePendingResult = 23U;
std::mutex channel_registry_mutex;
std::weak_ptr<DeviceManagementChannel> channel_registry;

std::atomic<std::uint64_t> &stop_request_ids() {
  static std::atomic<std::uint64_t> ids([] {
    std::random_device random;
    const auto seed = (static_cast<std::uint64_t>(random()) << 32U) | random();
    return seed == 0U ? 1U : seed;
  }());
  return ids;
}

std::uint64_t take_stop_request_id() noexcept {
  auto &ids = stop_request_ids();
  auto current = ids.load(std::memory_order_relaxed);
  while (current != 0U && !ids.compare_exchange_weak(
      current, current + 1U, std::memory_order_relaxed)) {}
  return current;
}

bool parse_id(const std::string &text, std::array<std::uint8_t, 16> *id) {
  if (text.size() != 32U) return false;
  unsigned combined = 0U;
  for (std::size_t index = 0; index < id->size(); ++index) {
    unsigned value = 0U;
    const char *begin = text.data() + index * 2U;
    const auto parsed = std::from_chars(begin, begin + 2, value, 16);
    if (parsed.ec != std::errc{} || parsed.ptr != begin + 2) return false;
    (*id)[index] = static_cast<std::uint8_t>(value);
    combined |= value;
  }
  return combined != 0U;
}

bool parse_hash(const std::string &text, std::array<std::uint8_t, 32> *hash) {
  if (text.size() != 64U) return false;
  unsigned combined = 0U;
  for (std::size_t index = 0U; index < hash->size(); ++index) {
    unsigned value = 0U;
    const char *begin = text.data() + index * 2U;
    const auto parsed = std::from_chars(begin, begin + 2, value, 16);
    if (parsed.ec != std::errc{} || parsed.ptr != begin + 2) return false;
    (*hash)[index] = static_cast<std::uint8_t>(value);
    combined |= value;
  }
  return combined != 0U;
}

bool parse_six_doubles(const std::string &text, std::array<double, 6> *values) {
  std::istringstream input(text);
  for (std::size_t axis = 0U; axis < values->size(); ++axis) {
    if (!(input >> (*values)[axis]) || !std::isfinite((*values)[axis])) return false;
    if (axis + 1U != values->size() && input.peek() == ',') input.get();
  }
  input >> std::ws;
  return input.eof();
}

bool parse_number(const std::string &text, std::uint32_t *value) {
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), *value);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size() && *value != 0U;
}
bool sequence_at_or_after(std::uint32_t value, std::uint32_t reference) {
  return static_cast<std::uint32_t>(value - reference) < 0x80000000U;
}
bool motion_authorizing_operation(std::uint16_t operation) noexcept {
  switch (operation) {
    case 3U:   // Capture reference
    case 4U:   // Clear reference
    case 6U:   // Prepare hold
    case 7U:   // Start motion
    case 8U:   // Complete motion
    case 9U:   // Cancel motion
    case 10U:  // Disable and release
    case 11U:  // Clear fault
    case 16U:  // Confirm manual home
    case 17U:  // Begin qualification
    case 18U:  // Validate rest
    case 19U:  // Complete qualification
      return true;
    default:
      return false;
  }
}
RawEvidenceIdentity raw_evidence_identity(const TransportParameters &parameters) noexcept {
  return {parameters.client_instance_id, parameters.contract_hash, parameters.model_hash,
          parameters.profile_hash, parameters.firmware_hash};
}
std::uint64_t management_request_id(const DeviceManagementRequest &pending) noexcept {
  Management::Request decoded;
  return wire::DummyManageV2RequestCodec::decode(pending.request.data(),pending.request.size(),&decoded)
      ==wire::CodecResult::kOk ? decoded.request_id() : 0U;
}
void log_management_handoff(const DeviceManagementRequest &pending,const char *reason,
                            dp::comm::Result result=dp::comm::Result::kOk) noexcept {
  static std::atomic<unsigned> async_count{0U}, reply_count{0U}, handoff_count{0U};
  auto &count=std::strcmp(reason,"async_request_failed")==0 ? async_count :
      std::strcmp(reason,"reply_failed")==0 ? reply_count : handoff_count;
  if(count.fetch_add(1U,std::memory_order_relaxed)>=128U) return;
  std::fprintf(stderr,"MANAGE_WORKER_DIAG reason=%s request_id=%llu result=%u\n",
      reason,static_cast<unsigned long long>(management_request_id(pending)),
      static_cast<unsigned>(result));
}
}  // namespace

std::uint64_t transport_steady_now_us() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool parse_transport_parameters(
    const std::unordered_map<std::string, std::string> &values,
    std::size_t joint_count, TransportParameters *output) {
  if (!output || (joint_count != 1U && joint_count != 6U)) return false;
  for (const char *key : {"endpoint", "device_id", "client_instance_id", "profile_id", "configured_mask",
                          "contract_sha256", "model_sha256", "profile_sha256", "firmware_sha256",
                          "compiled_policy_sha256", "position_min_rad", "position_max_rad",
                          "velocity_max_rad_s", "acceleration_max_rad_s", "control_period_us",
                          "state_publish_period_us"})
    if (values.find(key) == values.end()) return false;
  TransportParameters parsed;
  parsed.endpoint = values.at("endpoint");
  std::uint32_t mask = 0U;
  if (parsed.endpoint.empty() || parsed.endpoint.size() > 256U ||
      parsed.endpoint.find_first_of("\n\r\t ") != std::string::npos ||
      !parse_id(values.at("device_id"), &parsed.device_id) ||
      !parse_id(values.at("client_instance_id"), &parsed.client_instance_id) ||
      !parse_number(values.at("profile_id"), &parsed.profile_id) ||
      !parse_number(values.at("configured_mask"), &mask) ||
      mask != (joint_count == 1U ? 1U : 63U)) return false;
  parsed.configured_mask = static_cast<std::uint8_t>(mask);
  std::uint32_t control_period_us = 0U;
  std::uint32_t state_period_us = 0U;
  if (!parse_hash(values.at("contract_sha256"), &parsed.contract_hash) ||
      !parse_hash(values.at("model_sha256"), &parsed.model_hash) ||
      !parse_hash(values.at("profile_sha256"), &parsed.profile_hash) ||
      !parse_hash(values.at("firmware_sha256"), &parsed.firmware_hash) ||
      !parse_hash(values.at("compiled_policy_sha256"), &parsed.policy_hash) ||
      !parse_six_doubles(values.at("position_min_rad"), &parsed.position_min) ||
      !parse_six_doubles(values.at("position_max_rad"), &parsed.position_max) ||
      !parse_six_doubles(values.at("velocity_max_rad_s"), &parsed.velocity_max) ||
      !parse_six_doubles(values.at("acceleration_max_rad_s"), &parsed.acceleration_max) ||
      !parse_number(values.at("control_period_us"), &control_period_us) ||
      !parse_number(values.at("state_publish_period_us"), &state_period_us)) return false;
  parsed.control_period_us = control_period_us;
  parsed.state_publish_period_us = state_period_us;
  for (std::size_t axis = 0U; axis < parsed.position_min.size(); ++axis)
    if (parsed.position_min[axis] >= parsed.position_max[axis] || parsed.velocity_max[axis] <= 0.0 ||
        parsed.acceleration_max[axis] <= 0.0) return false;
  const auto age = values.find("max_state_age_us");
  if (age != values.end()) {
    std::uint32_t age_us = 0U;
    if (!parse_number(age->second, &age_us) || age_us > 1000000U) return false;
    parsed.max_state_age_us = age_us;
  }
  const auto ack_timeout = values.find("group_sent_ack_timeout_us");
  if (ack_timeout != values.end()) {
    std::uint32_t timeout_us = 0U;
    if (!parse_number(ack_timeout->second, &timeout_us) || timeout_us > 1000000U)
      return false;
    parsed.group_sent_ack_timeout_us = timeout_us;
  }
  const auto stop_timeout = values.find("stop_confirmation_timeout_us");
  if (stop_timeout != values.end()) {
    std::uint32_t timeout_us = 0U;
    if (!parse_number(stop_timeout->second, &timeout_us) || timeout_us > 1000000U)
      return false;
    parsed.stop_confirmation_timeout_us = timeout_us;
  }
  const auto heartbeat_budget = values.find("heartbeat_stop_budget_us");
  if (heartbeat_budget != values.end()) {
    std::uint32_t budget_us = 0U;
    if (!parse_number(heartbeat_budget->second, &budget_us) ||
        budget_us > parsed.max_state_age_us ||
        budget_us < parsed.state_publish_period_us * 2U) return false;
    parsed.heartbeat_stop_budget_us = budget_us;
  }
  const auto raw_evidence_path = values.find("raw_evidence_path");
  if (raw_evidence_path != values.end()) {
    if (raw_evidence_path->second.empty() || raw_evidence_path->second.front() != '/' ||
        raw_evidence_path->second.size() > 256U ||
        raw_evidence_path->second.find_first_of("\n\r\t") != std::string::npos) return false;
    parsed.raw_evidence_path = raw_evidence_path->second;
  }
  *output = std::move(parsed);
  return true;
}

struct DeviceTransport::Impl {
  GuardedZenbeddedSystem &system;
  const TransportParameters parameters;
  const std::shared_ptr<DeviceManagementChannel> management;
  RawEvidenceCollector raw_evidence;
  RawEvidenceBindings evidence_bindings;
  std::atomic<bool> running{false}, ready{false}, failed{false};
  std::thread worker;
  std::atomic<std::uint64_t> states{0}, statuses{0}, stale_run_status_rejections{0};
  std::atomic<std::uint64_t> rejected_states{0}, commands{0};
  std::atomic<std::uint64_t> state_callbacks{0}, state_decode_rejections{0};
  std::atomic<std::uint64_t> state_identity_rejections{0}, state_authority_rejections{0};
  std::atomic<std::uint64_t> state_sequence_rejections{0}, state_ingest_rejections{0};
  std::atomic<std::uint64_t> stop_latched_state_rejections{0}, group_sent_ack_misses{0};
  std::atomic<std::uint64_t> rejected_commands{0}, invalidations{0};
  std::atomic<std::uint64_t> stop_requests_sent{0}, stop_request_failures{0};
  std::atomic<std::uint64_t> stop_requests_confirmed{0}, unconfirmed_stop_requests{0};
  std::atomic<bool> stop_confirmation_pending{false};
  std::atomic<std::uint64_t> heartbeat_warnings{0}, heartbeat_stops{0}, bootstrap_rejections{0};
  std::atomic<std::uint64_t> group_sent_ack_timeouts{0};
  std::atomic<std::uint64_t> state_stale_stops{0}, ack_unconfirmed_on_state_stale_stop{0};
  std::atomic<std::uint64_t> raw_idle_reobservations{0};
  std::atomic<std::uint64_t> stale_payload_rejections{0}, clock_skew_rejections{0};
  std::atomic<std::uint64_t> state_heartbeat_gap_stops{0}, last_state_gap_us{0};
  Status status{};
  std::uint64_t boot = 0U, producer_anchor_us = 0U;
  std::uint64_t last_status_run_boot = 0U, last_status_run_session = 0U;
  std::uint64_t last_status_run_id = 0U;
  std::uint64_t historical_run_boot = 0U, historical_run_id = 0U;
  std::array<std::uint8_t, 32> historical_chain{};
  std::int64_t producer_offset_us = 0, clock_anchor_offset_us = 0;
  std::uint64_t clock_anchor_received_us = 0U;
  std::uint64_t last_produced_us = 0U, last_received_us = 0U;
  std::uint64_t last_status_received_us = 0U;
  std::uint64_t last_callback_received_us = 0U, last_callback_produced_us = 0U;
  std::uint32_t last_callback_sequence = 0U;
  std::uint64_t last_rejection_log_us = 0U;
  std::uint32_t last_sequence = 0U;
  std::uint64_t last_state_boot = 0U;
  std::uint64_t last_state_session = 0U;
  std::uint32_t last_state_reference = 0U, last_state_motion = 0U;
  std::uint32_t last_state_management_epoch = 0U;
  unsigned complete_binding_diag_count=0U;
  unsigned command_prime_reject_diag_count=0U;
  std::uint32_t authorized_motion_epoch = 0U;
  std::uint32_t authorized_management_epoch = 0U;
  struct PendingComplete {
    std::uint64_t request_id=0U, boot_epoch=0U, session_id=0U, started_us=0U;
    std::uint32_t reference_epoch=0U, motion_epoch=0U, management_epoch=0U, terminal_seq=0U;
    bool dispatched=false, queue_drained=false;
  } pending_complete;
  CommandIdentity last_ack_identity{};
  std::uint32_t last_ack_seq=0U;
  CommandIdentity last_processed_hold_identity{};
  std::uint64_t last_processed_hold_generation=0U;
  std::uint8_t last_processed_hold_disposition=0U;
  std::array<double,6> last_processed_hold_position{};
  bool hold_drain_rejected=false;
  struct ConfirmedComplete {
    std::uint64_t boot_epoch=0U, session_id=0U;
    std::uint32_t reference_epoch=0U, motion_epoch=0U, terminal_seq=0U;
  } confirmed_complete;
  struct CompleteBoundary {
    std::uint64_t boot_epoch=0U, session_id=0U;
    std::uint32_t reference_epoch=0U, hold_epoch=0U, terminal_seq=0U, first_prime_seq=0U;
    std::uint64_t prior_run_id=0U;
    std::array<double,6> first_prime_position{};
  } complete_boundary;
  mutable bool complete_boundary_hold_diag_logged=false;
  bool exported_motion_ready=false;
  std::array<std::uint8_t,32> exported_motion_chain{};
  std::uint64_t refresh_active_run_id=0U, last_refresh_acked_us=0U;
  std::uint32_t refresh_in_flight_seq=0U;
  std::array<std::uint8_t,32> last_terminal_chain{};
  std::uint64_t last_terminal_chain_run=0U;
  RawEvidenceTerminalFlushed accepted_completed_terminal{};
  struct CompletedTerminalChain {
    std::uint64_t boot_epoch=0U, session_id=0U, run_id=0U;
    std::array<std::uint8_t,32> chain{};
  };
  std::array<CompletedTerminalChain,8U> completed_terminal_chains{};
  std::size_t next_completed_terminal_chain=0U;
  std::uint64_t last_clean_cancel_boot=0U, last_clean_cancel_session=0U,
                last_clean_cancel_run=0U;
  struct ObservedHold {
    std::uint64_t boot_epoch=0U, session_id=0U, received_us=0U;
    std::uint32_t reference_epoch=0U, motion_epoch=0U, committed_seq=0U;
    std::array<double,6> position{};
  } observed_hold;
  std::uint64_t last_freshness_ticket = 0U;
  std::uint64_t last_ticket_received_us = 0U;
  std::uint64_t last_motion_request_id = 0U;
  std::uint64_t last_motion_operation_id = 0U;
  bool has_sequence = false;
  LatestOnlyCommandStream command_stream;
  GroupSentAck group_sent_ack;
  TransportHeartbeat heartbeat;
  bool bootstrap_ready = false;
  bool heartbeat_warned = false;
  bool heartbeat_stopped = false;
  bool command_sent = false;
  bool logged_prime_transfer = false;
  bool logged_copy_state_failure = false;
  bool logged_publish_attempt = false;
  bool logged_publish_success = false;
  std::uint32_t enabling_transfer_log_count = 0U;
  std::uint32_t enabling_copy_failure_log_count = 0U;
  std::uint32_t enabling_publish_log_count = 0U;
  std::uint32_t ack_mismatch_diag_count = 0U;
  std::uint64_t last_enable_visibility_log_us = 0U;
  bool stop_latched = false;
  bool stop_idle_observed = false;
  bool ever_authorized_this_boot = false;
  bool command_published_this_boot = false;
  bool last_observation_raw_idle = false;
  std::optional<RawEvidenceBinding> provisional_evidence_binding;
  std::optional<std::array<std::uint8_t, 32U>> provisional_release_chain;
  struct GateRelease {
    std::uint64_t request_id = 0U, boot_epoch = 0U, session_id = 0U;
    std::uint32_t reference_epoch = 0U, expected_motion_epoch = 0U;
    std::uint32_t expected_management_epoch = 0U;
    std::uint32_t motion_epoch = 0U, management_epoch = 0U;
    bool committed = false;
    bool idle_confirmed = false;
  } gate_release;
  std::array<std::uint64_t, 16> retired_boots{};
  std::size_t retired_count = 0U;

  Impl(GuardedZenbeddedSystem &target, TransportParameters config)
      : system(target), parameters(std::move(config)),
        management(std::make_shared<DeviceManagementChannel>(parameters)),
        raw_evidence(raw_evidence_identity(parameters), management.get()),
        heartbeat(parameters.state_publish_period_us) {
    (void)stop_request_ids();
    std::lock_guard<std::mutex> lock(channel_registry_mutex);
    if(channel_registry.expired()) channel_registry=management;
  }

  void invalidate(const Status *retired_authority = nullptr,
                  bool reset_watermarks = false) noexcept {
    pending_complete={};
    confirmed_complete={};
    complete_boundary={};
    management->post_complete_begin_ready.store(false,std::memory_order_release);
    complete_boundary_hold_diag_logged=false;
    exported_motion_ready=false;
    exported_motion_chain={};
    accepted_completed_terminal={};
    completed_terminal_chains={};
    next_completed_terminal_chain=0U;
    refresh_active_run_id=last_refresh_acked_us=0U;
    refresh_in_flight_seq=0U;
    observed_hold={};
    last_processed_hold_identity={};
    last_processed_hold_generation=0U;
    last_processed_hold_disposition=0U;
    last_processed_hold_position={};
    hold_drain_rejected=false;
    { std::lock_guard<std::mutex> lock(management->mutex); management->hold_drain={}; }
    CommandIdentity identity{};
    if (retired_authority != nullptr) {
      identity = CommandIdentity{
          retired_authority->boot_epoch(), retired_authority->session_id(),
          retired_authority->reference_epoch(), retired_authority->motion_epoch(), 0U};
    }
    if (retired_authority != nullptr) system.invalidate_authority_state(identity);
    else system.invalidate_transport_state();
    bootstrap_ready = false;
    group_sent_ack.reset();
    command_sent = false;
    command_stream.close_session();
    CommandGenerationRecord record{};
    while (system.take_publish_record(&record)) ++rejected_commands;
    boot = 0U;
    last_motion_request_id = 0U;
    last_motion_operation_id = 0U;
    if (reset_watermarks) {
      authorized_motion_epoch = 0U;
      authorized_management_epoch = 0U;
      has_sequence = false;
      last_state_management_epoch=0U;
      producer_anchor_us = 0U;
      producer_offset_us = 0;
      clock_anchor_offset_us = 0;
      clock_anchor_received_us = 0U;
    }
    last_received_us = 0U;
    last_freshness_ticket = 0U;
    last_ticket_received_us = 0U;
    heartbeat.reset();
    ++invalidations;
  }

  template <typename Message>
  bool matches_common(const Message &message) const noexcept {
    return
           std::equal(parameters.device_id.begin(), parameters.device_id.end(), message.device_id()) &&
           message.profile_id() == parameters.profile_id &&
           message.configured_mask() == parameters.configured_mask && message.boot_epoch() != 0U;
  }

  bool matches_status(const Status &message) const noexcept {
    return message.schema_major() == 2U && message.schema_minor() == 4U && matches_common(message);
  }

  bool matches_state(const State &message) const noexcept {
    return message.schema_major() == 2U && message.schema_minor() == 4U && matches_common(message);
  }

  bool bind_evidence_status(const Status &value) noexcept {
    if (value.evidence_lost()) return false;
    if (value.run_id() == 0U) return true;
    if (historical_released_idle_baseline(value, {boot, historical_run_boot,
            historical_run_id, last_status_run_id, raw_evidence.pages_written(),
            raw_evidence.terminal_written_run_id(), historical_chain})) {
      if (historical_run_id == 0U) {
        historical_run_boot = value.boot_epoch();
        historical_run_id = value.run_id();
        std::copy(std::begin(value.evidence_chain_sha256()),
                  std::end(value.evidence_chain_sha256()), historical_chain.begin());
      }
      return true;
    }
    if (value.boot_epoch() == historical_run_boot &&
        value.run_id() == historical_run_id) return false;
    return evidence_bindings.observe({value.run_id(), value.boot_epoch(), value.session_id(),
        value.parent_run_id(), value.attempt_kind()},
        raw_evidence.terminal_written_run_id()) && raw_evidence.expect_run(value.run_id());
  }

  const RawEvidenceBinding *find_evidence_binding(std::uint64_t run_id) const noexcept {
    return evidence_bindings.find(run_id);
  }

  template <typename Message>
  bool matches_authority(const Message &message) const noexcept {
    return message.session_id() == status.session_id() &&
           message.reference_epoch() == status.reference_epoch() &&
           message.motion_epoch() == status.motion_epoch();
  }

  bool matches_state_authority(const State &message) const noexcept {
    return message.session_id() == status.session_id() &&
           message.reference_epoch() == status.reference_epoch() &&
           message.motion_epoch() == authorized_motion_epoch;
  }

  bool recoverable_raw_idle_gap() const noexcept {
    return boot != 0U && status.boot_epoch() == boot && has_sequence &&
        last_state_boot == boot && last_observation_raw_idle &&
        (status.core_state() == 1U || status.core_state() == 2U) &&
        status.session_id() == 0U && status.reference_epoch() == 0U &&
        status.motion_epoch() == 0U && authorized_motion_epoch == 0U &&
        !ever_authorized_this_boot && !command_published_this_boot &&
        !system.commands_active() && !command_stream.has_in_flight() &&
        !group_sent_ack.pending() && !stop_latched &&
        !stop_confirmation_pending.load(std::memory_order_acquire);
  }

  bool provisional_release_idle(const State &message) const noexcept {
    return gate_release.request_id != 0U && boot == gate_release.boot_epoch &&
        status.boot_epoch() == boot && status.core_state() == kWireDisabling &&
        status.session_id() == gate_release.session_id &&
        status.reference_epoch() == gate_release.reference_epoch &&
        sequence_at_or_after(status.motion_epoch(), gate_release.expected_motion_epoch) &&
        sequence_at_or_after(status.management_epoch(), gate_release.expected_management_epoch) &&
        (!gate_release.committed ||
         (status.motion_epoch() == gate_release.motion_epoch &&
          status.management_epoch() == gate_release.management_epoch)) &&
        !status.real_motion_permitted() && status.permit_remaining_us() == 0U &&
        message.boot_epoch() == boot && message.core_state() == 1U &&
        message.session_id() == 0U && message.reference_epoch() == 0U &&
        message.motion_epoch() == status.motion_epoch() &&
        message.management_epoch() == status.management_epoch();
  }

  bool raw_idle_observation_valid(const State &message,
                                  const StateExpectation &expected,
                                  bool provisional_release = false) const noexcept {
    if ((message.core_state() != 1U && message.core_state() != 2U) ||
        (!provisional_release && status.core_state() != message.core_state()) ||
        message.position_valid_mask() != 0U ||
        (message.raw_position_valid_mask() & parameters.configured_mask) != parameters.configured_mask ||
        !sequence_at_or_after(message.management_epoch(), status.management_epoch()) ||
        (has_sequence && !sequence_newer(message.state_seq(), last_sequence)) ||
        message.produced_at_us() < expected.request_started_at_us ||
        message.produced_at_us() > expected.now_us ||
        expected.now_us - message.produced_at_us() > expected.max_age_us) return false;
    for (std::size_t axis = 0U; axis < parameters.position_min.size(); ++axis)
      if ((parameters.configured_mask & (1U << axis)) != 0U &&
          !std::isfinite(message.raw_motor_position_rev()[axis])) return false;
    return true;
  }

  bool confirmed_gate_released_idle(std::uint64_t now) const noexcept {
    return gate_release.committed && boot == gate_release.boot_epoch &&
        bootstrap_ready && last_observation_raw_idle &&
        last_state_boot == boot && last_state_session == 0U &&
        last_state_reference == 0U &&
        last_state_motion == gate_release.motion_epoch &&
        last_state_management_epoch == gate_release.management_epoch &&
        status.boot_epoch() == boot && status.core_state() == 1U &&
        status.session_id() == 0U && status.reference_epoch() == 0U &&
        status.motion_epoch() == gate_release.motion_epoch &&
        status.management_epoch() == gate_release.management_epoch &&
        status.release_committed() && !status.real_motion_permitted() &&
        status.permit_remaining_us() == 0U &&
        last_received_us != 0U && now >= last_received_us &&
        now - last_received_us <= parameters.max_state_age_us &&
        last_status_received_us != 0U && now >= last_status_received_us &&
        now - last_status_received_us <= parameters.max_state_age_us &&
        !stop_confirmation_pending.load(std::memory_order_acquire);
  }

  bool released_idle_latched() const noexcept {
    return gate_release.idle_confirmed && gate_release.committed && !stop_latched &&
        !stop_confirmation_pending.load(std::memory_order_acquire) &&
        boot == gate_release.boot_epoch && last_observation_raw_idle &&
        last_state_boot == boot && last_state_session == 0U &&
        last_state_reference == 0U &&
        last_state_motion == gate_release.motion_epoch &&
        last_state_management_epoch == gate_release.management_epoch &&
        status.boot_epoch() == boot && status.core_state() == 1U &&
        status.session_id() == 0U && status.reference_epoch() == 0U &&
        status.motion_epoch() == gate_release.motion_epoch &&
        status.management_epoch() == gate_release.management_epoch &&
        status.release_committed() && !status.real_motion_permitted() &&
        status.permit_remaining_us() == 0U;
  }

  void observe_gate_release_reply(const DeviceManagementRequest &pending) noexcept {
    if (gate_release.request_id == 0U) return;
    Management::Request request;
    Management::Response response;
    if (wire::DummyManageV2RequestCodec::decode(pending.request.data(), pending.request.size(),
            &request) != wire::CodecResult::kOk ||
        wire::DummyManageV2ResponseCodec::decode(pending.response.data(), pending.response.size(),
            &response) != wire::CodecResult::kOk) return;
    if (!((request.operation() == kWireDisableAndRelease &&
           request.request_id() == gate_release.request_id) ||
          (request.operation() == kWireGetOperation &&
           request.queried_operation_id() == gate_release.request_id)) ||
        request.expected_boot_epoch() != gate_release.boot_epoch ||
        request.expected_session_id() != gate_release.session_id ||
        request.expected_reference_epoch() != gate_release.reference_epoch ||
        request.expected_motion_epoch() != gate_release.expected_motion_epoch ||
        request.expected_management_epoch() != gate_release.expected_management_epoch ||
        response.request_id() != request.request_id() ||
        response.boot_epoch() != gate_release.boot_epoch ||
        !((response.session_id() == gate_release.session_id &&
           response.reference_epoch() == gate_release.reference_epoch) ||
          (response.session_id() == 0U && response.reference_epoch() == 0U)) ||
        !sequence_at_or_after(response.motion_epoch(), gate_release.expected_motion_epoch) ||
        !sequence_at_or_after(response.management_epoch(), gate_release.expected_management_epoch) ||
        response.schema_major() != 2U || response.schema_minor() != 4U ||
        response.profile_id() != parameters.profile_id ||
        response.receipt() != 1U || response.transaction_phase() != 3U ||
        response.result_code() != 0U || response.operation_id() == 0U ||
        !response.release_committed() ||
        !std::equal(parameters.device_id.begin(), parameters.device_id.end(), response.device_id()) ||
        !std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                    response.client_instance_id())) return;
    gate_release.motion_epoch = response.motion_epoch();
    gate_release.management_epoch = response.management_epoch();
    gate_release.committed = true;
  }

  void accept_management_authority(const DeviceManagementRequest &pending) noexcept {
    Management::Request request;
    Management::Response response;
    if (wire::DummyManageV2RequestCodec::decode(pending.request.data(), pending.request.size(), &request) !=
            wire::CodecResult::kOk ||
        wire::DummyManageV2ResponseCodec::decode(pending.response.data(), pending.response.size(), &response) !=
            wire::CodecResult::kOk) return;
    const bool complete_reply = pending_complete.request_id!=0U && pending_complete.dispatched &&
        transport_steady_now_us()-pending_complete.started_us<=2000000U &&
        (last_motion_operation_id==0U || response.operation_id()==last_motion_operation_id) &&
        (request.operation()==8U && request.request_id()==pending_complete.request_id ||
         request.operation()==kWireGetOperation &&
           request.queried_operation_id()==pending_complete.request_id) &&
        request.expected_boot_epoch()==pending_complete.boot_epoch &&
        request.expected_session_id()==pending_complete.session_id &&
        request.expected_reference_epoch()==pending_complete.reference_epoch &&
        request.expected_motion_epoch()==pending_complete.motion_epoch &&
        request.expected_management_epoch()==pending_complete.management_epoch;
    const bool staged_complete = complete_reply &&
        status.boot_epoch()==pending_complete.boot_epoch &&
        status.session_id()==pending_complete.session_id &&
        status.reference_epoch()==pending_complete.reference_epoch &&
        authorized_motion_epoch==pending_complete.motion_epoch+1U &&
        status.motion_epoch()==authorized_motion_epoch &&
        response.motion_epoch()==authorized_motion_epoch &&
        sequence_at_or_after(response.management_epoch(),status.management_epoch());
    const bool invalid_request_binding =
        request.schema_major() != 2U || request.schema_minor() != 4U ||
        response.schema_major() != 2U || response.schema_minor() != 4U ||
        request.request_id() == 0U || response.request_id() != request.request_id() ||
        response.operation_id() == 0U || boot == 0U ||
        request.expected_boot_epoch() != boot || response.boot_epoch() != boot ||
        request.expected_session_id() != status.session_id() ||
        response.session_id() != status.session_id() ||
        request.expected_reference_epoch() != status.reference_epoch() ||
        response.reference_epoch() != status.reference_epoch() ||
        (!staged_complete && request.expected_motion_epoch() != authorized_motion_epoch) ||
        (!staged_complete && !sequence_at_or_after(request.expected_management_epoch(), status.management_epoch())) ||
        request.expected_profile_id() != parameters.profile_id ||
        response.profile_id() != parameters.profile_id ||
        request.configured_mask() != parameters.configured_mask ||
        !std::equal(parameters.device_id.begin(), parameters.device_id.end(), request.device_id()) ||
        !std::equal(parameters.device_id.begin(), parameters.device_id.end(), response.device_id()) ||
        !std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                    request.client_instance_id()) ||
        !std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                    response.client_instance_id());
    if (invalid_request_binding) return;
    if (complete_reply && response.transaction_phase()==4U) {
      invalidate(&status,true);
      return;
    }
    if (motion_authorizing_operation(request.operation()) &&
        request.request_id() == last_motion_request_id &&
        (response.transaction_phase() == 2U || response.transaction_phase() == 3U) &&
        (response.result_code() == 0U || response.result_code() == 23U))
      last_motion_operation_id = response.operation_id();
    if (response.transaction_phase() != 3U || response.result_code() != 0U ||
        !(motion_authorizing_operation(request.operation()) || complete_reply ||
          (request.operation() == 12U && request.queried_operation_id() == last_motion_request_id &&
           last_motion_operation_id != 0U && response.operation_id() == last_motion_operation_id)) ||
        authorized_motion_epoch == UINT32_MAX ||
        (!staged_complete && response.motion_epoch() != authorized_motion_epoch + 1U) ||
        response.management_epoch() == request.expected_management_epoch() ||
        !sequence_at_or_after(response.management_epoch(), request.expected_management_epoch())) return;
    authorized_motion_epoch = response.motion_epoch();
    authorized_management_epoch = response.management_epoch();
    if (complete_reply) {
      confirmed_complete={response.boot_epoch(),response.session_id(),
        response.reference_epoch(),response.motion_epoch(),response.committed_producer_seq()};
      pending_complete={};
    } else if (motion_authorizing_operation(request.operation())) confirmed_complete={};
    ever_authorized_this_boot = true;
    if (request.operation() != kWireDisableAndRelease) stop_latched = false;
  }

  template <typename Message>
  void acknowledge_command(const Message &message) noexcept {
    if (!group_sent_ack.pending()) return;
    const auto acknowledged=command_stream.in_flight_record();
    if (group_sent_ack.observe(message.boot_epoch(), message.session_id(),
          message.reference_epoch(), message.motion_epoch(),
          message.committed_producer_seq(), message.can_sent_mask(),
          transport_steady_now_us()) &&
        command_stream.acknowledge_committed(message.committed_producer_seq())) {
      last_ack_identity={message.boot_epoch(),message.session_id(),
        message.reference_epoch(),message.motion_epoch(),0U};
      last_ack_seq=message.committed_producer_seq();
      if(acknowledged && status.core_state()==kWireHold &&
         acknowledged->identity.boot==message.boot_epoch() &&
         acknowledged->identity.session==message.session_id() &&
         acknowledged->identity.reference_epoch==message.reference_epoch() &&
         acknowledged->identity.motion_epoch==message.motion_epoch()) {
        if(acknowledged->generation>last_processed_hold_generation) {
          last_processed_hold_identity=acknowledged->identity;
          last_processed_hold_generation=acknowledged->generation;
          last_processed_hold_disposition=2U;
          last_processed_hold_position=acknowledged->position;
        }
        hold_drain_rejected=false;
      }
      if(refresh_in_flight_seq==message.committed_producer_seq()) {
        last_refresh_acked_us=transport_steady_now_us();
        refresh_in_flight_seq=0U;
      }
      command_sent = false;
    } else {
      ++group_sent_ack_misses;
      if (ack_mismatch_diag_count++ < 10U)
        fprintf(stderr,"COMMAND_ACK_DIAG expected_seq=%u observed_seq=%u status_seq=%u sent_mask=%u session=%llu/%llu motion=%u/%u\n",
                group_sent_ack.sequence(),message.committed_producer_seq(),
                status.committed_producer_seq(),message.can_sent_mask(),
                static_cast<unsigned long long>(message.session_id()),
                static_cast<unsigned long long>(status.session_id()),
                message.motion_epoch(),authorized_motion_epoch);
    }
  }

  enum class PostCompleteWrite : std::uint8_t { kAllowed, kLocalNoOp, kConflict };
  PostCompleteWrite post_complete_write(const CommandGenerationRecord &record,
                                        std::uint64_t now) const noexcept {
    const bool boundary=complete_boundary.boot_epoch==record.identity.boot &&
      complete_boundary.session_id==record.identity.session &&
      complete_boundary.reference_epoch==record.identity.reference_epoch &&
      complete_boundary.hold_epoch==record.identity.motion_epoch;
    if(!boundary) return PostCompleteWrite::kAllowed;
    const bool committed=confirmed_complete.boot_epoch==record.identity.boot &&
      confirmed_complete.session_id==record.identity.session &&
      confirmed_complete.reference_epoch==record.identity.reference_epoch &&
      confirmed_complete.motion_epoch==record.identity.motion_epoch;
    const bool fresh_hold=observed_hold.received_us!=0U && now>=observed_hold.received_us &&
      now-observed_hold.received_us<=parameters.max_state_age_us &&
      status.core_state()==kWireHold &&
      record.identity.boot==observed_hold.boot_epoch &&
      record.identity.session==observed_hold.session_id &&
      record.identity.reference_epoch==observed_hold.reference_epoch &&
      record.identity.motion_epoch==observed_hold.motion_epoch;
    const bool pending=pending_complete.request_id!=0U &&
      now>=pending_complete.started_us && now-pending_complete.started_us<=2000000U &&
      pending_complete.boot_epoch==record.identity.boot &&
      pending_complete.session_id==record.identity.session &&
      pending_complete.reference_epoch==record.identity.reference_epoch &&
      pending_complete.motion_epoch!=UINT32_MAX &&
      pending_complete.motion_epoch+1U==record.identity.motion_epoch;
    if(!committed && !fresh_hold && status.core_state()==kWireEnablingHold && pending) {
      const auto sequence=wire_producer_sequence(record.generation);
      if(complete_boundary.first_prime_seq==0U ||
         sequence==complete_boundary.first_prime_seq) return PostCompleteWrite::kAllowed;
      for(std::size_t axis=0U;axis<record.position.size();++axis) {
        if((parameters.configured_mask & (1U<<axis))==0U) continue;
        const auto tolerance=management->post_complete_hold_tolerance_rad[axis].load(
            std::memory_order_acquire);
        if(!std::isfinite(record.position[axis]) || !std::isfinite(tolerance) ||
           tolerance<0.0 ||
           std::abs(record.position[axis]-complete_boundary.first_prime_position[axis])>tolerance)
          return PostCompleteWrite::kConflict;
      }
      return PostCompleteWrite::kLocalNoOp;
    }
    if(exported_motion_ready && fresh_hold) {
      const auto seq=wire_producer_sequence(record.generation);
      const bool retry=refresh_in_flight_seq==seq;
      const bool first=last_refresh_acked_us==0U && refresh_in_flight_seq==0U;
      const bool repeat=last_refresh_acked_us!=0U && refresh_in_flight_seq==0U &&
        refresh_active_run_id>complete_boundary.prior_run_id &&
        now>=last_refresh_acked_us && now-last_refresh_acked_us>=1000000U;
      if(retry || first || repeat) {
        for(std::size_t axis=0U;axis<record.position.size();++axis) {
          if((parameters.configured_mask & (1U<<axis))==0U) continue;
          const auto tolerance=management->post_complete_hold_tolerance_rad[axis].load(
              std::memory_order_acquire);
          if(!std::isfinite(record.position[axis]) || !std::isfinite(tolerance) ||
             tolerance<=0.0 ||
             std::abs(record.position[axis]-observed_hold.position[axis])>tolerance)
            return PostCompleteWrite::kConflict;
        }
        return PostCompleteWrite::kAllowed;
      }
    }
    const auto terminal_seq=committed ? confirmed_complete.terminal_seq :
      fresh_hold ? observed_hold.committed_seq : complete_boundary.terminal_seq;
    if(!sequence_newer(wire_producer_sequence(record.generation),terminal_seq))
      return PostCompleteWrite::kAllowed;
    PostCompleteWrite result=PostCompleteWrite::kLocalNoOp;
    double max_delta=0.0, max_tolerance=0.0;
    if(!committed && !fresh_hold) result=PostCompleteWrite::kConflict;
    if(fresh_hold) for(std::size_t axis=0U;axis<record.position.size();++axis) {
      if((parameters.configured_mask & (1U<<axis))==0U) continue;
      const auto tolerance=management->post_complete_hold_tolerance_rad[axis].load(
          std::memory_order_acquire);
      const auto delta=std::abs(record.position[axis]-observed_hold.position[axis]);
      max_delta=std::max(max_delta,delta);
      max_tolerance=std::max(max_tolerance,tolerance);
      if(!std::isfinite(record.position[axis]) || !std::isfinite(tolerance) ||
         tolerance<0.0 || delta>tolerance)
        result=PostCompleteWrite::kConflict;
    }
    if(!complete_boundary_hold_diag_logged && status.core_state()==kWireHold) {
      complete_boundary_hold_diag_logged=true;
      std::fprintf(stderr,
          "POST_COMPLETE_WRITE seq=%u action=%u pending_age_us=%llu confirmed=%d hold_age_us=%llu delta_rad=%.9f tolerance_rad=%.9f\n",
          wire_producer_sequence(record.generation),static_cast<unsigned>(result),
          static_cast<unsigned long long>(pending_complete.request_id!=0U &&
              now>=pending_complete.started_us ? now-pending_complete.started_us : 0U),
          committed,static_cast<unsigned long long>(fresh_hold ? now-observed_hold.received_us : 0U),
          max_delta,max_tolerance);
    }
    return result;
  }

  bool superseded_by_pending_complete(const CommandGenerationRecord &record) const noexcept {
    return pending_complete.request_id!=0U && pending_complete.terminal_seq!=0U &&
      record.identity.boot==pending_complete.boot_epoch &&
      record.identity.session==pending_complete.session_id &&
      record.identity.reference_epoch==pending_complete.reference_epoch &&
      record.identity.motion_epoch==pending_complete.motion_epoch &&
      sequence_newer(wire_producer_sequence(record.generation),pending_complete.terminal_seq);
  }

  void reserve_complete_prime(const CommandGenerationRecord &record) noexcept {
    if(complete_boundary.first_prime_seq!=0U ||
       complete_boundary.boot_epoch!=record.identity.boot ||
       complete_boundary.session_id!=record.identity.session ||
       complete_boundary.reference_epoch!=record.identity.reference_epoch ||
       complete_boundary.hold_epoch!=record.identity.motion_epoch ||
       status.core_state()!=kWireEnablingHold) return;
    complete_boundary.first_prime_seq=wire_producer_sequence(record.generation);
    complete_boundary.first_prime_position=record.position;
  }

  void log_state_rejection(const char *reason, const State &value,
                           std::uint64_t received_us) noexcept {
    if (received_us - last_rejection_log_us < 1000000U) return;
    last_rejection_log_us = received_us;
    fprintf(stderr, "device transport state rejected: reason=%s seq=%u produced_us=%llu received_us=%llu boot=%llu session=%llu ref=%u motion=%u expected_session=%llu expected_ref=%u expected_motion=%u status_motion=%u state_management=%u status_management=%u\n",
            reason, value.state_seq(), static_cast<unsigned long long>(value.produced_at_us()),
            static_cast<unsigned long long>(received_us),
            static_cast<unsigned long long>(value.boot_epoch()),
            static_cast<unsigned long long>(value.session_id()),
            value.reference_epoch(), value.motion_epoch(),
            static_cast<unsigned long long>(status.session_id()), status.reference_epoch(),
            authorized_motion_epoch, status.motion_epoch(), value.management_epoch(),
            status.management_epoch());
  }

  static void receive_status(dp::comm::BorrowedMessage<Status> borrowed, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    Status value;
    if (borrowed.copy_to(&value) != dp::comm::Result::kOk) return;
    if (!self.parameters.raw_evidence_path.empty()) {
      std::array<std::uint8_t, wire::DummyDeviceStatusV2Codec::kEncodedSize> raw{};
      if (wire::DummyDeviceStatusV2Codec::encode(value, raw.data(), raw.size()) != wire::CodecResult::kOk ||
          !self.raw_evidence.offer_observation(RawEvidenceCollector::Kind::kStatus,
              raw.data(), raw.size(), transport_steady_now_us(), value.boot_epoch(), value.session_id()))
        self.failed = true;
    }
    ++self.statuses;
    if (!self.matches_status(value)) return;
    if (std::find(self.retired_boots.begin(), self.retired_boots.end(), value.boot_epoch()) !=
        self.retired_boots.end()) return;
    if(value.final_state()==6U && value.run_id()!=0U) {
      for(const auto &terminal:self.completed_terminal_chains) {
        if(terminal.run_id!=value.run_id() ||
           terminal.boot_epoch!=value.boot_epoch() ||
           terminal.session_id!=value.session_id()) continue;
        if(value.attempt_kind()!=1U || value.parent_run_id()!=0U ||
           !value.motion_complete() || value.evidence_lost() ||
           !std::equal(terminal.chain.begin(),terminal.chain.end(),
                       value.evidence_chain_sha256())) {
          self.raw_evidence.reject_invalid();
          self.failed=true;
          return;
        }
      }
    }
    if (value.boot_epoch() == self.last_status_run_boot &&
        value.session_id() == self.last_status_run_session && value.run_id() != 0U &&
        value.run_id() < self.last_status_run_id) {
      ++self.stale_run_status_rejections;
      return;
    }
    if (self.stop_latched && self.status.boot_epoch() == value.boot_epoch() &&
        (value.motion_epoch() != self.authorized_motion_epoch ||
         !sequence_at_or_after(value.management_epoch(), self.authorized_management_epoch))) return;
    if (self.status.boot_epoch() == value.boot_epoch()) {
      if (!sequence_at_or_after(value.management_epoch(), self.status.management_epoch())) return;
      if (value.management_epoch() == self.status.management_epoch()) {
        const bool same_owner_reference = value.session_id() == self.status.session_id() &&
            value.reference_epoch() == self.status.reference_epoch();
        const bool released_idle = value.session_id() == 0U &&
            value.reference_epoch() == 0U && value.core_state() == 1U &&
            value.release_committed() && !value.real_motion_permitted() &&
            value.permit_remaining_us() == 0U;
        if ((!same_owner_reference && !released_idle) ||
            !sequence_at_or_after(value.motion_epoch(), self.status.motion_epoch())) return;
      }
    }
    if (self.boot != 0U && self.boot != value.boot_epoch()) {
      if (self.retired_count >= self.retired_boots.size()) {
        self.invalidate();
        self.failed = true;
        self.running = false;
        return;
      }
      self.retired_boots[self.retired_count++] = self.boot;
      self.invalidate(nullptr, true);
    }
    if (self.historical_run_boot != 0U &&
        self.historical_run_boot != value.boot_epoch()) {
      self.historical_run_boot = 0U;
      self.historical_run_id = 0U;
      self.historical_chain.fill(0U);
    }
    const bool expected_complete_advance=self.pending_complete.request_id!=0U &&
        self.pending_complete.dispatched &&
        (value.core_state()==kWireEnablingHold || value.core_state()==kWireHold) &&
        value.boot_epoch()==self.pending_complete.boot_epoch &&
        value.session_id()==self.pending_complete.session_id &&
        value.reference_epoch()==self.pending_complete.reference_epoch &&
        self.authorized_motion_epoch==self.pending_complete.motion_epoch &&
        self.pending_complete.motion_epoch!=UINT32_MAX &&
        value.motion_epoch()==self.pending_complete.motion_epoch+1U &&
        sequence_at_or_after(value.management_epoch(),self.pending_complete.management_epoch) &&
        transport_steady_now_us()-self.pending_complete.started_us<=2000000U;
    if (self.boot != 0U && !expected_complete_advance &&
        (value.session_id() != self.status.session_id() ||
         value.reference_epoch() != self.status.reference_epoch() ||
         (value.motion_epoch() != self.status.motion_epoch() &&
          value.motion_epoch() != self.authorized_motion_epoch))) self.invalidate(&self.status, true);
    if (self.status.boot_epoch() != value.boot_epoch()) {
      self.ever_authorized_this_boot = false;
      self.command_published_this_boot = false;
      self.last_observation_raw_idle = false;
    }
    PositionOnlyCapability observed{};
    observed.configured_mask = value.configured_mask();
    observed.capabilities = value.capabilities();
    std::copy(std::begin(value.contract_sha256()), std::end(value.contract_sha256()), observed.contract_hash.begin());
    std::copy(std::begin(value.model_sha256()), std::end(value.model_sha256()), observed.model_hash.begin());
    std::copy(std::begin(value.profile_sha256()), std::end(value.profile_sha256()), observed.profile_hash.begin());
    std::copy(std::begin(value.firmware_sha256()), std::end(value.firmware_sha256()), observed.firmware_hash.begin());
    std::copy(std::begin(value.compiled_policy_sha256()), std::end(value.compiled_policy_sha256()),
              observed.policy_hash.begin());
    for (std::size_t axis = 0U; axis < observed.limits.size(); ++axis) {
      observed.limits[axis] = {value.position_min_rad()[axis], value.position_max_rad()[axis]};
      observed.velocity_max[axis] = value.velocity_max_rad_s()[axis];
      observed.acceleration_max[axis] = value.acceleration_max_rad_s()[axis];
    }
    observed.control_timebase_us = value.control_period_us();
    observed.state_publish_period_us = value.state_publish_period_us();
    PositionOnlyCapability expected{};
    expected.configured_mask = self.parameters.configured_mask;
    expected.capabilities = kPositionOnlyCapability;
    expected.contract_hash = self.parameters.contract_hash;
    expected.model_hash = self.parameters.model_hash;
    expected.profile_hash = self.parameters.profile_hash;
    expected.firmware_hash = self.parameters.firmware_hash;
    expected.policy_hash = self.parameters.policy_hash;
    expected.control_timebase_us = self.parameters.control_period_us;
    expected.state_publish_period_us = self.parameters.state_publish_period_us;
    for (std::size_t axis = 0U; axis < expected.limits.size(); ++axis) {
      expected.limits[axis] = {self.parameters.position_min[axis], self.parameters.position_max[axis]};
      expected.velocity_max[axis] = self.parameters.velocity_max[axis];
      expected.acceleration_max[axis] = self.parameters.acceleration_max[axis];
    }
    self.bootstrap_ready = position_only_capability_matches(expected, observed);
    if (self.provisional_evidence_binding && self.provisional_release_chain &&
        self.last_terminal_chain_run == self.provisional_evidence_binding->run_id &&
        self.raw_evidence.terminal_written_run_id() == self.provisional_evidence_binding->run_id &&
        self.last_terminal_chain == *self.provisional_release_chain) {
      self.provisional_evidence_binding.reset();
      self.provisional_release_chain.reset();
    }
    if (self.provisional_evidence_binding && self.bootstrap_ready) {
      const auto &pending = *self.provisional_evidence_binding;
      if (value.run_id() == pending.run_id) {
        if (value.boot_epoch() != pending.boot_epoch ||
            value.attempt_kind() != pending.attempt_kind ||
            value.parent_run_id() != pending.parent_run_id || value.evidence_lost()) {
          self.raw_evidence.reject_invalid();
          self.failed = true;
          return;
        }
        if (value.session_id() != pending.session_id) {
          const bool released_idle = value.session_id() == 0U &&
              value.reference_epoch() == 0U && value.core_state() == 1U &&
              value.final_state() == 2U && value.release_committed() &&
              !value.real_motion_permitted() && value.permit_remaining_us() == 0U;
          if (!released_idle) {
            self.raw_evidence.reject_invalid();
            self.failed = true;
            return;
          }
        }
        const bool has_chain = std::any_of(std::begin(value.evidence_chain_sha256()),
            std::end(value.evidence_chain_sha256()),
            [](auto byte) { return byte != 0U; });
        if (has_chain) {
          std::array<std::uint8_t, 32U> chain{};
          std::copy(std::begin(value.evidence_chain_sha256()),
              std::end(value.evidence_chain_sha256()), chain.begin());
          if ((self.provisional_release_chain && chain != *self.provisional_release_chain) ||
              (self.last_terminal_chain_run == pending.run_id &&
               chain != self.last_terminal_chain)) {
            self.raw_evidence.reject_invalid();
            self.failed = true;
            return;
          }
          self.provisional_release_chain = chain;
          if (self.last_terminal_chain_run == pending.run_id &&
              self.raw_evidence.terminal_written_run_id() == pending.run_id) {
            self.provisional_evidence_binding.reset();
            self.provisional_release_chain.reset();
          }
        }
      } else if (value.boot_epoch() != pending.boot_epoch ||
                 value.run_id() > pending.run_id ||
                 (value.session_id() != pending.session_id && value.session_id() != 0U)) {
        self.raw_evidence.reject_invalid();
        self.failed = true;
        return;
      }
    }
    if (!self.parameters.raw_evidence_path.empty() && self.bootstrap_ready &&
        !self.bind_evidence_status(value)) {
      std::fprintf(stderr,
          "RAW_BIND_DIAG run=%llu boot=%llu session=%llu kind=%u parent=%llu core=%u final=%u lost=%u terminal_written_run=%llu last_status_run=%llu failure=%u\n",
          static_cast<unsigned long long>(value.run_id()),
          static_cast<unsigned long long>(value.boot_epoch()),
          static_cast<unsigned long long>(value.session_id()), value.attempt_kind(),
          static_cast<unsigned long long>(value.parent_run_id()), value.core_state(),
          value.final_state(), value.evidence_lost() ? 1U : 0U,
          static_cast<unsigned long long>(self.raw_evidence.terminal_written_run_id()),
          static_cast<unsigned long long>(self.last_status_run_id),
          static_cast<unsigned>(self.raw_evidence.failure()));
      self.raw_evidence.reject_invalid();
      self.failed = true;
      return;
    }
    self.boot = value.boot_epoch();
    if (self.authorized_motion_epoch == 0U ||
        sequence_at_or_after(value.motion_epoch(), self.authorized_motion_epoch))
      self.authorized_motion_epoch = value.motion_epoch();
    if (self.authorized_management_epoch == 0U ||
        sequence_at_or_after(value.management_epoch(), self.authorized_management_epoch))
      self.authorized_management_epoch = value.management_epoch();
    self.status = value;
    self.last_status_received_us = transport_steady_now_us();
    if(self.complete_boundary.boot_epoch==0U && !self.stop_latched)
      self.management->post_complete_begin_ready.store(true,std::memory_order_release);
    if(value.boot_epoch()==self.complete_boundary.boot_epoch &&
       value.session_id()==self.complete_boundary.session_id &&
       value.reference_epoch()==self.complete_boundary.reference_epoch &&
       value.motion_epoch()==self.complete_boundary.hold_epoch) {
      const bool chain_nonzero=std::any_of(std::begin(value.evidence_chain_sha256()),
          std::end(value.evidence_chain_sha256()),[](auto byte){return byte!=0U;});
      const bool prior_run=value.run_id()==self.complete_boundary.prior_run_id;
      const bool contradictory=value.evidence_lost() ||
        (self.exported_motion_ready && prior_run &&
         (value.core_state()!=kWireHold || value.attempt_kind()!=1U ||
          value.parent_run_id()!=0U || value.final_state()!=6U ||
          !value.motion_complete() || !chain_nonzero ||
          !std::equal(self.exported_motion_chain.begin(),self.exported_motion_chain.end(),
                      value.evidence_chain_sha256())));
      if(contradictory) {
        self.exported_motion_ready=false;
        self.management->post_complete_begin_ready.store(false,std::memory_order_release);
        self.system.request_transport_stop();
      } else if(value.core_state()==kWireHold && prior_run && value.run_id()!=0U &&
         value.attempt_kind()==1U && value.final_state()==6U && value.motion_complete() &&
         chain_nonzero && (!self.parameters.raw_evidence_path.empty() ?
           self.raw_evidence.terminal_written_run_id()==value.run_id() &&
             self.last_terminal_chain_run==value.run_id() &&
             std::equal(self.last_terminal_chain.begin(),self.last_terminal_chain.end(),
                        value.evidence_chain_sha256()) : true))
      {
        self.exported_motion_ready=true;
        std::copy(std::begin(value.evidence_chain_sha256()),
                  std::end(value.evidence_chain_sha256()),self.exported_motion_chain.begin());
      }
      if(self.exported_motion_ready)
        self.management->post_complete_begin_ready.store(true,std::memory_order_release);
      if(self.exported_motion_ready && value.run_id()>self.complete_boundary.prior_run_id &&
         value.attempt_kind()==1U && value.final_state()==0U)
        self.refresh_active_run_id=value.run_id();
    }
    if (value.boot_epoch() != self.last_status_run_boot ||
        value.session_id() != self.last_status_run_session) {
      self.last_status_run_boot = value.boot_epoch();
      self.last_status_run_session = value.session_id();
      self.last_status_run_id = value.run_id();
    } else if (value.run_id() > self.last_status_run_id) self.last_status_run_id = value.run_id();
    if (value.session_id() != 0U || value.reference_epoch() != 0U ||
        value.motion_epoch() != 0U) self.ever_authorized_this_boot = true;
    if (!self.bootstrap_ready) {
      ++self.bootstrap_rejections;
      self.system.invalidate_transport_state();
    }
    if (!self.stop_latched && self.group_sent_ack.command_rejected(
            value.boot_epoch(), value.session_id(), value.reference_epoch(),
            value.motion_epoch(), value.committed_producer_seq(),
            value.command_reject_count(), transport_steady_now_us())) {
      const auto pending=self.command_stream.in_flight_record();
      if (pending) {
        std::lock_guard<std::mutex> lock(self.management->mutex);
        auto &observation=self.management->observation;
        observation.rejected_ack_boot_epoch=pending->identity.boot;
        observation.rejected_ack_session_id=pending->identity.session;
        observation.rejected_ack_reference_epoch=pending->identity.reference_epoch;
        observation.rejected_ack_motion_epoch=pending->identity.motion_epoch;
        observation.rejected_ack_sequence=self.group_sent_ack.sequence();
        observation.rejected_ack_received_us=transport_steady_now_us();
      }
      fprintf(stderr,"DEVICE_COMMAND_REJECTED_DURING_ACK seq=%u rejected=%u committed=%u\n",
              self.group_sent_ack.sequence(),value.command_reject_count(),
              value.committed_producer_seq());
      self.group_sent_ack.reset();
      self.command_stream.discard_in_flight();
      self.command_sent=false;
      self.hold_drain_rejected=true;
      self.stop_latched=true;
      self.system.request_transport_stop();
    } else self.acknowledge_command(value);
    {
      std::lock_guard<std::mutex> lock(self.management->mutex);
      auto &observation=self.management->observation;
      if(wire::DummyDeviceStatusV2Codec::encode(value,observation.status.data(),observation.status.size())==wire::CodecResult::kOk)
        observation.status_received_us=transport_steady_now_us();
    }
  }

  static void receive_state(dp::comm::BorrowedMessage<State> borrowed, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    ++self.state_callbacks;
    const auto now = transport_steady_now_us();
    State value;
    if (borrowed.copy_to(&value) != dp::comm::Result::kOk) {
      ++self.state_decode_rejections;
      ++self.rejected_states;
      return;
    }
    if (!self.parameters.raw_evidence_path.empty()) {
      std::array<std::uint8_t, wire::DummyJointStateV2Codec::kEncodedSize> raw{};
      if (wire::DummyJointStateV2Codec::encode(value, raw.data(), raw.size()) != wire::CodecResult::kOk ||
          !self.raw_evidence.offer_observation(RawEvidenceCollector::Kind::kState,
              raw.data(), raw.size(), now, value.boot_epoch(), value.session_id()))
        self.failed = true;
    }
    ++self.states;
    self.last_callback_received_us = now;
    self.last_callback_produced_us = value.produced_at_us();
    self.last_callback_sequence = value.state_seq();
    if (!self.bootstrap_ready || !self.matches_state(value) || self.boot == 0U ||
        value.boot_epoch() != self.boot) {
      ++self.state_identity_rejections;
      self.log_state_rejection("identity_or_bootstrap", value, now);
      ++self.rejected_states;
      return;
    }
    const bool provisional_release = self.provisional_release_idle(value);
    if (!self.matches_state_authority(value) && !provisional_release) {
      ++self.state_authority_rejections;
      self.log_state_rejection("authority", value, now);
      ++self.rejected_states;
      return;
    }
    const bool same_producer = self.last_state_boot == value.boot_epoch() &&
        self.last_state_session == value.session_id() &&
        self.last_state_reference == value.reference_epoch() &&
        self.last_state_motion == value.motion_epoch();
    // Clock epochs differ. The smallest observed receive-minus-produce offset
    // bounds later queueing delay without assuming the clocks are synchronized.
    const auto producer_anchor = self.has_sequence ? self.producer_anchor_us : value.produced_at_us();
    if (value.produced_at_us() == 0U ||
        (same_producer && value.produced_at_us() <= self.last_produced_us) ||
        (same_producer && !sequence_newer(value.state_seq(), self.last_sequence)) ||
        now > static_cast<std::uint64_t>(INT64_MAX) ||
        value.produced_at_us() > static_cast<std::uint64_t>(INT64_MAX)) {
      if (same_producer && value.produced_at_us() != 0U &&
          value.produced_at_us() <= self.last_produced_us &&
          self.last_produced_us - value.produced_at_us() > self.parameters.max_state_age_us)
        ++self.stale_payload_rejections;
      ++self.state_sequence_rejections;
      self.log_state_rejection("sequence_or_timestamp", value, now);
      ++self.rejected_states;
      return;
    }
    const auto observed_offset = static_cast<std::int64_t>(now) -
        static_cast<std::int64_t>(value.produced_at_us());
    if (self.has_sequence && observed_offset < self.clock_anchor_offset_us) {
      const auto cumulative_correction = static_cast<std::uint64_t>(self.clock_anchor_offset_us) -
          static_cast<std::uint64_t>(observed_offset);
      const auto elapsed_us = now - self.clock_anchor_received_us;
      if (cumulative_correction > self.parameters.max_state_age_us + elapsed_us / 1000U) {
        ++self.clock_skew_rejections;
        self.log_state_rejection("clock_skew", value, now);
        ++self.rejected_states;
        return;
      }
    }
    auto minimum_offset = self.has_sequence ? self.producer_offset_us : observed_offset;
    if (observed_offset < minimum_offset) {
      const auto correction = static_cast<std::uint64_t>(minimum_offset) -
          static_cast<std::uint64_t>(observed_offset);
      if (correction > self.parameters.max_state_age_us) {
        ++self.clock_skew_rejections;
        self.log_state_rejection("clock_skew", value, now);
        ++self.rejected_states;
        return;
      }
      minimum_offset = observed_offset;
    }
    const auto queue_age = static_cast<std::uint64_t>(observed_offset) -
        static_cast<std::uint64_t>(minimum_offset);
    if (value.produced_at_us() > UINT64_MAX - queue_age) {
      ++self.state_sequence_rejections;
      self.log_state_rejection("timestamp_overflow", value, now);
      ++self.rejected_states;
      return;
    }
    const StateExpectation expected{self.boot, self.parameters.profile_id, self.last_sequence,
        self.has_sequence, self.parameters.configured_mask, producer_anchor,
        value.produced_at_us() + queue_age, self.parameters.max_state_age_us};
    std::uint8_t bytes[wire::DummyJointStateV2Codec::kEncodedSize]{};
    if (wire::DummyJointStateV2Codec::encode(value, bytes, sizeof(bytes)) != wire::CodecResult::kOk) {
      ++self.state_decode_rejections;
      self.log_state_rejection("codec", value, now);
      ++self.rejected_states;
      return;
    }
    const bool raw_idle = self.raw_idle_observation_valid(value, expected, provisional_release);
    if (self.stop_latched && !raw_idle) {
      ++self.stop_latched_state_rejections;
      self.log_state_rejection("stop_latched", value, now);
      ++self.rejected_states;
      return;
    }
    if (!raw_idle && !self.system.ingest_wire_state(bytes, sizeof(bytes), expected)) {
      if (queue_age > self.parameters.max_state_age_us) ++self.stale_payload_rejections;
      ++self.state_ingest_rejections;
      self.log_state_rejection(queue_age > self.parameters.max_state_age_us ?
          "age" : "ingest", value, now);
      ++self.rejected_states;
      if ((value.position_valid_mask() & self.parameters.configured_mask) != self.parameters.configured_mask)
        self.system.invalidate_transport_state();
      return;
    }
    if (raw_idle) self.system.invalidate_transport_state();
    if (raw_idle && self.stop_latched) self.stop_idle_observed = true;
    self.last_observation_raw_idle = raw_idle;
    self.heartbeat.observe_state(now);
    self.heartbeat_warned = false;
    self.heartbeat_stopped = false;
    if (self.last_freshness_ticket != value.freshness_ticket() ||
        self.last_state_boot != value.boot_epoch() ||
        self.last_state_session != value.session_id() ||
        self.last_state_reference != value.reference_epoch() ||
        self.last_state_motion != value.motion_epoch())
      self.last_ticket_received_us = now;
    self.last_freshness_ticket = value.freshness_ticket();
    if (!self.has_sequence) {
      self.producer_anchor_us = value.produced_at_us();
      self.clock_anchor_offset_us = observed_offset;
      self.clock_anchor_received_us = now;
    }
    self.producer_offset_us = minimum_offset;
    self.last_produced_us = value.produced_at_us();
    if (!raw_idle) self.acknowledge_command(value);
    {
      std::lock_guard<std::mutex> lock(self.management->mutex);
      std::copy(std::begin(bytes),std::end(bytes),self.management->observation.state.begin());
      self.management->observation.state_received_us=now;
    }
    if (!raw_idle) {
      self.system.set_state_deadline(saturating_state_deadline(now, self.parameters.max_state_age_us));
      self.observed_hold={};
      if(value.core_state()==kWireHold && value.hold_ready() &&
         (value.position_valid_mask() & self.parameters.configured_mask)==self.parameters.configured_mask) {
        self.observed_hold.boot_epoch=value.boot_epoch();
        self.observed_hold.session_id=value.session_id();
        self.observed_hold.reference_epoch=value.reference_epoch();
        self.observed_hold.motion_epoch=value.motion_epoch();
        self.observed_hold.committed_seq=value.committed_producer_seq();
        self.observed_hold.received_us=now;
        std::copy(std::begin(value.position_rad()),std::end(value.position_rad()),
                  self.observed_hold.position.begin());
      }
    }
    self.last_sequence = value.state_seq();
    self.last_state_boot = value.boot_epoch();
    self.last_state_session = value.session_id();
    self.last_state_reference = value.reference_epoch();
    self.last_state_motion = value.motion_epoch();
    self.last_state_management_epoch = value.management_epoch();
    self.has_sequence = true;
    self.last_received_us = now;
  }

  static void receive_evidence(dp::comm::BorrowedMessage<Evidence> borrowed, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    Evidence value;
    if (borrowed.copy_to(&value) != dp::comm::Result::kOk) {
      std::fprintf(stderr,"RAW_EVIDENCE_DIAG copy_failed\n");
      self.raw_evidence.reject_invalid();
      self.failed = true;
      return;
    }
    if (self.boot == self.historical_run_boot && self.historical_run_id != 0U &&
        value.run_id() == self.historical_run_id) return;
    RawEvidencePage page{};
    const auto received_at_us = transport_steady_now_us();
    auto *binding = self.find_evidence_binding(value.run_id());
    const bool canceled_refresh_first_page =
        self.last_clean_cancel_run != 0U &&
        self.last_clean_cancel_run == self.status.run_id() &&
        self.last_clean_cancel_boot == self.boot &&
        self.last_clean_cancel_session == self.status.session_id() &&
        self.status.core_state() == 7U && self.status.final_state() == 0U;
    const bool completed_refresh_first_page =
        self.status.core_state() == kWireHold &&
        self.status.final_state() == 6U && self.status.motion_complete() &&
        self.last_terminal_chain_run == self.status.run_id() &&
        std::any_of(std::begin(self.status.evidence_chain_sha256()),
            std::end(self.status.evidence_chain_sha256()),
            [](auto byte) { return byte != 0U; }) &&
        std::equal(self.last_terminal_chain.begin(), self.last_terminal_chain.end(),
            self.status.evidence_chain_sha256()) &&
        self.raw_evidence.failure() == RawEvidenceCollector::Failure::kNone;
    const auto flushed_terminal = !binding && value.evidence_seq() == 0U
        ? self.management->raw_evidence_terminal_flushed() : RawEvidenceTerminalFlushed{};
    // A successor page may arrive after terminal chain validation but before
    // the writer publishes its flush receipt. Binding is provisional only.
    const bool terminal_writer_completed = self.accepted_completed_terminal.run_id != 0U &&
        self.raw_evidence.terminal_written_run_id() >= self.accepted_completed_terminal.run_id;
    const auto &completed_terminal = terminal_writer_completed ||
        (flushed_terminal.run_id >= self.accepted_completed_terminal.run_id &&
         flushed_terminal.run_id != 0U)
        ? flushed_terminal : self.accepted_completed_terminal;
    const bool completed_terminal_page_first_page =
        self.status.core_state() == kWireEnablingHold &&
        self.status.final_state() == 0U &&
        self.status.motion_epoch() == value.motion_epoch() &&
        completed_terminal.boot_epoch == self.boot &&
        completed_terminal.boot_epoch == self.status.boot_epoch() &&
        completed_terminal.session_id == self.status.session_id() &&
        completed_terminal.run_id == self.status.run_id() &&
        completed_terminal.run_id == self.last_terminal_chain_run &&
        completed_terminal.attempt_kind == 1U &&
        completed_terminal.parent_run_id == 0U &&
        completed_terminal.final_state == 6U &&
        completed_terminal.motion_complete && !completed_terminal.evidence_lost &&
        !completed_terminal.release_committed && completed_terminal.chain_valid &&
        completed_terminal.first_failure_source == 0U &&
        completed_terminal.first_failure_axis_mask == 0U &&
        self.raw_evidence.failure() == RawEvidenceCollector::Failure::kNone;
    const bool prior_provisional_completed = self.provisional_evidence_binding &&
        self.provisional_evidence_binding->run_id == self.status.run_id() &&
        self.provisional_evidence_binding->boot_epoch == completed_terminal.boot_epoch &&
        self.provisional_evidence_binding->session_id == completed_terminal.session_id &&
        (!self.provisional_release_chain ||
         *self.provisional_release_chain == self.last_terminal_chain) &&
        completed_terminal_page_first_page;
    const bool next_motion_first_page = !binding &&
        (!self.provisional_evidence_binding || prior_provisional_completed) &&
        value.schema_major() == 2U && value.schema_minor() == 4U &&
        value.evidence_seq() == 0U && value.attempt_kind() == 1U &&
        value.parent_run_id() == 0U && value.category() == 1U &&
        value.group_count() >= 1U && value.group_id() != 0U &&
        value.motion_epoch() != 0U && value.final() && !value.evidence_lost() &&
        value.final_state() == 0U && value.terminal_reason() == 0U &&
        !value.motion_complete() && !value.release_committed() &&
        value.first_failure_source() == 0U && value.first_failure_axis_mask() == 0U &&
        value.expected_mask() == 0x3fU && value.submitted_mask() == 0x3fU &&
        value.sent_mask() == 0x3fU && value.failed_mask() == 0U &&
        !self.stop_latched && !self.failed && self.boot != 0U &&
        self.status.boot_epoch() == self.boot && self.status.session_id() != 0U &&
        self.status.reference_epoch() != 0U &&
        self.status.attempt_kind() == 1U && self.status.parent_run_id() == 0U &&
        !self.status.evidence_lost() &&
        self.status.run_id() != 0U && self.status.run_id() != UINT64_MAX &&
        (completed_refresh_first_page || completed_terminal_page_first_page ||
         canceled_refresh_first_page) &&
        self.last_status_received_us != 0U &&
        received_at_us >= self.last_status_received_us &&
        received_at_us - self.last_status_received_us <= self.parameters.max_state_age_us &&
        value.run_id() == self.status.run_id() + 1U;
    if (next_motion_first_page) {
      const RawEvidenceBinding inferred{value.run_id(), self.status.boot_epoch(),
          self.status.session_id(), 0U, 1U};
      if (self.evidence_bindings.observe(inferred,
              self.raw_evidence.terminal_written_run_id()) &&
          self.raw_evidence.expect_run(inferred.run_id)) {
        self.provisional_evidence_binding = inferred;
        self.provisional_release_chain.reset();
        binding = self.find_evidence_binding(value.run_id());
      }
    }
    if (value.schema_major() != 2U || value.schema_minor() != 4U ||
        !binding || binding->attempt_kind != value.attempt_kind() ||
        binding->parent_run_id != value.parent_run_id() ||
        wire::DummyAttemptEvidenceV2Codec::encode(value, page.raw.data(), page.raw.size()) !=
            wire::CodecResult::kOk) {
      std::fprintf(stderr,
          "RAW_EVIDENCE_DIAG rejected run=%llu seq=%u kind=%u parent=%llu binding=%u binding_kind=%u binding_parent=%llu status_run=%llu status_core=%u status_motion=%u page_motion=%u terminal_written_run=%llu terminal_receipt_run=%llu terminal_receipt_final=%u failure=%u\n",
          static_cast<unsigned long long>(value.run_id()), value.evidence_seq(),
          value.attempt_kind(), static_cast<unsigned long long>(value.parent_run_id()),
          binding ? 1U : 0U, binding ? binding->attempt_kind : 0U,
          static_cast<unsigned long long>(binding ? binding->parent_run_id : 0U),
          static_cast<unsigned long long>(self.status.run_id()),
          self.status.core_state(), self.status.motion_epoch(), value.motion_epoch(),
          static_cast<unsigned long long>(self.raw_evidence.terminal_written_run_id()),
          static_cast<unsigned long long>(flushed_terminal.run_id),
          flushed_terminal.final_state,
          static_cast<unsigned>(self.raw_evidence.failure()));
      self.raw_evidence.reject_invalid();
      self.failed = true;
      return;
    }
    std::copy(std::begin(value.previous_chain_sha256()),
              std::end(value.previous_chain_sha256()), page.previous_chain_sha256.begin());
    std::copy(std::begin(value.chain_sha256()),
              std::end(value.chain_sha256()), page.chain_sha256.begin());
    page.boot_epoch = binding->boot_epoch;
    page.session_id = binding->session_id;
    page.run_id = value.run_id();
    page.parent_run_id = value.parent_run_id();
    page.evidence_seq = value.evidence_seq();
    page.group_count = value.group_count();
    page.terminal_reason = value.terminal_reason();
    page.attempt_kind = value.attempt_kind();
    page.category = value.category();
    page.final_state = value.final_state();
    page.final = value.final() != 0U;
    page.evidence_lost = value.evidence_lost();
    if (!self.raw_evidence.offer_page(page, received_at_us)) {
      std::fprintf(stderr,
          "RAW_EVIDENCE_DIAG collector_rejected run=%llu seq=%u kind=%u category=%u failure=%u\n",
          static_cast<unsigned long long>(page.run_id), page.evidence_seq,
          page.attempt_kind, page.category,
          static_cast<unsigned>(self.raw_evidence.failure()));
      self.failed = true;
    }
    else if(page.category==4U) {
      self.last_terminal_chain_run=page.run_id;
      self.last_terminal_chain=page.chain_sha256;
      self.accepted_completed_terminal = page.attempt_kind==1U &&
          page.parent_run_id==0U && page.final_state==6U &&
          value.motion_complete() && !page.evidence_lost &&
          !value.release_committed() && value.first_failure_source()==0U &&
          value.first_failure_axis_mask()==0U
          ? RawEvidenceTerminalFlushed{page.boot_epoch,page.session_id,page.run_id,
                page.parent_run_id,page.attempt_kind,page.final_state,0U,0U,
                false,true,false,true,0U,false}
          : RawEvidenceTerminalFlushed{};
      if(self.accepted_completed_terminal.run_id!=0U) {
        auto &witness=self.completed_terminal_chains[self.next_completed_terminal_chain];
        witness={page.boot_epoch,page.session_id,page.run_id,page.chain_sha256};
        self.next_completed_terminal_chain=
            (self.next_completed_terminal_chain+1U)%self.completed_terminal_chains.size();
      }
      if (page.attempt_kind == 1U && page.parent_run_id == 0U &&
          page.final_state == 7U && value.first_failure_source() == 0U &&
          value.first_failure_axis_mask() == 0U && !page.evidence_lost &&
          !value.motion_complete() && !value.release_committed()) {
        self.last_clean_cancel_boot=page.boot_epoch;
        self.last_clean_cancel_session=page.session_id;
        self.last_clean_cancel_run=page.run_id;
      } else {
        self.last_clean_cancel_run=0U;
      }
      if (self.provisional_evidence_binding && self.provisional_release_chain &&
          self.provisional_evidence_binding->run_id == page.run_id &&
          *self.provisional_release_chain != page.chain_sha256) {
        self.raw_evidence.reject_invalid();
        self.failed = true;
      }
    }
  }

  static void management_reply(dp::comm::Result result,const Management::Response *response,
      const dp::comm::ResponseInfo &,void *context) {
    auto &request=*static_cast<DeviceManagementRequest *>(context);
    if(result!=dp::comm::Result::kOk || !response ||
       wire::DummyManageV2ResponseCodec::encode(*response,request.response.data(),request.response.size())!=wire::CodecResult::kOk) {
      log_management_handoff(request,"reply_failed",result);
      request.failed=true; return;
    }
    request.complete.store(true,std::memory_order_release);
  }

  std::shared_ptr<DeviceManagementRequest> make_stop_request() noexcept {
    if (status.boot_epoch() == 0U || status.profile_id() != parameters.profile_id ||
        authorized_motion_epoch == 0U || authorized_management_epoch == 0U) return {};
    Management::Request request;
    request.set_schema_major(2U);
    request.set_schema_minor(4U);
    std::uint8_t device[16]{}, client[16]{};
    std::copy(parameters.device_id.begin(), parameters.device_id.end(), device);
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), client);
    request.set_device_id(device);
    request.set_client_instance_id(client);
    const auto request_id = take_stop_request_id();
    if (request_id == 0U) return {};
    request.set_request_id(request_id);
    request.set_request_seq(management->reserve_request_sequence());
    request.set_operation(kWireDisableAndRelease);
    request.set_expected_boot_epoch(status.boot_epoch());
    request.set_expected_session_id(status.session_id());
    request.set_expected_reference_epoch(status.reference_epoch());
    request.set_expected_motion_epoch(authorized_motion_epoch);
    request.set_expected_management_epoch(authorized_management_epoch);
    request.set_expected_profile_id(parameters.profile_id);
    request.set_freshness_ticket(last_freshness_ticket);
    request.set_configured_mask(parameters.configured_mask);
    auto pending = std::make_shared<DeviceManagementRequest>();
    if (wire::DummyManageV2RequestCodec::encode(
            request, pending->request.data(), pending->request.size()) != wire::CodecResult::kOk) return {};
    return pending;
  }

  bool stop_reply_matches(const Management::Request &original,
                          const Management::Request &sent,
                          const Management::Response &response,
                          bool released_identity = false) const noexcept {
    const bool same_owner = response.session_id() == original.expected_session_id() &&
        response.reference_epoch() == original.expected_reference_epoch();
    const bool released_owner = released_identity && response.session_id() == 0U &&
        response.reference_epoch() == 0U;
    return original.operation() == kWireDisableAndRelease &&
        (sent.operation() == kWireDisableAndRelease ||
         (sent.operation() == kWireGetOperation &&
          sent.queried_operation_id() == original.request_id() &&
          sent.configured_mask() == original.configured_mask())) &&
        response.schema_major() == 2U && response.schema_minor() == 4U &&
        response.receipt() == 1U && response.request_id() == sent.request_id() &&
        response.boot_epoch() == original.expected_boot_epoch() &&
        (same_owner || released_owner) &&
        response.profile_id() == original.expected_profile_id() &&
        sequence_at_or_after(response.motion_epoch(), original.expected_motion_epoch()) &&
        sequence_at_or_after(response.management_epoch(), original.expected_management_epoch()) &&
        std::equal(parameters.device_id.begin(), parameters.device_id.end(), response.device_id()) &&
        std::equal(parameters.client_instance_id.begin(), parameters.client_instance_id.end(),
                   response.client_instance_id());
  }

  std::shared_ptr<DeviceManagementRequest> make_stop_query(
      const Management::Request &original) noexcept {
    Management::Request query = original;
    const auto request_id = take_stop_request_id();
    if (request_id == 0U) return {};
    query.set_request_id(request_id);
    query.set_request_seq(management->reserve_request_sequence());
    query.set_operation(kWireGetOperation);
    query.set_queried_operation_id(original.request_id());
    auto pending = std::make_shared<DeviceManagementRequest>();
    if (wire::DummyManageV2RequestCodec::encode(
            query, pending->request.data(), pending->request.size()) != wire::CodecResult::kOk) return {};
    return pending;
  }

  void publish_pending(dp::comm::Publisher<Command> &publisher) noexcept {
    CommandGenerationRecord next{};
    while (system.take_publish_record(&next)) {
      // Complete freezes the terminal sequence; a later old-epoch update must
      // not enter REF's motion ledger after that terminal group was SENT.
      if(superseded_by_pending_complete(next)) continue;
      // No new-epoch write is authorized before a new Begin. Local no-ops do
      // not claim CAN SENT or extend REF's Hold budget; divergent writes Stop.
      const auto action=post_complete_write(next,transport_steady_now_us());
      if(action==PostCompleteWrite::kConflict) {
        ++rejected_commands;
        hold_drain_rejected=true;
        system.request_transport_stop();
      } else if(action==PostCompleteWrite::kAllowed) command_stream.offer(next);
      if(action==PostCompleteWrite::kLocalNoOp &&
         next.identity.boot==status.boot_epoch() &&
         next.identity.session==status.session_id() &&
         next.identity.reference_epoch==status.reference_epoch() &&
         next.identity.motion_epoch==authorized_motion_epoch &&
         status.core_state()==kWireHold) {
        last_processed_hold_identity=next.identity;
        last_processed_hold_generation=next.generation;
        last_processed_hold_disposition=1U;
        last_processed_hold_position=next.position;
      }
    }
    if(pending_complete.request_id!=0U && !pending_complete.dispatched)
      pending_complete.queue_drained=true;
    const auto candidate = command_sent ? std::optional<CommandGenerationRecord>{}
                                        : (command_stream.has_in_flight()
                                               ? command_stream.in_flight_record()
                                               : command_stream.begin_send());
    if (!candidate.has_value()) return;
    const auto &record = *candidate;
    if(!command_sent && superseded_by_pending_complete(record)) {
      command_stream.discard_in_flight();
      return;
    }
    const auto post_complete=!command_sent ? post_complete_write(record,transport_steady_now_us()) :
      PostCompleteWrite::kAllowed;
    if(post_complete!=PostCompleteWrite::kAllowed) {
      if(post_complete==PostCompleteWrite::kLocalNoOp && status.core_state()==kWireHold &&
         record.identity.boot==status.boot_epoch() && record.identity.session==status.session_id() &&
         record.identity.reference_epoch==status.reference_epoch() &&
         record.identity.motion_epoch==authorized_motion_epoch &&
         record.generation>last_processed_hold_generation) {
        last_processed_hold_identity=record.identity;
        last_processed_hold_generation=record.generation;
        last_processed_hold_disposition=1U;
        last_processed_hold_position=record.position;
      }
      command_stream.discard_in_flight();
      if(post_complete==PostCompleteWrite::kConflict) {
        ++rejected_commands;
        system.request_transport_stop();
      }
      if(post_complete==PostCompleteWrite::kConflict) hold_drain_rejected=true;
      return;
    }
    JointStateSnapshot snapshot{};
    const bool inactive_prime = management->inactive_prime_allowed.load(std::memory_order_acquire) ||
        status.core_state() == kWireEnablingHold;
    if (wire_producer_sequence(record.generation) == 0U || !command_record_current(record) ||
        (!system.commands_active() && !inactive_prime) || boot == 0U ||
        record.identity.boot != boot || record.identity.session != status.session_id() ||
        record.identity.reference_epoch != status.reference_epoch() ||
        record.identity.motion_epoch != authorized_motion_epoch || record.identity.ticket == 0U ||
        !bootstrap_ready || stop_latched) {
      if (command_prime_reject_diag_count++ < 16U) fprintf(stderr,
          "COMMAND_PRIME_DIAG reject seq=%u current=%d active=%d inactive_prime=%d boot=%llu/%llu session=%llu/%llu ref=%u/%u motion=%u/%u ticket=%llu ready=%d stop=%d\n",
          wire_producer_sequence(record.generation), command_record_current(record),
          system.commands_active(), inactive_prime,
          static_cast<unsigned long long>(record.identity.boot), static_cast<unsigned long long>(boot),
          static_cast<unsigned long long>(record.identity.session),
          static_cast<unsigned long long>(status.session_id()), record.identity.reference_epoch,
          status.reference_epoch(), record.identity.motion_epoch, authorized_motion_epoch,
          static_cast<unsigned long long>(record.identity.ticket), bootstrap_ready, stop_latched);
      ++rejected_commands;
      hold_drain_rejected=true;
      command_stream.discard_in_flight();
      command_sent = false;
      return;
    }
    // On a busy state read, keep the original unsent generation. Authority and
    // freshness are checked again before every retry; no new sample is made.
    if(!system.copy_state(&snapshot)) {
      if (record.identity.motion_epoch == 1U && enabling_copy_failure_log_count++ < 3U)
        fprintf(stderr, "COMMAND_ENABLE_DIAG copy_state_unavailable seq=%u status_core=%u status_motion=%u authorized_motion=%u\n",
                wire_producer_sequence(record.generation), status.core_state(),
                status.motion_epoch(), authorized_motion_epoch);
      if (!logged_copy_state_failure) {
        logged_copy_state_failure = true;
        fprintf(stderr, "COMMAND_PRIME_DIAG copy_state_unavailable seq=%u boot=%llu status_motion=%u authorized_motion=%u\n",
                wire_producer_sequence(record.generation),
                static_cast<unsigned long long>(boot), status.motion_epoch(), authorized_motion_epoch);
      }
      return;
    }
    // The current device profiles expire tickets after 50 ms from issuance.
    // First receipt is only an upper bound on issuance, so repeated States
    // carrying the same ticket must not renew the send window.
    const auto publish_now = transport_steady_now_us();
    constexpr std::uint64_t kTicketReceiveWindowUs = 50000U;
    if (last_freshness_ticket == 0U || last_ticket_received_us == 0U ||
        publish_now < last_ticket_received_us ||
        publish_now - last_ticket_received_us >= kTicketReceiveWindowUs ||
        last_state_boot != record.identity.boot ||
        last_state_session != record.identity.session ||
        last_state_reference != record.identity.reference_epoch ||
        last_state_motion != record.identity.motion_epoch) return;
    if(exported_motion_ready && complete_boundary.boot_epoch==record.identity.boot &&
       complete_boundary.session_id==record.identity.session &&
       complete_boundary.hold_epoch==record.identity.motion_epoch &&
       sequence_newer(wire_producer_sequence(record.generation),complete_boundary.terminal_seq))
      refresh_in_flight_seq=wire_producer_sequence(record.generation);
    Command message;
    message.set_schema_major(2U);
    message.set_schema_minor(4U);
    std::uint8_t device[16]{}, client[16]{};
    std::copy(parameters.device_id.begin(), parameters.device_id.end(), device);
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), client);
    message.set_device_id(device);
    message.set_client_instance_id(client);
    message.set_boot_epoch(record.identity.boot);
    message.set_session_id(record.identity.session);
    message.set_reference_epoch(record.identity.reference_epoch);
    message.set_motion_epoch(record.identity.motion_epoch);
    message.set_profile_id(parameters.profile_id);
    message.set_producer_seq(wire_producer_sequence(record.generation));
    message.set_freshness_ticket(last_freshness_ticket);
    message.set_configured_mask(parameters.configured_mask);
    double position[6]{};
    for (std::size_t axis = 0U; axis < record.position.size(); ++axis)
      position[axis] = (parameters.configured_mask & (1U << axis)) != 0U
                           ? record.position[axis]
                           : 0.0;
    message.set_position_rad(position);
    const auto published=publisher.publish(message);
    if(published==dp::comm::Result::kOk || published==dp::comm::Result::kBusy ||
       published==dp::comm::Result::kQueueFull) reserve_complete_prime(record);
    if(published!=dp::comm::Result::kOk && published!=dp::comm::Result::kBusy &&
       published!=dp::comm::Result::kQueueFull &&
       refresh_in_flight_seq==wire_producer_sequence(record.generation)) {
      refresh_in_flight_seq=0U;
      system.request_transport_stop();
    }
    if (message.motion_epoch() == 1U && enabling_publish_log_count++ < 3U)
      fprintf(stderr, "COMMAND_ENABLE_DIAG publish seq=%u result=%u status_core=%u authorized_motion=%u\n",
              message.producer_seq(), static_cast<unsigned>(published), status.core_state(),
              authorized_motion_epoch);
    if (!logged_publish_attempt || (published==dp::comm::Result::kOk && !logged_publish_success)) {
      logged_publish_attempt = true;
      if (published==dp::comm::Result::kOk) logged_publish_success = true;
      fprintf(stderr, "COMMAND_PRIME_DIAG publish seq=%u result=%u boot=%llu session=%llu ref=%u motion=%u\n",
              message.producer_seq(), static_cast<unsigned>(published),
              static_cast<unsigned long long>(message.boot_epoch()),
              static_cast<unsigned long long>(message.session_id()),
              message.reference_epoch(), message.motion_epoch());
    }
    if(published==dp::comm::Result::kBusy || published==dp::comm::Result::kQueueFull) return;
    if(published==dp::comm::Result::kOk) {
      ++commands;
      command_published_this_boot = true;
      command_sent = true;
      ack_mismatch_diag_count = 0U;
      const auto ack_timeout_us = parameters.group_sent_ack_timeout_us != 0U
          ? parameters.group_sent_ack_timeout_us : parameters.max_state_age_us;
      group_sent_ack.begin(record, transport_steady_now_us(), ack_timeout_us,
                           status.command_reject_count(), status.committed_producer_seq());
    } else {
      ++rejected_commands;
      hold_drain_rejected=true;
      command_stream.discard_in_flight();
      command_sent = false;
    }
  }

  bool run_connected() {
    const dp::comm::StaticStorageConfig storage_config{1U, 2U, 3U, 0U, 8U,
        wire::DummyDeviceStatusV2Codec::kEncodedSize, 96U, 1U, 1U, 1U, 1U};
    dp::comm::StaticStorage storage(storage_config);
    const dp::comm::TransportConfig transport{"peer", parameters.endpoint.c_str(), "", "dummy_ros_control"};
    auto initialized = dp::comm::Context::init(storage,
        dp::comm::InitOptions{dp::comm::MemoryMode::kStaticOnly, &transport, nullptr});
    if (!initialized.has_value()) {
      fprintf(stderr, "device transport context init failed result=%u endpoint=%s\\n",
              static_cast<unsigned>(initialized.result()), parameters.endpoint.c_str());
      return false;
    }
    auto context = std::move(initialized).take_value();
    dp::comm::NodeOptions options{};
    options.product_name = "dummy_robot";
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), options.device_id.bytes);
    options.node_name = "ros_control";
    auto created = context.create_node(options);
    if (!created.has_value()) {
      fprintf(stderr, "device transport node create failed result=%u\\n",
              static_cast<unsigned>(created.result()));
      return false;
    }
    auto node = std::move(created).take_value();
    static constexpr char hex[] = "0123456789abcdef";
    std::string route = "/dpsdk/dummy_robot/";
    for (auto byte : parameters.device_id) { route += hex[byte >> 4U]; route += hex[byte & 15U]; }
    route += "/ref_control/";
    auto state_result = node.create_subscription<State>((route + "state_v2").c_str(), {}, receive_state, this);
    auto status_result = node.create_subscription<Status>((route + "status_v2").c_str(), {}, receive_status, this);
    auto command_result = node.create_publisher<Command>((route + "command_v2").c_str(), {});
    auto management_result=node.create_client<Management>((route+"manage_v2").c_str());
    if (!state_result.has_value() || !status_result.has_value() || !command_result.has_value() || !management_result.has_value()) {
      fprintf(stderr, "device transport endpoint setup failed state=%u status=%u command=%u management=%u\\n",
              static_cast<unsigned>(state_result.result()), static_cast<unsigned>(status_result.result()),
              static_cast<unsigned>(command_result.result()), static_cast<unsigned>(management_result.result()));
      return false;
    }
    auto state_subscription = std::move(state_result).take_value();
    auto status_subscription = std::move(status_result).take_value();
    std::optional<dp::comm::Subscription<Evidence>> evidence_subscription;
    if (!parameters.raw_evidence_path.empty()) {
      auto evidence_result = node.create_subscription<Evidence>(
          (route + "attempt_evidence_v2").c_str(), {}, receive_evidence, this);
      if (!evidence_result.has_value()) {
        fprintf(stderr, "device transport evidence subscription failed result=%u\n",
                static_cast<unsigned>(evidence_result.result()));
        return false;
      }
      evidence_subscription.emplace(std::move(evidence_result).take_value());
    }
    auto command_publisher = std::move(command_result).take_value();
    auto management_client=std::move(management_result).take_value();
    dp::comm::StaticExecutor executor;
    const auto add_result = executor.add_node(node);
    const auto activate_result = add_result == dp::comm::Result::kOk ? node.activate() : add_result;
    if (add_result != dp::comm::Result::kOk || activate_result != dp::comm::Result::kOk) {
      fprintf(stderr, "device transport activate failed add=%u activate=%u\\n",
              static_cast<unsigned>(add_result), static_cast<unsigned>(activate_result));
      return false;
    }
    producer_anchor_us = 0U;
    producer_offset_us = 0;
    clock_anchor_offset_us = 0;
    clock_anchor_received_us = 0U;
    ready = true;
    management->ready=true;
    std::shared_ptr<DeviceManagementRequest> active_management;
    std::shared_ptr<DeviceManagementRequest> processed_management;
    std::optional<Management::Request> deferred_complete;
    std::optional<ManagementAdmissionRetry> complete_admission_retry;
    std::optional<Management::Request> admission_retry_request;
    std::optional<ManagementAdmissionRetry> admission_retry_state;
    std::shared_ptr<DeviceManagementRequest> pending_stop;
    std::shared_ptr<DeviceManagementRequest> stop_query;
    std::optional<ManagementAdmissionRetry> stop_admission_retry;
    std::optional<ManagementAdmissionRetry> stop_query_admission_retry;
    Management::Request stop_original;
    bool stop_received = false;
    bool stop_terminal = false;
    bool stop_receipt_diagnostic_emitted = false;
    std::uint64_t stop_deadline_us = 0U, next_stop_query_us = 0U;
    std::uint64_t stop_terminal_at_us = 0U, stop_operation_id = 0U;
    std::optional<dp::comm::RequestHandle<Management>> management_handle;
    auto capture_sent_request = [&](const DeviceManagementRequest &pending) {
      if (!parameters.raw_evidence_path.empty() &&
          !raw_evidence.offer_management_request(pending.request.data(), pending.request.size(),
                                                 transport_steady_now_us())) failed = true;
    };
    auto capture_completed_reply = [&](const std::shared_ptr<DeviceManagementRequest> &pending) {
      if (!pending || pending == processed_management ||
          !pending->complete.load(std::memory_order_acquire)) return;
      if (!parameters.raw_evidence_path.empty() &&
          !raw_evidence.offer_management_response(pending->response.data(),
              pending->response.size(), transport_steady_now_us())) failed = true;
      processed_management = pending;
    };
    auto fail_stop = [&](const char *reason) {
      if (!stop_confirmation_pending.exchange(false)) return;
      fprintf(stderr, "device transport UNCONFIRMED_STOP: %s\n", reason);
      ++unconfirmed_stop_requests;
      pending_stop.reset();
      stop_query.reset();
      stop_admission_retry.reset();
      stop_query_admission_retry.reset();
      management_handle.reset();
    };
    while (running.load(std::memory_order_acquire)) {
      if(active_management && active_management->canceled) {
        log_management_handoff(*active_management,"canceled");
        management_handle.reset(); active_management.reset();
        deferred_complete.reset(); complete_admission_retry.reset(); pending_complete={};
        admission_retry_request.reset(); admission_retry_state.reset();
      }
      std::shared_ptr<DeviceManagementRequest> queued;
      { std::lock_guard<std::mutex> lock(management->mutex); queued=std::move(management->queued); }
      if (queued && stop_confirmation_pending.load(std::memory_order_acquire))
        { log_management_handoff(*queued,"stop_pending");
          queued->failed.store(true, std::memory_order_release); }
      else if(queued) {
        if(admission_retry_request && active_management && !active_management->canceled)
          active_management->failed.store(true,std::memory_order_release);
        admission_retry_request.reset(); admission_retry_state.reset();
        if(deferred_complete) {
          if(active_management) {
            log_management_handoff(*active_management,"complete_superseded_by_queue");
            active_management->failed=true;
          }
          deferred_complete.reset();
          complete_admission_retry.reset();
          pending_complete={};
        }
        management_handle.reset(); active_management=std::move(queued);
        Management::Request decoded;
        if(wire::DummyManageV2RequestCodec::decode(active_management->request.data(),active_management->request.size(),&decoded)!=wire::CodecResult::kOk) {
          log_management_handoff(*active_management,"decode_failed");
          active_management->failed=true;
        }
        else {
          management->observe_request_sequence(decoded.request_seq());
          if (decoded.operation() == kWireDisableAndRelease) {
            gate_release = {decoded.request_id(), decoded.expected_boot_epoch(),
                decoded.expected_session_id(), decoded.expected_reference_epoch(),
                decoded.expected_motion_epoch(), decoded.expected_management_epoch(),
                0U, 0U, false, false};
          } else if (decoded.operation() != kWireGetOperation && decoded.operation() != 0U &&
                     decoded.operation() != 1U) {
            gate_release = {};
          }
          const bool valid_complete=decoded.operation()==8U && boot!=0U &&
               decoded.expected_boot_epoch()==boot &&
               decoded.expected_session_id()==status.session_id() &&
               decoded.expected_reference_epoch()==status.reference_epoch() &&
               decoded.expected_motion_epoch()==authorized_motion_epoch &&
               decoded.expected_management_epoch()==authorized_management_epoch &&
               decoded.expected_motion_epoch()!=UINT32_MAX &&
               decoded.expected_profile_id()==parameters.profile_id &&
               decoded.configured_mask()==parameters.configured_mask &&
               std::equal(parameters.device_id.begin(),parameters.device_id.end(),decoded.device_id()) &&
               std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),
                          decoded.client_instance_id());
          if(decoded.operation()==8U) {
            if(complete_binding_diag_count++<3U)
              std::fprintf(stderr,
                  "COMPLETE_BINDING_DIAG valid=%d req=%llu boot=%llu/%llu session=%llu/%llu ref=%u/%u motion=%u/%u management=%u/%u status_management=%u state_management=%u profile=%u/%u mask=%u/%u device_match=%d client_match=%d terminal_seq=%u\n",
                  valid_complete,static_cast<unsigned long long>(decoded.request_id()),
                  static_cast<unsigned long long>(decoded.expected_boot_epoch()),
                  static_cast<unsigned long long>(boot),
                  static_cast<unsigned long long>(decoded.expected_session_id()),
                  static_cast<unsigned long long>(status.session_id()),
                  decoded.expected_reference_epoch(),status.reference_epoch(),
                  decoded.expected_motion_epoch(),authorized_motion_epoch,
                  decoded.expected_management_epoch(),authorized_management_epoch,
                  status.management_epoch(),last_state_management_epoch,
                  decoded.expected_profile_id(),parameters.profile_id,
                  decoded.configured_mask(),parameters.configured_mask,
                  std::equal(parameters.device_id.begin(),parameters.device_id.end(),decoded.device_id()),
                  std::equal(parameters.client_instance_id.begin(),parameters.client_instance_id.end(),
                             decoded.client_instance_id()),decoded.terminal_producer_seq());
            if(!valid_complete) {
              log_management_handoff(*active_management,"complete_binding_failed");
              active_management->failed=true; continue;
            }
          }
          if(valid_complete && decoded.terminal_producer_seq()!=0U) {
            complete_boundary={boot,status.session_id(),status.reference_epoch(),
              authorized_motion_epoch+1U,decoded.terminal_producer_seq()};
            complete_boundary.prior_run_id=status.run_id();
            complete_boundary_hold_diag_logged=false;
            exported_motion_ready=false;
            management->post_complete_begin_ready.store(false,std::memory_order_release);
            refresh_active_run_id=last_refresh_acked_us=0U;
            refresh_in_flight_seq=0U;
            pending_complete={decoded.request_id(),boot,status.session_id(),
              transport_steady_now_us(),status.reference_epoch(),authorized_motion_epoch,
              authorized_management_epoch,decoded.terminal_producer_seq()};
            deferred_complete=decoded;
          }
          if(deferred_complete) continue;
          auto sent=management_client.async_request(decoded,management_reply,active_management.get(),dp::comm::Duration::from_milliseconds(1000));
          if(sent.has_value()) {
            capture_sent_request(*active_management);
            if(valid_complete)
              complete_boundary={boot,status.session_id(),status.reference_epoch(),
                authorized_motion_epoch+1U,decoded.terminal_producer_seq()};
            if(valid_complete) complete_boundary.prior_run_id=status.run_id();
            if(valid_complete) management->post_complete_begin_ready.store(false,std::memory_order_release);
            if(valid_complete)
              pending_complete={decoded.request_id(),boot,status.session_id(),
                transport_steady_now_us(),status.reference_epoch(),authorized_motion_epoch,
                authorized_management_epoch,decoded.terminal_producer_seq(),true,true};
            if (motion_authorizing_operation(decoded.operation())) {
              last_motion_request_id = decoded.request_id();
              last_motion_operation_id = 0U;
            }
            management_handle.emplace(std::move(sent).take_value());
          }
          else {
            if(sent.result()==dp::comm::Result::kQueueFull &&
               (decoded.operation()!=8U || decoded.terminal_producer_seq()==0U)) {
              admission_retry_request=decoded;
              admission_retry_state.emplace(transport_steady_now_us());
              (void)admission_retry_state->on_result(sent.result(),transport_steady_now_us(),false);
              log_management_handoff(*active_management,"async_request_would_block",sent.result());
            } else {
              log_management_handoff(*active_management,"async_request_failed",sent.result());
              active_management->failed=true;
            }
          }
        }
      }
      const auto stop_kind = system.take_stop_request_kind();
      if(deferred_complete && pending_complete.queue_drained &&
         stop_kind == GuardedZenbeddedSystem::StopRequestKind::kNone) {
        const auto &request=*deferred_complete;
        const bool same_ack=last_ack_seq==pending_complete.terminal_seq &&
          last_ack_identity.boot==pending_complete.boot_epoch &&
          last_ack_identity.session==pending_complete.session_id &&
          last_ack_identity.reference_epoch==pending_complete.reference_epoch &&
          last_ack_identity.motion_epoch==pending_complete.motion_epoch;
        const bool newer_ack=last_ack_identity.boot==pending_complete.boot_epoch &&
          last_ack_identity.session==pending_complete.session_id &&
          last_ack_identity.motion_epoch==pending_complete.motion_epoch &&
          sequence_newer(last_ack_seq,pending_complete.terminal_seq);
        const auto in_flight=command_stream.in_flight_record();
        const bool newer_in_flight=in_flight &&
          in_flight->identity.boot==pending_complete.boot_epoch &&
          in_flight->identity.session==pending_complete.session_id &&
          in_flight->identity.motion_epoch==pending_complete.motion_epoch &&
          sequence_newer(wire_producer_sequence(in_flight->generation),pending_complete.terminal_seq);
        const auto elapsed=transport_steady_now_us()-pending_complete.started_us;
        if(newer_ack || newer_in_flight || elapsed>2000000U ||
           (complete_admission_retry &&
            complete_admission_retry->expired(transport_steady_now_us()))) {
          log_management_handoff(*active_management,"complete_deferred_failed");
          active_management->failed=true;
          deferred_complete.reset(); pending_complete={};
          complete_admission_retry.reset();
        } else if(same_ack && !group_sent_ack.pending() &&
                  !command_stream.has_in_flight() && !command_stream.has_pending() &&
                  (!complete_admission_retry ||
                   complete_admission_retry->ready(transport_steady_now_us()))) {
          auto sent=management_client.async_request(request,management_reply,
              active_management.get(),dp::comm::Duration::from_milliseconds(1000));
          if(sent.has_value()) {
            capture_sent_request(*active_management);
            pending_complete.dispatched=true;
            last_motion_request_id=request.request_id();
            last_motion_operation_id=0U;
            management_handle.emplace(std::move(sent).take_value());
            deferred_complete.reset(); complete_admission_retry.reset();
          } else {
            const auto retry_now=transport_steady_now_us();
            if(sent.result()==dp::comm::Result::kQueueFull && !complete_admission_retry)
              complete_admission_retry.emplace(retry_now);
            if(complete_admission_retry &&
               complete_admission_retry->on_result(sent.result(),retry_now,
                   stop_confirmation_pending.load(std::memory_order_acquire))==
                   ManagementAdmissionRetry::Decision::kRetry) {
              log_management_handoff(*active_management,"async_request_would_block",sent.result());
            } else {
              log_management_handoff(*active_management,"async_request_failed",sent.result());
              active_management->failed=true; pending_complete={};
              deferred_complete.reset(); complete_admission_retry.reset();
            }
          }
        }
      }
      if (stop_kind == GuardedZenbeddedSystem::StopRequestKind::kDeactivation &&
          released_idle_latched()) continue;
      if (stop_kind != GuardedZenbeddedSystem::StopRequestKind::kNone) {
        admission_retry_request.reset(); admission_retry_state.reset();
        if(deferred_complete) {
          if(active_management) {
            log_management_handoff(*active_management,"complete_superseded_by_stop");
            active_management->failed=true;
          }
          deferred_complete.reset(); pending_complete={};
          complete_admission_retry.reset();
        }
        stop_latched = true;
        if (stop_confirmation_pending.load(std::memory_order_acquire))
          fail_stop("new Stop superseded pending confirmation");
        stop_confirmation_pending = true;
        stop_idle_observed = false;
        stop_received = false;
        stop_terminal = false;
        stop_terminal_at_us = 0U;
        stop_operation_id = 0U;
        stop_receipt_diagnostic_emitted = false;
        const auto stop_timeout_us = parameters.stop_confirmation_timeout_us != 0U
            ? parameters.stop_confirmation_timeout_us : parameters.max_state_age_us;
        stop_deadline_us = saturating_state_deadline(transport_steady_now_us(), stop_timeout_us);
        if (active_management && !active_management->complete.load(std::memory_order_acquire)) {
          log_management_handoff(*active_management,"superseded_by_stop");
          active_management->failed.store(true, std::memory_order_release);
        }
        management_handle.reset();
        active_management = make_stop_request();
        if (!active_management) {
          ++stop_request_failures;
          fail_stop("no authorized stop tuple");
        } else {
          Management::Request decoded;
          if (wire::DummyManageV2RequestCodec::decode(active_management->request.data(),
                active_management->request.size(), &decoded) != wire::CodecResult::kOk) {
            active_management->failed = true;
            ++stop_request_failures;
            fail_stop("Stop request encode/decode failed");
          } else {
            auto sent = management_client.async_request(
                decoded, management_reply, active_management.get(), dp::comm::Duration::from_milliseconds(1000));
            if (sent.has_value()) {
              capture_sent_request(*active_management);
              management_handle.emplace(std::move(sent).take_value());
              ++stop_requests_sent;
              pending_stop = active_management;
              stop_original = decoded;
            } else {
              if(sent.result()==dp::comm::Result::kQueueFull) {
                pending_stop=active_management;
                stop_original=decoded;
                stop_admission_retry.emplace(transport_steady_now_us());
                (void)stop_admission_retry->on_result(sent.result(),transport_steady_now_us(),false);
                log_management_handoff(*active_management,"stop_would_block",sent.result());
              } else {
                log_management_handoff(*active_management,"async_request_failed",sent.result());
                active_management->failed = true;
                ++stop_request_failures;
                fail_stop("Stop request dispatch failed");
              }
            }
          }
        }
        invalidate();
      }
      if(stop_admission_retry &&
         (stop_admission_retry->expired(transport_steady_now_us()) ||
          transport_steady_now_us()>stop_deadline_us)) {
        ++stop_request_failures;
        fail_stop("Stop request admission deadline expired");
      }
      if(stop_confirmation_pending.load(std::memory_order_acquire) && pending_stop &&
         stop_admission_retry && stop_admission_retry->ready(transport_steady_now_us())) {
        auto sent=management_client.async_request(stop_original,management_reply,
            pending_stop.get(),dp::comm::Duration::from_milliseconds(1000));
        if(sent.has_value()) {
          capture_sent_request(*pending_stop);
          management_handle.emplace(std::move(sent).take_value());
          ++stop_requests_sent;
          stop_admission_retry.reset();
        } else if(stop_admission_retry->on_result(sent.result(),transport_steady_now_us(),false)==
                  ManagementAdmissionRetry::Decision::kRetry) {
          log_management_handoff(*pending_stop,"stop_would_block",sent.result());
        } else {
          log_management_handoff(*pending_stop,"async_request_failed",sent.result());
          ++stop_request_failures;
          fail_stop("Stop request dispatch failed");
        }
      }
      if(stop_query_admission_retry &&
         (stop_query_admission_retry->expired(transport_steady_now_us()) ||
          transport_steady_now_us()>stop_deadline_us))
        fail_stop("Stop QueryResult admission deadline expired");
      if(stop_confirmation_pending.load(std::memory_order_acquire) && stop_query &&
         stop_query_admission_retry &&
         stop_query_admission_retry->ready(transport_steady_now_us())) {
        Management::Request query;
        if(wire::DummyManageV2RequestCodec::decode(stop_query->request.data(),
              stop_query->request.size(),&query)!=wire::CodecResult::kOk) {
          fail_stop("Stop QueryResult decode failed");
        } else {
          auto sent=management_client.async_request(query,management_reply,
              stop_query.get(),dp::comm::Duration::from_milliseconds(1000));
          if(sent.has_value()) {
            capture_sent_request(*stop_query);
            active_management=stop_query;
            management_handle.emplace(std::move(sent).take_value());
            stop_query_admission_retry.reset();
          } else if(stop_query_admission_retry->on_result(sent.result(),transport_steady_now_us(),false)==
                    ManagementAdmissionRetry::Decision::kRetry) {
            log_management_handoff(*stop_query,"stop_query_would_block",sent.result());
          } else {
            log_management_handoff(*stop_query,"async_request_failed",sent.result());
            fail_stop("Stop QueryResult dispatch failed");
          }
        }
      }
      if(admission_retry_state && admission_retry_state->expired(transport_steady_now_us())) {
        if(active_management) active_management->failed.store(true,std::memory_order_release);
        admission_retry_request.reset(); admission_retry_state.reset();
      }
      if(admission_retry_request && admission_retry_state && active_management &&
         !active_management->canceled &&
         admission_retry_state->ready(transport_steady_now_us()) &&
         !stop_confirmation_pending.load(std::memory_order_acquire)) {
        const auto &request=*admission_retry_request;
        auto sent=management_client.async_request(request,management_reply,
            active_management.get(),dp::comm::Duration::from_milliseconds(1000));
        if(sent.has_value()) {
          capture_sent_request(*active_management);
          if(motion_authorizing_operation(request.operation())) {
            last_motion_request_id=request.request_id();
            last_motion_operation_id=0U;
          }
          management_handle.emplace(std::move(sent).take_value());
          admission_retry_request.reset(); admission_retry_state.reset();
        } else if(admission_retry_state->on_result(sent.result(),transport_steady_now_us(),false)==
                  ManagementAdmissionRetry::Decision::kRetry) {
          log_management_handoff(*active_management,"async_request_would_block",sent.result());
        } else {
          log_management_handoff(*active_management,"async_request_failed",sent.result());
          active_management->failed.store(true,std::memory_order_release);
          admission_retry_request.reset(); admission_retry_state.reset();
        }
      }
      const auto spun=executor.spin_some(dp::comm::Duration::from_milliseconds(2U));
      if (pending_stop && !stop_received && (pending_stop->complete.load(std::memory_order_acquire) ||
                                             pending_stop->failed.load(std::memory_order_acquire))) {
        capture_completed_reply(pending_stop);
        Management::Response response;
        const bool decoded = pending_stop->complete.load(std::memory_order_acquire) &&
            wire::DummyManageV2ResponseCodec::decode(pending_stop->response.data(),
                pending_stop->response.size(), &response) == wire::CodecResult::kOk;
        const bool valid_pending = decoded &&
            stop_reply_matches(stop_original, stop_original, response, true) &&
            response.result_code() == kWirePendingResult && response.transaction_phase() == 1U &&
            response.operation_id() == 0U;
        if (!valid_pending) {
          if (!stop_receipt_diagnostic_emitted) {
            stop_receipt_diagnostic_emitted = true;
            fprintf(stderr, "device transport Stop receipt mismatch: decoded=%u req_id=%llu req_boot=%llu req_session=%llu req_ref=%u req_motion=%u req_management=%u resp_id=%llu resp_boot=%llu resp_session=%llu resp_ref=%u resp_motion=%u resp_management=%u resp_opid=%llu resp_phase=%u resp_result=%u\n",
                    decoded ? 1U : 0U,
                    static_cast<unsigned long long>(stop_original.request_id()),
                    static_cast<unsigned long long>(stop_original.expected_boot_epoch()),
                    static_cast<unsigned long long>(stop_original.expected_session_id()),
                    stop_original.expected_reference_epoch(), stop_original.expected_motion_epoch(),
                    stop_original.expected_management_epoch(),
                    static_cast<unsigned long long>(response.request_id()),
                    static_cast<unsigned long long>(response.boot_epoch()),
                    static_cast<unsigned long long>(response.session_id()),
                    response.reference_epoch(), response.motion_epoch(), response.management_epoch(),
                    static_cast<unsigned long long>(response.operation_id()),
                    response.transaction_phase(), response.result_code());
          }
          fail_stop("invalid Stop Received/Pending receipt");
        } else {
          stop_received = true;
          next_stop_query_us = transport_steady_now_us();
          management_handle.reset();
        }
      }
      if (stop_query && (stop_query->complete.load(std::memory_order_acquire) ||
                         stop_query->failed.load(std::memory_order_acquire))) {
        capture_completed_reply(stop_query);
        Management::Request query;
        Management::Response response;
        if (!stop_query->complete.load(std::memory_order_acquire) ||
            wire::DummyManageV2RequestCodec::decode(stop_query->request.data(),
                stop_query->request.size(), &query) != wire::CodecResult::kOk ||
            wire::DummyManageV2ResponseCodec::decode(stop_query->response.data(),
                stop_query->response.size(), &response) != wire::CodecResult::kOk ||
            !stop_reply_matches(stop_original, query, response, true)) {
          fail_stop("invalid Stop QueryResult response");
        } else if (response.result_code() == kWirePendingResult &&
                   response.transaction_phase() == 2U && response.operation_id() == 0U) {
          stop_query.reset();
          management_handle.reset();
          next_stop_query_us = saturating_state_deadline(transport_steady_now_us(),
              parameters.control_period_us);
        } else if (response.result_code() == 0U && response.transaction_phase() == 3U &&
                   response.operation_id() != 0U && response.release_committed() &&
                   response.session_id() == 0U && response.reference_epoch() == 0U) {
          authorized_motion_epoch = response.motion_epoch();
          authorized_management_epoch = response.management_epoch();
          stop_idle_observed = false;
          stop_terminal = true;
          stop_terminal_at_us = transport_steady_now_us();
          stop_operation_id = response.operation_id();
          stop_query.reset();
          management_handle.reset();
        } else fail_stop("Stop QueryResult failed or malformed");
      }
      const auto stop_now = transport_steady_now_us();
      const bool fresh_released_idle = stop_terminal && stop_idle_observed &&
          stop_terminal_at_us != 0U && stop_operation_id != 0U &&
          status.boot_epoch() == stop_original.expected_boot_epoch() &&
          status.core_state() == 1U && status.session_id() == 0U &&
          status.reference_epoch() == 0U && status.release_committed() &&
          !status.real_motion_permitted() && status.permit_remaining_us() == 0U &&
          status.motion_epoch() == authorized_motion_epoch &&
          status.management_epoch() == authorized_management_epoch &&
          status.operation_id() == stop_operation_id &&
          status.transaction_phase() == 3U &&
          last_state_boot == status.boot_epoch() && last_state_session == 0U &&
          last_state_reference == 0U && last_state_motion == status.motion_epoch() &&
          last_state_management_epoch == status.management_epoch() &&
          last_observation_raw_idle &&
          last_received_us >= stop_terminal_at_us &&
          last_status_received_us >= stop_terminal_at_us &&
          stop_now >= last_received_us &&
          stop_now - last_received_us <= parameters.max_state_age_us &&
          stop_now >= last_status_received_us &&
          stop_now - last_status_received_us <= parameters.max_state_age_us;
      if (stop_confirmation_pending.load(std::memory_order_acquire) &&
          fresh_released_idle) {
        ++stop_requests_confirmed;
        stop_confirmation_pending = false;
        pending_stop.reset();
      } else if (stop_confirmation_pending.load(std::memory_order_acquire) &&
                 stop_now > stop_deadline_us) fail_stop("terminal QueryResult/Idle deadline expired");
      else if (stop_confirmation_pending.load(std::memory_order_acquire) && stop_received &&
               !stop_terminal && !stop_query && stop_now >= next_stop_query_us) {
        stop_query = make_stop_query(stop_original);
        if (!stop_query) fail_stop("Stop QueryResult construction failed");
        else {
          Management::Request query;
          if (wire::DummyManageV2RequestCodec::decode(stop_query->request.data(),
                  stop_query->request.size(), &query) != wire::CodecResult::kOk)
            fail_stop("Stop QueryResult decode failed");
          else {
            auto sent = management_client.async_request(query, management_reply,
                stop_query.get(), dp::comm::Duration::from_milliseconds(1000));
            if (sent.has_value()) {
              capture_sent_request(*stop_query);
              active_management = stop_query;
              management_handle.emplace(std::move(sent).take_value());
            } else {
              if(sent.result()==dp::comm::Result::kQueueFull) {
                stop_query_admission_retry.emplace(transport_steady_now_us());
                (void)stop_query_admission_retry->on_result(sent.result(),transport_steady_now_us(),false);
                log_management_handoff(*stop_query,"stop_query_would_block",sent.result());
              } else {
                log_management_handoff(*stop_query,"async_request_failed",sent.result());
                fail_stop("Stop QueryResult dispatch failed");
              }
            }
          }
        }
      }
      if (active_management && active_management != processed_management &&
          active_management->complete.load(std::memory_order_acquire)) {
        capture_completed_reply(active_management);
        observe_gate_release_reply(*active_management);
        accept_management_authority(*active_management);
      }
      if(spun!=dp::comm::Result::kOk && spun!=dp::comm::Result::kBusy) {
        fprintf(stderr,"device transport spin result=%u\n",static_cast<unsigned>(spun));
        return false;
      }
      if (!gate_release.idle_confirmed && confirmed_gate_released_idle(transport_steady_now_us()))
        gate_release.idle_confirmed = true;
      dp::comm::LinkEvent event;
      while (context.take_link_event(&event)) {
        if (event.state != dp::comm::LinkState::kReady) invalidate();
      }
      const auto now = transport_steady_now_us();
      const bool released_idle = released_idle_latched();
      const auto stop_causes = classify_transport_stop_causes(
          group_sent_ack, now, last_received_us, parameters.max_state_age_us);
      const auto heartbeat_action = parameters.heartbeat_stop_budget_us == 0U
          ? heartbeat.poll(now)
          : heartbeat.poll(now, parameters.heartbeat_stop_budget_us);
      const bool reobserve_raw_idle = !released_idle && recoverable_raw_idle_gap() &&
          (heartbeat_action == HeartbeatAction::kStop || stop_causes.state_stale);
      if (reobserve_raw_idle) {
        ++raw_idle_reobservations;
        invalidate();
      }
      if (!released_idle && heartbeat_action == HeartbeatAction::kWarn && !heartbeat_warned) {
        ++heartbeat_warnings;
        heartbeat_warned = true;
      }
      if (!released_idle && !reobserve_raw_idle && heartbeat_action == HeartbeatAction::kStop &&
          !heartbeat_stopped && !stop_latched) {
        fprintf(stderr, "device transport state heartbeat stop: gap_us=%llu period_us=%llu callbacks=%llu decoded=%llu rejected=%llu decode=%llu identity=%llu authority=%llu sequence=%llu clock=%llu age=%llu ingest=%llu stop_latched=%llu ack_misses=%llu last_callback_seq=%u last_callback_produced_us=%llu last_callback_received_us=%llu last_accepted_seq=%u last_accepted_produced_us=%llu last_accepted_received_us=%llu\n",
                static_cast<unsigned long long>(now - last_received_us),
                static_cast<unsigned long long>(parameters.state_publish_period_us),
                static_cast<unsigned long long>(state_callbacks.load()),
                static_cast<unsigned long long>(states.load()),
                static_cast<unsigned long long>(rejected_states.load()),
                static_cast<unsigned long long>(state_decode_rejections.load()),
                static_cast<unsigned long long>(state_identity_rejections.load()),
                static_cast<unsigned long long>(state_authority_rejections.load()),
                static_cast<unsigned long long>(state_sequence_rejections.load()),
                static_cast<unsigned long long>(clock_skew_rejections.load()),
                static_cast<unsigned long long>(stale_payload_rejections.load()),
                static_cast<unsigned long long>(state_ingest_rejections.load()),
                static_cast<unsigned long long>(stop_latched_state_rejections.load()),
                static_cast<unsigned long long>(group_sent_ack_misses.load()),
                last_callback_sequence,
                static_cast<unsigned long long>(last_callback_produced_us),
                static_cast<unsigned long long>(last_callback_received_us), last_sequence,
                static_cast<unsigned long long>(last_produced_us),
                static_cast<unsigned long long>(last_received_us));
        ++state_heartbeat_gap_stops;
        last_state_gap_us = now - last_received_us;
        ++heartbeat_stops;
        heartbeat_stopped = true;
        stop_latched = true;
        system.request_transport_stop();
      }
      if (!released_idle && !reobserve_raw_idle && stop_causes.ack_timed_out) {
        fprintf(stderr, "device transport group sent stop: unacked_seq=%u sent_mask=%u required_mask=%u\n",
                group_sent_ack.sequence(), status.can_sent_mask(), parameters.configured_mask);
        ++heartbeat_stops;
        ++group_sent_ack_timeouts;
        group_sent_ack.reset();
        stop_latched = true;
        system.request_transport_stop();
      }
      if (!released_idle && !reobserve_raw_idle && stop_causes.state_stale) {
        ++state_stale_stops;
        if (stop_causes.ack_unconfirmed_on_state_stale_stop) {
          ++ack_unconfirmed_on_state_stale_stop;
          fprintf(stderr, "device transport state stale stop: ACK unconfirmed before deadline; CAN SENT unknown\n");
        }
        if (!stop_latched) {
          stop_latched = true;
          system.request_transport_stop();
        }
        invalidate();
      }
      if (management->inactive_prime_allowed.load(std::memory_order_acquire) ||
          status.core_state() == kWireEnablingHold) {
        const bool transferred = system.transfer_control_update();
        if (status.core_state() == kWireEnablingHold &&
            ((transferred && enabling_transfer_log_count < 3U) || enabling_transfer_log_count == 0U)) {
          ++enabling_transfer_log_count;
          fprintf(stderr, "COMMAND_ENABLE_DIAG transfer=%d inactive=%d status_motion=%u authorized_motion=%u\n",
                  transferred, !system.commands_active(), status.motion_epoch(), authorized_motion_epoch);
        }
        if (transferred && !logged_prime_transfer) {
          logged_prime_transfer = true;
          fprintf(stderr, "COMMAND_PRIME_DIAG transfer inactive=%d core_state=%u boot=%llu status_motion=%u authorized_motion=%u\n",
                  !system.commands_active(), status.core_state(),
                  static_cast<unsigned long long>(boot), status.motion_epoch(), authorized_motion_epoch);
        }
        if (status.core_state() == kWireEnablingHold && !transferred) {
          const auto observed_at = transport_steady_now_us();
          if (observed_at - last_enable_visibility_log_us >= 1000000U) {
            last_enable_visibility_log_us = observed_at;
            const auto visible = consume_control_update(0U);
            fprintf(stderr, "COMMAND_ENABLE_DIAG visible=%d generation=%llu current=%d inactive_allowed=%d\n",
                    visible.has_value(),
                    static_cast<unsigned long long>(visible ? visible->generation : 0U),
                    visible && command_record_current(*visible),
                    management->inactive_prime_allowed.load(std::memory_order_acquire));
          }
        }
      }
      publish_pending(command_publisher);
      HoldDrainSnapshot drain{};
      if(status.core_state()==kWireHold && !stop_latched && !command_sent &&
         !hold_drain_rejected &&
         !command_stream.has_pending() && !command_stream.has_in_flight() &&
         !group_sent_ack.pending() && last_processed_hold_generation!=0U &&
         last_ack_seq!=0U &&
         last_processed_hold_identity.boot==last_ack_identity.boot &&
         last_processed_hold_identity.session==last_ack_identity.session &&
         last_processed_hold_identity.reference_epoch==last_ack_identity.reference_epoch &&
         last_processed_hold_identity.motion_epoch==last_ack_identity.motion_epoch) {
        drain={last_ack_identity.boot,last_ack_identity.session,
               last_processed_hold_generation,last_ack_identity.reference_epoch,
               last_ack_identity.motion_epoch,last_ack_seq};
        drain.disposition=last_processed_hold_disposition;
        drain.quiescent=true;
        drain.observed_us=observed_hold.received_us;
        drain.max_state_age_us=parameters.max_state_age_us;
        drain.position=last_processed_hold_position;
        for(std::size_t axis=0;axis<drain.tolerance.size();++axis)
          drain.tolerance[axis]=management->post_complete_hold_tolerance_rad[axis].load(
            std::memory_order_acquire);
      }
      { std::lock_guard<std::mutex> lock(management->mutex); management->hold_drain=drain; }
    }
    if(active_management && !active_management->complete) {
      log_management_handoff(*active_management,"worker_exited");
      active_management->failed=true;
    }
    if (stop_confirmation_pending.load(std::memory_order_acquire))
      fail_stop("transport ended before terminal QueryResult/Idle");
    return true;
  }

  void run() noexcept {
    while (running.load(std::memory_order_acquire)) {
      try { failed = !run_connected(); }
      catch (...) { failed = true; }
      if (stop_confirmation_pending.exchange(false)) {
        fprintf(stderr, "device transport UNCONFIRMED_STOP: worker exited before confirmation\n");
        ++unconfirmed_stop_requests;
      }
      ready = false;
      management->ready=false;
      management->inactive_prime_allowed=false;
      {
        std::lock_guard<std::mutex> lock(management->mutex);
        if(management->queued) {
          log_management_handoff(*management->queued,"worker_unavailable");
          management->queued->failed=true;
        }
        management->queued.reset(); management->observation={};
      }
      invalidate();
      if (system.take_stop_request()) {
        fprintf(stderr, "device transport UNCONFIRMED_STOP: no connected worker\n");
        ++stop_request_failures;
        ++unconfirmed_stop_requests;
      }
      if (running) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
};

DeviceTransport::DeviceTransport(GuardedZenbeddedSystem &system, TransportParameters parameters)
    : impl_(std::make_unique<Impl>(system, std::move(parameters))) {}
DeviceTransport::~DeviceTransport() { stop(); }
bool DeviceTransport::start() {
  if (impl_->running.exchange(true)) return true;
  if (impl_->worker.joinable()) impl_->worker.join();
  if (!impl_->parameters.raw_evidence_path.empty() &&
      !impl_->raw_evidence.start(impl_->parameters.raw_evidence_path)) {
    impl_->running = false;
    impl_->failed = true;
    return false;
  }
  try { impl_->worker = std::thread([this] { impl_->run(); }); }
  catch (...) { impl_->running = false; impl_->raw_evidence.stop(); return false; }
  return true;
}
void DeviceTransport::stop() {
  if (impl_->running.load(std::memory_order_acquire) &&
      (impl_->system.stop_request_pending() ||
       impl_->stop_confirmation_pending.load(std::memory_order_acquire))) {
    const auto timeout_us = impl_->parameters.stop_confirmation_timeout_us != 0U
        ? impl_->parameters.stop_confirmation_timeout_us : impl_->parameters.max_state_age_us;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeout_us);
    while (impl_->running.load(std::memory_order_acquire) &&
           (impl_->system.stop_request_pending() ||
            impl_->stop_confirmation_pending.load(std::memory_order_acquire)) &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  impl_->running = false;
  if (impl_->worker.joinable()) impl_->worker.join();
  impl_->raw_evidence.stop();
  if (impl_->provisional_evidence_binding) {
    const auto run = impl_->provisional_evidence_binding->run_id;
    const bool release_reconciled = impl_->provisional_release_chain &&
        impl_->raw_evidence.terminal_written_run_id() == run &&
        impl_->last_terminal_chain_run == run &&
        *impl_->provisional_release_chain == impl_->last_terminal_chain;
    if (!release_reconciled) impl_->raw_evidence.reject_invalid();
  }
  if (impl_->raw_evidence.failure() != RawEvidenceCollector::Failure::kNone) impl_->failed = true;
}
TransportStatistics DeviceTransport::statistics() const noexcept {
  TransportStatistics result{};
  result.state_callbacks = impl_->state_callbacks.load();
  result.state_messages = impl_->states.load();
  result.status_messages = impl_->statuses.load();
  result.stale_run_status_rejections = impl_->stale_run_status_rejections.load();
  result.rejected_states = impl_->rejected_states.load();
  result.state_decode_rejections = impl_->state_decode_rejections.load();
  result.state_identity_rejections = impl_->state_identity_rejections.load();
  result.state_authority_rejections = impl_->state_authority_rejections.load();
  result.state_sequence_rejections = impl_->state_sequence_rejections.load();
  result.state_ingest_rejections = impl_->state_ingest_rejections.load();
  result.stop_latched_state_rejections = impl_->stop_latched_state_rejections.load();
  result.commands_published = impl_->commands.load();
  result.rejected_commands = impl_->rejected_commands.load();
  result.invalidations = impl_->invalidations.load();
  result.stop_requests_sent = impl_->stop_requests_sent.load();
  result.stop_request_failures = impl_->stop_request_failures.load();
  result.stop_requests_confirmed = impl_->stop_requests_confirmed.load();
  result.unconfirmed_stop_requests = impl_->unconfirmed_stop_requests.load();
  result.heartbeat_warnings = impl_->heartbeat_warnings.load();
  result.heartbeat_stops = impl_->heartbeat_stops.load();
  result.group_sent_ack_timeouts = impl_->group_sent_ack_timeouts.load();
  result.group_sent_ack_misses = impl_->group_sent_ack_misses.load();
  result.state_stale_stops = impl_->state_stale_stops.load();
  result.raw_idle_reobservations = impl_->raw_idle_reobservations.load();
  result.ack_unconfirmed_on_state_stale_stop =
      impl_->ack_unconfirmed_on_state_stale_stop.load();
  result.bootstrap_rejections = impl_->bootstrap_rejections.load();
  result.stale_payload_rejections = impl_->stale_payload_rejections.load();
  result.clock_skew_rejections = impl_->clock_skew_rejections.load();
  result.state_heartbeat_gap_stops = impl_->state_heartbeat_gap_stops.load();
  result.last_state_gap_us = impl_->last_state_gap_us.load();
  result.raw_evidence_written = impl_->raw_evidence.written();
  result.raw_evidence_pages_written = impl_->raw_evidence.pages_written();
  result.raw_evidence_last_terminal_written_run =
      impl_->raw_evidence.terminal_written_run_id();
  result.raw_evidence_failure = static_cast<std::uint8_t>(impl_->raw_evidence.failure());
  result.ready = impl_->ready.load() &&
      impl_->raw_evidence.failure() == RawEvidenceCollector::Failure::kNone;
  result.initialization_failed = impl_->failed.load() ||
      impl_->raw_evidence.failure() != RawEvidenceCollector::Failure::kNone;
  return result;
}
std::shared_ptr<DeviceManagementChannel> DeviceTransport::management_channel(const TransportParameters &parameters) noexcept {
  std::unique_lock<std::mutex> lock(channel_registry_mutex,std::try_to_lock);
  if(!lock) return {};
  const auto channel=channel_registry.lock();
  if(!channel) return {};
  const auto &actual=channel->parameters;
  if(actual.endpoint!=parameters.endpoint || actual.device_id!=parameters.device_id ||
     actual.client_instance_id!=parameters.client_instance_id || actual.profile_id!=parameters.profile_id ||
     actual.configured_mask!=parameters.configured_mask) return {};
  return channel;
}
}  // namespace dummy_ros_control
