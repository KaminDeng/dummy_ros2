#include "dummy_ros_control/device_transport.hpp"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cstring>
#include <sstream>
#include <thread>

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_management_channel.hpp"

namespace dummy_ros_control {
namespace {
namespace wire = dp::protocol::dummy_robot_v2;
using State = wire::DummyJointStateV2;
using Status = wire::DummyDeviceStatusV2;
using Command = wire::DummyJointCommandV2;
using Management = dp::protocol::services::DummyManageV2RawFixed;
constexpr std::uint16_t kWireEnablingHold = 3U;
constexpr std::uint16_t kWireDisableAndRelease = 10U;
std::mutex channel_registry_mutex;
std::weak_ptr<DeviceManagementChannel> channel_registry;

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
  *output = std::move(parsed);
  return true;
}

struct DeviceTransport::Impl {
  GuardedZenbeddedSystem &system;
  const TransportParameters parameters;
  const std::shared_ptr<DeviceManagementChannel> management;
  std::atomic<bool> running{false}, ready{false}, failed{false};
  std::thread worker;
  std::atomic<std::uint64_t> states{0}, statuses{0}, rejected_states{0}, commands{0};
  std::atomic<std::uint64_t> rejected_commands{0}, invalidations{0};
  std::atomic<std::uint64_t> stop_requests_sent{0}, stop_request_failures{0};
  std::atomic<std::uint64_t> heartbeat_warnings{0}, heartbeat_stops{0}, bootstrap_rejections{0};
  Status status{};
  std::uint64_t boot = 0U, anchor_us = 0U, last_received_us = 0U;
  std::uint32_t last_sequence = 0U;
  std::uint64_t last_freshness_ticket = 0U;
  std::uint64_t next_stop_request_id = 0x8000000000000001ULL;
  std::uint32_t next_stop_request_seq = 0x80000001U;
  bool has_sequence = false;
  LatestOnlyCommandStream command_stream;
  TransportHeartbeat heartbeat;
  std::uint64_t group_sent_deadline_us = 0U;
  bool bootstrap_ready = false;
  bool heartbeat_warned = false;
  bool heartbeat_stopped = false;
  bool command_sent = false;
  std::array<std::uint64_t, 16> retired_boots{};
  std::size_t retired_count = 0U;

  Impl(GuardedZenbeddedSystem &target, TransportParameters config)
      : system(target), parameters(std::move(config)),
        management(std::make_shared<DeviceManagementChannel>(parameters)),
        heartbeat(parameters.state_publish_period_us) {
    std::lock_guard<std::mutex> lock(channel_registry_mutex);
    if(channel_registry.expired()) channel_registry=management;
  }

  void invalidate(const Status *retired_authority = nullptr) noexcept {
    CommandIdentity identity{};
    if (retired_authority != nullptr) {
      identity = CommandIdentity{
          retired_authority->boot_epoch(), retired_authority->session_id(),
          retired_authority->reference_epoch(), retired_authority->motion_epoch(), 0U};
    }
    if (retired_authority != nullptr) system.invalidate_authority_state(identity);
    else system.invalidate_transport_state();
    bootstrap_ready = false;
    group_sent_deadline_us = 0U;
    command_sent = false;
    command_stream.close_session();
    CommandGenerationRecord record{};
    while (system.take_publish_record(&record)) ++rejected_commands;
    boot = 0U;
    has_sequence = false;
    anchor_us = transport_steady_now_us();
    last_received_us = 0U;
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
    return message.schema_major() == 2U && message.schema_minor() >= 1U && matches_common(message);
  }

  bool matches_state(const State &message) const noexcept {
    return message.schema_major() == 2U && message.schema_minor() == 0U && matches_common(message);
  }

  template <typename Message>
  bool matches_authority(const Message &message) const noexcept {
    return message.session_id() == status.session_id() &&
           message.reference_epoch() == status.reference_epoch() &&
           message.motion_epoch() == status.motion_epoch();
  }

  static void receive_status(dp::comm::BorrowedMessage<Status> borrowed, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    Status value;
    if (borrowed.copy_to(&value) != dp::comm::Result::kOk) return;
    ++self.statuses;
    if (!self.matches_status(value)) return;
    if (std::find(self.retired_boots.begin(), self.retired_boots.end(), value.boot_epoch()) !=
        self.retired_boots.end()) return;
    if (self.boot == value.boot_epoch() && self.status.boot_epoch() == value.boot_epoch()) {
      if (!sequence_at_or_after(value.management_epoch(), self.status.management_epoch())) return;
      if (value.management_epoch() == self.status.management_epoch()) {
        const bool same_owner_reference = value.session_id() == self.status.session_id() &&
            value.reference_epoch() == self.status.reference_epoch();
        if (!same_owner_reference ||
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
      self.invalidate();
    }
    if (self.boot != 0U && !self.matches_authority(value)) self.invalidate(&self.status);
    self.boot = value.boot_epoch();
    self.status = value;
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
    if (!self.bootstrap_ready) {
      ++self.bootstrap_rejections;
      self.system.invalidate_transport_state();
    }
    if (self.command_stream.acknowledge_committed(value.committed_producer_seq())) {
      self.group_sent_deadline_us = 0U;
      self.command_sent = false;
    }
    {
      std::lock_guard<std::mutex> lock(self.management->mutex);
      auto &observation=self.management->observation;
      if(wire::DummyDeviceStatusV2Codec::encode(value,observation.status.data(),observation.status.size())==wire::CodecResult::kOk)
        observation.status_received_us=transport_steady_now_us();
    }
  }

  static void receive_state(dp::comm::BorrowedMessage<State> borrowed, void *context) noexcept {
    auto &self = *static_cast<Impl *>(context);
    State value;
    if (borrowed.copy_to(&value) != dp::comm::Result::kOk) return;
    ++self.states;
    const auto now = transport_steady_now_us();
    const bool current_transport_state = self.matches_state(value) && self.boot != 0U &&
        value.boot_epoch() == self.boot && self.matches_authority(value);
    if (current_transport_state) {
      self.heartbeat.observe_state(now);
      self.heartbeat_warned = false;
      self.heartbeat_stopped = false;
    }
    if (!self.bootstrap_ready || !self.matches_state(value) || self.boot == 0U || value.boot_epoch() != self.boot ||
        !self.matches_authority(value)) {
      ++self.rejected_states;
      return;
    }
    self.last_freshness_ticket = value.freshness_ticket();
    const StateExpectation expected{self.boot, self.parameters.profile_id, self.last_sequence,
        self.has_sequence, self.parameters.configured_mask, self.anchor_us, now,
        self.parameters.max_state_age_us};
    std::uint8_t bytes[wire::DummyJointStateV2Codec::kEncodedSize]{};
    if (wire::DummyJointStateV2Codec::encode(value, bytes, sizeof(bytes)) != wire::CodecResult::kOk) {
      ++self.rejected_states;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(self.management->mutex);
      std::copy(std::begin(bytes),std::end(bytes),self.management->observation.state.begin());
      self.management->observation.state_received_us=now;
    }
    if (!self.system.ingest_wire_state(bytes, sizeof(bytes), expected)) {
      ++self.rejected_states;
      if ((value.position_valid_mask() & self.parameters.configured_mask) != self.parameters.configured_mask)
        self.system.invalidate_transport_state();
      return;
    }
    self.system.set_state_deadline(saturating_state_deadline(value.produced_at_us(), self.parameters.max_state_age_us));
    self.last_sequence = value.state_seq();
    self.has_sequence = true;
    self.last_received_us = now;
  }

  static void management_reply(dp::comm::Result result,const Management::Response *response,
      const dp::comm::ResponseInfo &,void *context) {
    auto &request=*static_cast<DeviceManagementRequest *>(context);
    if(result!=dp::comm::Result::kOk || !response ||
       wire::DummyManageV2ResponseCodec::encode(*response,request.response.data(),request.response.size())!=wire::CodecResult::kOk) {
      request.failed=true; return;
    }
    request.complete.store(true,std::memory_order_release);
  }

  std::shared_ptr<DeviceManagementRequest> make_stop_request() noexcept {
    if (status.boot_epoch() == 0U || status.profile_id() != parameters.profile_id) return {};
    Management::Request request;
    request.set_schema_major(2U);
    request.set_schema_minor(0U);
    std::uint8_t device[16]{}, client[16]{};
    std::copy(parameters.device_id.begin(), parameters.device_id.end(), device);
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), client);
    request.set_device_id(device);
    request.set_client_instance_id(client);
    request.set_request_id(next_stop_request_id++);
    request.set_request_seq(next_stop_request_seq++);
    request.set_operation(kWireDisableAndRelease);
    request.set_expected_boot_epoch(status.boot_epoch());
    request.set_expected_session_id(status.session_id());
    request.set_expected_reference_epoch(status.reference_epoch());
    request.set_expected_motion_epoch(status.motion_epoch());
    request.set_expected_management_epoch(status.management_epoch());
    request.set_expected_profile_id(parameters.profile_id);
    request.set_freshness_ticket(last_freshness_ticket);
    request.set_configured_mask(parameters.configured_mask);
    auto pending = std::make_shared<DeviceManagementRequest>();
    if (wire::DummyManageV2RequestCodec::encode(
            request, pending->request.data(), pending->request.size()) != wire::CodecResult::kOk) return {};
    return pending;
  }

  void publish_pending(dp::comm::Publisher<Command> &publisher) noexcept {
    CommandGenerationRecord next{};
    while (system.take_publish_record(&next)) command_stream.offer(next);
    const auto candidate = command_sent ? std::optional<CommandGenerationRecord>{}
                                        : (command_stream.has_in_flight()
                                               ? command_stream.in_flight_record()
                                               : command_stream.begin_send());
    if (!candidate.has_value()) return;
    const auto &record = *candidate;
    JointStateSnapshot snapshot{};
    const bool inactive_prime = management->inactive_prime_allowed.load(std::memory_order_acquire) ||
        status.core_state() == kWireEnablingHold;
    if (wire_producer_sequence(record.generation) == 0U || !command_record_current(record) ||
        (!system.commands_active() && !inactive_prime) || boot == 0U ||
        record.identity.boot != boot || record.identity.session != status.session_id() ||
        record.identity.reference_epoch != status.reference_epoch() ||
        record.identity.motion_epoch != status.motion_epoch() || record.identity.ticket == 0U ||
        !bootstrap_ready) {
      ++rejected_commands;
      command_stream.discard_in_flight();
      command_sent = false;
      return;
    }
    // On a busy state read, keep the original unsent generation. Authority and
    // freshness are checked again before every retry; no new sample is made.
    if(!system.copy_state(&snapshot)) return;
    Command message;
    message.set_schema_major(2U);
    message.set_schema_minor(0U);
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
    message.set_freshness_ticket(record.identity.ticket);
    message.set_configured_mask(parameters.configured_mask);
    double position[6]{};
    for (std::size_t axis = 0U; axis < record.position.size(); ++axis)
      position[axis] = (parameters.configured_mask & (1U << axis)) != 0U
                           ? record.position[axis]
                           : 0.0;
    message.set_position_rad(position);
    const auto published=publisher.publish(message);
    if(published==dp::comm::Result::kBusy || published==dp::comm::Result::kQueueFull) return;
    if(published==dp::comm::Result::kOk) {
      ++commands;
      command_sent = true;
      group_sent_deadline_us = transport_steady_now_us() + parameters.state_publish_period_us;
    } else {
      ++rejected_commands;
      command_stream.discard_in_flight();
      command_sent = false;
    }
  }

  bool run_connected() {
    const dp::comm::StaticStorageConfig storage_config{1U, 1U, 2U, 0U, 8U, 301U, 96U, 0U, 1U, 1U, 1U};
    dp::comm::StaticStorage storage(storage_config);
    const dp::comm::TransportConfig transport{"client", parameters.endpoint.c_str(), "", "dummy_ros_control"};
    auto initialized = dp::comm::Context::init(storage,
        dp::comm::InitOptions{dp::comm::MemoryMode::kStaticOnly, &transport, nullptr});
    if (!initialized.has_value()) return false;
    auto context = std::move(initialized).take_value();
    dp::comm::NodeOptions options{};
    options.product_name = "dummy_robot";
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), options.device_id.bytes);
    options.node_name = "ros_control";
    auto created = context.create_node(options);
    if (!created.has_value()) return false;
    auto node = std::move(created).take_value();
    static constexpr char hex[] = "0123456789abcdef";
    std::string route = "/dpsdk/dummy_robot/";
    for (auto byte : parameters.device_id) { route += hex[byte >> 4U]; route += hex[byte & 15U]; }
    route += "/ref_control/";
    auto state_result = node.create_subscription<State>((route + "state_v2").c_str(), {}, receive_state, this);
    auto status_result = node.create_subscription<Status>((route + "status_v2").c_str(), {}, receive_status, this);
    auto command_result = node.create_publisher<Command>((route + "command_v2").c_str(), {});
    auto management_result=node.create_client<Management>((route+"manage_v2").c_str());
    if (!state_result.has_value() || !status_result.has_value() || !command_result.has_value() || !management_result.has_value()) return false;
    auto state_subscription = std::move(state_result).take_value();
    auto status_subscription = std::move(status_result).take_value();
    auto command_publisher = std::move(command_result).take_value();
    auto management_client=std::move(management_result).take_value();
    dp::comm::StaticExecutor executor;
    if (executor.add_node(node) != dp::comm::Result::kOk || node.activate() != dp::comm::Result::kOk) return false;
    anchor_us = transport_steady_now_us();
    ready = true;
    management->ready=true;
    std::shared_ptr<DeviceManagementRequest> active_management;
    std::optional<dp::comm::RequestHandle<Management>> management_handle;
    while (running.load(std::memory_order_acquire)) {
      if(active_management && active_management->canceled) { management_handle.reset(); active_management.reset(); }
      std::shared_ptr<DeviceManagementRequest> queued;
      { std::lock_guard<std::mutex> lock(management->mutex); queued=std::move(management->queued); }
      if(queued) {
        management_handle.reset(); active_management=std::move(queued);
        Management::Request decoded;
        if(wire::DummyManageV2RequestCodec::decode(active_management->request.data(),active_management->request.size(),&decoded)!=wire::CodecResult::kOk)
          active_management->failed=true;
        else {
          auto sent=management_client.async_request(decoded,management_reply,active_management.get(),dp::comm::Duration::from_milliseconds(1000));
          if(sent.has_value()) management_handle.emplace(std::move(sent).take_value());
          else active_management->failed=true;
        }
      }
      if (system.take_stop_request()) {
        if (active_management && !active_management->complete.load(std::memory_order_acquire))
          active_management->failed.store(true, std::memory_order_release);
        management_handle.reset();
        active_management = make_stop_request();
        if (!active_management) {
          ++stop_request_failures;
        } else {
          Management::Request decoded;
          if (wire::DummyManageV2RequestCodec::decode(active_management->request.data(),
                active_management->request.size(), &decoded) != wire::CodecResult::kOk) {
            active_management->failed = true;
            ++stop_request_failures;
          } else {
            auto sent = management_client.async_request(
                decoded, management_reply, active_management.get(), dp::comm::Duration::from_milliseconds(1000));
            if (sent.has_value()) {
              management_handle.emplace(std::move(sent).take_value());
              ++stop_requests_sent;
            } else {
              active_management->failed = true;
              ++stop_request_failures;
            }
          }
        }
        invalidate();
      }
      const auto spun=executor.spin_some(dp::comm::Duration::from_milliseconds(2U));
      if(spun!=dp::comm::Result::kOk && spun!=dp::comm::Result::kBusy) {
        fprintf(stderr,"device transport spin result=%u\n",static_cast<unsigned>(spun));
        return false;
      }
      dp::comm::LinkEvent event;
      while (context.take_link_event(&event)) {
        if (event.state != dp::comm::LinkState::kReady) invalidate();
      }
      const auto now = transport_steady_now_us();
      if (last_received_us != 0U && now - last_received_us > parameters.max_state_age_us) {
        invalidate();
      }
      const auto heartbeat_action = heartbeat.poll(now);
      if (heartbeat_action == HeartbeatAction::kWarn && !heartbeat_warned) {
        ++heartbeat_warnings;
        heartbeat_warned = true;
      }
      if (heartbeat_action == HeartbeatAction::kStop && !heartbeat_stopped) {
        ++heartbeat_stops;
        heartbeat_stopped = true;
        system.request_transport_stop();
      }
      if (group_sent_deadline_us != 0U && now > group_sent_deadline_us &&
          (status.can_sent_mask() & parameters.configured_mask) != parameters.configured_mask) {
        ++heartbeat_stops;
        group_sent_deadline_us = 0U;
        system.request_transport_stop();
      }
      if (management->inactive_prime_allowed.load(std::memory_order_acquire) ||
          status.core_state() == kWireEnablingHold)
        system.transfer_control_update();
      publish_pending(command_publisher);
    }
    if(active_management && !active_management->complete) active_management->failed=true;
    return true;
  }

  void run() noexcept {
    while (running.load(std::memory_order_acquire)) {
      try { failed = !run_connected(); }
      catch (...) { failed = true; }
      ready = false;
      management->ready=false;
      management->inactive_prime_allowed=false;
      {
        std::lock_guard<std::mutex> lock(management->mutex);
        if(management->queued) management->queued->failed=true;
        management->queued.reset(); management->observation={};
      }
      invalidate();
      if (system.take_stop_request()) ++stop_request_failures;
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
  try { impl_->worker = std::thread([this] { impl_->run(); }); }
  catch (...) { impl_->running = false; return false; }
  return true;
}
void DeviceTransport::stop() {
  if (impl_->running.load(std::memory_order_acquire) && impl_->system.stop_request_pending()) {
    const auto completed_before = impl_->stop_requests_sent.load(std::memory_order_acquire) +
        impl_->stop_request_failures.load(std::memory_order_acquire);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (impl_->stop_requests_sent.load(std::memory_order_acquire) +
               impl_->stop_request_failures.load(std::memory_order_acquire) == completed_before &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  impl_->running = false;
  if (impl_->worker.joinable()) impl_->worker.join();
}
TransportStatistics DeviceTransport::statistics() const noexcept {
  return {impl_->states.load(), impl_->statuses.load(), impl_->rejected_states.load(), impl_->commands.load(),
          impl_->rejected_commands.load(), impl_->invalidations.load(), impl_->stop_requests_sent.load(),
          impl_->stop_request_failures.load(), impl_->heartbeat_warnings.load(), impl_->heartbeat_stops.load(),
          impl_->bootstrap_rejections.load(),
          impl_->ready.load(), impl_->failed.load()};
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
