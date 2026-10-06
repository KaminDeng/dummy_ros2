#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <openssl/sha.h>

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

namespace {
namespace wire = dp::protocol::dummy_robot_v2;
using State = wire::DummyJointStateV2;
using Status = wire::DummyDeviceStatusV2;
using Evidence = wire::DummyAttemptEvidenceV2;
using Command = wire::DummyJointCommandV2;
using Management = dp::protocol::services::DummyManageV2RawFixed;
using namespace dummy_ros_control;
int test_released_idle_startup_baseline() {
  HistoricalIdleBaselineContext context{};
  Status status;
  status.set_boot_epoch(7U);
  status.set_run_id(212U);
  status.set_parent_run_id(211U);
  status.set_attempt_kind(3U);
  status.set_final_state(8U);
  status.set_core_state(1U);
  status.set_session_id(0U);
  status.set_reference_epoch(0U);
  status.set_release_committed(true);
  status.set_real_motion_permitted(false);
  status.set_permit_remaining_us(0U);
  status.set_evidence_lost(false);
  if (!historical_released_idle_baseline(status, context)) return 301;
  auto prior_motion = status;
  prior_motion.set_attempt_kind(1U);
  prior_motion.set_parent_run_id(0U);
  prior_motion.set_final_state(2U);
  if (!historical_released_idle_baseline(prior_motion, context)) return 302;
  auto rejected = status;
  rejected.set_final_state(2U);
  if (historical_released_idle_baseline(rejected, context)) return 303;
  rejected = status;
  rejected.set_parent_run_id(0U);
  if (historical_released_idle_baseline(rejected, context)) return 304;
  rejected.set_parent_run_id(212U);
  if (historical_released_idle_baseline(rejected, context)) return 305;
  rejected = status;
  std::uint8_t nonzero_chain[32]{};
  nonzero_chain[0] = 1U;
  rejected.set_evidence_chain_sha256(nonzero_chain);
  if (!historical_released_idle_baseline(rejected, context)) return 306;
  context.pages_written = 1U;
  if (historical_released_idle_baseline(rejected, context)) return 318;
  context.pages_written = 0U;
  rejected = status;
  rejected.set_boot_epoch(0U);
  if (historical_released_idle_baseline(rejected, context)) return 307;
  rejected = status;
  rejected.set_session_id(8U);
  if (historical_released_idle_baseline(rejected, context)) return 308;
  rejected = status;
  rejected.set_permit_remaining_us(1U);
  if (historical_released_idle_baseline(rejected, context)) return 309;
  rejected = status;
  rejected.set_evidence_lost(true);
  if (historical_released_idle_baseline(rejected, context)) return 310;
  rejected = status;
  rejected.set_core_state(4U);
  if (historical_released_idle_baseline(rejected, context)) return 311;
  context.current_boot = 7U;
  if (historical_released_idle_baseline(status, context)) return 312;
  context.current_boot = 0U;
  context.pages_written = 1U;
  if (historical_released_idle_baseline(status, context)) return 313;
  context.pages_written = 0U;
  context.historical_run_boot = 7U;
  context.historical_run_id = 212U;
  if (!historical_released_idle_baseline(status, context)) return 314;
  rejected = status;
  rejected.set_evidence_chain_sha256(nonzero_chain);
  context.historical_chain[0] = 1U;
  if (!historical_released_idle_baseline(rejected, context)) return 319;
  if (historical_released_idle_baseline(status, context)) return 320;
  context.pages_written = 1U;
  if (historical_released_idle_baseline(rejected, context)) return 321;
  context.pages_written = 0U;
  context.terminal_written_run_id = 211U;
  if (historical_released_idle_baseline(rejected, context)) return 322;
  context.terminal_written_run_id = 0U;
  context.historical_chain[0] = 0U;
  context.historical_run_boot = 8U;
  if (historical_released_idle_baseline(status, context)) return 315;
  context.historical_run_boot = 0U;
  context.historical_run_id = 0U;
  context.terminal_written_run_id = 211U;
  if (historical_released_idle_baseline(status, context)) return 316;
  context.terminal_written_run_id = 0U;
  context.historical_run_boot = 8U;
  if (historical_released_idle_baseline(status, context)) return 317;
  return 0;
}
bool test_group_sent_ack() {
  CommandGenerationRecord record{};
  record.generation = 42U;
  record.identity = {7U, 8U, 9U, 10U, 11U};
  GroupSentAck first;
  first.begin(record, 1000U, 100000U);
  if (!first.pending() || first.overdue(11000U) || first.overdue(101000U)) return false;
  // A previous group's full mask does not prove this sequence was committed.
  if (first.observe(7U, 8U, 9U, 10U, 41U, 0x3fU, 11000U)) return false;
  if (first.observe(7U, 8U, 9U, 10U, 42U, 0x3fU, 101001U)) return false;
  if (!first.overdue(101001U)) return false;

  GroupSentAck second;
  second.begin(record, 1000U, 100000U);
  if (second.observe(6U, 8U, 9U, 10U, 42U, 0x3fU, 11000U) ||
      second.observe(7U, 6U, 9U, 10U, 42U, 0x3fU, 11000U) ||
      second.observe(7U, 8U, 6U, 10U, 42U, 0x3fU, 11000U) ||
      second.observe(7U, 8U, 9U, 6U, 42U, 0x3fU, 11000U) ||
      second.observe(7U, 8U, 9U, 10U, 43U, 0x3fU, 11000U)) return false;
  // Status can be suppressed or retain an old/empty mask; State's exact
  // committed sequence is sufficient evidence of the complete CAN group.
  if (!second.observe(7U, 8U, 9U, 10U, 42U, 0U, 11000U) || second.pending() ||
      second.overdue(200000U)) return false;

  GroupSentAck rejected;
  rejected.begin(record, 1000U, 100000U, 3U, 41U);
  if (rejected.command_rejected(7U, 8U, 9U, 10U, 41U, 3U, 11000U) ||
      rejected.command_rejected(6U, 8U, 9U, 10U, 41U, 4U, 11000U) ||
      rejected.command_rejected(7U, 6U, 9U, 10U, 41U, 4U, 11000U) ||
      rejected.command_rejected(7U, 8U, 6U, 10U, 41U, 4U, 11000U) ||
      rejected.command_rejected(7U, 8U, 9U, 6U, 41U, 4U, 11000U) ||
      rejected.command_rejected(7U, 8U, 9U, 10U, 41U, 2U, 11000U) ||
      rejected.command_rejected(7U, 8U, 9U, 10U, 41U, 4U, 101001U) ||
      rejected.command_rejected(7U, 8U, 9U, 10U, 42U, 4U, 11000U) ||
      !rejected.command_rejected(7U, 8U, 9U, 10U, 41U, 4U, 11000U) ||
      !rejected.pending()) return false;

  GroupSentAck default_budget;
  default_budget.begin(record, 1010U, 100000U);
  const auto state_stale_first = classify_transport_stop_causes(
      default_budget, 101001U, 1000U, 100000U);
  if (!state_stale_first.state_stale || state_stale_first.ack_timed_out ||
      !state_stale_first.ack_unconfirmed_on_state_stale_stop) return false;

  GroupSentAck simultaneous;
  simultaneous.begin(record, 1000U, 100000U);
  const auto both_overdue = classify_transport_stop_causes(
      simultaneous, 101001U, 1000U, 100000U);
  if (!both_overdue.state_stale || !both_overdue.ack_timed_out ||
      both_overdue.ack_unconfirmed_on_state_stale_stop) return false;
  return true;
}
bool test_hardware_state_age_heartbeat() {
  TransportHeartbeat legacy(10000U);
  legacy.observe_state(100U);
  if (legacy.poll(50099U) == HeartbeatAction::kStop ||
      legacy.poll(50100U) != HeartbeatAction::kStop) return false;
  TransportHeartbeat heartbeat(20000U);
  heartbeat.observe_state(1000U);
  if (heartbeat.poll(40999U, 250000U) != HeartbeatAction::kNone ||
      heartbeat.poll(41000U, 250000U) != HeartbeatAction::kWarn ||
      heartbeat.poll(102758U, 250000U) != HeartbeatAction::kWarn ||
      heartbeat.poll(251000U, 250000U) != HeartbeatAction::kWarn ||
      heartbeat.poll(251001U, 250000U) != HeartbeatAction::kStop) return false;
  heartbeat.reset();
  return heartbeat.poll(300000U, 250000U) == HeartbeatAction::kNone;
}
bool test_heartbeat_budget_parameter() {
  const std::string hash(64U, '1');
  std::unordered_map<std::string, std::string> values{
      {"endpoint", "serial/test"},
      {"device_id", "01000000000000000000000000000000"},
      {"client_instance_id", "02000000000000000000000000000000"},
      {"profile_id", "1"}, {"configured_mask", "63"},
      {"contract_sha256", hash}, {"model_sha256", hash},
      {"profile_sha256", hash}, {"firmware_sha256", hash},
      {"compiled_policy_sha256", hash},
      {"position_min_rad", "-1,-1,-1,-1,-1,-1"},
      {"position_max_rad", "1,1,1,1,1,1"},
      {"velocity_max_rad_s", "1,1,1,1,1,1"},
      {"acceleration_max_rad_s", "1,1,1,1,1,1"},
      {"control_period_us", "10000"},
      {"state_publish_period_us", "20000"},
      {"max_state_age_us", "250000"}};
  TransportParameters parsed{};
  if (!parse_transport_parameters(values, 6U, &parsed) ||
      parsed.heartbeat_stop_budget_us != 0U) return false;
  values["heartbeat_stop_budget_us"] = "250000";
  if (!parse_transport_parameters(values, 6U, &parsed) ||
      parsed.heartbeat_stop_budget_us != 250000U) return false;
  values["heartbeat_stop_budget_us"] = "250001";
  if (parse_transport_parameters(values, 6U, &parsed)) return false;
  values["heartbeat_stop_budget_us"] = "39999";
  return !parse_transport_parameters(values, 6U, &parsed);
}
struct Stimulus { Status status{}; State state{}; Evidence evidence{}; bool done = false; bool send_status = true; bool send_state = true; bool send_evidence = false; };
struct Peer {
  int commands_fd;
  int stops_fd;
  std::uint64_t stop_request_id = 0U;
  std::uint32_t stop_motion_epoch = 0U;
  std::uint32_t stop_management_epoch = 0U;
  unsigned stop_queries = 0U;
  bool never_complete_stop = false;
  bool revoke_owner_before_stop_receipt = false;
  bool direct_gate_release = false;
  std::uint64_t gate_release_request_id = 0U;
};
class SerialBridge {
 public:
  bool open_pair() {
    for (int index = 0; index < 2; ++index) {
      masters_[index] = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (masters_[index] < 0 || ::grantpt(masters_[index]) != 0 ||
          ::unlockpt(masters_[index]) != 0) return false;
      char name[128]{};
      if (::ptsname_r(masters_[index], name, sizeof(name)) != 0) return false;
      endpoints_[index] = std::string("serial/") + name + "#baudrate=115200";
    }
    return true;
  }
  const std::string &endpoint(int index) const { return endpoints_[index]; }
  void close_child_fds() const { for (int fd : masters_) ::close(fd); }
  void start() {
    running_ = true;
    worker_ = std::thread([this] {
      while (running_) {
        pollfd fds[2]{{masters_[0], POLLIN, 0}, {masters_[1], POLLIN, 0}};
        if (::poll(fds, 2, 20) <= 0) continue;
        if (fds[0].revents & POLLIN) forward(0, 1);
        if (fds[1].revents & POLLIN) forward(1, 0);
      }
    });
  }
  ~SerialBridge() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    for (int fd : masters_) if (fd >= 0) ::close(fd);
  }
 private:
  void forward(int source, int target) {
    std::uint8_t bytes[512];
    const auto count = ::read(masters_[source], bytes, sizeof(bytes));
    if (count <= 0) return;
    for (ssize_t offset = 0; running_ && offset < count;) {
      const auto written = ::write(masters_[target], bytes + offset,
                                   static_cast<std::size_t>(count - offset));
      if (written > 0) offset += written;
      else if (written < 0 && errno == EINTR) continue;
      else if (written < 0 && errno == EAGAIN) {
        pollfd writable{masters_[target], POLLOUT, 0};
        ::poll(&writable, 1, 20);
      } else return;
    }
  }
  int masters_[2]{-1, -1};
  std::string endpoints_[2];
  std::atomic<bool> running_{false};
  std::thread worker_;
};
void receive_command(dp::comm::BorrowedMessage<Command> borrowed, void *context) {
  Command message;
  if (borrowed.copy_to(&message) == dp::comm::Result::kOk)
    (void)::write(static_cast<Peer *>(context)->commands_fd, &message, sizeof(message));
}
dp::comm::Result receive_management(const Management::Request &request,
                                    Management::Response *response,
                                    const dp::comm::RequestInfo &, void *context) {
  auto &peer = *static_cast<Peer *>(context);
  if (request.operation() == 10U &&
      ::write(peer.stops_fd, &request, sizeof(request)) != sizeof(request))
    return dp::comm::Result::kInvalidArgument;
  if (request.operation() == 8U &&
      ::write(peer.stops_fd, &request, sizeof(request)) != sizeof(request))
    return dp::comm::Result::kInvalidArgument;
  const bool stop_query = request.operation() == 12U && peer.stop_request_id != 0U &&
      request.queried_operation_id() == peer.stop_request_id;
  if (stop_query && ::write(peer.stops_fd, &request, sizeof(request)) != sizeof(request))
    return dp::comm::Result::kInvalidArgument;
  if (request.operation() != 6U && request.operation() != 8U && request.operation() != 0U &&
      request.operation() != 12U && request.operation() != 10U)
    return dp::comm::Result::kInvalidArgument;
  response->set_schema_major(2U);
  response->set_schema_minor(4U);
  response->set_device_id(request.device_id());
  response->set_client_instance_id(request.client_instance_id());
  response->set_request_id(request.request_id());
  response->set_receipt(1U);
  response->set_result_code(0U);
  response->set_operation_id(request.request_id() == 45U ? 777U : 42U);
  response->set_transaction_phase(3U);
  response->set_boot_epoch(request.expected_boot_epoch());
  response->set_session_id(request.expected_session_id());
  response->set_reference_epoch(request.expected_reference_epoch());
  response->set_motion_epoch(request.expected_motion_epoch() +
      (request.request_id() == 43U ? 2U : 1U));
  response->set_management_epoch(request.expected_management_epoch() + 1U);
  response->set_profile_id(request.expected_profile_id());
  if (request.operation() == 8U && request.request_id() == 47U) {
    response->set_operation_id(0U);
    response->set_transaction_phase(1U);
    response->set_result_code(23U);
    response->set_motion_epoch(request.expected_motion_epoch());
    response->set_management_epoch(request.expected_management_epoch());
  } else if (request.operation() == 12U && request.queried_operation_id() == 47U) {
    response->set_operation_id(47U);
  }
  if (request.operation() == 10U) {
    if (request.request_id() == 80U) {
      peer.gate_release_request_id = request.request_id();
      response->set_run_id(2U);
      response->set_attempt_kind(1U);
      if (peer.direct_gate_release) {
        response->set_operation_id(80U);
        response->set_session_id(0U);
        response->set_reference_epoch(0U);
        response->set_release_committed(true);
        return dp::comm::Result::kOk;
      }
      response->set_operation_id(0U);
      response->set_transaction_phase(1U);
      response->set_result_code(23U);
      response->set_motion_epoch(request.expected_motion_epoch());
      response->set_management_epoch(request.expected_management_epoch());
      return dp::comm::Result::kOk;
    }
    peer.stop_request_id = request.request_id();
    peer.stop_motion_epoch = request.expected_motion_epoch();
    peer.stop_management_epoch = request.expected_management_epoch();
    peer.stop_queries = 0U;
    response->set_operation_id(0U);
    response->set_transaction_phase(1U);
    response->set_result_code(23U);
    response->set_motion_epoch(peer.stop_motion_epoch);
    response->set_management_epoch(peer.stop_management_epoch);
    if (peer.revoke_owner_before_stop_receipt) {
      response->set_session_id(0U);
      response->set_reference_epoch(0U);
    }
  } else if (stop_query) {
    ++peer.stop_queries;
    if (peer.stop_queries == 1U || peer.never_complete_stop) {
      response->set_operation_id(0U);
      response->set_transaction_phase(2U);
      response->set_result_code(23U);
      response->set_motion_epoch(peer.stop_motion_epoch);
      response->set_management_epoch(peer.stop_management_epoch);
      if (peer.revoke_owner_before_stop_receipt) {
        response->set_session_id(0U);
        response->set_reference_epoch(0U);
      }
    } else {
      response->set_operation_id(900U);
      response->set_session_id(0U);
      response->set_reference_epoch(0U);
      response->set_motion_epoch(peer.stop_motion_epoch + 1U);
      response->set_management_epoch(peer.stop_management_epoch + 1U);
    }
  } else if (request.operation() == 12U &&
             request.queried_operation_id() == peer.gate_release_request_id) {
    response->set_run_id(2U);
    response->set_attempt_kind(1U);
    response->set_session_id(0U);
    response->set_reference_epoch(0U);
    response->set_release_committed(true);
  }
  return dp::comm::Result::kOk;
}
int run_peer(const std::string &endpoint, int input, int output, int stops_output,
             bool never_complete_stop, bool revoke_owner_before_stop_receipt,
             bool direct_gate_release) {
  dp::comm::StaticStorage storage({1U, 3U, 1U, 0U, 8U,
      wire::DummyDeviceStatusV2Codec::kEncodedSize, 96U, 1U, 0U, 1U, 1U});
  const dp::comm::TransportConfig config{"peer", "", endpoint.c_str(), "component_peer"};
  auto initialized = dp::comm::Context::init(storage,
      dp::comm::InitOptions{dp::comm::MemoryMode::kStaticOnly, &config, nullptr});
  if (!initialized.has_value()) return 1;
  auto context = std::move(initialized).take_value();
  dp::comm::NodeOptions options{};
  options.product_name = "dummy_robot";
  options.device_id.bytes[0] = 1U;
  options.node_name = "ref_control";
  auto created = context.create_node(options);
  if (!created.has_value()) return 2;
  auto node = std::move(created).take_value();
  auto states = node.create_publisher<State>("state_v2", {});
  auto statuses = node.create_publisher<Status>("status_v2", {});
  auto evidences = node.create_publisher<Evidence>("attempt_evidence_v2", {});
  Peer peer{output, stops_output};
  peer.never_complete_stop = never_complete_stop;
  peer.revoke_owner_before_stop_receipt = revoke_owner_before_stop_receipt;
  peer.direct_gate_release = direct_gate_release;
  auto subscribed = node.create_subscription<Command>("command_v2", {}, receive_command, &peer);
  auto managed = node.create_service<Management>("manage_v2", receive_management, &peer);
  if (!states.has_value() || !statuses.has_value() || !evidences.has_value() ||
      !subscribed.has_value() || !managed.has_value()) return 3;
  auto state_publisher = std::move(states).take_value();
  auto status_publisher = std::move(statuses).take_value();
  auto evidence_publisher = std::move(evidences).take_value();
  auto subscription = std::move(subscribed).take_value();
  auto service = std::move(managed).take_value();
  dp::comm::StaticExecutor executor;
  if (executor.add_node(node) != dp::comm::Result::kOk || node.activate() != dp::comm::Result::kOk) return 4;
  const char ready = 'R';
  if (::write(output, &ready, 1) != 1) return 5;
  for (;;) {
    pollfd request{input, POLLIN, 0};
    if (::poll(&request, 1, 1) > 0) {
      Stimulus stimulus{};
      if (::read(input, &stimulus, sizeof(stimulus)) != sizeof(stimulus)) return 6;
      if (stimulus.done) return 0;
      const auto status_result = stimulus.send_status ? status_publisher.publish(stimulus.status) : dp::comm::Result::kOk;
      const auto state_result = stimulus.send_state ? state_publisher.publish(stimulus.state) : dp::comm::Result::kOk;
      const auto evidence_result = stimulus.send_evidence ? evidence_publisher.publish(stimulus.evidence) : dp::comm::Result::kOk;
      if (status_result != dp::comm::Result::kOk || state_result != dp::comm::Result::kOk ||
          evidence_result != dp::comm::Result::kOk) {
        std::fprintf(stderr, "peer publish failed status=%u state=%u evidence=%u\n",
                     static_cast<unsigned>(status_result), static_cast<unsigned>(state_result),
                     static_cast<unsigned>(evidence_result));
        return 7;
      }
    }
    const auto spun = executor.spin_some();
    if (spun != dp::comm::Result::kOk && spun != dp::comm::Result::kBusy) return 8;
  }
}
template <typename Predicate> bool wait_for(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!predicate() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return predicate();
}
int run_parent(const std::string &endpoint, int input, int output, int stops_input,
               bool six_axis, bool ack_timeout_mode, bool default_ack_age_mode,
               bool never_complete_stop, bool heartbeat_budget_mode,
               bool inactive_prime_mode, bool cold_idle_gap_mode,
               bool cold_fault_gap_mode, bool raw_capture_mode,
               bool raw_bad_release_mode, bool raw_incomplete_mode,
               bool raw_many_mode, unsigned raw_order_case, unsigned complete_order_mode,
               unsigned release_deactivate_mode, unsigned ticket_mode,
               bool reject_during_ack_mode) {
  pollfd response{input, POLLIN, 0};
  char ready{};
  if (::poll(&response, 1, 3000) != 1 || ::read(input, &ready, 1) != 1 || ready != 'R') return 10;
  TransportParameters parameters{};
  parameters.endpoint = endpoint;
  parameters.device_id[0] = 1U;
  parameters.client_instance_id[0] = 2U;
  parameters.profile_id = 1U;
  parameters.configured_mask = six_axis ? 63U : 1U;
  const bool cold_gap_mode = cold_idle_gap_mode || cold_fault_gap_mode;
  parameters.max_state_age_us = (heartbeat_budget_mode || cold_gap_mode ||
      release_deactivate_mode >= 8U) ? 250000U :
      (default_ack_age_mode ? 200000U : 1000000U);
  if (heartbeat_budget_mode || cold_gap_mode || release_deactivate_mode >= 8U)
    parameters.heartbeat_stop_budget_us = 250000U;
  if (ack_timeout_mode) parameters.group_sent_ack_timeout_us = 30000U;
  if (reject_during_ack_mode) parameters.group_sent_ack_timeout_us = 250000U;
  if ((complete_order_mode>=6U && complete_order_mode<=8U) || complete_order_mode==10U ||
      complete_order_mode==27U)
    parameters.group_sent_ack_timeout_us = 30000U;
  parameters.control_period_us = 10000U;
  if (release_deactivate_mode == 6U) parameters.heartbeat_stop_budget_us = 500000U;
  parameters.state_publish_period_us = (heartbeat_budget_mode || cold_gap_mode ||
      release_deactivate_mode >= 8U) ? 20000U :
      (six_axis ? 50000U : 1000000U);
  parameters.contract_hash[0] = 1U;
  parameters.model_hash[0] = 2U;
  parameters.profile_hash[0] = 3U;
  parameters.firmware_hash[0] = 4U;
  parameters.policy_hash[0] = 5U;
  parameters.position_min.fill(-1.0);
  parameters.position_max.fill(1.0);
  parameters.velocity_max.fill(1.0);
  parameters.acceleration_max.fill(2.0);
  std::string capture_path;
  struct CaptureCleanup {
    const std::string &path;
    ~CaptureCleanup() { if (!path.empty()) ::unlink(path.c_str()); }
  } capture_cleanup{capture_path};
  if (raw_capture_mode) {
    char path[] = "/tmp/dummy-transport-raw-XXXXXX";
    const int fd = ::mkstemp(path);
    if (fd < 0) return 137;
    ::close(fd);
    ::unlink(path);
    capture_path = path;
    parameters.raw_evidence_path = capture_path;
  }
  GuardedZenbeddedSystem system;
  DeviceTransport transport(system, parameters);
  if (!transport.start() || !wait_for([&] { return transport.statistics().ready; })) return 11;
  Stimulus valid{};
  valid.status.set_schema_major(2U);
  valid.status.set_schema_minor(4U);
  valid.state.set_schema_minor(4U);
  valid.state.set_schema_major(2U);
  std::uint8_t device[16]{1U};
  valid.status.set_device_id(device);
  valid.state.set_device_id(device);
  valid.status.set_boot_epoch(7U);
  valid.state.set_boot_epoch(7U);
  valid.status.set_session_id(8U);
  valid.state.set_session_id(8U);
  if(complete_order_mode>=18U && complete_order_mode<=27U) {
    valid.status.set_run_id(1U);
    valid.status.set_attempt_kind(1U);
  }
  if (raw_capture_mode) {
    valid.status.set_run_id(raw_order_case >= 34U && raw_order_case <= 50U ? 1U :
        raw_order_case >= 24U && raw_order_case <= 33U ? 123U :
        raw_order_case >= 20U && raw_order_case <= 23U ? 121U :
        raw_order_case != 0U ? 1U : 3U);
    valid.status.set_attempt_kind(1U);
  }
  valid.status.set_reference_epoch(9U);
  valid.state.set_reference_epoch(9U);
  valid.status.set_motion_epoch(10U);
  if (reject_during_ack_mode) valid.status.set_command_reject_count(3U);
  valid.state.set_motion_epoch(10U);
  if (cold_gap_mode) {
    valid.status.set_session_id(0U);
    valid.state.set_session_id(0U);
    valid.status.set_reference_epoch(0U);
    valid.state.set_reference_epoch(0U);
    valid.status.set_motion_epoch(0U);
    valid.state.set_motion_epoch(0U);
    valid.status.set_core_state(1U);
    valid.state.set_core_state(1U);
  }
  if (ack_timeout_mode || default_ack_age_mode || release_deactivate_mode != 0U) {
    valid.status.set_management_epoch(1U);
    valid.state.set_management_epoch(1U);
    valid.status.set_can_sent_mask(parameters.configured_mask);
  }
  valid.status.set_profile_id(1U);
  valid.state.set_profile_id(1U);
  valid.status.set_configured_mask(parameters.configured_mask);
  valid.state.set_configured_mask(parameters.configured_mask);
  valid.state.set_position_valid_mask(parameters.configured_mask);
  if (cold_gap_mode) {
    valid.state.set_position_valid_mask(0U);
    valid.state.set_raw_position_valid_mask(parameters.configured_mask);
    const double idle_raw[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    valid.state.set_raw_motor_position_rev(idle_raw);
  }
  valid.status.set_capabilities(kPositionOnlyCapability);
  valid.status.set_control_period_us(parameters.control_period_us);
  valid.status.set_state_publish_period_us(parameters.state_publish_period_us);
  std::uint8_t contract_hash[32]{};
  std::uint8_t model_hash[32]{};
  std::uint8_t profile_hash[32]{};
  std::uint8_t firmware_hash[32]{};
  std::uint8_t policy_hash[32]{};
  contract_hash[0] = 1U; model_hash[0] = 2U; profile_hash[0] = 3U;
  firmware_hash[0] = 4U; policy_hash[0] = 5U;
  valid.status.set_contract_sha256(contract_hash);
  valid.status.set_model_sha256(model_hash);
  valid.status.set_profile_sha256(profile_hash);
  valid.status.set_firmware_sha256(firmware_hash);
  valid.status.set_compiled_policy_sha256(policy_hash);
  const double minimum[6]{-1, -1, -1, -1, -1, -1};
  const double maximum[6]{1, 1, 1, 1, 1, 1};
  const double velocity[6]{1, 1, 1, 1, 1, 1};
  const double acceleration[6]{2, 2, 2, 2, 2, 2};
  valid.status.set_position_min_rad(minimum);
  valid.status.set_position_max_rad(maximum);
  valid.status.set_velocity_max_rad_s(velocity);
  valid.status.set_acceleration_max_rad_s(acceleration);
  valid.state.set_state_seq(1U);
  const double positions[6]{0.25, six_axis ? 0.2 : NAN, six_axis ? 0.3 : NAN,
                           six_axis ? 0.4 : NAN, six_axis ? 0.5 : NAN, six_axis ? 0.6 : NAN};
  valid.state.set_position_rad(positions);
  if (ticket_mode != 0U) valid.state.set_freshness_ticket(9007199254740996ULL);
  const auto host_clock_anchor_us = transport_steady_now_us();
  const auto device_now_us = [&] {
    return 5000000000ULL + transport_steady_now_us() - host_clock_anchor_us;
  };
  std::uint64_t synthetic_ticket = 9007199254740997ULL;
  auto send = [&](Stimulus stimulus) {
    if (stimulus.state.produced_at_us() == 0U)
      stimulus.state.set_produced_at_us(device_now_us());
    if (ticket_mode == 0U && stimulus.send_state)
      stimulus.state.set_freshness_ticket(++synthetic_ticket);
    return ::write(output, &stimulus, sizeof(stimulus)) == sizeof(stimulus);
  };
  auto send_rejected_state = [&](Stimulus stimulus) {
    const auto before = transport.statistics();
    return wait_for([&] {
      if (!send(stimulus)) return false;
      const auto after = transport.statistics();
      return after.state_messages > before.state_messages &&
             after.rejected_states > before.rejected_states;
    });
  };
  JointStateSnapshot snapshot{};
  const auto channel = DeviceTransport::management_channel(parameters);
  if (!channel) return 55;
  std::shared_ptr<DeviceManagementRequest> captured_release;
  std::shared_ptr<DeviceManagementRequest> captured_query;
  for(auto &value:channel->post_complete_hold_tolerance_rad)
    value.store(0.003,std::memory_order_release);
  auto unbound = valid;
  unbound.send_status = false;
  if (!wait_for([&] { send(unbound); return transport.statistics().rejected_states != 0U; }) ||
      system.copy_state(&snapshot)) return 48;
  if (release_deactivate_mode != 0U) {
    if (!wait_for([&] { send(valid); return system.copy_state(&snapshot); })) return 230;
    if (release_deactivate_mode != 4U) {
      Management::Request release;
      release.set_schema_major(2U);
      release.set_schema_minor(4U);
      release.set_device_id(device);
      std::uint8_t client[16]{2U};
      release.set_client_instance_id(client);
      release.set_request_id(80U);
      release.set_request_seq(80U);
      release.set_operation(10U);
      release.set_expected_boot_epoch(7U);
      release.set_expected_session_id(8U);
      release.set_expected_reference_epoch(9U);
      release.set_expected_motion_epoch(10U);
      release.set_expected_management_epoch(1U);
      release.set_expected_profile_id(1U);
      release.set_configured_mask(parameters.configured_mask);
      if (release_deactivate_mode != 5U) {
        auto explicit_release = std::make_shared<DeviceManagementRequest>();
        if (wire::DummyManageV2RequestCodec::encode(release, explicit_release->request.data(),
            explicit_release->request.size()) != wire::CodecResult::kOk) return 231;
        {
          std::lock_guard<std::mutex> lock(channel->mutex);
          channel->queued = explicit_release;
        }
        if (!wait_for([&] { return explicit_release->complete.load(); })) return 232;
        captured_release = explicit_release;
        Management::Response release_receipt;
        if (wire::DummyManageV2ResponseCodec::decode(explicit_release->response.data(),
            explicit_release->response.size(), &release_receipt) != wire::CodecResult::kOk ||
            release_receipt.transaction_phase() != (release_deactivate_mode == 7U ? 3U : 1U))
          return 233;
        if (release_deactivate_mode == 9U) {
          auto disabling = valid;
          disabling.send_state = false;
          disabling.status.set_core_state(7U);
          disabling.status.set_motion_epoch(11U);
          disabling.status.set_management_epoch(2U);
          const auto statuses_before = transport.statistics().status_messages;
          if (!send(disabling) || !wait_for([&] {
                return transport.statistics().status_messages > statuses_before;
              })) return 249;
          auto early_idle = valid;
          early_idle.send_status = false;
          early_idle.state.set_core_state(1U);
          early_idle.state.set_session_id(0U);
          early_idle.state.set_reference_epoch(0U);
          early_idle.state.set_motion_epoch(11U);
          early_idle.state.set_management_epoch(2U);
          early_idle.state.set_position_valid_mask(0U);
          early_idle.state.set_raw_position_valid_mask(parameters.configured_mask);
          const double raw[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
          early_idle.state.set_raw_motor_position_rev(raw);
          auto wrong_boot = early_idle;
          wrong_boot.state.set_boot_epoch(8U);
          if (!send_rejected_state(wrong_boot)) return 250;
          auto wrong_epoch = early_idle;
          wrong_epoch.state.set_motion_epoch(12U);
          if (!send_rejected_state(wrong_epoch)) return 251;
          for (std::uint32_t seq = 2U; seq <= 16U; ++seq) {
            early_idle.state.set_state_seq(seq);
            if (!wait_for([&] {
                  send(early_idle);
                  std::lock_guard<std::mutex> lock(channel->mutex);
                  State observed;
                  return wire::DummyJointStateV2Codec::decode(channel->observation.state.data(),
                      channel->observation.state.size(), &observed) == wire::CodecResult::kOk &&
                      observed.state_seq() == seq;
                })) return 249;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
        }
        if (release_deactivate_mode != 7U) {
          release.set_request_id(81U);
          release.set_request_seq(81U);
          release.set_operation(12U);
          release.set_queried_operation_id(80U);
          auto query = std::make_shared<DeviceManagementRequest>();
          if (wire::DummyManageV2RequestCodec::encode(release, query->request.data(),
              query->request.size()) != wire::CodecResult::kOk) return 237;
          {
            std::lock_guard<std::mutex> lock(channel->mutex);
            channel->queued = query;
          }
          if (!wait_for([&] { return query->complete.load(); })) return 238;
          captured_query = query;
          Management::Response release_reply;
          if (wire::DummyManageV2ResponseCodec::decode(query->response.data(),
              query->response.size(), &release_reply) != wire::CodecResult::kOk ||
              !release_reply.release_committed() || release_reply.transaction_phase() != 3U)
            return 239;
        }
      }
      if (release_deactivate_mode == 10U) {
        if (!wait_for([&] {
              return transport.statistics().state_heartbeat_gap_stops != 0U;
            })) return 252;
        transport.stop();
        return 0;
      }
      auto released = valid;
      released.status.set_core_state(1U);
      released.state.set_core_state(1U);
      released.status.set_session_id(0U);
      released.state.set_session_id(0U);
      released.status.set_reference_epoch(0U);
      released.state.set_reference_epoch(0U);
      released.status.set_motion_epoch(11U);
      released.state.set_motion_epoch(11U);
      released.status.set_management_epoch(2U);
      released.state.set_management_epoch(2U);
      released.status.set_release_committed(release_deactivate_mode != 2U);
      released.status.set_permit_remaining_us(0U);
      released.state.set_position_valid_mask(0U);
      released.state.set_raw_position_valid_mask(parameters.configured_mask);
      const double idle_raw[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
      released.state.set_raw_motor_position_rev(idle_raw);
      released.state.set_state_seq(2U);
      if (release_deactivate_mode == 9U) released.state.set_state_seq(17U);
      if (release_deactivate_mode == 6U) {
        auto old_owner_status = valid;
        old_owner_status.send_state = false;
        old_owner_status.status.set_core_state(7U);
        old_owner_status.status.set_motion_epoch(11U);
        old_owner_status.status.set_management_epoch(2U);
        const auto before_status = transport.statistics().status_messages;
        if (!send(old_owner_status) || !wait_for([&] {
              return transport.statistics().status_messages > before_status;
            })) return 240;
        auto early_idle_state = released;
        early_idle_state.send_status = false;
        if (!wait_for([&] {
              send(early_idle_state);
              std::lock_guard<std::mutex> lock(channel->mutex);
              State observed;
              return channel->observation.state_received_us != 0U &&
                  wire::DummyJointStateV2Codec::decode(channel->observation.state.data(),
                      channel->observation.state.size(), &observed) == wire::CodecResult::kOk &&
                  observed.state_seq() == 2U;
            })) return 241;
        for (std::uint32_t seq = 3U; seq <= 31U; ++seq) {
          early_idle_state.state.set_state_seq(seq);
          if (!wait_for([&] {
                send(early_idle_state);
                std::lock_guard<std::mutex> lock(channel->mutex);
                State observed;
                return wire::DummyJointStateV2Codec::decode(channel->observation.state.data(),
                    channel->observation.state.size(), &observed) == wire::CodecResult::kOk &&
                    observed.state_seq() == seq;
              })) return 242;
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (transport.statistics().heartbeat_stops != 0U ||
            transport.statistics().stop_requests_sent != 0U) return 243;
        released.state.set_state_seq(32U);
      }
      if (!wait_for([&] {
            send(released);
            std::lock_guard<std::mutex> lock(channel->mutex);
            State observed;
            return channel->observation.state_received_us != 0U &&
                wire::DummyJointStateV2Codec::decode(channel->observation.state.data(),
                    channel->observation.state.size(), &observed) == wire::CodecResult::kOk &&
                observed.state_seq() == released.state.state_seq();
          })) return 234;
      if (release_deactivate_mode == 8U || release_deactivate_mode == 9U) {
        std::this_thread::sleep_for(std::chrono::milliseconds(310));
        const auto stats = transport.statistics();
        if (stats.state_heartbeat_gap_stops != 0U || stats.stop_requests_sent != 0U ||
            stats.unconfirmed_stop_requests != 0U) return 248;
      }
    }
    const auto before = transport.statistics().stop_requests_sent;
    if (release_deactivate_mode == 3U) system.on_error(rclcpp_lifecycle::State());
    else system.on_deactivate(rclcpp_lifecycle::State());
    const bool expect_stop = release_deactivate_mode != 1U && release_deactivate_mode != 6U &&
        release_deactivate_mode != 7U &&
        release_deactivate_mode != 8U && release_deactivate_mode != 9U;
    if (expect_stop && !wait_for([&] {
          return transport.statistics().stop_requests_sent > before;
        })) return 235;
    if (!expect_stop) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (transport.statistics().stop_requests_sent != before) return 236;
    }
    transport.stop();
    if (raw_capture_mode && (release_deactivate_mode == 1U || release_deactivate_mode == 7U)) {
      std::FILE *file = std::fopen(capture_path.c_str(), "rb");
      if (!file || ::fseek(file, 8L, SEEK_SET) != 0) return 244;
      const auto le = [](const std::uint8_t *bytes, std::size_t width) {
        std::uint64_t value = 0U;
        for (std::size_t i = 0U; i < width; ++i) value |= std::uint64_t{bytes[i]} << (8U * i);
        return value;
      };
      unsigned management_records = 0U;
      std::uint64_t previous_time = 0U;
      bool valid_capture = true;
      std::uint8_t header[39]{};
      while (valid_capture) {
        const auto header_bytes = std::fread(header, 1U, sizeof(header), file);
        if (header_bytes == 0U && std::feof(file)) break;
        if (header_bytes != sizeof(header)) { valid_capture = false; break; }
        const auto size = static_cast<std::uint16_t>(header[1] | (header[2] << 8U));
        std::array<std::uint8_t, wire::DummyDeviceStatusV2Codec::kEncodedSize> raw{};
        const auto time = le(header + 3U, 8U);
        if (size == 0U || size > raw.size() ||
            std::fread(raw.data(), 1U, size, file) != size ||
            time == 0U || time < previous_time) { valid_capture = false; break; }
        previous_time = time;
        if (header[0] == 4U || header[0] == 5U) {
          const bool is_request = (management_records % 2U) == 0U;
          const auto &expected = management_records < 2U ? captured_release : captured_query;
          if (!expected || management_records >= (release_deactivate_mode == 7U ? 2U : 4U) ||
              header[0] != (is_request ? 4U : 5U) ||
              size != (is_request ? wire::DummyManageV2RequestCodec::kEncodedSize :
                                    wire::DummyManageV2ResponseCodec::kEncodedSize)) {
            valid_capture = false;
            break;
          }
          if (is_request) {
            Management::Request request;
            valid_capture = std::equal(raw.begin(), raw.begin() + size, expected->request.begin()) &&
                wire::DummyManageV2RequestCodec::decode(raw.data(), size, &request) ==
                    wire::CodecResult::kOk &&
                le(header + 11U, 8U) == request.expected_boot_epoch() &&
                le(header + 19U, 8U) == request.expected_session_id() &&
                le(header + 27U, 8U) == 0U &&
                le(header + 35U, 4U) == request.request_seq() &&
                request.operation() == (management_records == 0U ? 10U : 12U) &&
                request.request_id() == (management_records == 0U ? 80U : 81U) &&
                (management_records == 0U || request.queried_operation_id() == 80U);
          } else {
            Management::Response response;
            valid_capture = std::equal(raw.begin(), raw.begin() + size, expected->response.begin()) &&
                wire::DummyManageV2ResponseCodec::decode(raw.data(), size, &response) ==
                    wire::CodecResult::kOk &&
                le(header + 11U, 8U) == response.boot_epoch() &&
                le(header + 19U, 8U) == response.session_id() &&
                le(header + 27U, 8U) == response.run_id() &&
                le(header + 35U, 4U) == 0U && response.run_id() == 2U &&
                response.request_id() == (management_records == 1U ? 80U : 81U) &&
                response.transaction_phase() ==
                    (management_records == 1U && release_deactivate_mode == 1U ? 1U : 3U) &&
                (management_records == 1U && release_deactivate_mode == 1U ?
                    !response.release_committed() : response.release_committed());
          }
          ++management_records;
        }
      }
      std::fclose(file);
      if (!valid_capture || management_records != (release_deactivate_mode == 7U ? 2U : 4U))
        return 247;
    }
    return 0;
  }
  if (cold_gap_mode) {
    if (!wait_for([&] {
          send(valid);
          std::lock_guard<std::mutex> lock(channel->mutex);
          State observed;
          return channel->observation.state_received_us != 0U &&
              wire::DummyJointStateV2Codec::decode(channel->observation.state.data(),
                  channel->observation.state.size(), &observed) == wire::CodecResult::kOk &&
              observed.state_seq() == 1U;
        })) return 154;
    if (cold_fault_gap_mode) {
      auto fault = valid;
      fault.send_state = false;
      fault.status.set_core_state(9U);
      const auto before = transport.statistics().status_messages;
      if (!send(fault) || !wait_for([&] {
            return transport.statistics().status_messages > before;
          })) return 155;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(102));
    if (transport.statistics().heartbeat_stops != 0U) return 156;
    if (!wait_for([&] {
          const auto stats = transport.statistics();
          return cold_fault_gap_mode ? stats.heartbeat_stops != 0U :
              stats.raw_idle_reobservations != 0U;
        })) return 157;
    if (cold_fault_gap_mode) {
      if (transport.statistics().raw_idle_reobservations != 0U) return 158;
      if (!send_rejected_state(valid) || system.copy_state(&snapshot)) return 159;
    } else {
      if (transport.statistics().stop_requests_sent != 0U ||
          transport.statistics().unconfirmed_stop_requests != 0U) return 160;
      auto wrong_hash = valid;
      wrong_hash.send_state = false;
      std::uint8_t mismatched_hash[32]{};
      mismatched_hash[0] = 9U;
      wrong_hash.status.set_contract_sha256(mismatched_hash);
      const auto bootstrap_before = transport.statistics().bootstrap_rejections;
      if (!send(wrong_hash) || !wait_for([&] {
            return transport.statistics().bootstrap_rejections > bootstrap_before;
          })) return 164;
      auto premature = valid;
      premature.send_status = false;
      premature.state.set_state_seq(2U);
      if (!send_rejected_state(premature)) return 165;
      auto status_only = valid;
      status_only.send_state = false;
      const auto before = transport.statistics().status_messages;
      if (!send(status_only) || !wait_for([&] {
            return transport.statistics().status_messages > before;
          })) return 161;
      auto stale = valid;
      stale.send_status = false;
      if (!send_rejected_state(stale)) return 162;
      auto next = stale;
      next.state.set_state_seq(2U);
      if (!wait_for([&] {
            send(next);
            std::lock_guard<std::mutex> lock(channel->mutex);
            State observed;
            return wire::DummyJointStateV2Codec::decode(channel->observation.state.data(),
                channel->observation.state.size(), &observed) == wire::CodecResult::kOk &&
                observed.state_seq() == 2U;
          }) || transport.statistics().stop_requests_sent != 0U) return 163;
    }
    transport.stop();
    return 0;
  }
  if (raw_order_case >= 15U && raw_order_case <= 19U) {
    auto historical = valid;
    historical.status.set_run_id(2U);
    historical.status.set_attempt_kind(1U);
    historical.status.set_parent_run_id(0U);
    historical.status.set_session_id(0U);
    historical.status.set_reference_epoch(0U);
    historical.status.set_core_state(1U);
    historical.status.set_final_state(2U);
    historical.status.set_release_committed(raw_order_case != 16U);
    historical.status.set_permit_remaining_us(0U);
    historical.state.set_session_id(0U);
    historical.state.set_reference_epoch(0U);
    historical.state.set_core_state(1U);
    historical.state.set_position_valid_mask(0U);
    historical.state.set_raw_position_valid_mask(parameters.configured_mask);
    const double raw_motor_rev[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    historical.state.set_raw_motor_position_rev(raw_motor_rev);
    if (!send(historical)) return 285;
    if (raw_order_case == 16U) {
      if (!wait_for([&] { return transport.statistics().raw_evidence_failure != 0U; }))
        return 288;
      transport.stop();
      return 0;
    }
    if (!wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          return wire::DummyDeviceStatusV2Codec::decode(
              channel->observation.status.data(), channel->observation.status.size(),
              &observed) == wire::CodecResult::kOk && observed.run_id() == 2U;
        })) return 285;
    if (raw_order_case >= 17U) {
      auto changed = historical;
      changed.send_state = false;
      if (raw_order_case == 17U) changed.status.set_boot_epoch(8U);
      if (raw_order_case == 18U) changed.status.set_session_id(9U);
      if (raw_order_case == 19U) {
        changed.status.set_core_state(4U);
        changed.status.set_final_state(0U);
      }
      const auto status_count = transport.statistics().status_messages;
      if (raw_order_case == 18U) {
        if (!send(changed) || !wait_for([&] {
              return transport.statistics().status_messages > status_count;
            })) return 289;
        bool retained = false;
        {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          retained = wire::DummyDeviceStatusV2Codec::decode(
              channel->observation.status.data(), channel->observation.status.size(),
              &observed) == wire::CodecResult::kOk && observed.session_id() == 0U &&
              transport.statistics().raw_evidence_failure == 0U;
        }
        transport.stop();
        return retained ? 0 : 290;
      }
      if (!send(changed) || !wait_for([&] {
            return transport.statistics().raw_evidence_failure != 0U;
          })) return 289;
      transport.stop();
      return 0;
    }
    auto stale_page = historical;
    stale_page.send_state = false;
    stale_page.send_status = false;
    stale_page.send_evidence = true;
    stale_page.evidence.set_schema_major(2U);
    stale_page.evidence.set_schema_minor(4U);
    stale_page.evidence.set_run_id(2U);
    stale_page.evidence.set_attempt_id(2U);
    stale_page.evidence.set_evidence_seq(0U);
    stale_page.evidence.set_attempt_kind(1U);
    stale_page.evidence.set_parent_run_id(0U);
    stale_page.evidence.set_motion_epoch(10U);
    stale_page.evidence.set_category(2U);
    stale_page.evidence.set_final(1U);
    stale_page.evidence.set_group_count(1U);
    stale_page.evidence.set_group_id(2U);
    stale_page.evidence.set_expected_mask(63U);
    stale_page.evidence.set_submitted_mask(63U);
    stale_page.evidence.set_sent_mask(63U);
    if (!send(stale_page)) return 286;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto captured = transport.statistics();
    transport.stop();
    return captured.ready && !captured.initialization_failed &&
        captured.raw_evidence_failure == 0U &&
        captured.raw_evidence_pages_written == 0U ? 0 : 287;
  }
  if (six_axis && !heartbeat_budget_mode) {
    auto cold = valid;
    cold.status.set_core_state(1U);
    cold.state.set_core_state(1U);
    cold.state.set_position_valid_mask(0U);
    cold.state.set_velocity_valid_mask(63U);
    cold.state.set_raw_position_valid_mask(63U);
    const double raw_motor_rev[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    cold.state.set_raw_motor_position_rev(raw_motor_rev);
    if (!wait_for([&] {
          send(cold);
          std::lock_guard<std::mutex> lock(channel->mutex);
          State observed;
          return channel->observation.state_received_us != 0U &&
              wire::DummyJointStateV2Codec::decode(channel->observation.state.data(),
                channel->observation.state.size(), &observed) == wire::CodecResult::kOk &&
              observed.core_state() == 1U && observed.raw_position_valid_mask() == 63U;
        }) || system.copy_state(&snapshot)) return 84;
    std::uint64_t cold_received_us = 0U;
    {
      std::lock_guard<std::mutex> lock(channel->mutex);
      cold_received_us = channel->observation.state_received_us;
    }
    if (!send_rejected_state(cold)) return 85;
    {
      std::lock_guard<std::mutex> lock(channel->mutex);
      if (channel->observation.state_received_us != cold_received_us) return 86;
    }
    for (unsigned fault = 0U; fault < 5U; ++fault) {
      auto wrong = cold;
      const auto diagnostics_before = transport.statistics();
      wrong.state.set_state_seq(2U);
      if (fault == 0U) wrong.state.set_raw_position_valid_mask(0U);
      if (fault == 1U) wrong.state.set_produced_at_us(device_now_us() - 2000000U);
      if (fault == 2U) wrong.state.set_produced_at_us(device_now_us() + 2000000U);
      if (fault == 3U) wrong.state.set_core_state(4U);
      if (fault == 4U) {
        const double bad_raw[6]{NAN, 0.2, 0.3, 0.4, 0.5, 0.6};
        wrong.state.set_raw_motor_position_rev(bad_raw);
      }
      if (!send_rejected_state(wrong) || system.copy_state(&snapshot)) return 87 + fault;
      const auto diagnostics_after = transport.statistics();
      if (fault == 1U && diagnostics_after.stale_payload_rejections <=
              diagnostics_before.stale_payload_rejections) return 120;
      if (fault == 2U && diagnostics_after.clock_skew_rejections <=
              diagnostics_before.clock_skew_rejections) return 121;
      std::lock_guard<std::mutex> lock(channel->mutex);
      if (channel->observation.state_received_us != cold_received_us) return 92 + fault;
    }
    valid.state.set_state_seq(2U);
  }
  if (!wait_for([&] { send(valid); return system.copy_state(&snapshot); })) return 12;
  if (complete_order_mode != 0U) {
    valid.status.set_management_epoch(10U);
    valid.state.set_management_epoch(10U);
    valid.state.set_state_seq(2U);
    if (!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                              snapshot.envelope.state_seq == 2U; })) return 166;
    std::uint32_t terminal_sequence=0U;
    if(complete_order_mode==11U || complete_order_mode==12U) {
      channel->inactive_prime_allowed.store(true,std::memory_order_release);
      set_command_identity({7U,8U,9U,10U,9007199254740997ULL});
      for(unsigned sample=0U;sample<2U;++sample) {
        const auto generation=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!generation || !system.transfer_control_update() || ::poll(&response,1,3000)!=1)
          return 187;
        Command command;
        if(::read(input,&command,sizeof(command))!=sizeof(command) ||
           command.motion_epoch()!=10U || command.producer_seq()!=generation) return 188;
        terminal_sequence=command.producer_seq();
        valid.state.set_state_seq(3U+sample);
        valid.state.set_committed_producer_seq(terminal_sequence);
        if(complete_order_mode==12U && sample==1U) continue;
        if(!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq==3U+sample; })) return 189;
      }
    }
    Management::Request complete;
    complete.set_schema_major(2U); complete.set_schema_minor(4U);
    complete.set_device_id(device);
    const std::uint8_t client[16]{2U};
    complete.set_client_instance_id(client);
    complete.set_request_id(47U); complete.set_request_seq(47U);
    complete.set_operation(8U);
    complete.set_expected_boot_epoch(7U); complete.set_expected_session_id(8U);
    complete.set_expected_reference_epoch(9U); complete.set_expected_motion_epoch(10U);
    complete.set_expected_management_epoch(10U); complete.set_expected_profile_id(1U);
    complete.set_configured_mask(parameters.configured_mask);
    if(complete_order_mode==15U)
      complete.set_expected_management_epoch(11U);
    if(complete_order_mode==11U || complete_order_mode==12U)
      complete.set_terminal_producer_seq(terminal_sequence);
    auto pending=std::make_shared<DeviceManagementRequest>();
    if (wire::DummyManageV2RequestCodec::encode(complete,pending->request.data(),
          pending->request.size())!=wire::CodecResult::kOk) return 167;
    { std::lock_guard<std::mutex> lock(channel->mutex); channel->queued=pending; }
    if(complete_order_mode==15U) {
      pollfd observed_request{stops_input,POLLIN,0};
      if(::poll(&observed_request,1,100)!=0 ||
         !wait_for([&] { return pending->failed.load(std::memory_order_acquire); })) return 196;
      transport.stop();
      return 0;
    }
    if(complete_order_mode==17U) {
      pollfd observed_request{stops_input,POLLIN,0};
      Management::Request sent_complete;
      if(::poll(&observed_request,1,1000)!=1 ||
         ::read(stops_input,&sent_complete,sizeof(sent_complete))!=sizeof(sent_complete) ||
         sent_complete.operation()!=8U) return 205;
      auto enabling=valid;
      enabling.status.set_motion_epoch(11U);
      enabling.status.set_core_state(3U);
      enabling.state.set_motion_epoch(11U);
      enabling.state.set_core_state(3U);
      enabling.state.set_state_seq(3U);
      enabling.send_state=false;
      if(!send(enabling) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          return wire::DummyDeviceStatusV2Codec::decode(channel->observation.status.data(),
              channel->observation.status.size(),&observed)==wire::CodecResult::kOk &&
              observed.motion_epoch()==11U;
        })) return 199;
      enabling.send_status=false;
      enabling.send_state=true;
      if(!wait_for([&] { send(enabling); return system.copy_state(&snapshot) &&
                                snapshot.envelope.state_seq==3U; })) return 200;
      channel->inactive_prime_allowed.store(true,std::memory_order_release);
      set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
      const auto first=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
      if(!first || !system.transfer_control_update() || ::poll(&response,1,3000)!=1) return 201;
      Command first_command;
      if(::read(input,&first_command,sizeof(first_command))!=sizeof(first_command) ||
         first_command.motion_epoch()!=11U || first_command.producer_seq()!=first) return 202;
      double second_position[6]{};
      std::copy(std::begin(positions),std::end(positions),second_position);
      second_position[0]+=0.0008;
      const auto second=publish_control_update(second_position,six_axis ? 6U : 1U,0,nullptr);
      const auto after_first=transport.statistics();
      if(!second || !system.transfer_control_update()) return 203;
      enabling.state.set_state_seq(4U);
      enabling.state.set_committed_producer_seq(first_command.producer_seq());
      if(!wait_for([&] { send(enabling); return system.copy_state(&snapshot) &&
                                snapshot.envelope.state_seq==4U; }) ||
         ::poll(&response,1,100)!=0 ||
         transport.statistics().commands_published!=after_first.commands_published ||
         transport.statistics().group_sent_ack_timeouts!=0U) return 204;
      transport.stop();
      return 0;
    }
    if(complete_order_mode==11U || complete_order_mode==12U) {
      pollfd observed_request{stops_input,POLLIN,0};
      Management::Request sent_complete;
      if(complete_order_mode==12U) {
        if(::poll(&observed_request,1,100)!=0) return 192;
        if(!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq==4U; })) return 193;
      }
      if(::poll(&observed_request,1,1000)!=1 ||
         ::read(stops_input,&sent_complete,sizeof(sent_complete))!=sizeof(sent_complete) ||
         sent_complete.operation()!=8U || sent_complete.terminal_producer_seq()!=terminal_sequence)
        return 168;
      const auto before=transport.statistics();
      for(unsigned sample=0U;sample<3U;++sample) {
        const auto generation=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!generation || !system.transfer_control_update()) return 190;
      }
      if(::poll(&response,1,100)!=0 ||
         transport.statistics().commands_published!=before.commands_published ||
         transport.statistics().group_sent_ack_timeouts!=before.group_sent_ack_timeouts)
        return 191;
      transport.stop();
      return 0;
    }
    if (!wait_for([&] { return pending->complete.load(); })) return 168;
    auto next=valid;
    next.status.set_motion_epoch(11U);
    next.state.set_motion_epoch(11U);
    next.state.set_state_seq(3U);
    next.status.set_core_state(4U);
    next.state.set_core_state(4U);
    next.state.set_hold_ready(true);
    const auto baseline=transport.statistics();
    auto send_part=[&](bool status,bool state) {
      auto part=next;
      part.send_status=status; part.send_state=state;
      part.state.set_produced_at_us(device_now_us());
      return send(part);
    };
    if(complete_order_mode==4U) {
      next.status.set_motion_epoch(12U);
      if(!send_part(true,false) || !wait_for([&] {
          return transport.statistics().invalidations>baseline.invalidations;
        })) return 177;
      transport.stop();
      return 0;
    }
    if(complete_order_mode==9U) {
      next.status.set_core_state(9U);
      if(!send_part(true,false) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          return wire::DummyDeviceStatusV2Codec::decode(
              channel->observation.status.data(),channel->observation.status.size(),
              &observed)==wire::CodecResult::kOk && observed.motion_epoch()==11U;
        }) || transport.statistics().invalidations<=baseline.invalidations) return 184;
      transport.stop();
      return 0;
    }
    if(complete_order_mode==5U) {
      if(!send_part(false,true) || !wait_for([&] {
          return transport.statistics().state_authority_rejections>
            baseline.state_authority_rejections;
        }) || !wait_for([&] {
          return transport.statistics().state_heartbeat_gap_stops>
            baseline.state_heartbeat_gap_stops;
        })) return 178;
      transport.stop();
      return 0;
    }
    auto complete_query=[&] {
      auto query=complete;
      query.set_operation(12U); query.set_request_id(48U); query.set_request_seq(48U);
      query.set_queried_operation_id(47U);
      auto requested=std::make_shared<DeviceManagementRequest>();
      if(wire::DummyManageV2RequestCodec::encode(query,requested->request.data(),
            requested->request.size())!=wire::CodecResult::kOk) return false;
      { std::lock_guard<std::mutex> lock(channel->mutex); channel->queued=requested; }
      return wait_for([&] { return requested->complete.load(); });
    };
    if(complete_order_mode==1U || (complete_order_mode>=6U && complete_order_mode<=8U) ||
       complete_order_mode==10U || complete_order_mode==13U || complete_order_mode==14U ||
       (complete_order_mode>=18U && complete_order_mode<=27U)) {
      if(!send_part(true,false) || !wait_for([&] {
          return transport.statistics().status_messages>baseline.status_messages;
        }) || transport.statistics().invalidations!=baseline.invalidations) return 169;
      if(!send_part(false,true) || !wait_for([&] {
          return system.copy_state(&snapshot) && snapshot.envelope.state_seq==3U;
        })) return 170;
      if(complete_order_mode!=8U && complete_order_mode!=10U &&
         complete_order_mode!=14U && !complete_query()) return 171;
    } else if(complete_order_mode==2U) {
      if(!send_part(false,true) || !wait_for([&] {
          return transport.statistics().state_messages>baseline.state_messages;
        })) return 172;
      if(!send_part(true,false) || !wait_for([&] {
          return transport.statistics().status_messages>baseline.status_messages;
        })) return 173;
      next.state.set_state_seq(4U);
      if(!send_part(false,true) || !wait_for([&] {
          return system.copy_state(&snapshot) && snapshot.envelope.state_seq==4U;
        }) || !complete_query()) return 174;
    } else {
      if(!complete_query() || !send_part(false,true) || !wait_for([&] {
          return system.copy_state(&snapshot) && snapshot.envelope.state_seq==3U;
        }) || !send_part(true,false)) return 175;
    }
    if(complete_order_mode==14U) {
      for(unsigned sample=0U;sample<110U;++sample) {
        next.state.set_state_seq(4U+sample);
        if(!send_part(false,true)) return 195;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    if(complete_order_mode>=18U && complete_order_mode<=27U) {
      auto exported=next;
      exported.send_state=false;
      exported.status.set_final_state(6U);
      exported.status.set_motion_complete(true);
      if(complete_order_mode==21U) exported.status.set_run_id(2U);
      if(complete_order_mode==22U) exported.status.set_evidence_lost(true);
      std::uint8_t chain[32]{};
      if(complete_order_mode!=19U) chain[0]=0x5aU;
      exported.status.set_evidence_chain_sha256(chain);
      if(!send(exported) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          return wire::DummyDeviceStatusV2Codec::decode(channel->observation.status.data(),
              channel->observation.status.size(),&observed)==wire::CodecResult::kOk &&
              observed.final_state()==6U;
        })) return 206;
      if(channel->post_complete_begin_ready.load(std::memory_order_acquire) !=
         (complete_order_mode!=19U && complete_order_mode!=21U && complete_order_mode!=22U))
        return 217;
      if(complete_order_mode>=23U && complete_order_mode<=25U) {
        auto contradiction=exported;
        if(complete_order_mode==23U) contradiction.status.set_evidence_lost(true);
        else if(complete_order_mode==24U) {
          contradiction.status.set_final_state(0U);
          contradiction.status.set_motion_complete(false);
          std::uint8_t empty_chain[32]{};
          contradiction.status.set_evidence_chain_sha256(empty_chain);
        } else {
          contradiction.status.set_attempt_kind(2U);
          contradiction.status.set_parent_run_id(1U);
        }
        if(!send(contradiction) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed;
            return wire::DummyDeviceStatusV2Codec::decode(channel->observation.status.data(),
                channel->observation.status.size(),&observed)==wire::CodecResult::kOk &&
                (complete_order_mode==23U ? observed.evidence_lost() :
                 complete_order_mode==24U ? observed.final_state()==0U :
                 observed.attempt_kind()==2U);
          }) || channel->post_complete_begin_ready.load(std::memory_order_acquire)) return 219;
        channel->inactive_prime_allowed.store(true,std::memory_order_release);
        set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
        const auto before=transport.statistics();
        const auto generation=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!generation || !system.transfer_control_update() || ::poll(&response,1,100)!=0 ||
           transport.statistics().commands_published!=before.commands_published) return 220;
        transport.stop();
        return 0;
      }
      channel->inactive_prime_allowed.store(true,std::memory_order_release);
      set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
      if(complete_order_mode==26U) {
        const auto before=transport.statistics();
        const auto stale=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!stale || !system.transfer_control_update()) return 221;
        clear_command_identity();
        if(!wait_for([&] {
            return transport.statistics().rejected_commands>before.rejected_commands;
          }) || ::poll(&response,1,100)!=0) return 222;
        next.state.set_state_seq(4U);
        next.send_status=false;
        if(!send(next) || !wait_for([&] {
            return system.copy_state(&snapshot) && snapshot.envelope.state_seq==4U;
          })) return 258;
        set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
        const auto fresh=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!fresh || !system.transfer_control_update() || ::poll(&response,1,1000)!=1)
          return 223;
        Command recovered;
        if(::read(input,&recovered,sizeof(recovered))!=sizeof(recovered) ||
           recovered.producer_seq()!=fresh) return 224;
        transport.stop();
        return 0;
      }
      const auto before_refresh=transport.statistics();
      const auto refresh=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
      if(!refresh || !system.transfer_control_update()) return 207;
      if(complete_order_mode==19U || complete_order_mode==21U || complete_order_mode==22U) {
        if(::poll(&response,1,100)!=0 ||
           transport.statistics().commands_published!=before_refresh.commands_published) return 208;
        transport.stop();
        return 0;
      }
      if(::poll(&response,1,1000)!=1) return 209;
      Command first_refresh;
      if(::read(input,&first_refresh,sizeof(first_refresh))!=sizeof(first_refresh) ||
         first_refresh.motion_epoch()!=11U || first_refresh.producer_seq()!=refresh) return 210;
      if(complete_order_mode==20U) {
        std::lock_guard<std::mutex> lock(channel->mutex);
        if(channel->hold_drain.processed_generation>=refresh || channel->hold_drain.quiescent)
          return 259;
      }
      if(complete_order_mode==27U) {
        for(unsigned sample=0U;sample<8U;++sample) {
          next.state.set_state_seq(4U+sample);
          if(!send_part(false,true)) return 225;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if(!wait_for([&] {
            return transport.statistics().group_sent_ack_timeouts>
                before_refresh.group_sent_ack_timeouts;
          })) return 226;
        transport.stop();
        return 0;
      }
      next.state.set_state_seq(4U);
      next.state.set_committed_producer_seq(first_refresh.producer_seq());
      if(!send_part(false,true) || !wait_for([&] {
          return system.copy_state(&snapshot) && snapshot.envelope.state_seq==4U;
        }) || transport.statistics().group_sent_ack_timeouts!=0U) return 211;
      if(complete_order_mode==20U) {
        if(!wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            return channel->hold_drain.processed_generation>=refresh &&
                   channel->hold_drain.committed_seq==refresh &&
                   channel->hold_drain.quiescent && channel->hold_drain.disposition==2U;
          })) return 260;
        const auto before_cooldown=transport.statistics();
        const auto suppressed=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!suppressed || !system.transfer_control_update() || ::poll(&response,1,100)!=0 ||
           transport.statistics().commands_published!=before_cooldown.commands_published)
          return 218;
        if(!wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            return channel->hold_drain.processed_generation>=suppressed &&
                   channel->hold_drain.committed_seq==refresh &&
                   channel->hold_drain.motion_epoch==11U && channel->hold_drain.quiescent &&
                   channel->hold_drain.disposition==1U && channel->hold_drain.observed_us!=0U &&
                   channel->hold_drain.max_state_age_us!=0U &&
                   channel->hold_drain.position[0]==positions[0] &&
                   channel->hold_drain.tolerance[0]==0.003;
          })) return 261;
        auto active_run=exported;
        active_run.status.set_run_id(2U);
        active_run.status.set_final_state(0U);
        active_run.status.set_motion_complete(false);
        std::uint8_t empty_chain[32]{};
        active_run.status.set_evidence_chain_sha256(empty_chain);
        if(!send(active_run) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed;
            return wire::DummyDeviceStatusV2Codec::decode(channel->observation.status.data(),
                channel->observation.status.size(),&observed)==wire::CodecResult::kOk &&
                observed.run_id()==2U;
          })) return 212;
        for(unsigned i=0U;i<55U;++i) {
          next.state.set_state_seq(5U+i);
          if(!send_part(false,true)) return 213;
          if(i%12U==0U && !send(active_run)) return 214;
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        const auto before_rejected=transport.statistics();
        const auto rejected=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!rejected || !system.transfer_control_update()) return 262;
        clear_command_identity();
        if(!wait_for([&] {
            return transport.statistics().rejected_commands>before_rejected.rejected_commands;
          })) return 263;
        {
          std::lock_guard<std::mutex> lock(channel->mutex);
          if(channel->hold_drain.processed_generation>=rejected) return 264;
        }
        set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
        const auto next_refresh=publish_control_update(positions,six_axis ? 6U : 1U,0,nullptr);
        if(!next_refresh || !system.transfer_control_update() || ::poll(&response,1,1000)!=1)
          return 215;
        Command second_refresh;
        if(::read(input,&second_refresh,sizeof(second_refresh))!=sizeof(second_refresh) ||
           second_refresh.producer_seq()!=next_refresh) return 216;
      }
      transport.stop();
      return 0;
    }
    if((complete_order_mode>=6U && complete_order_mode<=8U) ||
       complete_order_mode==10U || complete_order_mode==13U || complete_order_mode==14U) {
      channel->inactive_prime_allowed.store(true,std::memory_order_release);
      set_command_identity({7U,8U,9U,11U,9007199254740997ULL});
      double hold[6]{};
      std::copy(std::begin(positions),std::end(positions),hold);
      if(complete_order_mode==7U) hold[0]+=0.02;
      if(complete_order_mode==13U || complete_order_mode==14U) hold[0]+=0.0008;
      const auto generation=publish_control_update(hold,six_axis ? 6U : 1U,0,nullptr);
      if(!generation || !system.transfer_control_update()) return 179;
      if(complete_order_mode==6U || complete_order_mode==7U || complete_order_mode==8U ||
         complete_order_mode==10U || complete_order_mode==13U || complete_order_mode==14U) {
        if(::poll(&response,1,100)!=0) return 185;
      } else {
        if(::poll(&response,1,3000)!=1) return 180;
        Command command;
        if(::read(input,&command,sizeof(command))!=sizeof(command) ||
           command.motion_epoch()!=11U || command.producer_seq()!=generation) return 180;
      }
      for(unsigned i=0U;i<8U;++i) {
        next.state.set_state_seq(4U+i);
        if(!send_part(false,true)) return 181;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if((complete_order_mode==6U || complete_order_mode==8U || complete_order_mode==10U ||
          complete_order_mode==13U || complete_order_mode==14U) &&
         (transport.statistics().commands_published!=baseline.commands_published ||
          transport.statistics().group_sent_ack_timeouts!=baseline.group_sent_ack_timeouts ||
          transport.statistics().stop_requests_sent!=baseline.stop_requests_sent)) return 182;
      if(complete_order_mode==7U &&
         (transport.statistics().commands_published!=baseline.commands_published ||
          !wait_for([&] { return transport.statistics().stop_requests_sent>
            baseline.stop_requests_sent; }))) return 194;
      if(complete_order_mode==10U && !complete_query()) return 186;
      transport.stop();
      return 0;
    }
    if(transport.statistics().invalidations!=baseline.invalidations ||
       transport.statistics().heartbeat_stops!=baseline.heartbeat_stops) return 176;
    transport.stop();
    return 0;
  }
  if (raw_capture_mode) {
    if (raw_order_case != 0U) {
      auto publish_page = [&](std::uint64_t run, unsigned sequence, unsigned category,
                              unsigned group_count, unsigned final_state,
                              const std::array<std::uint8_t, 32U> &previous,
                              std::array<std::uint8_t, 32U> *next,
                              unsigned kind = 1U, std::uint64_t parent = 0U,
                              bool deliver = true) {
        auto stimulus = valid;
        stimulus.send_status = false;
        stimulus.send_state = false;
        stimulus.send_evidence = true;
        auto &message = stimulus.evidence;
        message.set_schema_major(2U);
        message.set_schema_minor(4U);
        message.set_run_id(run);
        message.set_attempt_id(run);
        message.set_evidence_seq(sequence);
        message.set_attempt_kind(run == 2U &&
            (raw_order_case == 4U || (raw_order_case == 11U && sequence == 1U)) ? 2U : kind);
        message.set_parent_run_id(parent);
        message.set_motion_epoch(raw_order_case >= 43U && raw_order_case <= 50U ?
            (run == 1U ? 1U : run == 2U && sequence == 0U ? 3U : 5U) :
            raw_order_case >= 34U && raw_order_case <= 42U ?
            (run == 1U ? 1U : 3U) : 10U);
        if (run == 2U && sequence == 0U && raw_order_case == 38U)
          message.set_motion_epoch(0U);
        if (run == 124U && sequence == 0U && raw_order_case == 31U)
          message.set_motion_epoch(0U);
        message.set_category(category);
        message.set_final(1U);
        message.set_group_count(group_count);
        if (category == 4U) {
          message.set_final_state(final_state);
          if (((run == 1U && raw_order_case >= 34U && raw_order_case <= 50U &&
                raw_order_case != 41U) ||
               (run >= 2U && raw_order_case >= 43U && raw_order_case <= 50U)))
            message.set_motion_complete(true);
        }
        else {
          message.set_group_id(run);
          message.set_expected_mask(63U);
          message.set_submitted_mask(63U);
          message.set_sent_mask(63U);
          if ((run == 124U && sequence == 0U && raw_order_case == 30U) ||
              (run == 2U && sequence == 0U && raw_order_case == 40U))
            message.set_sent_mask(31U);
        }
        std::uint8_t previous_digest[32]{};
        std::copy(previous.begin(), previous.end(), previous_digest);
        if (run == 2U && (raw_order_case == 5U ||
                          (raw_order_case == 12U && sequence == 1U))) previous_digest[0] ^= 1U;
        message.set_previous_chain_sha256(previous_digest);
        std::array<std::uint8_t, wire::DummyAttemptEvidenceV2Codec::kEncodedSize> raw{};
        if (wire::DummyAttemptEvidenceV2Codec::encode(
                message, raw.data(), raw.size()) != wire::CodecResult::kOk) return false;
        std::array<std::uint8_t, 32U + wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize + 1U> input{};
        std::copy(previous.begin(), previous.end(), input.begin());
        std::copy_n(raw.begin(), wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize,
                    input.begin() + 32U);
        input.back() = raw[wire::DummyAttemptEvidenceV2Codec::kEvidenceLostOffset];
        ::SHA256(input.data(), input.size(), next->data());
        std::uint8_t next_digest[32]{};
        std::copy(next->begin(), next->end(), next_digest);
        message.set_chain_sha256(next_digest);
        return !deliver || send(stimulus);
      };
      auto seed_for = [&](std::uint64_t run) {
        std::array<std::uint8_t, 192U> seed{};
        for (unsigned i = 0U; i < 8U; ++i)
          seed[i] = static_cast<std::uint8_t>(run >> (8U * i));
        seed[8] = 7U;
        std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), seed.begin() + 16U);
        std::copy(parameters.contract_hash.begin(), parameters.contract_hash.end(), seed.begin() + 32U);
        std::copy(parameters.model_hash.begin(), parameters.model_hash.end(), seed.begin() + 64U);
        std::copy(parameters.profile_hash.begin(), parameters.profile_hash.end(), seed.begin() + 96U);
        std::copy(parameters.firmware_hash.begin(), parameters.firmware_hash.end(), seed.begin() + 128U);
        std::array<std::uint8_t, 32U> previous{};
        ::SHA256(seed.data(), seed.size(), previous.data());
        return previous;
      };
      if (raw_order_case >= 43U && raw_order_case <= 50U) {
        valid.status.set_core_state(3U);
        valid.status.set_final_state(0U);
        valid.status.set_motion_epoch(3U);
        valid.status.set_management_epoch(7U);
        valid.state.set_motion_epoch(3U);
        valid.state.set_management_epoch(7U);
        std::array<std::uint8_t, 32U> first_page{}, first_terminal{};
        std::array<std::uint8_t, 32U> second_page{}, second_terminal{}, third_page{};
        if (!send(valid) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 1U;
            }) || !publish_page(1U, 0U, 1U, 1U, 0U, seed_for(1U), &first_page) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 1U; }) ||
            !publish_page(1U, 1U, 4U, 1U, 6U, first_page, &first_terminal) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 1U; }))
          return 323;
        const auto stale_receipt = channel->raw_evidence_terminal_flushed();
        if (!publish_page(2U, 0U, 1U, 1U, 0U, seed_for(2U), &second_page) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 3U ||
                transport.statistics().raw_evidence_failure != 0U; }) ||
            transport.statistics().raw_evidence_failure != 0U) return 324;
        auto second_status = valid;
        second_status.send_state = false;
        second_status.status.set_run_id(2U);
        second_status.status.set_motion_epoch(5U);
        second_status.status.set_management_epoch(8U);
        if (!send(second_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 2U &&
                  observed.core_state() == 3U && observed.final_state() == 0U &&
                  observed.motion_epoch() == 5U;
            })) return 325;
        if (raw_order_case != 48U &&
            (!publish_page(2U, 1U, 4U, 1U, 6U, second_page, &second_terminal) ||
             !wait_for([&] {
               const auto receipt = channel->raw_evidence_terminal_flushed();
               return transport.statistics().raw_evidence_last_terminal_written_run == 2U &&
                   receipt.run_id == 2U && receipt.final_state == 6U &&
                   receipt.chain_valid && receipt.motion_complete;
             }))) return 326;
        {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          if (wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) != wire::CodecResult::kOk || observed.run_id() != 2U ||
              observed.final_state() != 0U ||
              std::any_of(std::begin(observed.evidence_chain_sha256()),
                  std::end(observed.evidence_chain_sha256()),
                  [](std::uint8_t byte) { return byte != 0U; })) return 327;
        }
        if (raw_order_case >= 44U && raw_order_case <= 47U) {
          auto receipt = raw_order_case == 44U ? stale_receipt :
              channel->raw_evidence_terminal_flushed();
          if (raw_order_case == 45U) receipt.boot_epoch = 8U;
          if (raw_order_case == 46U) receipt.session_id = 9U;
          if (raw_order_case == 47U) receipt.run_id = 3U;
          channel->publish_raw_evidence_terminal_flushed(receipt);
        }
        const auto before_third = transport.statistics();
        if (!publish_page(3U, 0U, 1U, 1U, 0U, seed_for(3U), &third_page) ||
            !wait_for([&] {
              const auto stats = transport.statistics();
              return stats.raw_evidence_pages_written > before_third.raw_evidence_pages_written ||
                  stats.raw_evidence_failure != 0U;
            })) return 328;
        if (raw_order_case >= 49U) {
          auto late_status = second_status;
          late_status.status.set_core_state(4U);
          late_status.status.set_final_state(6U);
          late_status.status.set_motion_complete(true);
          std::uint8_t late_digest[32]{};
          std::copy(second_terminal.begin(), second_terminal.end(), late_digest);
          if (raw_order_case == 49U) late_digest[0] ^= 1U;
          late_status.status.set_evidence_chain_sha256(late_digest);
          const auto before_late = transport.statistics().status_messages;
          if (!send(late_status) || !wait_for([&] {
                const auto stats = transport.statistics();
                return stats.status_messages > before_late ||
                    stats.raw_evidence_failure != 0U;
              })) return 335;
          if (raw_order_case == 49U) {
            const auto rejected = transport.statistics();
            transport.stop();
            const auto stopped = transport.statistics();
            return rejected.raw_evidence_failure == 6U &&
                stopped.raw_evidence_failure == 6U &&
                stopped.raw_evidence_pages_written == 5U ? 0 : 336;
          }
          if (!wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed;
                return wire::DummyDeviceStatusV2Codec::decode(
                    channel->observation.status.data(), channel->observation.status.size(),
                    &observed) == wire::CodecResult::kOk && observed.run_id() == 2U &&
                    observed.final_state() == 6U &&
                    std::equal(second_terminal.begin(), second_terminal.end(),
                               observed.evidence_chain_sha256());
              }) || transport.statistics().raw_evidence_failure != 0U) return 337;
        }
        if (raw_order_case >= 44U && raw_order_case <= 48U) {
          transport.stop();
          const auto result = transport.statistics();
          return result.raw_evidence_failure == 6U &&
              result.raw_evidence_pages_written == before_third.raw_evidence_pages_written ? 0 : 329;
        }
        if (transport.statistics().raw_evidence_failure != 0U ||
            transport.statistics().raw_evidence_pages_written != 5U) return 330;
        auto third_status = second_status;
        third_status.status.set_run_id(3U);
        third_status.status.set_core_state(4U);
        third_status.status.set_management_epoch(9U);
        if (!send(third_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 3U;
            })) return 331;
        std::array<std::uint8_t, 32U> third_terminal{};
        if (!publish_page(3U, 1U, 4U, 1U, 6U, third_page, &third_terminal) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 3U; }))
          return 332;
        auto third_final = third_status;
        third_final.status.set_final_state(6U);
        third_final.status.set_motion_complete(true);
        std::uint8_t third_digest[32]{};
        std::copy(third_terminal.begin(), third_terminal.end(), third_digest);
        third_final.status.set_evidence_chain_sha256(third_digest);
        if (!send(third_final) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 3U &&
                  observed.final_state() == 6U &&
                  std::equal(third_terminal.begin(), third_terminal.end(),
                             observed.evidence_chain_sha256());
            })) return 333;
        transport.stop();
        const auto result = transport.statistics();
        return result.raw_evidence_failure == 0U &&
            result.raw_evidence_pages_written == 6U &&
            result.raw_evidence_last_terminal_written_run == 3U ? 0 : 334;
      }
      if (raw_order_case >= 34U && raw_order_case <= 42U) {
        valid.status.set_core_state(3U);
        valid.status.set_final_state(0U);
        valid.status.set_motion_epoch(3U);
        valid.status.set_management_epoch(7U);
        valid.state.set_motion_epoch(3U);
        valid.state.set_management_epoch(7U);
        std::array<std::uint8_t, 32U> first_group{}, first_terminal{}, next_group{}, next_terminal{};
        if (!send(valid) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 1U &&
                  observed.core_state() == 3U && observed.final_state() == 0U &&
                  observed.motion_epoch() == 3U && observed.management_epoch() == 7U;
            }) || !publish_page(1U, 0U, 1U, 1U, 0U, seed_for(1U), &first_group) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 1U; }))
          return 307;
        if (raw_order_case != 37U &&
            (!publish_page(1U, 1U, 4U, 1U, 6U, first_group, &first_terminal) ||
             !wait_for([&] {
               const auto receipt = channel->raw_evidence_terminal_flushed();
               return transport.statistics().raw_evidence_last_terminal_written_run == 1U &&
                   receipt.boot_epoch == 7U && receipt.session_id == 8U &&
                   receipt.run_id == 1U && receipt.attempt_kind == 1U &&
                   receipt.final_state == 6U && receipt.chain_valid &&
                   receipt.motion_complete == (raw_order_case != 41U);
             }))) return 308;
        if (raw_order_case == 35U || raw_order_case == 36U || raw_order_case == 42U) {
          // Isolate receipt identity and publication order after the real terminal flush.
          auto receipt = channel->raw_evidence_terminal_flushed();
          if (raw_order_case == 35U) receipt.boot_epoch = 8U;
          if (raw_order_case == 36U) receipt.session_id = 9U;
          if (raw_order_case == 42U) receipt = {};
          channel->publish_raw_evidence_terminal_flushed(receipt);
        }
        {
          const auto before = transport.statistics();
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          if (wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) != wire::CodecResult::kOk || observed.run_id() != 1U ||
              observed.core_state() != 3U || observed.final_state() != 0U ||
              observed.motion_epoch() != 3U || observed.management_epoch() != 7U ||
              before.raw_evidence_last_terminal_written_run !=
                  (raw_order_case == 37U ? 0U : 1U)) return 316;
        }
        const auto before_page = transport.statistics();
        if (!publish_page(raw_order_case == 39U ? 3U : 2U, 0U, 1U, 1U, 0U,
                          seed_for(raw_order_case == 39U ? 3U : 2U), &next_group) ||
            !wait_for([&] {
              const auto observed = transport.statistics();
              return observed.raw_evidence_pages_written > before_page.raw_evidence_pages_written ||
                  observed.raw_evidence_failure != 0U;
            })) return 310;
        const auto first_page = transport.statistics();
        if (raw_order_case != 34U) {
          transport.stop();
          return first_page.raw_evidence_failure == 6U &&
              first_page.raw_evidence_pages_written == before_page.raw_evidence_pages_written ? 0 : 311;
        }
        if (first_page.raw_evidence_failure != 0U ||
            first_page.raw_evidence_pages_written != 3U ||
            first_page.status_messages != before_page.status_messages ||
            first_page.raw_evidence_last_terminal_written_run != 1U) {
          std::fprintf(stderr, "hold refresh first page: failure=%u pages=%llu status_delta=%llu terminal=%llu\n",
              static_cast<unsigned>(first_page.raw_evidence_failure),
              static_cast<unsigned long long>(first_page.raw_evidence_pages_written),
              static_cast<unsigned long long>(first_page.status_messages - before_page.status_messages),
              static_cast<unsigned long long>(first_page.raw_evidence_last_terminal_written_run));
          return 312;
        }
        auto next_status = valid;
        next_status.send_state = false;
        next_status.status.set_run_id(2U);
        next_status.status.set_core_state(4U);
        if (!send(next_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 2U;
            }) || !publish_page(2U, 1U, 4U, 1U, 6U, next_group, &next_terminal) ||
            !wait_for([&] {
              return transport.statistics().raw_evidence_last_terminal_written_run == 2U;
            })) return 313;
        auto final_status = next_status;
        final_status.status.set_final_state(6U);
        std::uint8_t digest[32]{};
        std::copy(next_terminal.begin(), next_terminal.end(), digest);
        final_status.status.set_evidence_chain_sha256(digest);
        if (!send(final_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 2U &&
                  observed.final_state() == 6U;
            })) return 314;
        transport.stop();
        return transport.statistics().raw_evidence_failure == 0U &&
            transport.statistics().raw_evidence_pages_written == 4U ? 0 : 315;
      }
      if (raw_order_case >= 20U && raw_order_case <= 33U) {
        const bool refresh = raw_order_case >= 24U;
        const auto motion_run = refresh ? 123U : 121U;
        const auto next_run = refresh ? 124U : 122U;
        std::array<std::uint8_t, 32U> motion_group{}, motion_terminal{};
        std::array<std::uint8_t, 32U> release_page{}, release_terminal{};
        if (refresh) valid.status.set_core_state(7U);
        if (!send(valid) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == motion_run &&
                  (!refresh || (observed.core_state() == 7U && observed.final_state() == 0U &&
                    std::all_of(std::begin(observed.evidence_chain_sha256()),
                        std::end(observed.evidence_chain_sha256()),
                        [](auto byte) { return byte == 0U; })));
            }) || !publish_page(motion_run, 0U, 2U, 1U, 0U, seed_for(motion_run), &motion_group) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 1U; }) ||
            !publish_page(motion_run, 1U, 4U, 1U, 7U, motion_group, &motion_terminal) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == motion_run; }))
          return 288;
        auto motion_final = valid;
        motion_final.send_state = false;
        motion_final.status.set_core_state(4U);
        motion_final.status.set_final_state(7U);
        std::uint8_t motion_digest[32]{};
        std::copy(motion_terminal.begin(), motion_terminal.end(), motion_digest);
        motion_final.status.set_evidence_chain_sha256(motion_digest);
        if (!refresh && (!send(motion_final) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == motion_run &&
                  observed.final_state() == 7U;
            }))) return 289;
        const auto statuses_before_page = transport.statistics().status_messages;
        const auto release_run = raw_order_case == 23U ? 120U :
            raw_order_case == 27U ? 122U : next_run;
        const auto release_parent = raw_order_case == 21U ? 120U :
            raw_order_case == 23U ? 119U : refresh ? 0U : 121U;
        if (!publish_page(release_run, 0U,
                          raw_order_case == 28U ? 2U : refresh ? 1U : 3U,
                          raw_order_case == 29U ? 2U : 1U, 0U,
                          seed_for(release_run), &release_page,
                          refresh ? 1U : 3U, release_parent) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 3U ||
                transport.statistics().raw_evidence_failure != 0U; })) return 290;
        const auto page_first = transport.statistics();
        if ((raw_order_case >= 20U && raw_order_case <= 23U) || raw_order_case == 27U) {
          transport.stop();
          return page_first.raw_evidence_failure == 6U &&
              page_first.raw_evidence_pages_written == 2U ? 0 : 296;
        }
        if (raw_order_case >= 28U && raw_order_case <= 31U) {
          std::fprintf(stderr, "invalid refresh first page: case=%u failure=%u pages=%llu\n",
              raw_order_case, static_cast<unsigned>(page_first.raw_evidence_failure),
              static_cast<unsigned long long>(page_first.raw_evidence_pages_written));
          transport.stop();
          return page_first.raw_evidence_failure == 6U &&
              page_first.raw_evidence_pages_written == 2U ? 0 : 299;
        }
        if (page_first.raw_evidence_failure != 0U ||
            page_first.raw_evidence_pages_written != 3U ||
            page_first.status_messages != statuses_before_page) {
          std::fprintf(stderr, "page-first: failure=%u pages=%llu status_delta=%llu\n",
              static_cast<unsigned>(page_first.raw_evidence_failure),
              static_cast<unsigned long long>(page_first.raw_evidence_pages_written),
              static_cast<unsigned long long>(page_first.status_messages - statuses_before_page));
          return 291;
        }
        auto release_status = valid;
        release_status.send_state = false;
        release_status.status.set_run_id(next_run);
        release_status.status.set_parent_run_id(refresh ? 0U : 121U);
        release_status.status.set_attempt_kind(refresh ? 1U : 3U);
        release_status.status.set_core_state(refresh ? 4U : 1U);
        release_status.status.set_session_id(refresh ? 8U : 0U);
        release_status.status.set_reference_epoch(refresh ? 9U : 0U);
        release_status.status.set_motion_epoch(refresh ? 10U : 11U);
        release_status.status.set_management_epoch(refresh ? 0U : 1U);
        release_status.status.set_release_committed(!refresh);
        release_status.status.set_permit_remaining_us(0U);
        release_status.status.set_final_state(refresh ? 0U : 8U);
        if (raw_order_case == 25U || raw_order_case == 26U) {
          if (raw_order_case == 26U) release_status.status.set_session_id(9U);
          else release_status.status.set_boot_epoch(8U);
          const auto statuses_before_bad_identity = transport.statistics().status_messages;
          if (!send(release_status) || !wait_for([&] {
                return transport.statistics().status_messages > statuses_before_bad_identity;
              })) return 297;
          if (raw_order_case == 25U && !wait_for([&] {
                return transport.statistics().raw_evidence_failure != 0U;
              })) return 297;
          transport.stop();
          const auto rejected = transport.statistics();
          return rejected.raw_evidence_last_terminal_written_run == motion_run &&
              (raw_order_case == 25U ? rejected.raw_evidence_failure == 6U :
                  rejected.raw_evidence_failure == 7U) ? 0 : 298;
        }
        if (!send(release_status) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == next_run &&
                  observed.parent_run_id() == (refresh ? 0U : 121U) &&
                  observed.attempt_kind() == (refresh ? 1U : 3U) &&
                  observed.core_state() == (refresh ? 4U : 1U) &&
                  observed.release_committed() == !refresh;
            }) || transport.statistics().raw_evidence_failure != 0U ||
            transport.statistics().raw_evidence_last_terminal_written_run != motion_run)
          return 292;
        if (raw_order_case == 33U) {
          if (!publish_page(next_run, 1U, 4U, 1U, 6U, release_page,
                            &release_terminal, 1U, 0U, false)) return 302;
          auto early_final = release_status;
          early_final.status.set_final_state(6U);
          early_final.status.set_motion_complete(true);
          std::uint8_t digest[32]{};
          std::copy(release_terminal.begin(), release_terminal.end(), digest);
          early_final.status.set_evidence_chain_sha256(digest);
          if (!send(early_final) || !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed;
                return wire::DummyDeviceStatusV2Codec::decode(
                    channel->observation.status.data(), channel->observation.status.size(),
                    &observed) == wire::CodecResult::kOk && observed.run_id() == next_run &&
                    observed.final_state() == 6U;
              })) return 303;
          if (!publish_page(next_run, 1U, 4U, 1U, 6U, release_page,
                            &release_terminal, 1U, 0U) ||
              !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == next_run; }))
            return 304;
          auto following = release_status;
          following.status.set_run_id(next_run + 1U);
          following.status.set_motion_epoch(11U);
          const auto before_following = transport.statistics().status_messages;
          if (!send(following) || !wait_for([&] {
                const auto stats = transport.statistics();
                return stats.status_messages > before_following || stats.raw_evidence_failure != 0U;
              })) return 305;
          const auto observed = transport.statistics();
          transport.stop();
          return observed.raw_evidence_failure == 0U &&
              observed.raw_evidence_last_terminal_written_run == next_run &&
              observed.status_messages > before_following ? 0 : 306;
        }
        if (!publish_page(next_run, 1U, 4U, 1U, refresh ? 6U : 8U,
                          release_page, &release_terminal,
                          refresh ? 1U : 3U, refresh ? 0U : 121U) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == next_run; }))
          return 293;
        auto release_final = release_status;
        if (refresh) release_final.status.set_final_state(6U);
        std::uint8_t release_digest[32]{};
        std::copy(release_terminal.begin(), release_terminal.end(), release_digest);
        if (raw_order_case == 32U) release_digest[0] ^= 1U;
        release_final.status.set_evidence_chain_sha256(release_digest);
        if (raw_order_case == 32U) {
          if (!send(release_final) || !wait_for([&] {
                return transport.statistics().raw_evidence_failure != 0U;
              })) {
            const auto unchecked = transport.statistics();
            std::fprintf(stderr, "wrong refresh final chain: failure=%u pages=%llu terminal=%llu\n",
                static_cast<unsigned>(unchecked.raw_evidence_failure),
                static_cast<unsigned long long>(unchecked.raw_evidence_pages_written),
                static_cast<unsigned long long>(unchecked.raw_evidence_last_terminal_written_run));
            return 300;
          }
          transport.stop();
          return transport.statistics().raw_evidence_failure == 6U ? 0 : 301;
        }
        if (!send(release_final) || !wait_for([&] {
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == next_run &&
                  observed.final_state() == (refresh ? 6U : 8U) &&
                  observed.core_state() == (refresh ? 4U : 1U) &&
                  observed.release_committed() == !refresh &&
                  std::equal(release_terminal.begin(), release_terminal.end(),
                             observed.evidence_chain_sha256());
            })) return 294;
        transport.stop();
        const auto capture = transport.statistics();
        return capture.raw_evidence_failure == 0U &&
            channel->raw_evidence_failure.load(std::memory_order_acquire) == 0U &&
            capture.raw_evidence_pages_written == 4U ? 0 : 295;
      }
      std::array<std::uint8_t, 32U> group_one{}, terminal_one{}, group_two{}, terminal_two{};
      if (!send(valid) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed;
            return wire::DummyDeviceStatusV2Codec::decode(
                channel->observation.status.data(), channel->observation.status.size(),
                &observed) == wire::CodecResult::kOk && observed.run_id() == 1U;
          }) || !publish_page(1U, 0U, 2U, 1U, 0U, seed_for(1U), &group_one) ||
          !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 1U; }))
        return 251;
      if (raw_order_case == 6U) {
        if (!publish_page(2U, 0U, 2U, 1U, 0U, seed_for(2U), &group_two) ||
            !wait_for([&] { return transport.statistics().raw_evidence_failure != 0U; })) return 260;
        transport.stop();
        return transport.statistics().raw_evidence_pages_written == 1U ? 0 : 261;
      }
      if (!publish_page(1U, 1U, 4U, 1U, 6U, group_one, &terminal_one) ||
          !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 1U; }))
        return 251;
      auto exported = valid;
      exported.send_state = false;
      exported.status.set_core_state(4U);
      exported.status.set_final_state(6U);
      exported.status.set_motion_complete(true);
      std::uint8_t terminal_digest[32]{};
      std::copy(terminal_one.begin(), terminal_one.end(), terminal_digest);
      exported.status.set_evidence_chain_sha256(terminal_digest);
      if (!send(exported) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed;
            return wire::DummyDeviceStatusV2Codec::decode(
                channel->observation.status.data(), channel->observation.status.size(),
                &observed) == wire::CodecResult::kOk && observed.run_id() == 1U &&
                observed.final_state() == 6U && observed.motion_complete() &&
                std::equal(terminal_one.begin(), terminal_one.end(), observed.evidence_chain_sha256());
          })) return 252;
      if (!publish_page(2U, 0U, 1U, 1U, 0U, seed_for(2U), &group_two) ||
          !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 3U ||
              transport.statistics().raw_evidence_failure != 0U; })) return 253;
      if (raw_order_case == 4U || raw_order_case == 5U) {
        if (!wait_for([&] { return transport.statistics().raw_evidence_failure != 0U; })) return 262;
        transport.stop();
        return transport.statistics().raw_evidence_pages_written == 2U ? 0 : 263;
      }
      if (transport.statistics().raw_evidence_failure != 0U) return 254;
      if (raw_order_case >= 9U && raw_order_case <= 14U) {
        auto released_state = valid;
        released_state.send_status = false;
        released_state.state.set_session_id(0U);
        released_state.state.set_reference_epoch(0U);
        released_state.state.set_motion_epoch(11U);
        released_state.state.set_management_epoch(1U);
        released_state.state.set_core_state(1U);
        released_state.state.set_position_valid_mask(0U);
        released_state.state.set_raw_position_valid_mask(63U);
        released_state.state.set_state_seq(2U);
        const double raw_motor_rev[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
        released_state.state.set_raw_motor_position_rev(raw_motor_rev);
        const auto state_count = transport.statistics().state_messages;
        if (!send(released_state) || !wait_for([&] {
              return transport.statistics().state_messages > state_count;
            })) return 270;
        auto released_status = valid;
        released_status.send_state = false;
        released_status.status.set_run_id(2U);
        released_status.status.set_session_id(0U);
        released_status.status.set_reference_epoch(0U);
        released_status.status.set_motion_epoch(11U);
        released_status.status.set_management_epoch(1U);
        released_status.status.set_core_state(1U);
        released_status.status.set_release_committed(true);
        released_status.status.set_permit_remaining_us(0U);
        released_status.status.set_final_state(2U);
        if (raw_order_case == 10U) released_status.status.set_boot_epoch(8U);
        if (!send(released_status) || !wait_for([&] {
              if (transport.statistics().raw_evidence_failure != 0U) return true;
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 2U &&
                  observed.session_id() == 0U;
            })) return 271;
        if (raw_order_case == 10U) {
          transport.stop();
          return transport.statistics().raw_evidence_failure != 0U ? 0 : 272;
        }
        if (transport.statistics().raw_evidence_failure != 0U) return 273;
        std::array<std::uint8_t, 32U> disable_chain{};
        if (!publish_page(2U, 1U, 3U, 2U, 0U, group_two, &disable_chain) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written == 4U ||
                transport.statistics().raw_evidence_failure != 0U; })) return 274;
        if (raw_order_case == 11U || raw_order_case == 12U) {
          transport.stop();
          return transport.statistics().raw_evidence_failure != 0U &&
              transport.statistics().raw_evidence_pages_written == 3U ? 0 : 275;
        }
        if (transport.statistics().raw_evidence_failure != 0U ||
            !publish_page(2U, 2U, 4U, 2U, 2U, disable_chain, &terminal_two) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 2U; }))
          return 276;
        if (raw_order_case == 13U) {
          transport.stop();
          return transport.statistics().raw_evidence_failure != 0U ? 0 : 277;
        }
        auto final_status = released_status;
        final_status.status.set_management_epoch(2U);
        std::uint8_t final_digest[32]{};
        std::copy(terminal_two.begin(), terminal_two.end(), final_digest);
        if (raw_order_case == 14U) final_digest[0] ^= 1U;
        final_status.status.set_evidence_chain_sha256(final_digest);
        if (raw_order_case == 14U) {
          if (!send(final_status) || !wait_for([&] {
                return transport.statistics().raw_evidence_failure != 0U;
              })) return 283;
          transport.stop();
          return transport.statistics().raw_evidence_pages_written == 5U ? 0 : 284;
        }
        if (!send(final_status) || !wait_for([&] {
              if (transport.statistics().raw_evidence_failure != 0U) return true;
              std::lock_guard<std::mutex> lock(channel->mutex);
              Status observed;
              return wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) == wire::CodecResult::kOk && observed.run_id() == 2U &&
                  observed.session_id() == 0U && observed.final_state() == 2U &&
                  std::equal(terminal_two.begin(), terminal_two.end(),
                             observed.evidence_chain_sha256());
            })) return 278;
        transport.stop();
        const auto capture = transport.statistics();
        if (capture.raw_evidence_failure != 0U || capture.raw_evidence_pages_written != 5U)
          return 279;
        std::FILE *file = std::fopen(capture_path.c_str(), "rb");
        if (!file) return 280;
        if (::fseek(file, 8L, SEEK_SET) != 0) { std::fclose(file); return 281; }
        std::uint8_t header[39]{};
        int index = 0, page_indices[3]{-1, -1, -1}, released_index = -1, final_index = -1;
        bool old_identity = true;
        while (std::fread(header, 1U, sizeof(header), file) == sizeof(header)) {
          const auto size = static_cast<std::uint16_t>(header[1] | (header[2] << 8U));
          std::array<std::uint8_t, wire::DummyDeviceStatusV2Codec::kEncodedSize> payload{};
          if (size > payload.size() || std::fread(payload.data(), 1U, size, file) != size) break;
          if (header[0] == 1U && size == wire::DummyAttemptEvidenceV2Codec::kEncodedSize) {
            Evidence observed;
            if (wire::DummyAttemptEvidenceV2Codec::decode(payload.data(), size, &observed) ==
                    wire::CodecResult::kOk && observed.run_id() == 2U && observed.evidence_seq() < 3U) {
              page_indices[observed.evidence_seq()] = index;
              old_identity = old_identity && header[11] == 7U && header[19] == 8U;
            }
          } else if (header[0] == 3U && size == wire::DummyDeviceStatusV2Codec::kEncodedSize) {
            Status observed;
            if (wire::DummyDeviceStatusV2Codec::decode(payload.data(), size, &observed) ==
                    wire::CodecResult::kOk && observed.run_id() == 2U) {
              const bool chain_present = std::any_of(
                  std::begin(observed.evidence_chain_sha256()),
                  std::end(observed.evidence_chain_sha256()),
                  [](std::uint8_t byte) { return byte != 0U; });
              if (chain_present) final_index = index;
              else released_index = index;
            }
          }
          ++index;
        }
        std::fclose(file);
        return old_identity && page_indices[0] >= 0 &&
            page_indices[0] < released_index && released_index < page_indices[1] &&
            page_indices[1] < page_indices[2] && page_indices[2] < final_index ? 0 : 282;
      }
      if (raw_order_case == 8U) {
        if (!publish_page(2U, 0U, 1U, 1U, 0U, seed_for(2U), &group_two) ||
            !wait_for([&] { return transport.statistics().raw_evidence_failure != 0U; })) return 266;
        transport.stop();
        return transport.statistics().raw_evidence_pages_written == 3U ? 0 : 267;
      }
      if (raw_order_case == 7U) {
        if (!publish_page(2U, 1U, 4U, 1U, 6U, group_two, &terminal_two) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 2U; }))
          return 268;
        transport.stop();
        return transport.statistics().raw_evidence_failure != 0U ? 0 : 269;
      }
      auto status_two = valid;
      status_two.send_state = false;
      status_two.status.set_run_id(2U);
      status_two.status.set_management_epoch(1U);
      if (raw_order_case == 2U) status_two.status.set_boot_epoch(8U);
      if (raw_order_case == 3U) status_two.status.set_session_id(9U);
      if (raw_order_case == 2U || raw_order_case == 3U) {
        if (!send(status_two) || !wait_for([&] {
              return transport.statistics().raw_evidence_failure != 0U;
            })) return 264;
        transport.stop();
        return transport.statistics().raw_evidence_pages_written == 3U ? 0 : 265;
      }
      if (!send(status_two) || !wait_for([&] {
            std::lock_guard<std::mutex> lock(channel->mutex);
            Status observed;
            return wire::DummyDeviceStatusV2Codec::decode(
                channel->observation.status.data(), channel->observation.status.size(),
                &observed) == wire::CodecResult::kOk && observed.run_id() == 2U;
          }) || !publish_page(2U, 1U, 4U, 1U, 6U, group_two, &terminal_two) ||
          !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == 2U; }))
        return 255;
      transport.stop();
      const auto capture = transport.statistics();
      if (capture.raw_evidence_failure != 0U ||
          channel->raw_evidence_failure.load(std::memory_order_acquire) != 0U ||
          capture.raw_evidence_pages_written != 4U) return 256;
      std::FILE *file = std::fopen(capture_path.c_str(), "rb");
      if (!file) return 257;
      if (::fseek(file, 8L, SEEK_SET) != 0) { std::fclose(file); return 258; }
      std::uint8_t header[39]{};
      int index = 0, page_two_index = -1, status_two_index = -1;
      while (std::fread(header, 1U, sizeof(header), file) == sizeof(header)) {
        const auto size = static_cast<std::uint16_t>(header[1] | (header[2] << 8U));
        std::array<std::uint8_t, wire::DummyDeviceStatusV2Codec::kEncodedSize> payload{};
        if (size > payload.size() || std::fread(payload.data(), 1U, size, file) != size) break;
        if (header[0] == 1U && size == wire::DummyAttemptEvidenceV2Codec::kEncodedSize) {
          Evidence observed;
          if (wire::DummyAttemptEvidenceV2Codec::decode(payload.data(), size, &observed) ==
                  wire::CodecResult::kOk && observed.run_id() == 2U && observed.evidence_seq() == 0U)
            page_two_index = index;
        } else if (header[0] == 3U && size == wire::DummyDeviceStatusV2Codec::kEncodedSize) {
          Status observed;
          if (wire::DummyDeviceStatusV2Codec::decode(payload.data(), size, &observed) ==
                  wire::CodecResult::kOk && observed.run_id() == 2U)
            status_two_index = index;
        }
        ++index;
      }
      std::fclose(file);
      return page_two_index >= 0 && status_two_index > page_two_index ? 0 : 259;
    }
    if (raw_many_mode) {
      auto publish_page = [&](std::uint64_t run, bool terminal,
                              const std::array<std::uint8_t, 32U> &previous,
                              std::array<std::uint8_t, 32U> *next) {
        auto stimulus = valid;
        stimulus.send_status = false;
        stimulus.send_state = false;
        stimulus.send_evidence = true;
        auto &message = stimulus.evidence;
        message.set_schema_major(2U);
        message.set_schema_minor(4U);
        message.set_run_id(run);
        message.set_attempt_id(run);
        message.set_evidence_seq(terminal ? 1U : 0U);
        message.set_attempt_kind(1U);
        message.set_motion_epoch(10U);
        message.set_category(terminal ? 4U : 2U);
        message.set_final(1U);
        message.set_group_count(1U);
        if (terminal) message.set_final_state(3U);
        else {
          message.set_group_id(run);
          message.set_expected_mask(63U);
          message.set_submitted_mask(63U);
          message.set_sent_mask(63U);
        }
        std::uint8_t prior[32]{};
        std::copy(previous.begin(), previous.end(), prior);
        message.set_previous_chain_sha256(prior);
        std::array<std::uint8_t, wire::DummyAttemptEvidenceV2Codec::kEncodedSize> raw{};
        if (wire::DummyAttemptEvidenceV2Codec::encode(
                message, raw.data(), raw.size()) != wire::CodecResult::kOk) return false;
        std::array<std::uint8_t, 32U + wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize + 1U> input{};
        std::copy(previous.begin(), previous.end(), input.begin());
        std::copy_n(raw.begin(), wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize,
                    input.begin() + 32U);
        input.back() = raw[wire::DummyAttemptEvidenceV2Codec::kEvidenceLostOffset];
        ::SHA256(input.data(), input.size(), next->data());
        std::uint8_t digest[32]{};
        std::copy(next->begin(), next->end(), digest);
        message.set_chain_sha256(digest);
        return send(stimulus);
      };
      for (std::uint64_t run = 3U; run <= 11U; ++run) {
        if (run != 3U) {
          auto status_only = valid;
          status_only.send_state = false;
          status_only.status.set_run_id(run);
          status_only.status.set_management_epoch(static_cast<std::uint32_t>(run - 3U));
          if (!send(status_only) || !wait_for([&] {
                std::lock_guard<std::mutex> lock(channel->mutex);
                Status observed;
                return wire::DummyDeviceStatusV2Codec::decode(
                    channel->observation.status.data(), channel->observation.status.size(),
                    &observed) == wire::CodecResult::kOk && observed.run_id() == run;
              })) return 145;
        }
        std::array<std::uint8_t, 192U> seed{};
        for (unsigned i = 0U; i < 8U; ++i) seed[i] = static_cast<std::uint8_t>(run >> (8U * i));
        seed[8] = 7U;
        std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), seed.begin() + 16U);
        std::copy(parameters.contract_hash.begin(), parameters.contract_hash.end(), seed.begin() + 32U);
        std::copy(parameters.model_hash.begin(), parameters.model_hash.end(), seed.begin() + 64U);
        std::copy(parameters.profile_hash.begin(), parameters.profile_hash.end(), seed.begin() + 96U);
        std::copy(parameters.firmware_hash.begin(), parameters.firmware_hash.end(), seed.begin() + 128U);
        std::array<std::uint8_t, 32U> previous{}, group_chain{}, terminal_chain{};
        ::SHA256(seed.data(), seed.size(), previous.data());
        if (!publish_page(run, false, previous, &group_chain) ||
            !wait_for([&] { return transport.statistics().raw_evidence_pages_written >=
                (run - 3U) * 2U + 1U; }) ||
            !publish_page(run, true, group_chain, &terminal_chain) ||
            !wait_for([&] { return transport.statistics().raw_evidence_last_terminal_written_run == run; }))
          return 146;
        if (run == 4U || run == 11U) {
          auto stale_status = valid;
          stale_status.send_state = false;
          stale_status.status.set_run_id(3U);
          stale_status.status.set_management_epoch(static_cast<std::uint32_t>(run - 3U));
          const auto rejected_before = transport.statistics().stale_run_status_rejections;
          if (!send(stale_status) || !wait_for([&] {
                return transport.statistics().stale_run_status_rejections > rejected_before;
              })) return 150;
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          if (wire::DummyDeviceStatusV2Codec::decode(
                  channel->observation.status.data(), channel->observation.status.size(),
                  &observed) != wire::CodecResult::kOk || observed.run_id() != run ||
              transport.statistics().raw_evidence_failure != 0U) return 151;
        }
      }
      transport.stop();
      const auto capture = transport.statistics();
      return capture.raw_evidence_failure == 0U &&
          channel->raw_evidence_failure.load(std::memory_order_acquire) == 0U &&
          capture.raw_evidence_pages_written == 18U ? 0 : 147;
    }
    if (raw_incomplete_mode) {
      transport.stop();
      const auto capture = transport.statistics();
      return capture.raw_evidence_failure ==
                 static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kIncomplete) &&
             channel->raw_evidence_failure.load(std::memory_order_acquire) ==
                 static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kIncomplete) &&
             capture.initialization_failed && !capture.ready ? 0 : 144;
    }
    auto released_status = valid;
    released_status.send_state = false;
    released_status.status.set_session_id(0U);
    released_status.status.set_reference_epoch(0U);
    released_status.status.set_management_epoch(1U);
    if (raw_bad_release_mode) released_status.status.set_parent_run_id(42U);
    if (raw_bad_release_mode) {
      if (!send(released_status) || !wait_for([&] {
            return transport.statistics().raw_evidence_failure != 0U;
          })) return 142;
      transport.stop();
      const auto capture = transport.statistics();
      return channel->raw_evidence_failure.load(std::memory_order_acquire) != 0U &&
             capture.initialization_failed && !capture.ready ? 0 : 143;
    }
    if (!send(released_status) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          Status observed;
          return wire::DummyDeviceStatusV2Codec::decode(
              channel->observation.status.data(), channel->observation.status.size(),
              &observed) == wire::CodecResult::kOk && observed.session_id() == 0U &&
              observed.run_id() == 3U;
        })) return 141;
    auto evidence = valid;
    evidence.send_status = false;
    evidence.send_state = false;
    evidence.send_evidence = true;
    evidence.evidence.set_schema_major(2U);
    evidence.evidence.set_schema_minor(4U);
    evidence.evidence.set_run_id(3U);
    evidence.evidence.set_attempt_id(3U);
    evidence.evidence.set_evidence_seq(0U);
    evidence.evidence.set_attempt_kind(1U);
    evidence.evidence.set_category(4U);
    evidence.evidence.set_final(1U);
    evidence.evidence.set_final_state(3U);
    evidence.evidence.set_terminal_reason(14U);
    std::array<std::uint8_t, 192U> seed{};
    seed[0] = 3U;
    seed[8] = 7U;
    std::copy(parameters.client_instance_id.begin(), parameters.client_instance_id.end(), seed.begin() + 16U);
    std::copy(parameters.contract_hash.begin(), parameters.contract_hash.end(), seed.begin() + 32U);
    std::copy(parameters.model_hash.begin(), parameters.model_hash.end(), seed.begin() + 64U);
    std::copy(parameters.profile_hash.begin(), parameters.profile_hash.end(), seed.begin() + 96U);
    std::copy(parameters.firmware_hash.begin(), parameters.firmware_hash.end(), seed.begin() + 128U);
    std::uint8_t previous[32]{};
    ::SHA256(seed.data(), seed.size(), previous);
    evidence.evidence.set_previous_chain_sha256(previous);
    std::array<std::uint8_t, wire::DummyAttemptEvidenceV2Codec::kEncodedSize> raw{};
    if (wire::DummyAttemptEvidenceV2Codec::encode(
            evidence.evidence, raw.data(), raw.size()) != wire::CodecResult::kOk) return 140;
    std::array<std::uint8_t, 32U + wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize + 1U> chain_input{};
    std::copy(std::begin(previous), std::end(previous), chain_input.begin());
    std::copy_n(raw.begin(), wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize,
                chain_input.begin() + 32U);
    chain_input.back() = raw[wire::DummyAttemptEvidenceV2Codec::kEvidenceLostOffset];
    std::uint8_t chain[32]{};
    ::SHA256(chain_input.data(), chain_input.size(), chain);
    evidence.evidence.set_chain_sha256(chain);
    if (!send(evidence) || !wait_for([&] {
          return transport.statistics().raw_evidence_pages_written == 1U;
        })) return 140;
    transport.stop();
    const auto capture = transport.statistics();
    std::FILE *file = std::fopen(capture_path.c_str(), "rb");
    if (!file) return 138;
    std::uint8_t header[39]{};
    bool found_page = false;
    if (::fseek(file, 8L, SEEK_SET) != 0) { std::fclose(file); return 139; }
    while (std::fread(header, 1U, sizeof(header), file) == sizeof(header)) {
      const auto payload_size = static_cast<std::uint16_t>(header[1] | (header[2] << 8U));
      if (header[0] == 1U && payload_size == wire::DummyAttemptEvidenceV2Codec::kEncodedSize &&
          header[11] == 7U && header[19] == 8U && header[27] == 3U) found_page = true;
      if (::fseek(file, payload_size, SEEK_CUR) != 0) break;
    }
    std::fclose(file);
    return capture.raw_evidence_failure == 0U &&
        channel->raw_evidence_failure.load(std::memory_order_acquire) == 0U && found_page ? 0 : 139;
  }
  auto stable_sequence = valid.state.state_seq();
  auto observed_invalidations = transport.statistics().invalidations;
  auto stable_since = std::chrono::steady_clock::now();
  if (!wait_for([&] {
        const auto statistics = transport.statistics();
        if (statistics.invalidations != observed_invalidations) {
          observed_invalidations = statistics.invalidations;
          valid.state.set_state_seq(++stable_sequence);
          stable_since = std::chrono::steady_clock::now();
        }
        send(valid);
        if (!system.copy_state(&snapshot) || snapshot.envelope.state_seq != stable_sequence) {
          stable_since = std::chrono::steady_clock::now();
          return false;
        }
        return std::chrono::steady_clock::now() - stable_since >= std::chrono::milliseconds(100);
      })) return 54;
  if (snapshot.position[0] != 0.25 ||
      (six_axis ? snapshot.position[1] != 0.2 : !std::isnan(snapshot.position[1]))) return 13;
  if (heartbeat_budget_mode) {
    const auto stops_before = transport.statistics().state_heartbeat_gap_stops;
    std::this_thread::sleep_for(std::chrono::milliseconds(102));
    if (transport.statistics().state_heartbeat_gap_stops != stops_before) return 148;
    if (!wait_for([&] {
          return transport.statistics().state_heartbeat_gap_stops > stops_before;
        }) || transport.statistics().last_state_gap_us <= 250000U) return 149;
    auto old_state = valid;
    old_state.send_status = false;
    if (!send_rejected_state(old_state) || system.copy_state(&snapshot)) return 150;
    auto rebind_status = valid;
    rebind_status.send_state = false;
    const auto status_before = transport.statistics().status_messages;
    if (!send(rebind_status) || !wait_for([&] {
          return transport.statistics().status_messages > status_before;
        })) return 151;
    auto newer_state = valid;
    newer_state.send_status = false;
    newer_state.state.set_state_seq(stable_sequence + 1U);
    const auto latched_before = transport.statistics().stop_latched_state_rejections;
    if (!send_rejected_state(newer_state) || system.copy_state(&snapshot) ||
        transport.statistics().stop_latched_state_rejections <= latched_before) return 152;
    transport.stop();
    return 0;
  }
  std::uint64_t accepted_observation_us = 0U;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    accepted_observation_us = channel->observation.state_received_us;
  }
  if (accepted_observation_us == 0U || !send_rejected_state(valid)) return 56;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    if (channel->observation.state_received_us != accepted_observation_us) return 57;
  }
  auto restore_valid_snapshot = [&] {
    valid.state.set_state_seq(++stable_sequence);
    return wait_for([&] {
      send(valid);
      return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
    });
  };
  if (default_ack_age_mode) {
    valid.state.set_state_seq(++stable_sequence);
    valid.state.set_produced_at_us(device_now_us());
    if (!wait_for([&] { return send(valid) && system.copy_state(&snapshot) &&
        snapshot.envelope.state_seq == stable_sequence; })) return 139;
  }
  if (inactive_prime_mode) {
    channel->inactive_prime_allowed.store(true);
    set_command_identity({valid.status.boot_epoch(), valid.status.session_id(),
        valid.status.reference_epoch(), valid.status.motion_epoch(), 9007199254740997ULL});
    const auto prime_generation = publish_control_update(positions, 6U, 0, nullptr);
    const bool sent = prime_generation != 0U && ::poll(&response, 1, 1500) == 1;
    Command prime_command;
    const bool valid_prime = sent && ::read(input, &prime_command, sizeof(prime_command)) == sizeof(prime_command) &&
        prime_command.producer_seq() == wire_producer_sequence(prime_generation) &&
        prime_command.motion_epoch() == valid.status.motion_epoch();
    transport.stop();
    clear_command_identity();
    return valid_prime ? 0 : 154;
  }
  if (system.on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 14;
  if (ticket_mode != 0U) {
    valid.state.set_state_seq(++stable_sequence);
    valid.state.set_produced_at_us(device_now_us());
    valid.state.set_freshness_ticket(9007199254740997ULL);
    if (!send(valid) || !wait_for([&] { return system.copy_state(&snapshot) &&
        snapshot.envelope.state_seq == stable_sequence; })) return 250;
  } else {
    valid.state.set_state_seq(++stable_sequence);
    valid.state.set_produced_at_us(device_now_us());
    if (!send(valid) || !wait_for([&] { return system.copy_state(&snapshot) &&
        snapshot.envelope.state_seq == stable_sequence; })) return 256;
    valid.state.set_produced_at_us(0U);
  }
  set_command_identity({7U, 8U, 9U, 10U, 9007199254740997ULL});
  const auto generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  if (!generation || !system.transfer_control_update()) return 15;
  if (::poll(&response, 1, 3000) != 1) return 16;
  Command command;
  const auto command_bytes = ::read(input, &command, sizeof(command));
  if (command_bytes != sizeof(command)) {
    std::fprintf(stderr, "first command read: bytes=%zd expected=%zu errno=%d revents=%d\n",
                 command_bytes, sizeof(command), errno, response.revents);
    return 17;
  }
  if (command.device_id()[0] != 1U || command.client_instance_id()[0] != 2U ||
      command.boot_epoch() != 7U || command.session_id() != 8U ||
      command.reference_epoch() != 9U || command.motion_epoch() != 10U || command.configured_mask() != parameters.configured_mask ||
      command.profile_id() != 1U || command.producer_seq() != generation ||
      command.freshness_ticket() == 0U || command.position_rad()[0] != 0.25 ||
      (six_axis ? command.position_rad()[1] != 0.2 : command.position_rad()[1] != 0.0) ||
      !std::all_of(std::begin(command.position_rad()), std::end(command.position_rad()),
                   [](double value) { return std::isfinite(value); })) {
    std::fprintf(stderr,
        "first command fields: device=%u client=%u boot=%llu session=%llu reference=%u motion=%u mask=%u profile=%u seq=%u expected_seq=%llu ticket=%llu position0=%.17g position1=%.17g\n",
        command.device_id()[0], command.client_instance_id()[0],
        static_cast<unsigned long long>(command.boot_epoch()),
        static_cast<unsigned long long>(command.session_id()),
        command.reference_epoch(), command.motion_epoch(), command.configured_mask(),
        command.profile_id(), command.producer_seq(),
        static_cast<unsigned long long>(generation),
        static_cast<unsigned long long>(command.freshness_ticket()),
        command.position_rad()[0], command.position_rad()[1]);
    return 17;
  }
  if (reject_during_ack_mode) {
    valid.status.set_command_reject_count(4U);
    valid.status.set_can_sent_mask(parameters.configured_mask);
    if (!send(valid) || !wait_for([&] {
          std::lock_guard<std::mutex> lock(channel->mutex);
          return channel->observation.rejected_ack_sequence == command.producer_seq();
        })) return 260;
    {
      std::lock_guard<std::mutex> lock(channel->mutex);
      const auto &observed=channel->observation;
      if (observed.rejected_ack_boot_epoch!=7U || observed.rejected_ack_session_id!=8U ||
          observed.rejected_ack_reference_epoch!=9U || observed.rejected_ack_motion_epoch!=10U ||
          observed.rejected_ack_sequence!=command.producer_seq()) return 261;
    }
    if (!wait_for([&] { const auto stats=transport.statistics();
          return stats.stop_requests_sent+stats.stop_request_failures!=0U; }) ||
        transport.statistics().group_sent_ack_timeouts!=0U) return 262;
    transport.stop();
    clear_command_identity();
    return 0;
  }
  if (ticket_mode != 0U) {
    const auto second_position = std::array<double, 6>{0.26, 0.2, 0.3, 0.4, 0.5, 0.6};
    const auto second_generation = publish_control_update(second_position.data(), 6U, 0, nullptr);
    if (second_generation == 0U || !system.transfer_control_update()) return 251;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(75);
    while (std::chrono::steady_clock::now() < deadline) {
      valid.state.set_state_seq(++stable_sequence);
      valid.state.set_produced_at_us(device_now_us());
      if (!send(valid)) return 252;
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
    valid.state.set_state_seq(++stable_sequence);
    valid.state.set_produced_at_us(device_now_us());
    valid.state.set_committed_producer_seq(static_cast<std::uint32_t>(generation));
    if (ticket_mode == 1U) valid.state.set_freshness_ticket(9007199254740998ULL);
    const auto before_authority = transport.statistics().invalidations;
    if (ticket_mode == 3U) {
      valid.status.set_session_id(99U);
      valid.state.set_session_id(99U);
      valid.state.set_freshness_ticket(9007199254740998ULL);
    }
    if (!send(valid)) return 253;
    if (ticket_mode == 3U && !wait_for([&] {
          return transport.statistics().invalidations > before_authority;
        })) return 259;
    if (ticket_mode == 1U) {
      if (::poll(&response, 1, 1500) != 1 || ::read(input, &command, sizeof(command)) != sizeof(command) ||
          command.producer_seq() != second_generation || command.position_rad()[0] != 0.26 ||
          command.freshness_ticket() != 9007199254740998ULL) return 254;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
      if (::poll(&response, 1, 0) == 1 ||
          transport.statistics().commands_published != 1U) return 255;
    }
    transport.stop();
    clear_command_identity();
    return 0;
  }
  if (ack_timeout_mode || default_ack_age_mode) {
    auto unacked = valid;
    unacked.send_status = false;
    const auto until = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(default_ack_age_mode ? 400 : 250);
    while (std::chrono::steady_clock::now() < until &&
           (default_ack_age_mode
                ? transport.statistics().state_stale_stops == 0U
                : transport.statistics().group_sent_ack_timeouts == 0U)) {
      if (default_ack_age_mode) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      unacked.state.set_state_seq(++stable_sequence);
      unacked.state.set_produced_at_us(device_now_us());
      if (!send(unacked)) return 134;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto statistics = transport.statistics();
    if (default_ack_age_mode) {
      if (statistics.state_stale_stops != 1U ||
          statistics.ack_unconfirmed_on_state_stale_stop +
              statistics.group_sent_ack_timeouts != 1U ||
          statistics.state_heartbeat_gap_stops != 0U) {
        std::fprintf(stderr,
            "default ack/age: stale=%llu unconfirmed=%llu ack_timeout=%llu heartbeat=%llu stop_sent=%llu\n",
            static_cast<unsigned long long>(statistics.state_stale_stops),
            static_cast<unsigned long long>(statistics.ack_unconfirmed_on_state_stale_stop),
            static_cast<unsigned long long>(statistics.group_sent_ack_timeouts),
            static_cast<unsigned long long>(statistics.state_heartbeat_gap_stops),
            static_cast<unsigned long long>(statistics.stop_requests_sent));
        return 140;
      }
      if (!wait_for([&] { return transport.statistics().stop_requests_sent == 1U; }))
        return 141;
      transport.stop();
      if (transport.statistics().stop_requests_confirmed != 0U ||
          transport.statistics().unconfirmed_stop_requests == 0U) return 143;
      return 0;
    }
    if (statistics.group_sent_ack_timeouts != 1U ||
        statistics.state_heartbeat_gap_stops != 0U ||
        statistics.group_sent_ack_misses == 0U ||
        statistics.state_callbacks < statistics.state_messages) return 135;
    transport.stop();
    return 0;
  }
  valid.status.set_committed_producer_seq(static_cast<std::uint32_t>(generation));
  valid.status.set_can_sent_mask(parameters.configured_mask);
  valid.state.set_state_seq(++stable_sequence);
  if (!wait_for([&] {
        send(valid);
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
      }) || transport.statistics().group_sent_ack_timeouts != 0U) return 67;
  // Replayed sequence, wrong device/profile and missing position evidence cannot replace feedback.
  auto invalid = valid;
  const double wrong_position[6]{0.75, NAN, NAN, NAN, NAN, NAN};
  invalid.state.set_position_rad(wrong_position);
  if (!send_rejected_state(invalid) ||
      !system.copy_state(&snapshot) || snapshot.position[0] != 0.25) return 18;
  for (unsigned fault = 0U; fault < 9U; ++fault) {
    bool rejected_while_snapshot_retained = false;
    for (unsigned attempt = 0U; attempt < 3U && !rejected_while_snapshot_retained; ++attempt) {
      auto wrong = valid;
      const auto sequence = stable_sequence + 1U;
      wrong.state.set_state_seq(sequence);
      wrong.state.set_produced_at_us(device_now_us());
      if (fault == 0U) { std::uint8_t other[16]{3U}; wrong.state.set_device_id(other); }
      if (fault == 1U) wrong.state.set_profile_id(2U);
      if (fault == 2U) wrong.state.set_configured_mask(six_axis ? 1U : 63U);
      if (fault == 3U) wrong.state.set_produced_at_us(device_now_us() + 2000000U);
      if (fault == 4U) wrong.state.set_produced_at_us(device_now_us() - 2000000U);
      if (fault == 5U) wrong.state.set_session_id(99U);
      if (fault == 6U) wrong.state.set_boot_epoch(99U);
      if (fault == 7U) wrong.state.set_reference_epoch(99U);
      if (fault == 8U) wrong.state.set_motion_epoch(99U);
      rejected_while_snapshot_retained = send_rejected_state(wrong) &&
          system.copy_state(&snapshot) && snapshot.position[0] == 0.25;
      if (!rejected_while_snapshot_retained && !restore_valid_snapshot()) return 21 + fault;
    }
    if (!rejected_while_snapshot_retained) return 21 + fault;
    valid.state.set_state_seq(++stable_sequence);
    if (!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq == stable_sequence; })) return 30 + fault;
  }
  // A status authority transition must immediately invalidate old state and commands.
  // State carrying the previous authority cannot repopulate the snapshot.
  for (unsigned changed_field = 0U; changed_field < 3U; ++changed_field) {
    auto transition = valid;
    transition.status.set_management_epoch(valid.status.management_epoch() + 1U);
    transition.state.set_management_epoch(transition.status.management_epoch());
    if (changed_field == 0U) transition.status.set_session_id(18U);
    if (changed_field == 1U) transition.status.set_reference_epoch(19U);
    if (changed_field == 2U) transition.status.set_motion_epoch(20U);
    transition.state.set_state_seq(2U);
    const auto before = transport.statistics().invalidations;
    if (!send(transition) || !wait_for([&] { return transport.statistics().invalidations > before; }) ||
        system.copy_state(&snapshot) || publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr) != 0U)
      return 40 + changed_field;
    if (changed_field == 0U) {
      const auto stops_after_invalidation = transport.statistics().state_heartbeat_gap_stops;
      std::this_thread::sleep_for(std::chrono::microseconds(
          parameters.state_publish_period_us * 5U + 20000U));
      if (transport.statistics().state_heartbeat_gap_stops != stops_after_invalidation)
        return 123;
    }
    valid.status = transition.status;
    valid.state.set_session_id(valid.status.session_id());
    valid.state.set_reference_epoch(valid.status.reference_epoch());
    valid.state.set_motion_epoch(valid.status.motion_epoch());
    valid.state.set_management_epoch(valid.status.management_epoch());
    valid.state.set_state_seq(1U);
    if (!wait_for([&] { send(valid); return system.copy_state(&snapshot); })) return 44 + changed_field;
  }
  valid.status.set_management_epoch(10U);
  valid.state.set_management_epoch(10U);
  valid.state.set_state_seq(2U);
  if (!wait_for([&] { send(valid); return system.copy_state(&snapshot); })) return 51;
  auto asynchronous_epoch = valid;
  asynchronous_epoch.status.set_motion_epoch(valid.status.motion_epoch() + 1U);
  asynchronous_epoch.state.set_motion_epoch(asynchronous_epoch.status.motion_epoch());
  asynchronous_epoch.state.set_state_seq(3U);
  const auto before_asynchronous_epoch = transport.statistics().invalidations;
  if (!wait_for([&] {
        send(asynchronous_epoch);
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == 3U;
      }) || transport.statistics().invalidations <= before_asynchronous_epoch) return 53;
  valid = asynchronous_epoch;
  auto reordered = valid;
  reordered.status.set_management_epoch(9U);
  reordered.status.set_session_id(0U);
  reordered.state.set_state_seq(4U);
  const auto before_reordered = transport.statistics().invalidations;
  if (!wait_for([&] { send(reordered); return system.copy_state(&snapshot); }) ||
      transport.statistics().invalidations != before_reordered) return 52;
  // A committed management reply authorizes the next epoch while status is suppressed.
  const auto next_motion_epoch = valid.status.motion_epoch() + 1U;
  channel->inactive_prime_allowed.store(true);
  set_command_identity({valid.status.boot_epoch(), valid.status.session_id(),
      valid.status.reference_epoch(), next_motion_epoch, 9007199254740997ULL});
  const auto unauthorized_generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  const auto before_unauthorized = transport.statistics().rejected_commands;
  if (!unauthorized_generation || !system.transfer_control_update() ||
      !wait_for([&] {
        valid.state.set_state_seq(++stable_sequence);
        valid.state.set_produced_at_us(device_now_us());
        send(valid);
        return transport.statistics().rejected_commands > before_unauthorized;
      })) return 66;
  auto send_fresh_state = [&] {
    auto heartbeat = valid;
    heartbeat.send_status = false;
    heartbeat.state.set_state_seq(++stable_sequence);
    heartbeat.state.set_produced_at_us(0U);
    return send(heartbeat);
  };
  auto wait_for_management = [&](std::shared_ptr<DeviceManagementRequest> &request) {
    const auto started = std::chrono::steady_clock::now();
    auto attempt_started = started;
    auto next_heartbeat = std::chrono::steady_clock::now();
    auto next_retry = started;
    bool sent = true;
    bool failed_in_flight = false;
    const bool completed = wait_for([&] {
      if (request->complete.load()) return true;
      const auto now = std::chrono::steady_clock::now();
      if (request->failed.load()) {
        // Retry only prompt failures, as with synchronous queue-full before peer delivery.
        if (now - attempt_started >= std::chrono::milliseconds(100)) {
          failed_in_flight = true;
          return true;
        }
        if (now >= next_retry) {
          auto retry = std::make_shared<DeviceManagementRequest>();
          retry->request = request->request;
          {
            std::lock_guard<std::mutex> lock(channel->mutex);
            channel->queued = retry;
          }
          request = std::move(retry);
          attempt_started = now;
          next_retry = now + std::chrono::milliseconds(10);
        }
      }
      if (std::chrono::steady_clock::now() >= next_heartbeat) {
        sent = send_fresh_state() && sent;
        next_heartbeat = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
      }
      return false;
    });
    if (!sent || !completed || failed_in_flight) {
      Management::Request decoded;
      wire::DummyManageV2RequestCodec::decode(request->request.data(), request->request.size(), &decoded);
      const auto stats = transport.statistics();
      std::fprintf(stderr,
          "management wait: request=%llu sent=%d failed=%d canceled=%d complete=%d elapsed_ms=%lld state=%llu rejected=%llu heartbeat=%llu stale=%llu stop_sent=%llu seq=%u\n",
          static_cast<unsigned long long>(decoded.request_id()), sent, request->failed.load(),
          request->canceled.load(), request->complete.load(),
          static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - started).count()),
          static_cast<unsigned long long>(stats.state_messages),
          static_cast<unsigned long long>(stats.rejected_states),
          static_cast<unsigned long long>(stats.state_heartbeat_gap_stops),
          static_cast<unsigned long long>(stats.state_stale_stops),
          static_cast<unsigned long long>(stats.stop_requests_sent), stable_sequence);
    }
    return sent && completed && !failed_in_flight;
  };
  Management::Request authorization;
  authorization.set_schema_major(2U);
  authorization.set_schema_minor(4U);
  authorization.set_device_id(device);
  const std::uint8_t client[16]{2U};
  authorization.set_client_instance_id(client);
  authorization.set_request_id(42U);
  authorization.set_request_seq(42U);
  authorization.set_operation(6U);
  authorization.set_expected_boot_epoch(valid.status.boot_epoch());
  authorization.set_expected_session_id(valid.status.session_id());
  authorization.set_expected_reference_epoch(valid.status.reference_epoch());
  authorization.set_expected_motion_epoch(valid.status.motion_epoch());
  authorization.set_expected_management_epoch(valid.status.management_epoch());
  authorization.set_expected_profile_id(parameters.profile_id);
  authorization.set_configured_mask(parameters.configured_mask);
  authorization.set_freshness_ticket(9007199254740997ULL);
  auto pending = std::make_shared<DeviceManagementRequest>();
  if (wire::DummyManageV2RequestCodec::encode(authorization, pending->request.data(),
        pending->request.size()) != wire::CodecResult::kOk) return 64;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = pending;
  }
  if (!wait_for_management(pending)) return 65;
  auto authorized_state = valid;
  authorized_state.send_status = false;
  authorized_state.state.set_motion_epoch(next_motion_epoch);
  authorized_state.state.set_state_seq(++stable_sequence);
  authorized_state.state.set_produced_at_us(device_now_us());
  if (!send(authorized_state) || !wait_for([&] {
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
      })) return 257;
  set_command_identity({valid.status.boot_epoch(), valid.status.session_id(),
      valid.status.reference_epoch(), next_motion_epoch, 9007199254740997ULL});
  const auto next_generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  if (!next_generation || ::poll(&response, 1, 3000) != 1)
    return 58;
  if (::read(input, &command, sizeof(command)) != sizeof(command) ||
      command.motion_epoch() != next_motion_epoch) return 59;
  auto status_suppressed = valid;
  status_suppressed.send_status = false;
  status_suppressed.state.set_produced_at_us(0U);
  status_suppressed.state.set_motion_epoch(next_motion_epoch);
  status_suppressed.state.set_state_seq(++stable_sequence);
  if (!wait_for([&] {
        send(status_suppressed);
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
      })) {
    const auto stats = transport.statistics();
    std::fprintf(stderr,
        "first new epoch: seq=%u expected=%u states=%llu rejected=%llu invalidations=%llu stops=%llu ready=%u\n",
        snapshot.envelope.state_seq, stable_sequence,
        static_cast<unsigned long long>(stats.state_messages),
        static_cast<unsigned long long>(stats.rejected_states),
        static_cast<unsigned long long>(stats.invalidations),
        static_cast<unsigned long long>(stats.heartbeat_stops), stats.ready);
    return 60;
  }
  status_suppressed.state.set_committed_producer_seq(static_cast<std::uint32_t>(next_generation));
  status_suppressed.state.set_state_seq(++stable_sequence);
  if (!wait_for([&] {
        send(status_suppressed);
        return system.copy_state(&snapshot) && snapshot.envelope.state_seq == stable_sequence;
      })) {
    const auto stats = transport.statistics();
    std::fprintf(stderr,
        "second committed state: seq=%u expected=%u states=%llu rejected=%llu invalidations=%llu stops=%llu\n",
        snapshot.envelope.state_seq, stable_sequence,
        static_cast<unsigned long long>(stats.state_messages),
        static_cast<unsigned long long>(stats.rejected_states),
        static_cast<unsigned long long>(stats.invalidations),
        static_cast<unsigned long long>(stats.heartbeat_stops));
    return 81;
  }
  const auto followup_generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  if (!followup_generation || !system.transfer_control_update() ||
      ::poll(&response, 1, 3000) != 1) return 82;
  if (::read(input, &command, sizeof(command)) != sizeof(command) ||
      command.motion_epoch() != next_motion_epoch ||
      command.producer_seq() != followup_generation) return 83;
  auto stale_epoch = status_suppressed;
  stale_epoch.state.set_motion_epoch(valid.status.motion_epoch());
  stale_epoch.state.set_state_seq(stable_sequence + 1U);
  if (!send_rejected_state(stale_epoch)) return 68;
  auto delayed_status = status_suppressed;
  delayed_status.send_status = true;
  delayed_status.status.set_committed_producer_seq(static_cast<std::uint32_t>(followup_generation));
  delayed_status.state.set_state_seq(++stable_sequence);
  const auto before_delayed_status = transport.statistics();
  if (!wait_for([&] {
        send(delayed_status);
        return transport.statistics().status_messages > before_delayed_status.status_messages;
      }) || transport.statistics().invalidations != before_delayed_status.invalidations ||
      !system.copy_state(&snapshot)) return 63;
  valid.status.set_motion_epoch(next_motion_epoch);
  valid.status.set_committed_producer_seq(static_cast<std::uint32_t>(followup_generation));
  valid.state.set_motion_epoch(next_motion_epoch);
  valid.state.set_state_seq(stable_sequence);
  const auto before_ack = transport.statistics().status_messages;
  if (!wait_for([&] {
        send(valid);
        return transport.statistics().status_messages > before_ack;
      })) return 62;
  authorization.set_request_id(43U);
  authorization.set_request_seq(43U);
  authorization.set_expected_motion_epoch(next_motion_epoch);
  auto jump_request = std::make_shared<DeviceManagementRequest>();
  if (wire::DummyManageV2RequestCodec::encode(authorization, jump_request->request.data(),
        jump_request->request.size()) != wire::CodecResult::kOk) return 69;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = jump_request;
  }
  if (!wait_for_management(jump_request)) return 70;
  Management::Response jump_reply;
  if (wire::DummyManageV2ResponseCodec::decode(jump_request->response.data(),
      jump_request->response.size(), &jump_reply) != wire::CodecResult::kOk) return 127;
  auto status_only = valid;
  status_only.send_state = false;
  const auto before_jump_barrier = transport.statistics().status_messages;
  if (!send(status_only) ||
      !wait_for([&] { return transport.statistics().status_messages > before_jump_barrier; })) return 75;
  auto jumped = valid;
  jumped.send_status = false;
  jumped.state.set_motion_epoch(next_motion_epoch + 2U);
  jumped.state.set_state_seq(stable_sequence + 1U);
  jumped.state.set_produced_at_us(device_now_us());
  if (!send_rejected_state(jumped) || !system.copy_state(&snapshot) ||
      snapshot.envelope.state_seq == jumped.state.state_seq()) return 71;
  if (!send_fresh_state()) return 261;
  authorization.set_request_id(44U);
  authorization.set_request_seq(44U);
  authorization.set_operation(0U);
  auto status_query = std::make_shared<DeviceManagementRequest>();
  if (wire::DummyManageV2RequestCodec::encode(authorization, status_query->request.data(),
        status_query->request.size()) != wire::CodecResult::kOk) return 72;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = status_query;
  }
  if (!wait_for_management(status_query)) return 73;
  const auto before_query_barrier = transport.statistics().status_messages;
  if (!send(status_only) ||
      !wait_for([&] { return transport.statistics().status_messages > before_query_barrier; })) return 76;
  auto unlicensed = valid;
  unlicensed.send_status = false;
  unlicensed.state.set_motion_epoch(next_motion_epoch + 1U);
  unlicensed.state.set_state_seq(stable_sequence + 1U);
  unlicensed.state.set_produced_at_us(device_now_us());
  if (!send_rejected_state(unlicensed) || !system.copy_state(&snapshot) ||
      snapshot.envelope.state_seq == unlicensed.state.state_seq()) return 74;
  if (!send_fresh_state()) return 262;
  authorization.set_request_id(45U);
  authorization.set_request_seq(45U);
  authorization.set_operation(12U);
  authorization.set_queried_operation_id(43U);
  auto unrelated_query = std::make_shared<DeviceManagementRequest>();
  if (wire::DummyManageV2RequestCodec::encode(authorization, unrelated_query->request.data(),
        unrelated_query->request.size()) != wire::CodecResult::kOk) return 77;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = unrelated_query;
  }
  if (!wait_for_management(unrelated_query)) return 78;
  const auto before_unrelated_barrier = transport.statistics().status_messages;
  if (!send(status_only) ||
      !wait_for([&] { return transport.statistics().status_messages > before_unrelated_barrier; })) return 79;
  unlicensed.state.set_produced_at_us(device_now_us());
  if (!send_rejected_state(unlicensed) || !system.copy_state(&snapshot) ||
      snapshot.envelope.state_seq == unlicensed.state.state_seq()) return 80;
  if (!send_fresh_state()) return 263;
  channel->inactive_prime_allowed.store(false);
  valid.state.set_state_seq(++stable_sequence);
  valid.state.set_produced_at_us(0U);
  if (!wait_for([&] { send(valid); return system.copy_state(&snapshot) &&
                                  snapshot.envelope.state_seq == stable_sequence; })) return 108;
  authorization.set_request_id(46U);
  authorization.set_request_seq(46U);
  authorization.set_operation(6U);
  authorization.set_expected_motion_epoch(next_motion_epoch);
  authorization.set_expected_management_epoch(jump_reply.management_epoch());
  auto stop_authority = std::make_shared<DeviceManagementRequest>();
  if (wire::DummyManageV2RequestCodec::encode(authorization, stop_authority->request.data(),
      stop_authority->request.size()) != wire::CodecResult::kOk) return 130;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    channel->queued = stop_authority;
  }
  if (!wait_for_management(stop_authority)) return 131;
  Management::Response stop_authority_reply;
  if (wire::DummyManageV2ResponseCodec::decode(stop_authority->response.data(),
      stop_authority->response.size(), &stop_authority_reply) != wire::CodecResult::kOk) return 132;
  const auto accepted_produced_us = snapshot.envelope.produced_at_us;
  const auto before_timeout = transport.statistics().invalidations;
  if (!wait_for([&] { return transport.statistics().invalidations > before_timeout; })) return 100;
  pollfd stop_response{stops_input, POLLIN, 0};
  if (::poll(&stop_response, 1, 3000) != 1) return 128;
  Management::Request stop_request;
  if (::read(stops_input, &stop_request, sizeof(stop_request)) != sizeof(stop_request) ||
      stop_request.operation() != 10U ||
      stop_request.expected_motion_epoch() != stop_authority_reply.motion_epoch() ||
      stop_request.expected_management_epoch() != stop_authority_reply.management_epoch()) {
    std::fprintf(stderr, "stop tuple: motion=%u/%u management=%u/%u\n",
        stop_request.expected_motion_epoch(), stop_authority_reply.motion_epoch(),
        stop_request.expected_management_epoch(), stop_authority_reply.management_epoch());
    return 129;
  }
  pollfd query_response{stops_input, POLLIN, 0};
  if (::poll(&query_response, 1, 3000) != 1) return 139;
  Management::Request first_stop_query;
  if (::read(stops_input, &first_stop_query, sizeof(first_stop_query)) != sizeof(first_stop_query) ||
      first_stop_query.operation() != 12U ||
      first_stop_query.configured_mask() != parameters.configured_mask ||
      first_stop_query.queried_operation_id() != stop_request.request_id()) return 140;
  if (transport.statistics().stop_requests_confirmed != 0U) return 141;
  if (never_complete_stop) {
    transport.stop();
    if (transport.statistics().stop_requests_confirmed != 0U ||
        transport.statistics().unconfirmed_stop_requests == 0U) return 142;
    return 0;
  }
  if (::poll(&query_response, 1, 3000) != 1) return 144;
  Management::Request terminal_stop_query;
  if (::read(stops_input, &terminal_stop_query, sizeof(terminal_stop_query)) !=
          sizeof(terminal_stop_query) ||
      terminal_stop_query.operation() != 12U ||
      terminal_stop_query.queried_operation_id() != stop_request.request_id() ||
      terminal_stop_query.request_id() == first_stop_query.request_id()) return 145;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (transport.statistics().stop_requests_confirmed != 0U) return 146;
  auto disabling = valid;
  disabling.send_state = false;
  disabling.status.set_motion_epoch(stop_request.expected_motion_epoch() + 1U);
  disabling.status.set_management_epoch(stop_request.expected_management_epoch() + 1U);
  disabling.status.set_core_state(8U);
  if (!send(disabling) || !wait_for([&] {
      std::lock_guard<std::mutex> lock(channel->mutex);
      Status observed;
      return wire::DummyDeviceStatusV2Codec::decode(channel->observation.status.data(),
          channel->observation.status.size(), &observed) == wire::CodecResult::kOk &&
          observed.core_state() == 8U;
    })) return 227;
  auto stop_idle = valid;
  stop_idle.status.set_session_id(0U);
  stop_idle.status.set_reference_epoch(0U);
  stop_idle.status.set_motion_epoch(stop_request.expected_motion_epoch() + 1U);
  stop_idle.status.set_management_epoch(stop_request.expected_management_epoch() + 1U);
  stop_idle.status.set_core_state(1U);
  stop_idle.status.set_release_committed(true);
  stop_idle.state.set_motion_epoch(stop_idle.status.motion_epoch());
  stop_idle.state.set_session_id(0U);
  stop_idle.state.set_reference_epoch(0U);
  stop_idle.state.set_management_epoch(stop_idle.status.management_epoch());
  stop_idle.state.set_core_state(1U);
  stop_idle.state.set_position_valid_mask(0U);
  stop_idle.state.set_raw_position_valid_mask(parameters.configured_mask);
  stop_idle.state.set_state_seq(stable_sequence + 1U);
  const double idle_raw[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  stop_idle.state.set_raw_motor_position_rev(idle_raw);
  auto uncommitted_idle = stop_idle;
  uncommitted_idle.send_state = false;
  uncommitted_idle.status.set_release_committed(false);
  const auto before_uncommitted = transport.statistics().status_messages;
  if (!send(uncommitted_idle) || !wait_for([&] {
      return transport.statistics().status_messages > before_uncommitted;
    })) return 228;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    Status observed;
    if (wire::DummyDeviceStatusV2Codec::decode(channel->observation.status.data(),
        channel->observation.status.size(), &observed) != wire::CodecResult::kOk ||
        observed.core_state() != 8U) return 229;
  }
  if (!wait_for([&] {
        send(stop_idle);
        return transport.statistics().stop_requests_confirmed > 0U;
      }) ||
      transport.statistics().unconfirmed_stop_requests != 0U) return 133;
  if (six_axis && (transport.statistics().state_heartbeat_gap_stops == 0U ||
      transport.statistics().last_state_gap_us < parameters.state_publish_period_us * 5U)) return 122;
  auto stale_status_after_stop = valid;
  stale_status_after_stop.send_state = false;
  const auto stale_status_count = transport.statistics().status_messages;
  if (!send(stale_status_after_stop) || !wait_for([&] {
      return transport.statistics().status_messages > stale_status_count;
    })) return 137;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    Status observed_status;
    if (wire::DummyDeviceStatusV2Codec::decode(channel->observation.status.data(),
        channel->observation.status.size(), &observed_status) != wire::CodecResult::kOk ||
        observed_status.motion_epoch() != stop_idle.status.motion_epoch()) return 138;
  }
  auto status_only_after_timeout = valid;
  status_only_after_timeout.send_state = false;
  status_only_after_timeout.status.set_session_id(0U);
  status_only_after_timeout.status.set_reference_epoch(0U);
  status_only_after_timeout.status.set_motion_epoch(stop_request.expected_motion_epoch() + 1U);
  status_only_after_timeout.status.set_management_epoch(stop_request.expected_management_epoch() + 1U);
  const auto before_rebind_status = transport.statistics().status_messages;
  if (!wait_for([&] {
        send(status_only_after_timeout);
        return transport.statistics().status_messages > before_rebind_status;
      })) return 101;
  auto replay = valid;
  replay.send_status = false;
  replay.state.set_state_seq(stable_sequence);
  replay.state.set_produced_at_us(device_now_us());
  if (!send_rejected_state(replay) || system.copy_state(&snapshot)) return 102;
  replay.state.set_state_seq(stable_sequence + 1U);
  replay.state.set_produced_at_us(accepted_produced_us);
  if (!send_rejected_state(replay) || system.copy_state(&snapshot)) return 107;
  replay.state.set_state_seq(stable_sequence + 1U);
  replay.state.set_produced_at_us(accepted_produced_us + 1U);
  if (!send_rejected_state(replay) || system.copy_state(&snapshot)) return 126;
  valid.state.set_state_seq(++stable_sequence);
  if (!send_rejected_state(valid) || system.copy_state(&snapshot)) return 103;
  if (six_axis) {
    auto released = valid;
    released.status = status_only_after_timeout.status;
    released.status.set_core_state(1U);
    released.state.set_core_state(1U);
    released.state.set_session_id(0U);
    released.state.set_reference_epoch(0U);
    released.state.set_motion_epoch(stop_request.expected_motion_epoch() + 1U);
    released.state.set_management_epoch(stop_request.expected_management_epoch() + 1U);
    released.state.set_position_valid_mask(0U);
    released.state.set_velocity_valid_mask(63U);
    released.state.set_raw_position_valid_mask(63U);
    released.state.set_state_seq(stable_sequence + 1U);
    released.state.set_produced_at_us(device_now_us());
    const double raw_motor_rev[6]{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    released.state.set_raw_motor_position_rev(raw_motor_rev);
    if (!wait_for([&] {
          send(released);
          std::lock_guard<std::mutex> lock(channel->mutex);
          State observed;
          return channel->observation.state_received_us != 0U &&
              wire::DummyJointStateV2Codec::decode(channel->observation.state.data(),
                channel->observation.state.size(), &observed) == wire::CodecResult::kOk &&
              observed.core_state() == 1U && observed.state_seq() == released.state.state_seq();
        }) || system.copy_state(&snapshot)) return 97;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  (void)system.copy_state(&snapshot);
  auto rebooted = valid;
  rebooted.status.set_boot_epoch(11U);
  rebooted.state.set_boot_epoch(11U);
  rebooted.state.set_position_valid_mask(0U);
  const auto invalidations = transport.statistics().invalidations;
  send(rebooted);
  if (!wait_for([&] {
        if (transport.statistics().invalidations <= invalidations) return false;
        std::lock_guard<std::mutex> lock(channel->mutex);
        Status observed_status;
        return wire::DummyDeviceStatusV2Codec::decode(
            channel->observation.status.data(), channel->observation.status.size(),
            &observed_status) == wire::CodecResult::kOk && observed_status.boot_epoch() == 11U;
      }) ||
      system.copy_state(&snapshot) || publish_control_update(positions, 1U, 0, nullptr) != 0U) return 19;
  send(valid);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  if (system.copy_state(&snapshot)) return 20;
  auto post_stop = valid;
  post_stop.status.set_boot_epoch(11U);
  post_stop.state.set_boot_epoch(11U);
  post_stop.state.set_state_seq(1U);
  post_stop.state.set_produced_at_us(device_now_us());
  if (!send_rejected_state(post_stop) || system.copy_state(&snapshot)) return 104;
  const auto diagnostics = transport.statistics();
  if (diagnostics.state_callbacks < diagnostics.state_messages ||
      diagnostics.state_identity_rejections == 0U ||
      diagnostics.state_authority_rejections == 0U ||
      diagnostics.state_sequence_rejections == 0U ||
      diagnostics.stop_latched_state_rejections == 0U) return 136;
  transport.stop();
  std::printf("component_only: synthetic_peer axes=%u full identity, device/profile/mask/age/sequence rejection, authority/boot invalidation PASS\n",
              six_axis ? 6U : 1U);
  return 0;
}
}  // namespace
int run_component_case(bool six_axis, bool ack_timeout_mode, bool default_ack_age_mode,
                       bool never_complete_stop = false,
                       bool heartbeat_budget_mode = false,
                       bool revoke_owner_before_stop_receipt = false,
                       bool inactive_prime_mode = false,
                       bool cold_idle_gap_mode = false,
                       bool cold_fault_gap_mode = false,
                       bool raw_capture_mode = false,
                       bool raw_bad_release_mode = false,
                       bool raw_incomplete_mode = false,
                       bool raw_many_mode = false,
                       unsigned raw_order_case = 0U,
                       unsigned complete_order_mode = 0U,
                       unsigned release_deactivate_mode = 0U,
                       unsigned ticket_mode = 0U,
                       bool reject_during_ack_mode = false) {
  SerialBridge bridge;
  if (!bridge.open_pair()) return 30;
  int requests[2]{}, responses[2]{};
  int stops[2]{};
  if (::pipe(requests) != 0 || ::pipe(responses) != 0 || ::pipe(stops) != 0) return 31;
  const auto child = ::fork();
  if (child < 0) return 32;
  if (child == 0) {
    bridge.close_child_fds();
    ::close(requests[1]); ::close(responses[0]); ::close(stops[0]);
    ::_exit(run_peer(bridge.endpoint(0), requests[0], responses[1], stops[1],
                     never_complete_stop, revoke_owner_before_stop_receipt,
                     release_deactivate_mode == 7U));
  }
  bridge.start();
  ::close(requests[0]); ::close(responses[1]); ::close(stops[1]);
  const auto result = run_parent(bridge.endpoint(1), responses[0], requests[1], stops[0],
                                 six_axis, ack_timeout_mode, default_ack_age_mode,
                                 never_complete_stop, heartbeat_budget_mode, inactive_prime_mode,
                                 cold_idle_gap_mode, cold_fault_gap_mode,
                                 raw_capture_mode, raw_bad_release_mode,
                                 raw_incomplete_mode, raw_many_mode, raw_order_case, complete_order_mode,
                                 release_deactivate_mode, ticket_mode, reject_during_ack_mode);
  if (result != 0) std::fprintf(stderr, "component_only assertion failed: %d\n", result);
  ::kill(child, SIGTERM);
  ::waitpid(child, nullptr, 0);
  ::close(requests[1]); ::close(responses[0]); ::close(stops[0]);
  return result;
}
int main(int argc, char **argv) {
  if (argc == 2 && std::strcmp(argv[1], "historical_idle_baseline") == 0)
    return test_released_idle_startup_baseline();
  if (!test_group_sent_ack()) return 124;
  if (!test_hardware_state_age_heartbeat()) return 147;
  if (!test_heartbeat_budget_parameter()) return 153;
  const bool six_axis = argc >= 2;
  const bool revoked_stop = argc == 3 && std::strcmp(argv[2], "revoked_stop") == 0;
  const bool never_complete_stop = argc == 3 &&
      (std::strcmp(argv[2], "pending_stop") == 0 || revoked_stop);
  const bool heartbeat_budget_mode = argc == 3 && std::strcmp(argv[2], "heartbeat_budget") == 0;
  const bool inactive_prime_mode = argc == 3 && std::strcmp(argv[2], "inactive_prime") == 0;
  const bool cold_idle_gap_mode = argc == 3 && std::strcmp(argv[2], "cold_idle_gap") == 0;
  const bool cold_fault_gap_mode = argc == 3 && std::strcmp(argv[2], "cold_fault_gap") == 0;
  const bool raw_bad_release_mode = argc == 3 && std::strcmp(argv[2], "raw_bad_release") == 0;
  const bool raw_incomplete_mode = argc == 3 && std::strcmp(argv[2], "raw_incomplete") == 0;
  const bool raw_many_mode = argc == 3 && std::strcmp(argv[2], "raw_many") == 0;
  const unsigned raw_order_case = argc == 3 && std::strcmp(argv[2], "raw_order") == 0 ? 1U :
      argc == 3 && std::strcmp(argv[2], "raw_order_bad_boot") == 0 ? 2U :
      argc == 3 && std::strcmp(argv[2], "raw_order_bad_session") == 0 ? 3U :
      argc == 3 && std::strcmp(argv[2], "raw_order_bad_kind") == 0 ? 4U :
      argc == 3 && std::strcmp(argv[2], "raw_order_bad_chain") == 0 ? 5U :
      argc == 3 && std::strcmp(argv[2], "raw_order_unverified_terminal") == 0 ? 6U :
      argc == 3 && std::strcmp(argv[2], "raw_order_missing_status") == 0 ? 7U :
      argc == 3 && std::strcmp(argv[2], "raw_order_duplicate_seq") == 0 ? 8U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release") == 0 ? 9U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_bad_boot") == 0 ? 10U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_bad_kind") == 0 ? 11U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_bad_chain") == 0 ? 12U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_missing_final") == 0 ? 13U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_wrong_final_chain") == 0 ? 14U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle") == 0 ? 15U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle_unreleased") == 0 ? 16U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle_wrong_boot") == 0 ? 17U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle_wrong_session") == 0 ? 18U :
      argc == 3 && std::strcmp(argv[2], "raw_historical_idle_active") == 0 ? 19U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_page_first") == 0 ? 20U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_page_first_bad_parent") == 0 ? 21U :
      argc == 3 && std::strcmp(argv[2], "raw_order_release_page_first_old_run") == 0 ? 23U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first") == 0 ? 24U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_boot") == 0 ? 25U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_session") == 0 ? 26U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_old_run") == 0 ? 27U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_category") == 0 ? 28U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_group_count") == 0 ? 29U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_bad_mask") == 0 ? 30U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_zero_motion_epoch") == 0 ? 31U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_page_first_wrong_final_chain") == 0 ? 32U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_final_status_before_terminal") == 0 ? 33U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_page_first") == 0 ? 34U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_boot") == 0 ? 35U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_session") == 0 ? 36U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_no_terminal") == 0 ? 37U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_motion") == 0 ? 38U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_run") == 0 ? 39U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_bad_mask") == 0 ? 40U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_no_motion_complete") == 0 ? 41U :
      argc == 3 && std::strcmp(argv[2], "raw_order_refresh_hold_no_receipt") == 0 ? 42U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor") == 0 ? 43U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_stale_terminal") == 0 ? 44U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_bad_boot") == 0 ? 45U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_bad_session") == 0 ? 46U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_bad_run") == 0 ? 47U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_no_terminal") == 0 ? 48U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_late_wrong_chain") == 0 ? 49U :
      argc == 3 && std::strcmp(argv[2], "raw_order_multi_successor_late_correct_chain") == 0 ? 50U : 0U;
  const bool raw_capture_mode = raw_bad_release_mode || raw_incomplete_mode || raw_many_mode || raw_order_case != 0U ||
      (argc == 3 && std::strcmp(argv[2], "raw_release_receipt") == 0) ||
      (argc == 3 && std::strcmp(argv[2], "raw_direct_receipt") == 0) ||
      (argc == 3 && std::strcmp(argv[2], "raw_capture") == 0);
  const unsigned complete_order_mode = argc == 3 && std::strcmp(argv[2],"complete_status_first")==0 ? 1U :
      argc == 3 && std::strcmp(argv[2],"complete_state_first")==0 ? 2U :
      argc == 3 && std::strcmp(argv[2],"complete_response_first")==0 ? 3U :
      argc == 3 && std::strcmp(argv[2],"complete_unsolicited_jump")==0 ? 4U :
      argc == 3 && std::strcmp(argv[2],"complete_state_timeout")==0 ? 5U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_ack")==0 ? 6U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_ack_mismatch")==0 ? 7U :
      argc == 3 && std::strcmp(argv[2],"complete_pending_hold_suppressed")==0 ? 8U :
      argc == 3 && std::strcmp(argv[2],"complete_fault_status")==0 ? 9U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_before_response")==0 ? 10U :
      argc == 3 && std::strcmp(argv[2],"complete_freeze_old_epoch")==0 ? 11U :
      argc == 3 && std::strcmp(argv[2],"complete_wait_terminal_ack")==0 ? 12U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_small_delta")==0 ? 13U :
      argc == 3 && std::strcmp(argv[2],"complete_hold_expired_pending")==0 ? 14U :
      argc == 3 && std::strcmp(argv[2],"complete_invalid_binding")==0 ? 15U :
      argc == 3 && std::strcmp(argv[2],"complete_one_prime")==0 ? 17U :
      argc == 3 && std::strcmp(argv[2],"complete_export_ready_refresh")==0 ? 18U :
      argc == 3 && std::strcmp(argv[2],"complete_no_export_refresh")==0 ? 19U :
      argc == 3 && std::strcmp(argv[2],"complete_periodic_refresh")==0 ? 20U :
      argc == 3 && std::strcmp(argv[2],"complete_wrong_run_chain")==0 ? 21U :
      argc == 3 && std::strcmp(argv[2],"complete_lost_chain")==0 ? 22U :
      argc == 3 && std::strcmp(argv[2],"complete_export_then_lost")==0 ? 23U :
      argc == 3 && std::strcmp(argv[2],"complete_export_then_regressed")==0 ? 24U :
      argc == 3 && std::strcmp(argv[2],"complete_export_then_wrong_kind")==0 ? 25U :
      argc == 3 && std::strcmp(argv[2],"complete_refresh_stale_authority")==0 ? 26U :
      argc == 3 && std::strcmp(argv[2],"complete_export_refresh_ack_unconfirmed")==0 ? 27U : 0U;
  const bool default_ack_age_mode = argc == 3 && std::strcmp(argv[2], "default_ack_age") == 0;
  const unsigned release_deactivate_mode = argc == 3 &&
      (std::strcmp(argv[2], "released_idle_deactivate") == 0 ||
       std::strcmp(argv[2], "raw_release_receipt") == 0) ? 1U :
      argc == 3 && std::strcmp(argv[2], "raw_direct_receipt") == 0 ? 7U :
      argc == 3 && std::strcmp(argv[2], "released_idle_unconfirmed") == 0 ? 2U :
      argc == 3 && std::strcmp(argv[2], "released_idle_on_error") == 0 ? 3U :
      argc == 3 && std::strcmp(argv[2], "running_deactivate") == 0 ? 4U :
      argc == 3 && std::strcmp(argv[2], "released_idle_no_gate_reply") == 0 ? 5U :
      argc == 3 && std::strcmp(argv[2], "released_state_before_status") == 0 ? 6U :
      argc == 3 && std::strcmp(argv[2], "released_idle_heartbeat_gap") == 0 ? 8U :
      argc == 3 && std::strcmp(argv[2], "released_idle_before_reply") == 0 ? 9U :
      argc == 3 && std::strcmp(argv[2], "released_no_final_idle") == 0 ? 10U : 0U;
  const unsigned ticket_mode = argc == 3 && std::strcmp(argv[2], "ticket_refresh") == 0 ? 1U :
      argc == 3 && std::strcmp(argv[2], "ticket_stale") == 0 ? 2U :
      argc == 3 && std::strcmp(argv[2], "ticket_authority") == 0 ? 3U : 0U;
  const bool reject_during_ack_mode = argc == 3 &&
      std::strcmp(argv[2], "reject_during_ack") == 0;
  const auto result = run_component_case(six_axis,
      argc == 3 && ticket_mode == 0U && !default_ack_age_mode && !never_complete_stop && !heartbeat_budget_mode &&
          !inactive_prime_mode && !cold_idle_gap_mode && !cold_fault_gap_mode && !raw_capture_mode &&
          release_deactivate_mode == 0U && !reject_during_ack_mode &&
          complete_order_mode==0U,
      default_ack_age_mode, never_complete_stop, heartbeat_budget_mode || complete_order_mode!=0U, revoked_stop,
      inactive_prime_mode, cold_idle_gap_mode, cold_fault_gap_mode,
      raw_capture_mode, raw_bad_release_mode, raw_incomplete_mode, raw_many_mode, raw_order_case,
      complete_order_mode, release_deactivate_mode, ticket_mode, reject_during_ack_mode);
  if (result != 0 || !six_axis || argc != 2) return result;
  return run_component_case(true, false, true);
}
