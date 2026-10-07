#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include <atomic>
#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <poll.h>
#include <pty.h>
#include <signal.h>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>
using namespace dummy_motion_client;
namespace wire = dp::protocol::dummy_robot_v2;
using Service = dp::protocol::services::DummyManageV2RawFixed;

namespace {
using Clock = std::chrono::steady_clock;
std::atomic<unsigned> reply_calls{0};

bool signal_peer(int fd, char value) { return write(fd, &value, 1) == 1; }

bool read_peer_signal(int fd, char expected, int timeout_ms) {
  pollfd ready{fd, POLLIN, 0};
  if (poll(&ready, 1, timeout_ms) != 1 || !(ready.revents & POLLIN)) return false;
  char actual = 0;
  return read(fd, &actual, 1) == 1 && actual == expected;
}

class PeerProcess {
 public:
  explicit PeerProcess(pid_t pid) : pid_(pid) {}
  ~PeerProcess() {
    if (pid_ <= 0) return;
    kill(pid_, SIGTERM);
    while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
  }
  PeerProcess(const PeerProcess &) = delete;
  PeerProcess &operator=(const PeerProcess &) = delete;

 private:
  pid_t pid_;
};

struct SignalReader {
  int fd;
  ~SignalReader() { close(fd); }
};

class PtyBridge {
 public:
  bool open_pair() {
    char left_name[128]{};
    char right_name[128]{};
    if (openpty(&left_master_, &left_slave_, left_name, nullptr, nullptr) != 0 ||
        openpty(&right_master_, &right_slave_, right_name, nullptr, nullptr) != 0) return false;
    if (fcntl(left_master_, F_SETFL, O_NONBLOCK) < 0 ||
        fcntl(right_master_, F_SETFL, O_NONBLOCK) < 0) return false;
    close(left_slave_); left_slave_ = -1;
    close(right_slave_); right_slave_ = -1;
    left_endpoint_ = std::string("serial/") + left_name + "#baudrate=115200";
    right_endpoint_ = std::string("serial/") + right_name + "#baudrate=115200";
    return true;
  }

  void close_child_fds() const {
    close(left_master_); close(right_master_);
  }

  void start() {
    running_ = true;
    worker_ = std::thread([this] {
      while (running_) {
        pollfd fds[]{{left_master_, POLLIN, 0}, {right_master_, POLLIN, 0}};
        if (poll(fds, 2, 20) <= 0) continue;
        if (fds[0].revents & POLLIN) forward(left_master_, right_master_);
        if (fds[1].revents & POLLIN) forward(right_master_, left_master_);
      }
    });
  }

  void stop() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (left_master_ >= 0) { close(left_master_); left_master_ = -1; }
    if (right_master_ >= 0) { close(right_master_); right_master_ = -1; }
  }
  ~PtyBridge() {
    stop();
    if (left_slave_ >= 0) close(left_slave_);
    if (right_slave_ >= 0) close(right_slave_);
  }

  const std::string &left_endpoint() const { return left_endpoint_; }
  const std::string &right_endpoint() const { return right_endpoint_; }

 private:
  void forward(int source, int target) {
    std::uint8_t bytes[512];
    const ssize_t size = read(source, bytes, sizeof(bytes));
    if (size <= 0) return;
    for (ssize_t offset = 0; running_ && offset < size;) {
      const ssize_t written = write(target, bytes + offset, static_cast<size_t>(size - offset));
      if (written > 0) offset += written;
      else if (written < 0 && errno == EINTR) continue;
      else if (written < 0 && errno == EAGAIN) {
        pollfd writable{target, POLLOUT, 0};
        poll(&writable, 1, 20);
      } else return;
    }
  }

  int left_master_ = -1, right_master_ = -1;
  int left_slave_ = -1, right_slave_ = -1;
  std::string left_endpoint_, right_endpoint_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

struct StopBridgeBeforeTransport {
  PtyBridge &bridge;
  ~StopBridgeBeforeTransport() { bridge.stop(); }
};

std::vector<std::uint8_t> request_bytes(std::uint64_t request_id = 0x20000000000001ULL) {
  ManageRequest request;
  request.set_schema_major(2); request.set_schema_minor(4U);
  request.set_request_id(request_id);
  std::vector<std::uint8_t> bytes(wire::DummyManageV2RequestCodec::kEncodedSize);
  if (wire::DummyManageV2RequestCodec::encode(request, bytes.data(), bytes.size()) != wire::CodecResult::kOk)
    return {};
  return bytes;
}

bool wait_ready(DpManagementTransport &transport) {
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  while (!transport.ready() && Clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return transport.ready();
}

int observation_continuity_tests() {
  DeviceObservation observation;
  observation.state_received_us = 1000000U;
  observation.status_received_us = 1000000U;
  ObservationContinuity hold_gap(1000000U, 2000000U, 500000U);
  if (hold_gap.evaluate(observation, 1000000U, true, true) != ObservationContinuity::Result::kReady ||
      hold_gap.evaluate(observation, 2053000U, true, true) != ObservationContinuity::Result::kWait)
    return 81;
  observation.state_received_us = 2053001U;
  if (hold_gap.evaluate(observation, 2053001U, true, true) != ObservationContinuity::Result::kReady)
    return 82;
  observation.state_received_us = 1000000U;
  ObservationContinuity continuity(250000U, 2000000U, 500000U);
  if (continuity.evaluate(observation, 1000000U, false, true) != ObservationContinuity::Result::kReady)
    return 41;
  observation.state_received_us = 2200000U;
  if (continuity.evaluate(observation, 2200000U, true, true) != ObservationContinuity::Result::kReady)
    return 42;  // Suppressed status cannot stop a live action.
  if (continuity.evaluate(observation, 2500000U, true, true) != ObservationContinuity::Result::kWait)
    return 43;  // A short state gap pauses admission without latching failure.
  observation.state_received_us = 2510000U;
  if (continuity.evaluate(observation, 2510000U, true, true) != ObservationContinuity::Result::kReady)
    return 44;
  if (continuity.evaluate(observation, 4900000U, true, true) != ObservationContinuity::Result::kWait ||
      continuity.evaluate(observation, 5400001U, true, true) != ObservationContinuity::Result::kLost)
    return 45;  // Sustained state loss remains fail closed.

  ObservationContinuity bootstrap(250000U, 2000000U, 500000U);
  observation.state_received_us = 1000000U;
  observation.status_received_us = 1000000U;
  if (bootstrap.evaluate(observation, 1000000U, false, false) != ObservationContinuity::Result::kWait ||
      bootstrap.evaluate(observation, 1100000U, false, true) != ObservationContinuity::Result::kReady)
    return 46;  // A transient status/state transition does not poison bootstrap.
  if (bootstrap.evaluate(observation, 1200000U, false, false) != ObservationContinuity::Result::kWait ||
      bootstrap.evaluate(observation, 3200001U, false, false) != ObservationContinuity::Result::kLost)
    return 47;
  ObservationContinuity cold(250000U, 2000000U, 500000U);
  observation.state_received_us = 3000000U;
  if (cold.evaluate(observation, 3000000U, false, true) != ObservationContinuity::Result::kWait ||
      cold.evaluate(observation, 5000001U, false, true) != ObservationContinuity::Result::kLost)
    return 48;  // Cold status must keep arriving.
  const StateAuthority expected{7U, 42U, 3U, 9U};
  if (!state_authority_is_valid(expected, expected, false, false) ||
      state_authority_is_valid(expected, expected, true, false)) return 49;
  if (state_authority_is_valid({8U,42U,3U,9U}, expected, false, false) ||
      state_authority_is_valid({7U,43U,3U,9U}, expected, false, false) ||
      state_authority_is_valid({7U,42U,4U,9U}, expected, false, false)) return 50;
  if (!state_authority_is_valid({7U,42U,3U,10U}, expected, false, false) ||
      state_authority_is_valid({7U,42U,3U,10U}, expected, false, true) ||
      state_authority_is_valid({7U,42U,3U,11U}, expected, false, false) ||
      state_authority_is_valid({7U,42U,3U,8U}, expected, false, false)) return 51;
  return 0;
}

int release_transition_tests() {
  DeviceObservation observation;
  observation.state_received_us = 1500000U;
  observation.status_received_us = 1000000U;
  ReleaseTransitionObservation transition{};
  transition.release_stage = 2;
  transition.original = {7U, 42U, 3U, 163U};
  transition.status = {7U, 42U, 3U, 164U};
  transition.state = {7U, 0U, 0U, 164U};
  transition.status_management_epoch = 254U;
  transition.state_management_epoch = 254U;
  transition.original_management_epoch = 253U;
  transition.status_core_state = 7U;
  transition.state_core_state = 1U;
  if (!release_idle_transition_pending(transition, observation, 1500000U, 250000U))
    return 52;  // The 500ms Status publication gap remains pending.
  if (release_idle_transition_pending(transition, observation, 1750001U, 250000U))
    return 53;  // An expired State must not extend the transition.
  if (release_idle_transition_pending(transition, observation, 2000001U, 250000U))
    return 60;  // A stale Status cannot hold the transition open.
  transition.release_stage = 1;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 54;
  transition.release_stage = 2;
  transition.state.boot_epoch = 8U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 55;
  transition.state.boot_epoch = 7U;
  transition.state_management_epoch = 255U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 56;
  transition.state_management_epoch = 254U;
  transition.status.session_id = 43U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 57;
  transition.status.session_id = 42U;
  transition.state_core_state = 9U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 58;
  transition.state_core_state = 1U;
  transition.status_core_state = 4U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 59;
  transition.status_core_state = 7U;
  transition.status.motion_epoch = transition.state.motion_epoch = 165U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 61;
  transition.status.motion_epoch = transition.state.motion_epoch = 164U;
  transition.status_management_epoch = transition.state_management_epoch = 255U;
  if (release_idle_transition_pending(transition, observation, 1500000U, 250000U)) return 62;
  ReleaseTransitionOrigin origin;
  origin.capture({7U, 42U, 3U, 163U}, 253U);
  const StateAuthority coherent_disabling{7U, 42U, 3U, 164U};
  const std::uint32_t updated_bootstrap_management_epoch = 254U;
  const auto delayed_idle = origin.observation(
      coherent_disabling, {7U, 0U, 0U, 164U},
      updated_bootstrap_management_epoch, 254U, 7U, 1U, 2);
  if (!release_idle_transition_pending(delayed_idle, observation, 1500000U, 250000U))
    return 63;  // Coherent Disabling must not rebase the release-start epoch.

  wire::DummyDeviceStatusV2 final_status;
  wire::DummyJointStateV2 final_state;
  final_status.set_boot_epoch(7U);
  final_status.set_session_id(0U);
  final_status.set_reference_epoch(0U);
  final_status.set_core_state(1U);
  final_status.set_motion_epoch(164U);
  final_status.set_management_epoch(254U);
  final_state.set_boot_epoch(7U);
  final_state.set_session_id(0U);
  final_state.set_reference_epoch(0U);
  final_state.set_core_state(1U);
  final_state.set_motion_epoch(164U);
  final_state.set_management_epoch(254U);
  if (released_idle_status_confirmed(final_status, final_state))
    return 64;  // A matching Idle Status without release_commit is not a terminal observation.
  final_status.set_release_committed(true);
  if (!released_idle_status_confirmed(final_status, final_state)) return 65;
  final_state.set_motion_epoch(165U);
  if (released_idle_status_confirmed(final_status, final_state)) return 66;
  final_state.set_motion_epoch(164U);
  final_status.set_permit_remaining_us(1U);
  if (released_idle_status_confirmed(final_status, final_state)) return 67;
  return 0;
}

int raw_first_page_snapshot_tests() {
  const DpManagementParameters facade_parameters{"tcp/127.0.0.1:29998", {1}, {2}, 2, 63};
  DpManagementTransport no_channel(facade_parameters,
      DpManagementTransport::Connection::kDeviceWorker);
  if (no_channel.raw_first_page_written().run_id != 0U) return 70;
  DpManagementTransport standalone(facade_parameters);
  if (standalone.raw_first_page_written().run_id != 0U) return 71;

  dummy_ros_control::TransportParameters parameters{
      facade_parameters.endpoint, facade_parameters.device, facade_parameters.client,
      facade_parameters.profile, facade_parameters.mask};
  dummy_ros_control::GuardedZenbeddedSystem system;
  auto provider = std::make_unique<dummy_ros_control::DeviceTransport>(system, parameters);
  auto channel = dummy_ros_control::DeviceTransport::management_channel(parameters);
  if (!channel) return 72;
  if (no_channel.raw_first_page_written().run_id != 0U) return 76;
  dummy_ros_control::RawEvidenceFirstPageWritten written{};
  written.boot_epoch = 7U;
  written.session_id = 8U;
  written.run_id = 9U;
  written.motion_epoch = 10U;
  written.evidence_seq = 0U;
  written.group_count = 1U;
  written.attempt_kind = 1U;
  written.category = 2U;
  written.expected_mask = 63U;
  written.submitted_mask = 63U;
  written.sent_mask = 63U;
  written.final = true;
  written.chain_valid = true;
  written.full_six = true;
  written.strict_refresh_successor = true;
  channel->publish_raw_evidence_first_page_written(written);
  const auto receipt = no_channel.raw_first_page_written();
  if (receipt.boot_epoch != 7U || receipt.session_id != 8U || receipt.run_id != 9U ||
      receipt.motion_epoch != 10U || receipt.evidence_seq != 0U ||
      receipt.group_count != 1U || receipt.attempt_kind != 1U || receipt.category != 2U ||
      receipt.expected_mask != 63U || receipt.submitted_mask != 63U ||
      receipt.sent_mask != 63U || receipt.failed_mask != 0U ||
      !receipt.final || receipt.evidence_lost || !receipt.chain_valid || !receipt.full_six ||
      !receipt.strict_refresh_successor)
    return 73;
  if (!raw_first_page_matches(receipt, 7U, 8U, 9U, 10U) ||
      raw_first_page_matches(receipt, 7U, 8U, 11U, 10U) ||
      raw_first_page_matches(receipt, 7U, 8U, 9U, 11U) ||
      raw_first_page_matches(receipt, 8U, 8U, 9U, 10U)) return 74;
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (std::uint64_t run = 10U; run < 10010U; ++run) {
      auto next = written;
      next.boot_epoch = run;
      next.session_id = run + 100U;
      next.run_id = run + 200U;
      next.motion_epoch = static_cast<std::uint32_t>(run);
      channel->publish_raw_evidence_first_page_written(next);
    }
    done.store(true, std::memory_order_release);
  });
  bool coherent = true;
  while (!done.load(std::memory_order_acquire)) {
    const auto current = no_channel.raw_first_page_written();
    if (current.run_id > 9U &&
        (current.session_id != current.boot_epoch + 100U ||
         current.run_id != current.boot_epoch + 200U ||
         current.motion_epoch != current.boot_epoch)) coherent = false;
  }
  writer.join();
  return coherent && no_channel.raw_first_page_written().run_id == 10209U ? 0 : 75;
}
}  // namespace

dp::comm::Result reply(const Service::Request &request, Service::Response *output,
           const dp::comm::RequestInfo &, void *context) {
  reply_calls.fetch_add(1, std::memory_order_relaxed);
  Service::Response response;
  response.set_schema_major(2); response.set_schema_minor(4U); response.set_request_id(request.request_id());
  response.set_device_id(request.device_id()); response.set_client_instance_id(request.client_instance_id());
  response.set_boot_epoch(request.expected_boot_epoch());
  response.set_session_id(request.expected_session_id());
  response.set_profile_id(request.expected_profile_id());
  response.set_receipt(1); response.set_transaction_phase(3); response.set_result_code(0);
  response.set_operation_id(88);
  *output=response;
  if (context != nullptr && request.request_id() == 0x20000000000002ULL)
    signal_peer(*static_cast<int *>(context), 'P');
  return dp::comm::Result::kOk;
}
int peer(const std::string &endpoint, int signal_fd, bool expect_rejection) {
  dp::comm::StaticStorage storage({1,0,0,0,8,wire::DummyDeviceStatusV2Codec::kEncodedSize,96,1,0,0,1});
  dp::comm::TransportConfig config{"peer","",endpoint.c_str(),"management_test"};
  auto init=dp::comm::Context::init(storage,{dp::comm::MemoryMode::kStaticOnly,&config,nullptr});
  if (!init.has_value()) return 10;
  auto context=std::move(init).take_value();
  dp::comm::NodeOptions options{}; options.product_name="dummy_robot";
  options.device_id.bytes[0]=1; options.node_name="ref_control";
  auto made=context.create_node(options); if(!made.has_value()) return 11;
  auto node=std::move(made).take_value();
  auto result=node.create_service<Service>("manage_v2",reply,expect_rejection ? nullptr : &signal_fd);
  if(!result.has_value()) return 12;
  auto service=std::move(result).take_value();
  dp::comm::StaticExecutor executor;
  if (executor.add_node(node) != dp::comm::Result::kOk || node.activate() != dp::comm::Result::kOk ||
      !signal_peer(signal_fd, 'R')) return 13;
  for (;;) {
    executor.spin_some(dp::comm::Duration::from_milliseconds(2));
    if (expect_rejection && (context.transport_drop_stats().protocol > 0 ||
                             reply_calls.load(std::memory_order_relaxed) > 0)) {
      const bool fail_closed = context.transport_drop_stats().protocol > 0 &&
          reply_calls.load(std::memory_order_relaxed) == 0;
      signal_peer(signal_fd, fail_closed ? 'D' : 'A');
      break;
    }
  }
  return 0;
}
int tcp_rejects_unauthenticated_request() {
  const std::string endpoint="tcp/127.0.0.1:"+std::to_string(24000+getpid()%10000);
  int signals[2];
  if (pipe(signals) != 0) return 20;
  const pid_t child = fork();
  if (child == 0) {
    close(signals[0]);
    const int result = peer(endpoint, signals[1], true);
    _exit(result);
  }
  close(signals[1]);
  if (child < 0) { close(signals[0]); return 21; }
  PeerProcess peer_guard(child);
  const bool peer_ready = read_peer_signal(signals[0], 'R', 3000);
  if (!peer_ready) { close(signals[0]); return 22; }
  DpManagementParameters config{endpoint,{1},{2},1,1};
  DpManagementTransport transport(config);
  if (!transport.start() || !wait_ready(transport)) { close(signals[0]); return 23; }
  const auto bytes = request_bytes();
  if (bytes.empty()) { close(signals[0]); return 26; }
  auto handle = transport.send(bytes);
  if (!handle) { close(signals[0]); return 24; }
  const bool rejected = read_peer_signal(signals[0], 'D', 3000);
  close(signals[0]);
  return rejected ? 0 : 25;
}

int serial_roundtrip() {
  PtyBridge bridge;
  if (!bridge.open_pair()) return 30;
  int signals[2];
  if (pipe(signals) != 0) return 31;
  const pid_t child = fork();
  if (child == 0) {
    close(signals[0]);
    bridge.close_child_fds();
    const int result = peer(bridge.left_endpoint(), signals[1], false);
    _exit(result);
  }
  close(signals[1]);
  if (child < 0) { close(signals[0]); return 32; }
  SignalReader signal_reader{signals[0]};
  PeerProcess peer_guard(child);
  bridge.start();
  if (!read_peer_signal(signal_reader.fd, 'R', 3000)) return 33;
  DpManagementParameters config{bridge.right_endpoint(),{1},{2},1,1};
  DpManagementTransport transport(config);
  StopBridgeBeforeTransport stop_bridge{bridge};
  if(!transport.start()) return 1;
  if(!wait_ready(transport)) return 2;
  const auto prime_bytes = request_bytes(0x20000000000002ULL);
  if (prime_bytes.empty()) return 37;
  auto prime = transport.send(prime_bytes);
  if (!prime) return 38;
  if (!read_peer_signal(signal_reader.fd, 'P', 5000)) return 39;
  prime.reset();
  const auto bytes = request_bytes();
  if (bytes.empty()) return 34;
  auto handle=transport.send(bytes); if(!handle) return 3;
  std::optional<std::vector<std::uint8_t>> response;
  const auto deadline = Clock::now() + std::chrono::seconds(4);
  while (!response && Clock::now() < deadline) {
    const auto before=std::chrono::steady_clock::now(); response=handle->poll();
    if(std::chrono::steady_clock::now()-before>std::chrono::milliseconds(20)) return 4;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if(!response) return 5;
  Service::Response decoded;
  if(wire::DummyManageV2ResponseCodec::decode(response->data(),response->size(),&decoded)!=wire::CodecResult::kOk ||
     decoded.request_id()!=0x20000000000001ULL || decoded.operation_id()!=88) return 6;
  dp::comm::StaticStorage second_storage({1,0,0,0,8,618,96});
  const dp::comm::TransportConfig second_config{"client",bridge.right_endpoint().c_str(),"","second_context"};
  const auto second=dp::comm::Context::init(second_storage,{dp::comm::MemoryMode::kStaticOnly,&second_config,nullptr});
  if(second.result()!=dp::comm::Result::kBusy) return 8;
  // Destroying an outstanding facade cancels on the worker, never on a caller thread.
  handle=transport.send(bytes); if (!handle) return 35;
  handle.reset();
  handle=transport.send(bytes); if (!handle) return 36;
  bridge.stop(); transport.stop(); handle.reset();
  return 0;
}

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "raw_first_page")
    return raw_first_page_snapshot_tests();
  if (argc == 2 && std::string(argv[1]) == "continuity") return observation_continuity_tests();
  if (argc == 2 && std::string(argv[1]) == "release_transition") return release_transition_tests();
  if (argc == 2 && std::string(argv[1]) == "tcp") {
    const int tcp_result = tcp_rejects_unauthenticated_request();
    if (tcp_result != 0) return tcp_result;
    std::cout << "{\"evidence_scope\":\"component_only\",\"tcp_untrusted_rpc_fail_closed\":true}\n";
    return 0;
  }
  if (argc != 1) return 40;
  const int serial_result = serial_roundtrip();
  if (serial_result != 0) return serial_result;
  std::cout << "{\"evidence_scope\":\"component_only\",\"cross_process_manage_v2\":true,\"poll_nonblocking\":true}\n";
}
