#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <unistd.h>
#include <thread>

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

namespace {
namespace wire = dp::protocol::dummy_robot_v2;
using State = wire::DummyJointStateV2;
using Status = wire::DummyDeviceStatusV2;
using Command = wire::DummyJointCommandV2;
using namespace dummy_ros_control;
struct Stimulus { Status status{}; State state{}; bool done = false; bool send_status = true; };
struct Peer { int commands_fd; };
void receive_command(dp::comm::BorrowedMessage<Command> borrowed, void *context) {
  Command message;
  if (borrowed.copy_to(&message) == dp::comm::Result::kOk)
    (void)::write(static_cast<Peer *>(context)->commands_fd, &message, sizeof(message));
}
int run_peer(const std::string &endpoint, int input, int output) {
  dp::comm::StaticStorage storage({1U, 2U, 1U, 0U, 8U, 618U, 96U});
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
  Peer peer{output};
  auto subscribed = node.create_subscription<Command>("command_v2", {}, receive_command, &peer);
  if (!states.has_value() || !statuses.has_value() || !subscribed.has_value()) return 3;
  auto state_publisher = std::move(states).take_value();
  auto status_publisher = std::move(statuses).take_value();
  auto subscription = std::move(subscribed).take_value();
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
      if ((stimulus.send_status && status_publisher.publish(stimulus.status) != dp::comm::Result::kOk) ||
          state_publisher.publish(stimulus.state) != dp::comm::Result::kOk) return 7;
    }
    if (executor.spin_some() != dp::comm::Result::kOk) return 8;
  }
}
template <typename Predicate> bool wait_for(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!predicate() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  return predicate();
}
int run_parent(const std::string &endpoint, int input, int output, bool six_axis) {
  pollfd response{input, POLLIN, 0};
  char ready{};
  if (::poll(&response, 1, 3000) != 1 || ::read(input, &ready, 1) != 1 || ready != 'R') return 10;
  TransportParameters parameters{};
  parameters.endpoint = endpoint;
  parameters.device_id[0] = 1U;
  parameters.client_instance_id[0] = 2U;
  parameters.profile_id = 1U;
  parameters.configured_mask = six_axis ? 63U : 1U;
  parameters.max_state_age_us = 1000000U;
  GuardedZenbeddedSystem system;
  DeviceTransport transport(system, parameters);
  if (!transport.start() || !wait_for([&] { return transport.statistics().ready; })) return 11;
  Stimulus valid{};
  valid.status.set_schema_major(2U);
  valid.state.set_schema_major(2U);
  std::uint8_t device[16]{1U};
  valid.status.set_device_id(device);
  valid.state.set_device_id(device);
  valid.status.set_boot_epoch(7U);
  valid.state.set_boot_epoch(7U);
  valid.status.set_session_id(8U);
  valid.state.set_session_id(8U);
  valid.status.set_reference_epoch(9U);
  valid.state.set_reference_epoch(9U);
  valid.status.set_motion_epoch(10U);
  valid.state.set_motion_epoch(10U);
  valid.status.set_profile_id(1U);
  valid.state.set_profile_id(1U);
  valid.status.set_configured_mask(parameters.configured_mask);
  valid.state.set_configured_mask(parameters.configured_mask);
  valid.state.set_position_valid_mask(parameters.configured_mask);
  valid.state.set_state_seq(1U);
  const double positions[6]{0.25, six_axis ? 0.2 : NAN, six_axis ? 0.3 : NAN,
                           six_axis ? 0.4 : NAN, six_axis ? 0.5 : NAN, six_axis ? 0.6 : NAN};
  valid.state.set_position_rad(positions);
  auto send = [&](Stimulus stimulus) {
    stimulus.state.set_produced_at_us(transport_steady_now_us());
    return ::write(output, &stimulus, sizeof(stimulus)) == sizeof(stimulus);
  };
  JointStateSnapshot snapshot{};
  auto unbound = valid;
  unbound.send_status = false;
  if (!wait_for([&] { send(unbound); return transport.statistics().rejected_states != 0U; }) ||
      system.copy_state(&snapshot)) return 48;
  if (!wait_for([&] { send(valid); return system.copy_state(&snapshot); })) return 12;
  if (snapshot.position[0] != 0.25 ||
      (six_axis ? snapshot.position[1] != 0.2 : !std::isnan(snapshot.position[1]))) return 13;
  if (system.on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) return 14;
  set_command_identity({7U, 8U, 9U, 10U, 9007199254740997ULL});
  const auto generation = publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr);
  if (!generation || !system.transfer_control_update()) return 15;
  if (::poll(&response, 1, 3000) != 1) return 16;
  Command command;
  if (::read(input, &command, sizeof(command)) != sizeof(command)) return 17;
  if (command.device_id()[0] != 1U || command.client_instance_id()[0] != 2U ||
      command.boot_epoch() != 7U || command.session_id() != 8U ||
      command.reference_epoch() != 9U || command.motion_epoch() != 10U || command.configured_mask() != parameters.configured_mask ||
      command.profile_id() != 1U || command.producer_seq() != generation ||
      command.freshness_ticket() != 9007199254740997ULL || command.position_rad()[0] != 0.25 ||
      (six_axis ? command.position_rad()[1] != 0.2 : command.position_rad()[1] != 0.0) ||
      !std::all_of(std::begin(command.position_rad()), std::end(command.position_rad()),
                   [](double value) { return std::isfinite(value); })) return 17;
  // Replayed sequence, wrong device/profile and missing position evidence cannot replace feedback.
  auto invalid = valid;
  const double wrong_position[6]{0.75, NAN, NAN, NAN, NAN, NAN};
  invalid.state.set_position_rad(wrong_position);
  const auto rejects = transport.statistics().rejected_states;
  send(invalid);
  if (!wait_for([&] { return transport.statistics().rejected_states > rejects; }) ||
      !system.copy_state(&snapshot) || snapshot.position[0] != 0.25) return 18;
  for (unsigned fault = 0U; fault < 9U; ++fault) {
    auto wrong = valid;
    wrong.state.set_state_seq(2U);
    wrong.state.set_produced_at_us(transport_steady_now_us());
    if (fault == 0U) { std::uint8_t other[16]{3U}; wrong.state.set_device_id(other); }
    if (fault == 1U) wrong.state.set_profile_id(2U);
    if (fault == 2U) wrong.state.set_configured_mask(six_axis ? 1U : 63U);
    if (fault == 3U) wrong.state.set_produced_at_us(transport_steady_now_us() + 2000000U);
    if (fault == 4U) wrong.state.set_produced_at_us(transport_steady_now_us() - 2000000U);
    if (fault == 5U) wrong.state.set_session_id(99U);
    if (fault == 6U) wrong.state.set_boot_epoch(99U);
    if (fault == 7U) wrong.state.set_reference_epoch(99U);
    if (fault == 8U) wrong.state.set_motion_epoch(99U);
    const auto before = transport.statistics().rejected_states;
    if (::write(output, &wrong, sizeof(wrong)) != sizeof(wrong) ||
        !wait_for([&] { return transport.statistics().rejected_states > before; }) ||
        !system.copy_state(&snapshot) || snapshot.position[0] != 0.25) return 21 + fault;
  }
  // A status authority transition must immediately invalidate old state and commands.
  // State carrying the previous authority cannot repopulate the snapshot.
  for (unsigned changed_field = 0U; changed_field < 3U; ++changed_field) {
    auto transition = valid;
    if (changed_field == 0U) transition.status.set_session_id(18U);
    if (changed_field == 1U) transition.status.set_reference_epoch(19U);
    if (changed_field == 2U) transition.status.set_motion_epoch(20U);
    transition.state.set_state_seq(2U);
    const auto before = transport.statistics().invalidations;
    if (!send(transition) || !wait_for([&] { return transport.statistics().invalidations > before; }) ||
        system.copy_state(&snapshot) || publish_control_update(positions, six_axis ? 6U : 1U, 0, nullptr) != 0U)
      return 40 + changed_field;
    valid.status = transition.status;
    valid.state.set_session_id(valid.status.session_id());
    valid.state.set_reference_epoch(valid.status.reference_epoch());
    valid.state.set_motion_epoch(valid.status.motion_epoch());
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
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  (void)system.copy_state(&snapshot);
  auto rebooted = valid;
  rebooted.status.set_boot_epoch(11U);
  rebooted.state.set_boot_epoch(11U);
  rebooted.state.set_position_valid_mask(0U);
  const auto invalidations = transport.statistics().invalidations;
  send(rebooted);
  if (!wait_for([&] { return transport.statistics().invalidations > invalidations; }) ||
      system.copy_state(&snapshot) || publish_control_update(positions, 1U, 0, nullptr) != 0U) return 19;
  send(valid);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  if (system.copy_state(&snapshot)) return 20;
  transport.stop();
  std::printf("component_only: synthetic_peer axes=%u full identity, device/profile/mask/age/sequence rejection, authority/boot invalidation PASS\n",
              six_axis ? 6U : 1U);
  return 0;
}
}  // namespace
int main(int argc, char **) {
  const int server = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  socklen_t length = sizeof(address);
  if (server < 0 || ::bind(server, reinterpret_cast<sockaddr *>(&address), length) != 0 ||
      ::getsockname(server, reinterpret_cast<sockaddr *>(&address), &length) != 0) return 30;
  const auto endpoint = std::string("tcp/127.0.0.1:") + std::to_string(ntohs(address.sin_port));
  ::close(server);
  int requests[2]{}, responses[2]{};
  if (::pipe(requests) != 0 || ::pipe(responses) != 0) return 31;
  const auto child = ::fork();
  if (child < 0) return 32;
  if (child == 0) {
    ::close(requests[1]); ::close(responses[0]);
    ::_exit(run_peer(endpoint, requests[0], responses[1]));
  }
  ::close(requests[0]); ::close(responses[1]);
  const auto result = run_parent(endpoint, responses[0], requests[1], argc == 2);
  if (result != 0) std::fprintf(stderr, "component_only assertion failed: %d\n", result);
  ::kill(child, SIGTERM);
  ::waitpid(child, nullptr, 0);
  ::close(requests[1]); ::close(responses[0]);
  return result;
}
