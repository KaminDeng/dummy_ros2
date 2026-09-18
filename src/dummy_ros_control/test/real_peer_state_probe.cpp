#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "dp_dummy_robot_v2.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

int main(int argc, char **argv) {
  if (argc != 4 && argc != 5) return 2;
  const auto profile = static_cast<std::uint32_t>(std::strtoul(argv[2], nullptr, 10));
  const auto mask = static_cast<std::uint8_t>(std::strtoul(argv[3], nullptr, 10));
  const auto missing_axis = argc == 5 ? static_cast<std::uint8_t>(std::strtoul(argv[4], nullptr, 10)) : 0U;
  if (profile == 0U || (mask != 1U && mask != 63U)) return 3;
  if (missing_axis > 6U || (missing_axis != 0U && (mask & (1U << (missing_axis - 1U))) == 0U)) return 3;
  dummy_ros_control::TransportParameters parameters{};
  parameters.endpoint = argv[1];
  parameters.device_id[0] = 1U;
  parameters.client_instance_id[0] = 2U;
  parameters.profile_id = profile;
  parameters.configured_mask = mask;
  parameters.max_state_age_us = 1000000U;
  const auto parse_hash = [](const char *text, std::array<std::uint8_t, 32> *out) {
    for (std::size_t i = 0; i < out->size(); ++i) {
      unsigned value = 0U;
      if (std::sscanf(text + i * 2U, "%2x", &value) != 1) return false;
      (*out)[i] = static_cast<std::uint8_t>(value);
    }
    return true;
  };
  parse_hash("daaea5b72d4d38e3b71873b5c4db5f7272d0a9e708aa9f43ecffbf28aae59b0f", &parameters.contract_hash);
  parse_hash("a2ed010f4a5e3d4981d805519e6d209b27e71ebb56b7479fb8be076f85425713", &parameters.model_hash);
  parse_hash("9678a26c2614e02e715ca037d8c5512e6536b44fdf91385d96b50145a852373e", &parameters.profile_hash);
  parse_hash("aa9e703cb390518d01e2ae46af1b84989ad33a70f60e7b16c9ec11c89bce67dc", &parameters.firmware_hash);
  parse_hash("9678a26c2614e02e715ca037d8c5512e6536b44fdf91385d96b50145a852373e", &parameters.policy_hash);
  parameters.position_min = {-3.2, -1.2740903539558606, 0.6108652381980153,
                             -3.141592653589793, -2.0943951023931953, -12.566370614359172};
  parameters.position_max = {3.2, 1.5707963267948966, 3.141592653589793,
                             3.141592653589793, 2.0943951023931953, 12.566370614359172};
  parameters.velocity_max.fill(0.5);
  parameters.acceleration_max.fill(1.0);
  parameters.control_period_us = 10000U;
  parameters.state_publish_period_us = 50000U;
  dummy_ros_control::GuardedZenbeddedSystem system;
  dummy_ros_control::DeviceTransport transport(system, parameters);
  if (!transport.start()) return 4;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(7);
  std::optional<dummy_ros_control::DeviceManagementObservation> observation;
  const auto channel = dummy_ros_control::DeviceTransport::management_channel(parameters);
  while (std::chrono::steady_clock::now() < deadline) {
    if (channel) {
      std::lock_guard<std::mutex> lock(channel->mutex);
      if (channel->observation.state_received_us != 0U &&
          channel->observation.status_received_us != 0U)
        observation = channel->observation;
    }
    if (observation && missing_axis == 0U) break;
    if (observation && missing_axis != 0U) {
      namespace wire = dp::protocol::dummy_robot_v2;
      wire::DummyJointStateV2 state;
      wire::DummyDeviceStatusV2 status;
      if (wire::DummyJointStateV2Codec::decode(observation->state.data(), observation->state.size(), &state) ==
              wire::CodecResult::kOk &&
          wire::DummyDeviceStatusV2Codec::decode(observation->status.data(), observation->status.size(), &status) ==
              wire::CodecResult::kOk && status.core_state() == 9U &&
          (state.raw_position_valid_mask() & (1U << (missing_axis - 1U))) == 0U) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!observation) {
    const auto statistics = transport.statistics();
    std::cerr << "transport_timeout states=" << statistics.state_messages
              << " statuses=" << statistics.status_messages
              << " rejected_states=" << statistics.rejected_states
              << " bootstrap_rejections=" << statistics.bootstrap_rejections
              << " invalidations=" << statistics.invalidations
              << " ready=" << statistics.ready
              << " initialization_failed=" << statistics.initialization_failed << '\n';
    transport.stop();
    return 5;
  }
  namespace wire = dp::protocol::dummy_robot_v2;
  wire::DummyJointStateV2 state;
  wire::DummyDeviceStatusV2 status;
  if (wire::DummyJointStateV2Codec::decode(observation->state.data(), observation->state.size(), &state) !=
          wire::CodecResult::kOk ||
      wire::DummyDeviceStatusV2Codec::decode(observation->status.data(), observation->status.size(), &status) !=
          wire::CodecResult::kOk)
    return 6;
  const auto missing_mask = missing_axis == 0U ? 0U : static_cast<std::uint8_t>(1U << (missing_axis - 1U));
  const bool cold_valid = (state.raw_position_valid_mask() & mask) == mask &&
      (state.position_valid_mask() & mask) == 0U && state.session_id() == 0U;
  const bool fault_valid = missing_axis != 0U && status.core_state() == 9U &&
      (state.raw_position_valid_mask() & missing_mask) == 0U && state.configured_mask() == mask;
  const bool valid = state.profile_id() == profile && status.profile_id() == profile &&
      state.configured_mask() == mask && status.configured_mask() == mask &&
      (missing_axis == 0U ? cold_valid : fault_valid);
  std::cout << "{\"profile_id\":" << profile << ",\"configured_mask\":"
            << static_cast<unsigned>(mask) << ",\"raw_valid_mask\":"
            << static_cast<unsigned>(state.raw_position_valid_mask())
            << ",\"position_valid_mask\":" << static_cast<unsigned>(state.position_valid_mask())
            << ",\"session_id\":" << state.session_id()
            << ",\"core_state\":" << status.core_state()
            << ",\"missing_axis\":" << static_cast<unsigned>(missing_axis)
            << ",\"auto_degraded\":false}\n";
  std::cout.flush();
  transport.stop();
  return valid ? 0 : 7;
}
