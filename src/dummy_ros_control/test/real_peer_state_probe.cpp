#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "dp_dummy_robot_v1.hpp"
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
      namespace wire = dp::protocol::dummy_robot_v1;
      wire::DummyJointStateV1 state;
      wire::DummyDeviceStatusV1 status;
      if (wire::DummyJointStateV1Codec::decode(observation->state.data(), observation->state.size(), &state) ==
              wire::CodecResult::kOk &&
          wire::DummyDeviceStatusV1Codec::decode(observation->status.data(), observation->status.size(), &status) ==
              wire::CodecResult::kOk && status.core_state() == 9U &&
          (state.raw_position_valid_mask() & (1U << (missing_axis - 1U))) == 0U) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  transport.stop();
  if (!observation) return 5;
  namespace wire = dp::protocol::dummy_robot_v1;
  wire::DummyJointStateV1 state;
  wire::DummyDeviceStatusV1 status;
  if (wire::DummyJointStateV1Codec::decode(observation->state.data(), observation->state.size(), &state) !=
          wire::CodecResult::kOk ||
      wire::DummyDeviceStatusV1Codec::decode(observation->status.data(), observation->status.size(), &status) !=
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
  return valid ? 0 : 7;
}
