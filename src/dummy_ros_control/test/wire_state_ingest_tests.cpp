#include <array>
#include <cmath>
#include <cstdint>

#include "dp_dummy_robot_v1.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"

int main() {
  namespace wire = dp::protocol::dummy_robot_v1;
  wire::DummyJointStateV1 state;
  state.set_schema_major(1U);
  state.set_schema_minor(0U);
  state.set_boot_epoch(9U);
  state.set_profile_id(7U);
  state.set_state_seq(101U);
  state.set_produced_at_us(1001U);
  state.set_configured_mask(0x3FU);
  state.set_position_valid_mask(0x3FU);
  state.set_velocity_valid_mask(0x01U);
  const double position[6]{0.1, 0.2, 0.7, 0.4, 0.5, 9.42477796076938};
  const double velocity[6]{0.01, 0.0, 0.0, 0.0, 0.0, 0.0};
  state.set_position_rad(position);
  state.set_estimated_velocity_rad_s(velocity);
  std::array<std::uint8_t, wire::DummyJointStateV1Codec::kEncodedSize> bytes{};
  if (wire::DummyJointStateV1Codec::encode(state, bytes.data(), bytes.size()) != wire::CodecResult::kOk)
    return 1;
  dummy_ros_control::GuardedZenbeddedSystem system;
  const dummy_ros_control::StateExpectation expected{9U, 7U, 100U, true, 0x3FU, 1000U, 1100U, 150U};
  if (!system.ingest_wire_state(bytes.data(), bytes.size(), expected)) return 2;
  dummy_ros_control::JointStateSnapshot snapshot{};
  if (!system.copy_state(&snapshot) || snapshot.position[5] != position[5] ||
      snapshot.velocity[0] != velocity[0] || !std::isnan(snapshot.velocity[1])) return 3;
  if (system.ingest_wire_state(bytes.data(), bytes.size() - 1U, expected)) return 4;
  auto wrong = expected;
  wrong.boot = 10U;
  if (system.ingest_wire_state(bytes.data(), bytes.size(), wrong)) return 5;
  dummy_ros_control::JointStateSnapshot unchanged{};
  if (!system.copy_state(&unchanged) || unchanged.envelope.boot != 9U || unchanged.position[5] != position[5])
    return 6;
  auto stale = expected;
  stale.request_started_at_us = 0U;
  stale.now_us = 1200U;
  if (system.ingest_wire_state(bytes.data(), bytes.size(), stale)) return 7;
  const double invalid_velocity[6]{NAN, 0.0, 0.0, 0.0, 0.0, 0.0};
  state.set_estimated_velocity_rad_s(invalid_velocity);
  if (wire::DummyJointStateV1Codec::encode(state, bytes.data(), bytes.size()) != wire::CodecResult::kOk)
    return 8;
  if (system.ingest_wire_state(bytes.data(), bytes.size(), expected)) return 9;
  if (!dummy_ros_control::GuardedZenbeddedSystem::active_joint_count_supported(1U) ||
      !dummy_ros_control::GuardedZenbeddedSystem::active_joint_count_supported(6U) ||
      dummy_ros_control::GuardedZenbeddedSystem::active_joint_count_supported(0U) ||
      dummy_ros_control::GuardedZenbeddedSystem::active_joint_count_supported(2U)) return 10;
  if (system.on_error(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS ||
      !system.take_stop_request() || system.take_stop_request()) return 11;
  return 0;
}
