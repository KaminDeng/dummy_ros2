#include <array>
#include <cstdint>

#include "dummy_ros_control/device_transport.hpp"

int main() {
  using namespace dummy_ros_control;
  PositionOnlyCapability expected{};
  expected.configured_mask = 0x3fU;
  expected.capabilities = kPositionOnlyCapability;
  expected.control_timebase_us = 10000U;
  expected.state_publish_period_us = 10000U;
  expected.policy_hash[0] = 1U;
  expected.contract_hash[0] = 2U;
  expected.model_hash[0] = 3U;
  expected.profile_hash[0] = 4U;
  expected.firmware_hash[0] = 5U;
  for (auto &limit : expected.limits) limit = {-1.0, 1.0};
  expected.velocity_max.fill(1.0);
  expected.acceleration_max.fill(2.0);

  auto observed = expected;
  if (!position_only_capability_matches(expected, observed)) return 1;
  expected.configured_mask = 0x01U;
  observed = expected;
  if (!position_only_capability_matches(expected, observed)) return 12;
  expected.configured_mask = 0x3fU;
  observed = expected;
  observed.firmware_hash[0] = 6U;
  if (position_only_capability_matches(expected, observed)) return 2;
  observed = expected;
  observed.has_policy_hash = false;
  if (position_only_capability_matches(expected, observed)) return 3;
  observed = expected;
  observed.control_timebase_us = 20000U;
  if (position_only_capability_matches(expected, observed)) return 4;
  observed = expected;
  observed.velocity_max[0] = 0.0;
  if (position_only_capability_matches(expected, observed)) return 5;

  const std::array<double, 6> valid{0.0, 0.5, -0.5, 1.0, -1.0, 0.25};
  if (!position_only_positions_valid(valid, expected)) return 6;
  auto out_of_limit = valid;
  out_of_limit[3] = 1.01;
  if (position_only_positions_valid(out_of_limit, expected)) return 7;
  auto missing_axis = expected;
  missing_axis.configured_mask = 0x03U;
  if (position_only_positions_valid(valid, missing_axis)) return 8;

  TransportHeartbeat heartbeat(10000U);
  heartbeat.observe_state(100U);
  if (heartbeat.poll(20100U) != HeartbeatAction::kWarn) return 9;
  if (heartbeat.poll(50100U) != HeartbeatAction::kStop) return 10;
  heartbeat.observe_state(50200U);
  if (heartbeat.poll(50200U) != HeartbeatAction::kNone) return 11;
  return 0;
}
