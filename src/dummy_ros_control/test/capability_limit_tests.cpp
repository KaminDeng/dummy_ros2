#include <array>
#include <cstdint>

#include "dummy_ros_control/device_transport.hpp"

int main() {
  using namespace dummy_ros_control;
  PositionOnlyCapability expected{};
  expected.configured_mask = 0x3fU;
  expected.capabilities = kPositionOnlyCapability;
  expected.control_timebase_us = 10000U;
  expected.policy_hash[0] = 1U;
  expected.contract_hash[0] = 2U;
  expected.model_hash[0] = 3U;
  expected.profile_hash[0] = 4U;
  expected.firmware_hash[0] = 5U;
  for (auto &limit : expected.limits) limit = {-1.0, 1.0};

  auto observed = expected;
  if (!position_only_capability_matches(expected, observed)) return 1;
  observed.firmware_hash[0] = 6U;
  if (position_only_capability_matches(expected, observed)) return 2;
  observed = expected;
  observed.has_policy_hash = false;
  if (position_only_capability_matches(expected, observed)) return 3;
  observed = expected;
  observed.control_timebase_us = 20000U;
  if (position_only_capability_matches(expected, observed)) return 4;

  const std::array<double, 6> valid{0.0, 0.5, -0.5, 1.0, -1.0, 0.25};
  if (!position_only_positions_valid(valid, expected)) return 5;
  auto out_of_limit = valid;
  out_of_limit[3] = 1.01;
  if (position_only_positions_valid(out_of_limit, expected)) return 6;
  auto missing_axis = expected;
  missing_axis.configured_mask = 0x03U;
  if (position_only_positions_valid(valid, missing_axis)) return 7;

  TransportHeartbeat heartbeat(10000U);
  heartbeat.observe_state(100U);
  if (heartbeat.poll(20100U) != HeartbeatAction::kWarn) return 8;
  if (heartbeat.poll(50100U) != HeartbeatAction::kStop) return 9;
  heartbeat.observe_state(50200U);
  if (heartbeat.poll(50200U) != HeartbeatAction::kNone) return 10;
  return 0;
}
