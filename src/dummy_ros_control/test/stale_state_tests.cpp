#include "dummy_ros_control/state_validator.hpp"
#include "dummy_ros_control/freshness.hpp"

int main() {
  using dummy_ros_control::StateEnvelope;
  using dummy_ros_control::StateExpectation;
  using dummy_ros_control::StateValidity;
  const StateExpectation expected{9U, 7U, 100U, true, 0x3FU, 1000U, 1100U, 150U};
  const StateEnvelope valid{9U, 7U, 101U, 1001U, 0x3FU, 0x3FU};
  if (dummy_ros_control::validate_state(valid, expected) != StateValidity::kValid) return 1;
  auto state = valid;
  state.state_seq = 99U;
  if (dummy_ros_control::validate_state(state, expected) != StateValidity::kOldSequence) return 2;
  state = valid;
  state.boot = 8U;
  if (dummy_ros_control::validate_state(state, expected) != StateValidity::kWrongIdentity) return 3;
  state = valid;
  state.position_valid_mask = 0x01U;
  if (dummy_ros_control::validate_state(state, expected) != StateValidity::kInvalidAxis) return 4;
  state = valid;
  state.produced_at_us = 949U;
  if (dummy_ros_control::validate_state(state, expected) != StateValidity::kBeforeRequestAnchor) return 5;
  state = valid;
  state.produced_at_us = 900U;
  StateExpectation old_request = expected;
  old_request.request_started_at_us = 0U;
  if (dummy_ros_control::validate_state(state, old_request) != StateValidity::kStale) return 6;
  if (!dummy_ros_control::sequence_newer(0U, 0xFFFFFFFFU)) return 7;
  if (dummy_ros_control::sequence_newer(0x80000000U, 0U)) return 8;
  if (!dummy_ros_control::ticket_valid(0U, 49U, 50U)) return 9;
  if (dummy_ros_control::ticket_valid(0U, 50U, 50U)) return 10;
  if (dummy_ros_control::ticket_valid(0U, 51U, 50U)) return 11;
  if (dummy_ros_control::ticket_valid(UINT64_MAX - 10U, UINT64_MAX, 50U)) return 12;
  return 0;
}
