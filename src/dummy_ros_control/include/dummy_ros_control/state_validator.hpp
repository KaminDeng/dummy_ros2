#pragma once

#include <cstdint>

namespace dummy_ros_control {

struct StateEnvelope {
  std::uint64_t boot = 0;
  std::uint32_t profile = 0;
  std::uint32_t state_seq = 0;
  std::uint64_t produced_at_us = 0;
  std::uint8_t configured_mask = 0;
  std::uint8_t position_valid_mask = 0;
  std::uint8_t velocity_valid_mask = 0;
};

struct StateExpectation {
  std::uint64_t boot = 0;
  std::uint32_t profile = 0;
  std::uint32_t last_state_seq = 0;
  bool has_last_state_seq = false;
  std::uint8_t configured_mask = 0;
  std::uint64_t request_started_at_us = 0;
  std::uint64_t now_us = 0;
  std::uint64_t max_age_us = 0;
};

enum class StateValidity {
  kValid, kWrongIdentity, kOldSequence, kInvalidAxis, kStale,
  kBeforeRequestAnchor, kFuture
};

inline bool sequence_newer(std::uint32_t value, std::uint32_t previous) noexcept {
  const auto difference = static_cast<std::uint32_t>(value - previous);
  return difference != 0U && difference < 0x80000000U;
}

inline StateValidity validate_state(const StateEnvelope &state,
                                    const StateExpectation &expected) noexcept {
  if (state.boot != expected.boot || state.profile != expected.profile ||
      state.configured_mask != expected.configured_mask) return StateValidity::kWrongIdentity;
  if (expected.has_last_state_seq && !sequence_newer(state.state_seq, expected.last_state_seq))
    return StateValidity::kOldSequence;
  if ((state.position_valid_mask & expected.configured_mask) != expected.configured_mask)
    return StateValidity::kInvalidAxis;
  if (state.produced_at_us > expected.now_us) return StateValidity::kFuture;
  if (state.produced_at_us < expected.request_started_at_us)
    return StateValidity::kBeforeRequestAnchor;
  if (expected.now_us - state.produced_at_us > expected.max_age_us) return StateValidity::kStale;
  return StateValidity::kValid;
}

}  // namespace dummy_ros_control
