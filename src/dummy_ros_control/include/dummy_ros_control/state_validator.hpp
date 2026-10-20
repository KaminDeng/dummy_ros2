/**
 * @file state_validator.hpp
 * @brief 对统一时钟域内的状态身份、序号、有效轴及时间进行纯校验。
 * StateEnvelope的produced_at_us来自源设备；调用者必须像DeviceTransport那样建立接收减生产时差锚，构造同域expected.now_us，不能直接拿宿主墙钟减设备启动时间。位置rad与速度rad/s的数值有限性由硬件ingest另查。
 */
#pragma once

#include <cstdint>

namespace dummy_ros_control {

/**
 * @brief 最小状态包络：boot/profile/32位序号及生产µs时间，配置/位置/速度有效掩码按六轴位索引。
 */
struct StateEnvelope {
  std::uint64_t boot = 0;
  std::uint32_t profile = 0;
  std::uint32_t state_seq = 0;
  std::uint64_t produced_at_us = 0;
  std::uint8_t configured_mask = 0;
  std::uint8_t position_valid_mask = 0;
  std::uint8_t velocity_valid_mask = 0;
};

/**
 * @brief 期望身份和上一状态序号；request_started_at_us/now_us/max_age_us均在生产者对齐域，单位µs。has_last_state_seq决定是否检查序号。
 */
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

/**
 * @brief 可审计拒绝类别；仅位置掩码在validate_state强制完整，速度掩码交给后续层解释。
 */
enum class StateValidity {
  kValid, kWrongIdentity, kOldSequence, kInvalidAxis, kStale,
  kBeforeRequestAnchor, kFuture
};

/**
 * @brief 按32位串行算术比较有界跨度的新旧序号。
 * @param value 新序号。
 * @param previous 上一序号。
 * @return bool：模差非零且小于2^31；相等和半圈以上均拒绝。
 */
inline bool sequence_newer(std::uint32_t value, std::uint32_t previous) noexcept {
  const auto difference = static_cast<std::uint32_t>(value - previous);
  return difference != 0U && difference < 0x80000000U;
}

/**
 * @brief 按固定优先级校验状态包络。
 * 先身份和序号，再有效位置轴、未来时间、请求锚及年龄；年龄恰等max_age_us仍有效。此函数不授权运动，不检查速度值或设备session。
 * @param state 待校验的状态元数据。
 * @param expected 同域时间、配置身份和请求/序号水位。
 * @return StateValidity：首个失败原因或kValid。
 */
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
