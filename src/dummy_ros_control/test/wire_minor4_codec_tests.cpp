/**
 * @file wire_minor4_codec_tests.cpp
 * @brief 检查当前schema minor编码与旧minor解码拒绝。
 *
 * 文件名保留历史minor4称呼；实际构造使用kSchemaMinor，并篡改为当前minor-1。此测试不更改合同或生成协议文件。
 */
#include <array>
#include <cstdint>

#include "dp_dummy_robot_v2.hpp"

namespace wire = dp::protocol::dummy_robot_v2;

/**
 * @brief 为指定消息/codec建立当前minor夹具后篡改minor字节。
 * @tparam Message 具有schema setter的当前wire消息。
 * @tparam Codec 对应固定长度编码器。
 * @return 当前消息编码成功且旧minor被decode拒绝为true。
 */
template <typename Message, typename Codec>
bool rejects_old_minor() {
  Message message;
  message.set_schema_major(2U);
  message.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  std::array<std::uint8_t, Codec::kEncodedSize> bytes{};
  if (Codec::encode(message, bytes.data(), bytes.size()) != wire::CodecResult::kOk)
    return false;
  bytes[2] = static_cast<std::uint8_t>(wire::kSchemaMinor - 1U);
  return Codec::decode(bytes.data(), bytes.size(), &message) != wire::CodecResult::kOk;
}

/**
 * @brief 覆盖命令、状态、管理请求/响应及Evidence的minor门槛。
 *
 * Evidence额外检查category、final_state、terminal_reason和固定偏移304/305；这仅证明字节布局/解码门槛，不提供业务授权或设备状态证明。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  if (!rejects_old_minor<wire::DummyJointCommandV2, wire::DummyJointCommandV2Codec>() ||
      !rejects_old_minor<wire::DummyDeviceStatusV2, wire::DummyDeviceStatusV2Codec>() ||
      !rejects_old_minor<wire::DummyManageV2Request, wire::DummyManageV2RequestCodec>() ||
      !rejects_old_minor<wire::DummyManageV2Response, wire::DummyManageV2ResponseCodec>())
    return 8;
  wire::DummyJointStateV2 state;
  state.set_schema_major(2U);
  state.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  std::array<std::uint8_t, wire::DummyJointStateV2Codec::kEncodedSize> state_bytes{};
  if (wire::DummyJointStateV2Codec::encode(state, state_bytes.data(), state_bytes.size()) != wire::CodecResult::kOk)
    return 1;
  wire::DummyJointStateV2 decoded_state;
  if (wire::DummyJointStateV2Codec::decode(state_bytes.data(), state_bytes.size(), &decoded_state) != wire::CodecResult::kOk)
    return 2;
  state_bytes[2] = static_cast<std::uint8_t>(wire::kSchemaMinor - 1U);
  if (wire::DummyJointStateV2Codec::decode(state_bytes.data(), state_bytes.size(), &decoded_state) == wire::CodecResult::kOk)
    return 3;

  wire::DummyAttemptEvidenceV2 evidence;
  evidence.set_schema_major(2U);
  evidence.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  evidence.set_category(4U);
  evidence.set_final(1U);
  evidence.set_final_state(8U);
  evidence.set_terminal_reason(7U);
  std::array<std::uint8_t, wire::DummyAttemptEvidenceV2Codec::kEncodedSize> bytes{};
  static_assert(wire::DummyAttemptEvidenceV2Codec::kEncodedSize == 372U);
  if (wire::DummyAttemptEvidenceV2Codec::encode(evidence, bytes.data(), bytes.size()) != wire::CodecResult::kOk)
    return 4;
  if (bytes[304] != 8U || bytes[305] != 7U) return 5;
  wire::DummyAttemptEvidenceV2 decoded;
  if (wire::DummyAttemptEvidenceV2Codec::decode(bytes.data(), bytes.size(), &decoded) != wire::CodecResult::kOk ||
      decoded.category() != 4U || decoded.final_state() != 8U || decoded.terminal_reason() != 7U)
    return 6;
  bytes[2] = static_cast<std::uint8_t>(wire::kSchemaMinor - 1U);
  if (wire::DummyAttemptEvidenceV2Codec::decode(bytes.data(), bytes.size(), &decoded) == wire::CodecResult::kOk)
    return 7;
  return 0;
}
