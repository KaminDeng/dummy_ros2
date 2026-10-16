#include <array>
#include <cstdint>

#include "dp_dummy_robot_v2.hpp"

namespace wire = dp::protocol::dummy_robot_v2;

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
