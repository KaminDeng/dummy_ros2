#include <cstdint>

#include "dp_dummy_robot_v1_dp_comm.hpp"

int main() {
  namespace wire = dp::protocol::dummy_robot_v1;
  static_assert(wire::DummyJointCommandV1Codec::kEncodedSize == 125U);
  static_assert(wire::DummyJointStateV1Codec::kEncodedSize == 356U);
  static_assert(wire::DummyDeviceStatusV1Codec::kEncodedSize == 297U);
  static_assert(wire::DummyManageV1RequestCodec::kEncodedSize == 233U);
  static_assert(wire::DummyManageV1ResponseCodec::kEncodedSize == 618U);
  static_assert(dp::comm::MessageCodec<wire::DummyManageV1Request>::kInterfaceId == 4135U);
  static_assert(dp::comm::MessageCodec<wire::DummyManageV1Response>::kInterfaceId == 4136U);

  wire::DummyJointCommandV1 command;
  command.set_schema_major(1U);
  command.set_schema_minor(0U);
  command.set_boot_epoch(9007199254740993ULL);
  command.set_session_id(9007199254740995ULL);
  command.set_freshness_ticket(9007199254740997ULL);
  command.set_configured_mask(0x3FU);
  const double positions[6]{0.1, -0.2, 0.7, -0.4, 0.5, 9.42477796076938};
  command.set_position_rad(positions);
  std::uint8_t command_bytes[wire::DummyJointCommandV1Codec::kEncodedSize]{};
  wire::DummyJointCommandV1 decoded_command;
  if (wire::DummyJointCommandV1Codec::encode(command, command_bytes, sizeof(command_bytes)) != wire::CodecResult::kOk) return 1;
  if (wire::DummyJointCommandV1Codec::decode(command_bytes, sizeof(command_bytes), &decoded_command) != wire::CodecResult::kOk) return 2;
  if (decoded_command.boot_epoch() != 9007199254740993ULL ||
      decoded_command.session_id() != 9007199254740995ULL ||
      decoded_command.freshness_ticket() != 9007199254740997ULL ||
      decoded_command.position_rad()[5] != positions[5]) return 3;

  wire::DummyManageV1Response queued;
  queued.set_schema_major(1U);
  queued.set_schema_minor(0U);
  queued.set_request_id(9007199254740999ULL);
  queued.set_receipt(1U);
  queued.set_accepted(false);
  queued.set_operation_result_valid(false);
  std::uint8_t response_bytes[wire::DummyManageV1ResponseCodec::kEncodedSize]{};
  wire::DummyManageV1Response decoded_response;
  if (wire::DummyManageV1ResponseCodec::encode(queued, response_bytes, sizeof(response_bytes)) != wire::CodecResult::kOk) return 4;
  if (wire::DummyManageV1ResponseCodec::decode(response_bytes, sizeof(response_bytes), &decoded_response) != wire::CodecResult::kOk) return 5;
  if (decoded_response.request_id() != 9007199254740999ULL || decoded_response.accepted() ||
      decoded_response.operation_result_valid()) return 6;

  decoded_command.set_boot_epoch(42U);
  if (wire::DummyJointCommandV1Codec::decode(command_bytes, sizeof(command_bytes) - 1U, &decoded_command) == wire::CodecResult::kOk ||
      decoded_command.boot_epoch() != 42U) return 7;
  return 0;
}
