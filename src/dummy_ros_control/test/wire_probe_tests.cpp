#include <cstdint>
#include <cstring>

#include "dp_dummy_robot_v1_dp_comm.hpp"

namespace {
using ManageService = dp::protocol::services::DummyManageV1RawFixed;

dp::comm::Result monitor_snapshot(
    const ManageService::Request &, ManageService::Response *response,
    const dp::comm::RequestInfo &info, void *) {
  std::uint8_t caller[16]{};
  std::uint8_t owner[16]{};
  std::memcpy(caller, info.source_device.bytes, sizeof(caller));
  owner[0] = 0xA1U;
  response->set_schema_major(1U);
  response->set_schema_minor(0U);
  response->set_client_instance_id(caller);
  response->set_owner_client_instance_id(owner);
  response->set_snapshot_valid(true);
  response->set_session_id(9007199254740995ULL);
  return dp::comm::Result::kOk;
}
}  // namespace

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

  wire::DummyManageV1Request monitor;
  monitor.set_schema_major(1U);
  monitor.set_schema_minor(0U);
  monitor.set_operation(0U);
  std::uint8_t monitor_request[wire::DummyManageV1RequestCodec::kEncodedSize]{};
  if (wire::DummyManageV1RequestCodec::encode(monitor, monitor_request, sizeof(monitor_request)) != wire::CodecResult::kOk) return 8;
  std::uint8_t monitor_response[wire::DummyManageV1ResponseCodec::kEncodedSize]{};
  std::uint16_t monitor_response_size = 0U;
  dp::comm::RequestInfo request_info{};
  request_info.authenticated = true;
  request_info.source_session = 7U;
  request_info.source_device.bytes[0] = 0xB2U;
  dp::comm::ServiceCallback<ManageService> callback = monitor_snapshot;
  if (dp::comm::service_invoke<ManageService>(
          monitor_request, sizeof(monitor_request), &callback, nullptr, request_info,
          monitor_response, sizeof(monitor_response), &monitor_response_size) != dp::comm::Result::kOk) return 9;
  wire::DummyManageV1Response snapshot;
  if (wire::DummyManageV1ResponseCodec::decode(monitor_response, monitor_response_size, &snapshot) != wire::CodecResult::kOk) return 10;
  if (!snapshot.snapshot_valid() || snapshot.client_instance_id()[0] != 0xB2U ||
      snapshot.owner_client_instance_id()[0] != 0xA1U ||
      snapshot.session_id() != 9007199254740995ULL) return 11;

  decoded_command.set_boot_epoch(42U);
  if (wire::DummyJointCommandV1Codec::decode(command_bytes, sizeof(command_bytes) - 1U, &decoded_command) == wire::CodecResult::kOk ||
      decoded_command.boot_epoch() != 42U) return 12;
  return 0;
}
