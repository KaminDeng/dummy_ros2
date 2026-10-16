#include <cstdint>
#include <cstring>

#include "dp_dummy_robot_v2_dp_comm.hpp"

namespace {
using ManageService = dp::protocol::services::DummyManageV2RawFixed;

dp::comm::Result monitor_snapshot(
    const ManageService::Request &, ManageService::Response *response,
    const dp::comm::RequestInfo &info, void *) {
  std::uint8_t caller[16]{};
  std::memcpy(caller, info.source_device.bytes, sizeof(caller));
  response->set_schema_major(2U);
  response->set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  response->set_client_instance_id(caller);
  response->set_receipt(1U);
  response->set_result_code(0U);
  response->set_operation_id(9U);
  response->set_transaction_phase(3U);
  response->set_session_id(9007199254740995ULL);
  return dp::comm::Result::kOk;
}
}  // namespace

int main() {
  namespace wire = dp::protocol::dummy_robot_v2;
  static_assert(wire::DummyJointCommandV2Codec::kEncodedSize == 133U);
  static_assert(wire::DummyJointStateV2Codec::kEncodedSize == 350U);
  static_assert(wire::DummyDeviceStatusV2Codec::kEncodedSize == 651U);
  static_assert(wire::DummyAttemptEvidenceV2Codec::kEncodedSize == 372U);
  static_assert(wire::DummyManageV2RequestCodec::kEncodedSize == 233U);
  static_assert(wire::DummyManageV2ResponseCodec::kEncodedSize == 132U);
  static_assert(dp::comm::MessageCodec<wire::DummyManageV2Request>::kInterfaceId == 4141U);
  static_assert(dp::comm::MessageCodec<wire::DummyManageV2Response>::kInterfaceId == 4142U);

  wire::DummyJointCommandV2 command;
  command.set_schema_major(2U);
  command.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  command.set_boot_epoch(9007199254740993ULL);
  command.set_session_id(9007199254740995ULL);
  command.set_freshness_ticket(9007199254740997ULL);
  command.set_configured_mask(0x3FU);
  const double positions[6]{0.1, -0.2, 0.7, -0.4, 0.5, 9.42477796076938};
  command.set_position_rad(positions);
  std::uint8_t command_bytes[wire::DummyJointCommandV2Codec::kEncodedSize]{};
  wire::DummyJointCommandV2 decoded_command;
  if (wire::DummyJointCommandV2Codec::encode(command, command_bytes, sizeof(command_bytes)) != wire::CodecResult::kOk) return 1;
  if (wire::DummyJointCommandV2Codec::decode(command_bytes, sizeof(command_bytes), &decoded_command) != wire::CodecResult::kOk) return 2;
  if (decoded_command.boot_epoch() != 9007199254740993ULL ||
      decoded_command.session_id() != 9007199254740995ULL ||
      decoded_command.freshness_ticket() != 9007199254740997ULL ||
      decoded_command.position_rad()[5] != positions[5]) return 3;
  command.set_schema_minor(0U);
  if (wire::DummyJointCommandV2Codec::encode(command, command_bytes, sizeof(command_bytes)) != wire::CodecResult::kOk ||
      wire::DummyJointCommandV2Codec::decode(command_bytes, sizeof(command_bytes), &decoded_command) == wire::CodecResult::kOk)
    return 14;

  wire::DummyDeviceStatusV2 status;
  status.set_schema_major(2U);
  status.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  status.set_configured_mask(0x3FU);
  status.set_capabilities(1ULL << 2U);
  status.set_control_period_us(10000U);
  status.set_state_publish_period_us(10000U);
  std::uint8_t policy[32]{};
  policy[0] = 1U;
  status.set_compiled_policy_sha256(policy);
  const double position_min[6]{-1, -1, -1, -1, -1, -1};
  const double position_max[6]{1, 1, 1, 1, 1, 1};
  const double velocity_max[6]{1, 1, 1, 1, 1, 1};
  const double acceleration_max[6]{2, 2, 2, 2, 2, 2};
  status.set_position_min_rad(position_min);
  status.set_position_max_rad(position_max);
  status.set_velocity_max_rad_s(velocity_max);
  status.set_acceleration_max_rad_s(acceleration_max);
  std::uint8_t status_bytes[wire::DummyDeviceStatusV2Codec::kEncodedSize]{};
  wire::DummyDeviceStatusV2 decoded_status;
  if (wire::DummyDeviceStatusV2Codec::encode(status, status_bytes, sizeof(status_bytes)) != wire::CodecResult::kOk ||
      wire::DummyDeviceStatusV2Codec::decode(status_bytes, sizeof(status_bytes), &decoded_status) != wire::CodecResult::kOk ||
      decoded_status.schema_minor() != dp::protocol::dummy_robot_v2::kSchemaMinor || decoded_status.compiled_policy_sha256()[0] != 1U ||
      decoded_status.position_min_rad()[0] != -1.0 || decoded_status.position_max_rad()[5] != 1.0 ||
      decoded_status.velocity_max_rad_s()[2] != 1.0 || decoded_status.acceleration_max_rad_s()[4] != 2.0 ||
      decoded_status.control_period_us() != 10000U || decoded_status.state_publish_period_us() != 10000U) return 13;

  wire::DummyManageV2Response queued;
  queued.set_schema_major(2U);
  queued.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  queued.set_request_id(9007199254740999ULL);
  queued.set_receipt(1U);
  queued.set_result_code(8U);
  queued.set_transaction_phase(1U);
  std::uint8_t response_bytes[wire::DummyManageV2ResponseCodec::kEncodedSize]{};
  wire::DummyManageV2Response decoded_response;
  if (wire::DummyManageV2ResponseCodec::encode(queued, response_bytes, sizeof(response_bytes)) != wire::CodecResult::kOk) return 4;
  if (wire::DummyManageV2ResponseCodec::decode(response_bytes, sizeof(response_bytes), &decoded_response) != wire::CodecResult::kOk) return 5;
  if (decoded_response.request_id() != 9007199254740999ULL ||
      decoded_response.receipt() != 1U || decoded_response.transaction_phase() != 1U) return 6;

  wire::DummyAttemptEvidenceV2 evidence;
  evidence.set_schema_major(2U);
  evidence.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  evidence.set_category(1U);
  evidence.set_run_id(9007199254740993ULL);
  evidence.set_evidence_seq(17U);
  evidence.set_expected_mask(1U);
  evidence.set_evidence_lost(true);
  std::uint8_t evidence_bytes[wire::DummyAttemptEvidenceV2Codec::kEncodedSize]{};
  wire::DummyAttemptEvidenceV2 decoded_evidence;
  if (wire::DummyAttemptEvidenceV2Codec::encode(evidence, evidence_bytes, sizeof(evidence_bytes)) != wire::CodecResult::kOk ||
      wire::DummyAttemptEvidenceV2Codec::decode(evidence_bytes, sizeof(evidence_bytes), &decoded_evidence) != wire::CodecResult::kOk ||
      decoded_evidence.run_id() != evidence.run_id() || decoded_evidence.evidence_seq() != 17U ||
      !decoded_evidence.evidence_lost()) return 15;

  wire::DummyManageV2Request monitor;
  monitor.set_schema_major(2U);
  monitor.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  monitor.set_operation(0U);
  std::uint8_t monitor_request[wire::DummyManageV2RequestCodec::kEncodedSize]{};
  if (wire::DummyManageV2RequestCodec::encode(monitor, monitor_request, sizeof(monitor_request)) != wire::CodecResult::kOk) return 8;
  std::uint8_t monitor_response[wire::DummyManageV2ResponseCodec::kEncodedSize]{};
  std::uint16_t monitor_response_size = 0U;
  dp::comm::RequestInfo request_info{};
  request_info.authenticated = true;
  request_info.source_session = 7U;
  request_info.source_device.bytes[0] = 0xB2U;
  dp::comm::ServiceCallback<ManageService> callback = monitor_snapshot;
  if (dp::comm::service_invoke<ManageService>(
          monitor_request, sizeof(monitor_request), &callback, nullptr, request_info,
          monitor_response, sizeof(monitor_response), &monitor_response_size, nullptr) != dp::comm::Result::kOk) return 9;
  wire::DummyManageV2Response snapshot;
  if (wire::DummyManageV2ResponseCodec::decode(monitor_response, monitor_response_size, &snapshot) != wire::CodecResult::kOk) return 10;
  if (snapshot.client_instance_id()[0] != 0xB2U || snapshot.operation_id() != 9U ||
      snapshot.transaction_phase() != 3U || snapshot.result_code() != 0U ||
      snapshot.session_id() != 9007199254740995ULL) return 11;

  decoded_command.set_boot_epoch(42U);
  if (wire::DummyJointCommandV2Codec::decode(command_bytes, sizeof(command_bytes) - 1U, &decoded_command) == wire::CodecResult::kOk ||
      decoded_command.boot_epoch() != 42U) return 12;
  return 0;
}
