/**
 * @file wire_probe_tests.cpp
 * @brief 检查wire固定布局、整数精度、管理语义字段与本地service桥。
 *
 * 所有消息为合成夹具；编码/roundtrip成功不表示远端已接受命令或完成管理事务。固定size和接口ID的static_assert只在编译时检查生成类型形状。
 */
#include <cstdint>
#include <cstring>

#include "dummy_ref/wire_contract.hpp"

namespace {
using ManageService = dp::protocol::services::DummyManageV2;

/**
 * @brief 处理进程内monitor_snapshot服务并回显请求来源身份。
 * @par 未命名管理请求参数
 * 本用例不依赖其正文。
 * @param response 调用方提供的可写wire响应。
 * @param info dp_comm桥传入的请求来源元数据，用source_device填client ID。
 * @par 未命名上下文参数
 * void*上下文在本夹具中不使用。
 * @return kOk；写入的phase/run/session等仅是固定响应夹具。
 */
dp::comm::Result monitor_snapshot(
    const ManageService::Request &, ManageService::Response *response,
    const dp::comm::RequestInfo &info, void *) {
  std::uint8_t caller[16]{};
  std::memcpy(caller, info.source_device.bytes, sizeof(caller));
  response->set_schema_major(2U);
  response->set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response->set_client_instance_id(caller);
  response->set_receipt(1U);
  response->set_result_code(0U);
  response->set_operation_id(9U);
  response->set_transaction_phase(3U);
  response->set_session_id(9007199254740995ULL);
  return dp::comm::Result::kOk;
}
}  // namespace

/**
 * @brief 验证当前wire roundtrip、异常长度不覆盖输出和来源桥接。
 *
 * 身份/ticket刻意大于2^53，J6=3πrad；必须保留uint64与展开角度。Queued响应夹具填receipt=1/phase=1/result=8，但只断言选定解码字段，未检查result_code；不能把夹具填值当作完整roundtrip证明或Accepted/Committed。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  namespace wire = dp::iface;
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
  command.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
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
  /**
   * @brief encode可承载minor=0原文，但decode必须拒绝不匹配schema。
   * 因此编码Ok不能单独当作完整schema/业务验证通过。
   */
  command.set_schema_minor(0U);
  if (wire::DummyJointCommandV2Codec::encode(command, command_bytes, sizeof(command_bytes)) != wire::CodecResult::kOk ||
      wire::DummyJointCommandV2Codec::decode(command_bytes, sizeof(command_bytes), &decoded_command) == wire::CodecResult::kOk)
    return 14;

  /**
   * @brief Status夹具包含能力位、策略hash、rad/rad/s/rad/s²限制及µs周期；下面只断言选定的hash/限额/周期等字段，未检查解码后的capabilities。
   */
  wire::DummyDeviceStatusV2 status;
  status.set_schema_major(2U);
  status.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
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
      decoded_status.schema_minor() != dummy_ref::protocol::kSchemaMinor || decoded_status.compiled_policy_sha256()[0] != 1U ||
      decoded_status.position_min_rad()[0] != -1.0 || decoded_status.position_max_rad()[5] != 1.0 ||
      decoded_status.velocity_max_rad_s()[2] != 1.0 || decoded_status.acceleration_max_rad_s()[4] != 2.0 ||
      decoded_status.control_period_us() != 10000U || decoded_status.state_publish_period_us() != 10000U) return 13;

  wire::DummyManageV2Response queued;
  queued.set_schema_major(2U);
  queued.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
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
  evidence.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
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

  /**
   * @brief raw-fixed service_invoke使用人工authenticated来源元数据。
   * 回调回显source_device[0]=0xB2；这检查桥是否传递RequestInfo，不执行真实认证握手。
   */
  wire::DummyManageV2Request monitor;
  monitor.set_schema_major(2U);
  monitor.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
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

  /**
   * @brief 截短一字节应解码失败且保持原输出boot=42，不能留下半解码对象。
   */
  decoded_command.set_boot_epoch(42U);
  if (wire::DummyJointCommandV2Codec::decode(command_bytes, sizeof(command_bytes) - 1U, &decoded_command) == wire::CodecResult::kOk ||
      decoded_command.boot_epoch() != 42U) return 12;
  return 0;
}
