/**
 * @file management_request_minor_tests.cpp
 * @brief 核WireManagementClient自动填写当前minor和permit绑定前提。
 *
 * 本地Transport只解码保存原请求，Handle永远Pending；成功submit仅本地准入，不代表远端接纳或运动许可。assert须启用。
 */
#include "dummy_motion_client/management_client.hpp"
#include <algorithm>
#include <cassert>

namespace {
namespace wire = dp::iface;

/**
 * @brief 保存最后请求及是否成功解出当前schema minor的单线程夹具。
 */
struct Transport : dummy_motion_client::ManagementTransport {
  /**
   * @brief 永远无响应的最小句柄，不伪造管理终态。
   */
  struct Handle : RequestHandle {
    /**
     * @brief 维持本地请求未完成。
     * @return 空optional，无服务结果。
     */
    std::optional<std::vector<std::uint8_t>> poll() override { return {}; }
  };

  /**
   * @brief send实际解码成功且minor==kSchemaMinor才置true；不是文件名推断。
   */
  bool decoded_current_minor = false;
  /**
   * @brief 最后解码请求副本，用于核实际发出的request_seq及permit摘要。
   */
  dummy_motion_client::ManageRequest sent{};

  /**
   * @brief 解码并保存提交bytes，只有当前minor可创建Handle。
   * @param bytes 完整管理请求。
   * @return 成功Handle，否则nullptr；不运行RPC。
   */
  std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &bytes) override {
    dummy_motion_client::ManageRequest request;
    decoded_current_minor = wire::DummyManageV2RequestCodec::decode(
        bytes.data(), bytes.size(), &request) == wire::CodecResult::kOk &&
        request.schema_minor() == dummy_ref::protocol::kSchemaMinor;
    sent = request;
    return decoded_current_minor ? std::make_unique<Handle>() : nullptr;
  }
};
}  // namespace

/**
 * @brief permit字段逐项补齐前不得消耗首请求序号。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 缺compiled policy、operator确认或permit时长均submit0；完整后首seq仍1且摘要用实际发送seq计算。binding换boot后拒绝。普通默认request也必须由client补当前minor。
 */
int main() {
  Transport transport;
  dummy_motion_client::WireManagementClient client(transport);
  dummy_motion_client::ManageRequest request;
  if (client.submit(request, 123U) == 0U || !transport.decoded_current_minor) return 1;
  // A second client gets the same first sequence after SetPermit material is bound.
  Transport permit_transport;
  dummy_motion_client::WireManagementClient permit_client(permit_transport);
  dummy_motion_client::PermitBinding binding{};
  binding.boot_epoch = 7U; binding.reference_epoch = 3U; binding.profile_id = 101U;
  binding.firmware_sha256[0] = 1U; binding.contract_sha256[0] = 2U;
  binding.model_sha256[0] = 3U; binding.profile_sha256[0] = 4U;
  std::uint8_t owner[16]{}; owner[0] = 0x55U;
  request.set_client_instance_id(owner);
  request.set_operation(dummy_motion_client::wire_value(dummy_motion_client::ManageOperation::kSetPermit));
  request.set_expected_boot_epoch(7U); request.set_expected_session_id(9U);
  request.set_expected_reference_epoch(3U); request.set_expected_profile_id(101U);
  assert(permit_client.submit_permit(request, binding, 456U) == 0U);
  std::uint8_t policy[32]{}; policy[0] = 4U;
  request.set_compiled_policy_sha256(policy);
  assert(permit_client.submit_permit(request, binding, 456U) == 0U);
  request.set_operator_confirmed(true);
  assert(permit_client.submit_permit(request, binding, 456U) == 0U);
  request.set_permit_remaining_us(1000000U);
  assert(permit_client.submit_permit(request, binding, 456U) != 0U);
  std::array<std::uint8_t, 16> client_id{}; client_id[0] = 0x55U;
  std::array<std::uint8_t, 32> expected{};
  assert(dummy_motion_client::trusted_bench_permit_digest(binding, client_id, 9U,
         permit_transport.sent.request_seq(), &expected));
  assert(std::equal(expected.begin(), expected.end(), permit_transport.sent.permit_sha256()));
  assert(permit_transport.sent.request_seq() == 1U);
  binding.boot_epoch = 8U;
  Transport stale_transport;
  dummy_motion_client::WireManagementClient stale_client(stale_transport);
  assert(stale_client.submit_permit(request, binding, 456U) == 0U);
  return 0;
}
