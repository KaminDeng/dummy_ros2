/**
 * @file wire_management_tests.cpp
 * @brief 真实管理codec/客户端与内存FIFO传输的协议、续租及终态边界测试。
 *
 * 不创建真实transport端点；请求编码解码走生产codec，响应由测试显式构造。Port只计数，Host时间人工µs。
 * Queued≠Accepted≠Committed；Query重试保留原请求身份/时间锚，续租不能挤掉Complete/Cancel或延长原终态期限。输出component证据不等于HIL/物理完成。
 */
#include "dummy_motion_client/management_client.hpp"
#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_motion_client/bootstrap_tick_policy.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include <deque>
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace dummy_motion_client;
namespace wire=dp::iface;
/**
 * @brief 编解码内存链路，保存请求并以FIFO提供合成响应。
 * timeout_next只使下一poll抛异常，reject_send使send返回空句柄。
 */
struct Transport : ManagementTransport {
  bool timeout_next=false;
  bool reject_send=false;
  std::vector<ManageRequest> requests;
  std::deque<std::vector<std::uint8_t>> responses;
  /**
   * @brief 借用Transport的请求句柄；Transport必须比句柄长寿。
   */
  struct Handle : RequestHandle {
    /**
     * @brief 保存传输引用，无新线程。
     * @param transport 所借用FIFO对象。
     */
    explicit Handle(Transport &transport) : transport(transport) {}
    Transport &transport;
    /**
     * @brief 非阻塞取FIFO首响应，或注入一次timeout异常。
     * @return 字节向量或空；不按句柄单独路由，生产WireManagementClient负责相关性。
     */
    std::optional<std::vector<std::uint8_t>> poll() override {
      if(transport.timeout_next) { transport.timeout_next=false; throw std::runtime_error("simulated query timeout"); }
      if (transport.responses.empty()) return {};
      auto result=transport.responses.front(); transport.responses.pop_front(); return result;
    }
  };
  /**
   * @brief 解码并记录请求，建立借用本Transport的Handle。
   * @param bytes 完整Manage请求wire字节。
   * @return 新句柄；reject_send时空，decode失败抛异常。
   */
  std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &bytes) override {
    if(reject_send) return {};
    ManageRequest request;
    if (wire::DummyManageV2RequestCodec::decode(bytes.data(),bytes.size(),&request)!=wire::CodecResult::kOk)
      throw std::runtime_error("request codec failed");
    requests.push_back(request); return std::make_unique<Handle>(*this);
  }
  /**
   * @brief 编码一份合成响应并排入FIFO。
   * @param response 由用例构造的wire响应。
   * @return 无；codec失败抛异常。
   */
  void respond(const wire::DummyManageV2Response &response) {
    std::vector<std::uint8_t> bytes(wire::DummyManageV2ResponseCodec::kEncodedSize);
    if (wire::DummyManageV2ResponseCodec::encode(response,bytes.data(),bytes.size())!=wire::CodecResult::kOk)
      throw std::runtime_error("response codec failed");
    responses.push_back(bytes);
  }
};
/**
 * @brief 非执行Port，primes/starts只累计接口调用次数。
 */
struct Port : ActionPort {
  int primes=0, starts=0;
  /**
   * @brief 未命名位置rad/epoch/generation不执行，只计prime。
   * @return 无。
   */
  void prime(const Position &,std::uint32_t,std::uint64_t) override { ++primes; }
  /**
   * @brief 未命名轨迹/generation不执行，只计start。
   * @return 无。
   */
  void start(const Trajectory &,std::uint64_t) override { ++starts; }
  /**
   * @brief 空失能，不向设备写管理请求。
   * @return 无。
   */
  void disable() override {}
  /**
   * @brief 空取消，不构成设备停止确认。
   * @return 无。
   */
  void cancel() override {}
};
/**
 * @brief Enable只Queued不能prime；管理Committed仍需新State Hold及已提交seq，双流证据齐备才ready。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool two_stream_handshake() {
  Transport transport; WireManagementClient client(transport); Port port;
  ActionGate gate(client,port,{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}});
  Authorization auth; auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_session_id(9); auth.identity.set_expected_boot_epoch(7);
  auth.identity.set_expected_profile_id(1); auth.identity.set_configured_mask(1);
  if (!gate.enable(auth,100)) return false;
  wire::DummyManageV2Response receipt; receipt.set_schema_major(2); receipt.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  receipt.set_request_id(1);
  receipt.set_boot_epoch(7); receipt.set_session_id(9);
  receipt.set_profile_id(1); receipt.set_receipt(1); receipt.set_operation_id(101);
  receipt.set_transaction_phase(1);
  transport.respond(receipt); gate.tick(101); if (port.primes) return false;
  wire::DummyJointStateV2 state; state.set_schema_major(2); state.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  state.set_boot_epoch(7);
  state.set_session_id(9); state.set_profile_id(1); state.set_state_seq(1);
  state.set_motion_epoch(1); state.set_configured_mask(1); state.set_position_valid_mask(1);
  state.set_hold_ready(true); state.set_core_state(3);
  auto observe=[&] {
    std::vector<std::uint8_t> bytes(wire::DummyJointStateV2Codec::kEncodedSize);
    if (wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())!=wire::CodecResult::kOk) return false;
    return gate.observe_state(bytes.data(),bytes.size(),102);
  };
  if (!observe() || port.primes!=1 || gate.ready()) return false;
  if (!gate.record_update(gate.generation(),1,1,Position{})) return false;
  receipt.set_request_id(transport.requests.back().request_id());
  receipt.set_result_code(0); receipt.set_transaction_phase(3);
  // Management completion and periodic HOLD evidence arrive independently.
  transport.respond(receipt); gate.tick(103); if (gate.ready()) return false;
  state.set_state_seq(2); state.set_committed_producer_seq(1);
  state.set_core_state(4); state.set_hold_ready(true);
  state.set_freshness_ticket(55);
  if(!observe() || !gate.ready() || observe()) return false;
  const Trajectory motion{{"Joint1"},{{{0.0},{},{},1.0}},{},{},0};
  return gate.submit(motion,104) && transport.requests.back().freshness_ticket()==55;
}
/**
 * @brief 不同process seed或同seed不同invocation构造的客户端ID不交叉；这里只模拟构造值，不fork进程。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool process_sessions_do_not_reuse_management_identity() {
  Transport first_transport, second_transport;
  WireManagementClient first(first_transport, 0x123456789abcdef0ULL);
  WireManagementClient second(second_transport, 0x123456789abcdef1ULL);
  ManageRequest select;
  select.set_operation(wire_value(ManageOperation::kSelectMode));
  select.set_expected_session_id(0U);
  std::array<std::uint64_t,8> first_ids{}, second_ids{};
  const auto issue=[&](WireManagementClient &client,Transport &transport,
                       std::array<std::uint64_t,8> &ids) {
    for (std::size_t index=0;index<ids.size();++index) {
      ids[index]=client.submit(select,index+1U);
      if (ids[index]==0U || transport.requests.back().request_seq()==1U) return false;
      wire::DummyManageV2Response response;
      response.set_schema_major(2U); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
      response.set_request_id(ids[index]); response.set_result_code(0U);
      response.set_receipt(wire_value(WireReceipt::kQueued));
      response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
      transport.respond(response);
      const auto result=client.poll(ids[index]);
      if (!result || !result->succeeded) return false;
    }
    return true;
  };
  if (!issue(first,first_transport,first_ids) || !issue(second,second_transport,second_ids)) return false;
  for (const auto first_id:first_ids)
    for (const auto second_id:second_ids)
      if (first_id==second_id) return false;
  Transport reused_first_transport, reused_second_transport;
  WireManagementClient reused_first(reused_first_transport,0x123456789abcdef0ULL,101U);
  WireManagementClient reused_second(reused_second_transport,0x123456789abcdef0ULL,102U);
  std::array<std::uint64_t,8> reused_first_ids{}, reused_second_ids{};
  if (!issue(reused_first,reused_first_transport,reused_first_ids) ||
      !issue(reused_second,reused_second_transport,reused_second_ids)) return false;
  for (const auto first_id:reused_first_ids)
    for (const auto second_id:reused_second_ids)
      if (first_id==second_id) return false;
  return first_transport.requests.front().request_seq()!=
      second_transport.requests.front().request_seq();
}
/**
 * @brief request_seq从UINT32_MAX回到1，跳过保留值0。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool sequence_wrap_skips_zero() {
  Transport transport;
  WireManagementClient client(transport,0x12345678fffffffeULL);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kSelectMode));
  const auto first=client.submit(request,1U);
  if (first==0U || transport.requests.back().request_seq()!=UINT32_MAX) return false;
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(first); response.set_result_code(0U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  transport.respond(response);
  if (!client.poll(first)) return false;
  const auto second=client.submit(request,2U);
  return second!=0U && transport.requests.back().request_seq()==1U;
}
/**
 * @brief 回绕permit必须用实际已发送seq1参与摘要，不能用旧MAX或未发送预测号。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool permit_wrap_uses_sent_sequence() {
  Transport transport;
  WireManagementClient client(transport,0x12345678fffffffeULL);
  ManageRequest select;
  select.set_operation(wire_value(ManageOperation::kSelectMode));
  const auto first=client.submit(select,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(first); response.set_result_code(0U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  transport.respond(response);
  if (!client.poll(first)) return false;
  PermitBinding binding{};
  binding.boot_epoch=7U; binding.reference_epoch=3U; binding.profile_id=201U;
  binding.firmware_sha256[0]=1U; binding.contract_sha256[0]=2U;
  binding.model_sha256[0]=3U; binding.profile_sha256[0]=4U;
  ManageRequest permit;
  permit.set_operation(wire_value(ManageOperation::kSetPermit));
  permit.set_expected_boot_epoch(binding.boot_epoch);
  permit.set_expected_session_id(9U);
  permit.set_expected_reference_epoch(binding.reference_epoch);
  permit.set_expected_profile_id(binding.profile_id);
  permit.set_operator_confirmed(true); permit.set_permit_remaining_us(1000000U);
  std::uint8_t policy[32]{1U}, owner[16]{0x55U};
  permit.set_compiled_policy_sha256(policy); permit.set_client_instance_id(owner);
  const auto id=client.submit_permit(permit,binding,2U);
  return id!=0U && transport.requests.back().request_seq()==1U;
}
/**
 * @brief 负例核response身份及Hold mask。
 * @param scenario device/profile分别改相应字段；历史mask模式实际改client，hold_mask模式核非完整Hold不可用。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool wire_boundary_case(const std::string &scenario) {
  Transport transport; WireManagementClient client(transport); ManageRequest request;
  request.set_expected_profile_id(4); request.set_configured_mask(3);
  std::uint8_t device[16]{1}; request.set_device_id(device);
  const auto id=client.submit(request,1);
  wire::DummyManageV2Response response; response.set_schema_major(2); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(id);
  response.set_profile_id(4); response.set_device_id(device);
  response.set_receipt(1); response.set_operation_id(1); response.set_transaction_phase(1);
  if (scenario=="device") { device[0]=2; response.set_device_id(device); }
  else if (scenario=="profile") response.set_profile_id(5);
  else if (scenario=="mask") { std::uint8_t client[16]{1}; response.set_client_instance_id(client); }
  else if (scenario!="hold_mask") return false;
  transport.respond(response);
  try {
    const auto reply=client.poll(id);
    return scenario=="hold_mask" && reply && !reply->hold_available;
  } catch (const std::runtime_error &) { return scenario!="hold_mask"; }
}
/**
 * @brief Pending23即使phase4也不当终态拒绝；客户端应Query原请求。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool pending_response_is_not_rejection() {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kSelectMode));
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(id); response.set_result_code(23U);
  response.set_transaction_phase(4U);
  transport.respond(response);
  const auto reply=client.poll(id);
  return reply && !reply->result_valid && !reply->rejected && !reply->succeeded &&
      transport.requests.size()==2U &&
      transport.requests.back().operation()==wire_value(ManageOperation::kQuery) &&
      transport.requests.back().queried_operation_id()==id;
}
/**
 * @brief Grant原session42可暂收session0 Prepared，但Committed仍session0必须拒绝。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool grant_session_pending_query_identity() {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kGrantSession));
  request.set_expected_boot_epoch(7U);
  request.set_expected_session_id(42U);
  request.set_expected_profile_id(3U);
  std::uint8_t device[16]{1U}, owner[16]{2U};
  request.set_device_id(device); request.set_client_instance_id(owner);
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(id); response.set_boot_epoch(7U);
  response.set_profile_id(3U); response.set_device_id(device);
  response.set_client_instance_id(owner);
  response.set_session_id(0U); response.set_operation_id(71U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  response.set_transaction_phase(wire_value(WireTransactionPhase::kReceived));
  response.set_result_code(wire_value(WireResult::kPending));
  transport.respond(response);
  const auto received=client.poll(id);
  if (!received || !received->queued || transport.requests.size()!=2U ||
      transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_transaction_phase(wire_value(WireTransactionPhase::kPrepared));
  transport.respond(response);
  const auto pending=client.poll(id);
  if (!pending || pending->result_valid || pending->rejected || !pending->accepted) return false;
  for (unsigned wait=0; wait<10; ++wait) if (client.poll(id)) return false;
  if (transport.requests.size()!=3U || transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  response.set_result_code(wire_value(WireResult::kOk));
  transport.respond(response);
  try { client.poll(id); } catch (const std::runtime_error &) { return true; }
  return false;
}
/**
 * @brief Prepared阶段也拒绝六种关联身份漂移。
 * @param mismatch 0..5分别破坏boot/device/client/profile/request_id/operation_id。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool grant_session_pending_query_rejects_mismatch(unsigned mismatch) {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kGrantSession));
  request.set_expected_boot_epoch(7U); request.set_expected_session_id(42U);
  request.set_expected_profile_id(3U);
  std::uint8_t device[16]{1U}, owner[16]{2U};
  request.set_device_id(device); request.set_client_instance_id(owner);
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(id); response.set_boot_epoch(7U);
  response.set_profile_id(3U); response.set_device_id(device);
  response.set_client_instance_id(owner); response.set_session_id(0U);
  response.set_operation_id(71U); response.set_transaction_phase(wire_value(WireTransactionPhase::kReceived));
  response.set_result_code(wire_value(WireResult::kPending));
  transport.respond(response);
  if (!client.poll(id)) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_transaction_phase(wire_value(WireTransactionPhase::kPrepared));
  switch (mismatch) {
    case 0U: response.set_boot_epoch(8U); break;
    case 1U: device[0]=2U; response.set_device_id(device); break;
    case 2U: owner[0]=3U; response.set_client_instance_id(owner); break;
    case 3U: response.set_profile_id(4U); break;
    case 4U: response.set_request_id(999U); break;
    case 5U: response.set_operation_id(72U); break;
    default: return false;
  }
  transport.respond(response);
  try { client.poll(id); } catch (const std::runtime_error &) { return true; }
  return false;
}
/**
 * @brief 初回执op0，再Prepared，最后Query Committed session42/op71才完成。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool grant_session_result_after_pending_lookup() {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kGrantSession));
  request.set_expected_session_id(42U);
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(id);
  response.set_session_id(0U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  response.set_transaction_phase(wire_value(WireTransactionPhase::kReceived));
  response.set_result_code(wire_value(WireResult::kPending));
  transport.respond(response);
  if (!client.poll(id) || transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_transaction_phase(wire_value(WireTransactionPhase::kPrepared));
  transport.respond(response);
  const auto pending=client.poll(id);
  if (!pending || pending->result_valid || pending->accepted) return false;
  for (unsigned wait=0; wait<10; ++wait) if (client.poll(id)) return false;
  if (transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_session_id(42U); response.set_operation_id(71U);
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  response.set_result_code(wire_value(WireResult::kOk));
  transport.respond(response);
  const auto committed=client.poll(id);
  return committed && committed->result_valid && committed->succeeded &&
      committed->operation_id==71U && committed->session_id==42U;
}
/**
 * @brief 释放强结果仍要求可接受receipt，不能只看Committed标签。
 * @param via_query 是否先经过Pending/Query。
 * @param terminal_receipt Queued1/Replayed2有效，0/Busy4无效。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool release_terminal_receipt(bool via_query, std::uint8_t terminal_receipt) {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kDisableAndRelease));
  request.set_expected_boot_epoch(7U);
  request.set_expected_session_id(42U);
  request.set_expected_profile_id(3U);
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(id); response.set_boot_epoch(7U);
  response.set_profile_id(3U); response.set_session_id(42U);
  response.set_operation_id(71U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  if(via_query) {
  response.set_transaction_phase(wire_value(WireTransactionPhase::kReceived));
  response.set_result_code(wire_value(WireResult::kPending));
  transport.respond(response);
  if(!client.poll(id) || transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_session_id(0U);
  response.set_transaction_phase(wire_value(WireTransactionPhase::kPrepared));
  transport.respond(response);
  const auto pending=client.poll(id);
  if(!pending || pending->result_valid || pending->release_committed) return false;
  for(unsigned wait=0;wait<10;++wait) if(client.poll(id)) return false;
  response.set_request_id(transport.requests.back().request_id());
  }
  response.set_receipt(terminal_receipt);
  response.set_session_id(0U);
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  response.set_result_code(wire_value(WireResult::kOk));
  response.set_release_committed(true);
  transport.respond(response);
  const bool valid_receipt=terminal_receipt==wire_value(WireReceipt::kQueued) ||
      terminal_receipt==wire_value(WireReceipt::kReplayed);
  try {
    const auto final=client.poll(id);
    return valid_receipt && final && final->request_id==id && final->operation_id==71U &&
      final->result_valid && final->succeeded && final->release_committed;
  } catch(const std::runtime_error &) { return !valid_receipt; }
}

/**
 * @brief RUN续租与终点Complete/Cancel共用客户端时守原身份和期限。
 * @param mode 0正常；1/2错误续租绑定或别的操作；3失败；4原deadline；5Cancel优先；6显式abort；9..14各种续租/观测身份边界。
 * 正例只核Complete准确发一次及冻结seq17/0.2rad，并非最终Succeeded或物理完成。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool running_renewal_does_not_drop_complete(unsigned mode=0U) {
  Transport transport; WireManagementClient client(transport); Port port;
  ActionGate gate(client,port,{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}});
  Authorization auth; auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_boot_epoch(7); auth.identity.set_expected_session_id(9);
  auth.identity.set_expected_reference_epoch(1); auth.identity.set_expected_profile_id(1);
  auth.identity.set_configured_mask(1); auth.identity.set_expected_management_epoch(1);
  auto commit=[&](std::uint32_t epoch) {
    const auto request=transport.requests.back();
    wire::DummyManageV2Response response; response.set_schema_major(2); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
    response.set_request_id(request.request_id()); response.set_boot_epoch(7); response.set_session_id(9);
    response.set_reference_epoch(1); response.set_profile_id(1); response.set_receipt(1);
    response.set_operation_id(request.request_id()+100); response.set_transaction_phase(3);
    response.set_motion_epoch(epoch); response.set_management_epoch(1);
    transport.respond(response); return client.poll(request.request_id());
  };
  if(!gate.enable(auth,1)) return false;
  auto reply=commit(1); if(!reply) return false;
  reply->snapshot_valid=true; reply->hold_available=true; reply->hold_ready=false;
  gate.observe(*reply,2);
  if(!gate.record_update(gate.generation(),1,1,Position{})) return false;
  reply->hold_ready=true; reply->accepted_seq=1; gate.observe(*reply,3);
  const Trajectory motion{{"Joint1"},{{{0.2},{0},{},1}},{},{},0};
  if(!gate.submit(motion,10)) return false;
  reply=commit(2); if(!reply) return false;
  reply->snapshot_valid=true; reply->hold_available=true; reply->hold_ready=false;
  gate.observe(*reply,11);
  if(!gate.record_update(gate.generation(),2,1,Position{})) return false;
  reply->hold_ready=true; reply->accepted_seq=1; gate.observe(*reply,12);
  if(gate.phase()!=Phase::kRunning) return false;
  BootstrapCoordinator bootstrap(client,auth.identity,{},0);
  BootstrapSnapshot snapshot{7,9,1,2,1,55,1,1,true,true};
  if(mode==1U) snapshot.reference_epoch=2;
  if(mode==9U) snapshot.management_epoch=2;
  snapshot.core_state=snapshot.state_core_state=wire_value(WireCoreState::kRunning); snapshot.run_id=5;
  if(!bootstrap.start_operation(snapshot,ManageOperation::kCaptureReference,20)) return false;
  // 独立管理协调器就绪后，真实共享WireClient发送RUN续租。
  const auto setup=transport.requests.back();
  // Bootstrap自行消费本次setup终态。
  wire::DummyManageV2Response response; response.set_schema_major(2); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(setup.request_id()); response.set_boot_epoch(7); response.set_session_id(9);
  response.set_reference_epoch(1); response.set_profile_id(1); response.set_receipt(1);
  response.set_operation_id(setup.request_id()+100); response.set_transaction_phase(3);
  response.set_reference_epoch(snapshot.reference_epoch);
  response.set_motion_epoch(2); response.set_management_epoch(snapshot.management_epoch);
  transport.respond(response); bootstrap.tick(21);
  if(!bootstrap.operation_complete() || !bootstrap.start_renew_lease(snapshot,30000000,22)) return false;
  if(mode==2U) {
    // 同共享client忙于其它operation，Complete不得把它冒充续租。
    auto setup_request=auth.identity; setup_request.set_operation(wire_value(ManageOperation::kCaptureReference));
    auto finished=transport.requests.back();
    response.set_request_id(finished.request_id()); response.set_operation_id(104);
    response.set_transaction_phase(3); transport.respond(response); bootstrap.tick(22);
    if(!client.submit(setup_request,22)) return false;
  }
  /**
   * @par 续租/Complete仲裁
   * 保留尚未被bootstrap消费的Renew回执；Gate不能绕开它发Complete，也不能因此重置目标终态deadline。
   */
  const auto renewal=transport.requests.back();
  response.set_request_id(renewal.request_id()); response.set_operation_id(0);
  response.set_transaction_phase(1); response.set_result_code(23);
  transport.respond(response); // 已到达Received Pending，Bootstrap尚未poll。
  Position end{}; end[0]=0.2;
  if(!gate.record_update(gate.generation(),2,17,end)) return false;
  gate.inner_result(gate.generation(),Terminal::kSucceeded,"",23);
  reply->accepted_seq=17; gate.observe(*reply,24);
  std::cout << "RENEW_COMPLETE_BOUNDARY renewal=" << renewal.request_id()
            << " phase=" << static_cast<unsigned>(gate.phase()) << " error=" << gate.error() << '\n';
  if(mode==1U || mode==2U || mode==9U)
    return gate.terminal()==Terminal::kAborted && gate.error()=="TRANSPORT_UNAVAILABLE";
  if(gate.terminal()!=Terminal::kNone || !gate.error().empty() ||
     transport.requests.back().operation()!=wire_value(ManageOperation::kRenewLease)) return false;
  if(!gate.completion_waiting_for_renewal(25) || gate.completion_waiting_for_renewal(2000024)) return false;
  if(mode==4U) { gate.tick(2000024); return gate.terminal()==Terminal::kAborted &&
      transport.requests.back().operation()!=wire_value(ManageOperation::kComplete); }
  if(mode==5U) {
    const auto before_cancel_count=transport.requests.size();
    gate.cancel(25); gate.tick(26);
    auto cancel_live=auth.identity;cancel_live.set_operation(wire_value(ManageOperation::kCancel));
    cancel_live.set_expected_motion_epoch(2);
    if(gate.phase()!=Phase::kStopping || gate.terminal()!=Terminal::kNone ||
       !gate.error().empty() || !gate.cancel_waiting_for_renewal(cancel_live,26) ||
       gate.cancel_waiting_for_renewal(cancel_live,2000025) ||
       transport.requests.size()!=before_cancel_count ||
       transport.requests.back().operation()!=wire_value(ManageOperation::kRenewLease)) return false;
    auto wrong=cancel_live;wrong.set_expected_session_id(10);
    if(gate.cancel_waiting_for_renewal(wrong,26)) return false;
    bootstrap.tick(27); // 原Received/Pending真实消费，生成匹配本Renew的Query。
    const auto query=transport.requests.back();
    if(query.operation()!=wire_value(ManageOperation::kQuery)) return false;
    response.set_request_id(query.request_id());response.set_operation_id(104);
    response.set_transaction_phase(3);response.set_result_code(0);
    transport.respond(response);bootstrap.tick(28); // 完成本次真实RUN续租，不重新续租。
    if(!bootstrap.operation_complete()) return false;
    gate.tick(29);
    const auto cancel_request=transport.requests.back();
    if(cancel_request.operation()!=wire_value(ManageOperation::kCancel) ||
       cancel_request.request_id()==0U || cancel_request.request_id()==renewal.request_id() ||
       cancel_request.request_id()==query.request_id() || transport.requests.size()!=before_cancel_count+2U ||
       gate.phase()!=Phase::kStopping || gate.terminal()!=Terminal::kNone || !gate.error().empty() ||
       gate.cancel_waiting_for_renewal(cancel_live,30)) return false;
    if(cancel_request.expected_boot_epoch()!=cancel_live.expected_boot_epoch() ||
       cancel_request.expected_session_id()!=cancel_live.expected_session_id() ||
       cancel_request.expected_reference_epoch()!=cancel_live.expected_reference_epoch() ||
       cancel_request.expected_motion_epoch()!=cancel_live.expected_motion_epoch() ||
       cancel_request.expected_management_epoch()!=cancel_live.expected_management_epoch() ||
       cancel_request.expected_profile_id()!=cancel_live.expected_profile_id() ||
       cancel_request.configured_mask()!=cancel_live.configured_mask() ||
       !std::equal(std::begin(cancel_request.device_id()),std::end(cancel_request.device_id()),std::begin(cancel_live.device_id())) ||
       !std::equal(std::begin(cancel_request.client_instance_id()),std::end(cancel_request.client_instance_id()),std::begin(cancel_live.client_instance_id()))) return false;
    gate.tick(31);
    return transport.requests.size()==before_cancel_count+2U &&
      std::count_if(transport.requests.begin()+static_cast<std::ptrdiff_t>(before_cancel_count),transport.requests.end(),
        [](const auto &q){return q.operation()==wire_value(ManageOperation::kCancel);})==1 &&
      std::none_of(transport.requests.begin()+static_cast<std::ptrdiff_t>(before_cancel_count),transport.requests.end(),
        [](const auto &q){return q.operation()==wire_value(ManageOperation::kComplete) || q.operation()==wire_value(ManageOperation::kRenewLease);});
  }
  if(mode==6U) { gate.abort("TEST_STOP",25); gate.tick(26); return gate.terminal()==Terminal::kAborted &&
      transport.requests.back().operation()!=wire_value(ManageOperation::kComplete); }
  bootstrap.tick(25); // 消费Received Pending并发送本续租Query
  response.set_request_id(transport.requests.back().request_id()); response.set_operation_id(104);
  response.set_transaction_phase(mode==3U ? 4 : 3); response.set_result_code(mode==3U ? 6 : 0);
  response.set_management_epoch(mode==12U ? 2 : 1);
  if(mode==10U) response.set_reference_epoch(2);
  if(mode==11U) response.set_motion_epoch(3);
  transport.respond(response); bootstrap.tick(26);
  if(mode==13U) {
    auto intervening=auth.identity; intervening.set_operation(wire_value(ManageOperation::kCaptureReference));
    if(!client.submit(intervening,26)) return false;
    if(!commit(2)) return false;
  }
  if(mode==14U) {
    wire::DummyJointStateV2 state; state.set_schema_major(2); state.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
    state.set_boot_epoch(7); state.set_session_id(9); state.set_reference_epoch(1);
    state.set_motion_epoch(2); state.set_management_epoch(2); state.set_profile_id(1);
    state.set_state_seq(1); state.set_configured_mask(1); state.set_position_valid_mask(1);
    state.set_core_state(wire_value(WireCoreState::kRunning)); state.set_freshness_ticket(55);
    std::array<std::uint8_t,wire::DummyJointStateV2Codec::kEncodedSize> encoded{};
    if(wire::DummyJointStateV2Codec::encode(state,encoded.data(),encoded.size())!=wire::CodecResult::kOk ||
       !gate.observe_state(encoded.data(),encoded.size(),26)) return false;
  }
  gate.tick(27);
  if(mode>=10U && mode<=14U) return gate.terminal()!=Terminal::kSucceeded &&
      transport.requests.back().operation()!=wire_value(ManageOperation::kComplete);
  if(mode==3U) return gate.phase()==Phase::kStopping &&
      transport.requests.back().operation()!=wire_value(ManageOperation::kComplete);
  const auto count=transport.requests.size();
  gate.tick(28);
  return transport.requests.size()==count && transport.requests.back().operation()==8 &&
      transport.requests.back().terminal_producer_seq()==17 &&
      transport.requests.back().expected_management_epoch()==1 &&
      transport.requests.back().terminal_position_rad()[0]==0.2 && gate.terminal()==Terminal::kNone;
}

/**
 * @brief 短暂Status/State不相干时ReleasePending仍可Query，避免因等Idle而阻塞推进。
 * @param fault 0正常；1错stage、2错phase、3错op、4超时、5错boot、6无inflight、7发送失败、8新Idle State配旧Stopping Status。
 * 强回执只到ReleaseAwaitIdle；仍缺完整双流Idle时不能released。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool release_pending_polls_during_observation_grace(unsigned fault=0U) {
  Transport transport; WireManagementClient client(transport);
  ManageRequest identity; identity.set_expected_session_id(9);
  BootstrapCoordinator bootstrap(client,identity,{},0);
  BootstrapSnapshot hold{7,9,1,10,1,55,1,1,true,true};
  hold.core_state=hold.state_core_state=wire_value(WireCoreState::kHold);
  if(fault==7U) {
    transport.reject_send=true;
    return !bootstrap.start_release(hold,100) && bootstrap.phase()==BootstrapPhase::kFault &&
        !bootstrap.release_request_inflight();
  }
  if(!bootstrap.start_release(hold,100) || !bootstrap.release_request_inflight()) return false;
  const auto original=transport.requests.back();
  wire::DummyManageV2Response pending; pending.set_schema_major(2); pending.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  pending.set_request_id(original.request_id()); pending.set_boot_epoch(7); pending.set_session_id(9);
  pending.set_reference_epoch(1); pending.set_motion_epoch(10); pending.set_management_epoch(1);
  pending.set_profile_id(1); pending.set_receipt(1); pending.set_transaction_phase(1); pending.set_result_code(23);
  transport.respond(pending);
  // 使用真实wire对象与Node同款continuity：新Status11/Stopping7，accepted State仍旧10/Hold4。
  wire::DummyDeviceStatusV2 status; status.set_boot_epoch(7); status.set_session_id(9);
  status.set_reference_epoch(1); status.set_motion_epoch(11); status.set_management_epoch(2); status.set_core_state(7);
  wire::DummyJointStateV2 accepted_state; accepted_state.set_boot_epoch(7); accepted_state.set_session_id(9);
  accepted_state.set_reference_epoch(1); accepted_state.set_motion_epoch(10); accepted_state.set_management_epoch(1);
  accepted_state.set_core_state(4);
  const bool coherent=status.motion_epoch()==accepted_state.motion_epoch() &&
      status.management_epoch()==accepted_state.management_epoch();
  DeviceObservation observation; observation.state_received_us=observation.status_received_us=100;
  ObservationContinuity continuity(250000,1000000,250000);
  const auto decision=continuity.evaluate(observation,101,true,coherent);
  const bool observation_ready=decision==ObservationContinuity::Result::kReady;
  const bool observation_lost=decision==ObservationContinuity::Result::kLost;
  const bool release_transition_pending=false; // accepted State尚非Idle，真实Idle仍在transport缓存。
  const bool old_caller_tick=observation_ready || observation_lost || release_transition_pending;
  if(old_caller_tick || coherent) return false;
  const bool tick=bootstrap_tick_required(observation_ready,observation_lost,release_transition_pending, fault==1U ? 1 : 2,
      fault==2U ? BootstrapPhase::kRenewLease : bootstrap.phase(),
      fault==3U ? ManageOperation::kRenewLease : bootstrap.last_operation(),
      fault==6U ? false : bootstrap.release_request_inflight());
  if((fault>=1U && fault<=3U) || fault==6U) return !tick && transport.requests.size()==1U;
  if(fault==4U) {
    bootstrap.tick(5000101U);
    return bootstrap.phase()==BootstrapPhase::kFault && transport.requests.size()==1U;
  }
  if(tick) bootstrap.tick(101);
  if(transport.requests.size()!=2U || transport.requests.back().operation()!=12U ||
     transport.requests.back().queried_operation_id()!=original.request_id()) return false;
  auto terminal=pending; terminal.set_request_id(transport.requests.back().request_id());
  terminal.set_boot_epoch(fault==5U ? 8 : 7); terminal.set_session_id(0); terminal.set_reference_epoch(0);
  terminal.set_motion_epoch(11); terminal.set_management_epoch(2); terminal.set_operation_id(42);
  terminal.set_run_id(2); terminal.set_transaction_phase(3); terminal.set_result_code(0); terminal.set_release_committed(true);
  transport.respond(terminal); bootstrap.tick(102);
  if(fault==5U) return bootstrap.phase()==BootstrapPhase::kFault && !bootstrap.confirmed_release();
  if(fault==8U) {
    if(bootstrap.phase()!=BootstrapPhase::kReleaseAwaitIdle || !bootstrap.confirmed_release()) return false;
    // 对齐v4 raw：已验证COMMITTED，State持续20ms；旧Stopping Status约500ms才更新。
    ReleaseTransitionOrigin origin; origin.capture({7,9,1,10},1);
    ObservationContinuity release_continuity(250000,1000000,250000);
    DeviceObservation observed; observed.status_received_us=113; observed.state_received_us=114;
    if(release_continuity.evaluate(observed,114,true,false)!=ObservationContinuity::Result::kWait) return false;
    const auto transition=origin.observation({7,9,1,11},{7,0,0,11},2,2,7,1,2);
    for(std::uint64_t now=20114;now<=480114;now+=20000) {
      observed.state_received_us=now;
      const bool waiting=release_idle_transition_pending(transition,observed,now,250000);
      if(!waiting && release_continuity.evaluate(observed,now,true,false)==ObservationContinuity::Result::kLost) {
        std::cout << "RELEASE_STATUS_GAP_LOST now=" << now << " status_age=" << now-113
                  << " state_age=0 committed_request=" << bootstrap.confirmed_release()->request_id << '\n';
        return false;
      }
      if(bootstrap.released()) return false;
    }
    return true;
  }
  return bootstrap.phase()==BootstrapPhase::kReleaseAwaitIdle && bootstrap.confirmed_release().has_value() &&
      transport.requests.size()==2U;
}

/**
 * @brief 20ms人工tick让Release Query在250ms State监督预算内拿到强回执。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool release_query_before_state_supervision_deadline() {
  Transport transport; WireManagementClient client(transport);
  ManageRequest identity; identity.set_expected_session_id(9);
  BootstrapCoordinator bootstrap(client,identity,{},0);
  BootstrapSnapshot hold{7,9,1,10,1,55,1,1,true,true};
  hold.core_state=hold.state_core_state=wire_value(WireCoreState::kHold);
  constexpr std::uint64_t request_time=1000000U, last_hold=request_time-32000U;
  if(!bootstrap.start_release(hold,request_time)) return false;
  const auto original=transport.requests.back();
  wire::DummyManageV2Response reply; reply.set_schema_major(2); reply.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  reply.set_boot_epoch(7); reply.set_session_id(9); reply.set_reference_epoch(1);
  reply.set_motion_epoch(10); reply.set_management_epoch(1); reply.set_profile_id(1);
  reply.set_request_id(original.request_id()); reply.set_receipt(1);
  reply.set_transaction_phase(1); reply.set_result_code(23);
  transport.respond(reply); bootstrap.tick(request_time+20000U);
  if(transport.requests.size()!=2U || transport.requests.back().queried_operation_id()!=original.request_id()) return false;
  reply.set_request_id(transport.requests.back().request_id()); reply.set_session_id(0);
  reply.set_reference_epoch(0); reply.set_motion_epoch(11); reply.set_management_epoch(2);
  reply.set_transaction_phase(2); // 真实第一Query Prepared23/op0
  transport.respond(reply); bootstrap.tick(request_time+40000U);
  std::uint64_t committed_at=0U;
  for(std::uint64_t now=request_time+60000U;now<request_time+300000U;now+=20000U) {
    const auto count=transport.requests.size(); bootstrap.tick(now);
    if(transport.requests.size()>count) {
      const auto query=transport.requests.back();
      if(query.operation()!=12U || query.queried_operation_id()!=original.request_id()) return false;
      reply.set_request_id(query.request_id()); reply.set_operation_id(42);
      reply.set_transaction_phase(3); reply.set_result_code(0); reply.set_run_id(2);
      reply.set_release_committed(true); transport.respond(reply);
      bootstrap.tick(now+4000U); committed_at=now+4000U; break;
    }
  }
  std::cout << "RELEASE_QUERY_SUPERVISION request=" << original.request_id() << " last_hold=" << last_hold
            << " committed_at=" << committed_at << " gap=" << committed_at-last_hold << '\n';
  return committed_at>0U && committed_at-last_hold<=250000U && bootstrap.confirmed_release().has_value();
}

/**
 * @brief Renew较慢Query节奏与Release较快节奏分开；Query两次重试沿用同RID/seq及queried原ID，第三次失败。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool query_backoff_and_retry_identity_remain_bounded() {
  for(const auto operation:{ManageOperation::kRenewLease,ManageOperation::kDisableAndRelease}) {
    Transport transport; WireManagementClient client(transport);
    ManageRequest mutation; mutation.set_operation(wire_value(operation));
    mutation.set_expected_boot_epoch(7); mutation.set_expected_session_id(9);
    mutation.set_expected_reference_epoch(1); mutation.set_expected_motion_epoch(10); mutation.set_expected_profile_id(1);
    const auto id=client.submit(mutation,100);
    wire::DummyManageV2Response reply; reply.set_schema_major(2); reply.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
    reply.set_request_id(id); reply.set_boot_epoch(7); reply.set_session_id(9);
    reply.set_reference_epoch(1); reply.set_motion_epoch(10); reply.set_profile_id(1);
    reply.set_receipt(1); reply.set_transaction_phase(1); reply.set_result_code(23);
    transport.respond(reply); (void)client.poll(id);
    reply.set_request_id(transport.requests.back().request_id()); reply.set_transaction_phase(2);
    transport.respond(reply); (void)client.poll(id);
    const auto count=transport.requests.size();
    if(operation==ManageOperation::kRenewLease) {
      for(unsigned poll=0;poll<9;++poll) { (void)client.poll(id); if(transport.requests.size()!=count) return false; }
    }
    (void)client.poll(id);
    if(transport.requests.size()!=count+1U) return false;
    const auto retry=transport.requests.back();
    for(unsigned attempt=0;attempt<2;++attempt) {
      transport.timeout_next=true; (void)client.poll(id);
      const auto resent=transport.requests.back();
      if(resent.request_id()!=retry.request_id() || resent.request_seq()!=retry.request_seq() ||
         resent.queried_operation_id()!=id || resent.operation()!=12U) return false;
    }
    transport.timeout_next=true;
    try { (void)client.poll(id); return false; } catch(const std::runtime_error &) {}
    for(const auto &request:transport.requests)
      if(request.operation()!=12U && request.request_id()!=id) return false;
  }
  return true;
}

/**
 * @brief 选择单个协议边界或运行完整内存管理握手序列。
 * @param argc 进程参数数量。
 * @param argv 模式/输入路径见下述说明。
 * @return 0表示所选断言通过；非零、assert或检查异常表示失败。
 * argv[1]选择模式；running_renewal_strict_identity可用argv[2]指定mode，否则循环9..14；complete_boundaries循环1..6。
 * 无参数覆盖身份/序号/permit/释放后，逐步核Queued、Prepared、Query失败和迟到无关RID，最后双流Enable。只打印组件层证据。
 */
int main(int argc,char **argv) {
  if(argc==2 && std::string(argv[1])=="release_query_backoff_guards")
    return query_backoff_and_retry_identity_remain_bounded() ? 0 : 60;
  if(argc==2 && std::string(argv[1])=="release_query_supervision_budget")
    return release_query_before_state_supervision_deadline() ? 0 : 59;
  if(argc==2 && std::string(argv[1])=="release_committed_status_gap")
    return release_pending_polls_during_observation_grace(8U) ? 0 : 58;
  if(argc==2 && std::string(argv[1])=="release_pending_observation_grace")
    return release_pending_polls_during_observation_grace() ? 0 : 50;
  if(argc==2 && std::string(argv[1])=="release_pending_observation_grace_guards") {
    for(unsigned fault=1;fault<=7;++fault) if(!release_pending_polls_during_observation_grace(fault)) return 50+fault;
    return 0;
  }
  if(argc==3 && std::string(argv[1])=="running_renewal_strict_identity")
    return running_renewal_does_not_drop_complete(static_cast<unsigned>(std::stoul(argv[2]))) ? 0 : 31;
  if(argc==2 && std::string(argv[1])=="running_renewal_complete")
    return running_renewal_does_not_drop_complete() ? 0 : 31;
  if(argc==2 && std::string(argv[1])=="running_renewal_strict_identity") {
    for(unsigned mode=9;mode<=14;++mode) if(!running_renewal_does_not_drop_complete(mode)) return 31+mode;
    return 0;
  }
  if(argc==2 && std::string(argv[1])=="running_renewal_complete_boundaries") {
    for(unsigned mode=1;mode<=6;++mode) if(!running_renewal_does_not_drop_complete(mode)) return 31+mode;
    return 0;
  }
  if (argc==2) return wire_boundary_case(argv[1])?0:9;
  if (!sequence_wrap_skips_zero()) return 22;
  if (!permit_wrap_uses_sent_sequence()) return 23;
  if (!process_sessions_do_not_reuse_management_identity()) return 21;
  if (!pending_response_is_not_rejection()) return 16;
  if (!grant_session_pending_query_identity()) return 17;
  for (unsigned mismatch=0U; mismatch<6U; ++mismatch)
    if (!grant_session_pending_query_rejects_mismatch(mismatch)) return 18;
  if (!grant_session_result_after_pending_lookup()) return 19;
  for(const bool via_query:{false,true})
    for(const std::uint8_t receipt:{std::uint8_t{0U},wire_value(WireReceipt::kQueued),
                                    wire_value(WireReceipt::kReplayed),wire_value(WireReceipt::kBusy)})
      if(!release_terminal_receipt(via_query,receipt)) return 20;
  Transport transport; WireManagementClient client(transport); ManageRequest request;
  request.set_expected_session_id(9007199254740993ULL);
  request.set_operation(wire_value(ManageOperation::kGrantSession));
  const auto id=client.submit(request,123); if (!id || client.poll(id)) return 1;
  ManageRequest competing=request;
  competing.set_operation(wire_value(ManageOperation::kRenewLease));
  competing.set_requested_session_lease_us(500000U);
  if(client.submit(competing,124)!=0U || transport.requests.size()!=1U ||
     client.original_sent_at_us()!=123U) return 15;
  if (transport.requests.size()!=1 || transport.requests[0].expected_session_id()!=9007199254740993ULL) return 2;
  wire::DummyManageV2Response response; response.set_schema_major(2); response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  response.set_request_id(id);
  response.set_session_id(9007199254740993ULL);
  response.set_receipt(1); response.set_operation_id(9007199254740995ULL); response.set_transaction_phase(1);
  transport.respond(response); const auto queued=client.poll(id);
  if (!queued || !queued->queued || queued->accepted || queued->result_valid) return 3;
  const auto query=transport.requests.back();
  if (query.operation()!=12 || query.queried_operation_id()!=id || query.request_id()==id) return 4;
  transport.timeout_next=true;
  if(client.poll(id) || transport.requests.back().request_id()!=query.request_id() ||
     transport.requests.back().queried_operation_id()!=id || client.original_sent_at_us()!=123) return 11;
  response.set_request_id(query.request_id());
  response.set_operation_id(9007199254740995ULL);
  response.set_transaction_phase(2);
  const auto accepted_query_count=transport.requests.size();
  transport.respond(response); const auto accepted=client.poll(id);
  if (!accepted || !accepted->accepted || accepted->queued || accepted->result_valid) return 5;
  if(transport.requests.size()!=accepted_query_count) return 12;
  for(unsigned wait=0;wait<10;++wait) if(client.poll(id)) return 13;
  if(transport.requests.size()!=accepted_query_count+1) return 14;
  const auto terminal_query_count=transport.requests.size();
  response.set_request_id(transport.requests.back().request_id());
  response.set_result_code(6); response.set_first_failure_reason(7);
  response.set_transaction_phase(4);
  transport.respond(response); const auto failed=client.poll(id);
  if (!failed || !failed->result_valid || failed->succeeded || client.original_sent_at_us()!=123 ||
      transport.requests.size()!=terminal_query_count || failed->result_code!=6 ||
      failed->first_failure_reason!=7 || failed->transaction_phase!=4) return 6;
  response.set_request_id(999); transport.respond(response);
  if (client.poll(id)) return 7;
  const bool handshake=two_stream_handshake(); if (!handshake) return 8;
  const bool round_trip=transport.requests[0].expected_session_id()==request.expected_session_id();
  const bool request_preserved=query.queried_operation_id()==id;
  if (!round_trip || !request_preserved) return 10;
  std::cout << std::boolalpha << "{\"two_stream_handshake\":" << handshake
    << ",\"codec_round_trip\":" << round_trip << ",\"queued_is_accepted\":" << queued->accepted
    << ",\"original_request_preserved\":" << request_preserved
    << ",\"causal_t0\":" << client.original_sent_at_us() << ",\"evidence_scope\":\"component\"}\n";
}
