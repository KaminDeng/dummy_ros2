#include "typed_management_fixture.hpp"
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

/**
 * @brief 编解码内存链路，保存请求并以FIFO提供合成响应。
 * timeout_next只使下一poll抛异常，reject_send使send返回空句柄。
 */
using typed_management_fixture::Transport;
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
  Authorization auth;auth.identity=owned_fixture::request(); auth.session=auth.reference=auth.permit=true;
  auth.identity.expected_session_id()=9; auth.identity.expected_boot_epoch()=7;
  auth.identity.expected_profile_id()=1; auth.identity.configured_mask()=1;
  if (!gate.enable(auth,100)) return false;
  ManagementResponse receipt{owned_fixture::response(),typed_management_fixture::source()};
  receipt.first.request_id()=*transport.requests.front().request_id();
  receipt.second.boot_id=7; receipt.first.session_id()=9;
  receipt.first.profile_id()=1; receipt.first.receipt()=1; receipt.first.operation_id()=101;
  receipt.first.transaction_phase()=1;
  transport.respond(receipt); gate.tick(101); if (port.primes) return false;
  auto state=owned_fixture::state();auto state_source=typed_management_fixture::source();
  state_source.boot_id=7;
  state.session_id()=9; state.profile_id()=1; state.state_seq()=1;
  state.motion_epoch()=1; typed_management_fixture::configure(state,1); typed_management_fixture::position_quality(state,1);
  state.hold_ready()=true; state.core_state()=3;
  auto observe=[&] {
    return gate.observe_state(state,state_source,102);
  };
  if (!observe() || port.primes!=1 || gate.ready()) return false;
  if (!gate.record_update(gate.generation(),1,1,Position{})) return false;
  receipt.first.request_id()=*transport.requests.back().request_id();
  receipt.first.result_code()=0; receipt.first.transaction_phase()=3;
  // Management completion and periodic HOLD evidence arrive independently.
  transport.respond(receipt); gate.tick(103); if (gate.ready()) return false;
  state.state_seq()=2; state.committed_producer_seq()=1;
  state.core_state()=4; state.hold_ready()=true;
  state.can_submitted_mask()=1; state.can_sent_mask()=1;
  state.freshness_ticket()=55;
  if(!observe() || !gate.ready() || observe()) return false;
  const Trajectory motion{{"Joint1"},{{{0.0},{},{},1.0}},{},{},0};
  return gate.submit(motion,104) && *transport.requests.back().freshness_ticket()==55;
}
/**
 * @brief 不同process seed或同seed不同invocation构造的客户端ID不交叉；这里只模拟构造值，不fork进程。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool process_sessions_do_not_reuse_management_identity() {
  Transport first_transport, second_transport;
  WireManagementClient first(first_transport, 0x123456789abcdef0ULL);
  WireManagementClient second(second_transport, 0x123456789abcdef1ULL);
  auto select=owned_fixture::request();
  select.operation()=wire_value(ManageOperation::SELECT_CONTROL_MODE);
  select.expected_session_id()=0U;
  std::array<std::uint64_t,8> first_ids{}, second_ids{};
  const auto issue=[&](WireManagementClient &client,Transport &transport,
                       std::array<std::uint64_t,8> &ids) {
    for (std::size_t index=0;index<ids.size();++index) {
      ids[index]=client.submit(select,index+1U);
      if (ids[index]==0U || *transport.requests.back().request_seq()==1U) return false;
      ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};

      response.first.request_id()=ids[index]; response.first.result_code()=0U;
      response.first.receipt()=wire_value(WireReceipt::kQueued);
      response.first.transaction_phase()=wire_value(WireTransactionPhase::COMMITTED);
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
  return *first_transport.requests.front().request_seq()!=
      *second_transport.requests.front().request_seq();
}
/**
 * @brief request_seq从UINT32_MAX回到1，跳过保留值0。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool sequence_wrap_skips_zero() {
  Transport transport;
  WireManagementClient client(transport,0x12345678fffffffeULL);
  auto request=owned_fixture::request();
  request.operation()=wire_value(ManageOperation::SELECT_CONTROL_MODE);
  const auto first=client.submit(request,1U);
  if (first==0U || *transport.requests.back().request_seq()!=UINT32_MAX) return false;
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};

  response.first.request_id()=first; response.first.result_code()=0U;
  response.first.receipt()=wire_value(WireReceipt::kQueued);
  response.first.transaction_phase()=wire_value(WireTransactionPhase::COMMITTED);
  transport.respond(response);
  if (!client.poll(first)) return false;
  const auto second=client.submit(request,2U);
  return second!=0U && *transport.requests.back().request_seq()==1U;
}
/**
 * @brief 回绕permit必须用实际已发送seq1参与摘要，不能用旧MAX或未发送预测号。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool permit_wrap_uses_sent_sequence() {
  Transport transport;
  WireManagementClient client(transport,0x12345678fffffffeULL);
  auto select=owned_fixture::request();
  select.operation()=wire_value(ManageOperation::SELECT_CONTROL_MODE);
  const auto first=client.submit(select,1U);
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};

  response.first.request_id()=first; response.first.result_code()=0U;
  response.first.receipt()=wire_value(WireReceipt::kQueued);
  response.first.transaction_phase()=wire_value(WireTransactionPhase::COMMITTED);
  transport.respond(response);
  if (!client.poll(first)) return false;
  PermitBinding binding{};
  binding.boot_epoch=7U; binding.reference_epoch=3U; binding.profile_id=201U;
  binding.firmware_sha256[0]=1U; binding.contract_sha256[0]=2U;
  binding.model_sha256[0]=3U; binding.profile_sha256[0]=4U;
  auto permit=owned_fixture::request();
  permit.operation()=wire_value(ManageOperation::INSTALL_PERMIT);
  permit.expected_boot_epoch()=binding.boot_epoch;
  permit.expected_session_id()=9U;
  permit.expected_reference_epoch()=binding.reference_epoch;
  permit.expected_profile_id()=binding.profile_id;
  permit.operator_confirmed()=true; permit.permit_remaining_us()=1000000U;
  std::uint8_t policy[32]{1U}, owner[16]{0x55U};
  permit.compiled_policy_sha256().emplace().assign(policy,32U); permit.client_instance_id().emplace().assign(owner,16U);
  const auto id=client.submit_permit(permit,binding,2U);
  return id!=0U && *transport.requests.back().request_seq()==1U;
}
/**
 * @brief 负例核response身份及Hold mask。
 * @param scenario device/profile分别改相应字段；历史mask模式实际改client，hold_mask模式核非完整Hold不可用。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool wire_boundary_case(const std::string &scenario) {
  Transport transport; WireManagementClient client(transport); auto request=owned_fixture::request();
  request.expected_profile_id()=4; request.configured_mask()=3;
  std::uint8_t device[16]{1}; request.domain_device_id().emplace().assign(device,16U);
  const auto id=client.submit(request,1);
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};
  response.first.request_id()=id;
  response.first.profile_id()=4; response.first.domain_device_id().emplace().assign(device,16U);
  response.first.receipt()=1; response.first.operation_id()=1; response.first.transaction_phase()=1;
  if (scenario=="device") { device[0]=2; response.first.domain_device_id().emplace().assign(device,16U); }
  else if (scenario=="profile") response.first.profile_id()=5;
  else if (scenario=="mask") { std::uint8_t client[16]{1}; response.first.client_instance_id().emplace().assign(client,16U); }
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
  auto request=owned_fixture::request();
  request.operation()=wire_value(ManageOperation::SELECT_CONTROL_MODE);
  const auto id=client.submit(request,1U);
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};

  response.first.request_id()=id; response.first.result_code()=23U;
  response.first.transaction_phase()=4U;
  transport.respond(response);
  const auto reply=client.poll(id);
  return reply && !reply->result_valid && !reply->rejected && !reply->succeeded &&
      transport.requests.size()==2U &&
      *transport.requests.back().operation()==wire_value(ManageOperation::GET_OPERATION) &&
      *transport.requests.back().queried_operation_id()==id;
}
/**
 * @brief Grant原session42可暂收session0 Prepared，但Committed仍session0必须拒绝。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool grant_session_pending_query_identity() {
  Transport transport;
  WireManagementClient client(transport);
  auto request=owned_fixture::request();
  request.operation()=wire_value(ManageOperation::GRANT_SESSION);
  request.expected_boot_epoch()=7U;
  request.expected_session_id()=42U;
  request.expected_profile_id()=3U;
  std::uint8_t device[16]{1U}, owner[16]{2U};
  request.domain_device_id().emplace().assign(device,16U); request.client_instance_id().emplace().assign(owner,16U);
  const auto id=client.submit(request,1U);
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};

  response.first.request_id()=id; response.second.boot_id=7U;
  response.first.profile_id()=3U; response.first.domain_device_id().emplace().assign(device,16U);
  response.first.client_instance_id().emplace().assign(owner,16U);
  response.first.session_id()=0U; response.first.operation_id()=71U;
  response.first.receipt()=wire_value(WireReceipt::kQueued);
  response.first.transaction_phase()=wire_value(WireTransactionPhase::RECEIVED);
  response.first.result_code()=wire_value(WireResult::PENDING);
  transport.respond(response);
  const auto received=client.poll(id);
  if (!received || !received->queued || transport.requests.size()!=2U ||
      *transport.requests.back().queried_operation_id()!=id) return false;
  response.first.request_id()=*transport.requests.back().request_id();
  response.first.transaction_phase()=wire_value(WireTransactionPhase::PREPARED);
  transport.respond(response);
  const auto pending=client.poll(id);
  if (!pending || pending->result_valid || pending->rejected || !pending->accepted) return false;
  for (unsigned wait=0; wait<10; ++wait) if (client.poll(id)) return false;
  if (transport.requests.size()!=3U || *transport.requests.back().queried_operation_id()!=id) return false;
  response.first.request_id()=*transport.requests.back().request_id();
  response.first.transaction_phase()=wire_value(WireTransactionPhase::COMMITTED);
  response.first.result_code()=wire_value(WireResult::OK);
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
  auto request=owned_fixture::request();
  request.operation()=wire_value(ManageOperation::GRANT_SESSION);
  request.expected_boot_epoch()=7U; request.expected_session_id()=42U;
  request.expected_profile_id()=3U;
  std::uint8_t device[16]{1U}, owner[16]{2U};
  request.domain_device_id().emplace().assign(device,16U); request.client_instance_id().emplace().assign(owner,16U);
  const auto id=client.submit(request,1U);
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};

  response.first.request_id()=id; response.second.boot_id=7U;
  response.first.profile_id()=3U; response.first.domain_device_id().emplace().assign(device,16U);
  response.first.client_instance_id().emplace().assign(owner,16U); response.first.session_id()=0U;
  response.first.operation_id()=71U; response.first.transaction_phase()=wire_value(WireTransactionPhase::RECEIVED);
  response.first.result_code()=wire_value(WireResult::PENDING);
  transport.respond(response);
  if (!client.poll(id)) return false;
  response.first.request_id()=*transport.requests.back().request_id();
  response.first.transaction_phase()=wire_value(WireTransactionPhase::PREPARED);
  switch (mismatch) {
    case 0U: response.second.boot_id=8U; break;
    case 1U: device[0]=2U; response.first.domain_device_id().emplace().assign(device,16U); break;
    case 2U: owner[0]=3U; response.first.client_instance_id().emplace().assign(owner,16U); break;
    case 3U: response.first.profile_id()=4U; break;
    case 4U: response.first.request_id()=999U; break;
    case 5U: response.first.operation_id()=72U; break;
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
  auto request=owned_fixture::request();
  request.operation()=wire_value(ManageOperation::GRANT_SESSION);
  request.expected_session_id()=42U;
  const auto id=client.submit(request,1U);
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};

  response.first.request_id()=id;
  response.first.session_id()=0U;
  response.first.receipt()=wire_value(WireReceipt::kQueued);
  response.first.transaction_phase()=wire_value(WireTransactionPhase::RECEIVED);
  response.first.result_code()=wire_value(WireResult::PENDING);
  transport.respond(response);
  if (!client.poll(id) || *transport.requests.back().queried_operation_id()!=id) return false;
  response.first.request_id()=*transport.requests.back().request_id();
  response.first.transaction_phase()=wire_value(WireTransactionPhase::PREPARED);
  transport.respond(response);
  const auto pending=client.poll(id);
  if (!pending || pending->result_valid || pending->accepted) return false;
  for (unsigned wait=0; wait<10; ++wait) if (client.poll(id)) return false;
  if (*transport.requests.back().queried_operation_id()!=id) return false;
  response.first.request_id()=*transport.requests.back().request_id();
  response.first.session_id()=42U; response.first.operation_id()=71U;
  response.first.transaction_phase()=wire_value(WireTransactionPhase::COMMITTED);
  response.first.result_code()=wire_value(WireResult::OK);
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
  auto request=owned_fixture::request();
  request.operation()=wire_value(ManageOperation::DISABLE_AND_RELEASE);
  request.expected_boot_epoch()=7U;
  request.expected_session_id()=42U;
  request.expected_profile_id()=3U;
  const auto id=client.submit(request,1U);
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};

  response.first.request_id()=id; response.second.boot_id=7U;
  response.first.profile_id()=3U; response.first.session_id()=42U;
  response.first.operation_id()=71U;
  response.first.receipt()=wire_value(WireReceipt::kQueued);
  if(via_query) {
  response.first.transaction_phase()=wire_value(WireTransactionPhase::RECEIVED);
  response.first.result_code()=wire_value(WireResult::PENDING);
  transport.respond(response);
  if(!client.poll(id) || *transport.requests.back().queried_operation_id()!=id) return false;
  response.first.request_id()=*transport.requests.back().request_id();
  response.first.session_id()=0U;
  response.first.transaction_phase()=wire_value(WireTransactionPhase::PREPARED);
  transport.respond(response);
  const auto pending=client.poll(id);
  if(!pending || pending->result_valid || pending->release_committed) return false;
  for(unsigned wait=0;wait<10;++wait) if(client.poll(id)) return false;
  response.first.request_id()=*transport.requests.back().request_id();
  }
  response.first.receipt()=terminal_receipt;
  response.first.session_id()=0U;
  response.first.transaction_phase()=wire_value(WireTransactionPhase::COMMITTED);
  response.first.result_code()=wire_value(WireResult::OK);
  response.first.release_committed()=true;
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
  Authorization auth;auth.identity=owned_fixture::request(); auth.session=auth.reference=auth.permit=true;
  auth.identity.expected_boot_epoch()=7; auth.identity.expected_session_id()=9;
  auth.identity.expected_reference_epoch()=1; auth.identity.expected_profile_id()=1;
  auth.identity.configured_mask()=1; auth.identity.expected_management_epoch()=1;
  auto commit=[&](std::uint32_t epoch) {
    const auto request=transport.requests.back();
    ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};
    response.first.request_id()=*request.request_id(); response.second.boot_id=7; response.first.session_id()=9;
    response.first.reference_epoch()=1; response.first.profile_id()=1; response.first.receipt()=1;
    response.first.operation_id()=*request.request_id()+100; response.first.transaction_phase()=3;
    response.first.motion_epoch()=epoch; response.first.management_epoch()=1;
    transport.respond(response); return client.poll(*request.request_id());
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
  if(!bootstrap.start_operation(snapshot,ManageOperation::CAPTURE_REFERENCE,20)) return false;
  // 独立管理协调器就绪后，真实共享WireClient发送RUN续租。
  const auto setup=transport.requests.back();
  // Bootstrap自行消费本次setup终态。
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};
  response.first.request_id()=*setup.request_id(); response.second.boot_id=7; response.first.session_id()=9;
  response.first.reference_epoch()=1; response.first.profile_id()=1; response.first.receipt()=1;
  response.first.operation_id()=*setup.request_id()+100; response.first.transaction_phase()=3;
  response.first.reference_epoch()=snapshot.reference_epoch;
  response.first.motion_epoch()=2; response.first.management_epoch()=snapshot.management_epoch;
  transport.respond(response); bootstrap.tick(21);
  if(!bootstrap.operation_complete() || !bootstrap.start_renew_lease(snapshot,30000000,22)) return false;
  if(mode==2U) {
    // 同共享client忙于其它operation，Complete不得把它冒充续租。
    auto setup_request=auth.identity; setup_request.operation()=wire_value(ManageOperation::CAPTURE_REFERENCE);
    auto finished=transport.requests.back();
    response.first.request_id()=*finished.request_id(); response.first.operation_id()=104;
    response.first.transaction_phase()=3; transport.respond(response); bootstrap.tick(22);
    if(!client.submit(setup_request,22)) return false;
  }
  /**
   * @par 续租/Complete仲裁
   * 保留尚未被bootstrap消费的Renew回执；Gate不能绕开它发Complete，也不能因此重置目标终态deadline。
   */
  const auto renewal=transport.requests.back();
  response.first.request_id()=*renewal.request_id(); response.first.operation_id()=0;
  response.first.transaction_phase()=1; response.first.result_code()=23;
  transport.respond(response); // 已到达Received Pending，Bootstrap尚未poll。
  Position end{}; end[0]=0.2;
  if(!gate.record_update(gate.generation(),2,17,end)) return false;
  gate.inner_result(gate.generation(),Terminal::kSucceeded,"",23);
  reply->accepted_seq=17; gate.observe(*reply,24);
  std::cout << "RENEW_COMPLETE_BOUNDARY renewal=" << *renewal.request_id()
            << " phase=" << static_cast<unsigned>(gate.phase()) << " error=" << gate.error() << '\n';
  if(mode==1U || mode==2U || mode==9U)
    return gate.terminal()==Terminal::kAborted && gate.error()=="TRANSPORT_UNAVAILABLE";
  if(gate.terminal()!=Terminal::kNone || !gate.error().empty() ||
     *transport.requests.back().operation()!=wire_value(ManageOperation::RENEW_LEASE)) return false;
  if(!gate.completion_waiting_for_renewal(25) || gate.completion_waiting_for_renewal(2000024)) return false;
  if(mode==4U) { gate.tick(2000024); return gate.terminal()==Terminal::kAborted &&
      *transport.requests.back().operation()!=wire_value(ManageOperation::COMPLETE_MOTION); }
  if(mode==5U) {
    const auto before_cancel_count=transport.requests.size();
    gate.cancel(25); gate.tick(26);
    auto cancel_live=auth.identity;cancel_live.operation()=wire_value(ManageOperation::CANCEL_MOTION);
    cancel_live.expected_motion_epoch()=2;
    if(gate.phase()!=Phase::kStopping || gate.terminal()!=Terminal::kNone ||
       !gate.error().empty() || !gate.cancel_waiting_for_renewal(cancel_live,26) ||
       gate.cancel_waiting_for_renewal(cancel_live,2000025) ||
       transport.requests.size()!=before_cancel_count ||
       *transport.requests.back().operation()!=wire_value(ManageOperation::RENEW_LEASE)) return false;
    auto wrong=cancel_live;wrong.expected_session_id()=10;
    if(gate.cancel_waiting_for_renewal(wrong,26)) return false;
    bootstrap.tick(27); // 原Received/Pending真实消费，生成匹配本Renew的Query。
    const auto query=transport.requests.back();
    if(*query.operation()!=wire_value(ManageOperation::GET_OPERATION)) return false;
    response.first.request_id()=*query.request_id();response.first.operation_id()=104;
    response.first.transaction_phase()=3;response.first.result_code()=0;
    transport.respond(response);bootstrap.tick(28); // 完成本次真实RUN续租，不重新续租。
    if(!bootstrap.operation_complete()) return false;
    gate.tick(29);
    const auto cancel_request=transport.requests.back();
    if(*cancel_request.operation()!=wire_value(ManageOperation::CANCEL_MOTION) ||
       *cancel_request.request_id()==0U || *cancel_request.request_id()==*renewal.request_id() ||
       *cancel_request.request_id()==*query.request_id() || transport.requests.size()!=before_cancel_count+2U ||
       gate.phase()!=Phase::kStopping || gate.terminal()!=Terminal::kNone || !gate.error().empty() ||
       gate.cancel_waiting_for_renewal(cancel_live,30)) return false;
    if(*cancel_request.expected_boot_epoch()!=*cancel_live.expected_boot_epoch() ||
       *cancel_request.expected_session_id()!=*cancel_live.expected_session_id() ||
       *cancel_request.expected_reference_epoch()!=*cancel_live.expected_reference_epoch() ||
       *cancel_request.expected_motion_epoch()!=*cancel_live.expected_motion_epoch() ||
       *cancel_request.expected_management_epoch()!=*cancel_live.expected_management_epoch() ||
       *cancel_request.expected_profile_id()!=*cancel_live.expected_profile_id() ||
       *cancel_request.configured_mask()!=*cancel_live.configured_mask() ||
       !std::equal(cancel_request.domain_device_id()->begin(),cancel_request.domain_device_id()->end(),cancel_live.domain_device_id()->begin()) ||
       !std::equal(cancel_request.client_instance_id()->begin(),cancel_request.client_instance_id()->end(),cancel_live.client_instance_id()->begin())) return false;
    gate.tick(31);
    return transport.requests.size()==before_cancel_count+2U &&
      std::count_if(transport.requests.begin()+static_cast<std::ptrdiff_t>(before_cancel_count),transport.requests.end(),
        [](const auto &q){return *q.operation()==wire_value(ManageOperation::CANCEL_MOTION);})==1 &&
      std::none_of(transport.requests.begin()+static_cast<std::ptrdiff_t>(before_cancel_count),transport.requests.end(),
        [](const auto &q){return *q.operation()==wire_value(ManageOperation::COMPLETE_MOTION) || *q.operation()==wire_value(ManageOperation::RENEW_LEASE);});
  }
  if(mode==6U) { gate.abort("TEST_STOP",25); gate.tick(26); return gate.terminal()==Terminal::kAborted &&
      *transport.requests.back().operation()!=wire_value(ManageOperation::COMPLETE_MOTION); }
  bootstrap.tick(25); // 消费Received Pending并发送本续租Query
  response.first.request_id()=*transport.requests.back().request_id(); response.first.operation_id()=104;
  response.first.transaction_phase()=mode==3U ? 4 : 3; response.first.result_code()=mode==3U ? 6 : 0;
  response.first.management_epoch()=mode==12U ? 2 : 1;
  if(mode==10U) response.first.reference_epoch()=2;
  if(mode==11U) response.first.motion_epoch()=3;
  transport.respond(response); bootstrap.tick(26);
  if(mode==13U) {
    auto intervening=auth.identity; intervening.operation()=wire_value(ManageOperation::CAPTURE_REFERENCE);
    if(!client.submit(intervening,26)) return false;
    if(!commit(2)) return false;
  }
  if(mode==14U) {
    auto state=owned_fixture::state();auto state_source=typed_management_fixture::source();
    state_source.boot_id=7; state.session_id()=9; state.reference_epoch()=1;
    state.motion_epoch()=2; state.management_epoch()=2; state.profile_id()=1;
    state.state_seq()=1; typed_management_fixture::configure(state,1); typed_management_fixture::position_quality(state,1);
    state.core_state()=wire_value(WireCoreState::kRunning); state.freshness_ticket()=55;
    if(!gate.observe_state(state,state_source,26))return false;
  }
  gate.tick(27);
  if(mode>=10U && mode<=14U) return gate.terminal()!=Terminal::kSucceeded &&
      *transport.requests.back().operation()!=wire_value(ManageOperation::COMPLETE_MOTION);
  if(mode==3U) return gate.phase()==Phase::kStopping &&
      *transport.requests.back().operation()!=wire_value(ManageOperation::COMPLETE_MOTION);
  const auto count=transport.requests.size();
  gate.tick(28);
  return transport.requests.size()==count && *transport.requests.back().operation()==8 &&
      *transport.requests.back().terminal_producer_seq()==17 &&
      *transport.requests.back().expected_management_epoch()==1 &&
      (*transport.requests.back().terminal_position_rad())[0]==0.2 && gate.terminal()==Terminal::kNone;
}

/**
 * @brief 短暂Status/State不相干时ReleasePending仍可Query，避免因等Idle而阻塞推进。
 * @param fault 0正常；1错stage、2错phase、3错op、4超时、5错boot、6无inflight、7发送失败、8新Idle State配旧Stopping Status。
 * 强回执只到ReleaseAwaitIdle；仍缺完整双流Idle时不能released。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool release_pending_polls_during_observation_grace(unsigned fault=0U) {
  Transport transport; WireManagementClient client(transport);
  auto identity=owned_fixture::request(); identity.expected_session_id()=9;
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
  ManagementResponse pending{owned_fixture::response(),typed_management_fixture::source()};
  pending.first.request_id()=*original.request_id(); pending.second.boot_id=7; pending.first.session_id()=9;
  pending.first.reference_epoch()=1; pending.first.motion_epoch()=10; pending.first.management_epoch()=1;
  pending.first.profile_id()=1; pending.first.receipt()=1; pending.first.transaction_phase()=1; pending.first.result_code()=23;
  transport.respond(pending);
  // 使用真实wire对象与Node同款continuity：新Status11/Stopping7，accepted State仍旧10/Hold4。
  auto status=owned_fixture::status();auto status_source=typed_management_fixture::source(); status_source.boot_id=7; status.session_id()=9;
  status.reference_epoch()=1; status.motion_epoch()=11; status.management_epoch()=2; status.core_state()=7;
  auto accepted_state=owned_fixture::state();auto accepted_state_source=typed_management_fixture::source(); accepted_state_source.boot_id=7; accepted_state.session_id()=9;
  accepted_state.reference_epoch()=1; accepted_state.motion_epoch()=10; accepted_state.management_epoch()=1;
  accepted_state.core_state()=4;
  const bool coherent=*status.motion_epoch()==*accepted_state.motion_epoch() &&
      *status.management_epoch()==*accepted_state.management_epoch();
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
      fault==3U ? ManageOperation::RENEW_LEASE : bootstrap.last_operation(),
      fault==6U ? false : bootstrap.release_request_inflight());
  if((fault>=1U && fault<=3U) || fault==6U) return !tick && transport.requests.size()==1U;
  if(fault==4U) {
    bootstrap.tick(5000101U);
    return bootstrap.phase()==BootstrapPhase::kFault && transport.requests.size()==1U;
  }
  if(tick) bootstrap.tick(101);
  if(transport.requests.size()!=2U || *transport.requests.back().operation()!=12U ||
     *transport.requests.back().queried_operation_id()!=*original.request_id()) return false;
  auto terminal=pending; terminal.first.request_id()=*transport.requests.back().request_id();
  terminal.second.boot_id=fault==5U ? 8 : 7; terminal.first.session_id()=0; terminal.first.reference_epoch()=0;
  terminal.first.motion_epoch()=11; terminal.first.management_epoch()=2; terminal.first.operation_id()=42;
  terminal.first.run_id()=2; terminal.first.transaction_phase()=3; terminal.first.result_code()=0; terminal.first.release_committed()=true;
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
  auto identity=owned_fixture::request(); identity.expected_session_id()=9;
  BootstrapCoordinator bootstrap(client,identity,{},0);
  BootstrapSnapshot hold{7,9,1,10,1,55,1,1,true,true};
  hold.core_state=hold.state_core_state=wire_value(WireCoreState::kHold);
  constexpr std::uint64_t request_time=1000000U, last_hold=request_time-32000U;
  if(!bootstrap.start_release(hold,request_time)) return false;
  const auto original=transport.requests.back();
  ManagementResponse reply{owned_fixture::response(),typed_management_fixture::source()};
  reply.second.boot_id=7; reply.first.session_id()=9; reply.first.reference_epoch()=1;
  reply.first.motion_epoch()=10; reply.first.management_epoch()=1; reply.first.profile_id()=1;
  reply.first.request_id()=*original.request_id(); reply.first.receipt()=1;
  reply.first.transaction_phase()=1; reply.first.result_code()=23;
  transport.respond(reply); bootstrap.tick(request_time+20000U);
  if(transport.requests.size()!=2U || *transport.requests.back().queried_operation_id()!=*original.request_id()) return false;
  reply.first.request_id()=*transport.requests.back().request_id(); reply.first.session_id()=0;
  reply.first.reference_epoch()=0; reply.first.motion_epoch()=11; reply.first.management_epoch()=2;
  reply.first.transaction_phase()=2; // 真实第一Query Prepared23/op0
  transport.respond(reply); bootstrap.tick(request_time+40000U);
  std::uint64_t committed_at=0U;
  for(std::uint64_t now=request_time+60000U;now<request_time+300000U;now+=20000U) {
    const auto count=transport.requests.size(); bootstrap.tick(now);
    if(transport.requests.size()>count) {
      const auto query=transport.requests.back();
      if(*query.operation()!=12U || *query.queried_operation_id()!=*original.request_id()) return false;
      reply.first.request_id()=*query.request_id(); reply.first.operation_id()=42;
      reply.first.transaction_phase()=3; reply.first.result_code()=0; reply.first.run_id()=2;
      reply.first.release_committed()=true; transport.respond(reply);
      bootstrap.tick(now+4000U); committed_at=now+4000U; break;
    }
  }
  std::cout << "RELEASE_QUERY_SUPERVISION request=" << *original.request_id() << " last_hold=" << last_hold
            << " committed_at=" << committed_at << " gap=" << committed_at-last_hold << '\n';
  return committed_at>0U && committed_at-last_hold<=250000U && bootstrap.confirmed_release().has_value();
}

/**
 * @brief Renew较慢Query节奏与Release较快节奏分开；Query两次重试沿用同RID/seq及queried原ID，第三次失败。
 * @return 所述检查全部成立为true；false供main报告失败。
 */
bool query_backoff_and_retry_identity_remain_bounded() {
  for(const auto operation:{ManageOperation::RENEW_LEASE,ManageOperation::DISABLE_AND_RELEASE}) {
    Transport transport; WireManagementClient client(transport);
    auto mutation=owned_fixture::request(); mutation.operation()=wire_value(operation);
    mutation.expected_boot_epoch()=7; mutation.expected_session_id()=9;
    mutation.expected_reference_epoch()=1; mutation.expected_motion_epoch()=10; mutation.expected_profile_id()=1;
    const auto id=client.submit(mutation,100);
    ManagementResponse reply{owned_fixture::response(),typed_management_fixture::source()};
    reply.first.request_id()=id; reply.second.boot_id=7; reply.first.session_id()=9;
    reply.first.reference_epoch()=1; reply.first.motion_epoch()=10; reply.first.profile_id()=1;
    reply.first.receipt()=1; reply.first.transaction_phase()=1; reply.first.result_code()=23;
    transport.respond(reply); (void)client.poll(id);
    reply.first.request_id()=*transport.requests.back().request_id(); reply.first.transaction_phase()=2;
    transport.respond(reply); (void)client.poll(id);
    const auto count=transport.requests.size();
    if(operation==ManageOperation::RENEW_LEASE) {
      for(unsigned poll=0;poll<9;++poll) { (void)client.poll(id); if(transport.requests.size()!=count) return false; }
    }
    (void)client.poll(id);
    if(transport.requests.size()!=count+1U) return false;
    const auto retry=transport.requests.back();
    for(unsigned attempt=0;attempt<2;++attempt) {
      transport.timeout_next=true; (void)client.poll(id);
      const auto resent=transport.requests.back();
      if(*resent.request_id()!=*retry.request_id() || *resent.request_seq()!=*retry.request_seq() ||
         *resent.queried_operation_id()!=id || *resent.operation()!=12U) return false;
    }
    transport.timeout_next=true;
    try { (void)client.poll(id); return false; } catch(const std::runtime_error &) {}
    for(const auto &request:transport.requests)
      if(*request.operation()!=12U && *request.request_id()!=id) return false;
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
  Transport transport; WireManagementClient client(transport); auto request=owned_fixture::request();
  request.expected_session_id()=9007199254740993ULL;
  request.operation()=wire_value(ManageOperation::GRANT_SESSION);
  const auto id=client.submit(request,123); if (!id || client.poll(id)) return 1;
  ManageRequest competing=request;
  competing.operation()=wire_value(ManageOperation::RENEW_LEASE);
  competing.requested_session_lease_us()=500000U;
  if(client.submit(competing,124)!=0U || transport.requests.size()!=1U ||
     client.original_sent_at_us()!=123U) return 15;
  if (transport.requests.size()!=1 || *transport.requests[0].expected_session_id()!=9007199254740993ULL) return 2;
  ManagementResponse response{owned_fixture::response(),typed_management_fixture::source()};
  response.first.request_id()=id;
  response.first.session_id()=9007199254740993ULL;
  response.first.receipt()=1; response.first.operation_id()=9007199254740995ULL; response.first.transaction_phase()=1;
  transport.respond(response); const auto queued=client.poll(id);
  if (!queued || !queued->queued || queued->accepted || queued->result_valid) return 3;
  const auto query=transport.requests.back();
  if (*query.operation()!=12 || *query.queried_operation_id()!=id || *query.request_id()==id) return 4;
  transport.timeout_next=true;
  if(client.poll(id) || *transport.requests.back().request_id()!=*query.request_id() ||
     *transport.requests.back().queried_operation_id()!=id || client.original_sent_at_us()!=123) return 11;
  response.first.request_id()=*query.request_id();
  response.first.operation_id()=9007199254740995ULL;
  response.first.transaction_phase()=2;
  const auto accepted_query_count=transport.requests.size();
  transport.respond(response); const auto accepted=client.poll(id);
  if (!accepted || !accepted->accepted || accepted->queued || accepted->result_valid) return 5;
  if(transport.requests.size()!=accepted_query_count) return 12;
  for(unsigned wait=0;wait<10;++wait) if(client.poll(id)) return 13;
  if(transport.requests.size()!=accepted_query_count+1) return 14;
  const auto terminal_query_count=transport.requests.size();
  response.first.request_id()=*transport.requests.back().request_id();
  response.first.result_code()=6; response.first.first_failure_reason()=7;
  response.first.transaction_phase()=4;
  transport.respond(response); const auto failed=client.poll(id);
  if (!failed || !failed->result_valid || failed->succeeded || client.original_sent_at_us()!=123 ||
      transport.requests.size()!=terminal_query_count || failed->result_code!=6 ||
      failed->first_failure_reason!=7 || failed->transaction_phase!=4) return 6;
  response.first.request_id()=999; transport.respond(response);
  if (client.poll(id)) return 7;
  const bool handshake=two_stream_handshake(); if (!handshake) return 8;
  const bool round_trip=*transport.requests[0].expected_session_id()==*request.expected_session_id();
  const bool request_preserved=*query.queried_operation_id()==id;
  if (!round_trip || !request_preserved) return 10;
  std::cout << std::boolalpha << "{\"two_stream_handshake\":" << handshake
    << ",\"owned_source_handoff\":" << round_trip << ",\"queued_is_accepted\":" << queued->accepted
    << ",\"original_request_preserved\":" << request_preserved
    << ",\"causal_t0\":" << client.original_sent_at_us() << ",\"evidence_scope\":\"component\"}\n";
}
