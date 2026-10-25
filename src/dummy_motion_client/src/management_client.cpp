/**
 * @file management_client.cpp
 * @brief 实现单槽wire管理mutation、因果Query链与可信许可提交。
 * 调用方串行使用client；本类拥有一个本地RequestHandle并借用transport。Queued、设备Accepted和COMMITTED/result分别报告，普通RPC不提供实际关节快照。poll不阻塞等网络，但会分配字节并可能在无诊断scope的失败路径fprintf，因此不是无分配硬实时API。
 */
#include "dummy_motion_client/management_client.hpp"
#include "dummy_ros_control/diagnostic_scope.hpp"
#include <algorithm>
#include <cstdio>
#include <stdexcept>
namespace dummy_motion_client {
namespace wire = dp::iface;
/**
 * @brief 只把完整同身份的在途RenewLease识别为Complete/Cancel的延期前因。
 * 比较boot/session/reference/motion/management/profile/mask以及device/client；不允许任意Busy事务占槽就成为延长完成/停止窗口的理由。
 * @param r 待协调的Complete或Cancel请求模板。
 * @return 原续租request_id；无句柄/待发Query、操作不符或任何绑定漂移为0。
 */
std::uint64_t WireManagementClient::matching_inflight_renewal(const ManageRequest &r) const {
  if (!(handle_ || pending_send_) || original_.operation()!=wire_value(ManageOperation::kRenewLease) ||
      (r.operation()!=wire_value(ManageOperation::kComplete) && r.operation()!=wire_value(ManageOperation::kCancel))) return 0U;
  const auto &o=original_;
  if(o.expected_boot_epoch()!=r.expected_boot_epoch() || o.expected_session_id()!=r.expected_session_id() ||
     o.expected_reference_epoch()!=r.expected_reference_epoch() || o.expected_motion_epoch()!=r.expected_motion_epoch() ||
     o.expected_profile_id()!=r.expected_profile_id() || o.configured_mask()!=r.configured_mask() ||
     o.expected_management_epoch()!=r.expected_management_epoch() ||
     !std::equal(std::begin(o.device_id()),std::end(o.device_id()),std::begin(r.device_id())) ||
     !std::equal(std::begin(o.client_instance_id()),std::end(o.client_instance_id()),std::begin(r.client_instance_id()))) return 0U;
  return o.request_id();
}
/**
 * @brief 报告当前或最近匹配续租根的有限完成状态。
 * @param id 原RenewLease请求ID，非Query wire ID或设备operation ID。
 * @return Pending表示仍有句柄/待发Query；Committed只来自同根成功且epoch未变的终结果；其余Failed。
 */
RenewalCompletion WireManagementClient::renewal_completion(std::uint64_t id) const {
  if(id!=0U && id==completed_renewal_id_ && original_.request_id()==id &&
     original_.operation()==wire_value(ManageOperation::kRenewLease) && !handle_ && !pending_send_)
    return completed_renewal_result_;
  if(id!=0U && original_.request_id()==id && original_.operation()==wire_value(ManageOperation::kRenewLease) &&
     (handle_ || pending_send_)) return RenewalCompletion::kPending;
  return RenewalCompletion::kFailed;
}
/**
 * @brief 编码本轮请求并替换本地等待句柄。
 * 复制到vector后传transport；销毁前一handle只取消本地等待。Query无handle时保留current/wire_id便于有限准入重试，不自动重发原mutation。
 * @param request 原mutation或后续Query完整wire值。
 * @return 有handle或Query可pending_send时true；普通mutation无handle则false。
 */
bool WireManagementClient::send(const ManageRequest &request) {
  std::vector<std::uint8_t> bytes(wire::DummyManageV2RequestCodec::kEncodedSize);
  if (wire::DummyManageV2RequestCodec::encode(request,bytes.data(),bytes.size()) != wire::CodecResult::kOk)
    return false;
  // Release an unsent query before a cancellation replaces it in the bounded queue.
  /**
   * @brief 替换本地请求所有权：旧handle析构通知取消观察，不等于远端管理操作已撤回或电机已停；actual结果仍需Query/状态验证。
   */
  handle_.reset();
  current_=request; pending_send_=false; query_retries_=0; query_delay_polls_=0;
  handle_=transport_.send(bytes); wire_id_=request.request_id();
  pending_send_=!handle_ && request.operation()==wire_value(ManageOperation::kQuery);
  return bool(handle_) || pending_send_;
}
/**
 * @brief 在管理单槽空闲时提交非SetPermit操作。
 * 补schema和独立request ID/序号后保存original；不在此直接验证全部设备前置，操作结果由poll解释，ActionGate/Bootstrap再与State合证。
 * @param request 包含设备/客户端与expected身份的模板。
 * @param sent_at_us 宿主steady µs原提交锚，整个Query链保持不变。
 * @return 新根request_id；忙、SetPermit误用或本地send失败为0。
 */
std::uint64_t WireManagementClient::submit(const ManageRequest &request, std::uint64_t sent_at_us) {
  if (handle_ || pending_send_ ||
      request.operation()==wire_value(ManageOperation::kSetPermit)) return 0U;
  original_=request; original_.set_schema_major(2); original_.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  original_.set_request_id(take_request_id());
  original_.set_request_seq(take_request_sequence());
  sent_at_us_=sent_at_us; operation_id_=0; accepted_=false;
  return send(original_) ? original_.request_id() : 0;
}
/**
 * @brief 仅按可信绑定构造SetPermit摘要并提交。
 * 实际预留request_seq参与trusted_bench_permit_digest，owner取client_instance_id并绑定session；随后把生成摘要填入副本，不能通过普通submit绕过此路径。摘要提交不表示设备已授权。
 * @param request SetPermit模板，需操作员确认、非零permit和非零compiled policy。
 * @param binding boot/reference/profile及产物身份，与请求对应。
 * @param sent_at_us 原始宿主提交µs。
 * @return 根ID或0；摘要/身份/操作/忙等任一条件不满足均不提交。
 */
std::uint64_t WireManagementClient::submit_permit(
    const ManageRequest &request, const PermitBinding &binding, std::uint64_t sent_at_us) {
  if (handle_ || pending_send_ ||
      request.operation()!=wire_value(ManageOperation::kSetPermit) ||
      !request.operator_confirmed() || request.permit_remaining_us()==0U ||
      std::all_of(std::begin(request.compiled_policy_sha256()),
                  std::end(request.compiled_policy_sha256()),
                  [](auto value) { return value==0U; }) ||
      binding.boot_epoch!=request.expected_boot_epoch() ||
      binding.reference_epoch!=request.expected_reference_epoch() ||
      binding.profile_id!=request.expected_profile_id()) return 0U;
  std::array<std::uint8_t,16> owner{};
  std::copy(std::begin(request.client_instance_id()), std::end(request.client_instance_id()), owner.begin());
  std::array<std::uint8_t,32> digest{};
  /**
   * @brief 先预留真实共享序号，再把它和owner/session及可信binding用于permit摘要；不能先计算摘要后随意更换序号。
   */
  const auto permit_sequence = take_request_sequence();
  if (!trusted_bench_permit_digest(binding, owner, request.expected_session_id(), permit_sequence, &digest))
    return 0U;
  original_=request; original_.set_schema_major(2); original_.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  original_.set_request_id(take_request_id()); original_.set_request_seq(permit_sequence);
  std::uint8_t wire_digest[32]{};
  std::copy(digest.begin(), digest.end(), wire_digest);
  original_.set_permit_sha256(wire_digest);
  sent_at_us_=sent_at_us; operation_id_=0; accepted_=false;
  return send(original_) ? original_.request_id() : 0U;
}
/**
 * @brief 非阻塞消费一个根的响应并推进有限Query状态机。
 * 同一root只发送一次mutation。Pending后Query使用新wire ID/序号追问original request_id；保持原sent_at，绑定稳定operation。返回succeeded只表示可信COMMITTED结果，不补实际位置(snapshot_valid=false)，运动/释放验收由上层合证。
 * @param request_id 调用者持有的原mutation根ID，不是每轮Query ID。
 * @return optional中间/终态ManageReply；未完成为空。codec/身份/receipt错误、不可重试传输失败或Query重试耗尽抛runtime_error。
 */
std::optional<ManageReply> WireManagementClient::poll(std::uint64_t request_id) {
  if (request_id != original_.request_id()) return {};
  /**
   * @brief 只对当前Query的本地传输问题做至多3次重试。
   * 原mutation失败不进入Query重试；第四次失败抛runtime_error。重编码同current Query，不重发Enable/Begin/Cancel等副作用根；不能靠新handle重启外层原deadline。
   * @param cause 失败原因文本，写固定诊断或无scope时限路径stderr。
   */
  const auto retry_query=[this](const char *cause) {
    if(current_.operation()!=wire_value(ManageOperation::kQuery) || ++query_retries_>3) {
      if(auto* journal=dummy_ros_control::current_host_diagnostics) {
        const dummy_ros_control::DiagnosticIdentity expected{original_.expected_boot_epoch(),original_.expected_session_id(),
          original_.expected_reference_epoch(),original_.expected_motion_epoch(),current_.request_seq(),original_.expected_management_epoch(),true};
        journal->reject(dummy_ros_control::DiagnosticReason::kManagementRejected,{},expected,dummy_ros_control::host_diagnostic_now_us(),cause,query_retries_);
        journal->detail("MANAGE_RETRY_EXHAUSTED cause=%s retries=%u",cause,query_retries_);
      } else {
      std::fprintf(stderr,
          "MANAGE_RETRY_EXHAUSTED cause=%s original_operation=%u original_request=%llu "
          "wire_request=%llu retries=%u pending_send=%d\n",
          cause,static_cast<unsigned>(original_.operation()),
          static_cast<unsigned long long>(original_.request_id()),
          static_cast<unsigned long long>(wire_id_),query_retries_,pending_send_);
      }
      throw std::runtime_error("manage_v2 request transport unavailable");
    }
    handle_.reset();
    std::vector<std::uint8_t> bytes(wire::DummyManageV2RequestCodec::kEncodedSize);
    if(wire::DummyManageV2RequestCodec::encode(current_,bytes.data(),bytes.size())!=wire::CodecResult::kOk)
      throw std::runtime_error("manage_v2 query encoding failed");
    handle_=transport_.send(bytes); pending_send_=!handle_;
  };
  /**
   * @brief 本地尚未接纳的Query只在节流轮询数归零后重试；返回空不是设备Pending证据，外层仍按原submit锚监督总期限。
   */
  if(pending_send_) {
    if(query_delay_polls_!=0U) { --query_delay_polls_; return {}; }
    retry_query("pending_send"); return {};
  }
  if(!handle_) return {};
  std::optional<std::vector<std::uint8_t>> bytes;
  try { bytes=handle_->poll(); }
  catch(const std::runtime_error &) { retry_query("handle_failed"); return {}; }
  if (!bytes) return {};
  wire::DummyManageV2Response response;
  if (wire::DummyManageV2ResponseCodec::decode(bytes->data(),bytes->size(),&response) != wire::CodecResult::kOk)
    throw std::runtime_error("invalid manage_v2 response codec");
  /**
   * @brief 首响应与Query响应的session例外不同：首次Received可尚无session，GrantSession的Pending Query可session0，释放允许owner清零；其他错session拒绝。
   */
  const bool original_receipt = wire_id_ == original_.request_id();
  const bool release = original_.operation()==wire_value(ManageOperation::kDisableAndRelease);
  const bool pending_grant_query = !original_receipt &&
      original_.operation()==wire_value(ManageOperation::kGrantSession) &&
      current_.operation()==wire_value(ManageOperation::kQuery) &&
      current_.queried_operation_id()==request_id &&
      response.transaction_phase()==wire_value(WireTransactionPhase::kPrepared) &&
      response.result_code()==wire_value(WireResult::kPending) &&
      (response.operation_id()==0U || operation_id_==0U || response.operation_id()==operation_id_);
  const bool session_matches = response.session_id()==original_.expected_session_id() ||
      (original_receipt && response.transaction_phase()==wire_value(WireTransactionPhase::kReceived) && response.session_id()==0U) ||
      (pending_grant_query && response.session_id()==0U) ||
      (release && response.session_id()==0U);
  /**
   * @brief 身份检查使用实际本轮wire_id回显，boot/profile/device/client匹配原mutation根；不能仅凭result=Ok把别的请求或设备响应接入。
   */
  if (
      response.schema_major()!=2 || response.request_id()!=wire_id_ ||
      response.boot_epoch()!=original_.expected_boot_epoch() ||
      !session_matches ||
      response.profile_id()!=original_.expected_profile_id() ||
      !std::equal(std::begin(response.device_id()),std::end(response.device_id()),std::begin(original_.device_id())) ||
      !std::equal(std::begin(response.client_instance_id()),std::end(response.client_instance_id()),std::begin(original_.client_instance_id())))
    throw std::runtime_error("invalid manage_v2 response identity or codec");
  ManageReply result; result.request_id=request_id;
  result.result_code=response.result_code();
  result.first_failure_reason=response.first_failure_reason();
  result.transaction_phase=response.transaction_phase();
  result.release_committed=response.release_committed();
  const bool original_response=wire_id_==request_id;
  const bool initial_query = !original_response && operation_id_==0U &&
      current_.queried_operation_id()==request_id;
  /**
   * @brief operation_id首次出现可建立因果绑定，之后必须稳定；原root和其首Query可先于非零operation到达，故不把所有Pending都强称已接纳。
   */
  const bool correlated=original_response || initial_query ||
      (operation_id_!=0U && response.operation_id()==operation_id_);
  result.operation_id=response.operation_id();
  const auto phase=response.transaction_phase();
  if (correlated && response.operation_id()!=0U) {
    if (operation_id_!=0U && operation_id_!=response.operation_id())
      throw std::runtime_error("manage_v2 operation identity changed");
    operation_id_=response.operation_id();
  }
  /**
   * @brief Accepted要求已关联非零operation且phase为Prepared/Committed；Queued只指Received的队列/重放receipt，两者不是动作完成。
   */
  result.accepted=correlated && response.operation_id()!=0U &&
      phase>=wire_value(WireTransactionPhase::kPrepared) &&
      phase<=wire_value(WireTransactionPhase::kCommitted);
  if (result.accepted) {
    accepted_=true;
  }
  result.queued=phase==wire_value(WireTransactionPhase::kReceived) &&
      (response.receipt()==wire_value(WireReceipt::kQueued) || response.receipt()==wire_value(WireReceipt::kReplayed));
  /**
   * @brief 只有Committed或Failed且result非Pending才可形成result_valid；终receipt还必须Queued/Replayed，畸形组合直接异常，不猜测成功。
   */
  const bool terminal=phase==wire_value(WireTransactionPhase::kCommitted) ||
      phase==wire_value(WireTransactionPhase::kFailed);
  if(terminal && response.result_code()!=wire_value(WireResult::kPending) &&
     response.receipt()!=wire_value(WireReceipt::kQueued) &&
     response.receipt()!=wire_value(WireReceipt::kReplayed))
    throw std::runtime_error("invalid manage_v2 terminal receipt");
  result.result_valid=correlated && terminal && response.result_code()!=wire_value(WireResult::kPending) &&
      (operation_id_==0 || response.operation_id()==operation_id_);
  result.succeeded=result.result_valid && phase==wire_value(WireTransactionPhase::kCommitted) &&
      response.result_code()==wire_value(WireResult::kOk);
  result.rejected=result.result_valid && phase==wire_value(WireTransactionPhase::kFailed);
  /**
   * @brief 普通manage响应只给事务/epoch/commit元数据，不携可信实际位置；State快照必须由ActionGate独立接收后合并。
   */
  result.snapshot_valid=false;
  result.motion_epoch=response.motion_epoch();
  result.session_id=response.session_id(); result.reference_epoch=response.reference_epoch();
  result.management_epoch=response.management_epoch();
  result.accepted_seq=response.committed_producer_seq();
  if (result.result_valid) {
    /**
     * @brief 续租终态除了succeeded还要求reference/motion/management与原请求完全不变，才能缓存Committed；不因此延长motion permit或跨run套用。
     */
    if(original_.operation()==wire_value(ManageOperation::kRenewLease)) {
      completed_renewal_id_=original_.request_id();
      const bool same_binding=result.reference_epoch==original_.expected_reference_epoch() &&
          result.motion_epoch==original_.expected_motion_epoch() &&
          result.management_epoch==original_.expected_management_epoch();
      completed_renewal_result_=result.succeeded && same_binding
          ? RenewalCompletion::kCommitted : RenewalCompletion::kFailed;
    }
    handle_.reset();
    pending_send_=false;
    return result;
  }
  // Poll with a fresh query ID, retaining the original mutation ID and causal t0.
  // A lost transport reply retains this same handle; it never resubmits Enable/Begin.
  /**
   * @brief 从original复制查询保留固定expected身份和因果根，重新分配本轮wire请求ID/序号；queried_operation_id始终指原mutation request_id。
   */
  auto query=original_; query.set_operation(static_cast<std::uint16_t>(ManageOperation::kQuery));
  query.set_request_id(take_request_id());
  query.set_request_seq(take_request_sequence());
  query.set_queried_operation_id(request_id);
  if(original_response) {
    if (!send(query)) throw std::runtime_error("manage_v2 query transport unavailable");
  } else {
    handle_.reset(); current_=query; wire_id_=query.request_id(); pending_send_=true;
    query_retries_=0U;
    // Release的原owner State可能已清除；20ms Node周期下9次poll会耗尽250ms监督。
    // Cancel/Release停止事务下一tick继续Query，其它mutation保留既有节流。
    /**
     * @brief 后续Cancel/Release Query不做9次poll节流以免耗尽原停止窗口；其他mutation保留节流。这是查询频率，不改变外层deadline。
     */
    query_delay_polls_=(original_.operation()==wire_value(ManageOperation::kDisableAndRelease) ||
                        original_.operation()==wire_value(ManageOperation::kCancel)) ? 0U : 9U;
  }
  return result;
}
}  // namespace dummy_motion_client
