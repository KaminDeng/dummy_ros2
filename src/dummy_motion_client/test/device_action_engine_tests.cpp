/**
 * @file device_action_engine_tests.cpp
 * @brief ActionGate纯内存生命周期、ACK/Hold排空及取消失败场景。
 *
 * Client为FIFO合成回执，Port只累计方法调用；State wire字节只注入内存ObservationTracker，没有ROS/USB/CAN。
 * Host时间为人工µs，点时间秒、位置rad。Canceled/Succeeded、ready与强cancel receipt分别核，不将状态标签当物理停止。
 */
#include "dummy_motion_client/action_gate.hpp"
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace dummy_motion_client;
/**
 * @brief 用异常保留Release构建中的断言。
 * @param value 待核关系。
 * @return 无；false抛runtime_error。
 */
void check(bool value) { if (!value) throw std::runtime_error("lifecycle assertion failed"); }
/**
 * @brief FIFO管理回执夹具，可使Cancel发送失败或poll抛异常。
 */
struct Client : ManagementClient {
  /**
   * @brief reject_cancel控制Cancel发送0，throw_on_poll持续控制异常；requests记录请求，replies由用例按需压入。
   */
  bool reject_cancel=false;
  bool throw_on_poll=false;
  std::vector<ManageRequest> requests;
  std::deque<ManageReply> replies;
  /**
   * @brief 按需拒绝Cancel，否则保存请求并分配本地ID。
   * @param request 管理请求。
   * @par 未命名参数
   * Hostµs被忽略。
   * @return 0发送失败或requests.size()。
   */
  std::uint64_t submit(const ManageRequest &request, std::uint64_t) override {
    if(reject_cancel && request.operation()==wire_value(ManageOperation::kCancel)) return 0;
    auto copy = request; copy.set_request_id(requests.size()+1); requests.push_back(copy);
    return copy.request_id();
  }
  /**
   * @brief 从FIFO取一份回执或触发注入异常。
   * @par 未命名参数
   * 本地请求ID不用于匹配，测试构造响应负责相关性。
   * @return 首回执或空；throw_on_poll时抛runtime_error。
   */
  std::optional<ManageReply> poll(std::uint64_t) override {
    if (throw_on_poll) throw std::runtime_error("management poll failed");
    if (replies.empty()) return {}; auto next = replies.front(); replies.pop_front(); return next;
  }
};
/**
 * @brief 计数ActionPort；stream_hold只模拟存在持续Hold生产者，不执行轨迹。
 */
struct Port : ActionPort {
  /**
   * @brief prime/start/disable/cancel累计调用数；与producer_seq、CAN组或轴数不同。
   */
  int primes=0, starts=0, disables=0, cancels=0;
  bool stream_hold=false;
  /**
   * @brief 未命名Position(rad)/epoch/generation只匹配接口，累加primes。
   * @return 无。
   */
  void prime(const Position &, std::uint32_t, std::uint64_t) override { ++primes; }
  /**
   * @brief 未命名Trajectory/generation不执行，累加starts。
   * @return 无。
   */
  void start(const Trajectory &, std::uint64_t) override { ++starts; }
  /**
   * @brief 累加disables，不发送设备失能。
   * @return 无。
   */
  void disable() override { ++disables; }
  /**
   * @brief 累加cancels，不构成停止回执。
   * @return 无。
   */
  void cancel() override { ++cancels; }
  /**
   * @brief 模拟冻结持续Hold生产者；stream_hold时调用本Port disable。
   * @return 是否存在可冻结的流；这只是本夹具的排空前提。
   */
  bool freeze_hold() override { if (!stream_hold) return false; disable(); return true; }
};
/**
 * @brief 构造J1目标0.2rad、点时间1s、终速度0rad/s的单点轨迹。
 * @return 仅运动学输入值，无碰撞/障碍物信息。
 */
Trajectory trajectory() { return {{"Joint1"}, {{{0.2},{0.0},{},1.0}}, {}, {},0.0}; }
/**
 * @brief 构造J1范围±1rad、速度1rad/s、加速度2rad/s²、path/goal容差0.1rad。
 * @return 值类型profile；不从真实模型或设备加载。
 */
JointProfile profile() { return {{"Joint1"}, {-1}, {1}, {1}, {2}, {0.1}, {0.1}}; }
/**
 * @brief 合成管理快照及可选成功结果。
 * @param id 关联请求ID，operation=id+100。
 * @param epoch 人工motion epoch。
 * @param success 同时置result_valid/succeeded/hold_ready。
 * @return 位置0rad、accepted_seq1、session9的夹具回执。
 */
ManageReply reply(std::uint64_t id, std::uint32_t epoch, bool success=false) {
  ManageReply result; result.request_id=id; result.accepted=true; result.operation_id=id+100;
  result.result_valid=success; result.succeeded=success; result.snapshot_valid=true;
  result.motion_epoch=epoch; result.position={}; result.hold_position={};
  result.hold_available=true; result.hold_ready=success; result.accepted_seq=1;
  result.session_id=9U;
  return result;
}
/**
 * @brief 外来session/ref不得成为confirmed_cancel_receipt。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 即使路径最终为Canceled，强取消凭据仍须为空；终态标签和可信owner证明不可混同。
 */
void cancel_receipt_rejects_foreign_owner() {
  for (const bool wrong_session : {true,false}) {
    Client client; Port port; ActionGate gate(client,port,profile());
    Authorization authorization;
    authorization.session=authorization.reference=authorization.permit=true;
    authorization.identity.set_expected_session_id(9U);
    authorization.identity.set_expected_reference_epoch(3U);
    check(gate.enable(authorization,1U));
    client.replies.push_back(reply(1U,1U)); gate.tick(2U);
    check(gate.record_update(gate.generation(),1U,1U,Position{}));
    client.replies.push_back(reply(1U,1U,true)); gate.tick(3U);
    check(gate.submit(trajectory(),4U));
    client.replies.push_back(reply(2U,2U)); gate.tick(5U);
    check(gate.record_update(gate.generation(),2U,1U,Position{}));
    client.replies.push_back(reply(2U,2U,true)); gate.tick(6U);
    gate.cancel(7U);
    client.replies.push_back(reply(3U,3U)); gate.tick(8U);
    if(!gate.record_update(gate.generation(),3U,1U,Position{}))
      throw std::runtime_error("foreign owner stop hold update rejected");
    auto committed=reply(3U,3U,true);
    committed.transaction_phase=wire_value(WireTransactionPhase::kCommitted);
    committed.session_id=wrong_session ? 8U : 9U;
    committed.reference_epoch=wrong_session ? 3U : 4U;
    client.replies.push_back(committed); gate.tick(9U);
    if(gate.terminal()!=Terminal::kCanceled)
      throw std::runtime_error("foreign owner setup did not reach canceled terminal");
    check(!gate.confirmed_cancel_receipt());
  }
}
/**
 * @brief 允许当前测量Hold在规划范围外，但不放行改变后的越界目标。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 正限位[0.6108,π]rad内的J3式profile中实际Hold0rad可作为原地参考，改到0.1rad则拒绝。
 */
void hold_prime_outside_planning_range() {
  Client client; Port port;
  auto limits=profile();
  limits.lower[0]=0.6108652382;
  limits.upper[0]=3.1415926536;
  ActionGate gate(client,port,limits);
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_session_id(9U);
  check(gate.enable(authorization,1U));
  client.replies.push_back(reply(1U,1U));
  gate.tick(2U);
  check(port.primes==1);
  check(gate.record_update(gate.generation(),1U,1U,Position{}));
  client.replies.push_back(reply(1U,1U,true));
  gate.tick(3U);
  check(gate.ready());
  check(gate.record_update(gate.generation(),1U,2U,Position{}));
  Position changed{}; changed[0]=0.1;
  check(!gate.record_update(gate.generation(),1U,3U,changed));
}
/**
 * @brief 重复失权不重发disable/cancel或重复增加generation。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 保留第一次SESSION_CHANGED和Aborted，晚到的旧代回执不恢复权限。
 */
void repeated_authorization_loss_is_idempotent() {
  Client client; Port port; ActionGate gate(client,port,profile());
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_session_id(9U);
  check(gate.enable(authorization,1U));
  gate.invalidate_authorization(2U);
  const auto generation=gate.generation();
  const int disables=port.disables;
  const int cancels=port.cancels;
  gate.invalidate_authorization(3U);
  check(gate.generation()==generation && port.disables==disables && port.cancels==cancels);
  check(gate.terminal()==Terminal::kAborted && gate.error()=="SESSION_CHANGED");
}
/**
 * @brief Enable以bootstrap motion4为锚，等新motion5才prime，Begin再转6。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 同时覆盖大于32位的逻辑ID，不能被截断或把旧epoch快照当新保持。
 */
void enable_preserves_bootstrap_motion_epoch() {
  Client client; Port port; ActionGate gate(client,port,profile());
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_boot_epoch(14003434958659749721ULL);
  authorization.identity.set_expected_session_id(6420589637319258177ULL);
  authorization.identity.set_expected_reference_epoch(1U);
  authorization.identity.set_expected_motion_epoch(4U);
  authorization.identity.set_expected_management_epoch(12U);
  check(gate.enable(authorization,1U));
  check(client.requests.size()==1U);
  const auto &enable=client.requests.front();
  check(enable.operation()==wire_value(ManageOperation::kEnable));
  check(enable.expected_boot_epoch()==authorization.identity.expected_boot_epoch());
  check(enable.expected_session_id()==authorization.identity.expected_session_id());
  check(enable.expected_reference_epoch()==1U);
  check(enable.expected_motion_epoch()==4U);
  check(enable.expected_management_epoch()==12U);
  gate.observe(reply(1U,4U),2U);
  check(port.primes==0 && gate.phase()==Phase::kIdle);
  gate.observe(reply(1U,5U),3U);
  check(port.primes==1);
  check(gate.record_update(gate.generation(),5U,1U,Position{}));
  gate.observe(reply(1U,5U,true),4U);
  check(gate.ready());
  check(gate.submit(trajectory(),5U));
  check(client.requests.back().operation()==wire_value(ManageOperation::kBegin));
  check(client.requests.back().expected_motion_epoch()==5U);
  gate.observe(reply(2U,6U),6U);
  const auto generation=gate.generation();
  check(gate.record_update(generation,6U,1U,Position{}));
  gate.observe(reply(2U,6U,true),7U);
  Position terminal{}; terminal[0]=0.2;
  check(gate.record_update(generation,6U,2U,terminal));
  /**
   * @brief JTC式内部成功仅触发终点冻结，不能直接形成公开Succeeded；还须终点ACK、Complete及新Hold。
   */
  gate.inner_result(generation,Terminal::kSucceeded,"",8U);
  auto accepted=reply(2U,6U,true);
  accepted.accepted_seq=2U;
  gate.observe(accepted,9U);
  check(client.requests.back().operation()==wire_value(ManageOperation::kComplete));
  check(client.requests.back().expected_motion_epoch()==6U);
}
/**
 * @brief 六轴Hold全部submitted/sent及正式drain后才可发Begin。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 部分mask31、完整63但未排空都不够；管理回执不能代替最后一组Hold提交。
 */
void begin_waits_for_final_hold_commit() {
  namespace wire=dp::iface;
  auto six_axis=profile();
  six_axis.joints={"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"};
  six_axis.lower.assign(6U,-1.0); six_axis.upper.assign(6U,1.0);
  six_axis.velocity.assign(6U,1.0); six_axis.acceleration.assign(6U,2.0);
  six_axis.path_tolerance.assign(6U,0.1); six_axis.goal_tolerance.assign(6U,0.1);
  auto motion=trajectory(); motion.joints=six_axis.joints;
  motion.points.front().positions.assign(6U,0.2);
  motion.points.front().velocities.assign(6U,0.0);
  Client client; Port port; port.stream_hold=true; ActionGate gate(client,port,six_axis);
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_boot_epoch(7U);
  authorization.identity.set_expected_session_id(9U);
  authorization.identity.set_expected_profile_id(1U);
  authorization.identity.set_configured_mask(0x3FU);
  check(gate.enable(authorization,1U));
  gate.observe(reply(1U,1U),2U);
  check(gate.record_update(gate.generation(),1U,1U,Position{}));
  gate.observe(reply(1U,1U,true),3U);
  check(gate.ready());
  check(gate.submit(motion,4U));
  check(port.disables==2 && client.requests.size()==1U && !gate.ready());
  wire::DummyJointStateV2 state;
  state.set_schema_major(2U); state.set_schema_minor(dummy_ref::protocol::kSchemaMinor); state.set_boot_epoch(7U);
  state.set_session_id(9U); state.set_profile_id(1U);
  state.set_configured_mask(0x3FU); state.set_position_valid_mask(0x3FU);
  state.set_core_state(wire_value(WireCoreState::kHold));
  state.set_motion_epoch(1U); state.set_hold_ready(true);
  state.set_can_submitted_mask(0x3FU); state.set_can_sent_mask(0x1FU);
  auto observe=[&](std::uint32_t state_seq,std::uint32_t committed,std::uint64_t now) {
    state.set_state_seq(state_seq); state.set_committed_producer_seq(committed);
    std::vector<std::uint8_t> bytes(wire::DummyJointStateV2Codec::kEncodedSize);
    check(wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())==wire::CodecResult::kOk);
    check(gate.observe_state(bytes.data(),bytes.size(),now));
  };
  observe(1U,1U,5U);
  check(client.requests.size()==1U);
  state.set_can_submitted_mask(0x3FU); state.set_can_sent_mask(0x3FU);
  observe(2U,1U,6U);
  check(client.requests.size()==1U);
  gate.observe_hold_drain({7U,9U,1U,0U,1U,1U},7U);
  observe(3U,1U,8U);
  check(client.requests.size()==2U &&
        client.requests.back().operation()==wire_value(ManageOperation::kBegin));
}
/**
 * @brief 本地已处理但被丢弃的Hold代不要求不存在的CAN提交。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * processed generation343、committed342用于区分本地排队进度与实际发组水位。
 */
void discarded_local_hold_generation_does_not_block_begin() {
  namespace wire=dp::iface;
  Client client; Port port; port.stream_hold=true;
  ActionGate gate(client,port,profile());
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_session_id(9U);
  authorization.identity.set_configured_mask(1U);
  check(gate.enable(authorization,1U));
  gate.observe(reply(1U,1U),2U);
  check(gate.record_update(gate.generation(),1U,342U,Position{}));
  gate.observe(reply(1U,1U,true),3U);
  wire::DummyJointStateV2 state;
  state.set_schema_major(2U); state.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  state.set_session_id(9U); state.set_configured_mask(1U);
  state.set_position_valid_mask(1U); state.set_motion_epoch(1U);
  state.set_core_state(wire_value(WireCoreState::kHold)); state.set_hold_ready(true);
  state.set_committed_producer_seq(342U);
  state.set_can_submitted_mask(1U); state.set_can_sent_mask(1U);
  auto observe=[&](std::uint32_t seq,std::uint64_t now) {
    state.set_state_seq(seq);
    std::vector<std::uint8_t> bytes(wire::DummyJointStateV2Codec::kEncodedSize);
    check(wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())==wire::CodecResult::kOk);
    check(gate.observe_state(bytes.data(),bytes.size(),now));
  };
  observe(1U,4U);
  check(gate.record_update(gate.generation(),1U,343U,Position{}));
  check(gate.submit(trajectory(),5U));
  check(client.requests.size()==1U);
  gate.observe_hold_drain({0U,9U,343U,0U,1U,342U},6U);
  observe(2U,7U);
  check(client.requests.size()==2U &&
        client.requests.back().operation()==wire_value(ManageOperation::kBegin));
}
/**
 * @brief 等待Hold排空期间仍接受取消且遵守超时。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 内部make_ready(client,gate)只布置内存排空等待，无返回；超时走Cancel并保留HOLD_DRAIN_TIMEOUT。
 */
void pending_hold_drain_can_cancel_or_timeout() {
  const auto make_ready=[](Client &client,ActionGate &gate) {
    Authorization authorization;
    authorization.session=authorization.reference=authorization.permit=true;
    authorization.identity.set_expected_session_id(9U);
    check(gate.enable(authorization,1U));
    gate.observe(reply(1U,1U),2U);
    check(gate.record_update(gate.generation(),1U,1U,Position{}));
    gate.observe(reply(1U,1U,true),3U);
    check(gate.ready());
    check(gate.submit(trajectory(),4U));
    check(client.requests.size()==1U && !gate.ready());
  };
  Client canceled_client; Port canceled_port; canceled_port.stream_hold=true;
  ActionGate canceled(canceled_client,canceled_port,profile());
  make_ready(canceled_client,canceled);
  canceled.cancel(5U);
  check(canceled_client.requests.size()==2U &&
        canceled_client.requests.back().operation()==wire_value(ManageOperation::kCancel) &&
        canceled_port.cancels==1);
  Client timed_client; Port timed_port; timed_port.stream_hold=true;
  ActionGate timed(timed_client,timed_port,profile());
  make_ready(timed_client,timed);
  timed.tick(1000004U);
  check(timed_client.requests.size()==2U &&
        timed_client.requests.back().operation()==wire_value(ManageOperation::kCancel) &&
        timed.error()=="HOLD_DRAIN_TIMEOUT");
}
/**
 * @brief ACK设备拒绝先发生时后续失权不得覆盖其类型错误。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 保留DEVICE_COMMAND_REJECTED_DURING_ACK，避免故障归因漂移。
 */
void typed_ack_reject_survives_session_invalidation() {
  Client client; Port port; port.stream_hold=true;
  ActionGate gate(client,port,profile());
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_session_id(9U);
  check(gate.enable(authorization,1U));
  gate.observe(reply(1U,1U),2U);
  check(gate.record_update(gate.generation(),1U,1U,Position{}));
  gate.observe(reply(1U,1U,true),3U);
  check(gate.submit(trajectory(),4U));
  gate.abort("DEVICE_COMMAND_REJECTED_DURING_ACK",5U);
  gate.invalidate_authorization(6U);
  check(gate.terminal()==Terminal::kAborted &&
        gate.error()=="DEVICE_COMMAND_REJECTED_DURING_ACK");
}
/**
 * @brief Hold排空与Begin结果共用原提交deadline。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * budget100µs；排空后才发Begin也不重新给100µs，超时停止并报RESULT_UNCONFIRMED。
 */
void begin_drain_and_reply_share_deadline() {
  Client client; Port port; port.stream_hold=true;
  ActionGate gate(client,port,profile(),100U);
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_session_id(9U);
  authorization.identity.set_configured_mask(1U);
  check(gate.enable(authorization,1U));
  gate.observe(reply(1U,1U),2U);
  check(gate.record_update(gate.generation(),1U,1U,Position{}));
  gate.observe(reply(1U,1U,true),3U);
  check(gate.submit(trajectory(),4U));
  gate.observe_hold_drain({0U,9U,1U,0U,1U,1U},102U);
  dp::iface::DummyJointStateV2 state;
  state.set_schema_major(2U); state.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  state.set_session_id(9U); state.set_state_seq(1U);
  state.set_motion_epoch(1U);
  state.set_configured_mask(1U); state.set_position_valid_mask(1U);
  state.set_core_state(wire_value(WireCoreState::kHold));
  state.set_hold_ready(true); state.set_committed_producer_seq(1U);
  state.set_can_submitted_mask(1U); state.set_can_sent_mask(1U);
  std::vector<std::uint8_t> bytes(dp::iface::DummyJointStateV2Codec::kEncodedSize);
  check(dp::iface::DummyJointStateV2Codec::encode(
    state,bytes.data(),bytes.size())==dp::iface::CodecResult::kOk);
  check(gate.observe_state(bytes.data(),bytes.size(),103U));
  check(client.requests.size()==2U &&
        client.requests.back().operation()==wire_value(ManageOperation::kBegin));
  gate.tick(104U);
  check(client.requests.size()==3U &&
        client.requests.back().operation()==wire_value(ManageOperation::kCancel) &&
        gate.error()=="RESULT_UNCONFIRMED");
}
/**
 * @brief 排空序号接受MAX→1回绕，拒绝旧/半区歧义提交。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 0x80000001相对当前1不能作为明确新提交，后续准确seq1才释放等待。
 */
void hold_drain_accepts_wrap_but_rejects_old_commit() {
  namespace wire=dp::iface;
  Client client; Port port; port.stream_hold=true;
  ActionGate gate(client,port,profile());
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_session_id(9U);
  authorization.identity.set_configured_mask(1U);
  check(gate.enable(authorization,1U));
  gate.observe(reply(1U,1U),2U);
  check(gate.record_update(gate.generation(),1U,UINT32_MAX-1U,Position{}));
  gate.observe(reply(1U,1U,true),3U);
  check(gate.record_update(gate.generation(),1U,UINT32_MAX,Position{}));
  check(gate.record_update(gate.generation(),1U,1U,Position{}));
  check(!gate.record_update(gate.generation(),1U,UINT32_MAX,Position{}));
  wire::DummyJointStateV2 state;
  state.set_schema_major(2U); state.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  state.set_session_id(9U); state.set_configured_mask(1U);
  state.set_position_valid_mask(1U); state.set_motion_epoch(1U);
  state.set_core_state(wire_value(WireCoreState::kHold)); state.set_hold_ready(true);
  state.set_can_submitted_mask(1U); state.set_can_sent_mask(1U);
  auto observe=[&](std::uint32_t state_seq,std::uint32_t committed,std::uint64_t now) {
    state.set_state_seq(state_seq); state.set_committed_producer_seq(committed);
    std::vector<std::uint8_t> bytes(wire::DummyJointStateV2Codec::kEncodedSize);
    check(wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())==wire::CodecResult::kOk);
    return gate.observe_state(bytes.data(),bytes.size(),now);
  };
  check(observe(UINT32_MAX-1U,UINT32_MAX,4U));
  check(gate.submit(trajectory(),5U));
  gate.observe_hold_drain({0U,9U,1U,0U,1U,UINT32_MAX},5U);
  check(observe(UINT32_MAX,0x80000001U,6U));
  check(client.requests.size()==1U);
  check(observe(1U,1U,7U));
  check(client.requests.size()==2U &&
        client.requests.back().operation()==wire_value(ManageOperation::kBegin));
}
/**
 * @brief 执行十个边界用例及Enable→Begin→Complete/Cancel故障矩阵。
 * @return 0表示所选断言通过；非零、assert或检查异常表示失败。
 * 最终producer_seq17冻结后晚到18不能前移终点；Complete只在ACK17后发送。Succeeded不会直接ready，必须新permit。
 * 覆盖Cancel发送/回执/超时失败、poll异常、旧generation回调及轨迹速度/加速度/时间/容差验证；轨迹验证不是碰撞检查。
 */

/** panel-interface-01原24点整路径：零时长首点不能先Begin才发现；shift20ms保留所有位置且不填v/a。 */
void positions_only_real_command_entry_before_begin() {
  namespace wire=dp::iface;
  const std::vector<std::vector<double>> positions{
    {-3.681404088114222e-05, -1.0471299231335476, 2.895613308401128, 7.117693326211072e-05, 0.0001227184630308513, 6.381360072118324e-05},
    {-3.681404088114222e-05, -1.0471299231335476, 2.895613308401128, 7.117693326211072e-05, 0.0001227184630308513, 6.381360072118324e-05},
    {-2.1935834302477296e-05, -1.0471295595038579, 2.8956131943970904, 7.109736128078043e-05, 0.00012254684158644367, 6.415792737528089e-05},
    {2.3627184153735917e-05, -1.0471284459243293, 2.8956128452711365, 7.085368005248312e-05, 0.00012202126809089619, 6.521239328907687e-05},
    {0.00010126761256782518, -1.0471265483592735, 2.895612250352505, 7.043844165075926e-05, 0.0001211256788007219, 6.700922738982574e-05},
    {0.00021237804902011795, -1.0471238327730015, 2.8956113989704324, 6.984419814914925e-05, 0.00011984400997243393, 6.958065860478206e-05},
    {0.00035835109159094243, -1.0471202651298246, 2.8956102804541564, 6.906350162119346e-05, 0.00011816019786254532, 7.295891586120038e-05},
    {0.0005405793383606256, -1.0471158113940542, 2.8956088841329164, 6.808890414043235e-05, 0.00011605817872756914, 7.717622808633527e-05},
    {0.0007604553874094952, -1.0471104375300013, 2.895607199335949, 6.691295778040631e-05, 0.00011352188882401848, 8.226482420744124e-05},
    {0.0010185265369199753, -1.047104130161466, 2.8956052218695905, 6.55327354678472e-05, 0.00011054501501837873, 8.823737035809775e-05},
    {0.0013058669463535243, -1.0470971074397832, 2.895603020127962, 6.399597459724109e-05, 0.00010723051768007864, 9.488729561919603e-05},
    {0.0016095555860625899, -1.047089685160407, 2.8956006931182605, 6.237177972930546e-05, 0.0001037274420424682, 0.00010191556832435108},
    {0.0019166712285407362, -1.047082179123627, 2.895598339849202, 6.072925648295148e-05, 0.00010018483562121734, 0.00010902315222812551},
    {0.0022142926462815297, -1.0470749051297334, 2.8955960593294994, 5.9137510477090306e-05, 9.675174593199598e-05, 0.00011591101108508204},
    {0.002489498611778533, -1.047068178979016, 2.895593950567867, 5.7665647330633064e-05, 9.357722049047405e-05, 0.0001222801086497832},
    {0.0027293678975253105, -1.0470623164717638, 2.89559211257302, 5.638277266249094e-05, 9.081030681232151e-05, 0.00012783140867679172},
    {0.002931501494425341, -1.0470573762407795, 2.8955905637273296, 5.530171691066264e-05, 8.847867768216565e-05, 0.00013250939084711024},
    {0.003102623379205835, -1.0470531939492145, 2.8955892525084233, 5.438651874151738e-05, 8.650477142669687e-05, 0.0001364696682984157},
    {0.0032423021664489396, -1.0470497801403122, 2.895588182221786, 5.363948530089525e-05, 8.489356411531688e-05, 0.00013970225746872626},
    {0.0033501018188875544, -1.0470471454710077, 2.895587356208547, 5.3062948613767996e-05, 8.365008547692475e-05, 0.00014219706713821322},
    {0.0034255862992545825, -1.047045300598238, 2.895586777809835, 5.265924070510745e-05, 8.277936524041956e-05, 0.00014394400608704793},
    {0.0034683195702829247, -1.047044256178939, 2.8955864503667788, 5.2430693599885376e-05, 8.228643313470035e-05, 0.00014493298309540168},
    {0.003478635691971859, -1.0470440040485423, 2.8955863713196495, 5.237552067040004e-05, 8.216743574777883e-05, 0.0001451717293210058},
    {0.003478635691971859, -1.0470440040485423, 2.8955863713196495, 5.237552067040004e-05, 8.216743574777883e-05, 0.0001451717293210058},
  };
  JointProfile limits;
  limits.joints={"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"};
  limits.lower.assign(6,-4.); limits.upper.assign(6,4.);
  limits.velocity.assign(6,0.03490658503988659); limits.acceleration.assign(6,0.1);
  limits.path_tolerance.assign(6,0.001); limits.goal_tolerance.assign(6,0.001);
  Position actual{-3.681404088114222e-05,-1.0471225528352308,2.895613308401128,7.117693326211072e-05,0.00013008162230741875,6.381360072118324e-05};
  for (unsigned scenario=0;scenario<7;++scenario) {
    Client client; Port port; port.stream_hold=scenario==6; ActionGate gate(client,port,limits);
    Authorization auth; auth.session=auth.reference=auth.permit=true;
    auth.identity.set_expected_session_id(9); auth.identity.set_configured_mask(63);
    check(gate.enable(auth,1));
    auto ready_reply=reply(1,1); ready_reply.position=ready_reply.hold_position=actual;
    gate.observe(ready_reply,2);
    check(gate.record_update(gate.generation(),1,1,actual));
    ready_reply.result_valid=ready_reply.succeeded=ready_reply.hold_ready=true;
    gate.observe(ready_reply,3); check(gate.ready());
    wire::DummyJointStateV2 state;
    state.set_schema_major(2); state.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
    state.set_session_id(9); state.set_configured_mask(63); state.set_position_valid_mask(63);
    state.set_motion_epoch(1); state.set_core_state(wire_value(WireCoreState::kHold)); state.set_hold_ready(true);
    state.set_can_submitted_mask(63); state.set_can_sent_mask(63); state.set_committed_producer_seq(1);
    state.set_state_seq(1); double actual_wire[6]; std::copy(actual.begin(),actual.end(),actual_wire); state.set_position_rad(actual_wire);
    std::vector<std::uint8_t> bytes(wire::DummyJointStateV2Codec::kEncodedSize);
    check(wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())==wire::CodecResult::kOk);
    if(scenario!=4) check(gate.observe_state(bytes.data(),bytes.size(),4));
    Trajectory motion; motion.joints=limits.joints;
    for(std::size_t i=0;i<positions.size();++i)
      motion.points.push_back({positions[i],{},{},0.02*(i+(scenario==0?0:1))});
    if(scenario==2) for(auto &point:motion.points) point.positions[0]+=0.01;
    if(scenario==3) { motion.points.resize(1); motion.points[0].positions.assign(actual.begin(),actual.end()); motion.points[0].positions[0]+=0.0002; }
    const auto submitted=gate.submit(motion,scenario==5?250005:10);
    if(scenario==6) {
      check(submitted && client.requests.size()==1);
      gate.observe_hold_drain({0,9,1,0,1,1},11);
      actual_wire[0]+=0.01; state.set_position_rad(actual_wire); state.set_state_seq(2);
      check(wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())==wire::CodecResult::kOk);
      check(gate.observe_state(bytes.data(),bytes.size(),12));
      check(client.requests.size()==1 && port.cancels==0 && port.starts==0);
      check(gate.terminal()==Terminal::kAborted && gate.error()=="TRAJECTORY_LIMIT_VIOLATION");
    } else if(scenario!=1) {
      check(!submitted && client.requests.size()==1 && port.cancels==0 && port.starts==0);
      check(gate.error()=="TRAJECTORY_LIMIT_VIOLATION" || gate.error()=="FRESH_HOLD_START_REQUIRED");
    } else {
      check(submitted && client.requests.size()==2 && client.requests.back().operation()==wire_value(ManageOperation::kBegin));
      auto begin=reply(2,2); begin.position=actual; gate.observe(begin,11);
      check(port.primes==2 && port.starts==0);
      check(gate.record_update(gate.generation(),2,1,actual));
      begin.result_valid=begin.succeeded=true; begin.hold_ready=true; gate.observe(begin,12);
      check(port.starts==1);
    }
  }
}
// GUI41真实J3工作边界外起点必须在真正Running阶段保持向内资格。
void running_manual_inward_boundary_samples(bool manual=true,bool reproduce=false,bool projected=false) {
 namespace wire=dp::iface;
 constexpr double rad=3.14159265358979323846/180.;
 JointProfile limits;limits.joints={"Joint1","Joint2","Joint3","Joint4","Joint5","Joint6"};
 limits.lower.assign(6,-10.);limits.upper.assign(6,10.);
 limits.velocity.assign(6,1.1);limits.acceleration.assign(6,4.);
 limits.path_tolerance.assign(6,0.);limits.goal_tolerance.assign(6,.003);
 limits.nominal_lower.assign(6,-11.);limits.nominal_upper.assign(6,11.);limits.manual_inward_start=manual;
 limits.lower[1]=-72.45*rad;limits.nominal_lower[1]=-73.*rad;
 limits.upper[2]=179.45*rad;limits.nominal_upper[2]=180.*rad;
 Position actual{};actual[1]=(manual?-72.46:-72.44)*rad;actual[2]=(manual?179.45353124:179.44)*rad;
 if(reproduce){actual[1]=-72.452968729*rad;actual[2]=179.451281297*rad;}
 Client client;Port port;ActionGate gate(client,port,limits);
 Authorization auth;auth.session=auth.reference=auth.permit=true;
 auth.identity.set_expected_session_id(9);auth.identity.set_configured_mask(63);
 check(gate.enable(auth,1));auto ready=reply(1,1);ready.position=ready.hold_position=actual;
 gate.observe(ready,2);check(gate.record_update(gate.generation(),1,1,actual));
 ready.result_valid=ready.succeeded=ready.hold_ready=true;gate.observe(ready,3);check(gate.ready());
 wire::DummyJointStateV2 state;state.set_schema_major(2);state.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
 state.set_session_id(9);state.set_configured_mask(63);state.set_position_valid_mask(63);state.set_motion_epoch(1);
 state.set_core_state(wire_value(WireCoreState::kHold));state.set_hold_ready(true);state.set_can_submitted_mask(63);
 state.set_can_sent_mask(63);state.set_committed_producer_seq(1);state.set_state_seq(1);double actual_wire[6];std::copy(actual.begin(),actual.end(),actual_wire);state.set_position_rad(actual_wire);
 std::vector<std::uint8_t> bytes(wire::DummyJointStateV2Codec::kEncodedSize);
 check(wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())==wire::CodecResult::kOk);
 check(gate.observe_state(bytes.data(),bytes.size(),4));
 Position home{};home[2]=90.*rad;Trajectory motion;motion.joints=limits.joints;
 motion.points={{{actual.begin(),actual.end()},{},{},1.},{{home.begin(),home.end()},{},{},4.},{{home.begin(),home.end()},{},{},7.}};
 if(projected) {
   motion.points.front().positions[1]=limits.lower[1];
   motion.points.front().positions[2]=limits.upper[2];
 }
 auto wrong_profile=limits;wrong_profile.velocity[0]*=.5;
 check(!gate.submit_prepared(prepare_trajectory(motion,wrong_profile),5));
 const auto prepared=prepare_trajectory(motion,limits);check(bool(prepared));
 check(gate.submit_prepared(prepared,5));auto begin=reply(2,2);begin.position=begin.hold_position=actual;
 gate.observe(begin,6);check(gate.record_update(gate.generation(),2,1,actual));
 begin.result_valid=begin.succeeded=begin.hold_ready=true;gate.observe(begin,7);check(gate.phase()==Phase::kRunning);
 const auto generation=gate.generation();
 if(projected) {
   auto tail=motion;tail.points.back().time=8.;
   auto candidate=prepare_trajectory(tail,limits);check(bool(candidate));
   auto update=prepare_trajectory_tail(prepared,candidate,4000000000ULL);check(bool(update));
   check(!gate.extend_prepared(update,3900000000ULL));
   check(gate.extend_prepared(update,1000000000ULL));
   check(!gate.extend_prepared(update,1000000000ULL));
   auto second=tail;second.points.back().time=9.;
   auto next=prepare_trajectory_tail(candidate,prepare_trajectory(second,limits),4000000000ULL);check(bool(next));
   check(gate.extend_prepared(next,1100000000ULL));
   auto changed=second;changed.points[1].positions[0]+=.001;
   check(!prepare_trajectory_tail(prepare_trajectory(second,limits),prepare_trajectory(changed,limits),4000000000ULL));

   auto command=actual;command[1]=limits.lower[1];command[2]=limits.upper[2];
   check(gate.record_update(generation,2,2,command));
   auto inward=command;inward[1]+=.00001;inward[2]-=.00001;
   check(!gate.record_update(generation+1,2,3,inward));
   check(gate.record_update(generation,2,3,inward));
   auto outward=actual;check(!gate.record_update(generation,2,4,outward));
   return;
 }
 if(reproduce) {
   auto submitted_first=actual;submitted_first[1]=-72.451140642*rad;submitted_first[2]=179.452265668*rad;
   check(!gate.record_update(generation,2,2,submitted_first));
   const auto& rejected=gate.first_update_rejection();
   check(rejected.reason==UpdateRejectReason::kInward && rejected.axis==2 && rejected.phase==Phase::kRunning);
   check(rejected.candidate==submitted_first && rejected.prime==actual && rejected.previous==actual);
   auto another=submitted_first;another[2]+=1e-5;
   check(!gate.record_update(generation+1,2,2,another));
   check(gate.first_update_rejection().candidate==submitted_first);
   return;
 }
 check(gate.record_update(generation,2,2,actual));
 if(!manual) {auto outside=actual;outside[2]=179.45353124*rad;check(!gate.record_update(generation,2,3,outside));return;}
 auto inward=actual;inward[1]+=.00001;inward[2]-=.00001;
 check(!gate.record_update(generation+1,2,3,inward));check(!gate.record_update(generation,3,3,inward));
 auto outward=actual;outward[2]+=.000001;check(!gate.record_update(generation,2,3,outward));
 check(gate.record_update(generation,2,3,inward));
 auto entered=actual;entered[1]=limits.lower[1]+.001;entered[2]=limits.upper[2]-.001;
 check(gate.record_update(generation,2,4,entered));
 check(!gate.record_update(generation,2,5,inward));
}
// 大轨迹的全路径证明不能遗漏中间点，且原输入后续修改不能污染准备结果。
void immutable_prepared_trajectory_checks() {
 auto p=profile();Trajectory path;path.joints=p.joints;
 for(unsigned i=0;i<9132;++i)path.points.push_back({{0.},{},{},.04+.02*i});
 const auto prepared=prepare_trajectory(path,p);check(bool(prepared));
 Position actual{};check(validate_prepared_start(*prepared,actual));
 path.points[4500].positions[0]=2.;check(!prepare_trajectory(path,p));
 check(prepared->trajectory().points[4500].positions[0]==0.);
 actual[0]=.1;check(!validate_prepared_start(*prepared,actual));
}
// 一母RUN两次续接；最后只发一次Complete，取消后任何迟到续接均拒绝。
void rolling_tail_lifecycle() {
 for(bool cancel:{false,true}) {
  Client client;Port port;auto limits=profile();ActionGate gate(client,port,limits);
  Authorization auth;auth.session=auth.reference=auth.permit=true;auth.identity.set_expected_session_id(9);
  check(gate.enable(auth,1));gate.observe(reply(1,1),2);check(gate.record_update(gate.generation(),1,1,Position{}));
  gate.observe(reply(1,1,true),3);check(gate.ready());
  Trajectory path;path.joints=limits.joints;path.points={{{0.},{0.},{},.1},{{.2},{0.},{},1.}};
  auto base=prepare_trajectory(path,limits);check(bool(base));check(gate.submit_prepared(base,4));
  gate.observe(reply(2,2),5);check(gate.record_update(gate.generation(),2,1,Position{}));
  gate.observe(reply(2,2,true),6);check(gate.phase()==Phase::kRunning);
  std::shared_ptr<const PreparedTrajectoryTail> tail;
  for(unsigned version=0;version<2;++version) {
   const auto through=static_cast<std::uint64_t>((version+1)*1000000000ULL);
   path.points.push_back({{.3+.1*version},{0.},{},2.+version});auto candidate=prepare_trajectory(path,limits);
   tail=prepare_trajectory_tail(base,candidate,through);check(bool(tail));
   check(!gate.extend_prepared(tail,through-399999999ULL));
   check(gate.extend_prepared(tail,200000000ULL+version*100000000ULL));
   check(!gate.extend_prepared(tail,200000000ULL));base=std::move(candidate);
  }
  check(port.starts==1 && client.requests.size()==2);
  if(cancel) {gate.cancel(7);check(!gate.extend_prepared(tail,300000000ULL));check(port.cancels==1);continue;}
  Position end{};end[0]=.4;check(gate.record_update(gate.generation(),2,2,end));
  gate.inner_result(gate.generation(),Terminal::kSucceeded,"",8);auto accepted=reply(2,2,true);accepted.accepted_seq=2;
  gate.observe(accepted,9);check(client.requests.size()==3 && client.requests.back().operation()==wire_value(ManageOperation::kComplete));
  check(client.requests.back().terminal_position_rad()[0]==.4);
  gate.observe(reply(3,3),10);check(gate.record_update(gate.generation(),3,1,Position{}));
  gate.observe(reply(3,3,true),11);check(gate.terminal()==Terminal::kSucceeded);
 }
}
int main() {
  rolling_tail_lifecycle();
  immutable_prepared_trajectory_checks();
  running_manual_inward_boundary_samples(true,true);
  running_manual_inward_boundary_samples(true,true,true);
  running_manual_inward_boundary_samples();
  running_manual_inward_boundary_samples(false);
  positions_only_real_command_entry_before_begin();
  cancel_receipt_rejects_foreign_owner();
  discarded_local_hold_generation_does_not_block_begin();
  begin_waits_for_final_hold_commit();
  pending_hold_drain_can_cancel_or_timeout();
  typed_ack_reject_survives_session_invalidation();
  hold_drain_accepts_wrap_but_rejects_old_commit();
  begin_drain_and_reply_share_deadline();
  hold_prime_outside_planning_range();
  repeated_authorization_loss_is_idempotent();
  enable_preserves_bootstrap_motion_epoch();
  Client client; Port port; ActionGate gate(client,port,profile());
  Authorization authorization; authorization.session=true; authorization.reference=true;
  authorization.permit=true; authorization.identity.set_expected_session_id(9);
  check(!gate.submit(trajectory(),1));
  check(gate.enable(authorization,1));
  ManageReply queued; queued.request_id=1; queued.queued=true;
  client.replies.push_back(queued); gate.tick(2);
  const bool queued_started_prime=port.primes!=0; check(!queued_started_prime);
  client.replies.push_back(reply(1,1)); gate.tick(3); check(port.primes==1);
  check(gate.record_update(gate.generation(),1,1,Position{}));
  check(gate.record_update(gate.generation(),1,2,Position{}));
  client.replies.push_back(reply(1,1,true)); gate.tick(4); check(gate.ready());
  check(gate.submit(trajectory(),5)); const bool busy_rejected=!gate.submit(trajectory(),5); check(busy_rejected);
  client.replies.push_back(reply(2,2)); gate.tick(6);
  const bool prime_before_result=port.starts==0 && port.primes==2; check(prime_before_result);
  const auto generation=gate.generation();
  const bool late_generation_accepted=gate.record_update(generation-1,2,1,Position{});
  check(!late_generation_accepted);
  check(gate.record_update(generation,2,1,Position{}));
  Position premature_motion{}; premature_motion[0]=0.1;
  check(!gate.record_update(generation,2,2,premature_motion));
  client.replies.push_back(reply(2,2,true)); gate.tick(7); check(port.starts==1);
  Position terminal{}; terminal.fill(std::numeric_limits<double>::quiet_NaN()); terminal[0]=0.2;
  check(gate.record_update(generation,2,17,terminal));
  gate.inner_result(generation,Terminal::kSucceeded,"",8);
  check(gate.terminal()==Terminal::kNone && client.requests.size()==2);
  check(!gate.record_update(generation,2,18,terminal));
  auto accepted=reply(2,2,true); accepted.accepted_seq=17;
  gate.observe(accepted,9);
  const auto complete_seq=client.requests.back().terminal_producer_seq(); check(complete_seq==17);
  check(client.requests.back().terminal_position_rad()[0]==0.2);
  for(std::size_t axis=1;axis<6;++axis) check(client.requests.back().terminal_position_rad()[axis]==0.0);
  client.replies.push_back(reply(3,3)); gate.tick(10); check(gate.terminal()==Terminal::kNone);
  check(gate.record_update(gate.generation(),3,1,Position{}));
  client.replies.push_back(reply(3,3,true)); gate.tick(11);
  const bool completion_barrier=gate.terminal()==Terminal::kSucceeded && !gate.ready() &&
      gate.permit_refresh_allowed();
  check(completion_barrier);
  check(gate.renew_permit(authorization) && gate.ready());
  check(gate.submit(trajectory(),12)); client.replies.push_back(reply(4,4)); gate.tick(13);
  gate.cancel(14);
  const bool cancel_pending=port.cancels==1 && !gate.submit(trajectory(),14) && gate.terminal()==Terminal::kNone;
  check(cancel_pending);
  check(!gate.confirmed_cancel_receipt());
  gate.inner_result(generation,Terminal::kSucceeded,"",15); check(gate.terminal()==Terminal::kNone);
  client.replies.push_back(reply(5,5)); gate.tick(16);
  check(gate.record_update(gate.generation(),5,1,Position{}));
  auto committed_cancel=reply(5,5,true);
  committed_cancel.transaction_phase=wire_value(WireTransactionPhase::kCommitted);
  client.replies.push_back(committed_cancel); gate.tick(17);
  const bool cancel_barrier=cancel_pending && gate.terminal()==Terminal::kCanceled && !gate.ready() &&
      gate.permit_refresh_allowed();
  check(cancel_barrier);
  const auto cancel_receipt=gate.confirmed_cancel_receipt();
  check(cancel_receipt && cancel_receipt->request_id==5U && cancel_receipt->operation_id==105U);
  check(gate.renew_permit(authorization) && gate.ready());
  check(gate.submit(trajectory(),18));
  check(!gate.confirmed_cancel_receipt());
  client.replies.push_back(reply(6,6)); gate.tick(19);
  check(gate.record_update(gate.generation(),6,1,Position{}));
  client.replies.push_back(reply(6,6,true)); gate.tick(20);
  const auto aborted_generation=gate.generation();
  gate.inner_result(aborted_generation,Terminal::kAborted,"PATH_TOLERANCE",21);
  check(gate.terminal()==Terminal::kNone && port.cancels==2);
  gate.inner_result(aborted_generation,Terminal::kSucceeded,"",22);
  client.replies.push_back(reply(7,7)); gate.tick(23);
  check(gate.record_update(gate.generation(),7,1,Position{}));
  client.replies.push_back(reply(7,7,true)); gate.tick(24);
  const bool abort_preserved=gate.terminal()==Terminal::kAborted && gate.error()=="PATH_TOLERANCE";
  check(abort_preserved);
  check(!gate.confirmed_cancel_receipt());
  check(gate.renew_permit(authorization) && gate.ready());
  check(gate.submit(trajectory(),25)); gate.cancel(26); gate.tick(2000000);
  const bool stop_timeout_aborted=gate.terminal()==Terminal::kAborted && gate.error()=="STOP_FAILED";
  check(stop_timeout_aborted);
  Client unavailable; unavailable.reject_cancel=true; Port failed_port;
  ActionGate failed_gate(unavailable,failed_port,profile());
  check(failed_gate.enable(authorization,1)); failed_gate.abort("PATH_TOLERANCE",2);
  check(failed_gate.terminal()==Terminal::kAborted && failed_gate.error()=="STOP_FAILED" && !failed_gate.ready());
  Client polling; Port polling_port; ActionGate polling_gate(polling,polling_port,profile());
  check(polling_gate.enable(authorization,1)); polling.replies.push_back(reply(1,1)); polling_gate.tick(2);
  check(polling_gate.record_update(polling_gate.generation(),1,1,Position{}));
  polling.replies.push_back(reply(1,1,true)); polling_gate.tick(3); check(polling_gate.ready());
  check(polling_gate.submit(trajectory(),4)); polling.replies.push_back(reply(2,2)); polling_gate.tick(5);
  check(polling_gate.record_update(polling_gate.generation(),2,1,Position{}));
  polling.replies.push_back(reply(2,2,true)); polling_gate.tick(6); check(polling_port.starts==1);
  polling.throw_on_poll=true; polling_gate.tick(7);
  const bool running_poll_cancel_barrier=polling_port.cancels==1 && polling_gate.terminal()==Terminal::kNone &&
      polling.requests.back().operation()==wire_value(ManageOperation::kCancel) && !polling_gate.submit(trajectory(),7);
  check(running_poll_cancel_barrier);
  polling.throw_on_poll=false; polling.replies.push_back(reply(3,3)); polling_gate.tick(8);
  check(polling_gate.record_update(polling_gate.generation(),3,1,Position{}));
  polling.replies.push_back(reply(3,3,true)); polling_gate.tick(9);
  check(polling_gate.terminal()==Terminal::kAborted && polling_gate.error()=="RESULT_UNCONFIRMED");
  check(polling_gate.renew_permit(authorization) && polling_gate.ready());
  check(polling_gate.submit(trajectory(),10)); polling.replies.push_back(reply(4,4)); polling_gate.tick(11);
  check(polling_gate.record_update(polling_gate.generation(),4,1,Position{}));
  polling.replies.push_back(reply(4,4,true)); polling_gate.tick(12);
  Position completed{}; completed[0]=0.2;
  check(polling_gate.record_update(polling_gate.generation(),4,2,completed));
  polling_gate.inner_result(polling_gate.generation(),Terminal::kSucceeded,"",13);
  polling.throw_on_poll=true; polling_gate.tick(14);
  const bool completing_poll_cancel_barrier=polling_port.cancels==2 && polling_gate.terminal()==Terminal::kNone &&
      polling.requests.back().operation()==wire_value(ManageOperation::kCancel);
  check(completing_poll_cancel_barrier);
  Client enable_poll; Port enable_port; ActionGate enable_gate(enable_poll,enable_port,profile());
  check(enable_gate.enable(authorization,1)); enable_poll.throw_on_poll=true; enable_gate.tick(2);
  const bool enable_poll_fault=enable_gate.terminal()==Terminal::kAborted &&
      enable_gate.error()=="ENABLE_FAILED" && enable_port.cancels==1;
  check(enable_poll_fault);
  Client stopping; Port stopping_port; ActionGate stopping_gate(stopping,stopping_port,profile());
  check(stopping_gate.enable(authorization,1)); stopping_gate.abort("PATH_TOLERANCE",2);
  stopping.throw_on_poll=true; stopping_gate.tick(3);
  const bool stopping_poll_fault=stopping_gate.terminal()==Terminal::kAborted &&
      stopping_gate.error()=="STOP_FAILED" && stopping_port.cancels>=2 && !stopping_gate.ready();
  check(stopping_poll_fault);
  Trajectory fast_segment{{"Joint1"},{{{0.2},{},{},1.0},{{0.5},{},{},1.1}},{},{},0.0};
  check(!validate_trajectory(fast_segment,profile()));
  JointProfile acceleration_profile=profile(); acceleration_profile.velocity={3}; acceleration_profile.acceleration={1};
  Trajectory acceleration_jump{{"Joint1"},{{{0.0},{},{},1.0},{{0.2},{},{},1.5},{{1.2},{},{},2.0}},{},{},0.0};
  check(!validate_trajectory(acceleration_jump,acceleration_profile));
  Client deferred; Port deferred_port; ActionGate deferred_gate(deferred,deferred_port,profile());
  check(deferred_gate.enable(authorization,1)); deferred.replies.push_back(reply(1,1)); deferred_gate.tick(2);
  check(deferred_gate.record_update(deferred_gate.generation(),1,1,Position{}));
  deferred.replies.push_back(reply(1,1,true)); deferred_gate.tick(3); check(deferred_gate.ready());
  auto fast_first=trajectory(); fast_first.points.front().velocities.clear(); fast_first.points.front().time=0.1;
  const bool deferred_start_rejected=!deferred_gate.submit(fast_first,4) && deferred_port.starts==0 && deferred_port.cancels==0 && deferred.requests.size()==1;
  check(deferred_start_rejected);
  auto invalid=trajectory(); invalid.path_tolerance={-1}; check(!validate_trajectory(invalid,profile()));
  invalid=trajectory(); invalid.points[0].time=std::numeric_limits<double>::quiet_NaN();
  check(!validate_trajectory(invalid,profile()));
  invalid=trajectory(); invalid.joints={}; check(!validate_trajectory(invalid,profile()));
  std::cout << std::boolalpha << "{\"busy_rejected\":" << busy_rejected
    << ",\"completion_barrier\":" << completion_barrier
    << ",\"abort_error_preserved\":" << abort_preserved
    << ",\"queued_started_prime\":" << queued_started_prime
    << ",\"prime_before_result\":" << prime_before_result
    << ",\"complete_seq\":" << complete_seq
    << ",\"cancel_barrier\":" << cancel_barrier
    << ",\"stop_timeout_aborted\":" << stop_timeout_aborted
    << ",\"running_poll_cancel_barrier\":" << running_poll_cancel_barrier
    << ",\"completing_poll_cancel_barrier\":" << completing_poll_cancel_barrier
    << ",\"enable_poll_fault\":" << enable_poll_fault
    << ",\"stopping_poll_fault\":" << stopping_poll_fault
    << ",\"deferred_start_rejected\":" << deferred_start_rejected
    << ",\"late_generation_accepted\":" << late_generation_accepted << "}\n";
}
