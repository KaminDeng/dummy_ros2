/**
 * @file queued_renew_completion_tests.cpp
 * @brief 核续租与完成/取消冻结相遇时的严格身份、队列预约及原期限。
 *
 * 宏重命名包含wire_management_tests.cpp只为复用Transport/Port类型；DUMMY_ACTION_GATE_TEST再包含生产device_action_gate.cpp，排除其main但不自动创建DeviceActionGate节点。
 * 实际实例是内存WireManagementClient与ActionGate，加生产纯queue/lease helper；不能称为完整ROS Node对象、远端设备或实机执行覆盖。
 * 位置rad，轨迹time秒；gate/tick参数为人工Hostµs，续租时长30000000µs。assert须启用；模式仅由main字符串选择。
 */
#define main shared_wire_fixture_main
#include "wire_management_tests.cpp"
#undef main
#define DUMMY_ACTION_GATE_TEST
#include "../src/device_action_gate.cpp"
#include <cassert>

/**
 * @brief 派生自复用Port计数夹具，增加executing布尔模拟producer状态；不是实际JTC线程。
 */
struct CompletionPort : Port {
  /**
   * @brief start置true、disable置false；冻结资格读取此字段，不能据true声称电机在Run。
   */
  bool executing=false;
  /**
   * @brief 沿用Port启动次数统计，再标executing。
   * @param t 轨迹夹具，仅传给无执行副作用的基类。
   * @param g 目标generation。
   * @return 无。
   */
  void start(const Trajectory &t,std::uint64_t g) override {Port::start(t,g);executing=true;}
  /**
   * @brief 停本地executing标志。
   * @return 无；不发送硬件Disable或等待回执。
   */
  void disable() override {executing=false;}
};
/**
 * @brief 冻结末目标且正式State ACK后，核Complete意图与未发续租撤回。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 末seq17 ACK经当前State codec/公共observe_state输入；inner_result后冻结producer但尚未发送Complete，queue仍只有op20预约。
 * 正例允许bounded completion；sent_complete已发operation8仍须保持相同有界意图，原始目标/身份/末ACK不可丢。
 */
int main(int argc,char **argv) {
  const std::string mode=argc>1?argv[1]:"positive";
  Transport transport;WireManagementClient client(transport);CompletionPort port;
  ActionGate gate(client,port,{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}},2000000);
  Authorization auth;auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_boot_epoch(7);auth.identity.set_expected_session_id(9);
  auth.identity.set_expected_reference_epoch(1);auth.identity.set_expected_profile_id(201);
  auth.identity.set_expected_management_epoch(1);auth.identity.set_configured_mask(1);
  /**
   * @par 内部commit回调
   * epoch为合成新motion代次；从Transport最后实际请求取request_id，构造固定boot7/session9/ref1/management1/profile201的Committed响应并poll原请求。
   * 返回client解析结果；此处不模拟设备真实操作，只保证测试确经当前wire codec/请求关联路径。
   */
  auto commit=[&](std::uint32_t epoch) {
    const auto request=transport.requests.back();
    wire::DummyManageV2Response response;response.set_schema_major(2);response.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
    response.set_request_id(request.request_id());response.set_boot_epoch(7);response.set_session_id(9);
    response.set_reference_epoch(1);response.set_motion_epoch(epoch);response.set_management_epoch(1);
    response.set_profile_id(201);response.set_receipt(1);response.set_result_code(0);
    response.set_operation_id(request.request_id()+100);response.set_transaction_phase(3);
    transport.respond(response);return client.poll(request.request_id());
  };
  /**
   * @brief 通过Enable首样本及Begin首样本双屏障到J1 Running，保留当前完整身份供随后续租/完成/取消匹配。
   * prime只是Port计数，accepted_seq1为人工回执，不能把它当六轴CAN组实测。
   */
  assert(gate.enable(auth,1));auto reply=commit(1);assert(reply);
  reply->snapshot_valid=true;reply->hold_available=true;reply->hold_ready=false;gate.observe(*reply,2);
  assert(gate.record_update(gate.generation(),1,1,Position{}));reply->hold_ready=true;reply->accepted_seq=1;gate.observe(*reply,3);
  assert(gate.submit({{"Joint1"},{{{0.2},{0},{},1}},{},{},0},10));reply=commit(2);assert(reply);
  reply->snapshot_valid=true;reply->hold_available=true;reply->hold_ready=false;gate.observe(*reply,11);
  assert(gate.record_update(gate.generation(),2,1,Position{}));reply->hold_ready=true;reply->accepted_seq=1;gate.observe(*reply,12);
  assert(gate.phase()==Phase::kRunning);
  auto live=auth.identity;live.set_expected_motion_epoch(2);
  std::atomic<int> queued{0},explicit_status{0};std::atomic<bool> busy{true},reserved{false},started{false};
  // 对应真实Node读观测末尾排队；本轮尚未调用Bootstrap start_renew_lease。
  assert(queue_lease_renewal(queued,busy,reserved,true));assert(queued==20&&busy&&reserved&&!started);
  const auto wire_count_before_freeze=transport.requests.size();
  assert(client.matching_inflight_renewal(live)==0U);
  Position end{};end[0]=0.2;assert(gate.record_update(gate.generation(),2,17,end));
  // ACK来自正式State codec/公共observe_state，不从私有Gate字段伪造。
  wire::DummyJointStateV2 accepted;accepted.set_schema_major(2);accepted.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  accepted.set_boot_epoch(7);accepted.set_session_id(9);accepted.set_reference_epoch(1);
  accepted.set_motion_epoch(2);accepted.set_management_epoch(1);accepted.set_profile_id(201);
  accepted.set_configured_mask(1);accepted.set_position_valid_mask(1);accepted.set_state_seq(1);
  accepted.set_core_state(5);accepted.set_committed_producer_seq(17);accepted.set_freshness_ticket(55);
  double measured[6]{};measured[0]=.2;accepted.set_position_rad(measured);
  std::vector<std::uint8_t> state_bytes(wire::DummyJointStateV2Codec::kEncodedSize);
  assert(wire::DummyJointStateV2Codec::encode(accepted,state_bytes.data(),state_bytes.size())==wire::CodecResult::kOk);
  assert(gate.observe_state(state_bytes.data(),state_bytes.size(),22));
  if(mode!="not_frozen")gate.inner_result(gate.generation(),Terminal::kSucceeded,"",23);
  if(mode=="sent_complete") {reply->accepted_seq=17;gate.observe(*reply,24);assert(transport.requests.back().operation()==8);}
  /**
   * @brief 复现旧资格合取在冻结producer时的冲突输入，但断言修正后的continuation不再original_fault。
   * 名字含red不表示该分支必须失败，也不实例化完整Node对象。
   */
  if(mode=="original_policy_red") {
    const RunLeaseEligibility observed{true,true,true,true,port.executing && gate.phase()==Phase::kRunning,true,true};
    const bool deferred=gate.completion_waiting_for_renewal(25);
    const bool original_fault=reserved && !run_lease_eligible(run_lease_continuation_inputs(observed,true,deferred));
    std::cerr << "ORIGINAL_POLICY_COUNTEREXAMPLE scope=REAL_WIRE_GATE_PLUS_NODE_POLICY_NOT_NODE_OBJECT"
              << " frozen_phase="<<unsigned(gate.phase())<<" port_executing="<<port.executing
              <<" queued="<<queued<<" reserved="<<reserved<<" started="<<started<<" busy="<<busy
              <<" actual_inflight_renew="<<client.matching_inflight_renewal(live)
              <<" original_fault="<<original_fault<<'\n';
    assert(!original_fault);
  }
  /**
   * @brief wrong_boot/session/reference/motion/management/profile/mask/device/client、missing_ack/old_ack、expired、not_frozen均要求bounded_completion_intent=false。
   * Status稀疏不能替代当前State末ACK，seq16与17需精确区别。
   */
  if(mode=="wrong_boot")live.set_expected_boot_epoch(8);
  if(mode=="wrong_session")live.set_expected_session_id(10);
  if(mode=="wrong_reference")live.set_expected_reference_epoch(2);
  if(mode=="wrong_motion")live.set_expected_motion_epoch(3);
  if(mode=="wrong_management")live.set_expected_management_epoch(2);
  if(mode=="wrong_profile")live.set_expected_profile_id(202);
  if(mode=="wrong_mask")live.set_configured_mask(63);
  if(mode=="wrong_device") {std::uint8_t v[16]{};v[15]=1;live.set_device_id(v);}
  if(mode=="wrong_client") {std::uint8_t v[16]{};v[15]=1;live.set_client_instance_id(v);}
  const auto ack=mode=="missing_ack"?0U:mode=="old_ack"?16U:17U;
  const auto now=mode=="expired"?2000023U:25U;
  const bool intent=gate.bounded_completion_intent(live,ack,now);
  /**
   * @brief withdraw_positive/explicit/status_lag可撤未发预约；owns_busy、started、nonrenew、not_reserved、not_running、actual_inflight均不得撤且保原槽状态。
   * stale_state/wrong_run在本源仅显式把bounded设false，未通过真实State/Status回调制造对应负例；不能扩大为隔离了在线新鲜性/场景run检查。
   */
  if(mode.rfind("withdraw_",0U)==0U) {
    std::atomic<bool> running{true};
    BootstrapCoordinator bootstrap(client,auth.identity,{},0);
    bool owns_busy=mode=="withdraw_owns_busy";
    bool actual_inflight=false;
    if(mode=="withdraw_started")started=true;
    if(mode=="withdraw_nonrenew")queued=10;
    if(mode=="withdraw_not_reserved")reserved=false;
    if(mode=="withdraw_not_running")running=false;
    if(mode=="withdraw_explicit")explicit_status=1;
    /**
     * @brief 通过同client的Bootstrap CaptureReference准备完成后真实submit Renew，核renewal_request_inflight仍true。
     * 该inflight不得按“仅预约”撤回；fixture响应仍为合成wire，不是远端续租完成。
     */
    if(mode=="withdraw_actual_inflight") {
      BootstrapSnapshot snapshot{7,9,1,2,1,55,201,1,true,true};
      snapshot.core_state=snapshot.state_core_state=wire_value(WireCoreState::kRunning);snapshot.run_id=5;
      assert(bootstrap.start_operation(snapshot,ManageOperation::kCaptureReference,24));
      const auto setup=transport.requests.back();
      wire::DummyManageV2Response response;response.set_schema_major(2);response.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
      response.set_request_id(setup.request_id());response.set_boot_epoch(7);response.set_session_id(9);
      response.set_reference_epoch(1);response.set_motion_epoch(2);response.set_management_epoch(1);
      response.set_profile_id(201);response.set_receipt(1);response.set_transaction_phase(3);
      response.set_result_code(0);response.set_operation_id(100+setup.request_id());
      transport.respond(response);bootstrap.tick(24);assert(bootstrap.operation_complete());
      assert(bootstrap.start_renew_lease(snapshot,30000000,24));
      actual_inflight=bootstrap.renewal_request_inflight();assert(actual_inflight);
    }
    if(mode=="withdraw_status_lag") {
      // Status可保持同身份/SENT证明但序号较稀疏；终点ACK来自新鲜State。
      assert(gate.bounded_completion_intent(live,17U,now));
      assert(!gate.bounded_completion_intent(live,16U,now));
    }
    const bool bounded=intent && mode!="withdraw_stale_state" && mode!="withdraw_wrong_run";
    const auto original_request=queued.load();const auto original_reserved=reserved.load();
    const auto original_running=running.load();const auto original_started=started.load();
    const bool withdrawn=withdraw_queued_running_renewal(queued,reserved,running,started,
        owns_busy,actual_inflight,bounded,explicit_status);
    const bool expect=mode=="withdraw_positive"||mode=="withdraw_explicit"||mode=="withdraw_status_lag";
    assert(withdrawn==expect);assert(busy);
    if(expect) {
      assert(queued==0&&!reserved&&!running&&!started);
      assert(client.matching_inflight_renewal(live)==0U);
      /**
       * @brief 显式pending状态1撤回为−1，不能误consume为已确认续租。
       * 之后自动预约可继续但保留typed撤回提示；take_withdrawn只消费一次−1→0，不签发新permit。
       */
      if(mode=="withdraw_explicit") {
        assert(explicit_status==-1);
        assert(!take_confirmed_lease_renewal(explicit_status,true,true,true,ManageOperation::kRenewLease));
        // 自动预约不覆盖旧typed撤回结果，也不被未消费结果阻塞。
        assert(queue_lease_renewal(queued,busy,reserved,true));assert(explicit_status==-1);
        assert(take_withdrawn_lease_renewal(explicit_status));assert(explicit_status==0);
        assert(!take_withdrawn_lease_renewal(explicit_status));
        queued=0;reserved=false;
      }
      /**
       * @brief 撤预约后恰新增一条Complete8且terminal_producer_seq17、身份原样，terminal仍None。
       * 此测试到“Complete已发送”为止，没有完成后续Complete COMMITTED/Hold/机械到位的全过程。
       */
      reply->accepted_seq=17;gate.observe(*reply,26);
      assert(gate.terminal()==Terminal::kNone && transport.requests.back().operation()==8);
      assert(transport.requests.size()==wire_count_before_freeze+1U);
      assert(transport.requests.back().terminal_producer_seq()==17U);
      assert(transport.requests.back().expected_boot_epoch()==live.expected_boot_epoch());
      assert(transport.requests.back().expected_session_id()==live.expected_session_id());
      std::cerr<<"WITHDRAW_PIPELINE scope=REAL_WIRE_GATE_AND_PRODUCTION_QUEUE_HELPER_NOT_NODE_OBJECT no_new_renew=1 complete_sent=1 busy_preserved=1 explicit="<<mode<<'\n';
    } else {
      assert(queued==original_request && reserved==original_reserved && running==original_running && started==original_started);
      if(mode=="withdraw_actual_inflight")assert(bootstrap.renewal_request_inflight());
    }
    return 0;
  }
  if(mode=="positive"||mode=="sent_complete") {
    std::cout<<"QUEUED_UNSENT_BOUNDARY scope=REAL_WIRE_GATE_AND_NODE_QUEUE_POLICY_NOT_NODE_OBJECT phase="<<unsigned(gate.phase())
             <<" reserved="<<reserved<<" started="<<started<<" request="<<queued<<" busy="<<busy
             <<" ack="<<ack<<" now="<<now<<" intent="<<intent<<" renew_wire_count=0\n";
    assert(intent);
    assert(transport.requests.size()==wire_count_before_freeze+(mode=="sent_complete"?1U:0U));
  } else assert(!intent);
}
