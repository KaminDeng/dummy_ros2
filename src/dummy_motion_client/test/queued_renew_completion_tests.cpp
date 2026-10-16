#define main shared_wire_fixture_main
#include "wire_management_tests.cpp"
#undef main
#define DUMMY_ACTION_GATE_TEST
#include "../src/device_action_gate.cpp"
#include <cassert>

struct CompletionPort : Port {
  bool executing=false;
  void start(const Trajectory &t,std::uint64_t g) override {Port::start(t,g);executing=true;}
  void disable() override {executing=false;}
};
int main(int argc,char **argv) {
  const std::string mode=argc>1?argv[1]:"positive";
  Transport transport;WireManagementClient client(transport);CompletionPort port;
  ActionGate gate(client,port,{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}},2000000);
  Authorization auth;auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_boot_epoch(7);auth.identity.set_expected_session_id(9);
  auth.identity.set_expected_reference_epoch(1);auth.identity.set_expected_profile_id(201);
  auth.identity.set_expected_management_epoch(1);auth.identity.set_configured_mask(1);
  auto commit=[&](std::uint32_t epoch) {
    const auto request=transport.requests.back();
    wire::DummyManageV2Response response;response.set_schema_major(2);response.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
    response.set_request_id(request.request_id());response.set_boot_epoch(7);response.set_session_id(9);
    response.set_reference_epoch(1);response.set_motion_epoch(epoch);response.set_management_epoch(1);
    response.set_profile_id(201);response.set_receipt(1);response.set_result_code(0);
    response.set_operation_id(request.request_id()+100);response.set_transaction_phase(3);
    transport.respond(response);return client.poll(request.request_id());
  };
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
      if(mode=="withdraw_explicit") {
        assert(explicit_status==-1);
        assert(!take_confirmed_lease_renewal(explicit_status,true,true,true,ManageOperation::kRenewLease));
        // 自动预约不覆盖旧typed撤回结果，也不被未消费结果阻塞。
        assert(queue_lease_renewal(queued,busy,reserved,true));assert(explicit_status==-1);
        assert(take_withdrawn_lease_renewal(explicit_status));assert(explicit_status==0);
        assert(!take_withdrawn_lease_renewal(explicit_status));
        queued=0;reserved=false;
      }
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
