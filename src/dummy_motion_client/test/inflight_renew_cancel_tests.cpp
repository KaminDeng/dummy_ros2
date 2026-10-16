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
  auto renew=auth.identity;renew.set_operation(wire_value(ManageOperation::kRenewLease));renew.set_expected_motion_epoch(2);renew.set_requested_session_lease_us(30000000);
  if(mode=="wrong_boot")renew.set_expected_boot_epoch(8);
  if(mode=="wrong_session")renew.set_expected_session_id(10);
  if(mode=="wrong_ref")renew.set_expected_reference_epoch(2);
  if(mode=="wrong_motion")renew.set_expected_motion_epoch(3);
  if(mode=="wrong_management")renew.set_expected_management_epoch(2);
  if(mode=="wrong_profile")renew.set_expected_profile_id(202);
  if(mode=="wrong_mask")renew.set_configured_mask(3);
  if(mode=="wrong_device"){uint8_t bytes[16]{};bytes[0]=3;renew.set_device_id(bytes);}
  if(mode=="wrong_client"){uint8_t bytes[16]{};bytes[0]=4;renew.set_client_instance_id(bytes);}
  auto rid=client.submit(renew,20);assert(rid!=0 && transport.requests.back().operation()==20);
  auto replywire=[&](const ManageRequest&q,std::uint8_t phase){wire::DummyManageV2Response r;r.set_schema_major(2);r.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);r.set_request_id(q.request_id());r.set_boot_epoch(7);r.set_session_id(9);r.set_reference_epoch(1);r.set_motion_epoch(2);r.set_management_epoch(1);r.set_profile_id(201);r.set_receipt(1);r.set_operation_id(rid+100);r.set_transaction_phase(phase);r.set_result_code(0);return r;};
  if(mode.rfind("wrong_",0)==0){ gate.cancel(25);assert(gate.error()=="STOP_FAILED");assert(!gate.cancel_waiting_for_renewal(renew,26));return 0;}
  transport.respond(replywire(transport.requests.back(),2));auto prepared=client.poll(rid);
  assert(prepared);
  assert(transport.requests.back().operation()==12);
  auto query=transport.requests.back();transport.respond(replywire(query,3));
  assert(transport.responses.size()==1);const auto wirebefore=transport.requests.size();
  if(mode=="no_cancel"){auto x=auth.identity;x.set_operation(wire_value(ManageOperation::kCancel));x.set_expected_motion_epoch(2);assert(!gate.cancel_waiting_for_renewal(x,25));return 0;}
  if(mode=="abort"){gate.abort("REAL_PRODUCER_LOST",25);assert(gate.error()=="STOP_FAILED");return 0;}
  gate.cancel(25);

  std::cerr<<"REAL_WIRE_COMMITTED_UNCONSUMED_CANCEL renewrid="<<rid<<" queryrid="<<query.request_id()<<" bufferedCommitted="<<transport.responses.size()<<" phase="<<unsigned(gate.phase())<<" error="<<gate.error()<<" wirebefore="<<wirebefore<<" wireafter="<<transport.requests.size()<<" portexecuting="<<port.executing<<"\n";
  assert(gate.error().empty());
  auto live=auth.identity;live.set_expected_motion_epoch(2);live.set_operation(wire_value(ManageOperation::kCancel));
  assert(gate.cancel_waiting_for_renewal(live,26));
  if(mode=="waiting_abort"){
    gate.abort("EVIDENCE_INCOMPLETE",26);
    const auto before=transport.requests.size();auto renewal=client.poll(rid);assert(renewal);gate.tick(27);
    assert(transport.requests.size()==before);assert(gate.error()=="EVIDENCE_INCOMPLETE");return 0;
  }
    if(mode=="invalidate"){
    gate.invalidate_authorization(26);assert(gate.error()=="SESSION_CHANGED");
    const auto before=transport.requests.size();auto renewreply=client.poll(rid);assert(renewreply);gate.tick(27);
    assert(transport.requests.size()==before);assert(gate.error()=="SESSION_CHANGED");return 0;
  }
  if(mode.rfind("guard_",0)==0){auto bad=live;
    if(mode=="guard_boot")bad.set_expected_boot_epoch(8);
    if(mode=="guard_session")bad.set_expected_session_id(10);
    if(mode=="guard_ref")bad.set_expected_reference_epoch(2);
    if(mode=="guard_motion")bad.set_expected_motion_epoch(3);
    if(mode=="guard_management")bad.set_expected_management_epoch(2);
    if(mode=="guard_profile")bad.set_expected_profile_id(202);
    if(mode=="guard_mask")bad.set_configured_mask(3);
    if(mode=="guard_operation")bad.set_operation(20);
    assert(!gate.cancel_waiting_for_renewal(bad,26));assert(gate.cancel_waiting_for_renewal(live,26));
    assert(!run_lease_eligible(run_lease_continuation_inputs({true,true,true,true,false,true,true},true,false)));
    assert(run_lease_eligible(run_lease_continuation_inputs({true,true,true,true,false,true,true},true,gate.cancel_waiting_for_renewal(live,26))));
    assert(!gate.cancel_waiting_for_renewal(live,2000025));return 0;
  }

  if(mode=="expired"){gate.tick(2000025);assert(gate.error()=="STOP_FAILED");return 0;}
  if(mode=="failed_renew"){transport.responses.clear();auto bad=replywire(query,4);bad.set_result_code(10);transport.respond(bad);}
  auto committed=client.poll(rid);assert(committed);
  if(mode=="failed_renew"){gate.tick(26);assert(gate.error()=="STOP_FAILED");return 0;}
  gate.tick(26);
  assert(transport.requests.back().operation()==9);
  auto cancelrid=transport.requests.back().request_id();assert(cancelrid!=rid && cancelrid!=query.request_id());
  assert(!gate.cancel_waiting_for_renewal(live,27));
  std::cerr<<"CANCEL_AFTER_RENEW_COMMITTED cancelRID="<<cancelrid<<" oldrenewRID="<<rid<<" no_new_renew=1\n";
  assert(gate.phase()==Phase::kStopping);return 0;
}
