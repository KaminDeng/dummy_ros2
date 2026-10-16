#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_ros_control/diagnostic_scope.hpp"
#include <cassert>
#include <cstring>
#include "dummy_ros_control/diagnostic_json.hpp"
#include <iostream>
using namespace dummy_motion_client;
struct Client final : ManagementClient {
  std::uint64_t next=0U;
  std::uint64_t submit(const ManageRequest&,std::uint64_t) override {return ++next;}
  std::optional<ManageReply> poll(std::uint64_t) override {return {};}
};
struct Port final : ActionPort {
  void prime(const Position&,std::uint32_t,std::uint64_t) override {}
  void start(const Trajectory&,std::uint64_t) override {}
  void disable() override {}
  void cancel() override {}
  bool freeze_hold() override {return true;}
};
void action_chain() {
  Client client;Port port;
  ActionGate gate(client,port,{{"Joint1"},{-1},{1},{1},{2},{0.1},{0.1}});
  Authorization auth;auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_boot_epoch(7U);auth.identity.set_expected_session_id(9U);
  auth.identity.set_expected_reference_epoch(3U);auth.identity.set_expected_motion_epoch(1U);
  assert(gate.enable(auth,1U));
  ManageReply reply;reply.request_id=1U;reply.accepted=true;reply.snapshot_valid=true;
  reply.motion_epoch=2U;reply.hold_available=true;reply.session_id=9U;
  gate.observe(reply,2U);
  assert(gate.record_update(gate.generation(),2U,1U,Position{},9U));
  reply.result_valid=reply.succeeded=reply.hold_ready=true;reply.accepted_seq=1U;
  gate.observe(reply,3U);assert(gate.ready());
  assert(gate.submit({{"Joint1"},{{{0.2},{0.0},{},1.0}},{},{},0.0},4U));
  gate.observe_hold_drain({7U,9U,9U,3U,2U,1U},5U);
  gate.tick(1000004U);assert(gate.error()=="HOLD_DRAIN_TIMEOUT");
  gate.tick(2000004U);assert(gate.error()=="STOP_FAILED" && gate.terminal()==Terminal::kAborted);
}
struct ReplyClient final : ManagementClient {
  std::uint64_t submit(const ManageRequest&,std::uint64_t) override {return 1U;}
  std::optional<ManageReply> poll(std::uint64_t) override {
    ManageReply reply;reply.request_id=1U;reply.result_valid=reply.succeeded=reply.release_committed=true;
    reply.transaction_phase=3U;reply.operation_id=91U;return reply;
  }
};
void bootstrap_chain() {
  ReplyClient client;ManageRequest identity;identity.set_expected_session_id(9U);
  BootstrapCoordinator coordinator(client,identity,{},1000000U);
  BootstrapSnapshot snapshot;snapshot.boot_epoch=7U;snapshot.session_id=9U;
  snapshot.reference_epoch=3U;snapshot.motion_epoch=4U;snapshot.management_epoch=5U;
  snapshot.profile_id=201U;snapshot.configured_mask=63U;snapshot.freshness_ticket=1U;
  snapshot.raw_ready=snapshot.motion_supported=true;snapshot.core_state=snapshot.state_core_state=4U;
  assert(coordinator.start_release(snapshot,100U));coordinator.tick(101U);
  assert(coordinator.confirmed_release());coordinator.observe(snapshot,5000100U);
  assert(coordinator.phase()==BootstrapPhase::kFault && std::strcmp(coordinator.error(),"RELEASE_UNCONFIRMED")==0);
}
int main(int argc,char**argv) {
  dummy_ros_control::DiagnosticJournal journal;
  dummy_ros_control::HostDiagnosticScope scope(journal);
  const bool bootstrap=argc==2 && std::strcmp(argv[1],"bootstrap")==0;
  if(bootstrap)bootstrap_chain();else action_chain();
  using dummy_ros_control::DiagnosticReason;
  if(bootstrap) {
    const auto reply=journal.first_record(DiagnosticReason::kBootstrapReleaseReply,true);
    const auto timeout=journal.first_record(DiagnosticReason::kBootstrapReleaseTimeout,true);
    assert(reply && reply->fields[0]==1U && reply->fields[8]==91U);
    assert(timeout && timeout->fields[0]==10U && timeout->fields[2]==9U && timeout->fields[8]==0U);
    assert(journal.snapshot().first.reason==DiagnosticReason::kBootstrapReleaseTimeout);
  } else {
    const auto timeout=journal.first_record(DiagnosticReason::kActionHoldDrainTimeout,true);
    const auto stop=journal.first_record(DiagnosticReason::kActionStop,true);
    const auto fail=journal.first_record(DiagnosticReason::kActionFail,true);
    assert(timeout && timeout->fields[0]==1U && timeout->fields[1]==9U && timeout->fields[7]==2U);
    assert(stop && stop->fields[3]==1000004U && stop->expected.boot==7U);
    assert(fail && fail->fields[2]==2U && std::strcmp(fail->detail.data(),"STOP_FAILED")==0);
    assert(journal.snapshot().detail_suppressed>=6U);
  }
  std::cout << dummy_ros_control::diagnostic_journal_json(journal) << '\n';
}
