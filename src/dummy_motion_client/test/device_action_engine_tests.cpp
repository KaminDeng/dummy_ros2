#include "dummy_motion_client/action_gate.hpp"
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace dummy_motion_client;
void check(bool value) { if (!value) throw std::runtime_error("lifecycle assertion failed"); }
struct Client : ManagementClient {
  bool reject_cancel=false;
  bool throw_on_poll=false;
  std::vector<ManageRequest> requests;
  std::deque<ManageReply> replies;
  std::uint64_t submit(const ManageRequest &request, std::uint64_t) override {
    if(reject_cancel && request.operation()==wire_value(ManageOperation::kCancel)) return 0;
    auto copy = request; copy.set_request_id(requests.size()+1); requests.push_back(copy);
    return copy.request_id();
  }
  std::optional<ManageReply> poll(std::uint64_t) override {
    if (throw_on_poll) throw std::runtime_error("management poll failed");
    if (replies.empty()) return {}; auto next = replies.front(); replies.pop_front(); return next;
  }
};
struct Port : ActionPort {
  int primes=0, starts=0, disables=0, cancels=0;
  bool stream_hold=false;
  void prime(const Position &, std::uint32_t, std::uint64_t) override { ++primes; }
  void start(const Trajectory &, std::uint64_t) override { ++starts; }
  void disable() override { ++disables; }
  void cancel() override { ++cancels; }
  bool freeze_hold() override { if (!stream_hold) return false; disable(); return true; }
};
Trajectory trajectory() { return {{"Joint1"}, {{{0.2},{0.0},{},1.0}}, {}, {},0.0}; }
JointProfile profile() { return {{"Joint1"}, {-1}, {1}, {1}, {2}, {0.1}, {0.1}}; }
ManageReply reply(std::uint64_t id, std::uint32_t epoch, bool success=false) {
  ManageReply result; result.request_id=id; result.accepted=true; result.operation_id=id+100;
  result.result_valid=success; result.succeeded=success; result.snapshot_valid=true;
  result.motion_epoch=epoch; result.position={}; result.hold_position={};
  result.hold_available=true; result.hold_ready=success; result.accepted_seq=1;
  result.session_id=9U;
  return result;
}
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
  gate.inner_result(generation,Terminal::kSucceeded,"",8U);
  auto accepted=reply(2U,6U,true);
  accepted.accepted_seq=2U;
  gate.observe(accepted,9U);
  check(client.requests.back().operation()==wire_value(ManageOperation::kComplete));
  check(client.requests.back().expected_motion_epoch()==6U);
}
void begin_waits_for_final_hold_commit() {
  namespace wire=dp::protocol::dummy_robot_v2;
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
  state.set_schema_major(2U); state.set_schema_minor(4U); state.set_boot_epoch(7U);
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
void discarded_local_hold_generation_does_not_block_begin() {
  namespace wire=dp::protocol::dummy_robot_v2;
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
  state.set_schema_major(2U); state.set_schema_minor(4U);
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
  dp::protocol::dummy_robot_v2::DummyJointStateV2 state;
  state.set_schema_major(2U); state.set_schema_minor(4U);
  state.set_session_id(9U); state.set_state_seq(1U);
  state.set_motion_epoch(1U);
  state.set_configured_mask(1U); state.set_position_valid_mask(1U);
  state.set_core_state(wire_value(WireCoreState::kHold));
  state.set_hold_ready(true); state.set_committed_producer_seq(1U);
  state.set_can_submitted_mask(1U); state.set_can_sent_mask(1U);
  std::vector<std::uint8_t> bytes(dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::kEncodedSize);
  check(dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::encode(
    state,bytes.data(),bytes.size())==dp::protocol::dummy_robot_v2::CodecResult::kOk);
  check(gate.observe_state(bytes.data(),bytes.size(),103U));
  check(client.requests.size()==2U &&
        client.requests.back().operation()==wire_value(ManageOperation::kBegin));
  gate.tick(104U);
  check(client.requests.size()==3U &&
        client.requests.back().operation()==wire_value(ManageOperation::kCancel) &&
        gate.error()=="RESULT_UNCONFIRMED");
}
void hold_drain_accepts_wrap_but_rejects_old_commit() {
  namespace wire=dp::protocol::dummy_robot_v2;
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
  state.set_schema_major(2U); state.set_schema_minor(4U);
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
int main() {
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
  check(deferred_gate.submit(fast_first,4)); deferred.replies.push_back(reply(2,2)); deferred_gate.tick(5);
  const bool deferred_start_rejected=deferred_port.starts==0 && deferred_port.cancels==1 &&
      deferred_gate.terminal()==Terminal::kNone && deferred.requests.back().operation()==wire_value(ManageOperation::kCancel);
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
