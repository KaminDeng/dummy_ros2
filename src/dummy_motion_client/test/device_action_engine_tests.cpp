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
  void prime(const Position &, std::uint32_t, std::uint64_t) override { ++primes; }
  void start(const Trajectory &, std::uint64_t) override { ++starts; }
  void disable() override { ++disables; }
  void cancel() override { ++cancels; }
};
Trajectory trajectory() { return {{"Joint1"}, {{{0.2},{0.0},{},1.0}}, {}, {},0.0}; }
JointProfile profile() { return {{"Joint1"}, {-1}, {1}, {1}, {2}, {0.1}, {0.1}}; }
ManageReply reply(std::uint64_t id, std::uint32_t epoch, bool success=false) {
  ManageReply result; result.request_id=id; result.accepted=true; result.operation_id=id+100;
  result.result_valid=success; result.succeeded=success; result.snapshot_valid=true;
  result.motion_epoch=epoch; result.position={}; result.hold_position={};
  result.hold_available=true; result.hold_ready=success; result.accepted_seq=1;
  return result;
}
int main() {
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
  const bool completion_barrier=gate.terminal()==Terminal::kSucceeded && gate.ready();
  check(completion_barrier);
  check(gate.submit(trajectory(),12)); client.replies.push_back(reply(4,4)); gate.tick(13);
  gate.cancel(14);
  const bool cancel_pending=port.cancels==1 && !gate.submit(trajectory(),14) && gate.terminal()==Terminal::kNone;
  check(cancel_pending);
  gate.inner_result(generation,Terminal::kSucceeded,"",15); check(gate.terminal()==Terminal::kNone);
  client.replies.push_back(reply(5,5)); gate.tick(16);
  check(gate.record_update(gate.generation(),5,1,Position{}));
  client.replies.push_back(reply(5,5,true)); gate.tick(17);
  const bool cancel_barrier=cancel_pending && gate.terminal()==Terminal::kCanceled && gate.ready();
  check(cancel_barrier);
  check(gate.submit(trajectory(),18)); client.replies.push_back(reply(6,6)); gate.tick(19);
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
