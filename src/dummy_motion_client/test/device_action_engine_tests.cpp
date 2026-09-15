#include "dummy_motion_client/action_gate.hpp"
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace dummy_motion_client;
void check(bool value) { if (!value) throw std::runtime_error("lifecycle assertion failed"); }
struct Client : ManagementClient {
  std::vector<ManageRequest> requests;
  std::deque<ManageReply> replies;
  std::uint64_t submit(const ManageRequest &request, std::uint64_t) override {
    auto copy = request; copy.set_request_id(requests.size()+1); requests.push_back(copy);
    return copy.request_id();
  }
  std::optional<ManageReply> poll(std::uint64_t) override {
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
  Position terminal{}; terminal[0]=0.2;
  check(gate.record_update(generation,2,17,terminal));
  gate.inner_result(generation,Terminal::kSucceeded,"",8);
  check(gate.terminal()==Terminal::kNone && client.requests.size()==2);
  check(!gate.record_update(generation,2,18,terminal));
  auto accepted=reply(2,2,true); accepted.accepted_seq=17;
  gate.observe(accepted,9);
  const auto complete_seq=client.requests.back().terminal_producer_seq(); check(complete_seq==17);
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
    << ",\"late_generation_accepted\":" << late_generation_accepted << "}\n";
}
