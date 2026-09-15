#include "action_test_support.hpp"
#include <iostream>
int main() {
  using namespace action_test;
  Harness harness; Authorization auth; auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_session_id(1); require(harness.gate.enable(auth,1));
  ManageReply queued; queued.request_id=1; queued.queued=true;
  harness.gate.observe(queued,2); require(harness.port.primes==0);
  ++harness.epoch; harness.gate.observe(harness.response(),3);
  const bool enable_prime=harness.port.primes==1 && !harness.gate.ready(); require(enable_prime);
  Position discontinuous{}; discontinuous[0]=0.2;
  require(!harness.gate.record_update(harness.gate.generation(),harness.epoch,1,discontinuous));
  require(harness.gate.record_update(harness.gate.generation(),harness.epoch,1,Position{}));
  require(harness.gate.record_update(harness.gate.generation(),harness.epoch,2,Position{}));
  harness.gate.observe(harness.response(true,1),4);
  const bool ack_first_seq_not_latest=harness.gate.ready(); require(ack_first_seq_not_latest);
  const Trajectory trajectory{{"Joint1"},{{{0.2},{0},{},1}},{},{},0};
  require(harness.gate.submit(trajectory,10)); ++harness.epoch;
  harness.gate.observe(harness.response(),11);
  const bool begin_prime_before_trajectory=harness.port.primes==2 && harness.port.starts==0;
  require(begin_prime_before_trajectory);
  require(harness.gate.record_update(harness.gate.generation(),harness.epoch,1,Position{}));
  harness.gate.observe(harness.response(true),12); require(harness.port.starts==1);
  std::cout << std::boolalpha << "{\"enable_prime_before_success\":" << enable_prime
    << ",\"ack_first_seq_not_latest\":" << ack_first_seq_not_latest
    << ",\"begin_prime_before_trajectory\":" << begin_prime_before_trajectory << "}\n";
}
