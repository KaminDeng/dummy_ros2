#include "dummy_motion_client/native_wait_policy.hpp"
#include <limits>
#include <stdexcept>
void require(bool value) { if (!value) throw std::runtime_error("native deadline requirement failed"); }
int main() {
  using dummy_motion_client::NativeWaitPolicy;
  NativeWaitPolicy policy;
  require(policy.try_acquire()); require(!policy.try_acquire());
  policy.begin(100,50); require(!policy.expired(149)); require(policy.expired(150));
  policy.terminal_confirmed();
  policy.finish(); require(!policy.busy()); require(policy.try_acquire());
  policy.begin(200,100); policy.cancel(210,20);
  policy.cancel(229,10000);
  require(!policy.expired(229)); require(policy.expired(230));
  policy.finish(); require(policy.busy()); require(policy.faulted()); require(!policy.try_acquire());
  policy.cancel_acknowledged(); policy.terminal_confirmed(); policy.finish();
  require(policy.faulted()); require(!policy.try_acquire());
  NativeWaitPolicy acknowledged; require(acknowledged.try_acquire());
  acknowledged.begin(100,50); acknowledged.cancel(150,20);
  require(!acknowledged.expired(151));
  acknowledged.cancel_acknowledged(); require(!acknowledged.cancel_complete());
  acknowledged.terminal_confirmed(); require(acknowledged.cancel_complete());
  acknowledged.finish(); require(!acknowledged.busy()); require(acknowledged.try_acquire());
  NativeWaitPolicy terminal_only; require(terminal_only.try_acquire());
  terminal_only.begin(100,50); terminal_only.cancel(150,20); terminal_only.terminal_confirmed();
  require(!terminal_only.cancel_complete()); terminal_only.finish();
  require(terminal_only.faulted()); require(!terminal_only.try_acquire());
  NativeWaitPolicy ack_only; require(ack_only.try_acquire());
  ack_only.begin(100,50); ack_only.cancel(150,20); ack_only.cancel_acknowledged();
  require(!ack_only.cancel_complete()); ack_only.finish();
  require(ack_only.faulted()); require(!ack_only.try_acquire());
  NativeWaitPolicy restarted; require(restarted.try_acquire());
  const auto maximum=std::numeric_limits<std::uint64_t>::max();
  restarted.begin(maximum-5,50); require(!restarted.expired(maximum-4)); require(restarted.expired(maximum));
  restarted.finish(); require(restarted.busy()); require(restarted.faulted());
}
