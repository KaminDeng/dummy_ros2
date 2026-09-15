#pragma once
#include <atomic>
#include "dummy_motion_client/deadline.hpp"
namespace dummy_motion_client {
class NativeWaitPolicy {
 public:
  enum class Phase { kIdle, kAdmitted, kWaiting, kCanceling, kFault };
  bool try_acquire() {
    auto expected=Phase::kIdle;
    return phase_.compare_exchange_strong(expected,Phase::kAdmitted);
  }
  bool busy() const { return phase_.load()!=Phase::kIdle; }
  bool faulted() const { return phase_.load()==Phase::kFault; }
  void begin(std::uint64_t now,std::uint64_t budget) {
    if (faulted()) return;
    cancel_ack_=false; terminal_=false;
    deadline_=saturating_deadline(now,budget); phase_.store(Phase::kWaiting);
  }
  void cancel(std::uint64_t now,std::uint64_t budget) {
    if (faulted() || phase_.load()==Phase::kCanceling) return;
    deadline_=saturating_deadline(now,budget); phase_.store(Phase::kCanceling);
  }
  void cancel_acknowledged() { if (!faulted()) cancel_ack_=true; }
  void terminal_confirmed() { if (!faulted()) terminal_=true; }
  bool cancel_complete() const { return phase_.load()==Phase::kCanceling && cancel_ack_ && terminal_; }
  bool expired(std::uint64_t now) const { return now>=deadline_; }
  void latch_fault() { phase_.store(Phase::kFault); }
  void finish() {
    const auto phase=phase_.load();
    if (phase==Phase::kFault) return;
    const bool confirmed=phase==Phase::kAdmitted ||
        (phase==Phase::kWaiting && terminal_) || cancel_complete();
    phase_.store(confirmed?Phase::kIdle:Phase::kFault);
  }
 private:
  std::atomic<Phase> phase_{Phase::kIdle};
  // Deadline methods run on the single action worker; admission is atomic.
  std::uint64_t deadline_=0;
  bool cancel_ack_=false, terminal_=false;
};
}  // namespace dummy_motion_client
