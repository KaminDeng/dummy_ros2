#include "dummy_motion_client/action_coordinator.hpp"

int main() {
  using dummy_motion_client::ActionCoordinator;
  using dummy_motion_client::Phase;
  using dummy_motion_client::Terminal;
  ActionCoordinator state;
  auto prime_hold = state.enable_accepted(1U);
  if (!prime_hold || prime_hold->phase != Phase::kPrimeHold || !prime_hold->producer_enabled) return 1;
  if (prime_hold->hold_sample_accepted(1U, false).has_value()) return 2;
  auto hold = prime_hold->hold_sample_accepted(1U, true);
  if (!hold || hold->phase != Phase::kHold) return 3;
  auto prime = hold->begin_accepted(10U, 2U);
  if (!prime || prime->phase != Phase::kPrimeMotion) return 4;
  if (prime->begin_accepted(11U, 3U).has_value()) return 5;
  if (prime->motion_sample_accepted(1U, true).has_value()) return 6;
  auto running = prime->motion_sample_accepted(2U, true);
  if (!running || running->phase != Phase::kRunning) return 7;
  auto completing = running->inner_succeeded(88U);
  if (!completing || completing->producer_enabled || completing->terminal_producer_seq != 88U) return 8;
  auto completion_hold = completing->complete_stable(3U);
  if (!completion_hold || completion_hold->phase != Phase::kPrimeCompletionHold) return 9;
  auto succeeded = completion_hold->hold_sample_accepted(3U, true);
  if (!succeeded || succeeded->phase != Phase::kHold || succeeded->terminal != Terminal::kSucceeded)
    return 10;
  auto second = succeeded->begin_accepted(11U, 4U);
  return second && second->terminal == Terminal::kNone && second->goal == 11U ? 0 : 11;
}
