#include "dummy_motion_client/action_coordinator.hpp"

int main() {
  using dummy_motion_client::ActionCoordinator;
  using dummy_motion_client::Phase;
  using dummy_motion_client::Terminal;
  auto hold = ActionCoordinator{}.enable_accepted(1U)->hold_sample_accepted(1U, true).value();
  auto running = hold.begin_accepted(20U, 2U)->motion_sample_accepted(2U, true).value();
  auto stopping = running.request_stop(true);
  if (!stopping || stopping->producer_enabled || stopping->phase != Phase::kStopping) return 1;
  if (stopping->inner_succeeded(91U).has_value()) return 2;
  if (stopping->begin_accepted(21U, 3U).has_value()) return 3;
  auto stop_hold = stopping->stop_ready(3U);
  if (!stop_hold || stop_hold->phase != Phase::kPrimeStopHold) return 4;
  auto canceled = stop_hold->hold_sample_accepted(3U, true);
  if (!canceled || canceled->terminal != Terminal::kCanceled || canceled->goal != 0U) return 5;
  auto aborted_stop = running.request_stop(false)->stop_ready(3U)->hold_sample_accepted(3U, true);
  if (!aborted_stop || aborted_stop->terminal != Terminal::kAborted) return 6;
  const auto failed = running.request_stop(true)->stop_failed();
  return failed.phase == Phase::kFault && failed.terminal == Terminal::kAborted ? 0 : 7;
}
