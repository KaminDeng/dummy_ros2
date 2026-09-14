#include "dummy_motion_client/action_coordinator.hpp"

int main() {
  using dummy_motion_client::ManagementOperationTracker;
  using dummy_motion_client::OperationStatus;
  using dummy_motion_client::Receipt;
  ManagementOperationTracker tracker;
  auto queued = tracker.queued(9007199254740993ULL);
  if (!queued || queued->receipt != Receipt::kQueued || queued->status != OperationStatus::kWaiting ||
      queued->operation_id != 0U) return 1;
  if (queued->result(7U, true).has_value()) return 2;
  if (queued->accepted(8U, 7U).has_value()) return 3;
  auto accepted = queued->accepted(9007199254740993ULL, 9007199254740995ULL);
  if (!accepted || accepted->status != OperationStatus::kAccepted) return 4;
  if (accepted->result(2U, true).has_value()) return 5;
  auto completed = accepted->result(9007199254740995ULL, true);
  return completed && completed->status == OperationStatus::kSucceeded ? 0 : 6;
}
