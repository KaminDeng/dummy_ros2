#pragma once

#include <cstdint>
#include <optional>

namespace dummy_motion_client {

enum class Phase { kIdle, kPrimeHold, kHold, kPrimeMotion, kRunning, kCompleting,
                   kPrimeCompletionHold, kStopping, kPrimeStopHold, kFault };
enum class Terminal { kNone, kSucceeded, kCanceled, kAborted };

struct ActionCoordinator {
  Phase phase = Phase::kIdle;
  Terminal terminal = Terminal::kNone;
  std::uint64_t goal = 0;
  std::uint32_t motion_epoch = 0;
  std::uint32_t terminal_producer_seq = 0;
  bool producer_enabled = false;
  bool cancel_terminal = false;

  std::optional<ActionCoordinator> enable_accepted(std::uint32_t hold_epoch) const {
    if (phase != Phase::kIdle || hold_epoch == 0U) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kPrimeHold;
    next.motion_epoch = hold_epoch;
    next.producer_enabled = true;
    return next;
  }

  std::optional<ActionCoordinator> hold_sample_accepted(std::uint32_t epoch,
                                                         bool continuous) const {
    if ((phase != Phase::kPrimeHold && phase != Phase::kPrimeCompletionHold &&
         phase != Phase::kPrimeStopHold) || epoch != motion_epoch || !continuous)
      return std::nullopt;
    auto next = *this;
    next.phase = Phase::kHold;
    next.producer_enabled = true;
    if (phase == Phase::kPrimeCompletionHold) next.terminal = Terminal::kSucceeded;
    if (phase == Phase::kPrimeStopHold)
      next.terminal = cancel_terminal ? Terminal::kCanceled : Terminal::kAborted;
    next.goal = 0;
    return next;
  }

  std::optional<ActionCoordinator> begin_accepted(std::uint64_t goal_id,
                                                   std::uint32_t epoch) const {
    if (phase != Phase::kHold || goal_id == 0U || epoch <= motion_epoch) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kPrimeMotion;
    next.terminal = Terminal::kNone;
    next.goal = goal_id;
    next.motion_epoch = epoch;
    next.producer_enabled = true;
    return next;
  }

  std::optional<ActionCoordinator> motion_sample_accepted(std::uint32_t epoch,
                                                           bool continuous) const {
    if (phase != Phase::kPrimeMotion || epoch != motion_epoch || !continuous) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kRunning;
    return next;
  }

  std::optional<ActionCoordinator> inner_succeeded(std::uint32_t accepted_seq) const {
    if (phase != Phase::kRunning || accepted_seq == 0U) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kCompleting;
    next.terminal_producer_seq = accepted_seq;
    next.producer_enabled = false;
    return next;
  }

  std::optional<ActionCoordinator> complete_stable(std::uint32_t hold_epoch) const {
    if (phase != Phase::kCompleting || hold_epoch <= motion_epoch) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kPrimeCompletionHold;
    next.motion_epoch = hold_epoch;
    next.producer_enabled = true;
    return next;
  }

  std::optional<ActionCoordinator> request_stop(bool canceled) const {
    if (phase != Phase::kPrimeMotion && phase != Phase::kRunning && phase != Phase::kCompleting)
      return std::nullopt;
    auto next = *this;
    next.phase = Phase::kStopping;
    next.producer_enabled = false;
    next.cancel_terminal = canceled;
    return next;
  }

  std::optional<ActionCoordinator> stop_ready(std::uint32_t hold_epoch) const {
    if (phase != Phase::kStopping || hold_epoch <= motion_epoch) return std::nullopt;
    auto next = *this;
    next.phase = Phase::kPrimeStopHold;
    next.motion_epoch = hold_epoch;
    next.producer_enabled = true;
    return next;
  }

  ActionCoordinator stop_failed() const {
    auto next = *this;
    next.phase = Phase::kFault;
    next.terminal = Terminal::kAborted;
    next.producer_enabled = false;
    return next;
  }
};

enum class Receipt { kNone, kQueued, kRejected };
enum class OperationStatus { kIdle, kWaiting, kAccepted, kSucceeded, kFailed };

struct ManagementOperationTracker {
  std::uint64_t request_id = 0;
  std::uint64_t operation_id = 0;
  Receipt receipt = Receipt::kNone;
  OperationStatus status = OperationStatus::kIdle;

  std::optional<ManagementOperationTracker> queued(std::uint64_t request) const {
    if (status != OperationStatus::kIdle || request == 0U) return std::nullopt;
    return ManagementOperationTracker{request, 0U, Receipt::kQueued, OperationStatus::kWaiting};
  }
  std::optional<ManagementOperationTracker> accepted(std::uint64_t request,
                                                      std::uint64_t operation) const {
    if (status != OperationStatus::kWaiting || request != request_id || operation == 0U)
      return std::nullopt;
    return ManagementOperationTracker{request, operation, receipt, OperationStatus::kAccepted};
  }
  std::optional<ManagementOperationTracker> result(std::uint64_t operation, bool success) const {
    if (status != OperationStatus::kAccepted || operation != operation_id) return std::nullopt;
    auto next = *this;
    next.status = success ? OperationStatus::kSucceeded : OperationStatus::kFailed;
    return next;
  }
};
}  // namespace dummy_motion_client
