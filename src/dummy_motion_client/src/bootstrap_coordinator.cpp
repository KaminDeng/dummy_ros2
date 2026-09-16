#include "dummy_motion_client/bootstrap_coordinator.hpp"

#include <algorithm>

namespace dummy_motion_client {
BootstrapCoordinator::BootstrapCoordinator(
    ManagementClient &client, ManageRequest identity,
    std::array<std::uint8_t, 32> permit_digest, std::uint64_t permit_remaining_us)
    : client_(client), identity_(identity), permit_digest_(permit_digest),
      permit_remaining_us_(permit_remaining_us) {}

bool BootstrapCoordinator::issue(ManageOperation operation, std::uint64_t now_us) {
  auto request = identity_;
  request.set_operation(wire_value(operation));
  request.set_expected_boot_epoch(snapshot_.boot_epoch);
  request.set_expected_session_id(
      operation == ManageOperation::kSelectMode ? 0U : identity_.expected_session_id());
  request.set_expected_reference_epoch(snapshot_.reference_epoch);
  request.set_expected_motion_epoch(snapshot_.motion_epoch);
  request.set_expected_management_epoch(snapshot_.management_epoch);
  request.set_expected_profile_id(snapshot_.profile_id);
  request.set_freshness_ticket(snapshot_.freshness_ticket);
  request.set_configured_mask(snapshot_.configured_mask);
  if (operation == ManageOperation::kSelectMode) request.set_requested_control_mode(1U);
  if (operation == ManageOperation::kSetPermit) {
    std::uint8_t digest[32]{};
    std::copy(permit_digest_.begin(), permit_digest_.end(), digest);
    request.set_permit_sha256(digest);
    request.set_permit_remaining_us(permit_remaining_us_);
    request.set_operator_confirmed(true);
  }
  issued_management_epoch_ = snapshot_.management_epoch;
  request_id_ = client_.submit(request, now_us);
  if (request_id_ == 0U) {
    phase_ = BootstrapPhase::kFault;
    error_ = "BOOTSTRAP_TRANSPORT";
    return false;
  }
  return true;
}

bool BootstrapCoordinator::start(const BootstrapSnapshot &snapshot, std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kOperationComplete) || snapshot.boot_epoch == 0U ||
      snapshot.profile_id == 0U || snapshot.configured_mask == 0U ||
      snapshot.freshness_ticket == 0U || !snapshot.raw_ready ||
      !snapshot.motion_supported || identity_.expected_session_id() == 0U ||
      permit_remaining_us_ == 0U ||
      std::all_of(permit_digest_.begin(), permit_digest_.end(), [](auto value) { return value == 0U; }))
    return false;
  snapshot_ = snapshot;
  automatic_ = true;
  error_.clear();
  phase_ = BootstrapPhase::kSelectMode;
  return issue(ManageOperation::kSelectMode, now_us);
}

bool BootstrapCoordinator::start_operation(
    const BootstrapSnapshot &snapshot, ManageOperation operation, std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kOperationComplete) ||
      snapshot.boot_epoch == 0U || snapshot.profile_id == 0U ||
      snapshot.configured_mask == 0U || snapshot.freshness_ticket == 0U ||
      !snapshot.raw_ready || !snapshot.motion_supported) return false;
  switch (operation) {
    case ManageOperation::kSelectMode: phase_ = BootstrapPhase::kSelectMode; break;
    case ManageOperation::kGrantSession: phase_ = BootstrapPhase::kGrantSession; break;
    case ManageOperation::kCaptureReference: phase_ = BootstrapPhase::kCaptureReference; break;
    case ManageOperation::kSetPermit: phase_ = BootstrapPhase::kSetPermit; break;
    default: return false;
  }
  snapshot_ = snapshot;
  automatic_ = false;
  last_operation_ = operation;
  error_.clear();
  return issue(operation, now_us);
}

void BootstrapCoordinator::observe(const BootstrapSnapshot &snapshot) noexcept {
  if (phase_ == BootstrapPhase::kIdle) {
    snapshot_ = snapshot;
    return;
  }
  if (snapshot.boot_epoch != snapshot_.boot_epoch) {
    phase_ = BootstrapPhase::kFault;
    error_ = "BOOT_CHANGED";
    request_id_ = 0U;
    return;
  }
  snapshot_ = snapshot;
}

bool BootstrapCoordinator::precondition_for_next() const noexcept {
  if (snapshot_.management_epoch <= issued_management_epoch_) return false;
  switch (phase_) {
    case BootstrapPhase::kSelectMode: return snapshot_.session_id == 0U;
    case BootstrapPhase::kGrantSession:
    case BootstrapPhase::kCaptureReference:
    case BootstrapPhase::kSetPermit:
      return snapshot_.session_id == identity_.expected_session_id();
    default: return false;
  }
}

void BootstrapCoordinator::advance(std::uint64_t now_us) {
  if (!precondition_for_next()) return;
  if (phase_ == BootstrapPhase::kSelectMode) {
    phase_ = BootstrapPhase::kGrantSession;
    issue(ManageOperation::kGrantSession, now_us);
  } else if (phase_ == BootstrapPhase::kGrantSession) {
    phase_ = BootstrapPhase::kCaptureReference;
    issue(ManageOperation::kCaptureReference, now_us);
  } else if (phase_ == BootstrapPhase::kCaptureReference) {
    if (snapshot_.reference_epoch == 0U) return;
    phase_ = BootstrapPhase::kSetPermit;
    issue(ManageOperation::kSetPermit, now_us);
  } else if (phase_ == BootstrapPhase::kSetPermit) {
    phase_ = BootstrapPhase::kReady;
  }
}

void BootstrapCoordinator::tick(std::uint64_t now_us) {
  if (phase_ == BootstrapPhase::kIdle || phase_ == BootstrapPhase::kReady ||
      phase_ == BootstrapPhase::kFault) return;
  if (request_id_ == 0U) {
    advance(now_us);
    return;
  }
  try {
    const auto reply = client_.poll(request_id_);
    if (!reply) return;
    if (reply->rejected || (reply->result_valid && !reply->succeeded)) {
      phase_ = BootstrapPhase::kFault;
      error_ = "BOOTSTRAP_REJECTED";
      request_id_ = 0U;
      return;
    }
    if (reply->result_valid && reply->succeeded) {
      request_id_ = 0U;
      if (!automatic_) {
        phase_ = BootstrapPhase::kOperationComplete;
        return;
      }
      advance(now_us);
    }
  } catch (const std::exception &error) {
    const auto failed_phase=phase_;
    phase_ = BootstrapPhase::kFault;
    error_ = std::string("BOOTSTRAP_UNCONFIRMED phase=")+
      std::to_string(static_cast<unsigned>(failed_phase))+" error="+error.what();
    request_id_ = 0U;
  }
}

std::optional<Authorization> BootstrapCoordinator::authorization() const {
  if (!ready() && !(operation_complete() && last_operation_ == ManageOperation::kSetPermit)) return {};
  auto request = identity_;
  request.set_expected_boot_epoch(snapshot_.boot_epoch);
  request.set_expected_reference_epoch(snapshot_.reference_epoch);
  request.set_expected_motion_epoch(snapshot_.motion_epoch);
  request.set_expected_management_epoch(snapshot_.management_epoch);
  request.set_expected_profile_id(snapshot_.profile_id);
  request.set_freshness_ticket(snapshot_.freshness_ticket);
  request.set_configured_mask(snapshot_.configured_mask);
  return Authorization{request, true, true, true};
}
}  // namespace dummy_motion_client
