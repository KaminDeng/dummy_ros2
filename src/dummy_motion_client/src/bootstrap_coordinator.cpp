#include "dummy_ros_control/diagnostic_scope.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"

#include <algorithm>
#include <cstdio>

namespace dummy_motion_client {
namespace {
// Preserve the 5 s confirmation reserve for both Hold and long RUN leases.
constexpr std::uint64_t kRenewLeadUs=25000000U;
constexpr std::uint64_t kRenewReserveUs=5000000U;
bool same_binding(const BootstrapSnapshot &left, const BootstrapSnapshot &right) noexcept {
  return left.boot_epoch == right.boot_epoch && left.profile_id == right.profile_id &&
    left.firmware_sha256 == right.firmware_sha256 &&
    left.contract_sha256 == right.contract_sha256 &&
    left.model_sha256 == right.model_sha256 &&
    left.profile_sha256 == right.profile_sha256;
}
}  // namespace
BootstrapCoordinator::BootstrapCoordinator(
    ManagementClient &client, ManageRequest identity,
    std::array<std::uint8_t, 32> compiled_policy_digest,
    std::uint64_t permit_remaining_us,
    std::uint64_t configured_session_lease_us)
    : client_(client), identity_(identity),
      compiled_policy_digest_(compiled_policy_digest),
      permit_remaining_us_(permit_remaining_us),
      configured_session_lease_us_(configured_session_lease_us) {}

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
  if (operation == ManageOperation::kRenewLease)
    request.set_requested_session_lease_us(requested_lease_us_);
  if (operation == ManageOperation::kSetPermit) {
    std::uint8_t digest[32]{};
    std::copy(compiled_policy_digest_.begin(), compiled_policy_digest_.end(), digest);
    request.set_compiled_policy_sha256(digest);
    request.set_permit_remaining_us(permit_remaining_us_);
    request.set_operator_confirmed(true);
  }
  issued_management_epoch_ = snapshot_.management_epoch;
  if (operation == ManageOperation::kSetPermit) {
    const PermitBinding binding{snapshot_.boot_epoch, snapshot_.reference_epoch, snapshot_.profile_id,
      snapshot_.firmware_sha256, snapshot_.contract_sha256, snapshot_.model_sha256,
      snapshot_.profile_sha256};
    request_id_ = client_.submit_permit(request, binding, now_us);
  } else request_id_ = client_.submit(request, now_us);
  if (request_id_ == 0U) {
    phase_ = BootstrapPhase::kFault;
    error_ = "BOOTSTRAP_TRANSPORT";
    return false;
  }
  if (operation == ManageOperation::kGrantSession || operation == ManageOperation::kRenewLease)
    lease_submit_at_us_=now_us;
  return true;
}

bool BootstrapCoordinator::start(const BootstrapSnapshot &snapshot, std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kOperationComplete) || snapshot.boot_epoch == 0U ||
      snapshot.profile_id == 0U || snapshot.configured_mask == 0U ||
      snapshot.freshness_ticket == 0U || !snapshot.raw_ready ||
      !snapshot.motion_supported || identity_.expected_session_id() == 0U ||
      permit_remaining_us_ == 0U ||
      std::all_of(compiled_policy_digest_.begin(), compiled_policy_digest_.end(), [](auto value) { return value == 0U; }))
    return false;
  snapshot_ = snapshot;
  confirmed_release_.reset();
  automatic_ = true;
  error_.clear();
  phase_ = BootstrapPhase::kSelectMode;
  return issue(ManageOperation::kSelectMode, now_us);
}

bool BootstrapCoordinator::start_operation(
    const BootstrapSnapshot &snapshot, ManageOperation operation, std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kOperationComplete &&
       !(phase_ == BootstrapPhase::kReady && operation == ManageOperation::kSetPermit)) ||
      snapshot.boot_epoch == 0U || snapshot.profile_id == 0U ||
      snapshot.configured_mask == 0U || snapshot.freshness_ticket == 0U ||
      !snapshot.raw_ready || !snapshot.motion_supported ||
      (phase_ != BootstrapPhase::kIdle && !same_binding(snapshot, snapshot_)) ||
      (operation == ManageOperation::kCaptureReference &&
       snapshot.session_id != identity_.expected_session_id()) ||
      (operation == ManageOperation::kSetPermit &&
       (snapshot.session_id != identity_.expected_session_id() || snapshot.reference_epoch == 0U))) return false;
  switch (operation) {
    case ManageOperation::kSelectMode: phase_ = BootstrapPhase::kSelectMode; break;
    case ManageOperation::kGrantSession: phase_ = BootstrapPhase::kGrantSession; break;
    case ManageOperation::kCaptureReference: phase_ = BootstrapPhase::kCaptureReference; break;
    case ManageOperation::kSetPermit: phase_ = BootstrapPhase::kSetPermit; break;
    default: return false;
  }
  snapshot_ = snapshot;
  if (operation == ManageOperation::kSelectMode || operation == ManageOperation::kGrantSession)
    confirmed_release_.reset();
  automatic_ = false;
  last_operation_ = operation;
  error_.clear();
  return issue(operation, now_us);
}

bool BootstrapCoordinator::start_renew_lease(
    const BootstrapSnapshot &snapshot, std::uint64_t requested_lease_us,
    std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kReady && phase_ != BootstrapPhase::kOperationComplete) ||
      requested_lease_us == 0U || requested_lease_us > 3600000000ULL ||
      (configured_session_lease_us_ != 0U && !renewal_due(now_us,requested_lease_us)) ||
      snapshot.boot_epoch == 0U || snapshot.session_id != identity_.expected_session_id() ||
      snapshot.reference_epoch == 0U || snapshot.profile_id == 0U ||
      snapshot.configured_mask == 0U || snapshot.freshness_ticket == 0U ||
      !same_binding(snapshot, snapshot_) ||
      (snapshot.core_state == wire_value(WireCoreState::kRunning) &&
       (snapshot.state_core_state != wire_value(WireCoreState::kRunning) ||
        snapshot.run_id == 0U)) ||
      !snapshot.raw_ready || !snapshot.motion_supported) return false;
  snapshot_ = snapshot;
  renewing_run_id_ = snapshot.core_state == wire_value(WireCoreState::kRunning)
      ? snapshot.run_id : 0U;
  renewing_motion_epoch_ = snapshot.motion_epoch;
  renewing_reference_epoch_ = snapshot.reference_epoch;
  requested_lease_us_ = requested_lease_us;
  automatic_ = false;
  last_operation_ = ManageOperation::kRenewLease;
  error_.clear();
  phase_ = BootstrapPhase::kRenewLease;
  return issue(ManageOperation::kRenewLease, now_us);
}

bool BootstrapCoordinator::renewal_due(std::uint64_t now_us,
                                      std::uint64_t requested_lease_us) const noexcept {
  if (configured_session_lease_us_ == 0U ||
      (phase_ != BootstrapPhase::kReady && phase_ != BootstrapPhase::kOperationComplete) ||
      lease_term_us_ == 0U || lease_guaranteed_until_us_ <= kRenewReserveUs ||
      requested_lease_us <= kRenewLeadUs ||
      requested_lease_us > configured_session_lease_us_ ||
      now_us > UINT64_MAX-requested_lease_us ||
      lease_confirmed_at_us_ > UINT64_MAX-lease_term_us_) return false;
  const auto safe_cutoff=lease_guaranteed_until_us_-kRenewReserveUs;
  const auto due_at=lease_guaranteed_until_us_ > kRenewLeadUs
      ? lease_guaranteed_until_us_-kRenewLeadUs : 0U;
  return now_us >= due_at && now_us < safe_cutoff &&
      now_us+requested_lease_us > lease_confirmed_at_us_+lease_term_us_;
}

bool BootstrapCoordinator::start_release(const BootstrapSnapshot &snapshot,
                                         std::uint64_t now_us) {
  if ((phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kReady &&
       phase_ != BootstrapPhase::kOperationComplete) ||
      (phase_ != BootstrapPhase::kIdle && !same_binding(snapshot, snapshot_)) ||
      snapshot.boot_epoch == 0U || snapshot.session_id != identity_.expected_session_id() ||
      snapshot.profile_id == 0U || snapshot.configured_mask == 0U ||
      snapshot.freshness_ticket == 0U || !snapshot.raw_ready || !snapshot.motion_supported ||
      snapshot.core_state != wire_value(WireCoreState::kHold) ||
      snapshot.state_core_state != wire_value(WireCoreState::kHold) ||
      now_us > UINT64_MAX - 5000000U) return false;
  snapshot_ = snapshot;
  confirmed_release_.reset();
  automatic_ = false;
  last_operation_ = ManageOperation::kDisableAndRelease;
  error_.clear();
  release_deadline_us_ = now_us + 5000000U;
  phase_ = BootstrapPhase::kReleasePending;
  return issue(ManageOperation::kDisableAndRelease, now_us);
}

bool BootstrapCoordinator::release_idle() const noexcept {
  return snapshot_.session_id == 0U && snapshot_.reference_epoch == 0U &&
      snapshot_.core_state == wire_value(WireCoreState::kUnreferencedIdle) &&
      snapshot_.state_core_state == wire_value(WireCoreState::kUnreferencedIdle) &&
      !snapshot_.real_motion_permitted && snapshot_.permit_remaining_us == 0U;
}

void BootstrapCoordinator::observe(const BootstrapSnapshot &snapshot, std::uint64_t now_us) noexcept {
  if (phase_ == BootstrapPhase::kIdle) {
    snapshot_ = snapshot;
    return;
  }
  if ((phase_ == BootstrapPhase::kReleasePending ||
       phase_ == BootstrapPhase::kReleaseAwaitIdle) && now_us >= release_deadline_us_) {
    if(auto* journal=dummy_ros_control::current_host_diagnostics) {
      const dummy_ros_control::DiagnosticIdentity observed{snapshot_.boot_epoch,snapshot_.session_id,snapshot_.reference_epoch,snapshot_.motion_epoch,0U,snapshot_.management_epoch,snapshot_.boot_epoch!=0U};
      const dummy_ros_control::DiagnosticIdentity expected{snapshot_.boot_epoch,identity_.expected_session_id(),snapshot_.reference_epoch,snapshot_.motion_epoch,0U,snapshot_.management_epoch,snapshot_.boot_epoch!=0U};
      journal->reject(dummy_ros_control::DiagnosticReason::kBootstrapReleaseTimeout,observed,expected,now_us,"RELEASE_TIMEOUT_DIAG",0U,{static_cast<unsigned>(phase_),request_id_,snapshot_.session_id,snapshot_.reference_epoch,snapshot_.core_state,snapshot_.state_core_state,snapshot_.permit_remaining_us,release_idle()?1U:0U,now_us-release_deadline_us_});
      journal->detail(
        "RELEASE_TIMEOUT_DIAG phase=%u request=%llu session=%llu ref=%u core=%u state_core=%u permit_us=%llu idle=%u overdue_us=%llu\n",
        static_cast<unsigned>(phase_), static_cast<unsigned long long>(request_id_),
        static_cast<unsigned long long>(snapshot_.session_id), snapshot_.reference_epoch,
        snapshot_.core_state, snapshot_.state_core_state,
        static_cast<unsigned long long>(snapshot_.permit_remaining_us),
        release_idle() ? 1U : 0U,
        static_cast<unsigned long long>(now_us-release_deadline_us_));
    } else std::fprintf(stderr,
        "RELEASE_TIMEOUT_DIAG phase=%u request=%llu session=%llu ref=%u core=%u state_core=%u permit_us=%llu idle=%u overdue_us=%llu\n",
        static_cast<unsigned>(phase_), static_cast<unsigned long long>(request_id_),
        static_cast<unsigned long long>(snapshot_.session_id), snapshot_.reference_epoch,
        snapshot_.core_state, snapshot_.state_core_state,
        static_cast<unsigned long long>(snapshot_.permit_remaining_us),
        release_idle() ? 1U : 0U,
        static_cast<unsigned long long>(now_us-release_deadline_us_));
    phase_ = BootstrapPhase::kFault;
    error_ = "RELEASE_UNCONFIRMED";
    confirmed_release_.reset();
    request_id_ = 0U;
    return;
  }
  const bool releasing = phase_ == BootstrapPhase::kReleasePending ||
      phase_ == BootstrapPhase::kReleaseAwaitIdle || phase_ == BootstrapPhase::kReleased;
  if (phase_ == BootstrapPhase::kRenewLease && renewing_run_id_ != 0U &&
      (snapshot.run_id != renewing_run_id_ ||
       snapshot.motion_epoch != renewing_motion_epoch_ ||
       snapshot.reference_epoch != renewing_reference_epoch_ ||
       snapshot.core_state != wire_value(WireCoreState::kRunning) ||
       snapshot.state_core_state != wire_value(WireCoreState::kRunning))) {
    phase_ = BootstrapPhase::kFault;
    error_ = "RUN_LEASE_BINDING_CHANGED";
    request_id_ = 0U;
    return;
  }
  if (!same_binding(snapshot, snapshot_) ||
      (snapshot_.session_id != 0U && snapshot.session_id != snapshot_.session_id &&
       !(releasing && snapshot.session_id == 0U)) ||
      snapshot.configured_mask != snapshot_.configured_mask ||
      (releasing && snapshot.reference_epoch != snapshot_.reference_epoch &&
       snapshot.reference_epoch != 0U) ||
      (phase_ == BootstrapPhase::kReleased &&
       (snapshot.session_id != 0U || snapshot.reference_epoch != 0U ||
        snapshot.core_state != wire_value(WireCoreState::kUnreferencedIdle) ||
        snapshot.state_core_state != wire_value(WireCoreState::kUnreferencedIdle) ||
        snapshot.real_motion_permitted || snapshot.permit_remaining_us != 0U))) {
    phase_ = BootstrapPhase::kFault;
    error_ = "IDENTITY_CHANGED";
    confirmed_release_.reset();
    request_id_ = 0U;
    return;
  }
  snapshot_ = snapshot;
  if (phase_ == BootstrapPhase::kReleaseAwaitIdle && release_idle())
    phase_ = BootstrapPhase::kReleased;
}

void BootstrapCoordinator::invalidate(const char *reason) noexcept {
  if (phase_ == BootstrapPhase::kIdle || phase_ == BootstrapPhase::kFault) return;
  phase_ = BootstrapPhase::kFault;
  error_ = reason;
  confirmed_release_.reset();
  request_id_ = 0U;
}

bool BootstrapCoordinator::precondition_for_next() const noexcept {
  if (snapshot_.management_epoch <= issued_management_epoch_) return false;
  switch (phase_) {
    case BootstrapPhase::kSelectMode:
      return snapshot_.session_id == 0U && snapshot_.control_mode != 0U;
    case BootstrapPhase::kCaptureReference:
      return snapshot_.session_id == identity_.expected_session_id() &&
          snapshot_.reference_epoch != 0U && snapshot_.reference_kind != 0U;
    case BootstrapPhase::kGrantSession:
      return snapshot_.session_id == identity_.expected_session_id() && snapshot_.control_mode != 0U;
    case BootstrapPhase::kSetPermit:
      return snapshot_.session_id == identity_.expected_session_id() && snapshot_.control_mode != 0U &&
          snapshot_.reference_epoch != 0U && snapshot_.reference_kind != 0U &&
          snapshot_.real_motion_permitted && snapshot_.permit_remaining_us != 0U;
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
    phase_ = BootstrapPhase::kSetPermit;
    issue(ManageOperation::kSetPermit, now_us);
  } else if (phase_ == BootstrapPhase::kSetPermit) {
    phase_ = BootstrapPhase::kReady;
  }
}

void BootstrapCoordinator::tick(std::uint64_t now_us) {
  if (lease_guaranteed_until_us_ > kRenewReserveUs &&
      now_us >= lease_guaranteed_until_us_-kRenewReserveUs &&
      phase_ != BootstrapPhase::kIdle && phase_ != BootstrapPhase::kFault &&
      phase_ != BootstrapPhase::kReleasePending &&
      phase_ != BootstrapPhase::kReleaseAwaitIdle && phase_ != BootstrapPhase::kReleased) {
    phase_ = BootstrapPhase::kFault;
    error_ = "LEASE_RENEWAL_UNCONFIRMED";
    request_id_ = 0U;
    return;
  }
  if (phase_ == BootstrapPhase::kIdle || phase_ == BootstrapPhase::kReady ||
      phase_ == BootstrapPhase::kReleased || phase_ == BootstrapPhase::kFault) return;
  if ((phase_ == BootstrapPhase::kReleasePending ||
       phase_ == BootstrapPhase::kReleaseAwaitIdle) && now_us >= release_deadline_us_) {
    phase_ = BootstrapPhase::kFault;
    error_ = "RELEASE_UNCONFIRMED";
    confirmed_release_.reset();
    request_id_ = 0U;
    return;
  }
  if (phase_ == BootstrapPhase::kReleaseAwaitIdle) return;
  if (request_id_ == 0U) {
    advance(now_us);
    return;
  }
  try {
    const auto reply = client_.poll(request_id_);
    if (!reply) return;
    if (phase_ == BootstrapPhase::kReleasePending &&
        (reply->result_valid || reply->rejected))
      {
if(auto* journal=dummy_ros_control::current_host_diagnostics) {
      const dummy_ros_control::DiagnosticIdentity observed{snapshot_.boot_epoch,snapshot_.session_id,snapshot_.reference_epoch,snapshot_.motion_epoch,0U,snapshot_.management_epoch,snapshot_.boot_epoch!=0U};
      const dummy_ros_control::DiagnosticIdentity expected{snapshot_.boot_epoch,identity_.expected_session_id(),snapshot_.reference_epoch,snapshot_.motion_epoch,0U,snapshot_.management_epoch,snapshot_.boot_epoch!=0U};
      journal->reject(dummy_ros_control::DiagnosticReason::kBootstrapReleaseReply,observed,expected,now_us,"RELEASE_REPLY_DIAG",0U,{request_id_,reply->request_id,reply->transaction_phase,reply->result_code,reply->result_valid?1U:0U,reply->succeeded?1U:0U,reply->rejected?1U:0U,reply->release_committed?1U:0U,reply->operation_id},!reply->succeeded || reply->rejected);
      journal->detail(
          "RELEASE_REPLY_DIAG request=%llu response=%llu phase=%u result=%u valid=%u success=%u rejected=%u release_committed=%u opid=%llu\n",
          static_cast<unsigned long long>(request_id_),
          static_cast<unsigned long long>(reply->request_id),
          reply->transaction_phase, reply->result_code,
          reply->result_valid ? 1U : 0U, reply->succeeded ? 1U : 0U,
          reply->rejected ? 1U : 0U, reply->release_committed ? 1U : 0U,
          static_cast<unsigned long long>(reply->operation_id));
    } else std::fprintf(stderr,
          "RELEASE_REPLY_DIAG request=%llu response=%llu phase=%u result=%u valid=%u success=%u rejected=%u release_committed=%u opid=%llu\n",
          static_cast<unsigned long long>(request_id_),
          static_cast<unsigned long long>(reply->request_id),
          reply->transaction_phase, reply->result_code,
          reply->result_valid ? 1U : 0U, reply->succeeded ? 1U : 0U,
          reply->rejected ? 1U : 0U, reply->release_committed ? 1U : 0U,
          static_cast<unsigned long long>(reply->operation_id));
    }
    const bool lease_operation=configured_session_lease_us_ != 0U &&
        (phase_ == BootstrapPhase::kGrantSession || phase_ == BootstrapPhase::kRenewLease);
    if (lease_operation && reply->request_id != request_id_) {
      phase_ = BootstrapPhase::kFault;
      error_ = "LEASE_RENEWAL_UNCONFIRMED";
      request_id_ = 0U;
      return;
    }
    if (phase_ == BootstrapPhase::kReleasePending && reply->result_valid && reply->succeeded &&
        (reply->request_id != request_id_ ||
         reply->transaction_phase != wire_value(WireTransactionPhase::kCommitted) ||
         reply->result_code != wire_value(WireResult::kOk) ||
         !reply->release_committed || reply->operation_id == 0U)) {
      phase_ = BootstrapPhase::kFault;
      error_ = "RELEASE_EVIDENCE_MISSING";
      confirmed_release_.reset();
      request_id_ = 0U;
      return;
    }
    if (reply->rejected || (reply->result_valid && !reply->succeeded)) {
      const auto failed_phase = phase_;
      phase_ = BootstrapPhase::kFault;
      confirmed_release_.reset();
      error_ = "BOOTSTRAP_REJECTED phase=" +
          std::to_string(static_cast<unsigned>(failed_phase)) +
          " result=" + std::to_string(reply->result_code) +
          " reason=" + std::to_string(reply->first_failure_reason);
      request_id_ = 0U;
      return;
    }
    if (reply->result_valid && reply->succeeded) {
      if (lease_operation &&
          (reply->transaction_phase != wire_value(WireTransactionPhase::kCommitted) ||
           reply->operation_id == 0U ||
           reply->result_code != wire_value(WireResult::kOk))) {
        phase_ = BootstrapPhase::kFault;
        error_ = "LEASE_RENEWAL_UNCONFIRMED";
        request_id_ = 0U;
        return;
      }
      if (configured_session_lease_us_ != 0U &&
          (phase_ == BootstrapPhase::kGrantSession || phase_ == BootstrapPhase::kRenewLease)) {
        lease_term_us_=phase_ == BootstrapPhase::kGrantSession
            ? configured_session_lease_us_ : requested_lease_us_;
        lease_confirmed_at_us_=now_us;
        if (lease_submit_at_us_ > UINT64_MAX-lease_term_us_) {
          phase_ = BootstrapPhase::kFault;
          error_ = "LEASE_DEADLINE_OVERFLOW";
          request_id_ = 0U;
          return;
        }
        lease_guaranteed_until_us_=lease_submit_at_us_+lease_term_us_;
      }
      if (phase_ == BootstrapPhase::kReleasePending) {
        confirmed_release_ = ConfirmedRelease{request_id_, reply->operation_id};
        request_id_ = 0U;
        phase_ = release_idle() ? BootstrapPhase::kReleased : BootstrapPhase::kReleaseAwaitIdle;
        return;
      }
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
    confirmed_release_.reset();
    error_ = std::string("BOOTSTRAP_UNCONFIRMED phase=")+
      std::to_string(static_cast<unsigned>(failed_phase))+" error="+error.what();
    request_id_ = 0U;
  }
}

std::optional<Authorization> BootstrapCoordinator::authorization() const {
  if (!ready() && !(operation_complete() && last_operation_ == ManageOperation::kSetPermit)) return {};
  if (snapshot_.session_id != identity_.expected_session_id() || snapshot_.control_mode == 0U ||
      snapshot_.reference_epoch == 0U || snapshot_.reference_kind == 0U ||
      !snapshot_.real_motion_permitted || snapshot_.permit_remaining_us == 0U) return {};
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
