#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "dummy_motion_client/action_gate.hpp"

namespace dummy_motion_client {
enum class BootstrapPhase {
  kIdle, kSelectMode, kGrantSession, kCaptureReference, kSetPermit,
  kRenewLease, kOperationComplete, kReady, kFault,
  kReleasePending, kReleaseAwaitIdle, kReleased
};

struct BootstrapSnapshot {
  std::uint64_t boot_epoch = 0;
  std::uint64_t session_id = 0;
  std::uint32_t reference_epoch = 0;
  std::uint32_t motion_epoch = 0;
  std::uint32_t management_epoch = 0;
  std::uint64_t freshness_ticket = 0;
  std::uint32_t profile_id = 0;
  std::uint8_t configured_mask = 0;
  bool raw_ready = false;
  bool motion_supported = false;
  std::uint16_t control_mode = 0;
  std::uint16_t reference_kind = 0;
  bool real_motion_permitted = false;
  std::uint64_t permit_remaining_us = 0;
  std::uint16_t core_state = 0;
  std::uint16_t state_core_state = 0;
  std::array<std::uint8_t, 32> firmware_sha256{};
  std::array<std::uint8_t, 32> contract_sha256{};
  std::array<std::uint8_t, 32> model_sha256{};
  std::array<std::uint8_t, 32> profile_sha256{};
  std::uint64_t run_id = 0;
};

struct ConfirmedRelease {
  std::uint64_t request_id = 0;
  std::uint64_t operation_id = 0;
};

class BootstrapCoordinator {
 public:
  BootstrapCoordinator(ManagementClient &client, ManageRequest identity,
                       std::array<std::uint8_t, 32> compiled_policy_digest,
                       std::uint64_t permit_remaining_us,
                       std::uint64_t configured_session_lease_us = 0U);
  bool start(const BootstrapSnapshot &snapshot, std::uint64_t now_us);
  bool start_operation(const BootstrapSnapshot &snapshot, ManageOperation operation,
                       std::uint64_t now_us);
  bool start_renew_lease(const BootstrapSnapshot &snapshot,
                         std::uint64_t requested_lease_us, std::uint64_t now_us);
  // Available only when a configured session lease was supplied; uses Host-side
  // submit/commit bounds without changing the permit or command producer.
  bool renewal_due(std::uint64_t now_us, std::uint64_t requested_lease_us) const noexcept;
  bool start_release(const BootstrapSnapshot &snapshot, std::uint64_t now_us);
  void observe(const BootstrapSnapshot &snapshot, std::uint64_t now_us) noexcept;
  void invalidate(const char *reason) noexcept;
  void tick(std::uint64_t now_us);
  BootstrapPhase phase() const noexcept { return phase_; }
  bool ready() const noexcept { return phase_ == BootstrapPhase::kReady; }
  bool operation_complete() const noexcept { return phase_ == BootstrapPhase::kOperationComplete; }
  bool released() const noexcept { return phase_ == BootstrapPhase::kReleased; }
  std::optional<ConfirmedRelease> confirmed_release() const noexcept { return confirmed_release_; }
  ManageOperation last_operation() const noexcept { return last_operation_; }
  const char *error() const noexcept { return error_.c_str(); }
  std::optional<Authorization> authorization() const;

 private:
  bool issue(ManageOperation operation, std::uint64_t now_us);
  bool precondition_for_next() const noexcept;
  bool release_idle() const noexcept;
  void advance(std::uint64_t now_us);
  ManagementClient &client_;
  ManageRequest identity_{};
  std::array<std::uint8_t, 32> compiled_policy_digest_{};
  std::uint64_t permit_remaining_us_ = 0;
  std::uint64_t configured_session_lease_us_ = 0;
  std::uint64_t requested_lease_us_ = 0;
  std::uint64_t lease_submit_at_us_ = 0;
  std::uint64_t lease_confirmed_at_us_ = 0;
  std::uint64_t lease_guaranteed_until_us_ = 0;
  std::uint64_t lease_term_us_ = 0;
  std::uint64_t renewing_run_id_ = 0;
  std::uint32_t renewing_motion_epoch_ = 0;
  std::uint32_t renewing_reference_epoch_ = 0;
  std::uint64_t release_deadline_us_ = 0;
  BootstrapSnapshot snapshot_{};
  BootstrapPhase phase_ = BootstrapPhase::kIdle;
  std::uint64_t request_id_ = 0;
  std::optional<ConfirmedRelease> confirmed_release_;
  std::uint32_t issued_management_epoch_ = 0;
  std::string error_;
  ManageOperation last_operation_ = ManageOperation::kSelectMode;
  bool automatic_ = true;
};
}  // namespace dummy_motion_client
