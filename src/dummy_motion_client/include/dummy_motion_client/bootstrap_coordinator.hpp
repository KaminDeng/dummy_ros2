#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "dummy_motion_client/action_gate.hpp"

namespace dummy_motion_client {
enum class BootstrapPhase {
  kIdle, kSelectMode, kGrantSession, kCaptureReference, kSetPermit,
  kOperationComplete, kReady, kFault
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
};

class BootstrapCoordinator {
 public:
  BootstrapCoordinator(ManagementClient &client, ManageRequest identity,
                       std::array<std::uint8_t, 32> permit_digest,
                       std::array<std::uint8_t, 32> compiled_policy_digest,
                       std::uint64_t permit_remaining_us);
  bool start(const BootstrapSnapshot &snapshot, std::uint64_t now_us);
  bool start_operation(const BootstrapSnapshot &snapshot, ManageOperation operation,
                       std::uint64_t now_us);
  void observe(const BootstrapSnapshot &snapshot) noexcept;
  void tick(std::uint64_t now_us);
  BootstrapPhase phase() const noexcept { return phase_; }
  bool ready() const noexcept { return phase_ == BootstrapPhase::kReady; }
  bool operation_complete() const noexcept { return phase_ == BootstrapPhase::kOperationComplete; }
  ManageOperation last_operation() const noexcept { return last_operation_; }
  const char *error() const noexcept { return error_.c_str(); }
  std::optional<Authorization> authorization() const;

 private:
  bool issue(ManageOperation operation, std::uint64_t now_us);
  bool precondition_for_next() const noexcept;
  void advance(std::uint64_t now_us);
  bool advance_permit_digest() noexcept;
  ManagementClient &client_;
  ManageRequest identity_{};
  std::array<std::uint8_t, 32> permit_digest_{};
  std::array<std::uint8_t, 32> compiled_policy_digest_{};
  std::uint64_t permit_remaining_us_ = 0;
  BootstrapSnapshot snapshot_{};
  BootstrapPhase phase_ = BootstrapPhase::kIdle;
  std::uint64_t request_id_ = 0;
  std::uint32_t issued_management_epoch_ = 0;
  std::string error_;
  ManageOperation last_operation_ = ManageOperation::kSelectMode;
  bool automatic_ = true;
};
}  // namespace dummy_motion_client
