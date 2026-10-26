#include "dummy_motion_client/manual_reference_coordinator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace dummy_motion_client {
ManualReferenceCoordinator::ManualReferenceCoordinator(
    BootstrapCoordinator &bootstrap, ManualReferenceOptions options)
    : bootstrap_(bootstrap), options_(options) {}
void ManualReferenceCoordinator::clear_window() noexcept { window_ = {}; }
bool ManualReferenceCoordinator::window_ready() const noexcept {
  return window_.ready(options_.stationary_window_us, options_.minimum_samples);
}
void ManualReferenceCoordinator::fail(const char *reason) {
  error_ = reason;
  phase_ = ManualReferencePhase::kFault;
  waiting_ = false;
  clear_window();
  // 不伪造停止完成；外层清理所有者收到Fault后必须执行有界Stop/D10。
}
bool ManualReferenceCoordinator::issue(ManageOperation operation,
                                       std::uint64_t now, bool confirmed) {
  if (!bootstrap_.start_operation(snapshot_, operation, now, confirmed, true)) {
    fail("MANUAL_SUBMIT_REJECTED");
    return false;
  }
  operation_ = operation;
  waiting_ = true;
  if (operation == ManageOperation::kValidateRest ||
      operation == ManageOperation::kCompleteQualification) {
    if (quiet_deadline_ == 0)
      quiet_deadline_ =
          now + std::min(options_.operation_timeout_us, deadline_ - now);
    operation_deadline_ = quiet_deadline_;
  } else
    operation_deadline_ =
        now + std::min(options_.operation_timeout_us, deadline_ - now);
  return true;
}
bool ManualReferenceCoordinator::start(const ConfirmedManualStart &confirm,
                                       const BootstrapSnapshot &snapshot,
                                       std::uint64_t now) {
  const bool reuse = confirm.reuse_qualified_reference;
  const bool allowed_reference =
      reuse
          ? snapshot.reference_epoch != 0 && snapshot.reference_kind == 1 &&
                snapshot.core_state == wire_value(WireCoreState::kIdle) &&
                snapshot.state_core_state == wire_value(WireCoreState::kIdle) &&
                snapshot.permit_remaining_us == 0
          : snapshot.reference_epoch == 0;
  if (phase_ != ManualReferencePhase::kReadOnly ||
      (!confirm.operator_confirmed_double_limits && !reuse) ||
      confirm.boot_epoch == 0 || confirm.boot_epoch != snapshot.boot_epoch ||
      confirm.state_sequence == 0 ||
      confirm.deployment_sha256 != options_.deployment_sha256 ||
      std::all_of(options_.deployment_sha256.begin(),
                  options_.deployment_sha256.end(),
                  [](auto v) { return v == 0; }) ||
      snapshot.session_id != 0 || !allowed_reference ||
      snapshot.configured_mask != 63 || !snapshot.raw_ready ||
      snapshot.real_motion_permitted || options_.minimum_samples < 2 ||
      options_.stationary_window_us == 0 || options_.max_feedback_age_us == 0 ||
      !std::isfinite(options_.raw_span_rad) || options_.raw_span_rad <= 0 ||
      !std::isfinite(options_.rest_span_rad) || options_.rest_span_rad <= 0 ||
      !std::isfinite(options_.completion_tolerance_rad) ||
      options_.completion_tolerance_rad <= 0 ||
      options_.operation_timeout_us == 0 || options_.total_timeout_us == 0 ||
      now > std::numeric_limits<std::uint64_t>::max() -
                options_.total_timeout_us ||
      options_.operation_timeout_us > options_.total_timeout_us)
    return false;
  for (unsigned i = 0; i < 6; ++i)
    if (!std::isfinite(options_.manual_limit_position[i]) ||
        !std::isfinite(options_.rest_position[i]) ||
        !std::isfinite(options_.position_min[i]) ||
        !std::isfinite(options_.position_max[i]) ||
        options_.position_min[i] >= options_.position_max[i] ||
        !std::isfinite(options_.rest_velocity_max[i]) ||
        options_.rest_velocity_max[i] <= 0)
      return false;
  if (!std::isfinite(options_.working_margin_rad) ||
      !std::isfinite(options_.hold_tolerance_rad) ||
      options_.working_margin_rad <= options_.hold_tolerance_rad ||
      options_.hold_tolerance_rad <= 0)
    return false;
  binding_ = snapshot_ = snapshot;
  confirmed_ = true;
  sequence_ = confirm.state_sequence;
  last_received_ = now;
  reuse_qualified_reference_ = reuse;
  if (reuse) {
    // 历史资格证据由外层确认；不重采绝对零点，保留原参考用于任意中途清理。
    reference_ = qualified_reference_epoch_ = snapshot.reference_epoch;
    raw_window_ = false;
  }
  deadline_ = now + options_.total_timeout_us;
  phase_ = ManualReferencePhase::kSelectMode;
  return issue(ManageOperation::kSelectMode, now);
}
void ManualReferenceCoordinator::observe(const BootstrapSnapshot &s,
                                         const Position &p, std::uint32_t seq,
                                         std::uint64_t at) {
  sample(s, p, seq, at, false);
}
void ManualReferenceCoordinator::observe_raw(const BootstrapSnapshot &s,
                                             const Position &p,
                                             std::uint32_t seq,
                                             std::uint64_t at) {
  sample(s, p, seq, at, true);
}
void ManualReferenceCoordinator::sample(const BootstrapSnapshot &s,
                                        const Position &p, std::uint32_t seq,
                                        std::uint64_t at, bool raw) {
  if (!active())
    return;
  if (s.boot_epoch != binding_.boot_epoch ||
      s.profile_id != binding_.profile_id ||
      s.firmware_sha256 != binding_.firmware_sha256 ||
      s.contract_sha256 != binding_.contract_sha256 ||
      s.model_sha256 != binding_.model_sha256 ||
      s.profile_sha256 != binding_.profile_sha256 || s.configured_mask != 63 ||
      !s.raw_ready ||
      (snapshot_.session_id != 0 && s.session_id != snapshot_.session_id &&
       phase_ != ManualReferencePhase::kReleasing) ||
      (reference_ != 0 &&
       (s.reference_epoch != reference_ ||
        (reuse_qualified_reference_ && s.reference_kind != 1)) &&
       phase_ != ManualReferencePhase::kReleasing) ||
      (rest_anchor_ != 0 && s.motion_epoch != motion_ &&
       phase_ != ManualReferencePhase::kReleasing)) {
    qualified_reference_epoch_ = 0;
    fail("MANUAL_IDENTITY_CHANGED");
    return;
  }
  snapshot_ = s;
  if (raw != raw_window_)
    return;
  if (seq == 0 || seq == sequence_ ||
      static_cast<std::int32_t>(seq - sequence_) <= 0 || at <= last_received_)
    return;
  sequence_ = seq;
  last_received_ = at;
  if (!raw)
    latest_position_ = p;
  bool valid = true;
  for (unsigned i = 0; i < 6; ++i) {
    valid = valid && std::isfinite(p[i]);
    // 复用参考下的17前静止窗不把当前姿态假定为双限位，也不假定已到REST。
    if (!raw && !(reuse_qualified_reference_ &&
                  phase_ == ManualReferencePhase::kCaptureAbsolute))
      valid = valid &&
              std::abs(p[i] - (phase_ == ManualReferencePhase::kCaptureAbsolute
                                   ? options_.manual_limit_position[i]
                                   : rest_target_[i])) <=
                  options_.completion_tolerance_rad;
  }
  if (!raw && reuse_qualified_reference_ &&
      phase_ == ManualReferencePhase::kCaptureAbsolute)
    valid = valid && s.core_state == wire_value(WireCoreState::kIdle) &&
            s.state_core_state == wire_value(WireCoreState::kIdle);
  if (!raw && phase_ == ManualReferencePhase::kRestQualification)
    valid = valid && s.core_state == wire_value(WireCoreState::kIdle) &&
            s.state_core_state == wire_value(WireCoreState::kIdle);
  window_.observe_position(
      seq, at, valid, p, raw ? options_.raw_span_rad : options_.rest_span_rad,
      options_.stationary_window_us, options_.minimum_samples, false);
}
void ManualReferenceCoordinator::tick(std::uint64_t now) {
  if (!active() || phase_ == ManualReferencePhase::kReady)
    return;
  if (now >= deadline_ || now < last_received_ ||
      now - last_received_ > options_.max_feedback_age_us) {
    fail("MANUAL_FEEDBACK_OR_TOTAL_TIMEOUT");
    return;
  }
  if (quiet_deadline_ != 0 && now >= quiet_deadline_ &&
      phase_ != ManualReferencePhase::kReleasing) {
    fail("MANUAL_QUIET_TIMEOUT");
    return;
  }
  if (bootstrap_.phase() == BootstrapPhase::kFault) {
    fail(bootstrap_.error());
    return;
  }
  if (phase_ == ManualReferencePhase::kReleasing) {
    if (bootstrap_.released())
      phase_ = ManualReferencePhase::kReleased;
    return;
  }
  if (waiting_) {
    if (now >= operation_deadline_) {
      fail("MANUAL_OPERATION_TIMEOUT");
      return;
    }
    if (!bootstrap_.operation_complete())
      return;
    waiting_ = false;
    if (bootstrap_.manual_policy_denied()) {
      retry_sequence_ = sequence_;
      return;
    }
    switch (operation_) {
    case ManageOperation::kSelectMode:
      if (snapshot_.control_mode == 0 || snapshot_.session_id != 0) {
        waiting_ = true;
        return;
      }
      phase_ = ManualReferencePhase::kGrantSession;
      issue(ManageOperation::kGrantSession, now);
      return;
    case ManageOperation::kGrantSession:
      if (snapshot_.session_id == 0) {
        waiting_ = true;
        return;
      }
      phase_ = ManualReferencePhase::kCaptureAbsolute;
      clear_window();
      return;
    case ManageOperation::kConfirmManualHome:
      if (snapshot_.reference_epoch == 0 || snapshot_.reference_kind != 1) {
        waiting_ = true;
        return;
      }
      reference_ = snapshot_.reference_epoch;
      raw_window_ = false;
      clear_window();
      return;
    case ManageOperation::kBeginQualification:
      motion_ = snapshot_.motion_epoch;
      rest_anchor_ = now;
      clear_window();
      return;
    case ManageOperation::kValidateRest:
      if (!window_ready())
        return;
      issue(ManageOperation::kCompleteQualification, now);
      return;
    case ManageOperation::kCompleteQualification:
      qualified_reference_epoch_ = reference_;
      if (!window_ready())
        return;
      issue(ManageOperation::kSetPermit, now);
      return;
    case ManageOperation::kSetPermit:
      if (bootstrap_.authorization())
        phase_ = ManualReferencePhase::kReady;
      return;
    default:
      fail("MANUAL_UNEXPECTED_OPERATION");
      return;
    }
  }
  if (phase_ == ManualReferencePhase::kCaptureAbsolute) {
    if (raw_window_) {
      if (window_ready())
        issue(ManageOperation::kConfirmManualHome, now, confirmed_);
      return;
    }
    if (snapshot_.reference_epoch == 0 || snapshot_.reference_kind != 1 ||
        !window_ready())
      return;
    // Native普通RESUME复用原资格与当前实测姿态；只有危险侧才重走退让链。
    // 余量沿用SDK策略和部署限位，不在ROS另定角度或重采绝对零点。
    bool ordinary_resume = reuse_qualified_reference_;
    for (unsigned axis = 0; axis < 6; ++axis) {
      const double q = latest_position_[axis];
      ordinary_resume = ordinary_resume && std::isfinite(q) &&
          q >= options_.position_min[axis] && q <= options_.position_max[axis] &&
          q >= options_.position_min[axis] + options_.working_margin_rad - options_.hold_tolerance_rad &&
          q <= options_.position_max[axis] - options_.working_margin_rad + options_.hold_tolerance_rad;
    }
    if (ordinary_resume) {
      issue(ManageOperation::kSetPermit, now);
      return;
    }
    if (!freeze_rest_plan()) {
      fail("MANUAL_REST_PLAN_INVALID");
      return;
    }
    reference_ = snapshot_.reference_epoch;
    phase_ = ManualReferencePhase::kRestQualification;
    clear_window();
    issue(ManageOperation::kBeginQualification, now);
    return;
  }
  if (phase_ == ManualReferencePhase::kRestQualification && window_ready() &&
      sequence_ != retry_sequence_) {
    if (operation_ == ManageOperation::kBeginQualification &&
        now >= rest_anchor_ && now - rest_anchor_ >= rest_duration_us_)
      issue(ManageOperation::kValidateRest, now, confirmed_);
    else if (operation_ == ManageOperation::kValidateRest) {
      if (bootstrap_.manual_policy_denied())
        issue(ManageOperation::kValidateRest, now, confirmed_);
      else
        issue(ManageOperation::kCompleteQualification, now);
    } else if (operation_ == ManageOperation::kCompleteQualification) {
      if (bootstrap_.manual_policy_denied())
        issue(ManageOperation::kCompleteQualification, now);
      else
        issue(ManageOperation::kSetPermit, now);
    }
  }
}
// 17前冻结实际目标。与REF
// qualification_rest_target相同：仅贴双限位轴退让，其他轴冻结。
bool ManualReferenceCoordinator::freeze_rest_plan() noexcept {
  dummy_ref::host::ManualReferencePolicy policy{};
  policy.lower = options_.position_min;
  policy.upper = options_.position_max;
  policy.rest = options_.rest_position;
  policy.working_margin_rad = options_.working_margin_rad;
  policy.hold_tolerance_rad = options_.hold_tolerance_rad;
  policy.velocity_max = options_.rest_velocity_max;
  // 已参考但不在普通工作区时，SDK只允许J2低侧/J3高侧的原受限重入。
  Position projected{};
  if (reuse_qualified_reference_ &&
      !dummy_ref::host::manual_reference_limit_rest_reentry(
          policy, latest_position_, &projected, &rest_target_))
    return false;
  return dummy_ref::host::manual_reference_rest_target(policy, latest_position_,
                                                       &rest_target_) &&
         dummy_ref::host::manual_reference_rest_plan_duration_us(
             policy, latest_position_, rest_target_, &rest_duration_us_);
}
void ManualReferenceCoordinator::request_release(std::uint64_t now) {
  if (bootstrap_.start_release(snapshot_, now, true,
                               qualified_reference_epoch_)) {
    phase_ = ManualReferencePhase::kReleasing;
    waiting_ = false;
  } else
    fail("MANUAL_RELEASE_REQUIRES_QUIET_STATE_AND_FREE_CLIENT");
}
std::optional<Authorization> ManualReferenceCoordinator::authorization() const {
  return phase_ == ManualReferencePhase::kReady
             ? bootstrap_.authorization()
             : std::optional<Authorization>{};
}
} // namespace dummy_motion_client
