#pragma once
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_ref/manual_reference_policy.hpp"
namespace dummy_motion_client {
struct ConfirmedManualStart {
  std::uint64_t boot_epoch{};
  std::uint32_t state_sequence{};
  std::array<std::uint8_t, 32> deployment_sha256{};
  bool operator_confirmed_double_limits{};
  // 仅由外层核实同boot已资格且释放的绝对参考后显式置true；不授权16重采。
  bool reuse_qualified_reference{};
};
enum class ManualReferencePhase {
  kReadOnly,
  kSelectMode,
  kGrantSession,
  kCaptureAbsolute,
  kRestQualification,
  kReady,
  kReleasing,
  kReleased,
  kFault
};
struct ManualReferenceOptions {
  std::array<std::uint8_t, 32> deployment_sha256{};
  Position manual_limit_position{}, rest_position{};
  std::uint64_t stationary_window_us = 200000;
  std::uint32_t minimum_samples = 5;
  double raw_span_rad = 0.00017453292519943296;
  double rest_span_rad = 0.00017453292519943296;
  double completion_tolerance_rad = 0.008726646259971648;
  // 实际REF位置域与固定策略速度交集；部署loader绑定真实模型。
  Position position_min{{-100, -100, -100, -100, -100, -100}},
      position_max{{100, 100, 100, 100, 100, 100}};
  Position rest_velocity_max{{0.03490658503988659, 0.03490658503988659,
                              0.03490658503988659, 0.03490658503988659,
                              0.03490658503988659, 0.03490658503988659}};
  double working_margin_rad = dummy_ref::host::kManualReferenceWorkingMarginRad,
         hold_tolerance_rad = 0.003;
  std::uint64_t operation_timeout_us = 25000000;
  std::uint64_t total_timeout_us = 90000000;
  std::uint64_t max_feedback_age_us = 500000;
};
// 借用唯一bootstrap；外部先bootstrap.observe/tick，再本协调器tick，不能双poll。
class ManualReferenceCoordinator {
public:
  ManualReferenceCoordinator(BootstrapCoordinator &, ManualReferenceOptions);
  bool start(const ConfirmedManualStart &, const BootstrapSnapshot &,
             std::uint64_t);
  void observe(const BootstrapSnapshot &, const Position &, std::uint32_t,
               std::uint64_t);
  void observe_raw(const BootstrapSnapshot &, const Position &, std::uint32_t,
                   std::uint64_t);
  void tick(std::uint64_t);
  void request_release(std::uint64_t);
  ManualReferencePhase phase() const noexcept { return phase_; }
  bool active() const noexcept {
    return phase_ != ManualReferencePhase::kReadOnly &&
           phase_ != ManualReferencePhase::kReleased &&
           phase_ != ManualReferencePhase::kFault;
  }
  std::optional<Authorization> authorization() const;
  std::uint32_t qualified_reference_epoch() const noexcept {
    return qualified_reference_epoch_;
  }
  const char *error() const noexcept { return error_.c_str(); }

private:
  void fail(const char *);
  bool issue(ManageOperation, std::uint64_t, bool = false);
  void sample(const BootstrapSnapshot &, const Position &, std::uint32_t,
              std::uint64_t, bool);
  bool window_ready() const noexcept;
  void clear_window() noexcept;
  bool freeze_rest_plan() noexcept;
  BootstrapCoordinator &bootstrap_;
  ManualReferenceOptions options_;
  BootstrapSnapshot snapshot_{}, binding_{};
  ManualReferencePhase phase_ = ManualReferencePhase::kReadOnly;
  ManageOperation operation_ = ManageOperation::kSelectMode;
  std::uint64_t deadline_ = 0, operation_deadline_ = 0, rest_anchor_ = 0,
                quiet_deadline_ = 0, last_received_ = 0;
  dummy_ref::host::RestSettleWindow window_{};
  std::uint32_t sequence_ = 0, reference_ = 0, motion_ = 0, retry_sequence_ = 0;
  std::uint32_t qualified_reference_epoch_ = 0;
  Position minimum_{}, maximum_{}, latest_position_{}, rest_target_{};
  std::uint64_t rest_duration_us_ = 0;
  bool waiting_ = false, confirmed_ = false, raw_window_ = true,
       release_requested_ = false;
  bool reuse_qualified_reference_ = false;
  std::string error_;
};
} // namespace dummy_motion_client
