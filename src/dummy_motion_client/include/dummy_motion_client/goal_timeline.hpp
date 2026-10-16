#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dummy_motion_client {

struct TimelineIdentity {
  std::uint64_t boot=0, session=0;
  std::uint32_t reference_epoch=0, motion_epoch=0;
  std::uint64_t run_id=0;
};
inline bool trusted_begin_observation(const TimelineIdentity &before,
                                      const TimelineIdentity &after,
                                      bool state_running,bool begin_confirmed) noexcept {
  return state_running && begin_confirmed && before.boot!=0U && before.session!=0U &&
    before.boot==after.boot && before.session==after.session &&
    before.reference_epoch==after.reference_epoch && before.motion_epoch!=UINT32_MAX &&
    after.motion_epoch==before.motion_epoch+1U && after.run_id!=0U &&
    after.run_id>=before.run_id;
}

struct FaultCleanupTimelineProof {
  std::uint64_t boot=0,session=0,run=0;
  std::uint32_t attempt_epoch=0,active_epoch=0,stop_epoch=0;
  std::uint16_t failure_source=0;
  std::uint8_t axis_mask=0,final_state=0;
  std::array<std::uint8_t,32> terminal_chain{};
  bool full_groups_valid=false,chain_valid=false,release_committed=false;
  bool lost=false,motion_complete=false,current_fault_clear_verified=false;
};
struct TimelineEvent {
  enum class Kind { kAccepted, kBeforeBegin, kAfterBegin, kFeedback,
                    kTerminalSuccess, kTerminalCanceled, kTerminalAborted, kHold, kRelease, kFaultCleanup };
  Kind kind=Kind::kAccepted;
  FaultCleanupTimelineProof fault_cleanup{};
  std::array<std::uint8_t,16> uuid{};
  std::array<std::uint8_t,32> trajectory_sha256{};
  TimelineIdentity identity{};
  std::uint64_t steady_us=0;
  std::uint8_t axis_mask=0;
  std::uint16_t status_core_state=0, state_core_state=0;
  std::array<std::uint32_t,6> feedback_seq{}, feedback_age_us{};
  std::uint64_t release_request_id=0, release_operation_id=0;
  std::uint64_t cancel_request_id=0, cancel_operation_id=0;
};

enum class TimelineSuccessDecision { kWait, kSucceed, kEvidenceIncomplete };
inline TimelineSuccessDecision timeline_success_decision(bool timeline_configured,bool timeline_failed,
                                                         bool hold_recorded,bool writer_drained,
                                                         bool raw_configured=false,bool raw_failed=false,
                                                         bool raw_terminal_flushed=false) noexcept {
  if(timeline_failed || raw_failed) return TimelineSuccessDecision::kEvidenceIncomplete;
  if((timeline_configured && (!hold_recorded || !writer_drained)) ||
     (raw_configured && !raw_terminal_flushed)) return TimelineSuccessDecision::kWait;
  return TimelineSuccessDecision::kSucceed;
}

class GoalTimeline final {
 public:
  explicit GoalTimeline(const std::string &path, std::size_t capacity=2048);
  ~GoalTimeline();
  GoalTimeline(const GoalTimeline &)=delete;
  GoalTimeline &operator=(const GoalTimeline &)=delete;
  bool record(TimelineEvent event) noexcept;
  void close();
  bool complete() const noexcept { return complete_.load(); }
  bool failed() const noexcept { return failed_.load(); }
  bool drained() const noexcept {
    const auto tail=tail_.load(std::memory_order_acquire);
    return tail==head_.load(std::memory_order_acquire) &&
      !failed_.load(std::memory_order_acquire);
  }
 private:
  void write_loop();
  static bool same_authority(const TimelineIdentity &a,const TimelineIdentity &b) noexcept;
  std::FILE *output_=nullptr;
  const std::size_t capacity_;
  std::mutex wait_mutex_;
  std::condition_variable cv_;
  std::vector<TimelineEvent> ring_;
  std::atomic<std::size_t> head_{0}, tail_{0};
  std::thread writer_;
  std::atomic<bool> failed_{false}, complete_{false};
  std::atomic<bool> closing_{false};
  bool active_=false, before_=false, after_=false, feedback_=false;
  bool terminal_=false, release_recorded_=false, fault_cleanup_recorded_=false, terminal_aborted_=false;
  std::size_t goals_=0;
  TimelineEvent current_{};
  TimelineIdentity before_identity_{}, run_identity_{};
};

}  // namespace dummy_motion_client
