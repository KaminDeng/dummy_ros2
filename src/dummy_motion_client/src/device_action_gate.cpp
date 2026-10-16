#include "dummy_motion_client/bootstrap_tick_policy.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>
#include "controller_manager/controller_manager.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_motion_client/device_profile.hpp"
#include "dummy_motion_client/limited_goal_budget.hpp"
#include "dummy_motion_client/goal_timeline.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_motion_client/native_goal_validation.hpp"
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace dummy_motion_client {
namespace wire=dp::protocol::dummy_robot_v2;
using Follow=control_msgs::action::FollowJointTrajectory;
using GoalHandle=rclcpp_action::ServerGoalHandle<Follow>;
using Controller=dummy_ros_control::GuardedJointTrajectoryController;

bool verified_refresh_successor(const RawFirstPageWritten &page,
                                const TimelineIdentity &identity) noexcept {
  return page.strict_refresh_successor && page.chain_valid && page.full_six &&
    page.evidence_seq==0U && page.group_count>=1U && page.attempt_kind==1U &&
    page.category==1U && page.expected_mask==63U && page.submitted_mask==63U &&
    page.sent_mask==63U && page.failed_mask==0U && page.final &&
    !page.evidence_lost && raw_first_page_matches(
      page,identity.boot,identity.session,identity.run_id,identity.motion_epoch);
}

bool clean_cancel_terminal(const RawTerminalIdentity &terminal,
                           const TimelineIdentity &run) noexcept {
  return run.motion_epoch!=UINT32_MAX &&
    raw_terminal_matches(terminal,run.boot,run.session,run.run_id) &&
    terminal.chain_valid && terminal.attempt_kind==1U &&
    terminal.parent_run_id==0U && terminal.final_state==7U &&
    terminal.first_failure_source==0U &&
    terminal.first_failure_axis_mask==0U && !terminal.evidence_lost &&
    !terminal.motion_complete && !terminal.release_committed &&
    terminal.safety_hold_valid &&
    terminal.safety_hold_motion_epoch==run.motion_epoch+1U;
}

bool timeline_completed_refresh_hold_confirmed(
    const wire::DummyDeviceStatusV2 &status,const wire::DummyJointStateV2 &state,
    const TimelineIdentity &prior,const RawTerminalIdentity &terminal,
    const RawFirstPageWritten &successor,bool raw_capture_enabled,
    std::uint8_t raw_failure,std::uint8_t axis_mask) noexcept {
  if(!raw_capture_enabled || raw_failure!=0U || axis_mask!=63U ||
     prior.boot==0U || prior.session==0U || prior.reference_epoch==0U ||
     prior.run_id==0U || prior.run_id==UINT64_MAX ||
     prior.motion_epoch==0U || prior.motion_epoch==UINT32_MAX ||
     !raw_terminal_matches(terminal,prior.boot,prior.session,prior.run_id) ||
     !terminal.chain_valid || terminal.attempt_kind!=1U ||
     terminal.parent_run_id!=0U || terminal.final_state!=6U ||
     !terminal.motion_complete || terminal.evidence_lost ||
     terminal.release_committed || terminal.first_failure_source!=0U ||
     terminal.first_failure_axis_mask!=0U) return false;
  const TimelineIdentity next{status.boot_epoch(),status.session_id(),
      status.reference_epoch(),status.motion_epoch(),status.run_id()};
  return next.boot==prior.boot && next.session==prior.session &&
    next.reference_epoch==prior.reference_epoch &&
    next.run_id==prior.run_id+1U && next.motion_epoch==prior.motion_epoch+1U &&
    status.attempt_kind()==1U && status.parent_run_id()==0U &&
    status.final_state()==0U && !status.motion_complete() &&
    !status.evidence_lost() &&
    status.core_state()==wire_value(WireCoreState::kHold) &&
    state.boot_epoch()==next.boot && state.session_id()==next.session &&
    state.reference_epoch()==next.reference_epoch &&
    state.motion_epoch()==next.motion_epoch &&
    state.management_epoch()==status.management_epoch() &&
    state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
    state.can_submitted_mask()==axis_mask && state.can_sent_mask()==axis_mask &&
    state.can_failed_mask()==0U && verified_refresh_successor(successor,next);
}

bool timeline_cancel_hold_confirmed(const wire::DummyDeviceStatusV2 &status,
                                    const wire::DummyJointStateV2 &state,
                                    const TimelineIdentity &run,
                                    const RawTerminalIdentity &raw_terminal,
                                    std::uint8_t axis_mask) noexcept {
  return axis_mask==63U && run.motion_epoch!=UINT32_MAX &&
    status.boot_epoch()==run.boot && status.session_id()==run.session &&
    status.reference_epoch()==run.reference_epoch && status.run_id()==run.run_id &&
    status.motion_epoch()==run.motion_epoch+1U &&
    state.boot_epoch()==run.boot && state.session_id()==run.session &&
    state.reference_epoch()==run.reference_epoch &&
    state.motion_epoch()==status.motion_epoch() &&
    status.attempt_kind()==1U && status.parent_run_id()==0U &&
    status.final_state()==7U && !status.motion_complete() && !status.evidence_lost() &&
    status.core_state()==wire_value(WireCoreState::kHold) &&
    state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
    state.can_submitted_mask()==axis_mask && state.can_sent_mask()==axis_mask &&
    clean_cancel_terminal(raw_terminal,run);
}

TimelineSuccessDecision timeline_terminal_decision(
    Terminal terminal,bool timeline_configured,bool timeline_failed,
    bool hold_recorded,bool writer_drained,bool raw_failed,
    bool raw_terminal_flushed) noexcept {
  if(terminal==Terminal::kNone) return TimelineSuccessDecision::kWait;
  return timeline_success_decision(timeline_configured,timeline_failed,
    hold_recorded,writer_drained,true,raw_failed,raw_terminal_flushed);
}

std::string evidence_incomplete_result_error(const std::string &first_error) {
  if(first_error.empty() || first_error=="EVIDENCE_INCOMPLETE")
    return "EVIDENCE_INCOMPLETE";
  return first_error+";EVIDENCE_INCOMPLETE";
}

std::optional<TimelineIdentity> timeline_refresh_closure_identity(
    const TimelineIdentity &observed, const TimelineIdentity &confirmed_hold,
    bool prior_hold_confirmed, bool raw_terminal_flushed,
    bool successor_page_written, bool status_hold, bool state_hold,
    bool refresh_status) noexcept {
  if(!prior_hold_confirmed || !raw_terminal_flushed ||
     !successor_page_written || !status_hold ||
     !state_hold || !refresh_status || confirmed_hold.run_id==UINT64_MAX ||
     observed.run_id!=confirmed_hold.run_id+1U ||
     observed.boot!=confirmed_hold.boot || observed.session!=confirmed_hold.session ||
     observed.reference_epoch!=confirmed_hold.reference_epoch ||
     observed.motion_epoch!=confirmed_hold.motion_epoch) return std::nullopt;
  return confirmed_hold;
}

const char *refresh_closure_failure(const TimelineIdentity &observed,
    const TimelineIdentity &confirmed_hold,bool prior_hold_confirmed,
    bool raw_terminal_matched,bool successor_page_verified,
    bool status_hold,bool state_hold,bool refresh_status) noexcept {
  if(!prior_hold_confirmed) return "prior_hold";
  if(!raw_terminal_matched) return "raw_terminal";
  if(!successor_page_verified) return "successor_page";
  if(!status_hold) return "status_hold";
  if(!state_hold) return "state_hold";
  if(!refresh_status) return "refresh_status";
  if(confirmed_hold.run_id==UINT64_MAX || observed.run_id!=confirmed_hold.run_id+1U)
    return "run_id";
  if(observed.boot!=confirmed_hold.boot) return "boot";
  if(observed.session!=confirmed_hold.session) return "session";
  if(observed.reference_epoch!=confirmed_hold.reference_epoch) return "reference_epoch";
  if(observed.motion_epoch!=confirmed_hold.motion_epoch) return "motion_epoch";
  return "none";
}

const char *refresh_wait_failure(const TimelineIdentity &observed,
    const TimelineIdentity &confirmed_hold,bool prior_hold_confirmed,bool refresh_status,
    std::uint64_t first_seen_us,std::uint64_t now,std::uint64_t age_budget_us) noexcept {
  const char *identity_failure=refresh_closure_failure(observed,confirmed_hold,
    prior_hold_confirmed,true,true,true,true,refresh_status);
  if(std::string_view(identity_failure)!="none") return identity_failure;
  if(first_seen_us==0U) return "first_seen_missing";
  if(now<first_seen_us) return "clock_regressed";
  if(now-first_seen_us>=age_budget_us) return "age_budget";
  return "none";
}

bool timeline_refresh_wait_allowed(const TimelineIdentity &observed,
    const TimelineIdentity &confirmed_hold,bool prior_hold_confirmed,bool refresh_status,
    std::uint64_t first_seen_us,std::uint64_t now,std::uint64_t age_budget_us) noexcept {
  // Only a known refresh successor may wait for its independently delivered State.
  return timeline_refresh_closure_identity(observed,confirmed_hold,prior_hold_confirmed,
           true,true,true,true,refresh_status).has_value() && first_seen_us!=0U &&
    now>=first_seen_us && now-first_seen_us<age_budget_us;
}

bool queue_management_request(std::atomic<int> &request, const std::atomic<bool> &busy,
                              const std::atomic<bool> &renewal_reserved,
                              int operation, bool available) noexcept {
  int expected=0;
  return available && !busy.load() && !renewal_reserved.load() &&
    request.compare_exchange_strong(expected,operation);
}

struct PermitRefreshLatch {
  void begin(std::uint64_t boot, std::uint64_t session) noexcept {
    ++generation;
    request_boot=boot;
    request_session=session;
  }
  bool eligible(std::uint64_t boot, std::uint64_t session) const noexcept {
    return generation!=applied_generation && boot==request_boot &&
      session==request_session;
  }
  void applied() noexcept { applied_generation=generation; }

  std::uint64_t generation=0, applied_generation=0;
  std::uint64_t request_boot=0, request_session=0;
};

void clear_failed_lease_renewal(std::atomic<int> &request, std::atomic<bool> &busy,
                                std::atomic<bool> &reserved, std::atomic<bool> &started,
                                std::atomic<int> &explicit_status,
                                bool owns_busy=true) noexcept {
  int renewal=static_cast<int>(wire_value(ManageOperation::kRenewLease));
  request.compare_exchange_strong(renewal,0);
  explicit_status=0;
  started=false;
  reserved=false;
  if(owns_busy) busy=false;
}

struct RunLeaseEligibility {
  bool status_running=false, state_running=false, status_state_coherent=false;
  bool same_goal_run=false, producer_active=false, producer_recent=false;
  bool transport_ready=false;
};

bool run_lease_eligible(const RunLeaseEligibility &state) noexcept {
  return state.status_running && state.state_running &&
    state.status_state_coherent && state.same_goal_run &&
    state.producer_active && state.producer_recent && state.transport_ready;
}

RunLeaseEligibility run_lease_continuation_inputs(RunLeaseEligibility inputs,
    bool renewal_running,bool bounded_completion) noexcept {
  if(renewal_running && bounded_completion) inputs.producer_active=true;
  return inputs;
}

const char *run_lease_first_failed(const RunLeaseEligibility &state) noexcept {
  if(!state.status_running) return "status_running";
  if(!state.state_running) return "state_running";
  if(!state.status_state_coherent) return "status_state_coherent";
  if(!state.same_goal_run) return "same_goal_run";
  if(!state.producer_active) return "producer_active";
  if(!state.producer_recent) return "producer_recent";
  if(!state.transport_ready) return "transport_ready";
  return "none";
}

std::uint64_t run_lease_age_us(std::uint64_t now,std::uint64_t received_us) noexcept {
  return received_us!=0U && now>=received_us ? now-received_us : UINT64_MAX;
}

bool validated_requested_motion_permit(std::int64_t requested_us,
                                       const DeviceProfile &profile) noexcept {
  return requested_us > 0 &&
    static_cast<std::uint64_t>(requested_us) <= profile.motion_permit_max_us;
}

bool queue_lease_renewal(std::atomic<int> &request, std::atomic<bool> &busy,
                         std::atomic<bool> &reserved, bool running) noexcept {
  bool expected=false;
  if(!reserved.compare_exchange_strong(expected,true)) return false;
  if(running) {
    if(!busy.load()) { reserved=false; return false; }
  } else if(!busy.compare_exchange_strong(expected,true)) {
    reserved=false;
    return false;
  }
  int no_request=0;
  if(request.compare_exchange_strong(no_request,
       static_cast<int>(wire_value(ManageOperation::kRenewLease)))) return true;
  if(!running) busy=false;
  reserved=false;
  return false;
}

// 调用方持有同一control admission锁；只退尚未提交的RUN预约，不动goal busy。
bool withdraw_queued_running_renewal(std::atomic<int> &request,
    std::atomic<bool> &reserved, std::atomic<bool> &running,
    std::atomic<bool> &started, bool owns_busy, bool renewal_inflight,
    bool bounded_intent, std::atomic<int> &explicit_status) noexcept {
  if(!reserved.load() || !running.load() || started.load() || owns_busy ||
     renewal_inflight || !bounded_intent) return false;
  int renewal=static_cast<int>(wire_value(ManageOperation::kRenewLease));
  if(!request.compare_exchange_strong(renewal,0)) return false;
  reserved=false; running=false; started=false;
  int pending=1;
  explicit_status.compare_exchange_strong(pending,-1);
  return true;
}

bool take_withdrawn_lease_renewal(std::atomic<int> &status) noexcept {
  int withdrawn=-1;
  return status.compare_exchange_strong(withdrawn,0);
}

bool take_confirmed_lease_renewal(std::atomic<int> &explicit_status,
                                  bool status_available, bool operation_complete, bool hold,
                                  ManageOperation last_operation) noexcept {
  int confirmed=2;
  return status_available && operation_complete && hold &&
    last_operation==ManageOperation::kRenewLease &&
    explicit_status.compare_exchange_strong(confirmed,0);
}

const char *bootstrap_status_message(BootstrapPhase phase, const char *error,
                                     bool renewal_complete, bool enabled,
                                     bool gate_ready, bool operation_complete) noexcept {
  if(phase==BootstrapPhase::kFault) return error;
  if(renewal_complete) return "LEASE_RENEWED";
  if(enabled && gate_ready) return "ENABLED_HOLD";
  return operation_complete ? "OPERATION_COMPLETE" : "BOOTSTRAP_PENDING";
}

bool released_idle_observation(const wire::DummyDeviceStatusV2 &status,
                               const wire::DummyJointStateV2 &state) noexcept {
  return released_idle_status_confirmed(status,state);
}

bool bootstrap_authority_coherent(const wire::DummyDeviceStatusV2 &status,
                                  const wire::DummyJointStateV2 &state) noexcept {
  return state.boot_epoch()==status.boot_epoch() &&
    state.session_id()==status.session_id() &&
    state.reference_epoch()==status.reference_epoch() &&
    state.motion_epoch()==status.motion_epoch() &&
    state.management_epoch()==status.management_epoch();
}

bool refresh_observation_coherent(const wire::DummyDeviceStatusV2 &status,
                                  const wire::DummyJointStateV2 &state) noexcept {
  return bootstrap_authority_coherent(status,state);
}

bool timeline_cancel_refresh_hold_confirmed(
    const wire::DummyDeviceStatusV2 &status,
    const wire::DummyJointStateV2 &state,
    const TimelineIdentity &run,
    const RawTerminalIdentity &terminal,
    const RawFirstPageWritten &successor,
    const std::optional<CancelReceipt> &cancel_receipt,
    bool raw_capture_enabled,std::uint8_t raw_failure,
    std::uint8_t axis_mask) noexcept {
  if(!raw_capture_enabled || raw_failure!=0U || axis_mask!=63U ||
     !cancel_receipt || cancel_receipt->request_id==0U ||
     cancel_receipt->operation_id==0U || run.run_id==0U ||
     run.run_id==UINT64_MAX || run.motion_epoch==UINT32_MAX ||
     !clean_cancel_terminal(terminal,run)) return false;
  const TimelineIdentity next{status.boot_epoch(),status.session_id(),
      status.reference_epoch(),status.motion_epoch(),status.run_id()};
  return next.boot==run.boot && next.session==run.session &&
    next.reference_epoch==run.reference_epoch &&
    next.motion_epoch==run.motion_epoch+1U &&
    next.run_id==run.run_id+1U &&
    status.core_state()==wire_value(WireCoreState::kHold) &&
    status.attempt_kind()==1U && status.parent_run_id()==0U &&
    status.final_state()==0U && !status.motion_complete() &&
    !status.evidence_lost() &&
    refresh_observation_coherent(status,state) &&
    state.core_state()==wire_value(WireCoreState::kHold) &&
    state.hold_ready() && state.can_submitted_mask()==axis_mask &&
    state.can_sent_mask()==axis_mask &&
    verified_refresh_successor(successor,next);
}

struct ReleaseAvailability {
  bool releasing=false;
  bool status_fresh=false;
  bool management_epoch_matches=false;
  bool functional_development_basis=false;
  bool enabled=false;
  bool gate_ready=false;
  bool permit_refresh_allowed=false;
  bool busy=false;
  bool management_request_pending=false;
  bool bootstrap_ready=false;
  bool status_hold=false;
  bool state_hold=false;
  bool transport_ready=false;
};

bool release_operation_available(const ReleaseAvailability &state) noexcept {
  return !state.releasing && state.status_fresh && state.management_epoch_matches &&
    state.functional_development_basis && state.enabled &&
    (state.gate_ready || state.permit_refresh_allowed) &&
    !state.busy && !state.management_request_pending && state.bootstrap_ready &&
    state.status_hold && state.state_hold && state.transport_ready;
}

bool queue_disable_and_release(const std::atomic<bool> &available,
                               std::atomic<int> &request, std::atomic<bool> &busy,
                               std::atomic<int> &stage) noexcept {
  bool idle=false;
  if(!available.load() || request.load()!=0 || !busy.compare_exchange_strong(idle,true))
    return false;
  int inactive=0;
  int no_request=0;
  if(!available.load() || !stage.compare_exchange_strong(inactive,1) ||
     !request.compare_exchange_strong(
       no_request,static_cast<int>(wire_value(ManageOperation::kDisableAndRelease)))) {
    stage=0;
    busy=false;
    return false;
  }
  return true;
}

enum class GoalRejectionReason { kInvalid, kNotReady, kBusy };

struct AdmissionPrerequisites {
  bool enabled=false;
  bool observation_ready=false;
  bool gate_ready=false;
  bool port_ready=false;
  bool hold_tolerance_bound=false;
  bool transport_begin_ready=false;

  bool ready() const noexcept {
    return enabled && observation_ready && gate_ready && port_ready &&
      hold_tolerance_bound && transport_begin_ready;
  }
};

class GoalRejectionReceipt {
 public:
  using Clock=std::chrono::steady_clock;

  void record(const rclcpp_action::GoalUUID &uuid,GoalRejectionReason reason,
              bool renewal_reserved,bool admission_prerequisites,int release_stage,
              bool evidence_incomplete,bool fault,
              Clock::time_point now) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool eligible=reason==GoalRejectionReason::kNotReady && renewal_reserved &&
      admission_prerequisites &&
      release_stage==0 && !evidence_incomplete && !fault;
    if(!eligible) { latest_.reset(); return; }
    latest_=Entry{uuid,now};
  }

  std::optional<std::string> message(Clock::time_point now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!latest_ || now<latest_->at || now-latest_->at>std::chrono::seconds(2)) return {};
    static constexpr char digits[]="0123456789abcdef";
    std::string result="RENEWAL_PENDING:";
    result.reserve(result.size()+latest_->uuid.size()*2);
    for(const auto byte:latest_->uuid) {
      result.push_back(digits[byte>>4U]);
      result.push_back(digits[byte&0x0fU]);
    }
    return result;
  }

 private:
  struct Entry { rclcpp_action::GoalUUID uuid; Clock::time_point at; };
  mutable std::mutex mutex_;
  std::optional<Entry> latest_;
};

template<std::size_t Size> std::array<std::uint8_t,Size> hex_id(const std::string &text) {
  if(text.size()!=Size*2) throw std::invalid_argument("invalid hexadecimal identity length");
  std::array<std::uint8_t,Size> bytes{};
  for(std::size_t i=0;i<Size;++i) {
    unsigned value=0; const auto result=std::from_chars(text.data()+i*2,text.data()+i*2+2,value,16);
    if(result.ec!=std::errc{} || result.ptr!=text.data()+i*2+2) throw std::invalid_argument("invalid hexadecimal identity");
    bytes[i]=value;
  }
  return bytes;
}

template<class Handle,std::size_t Capacity=8U> class PendingGoalCancels {
 public:
  bool request(const std::shared_ptr<Handle> &goal) noexcept {
    if(!goal) return false;
    for(auto &slot:slots_) {
      unsigned empty=0U;
      if(!slot.state.compare_exchange_strong(empty,1U,std::memory_order_acquire)) continue;
      slot.goal=goal;
      slot.state.store(2U,std::memory_order_release);
      return true;
    }
    return false;
  }

  bool take_for(const std::shared_ptr<Handle> &active) noexcept {
    if(!active) return false;
    bool matched=false;
    for(auto &slot:slots_) {
      if(slot.state.load(std::memory_order_acquire)!=2U) continue;
      auto requested=std::move(slot.goal);
      slot.state.store(0U,std::memory_order_release);
      matched=matched || requested==active;
    }
    return matched;
  }

 private:
  struct Slot {
    std::atomic<unsigned> state{0U};
    std::shared_ptr<Handle> goal;
  };
  std::array<Slot,Capacity> slots_{};
};

template<class Handle> class GoalCancelAdmission {
 public:
  class Control {
   public:
    explicit Control(GoalCancelAdmission &owner) noexcept
        :owner_(owner),lock_(owner.mutex_,std::try_to_lock) {}
    explicit operator bool() const noexcept { return lock_.owns_lock(); }
    void open(const std::shared_ptr<Handle> &goal) noexcept { owner_.active_=goal; }
    void close() noexcept { owner_.active_.reset(); }

   private:
    GoalCancelAdmission &owner_;
    std::unique_lock<std::mutex> lock_;
  };

  Control try_control() noexcept { return Control(*this); }

  template<std::size_t Capacity>
  bool request(const std::shared_ptr<Handle> &goal,
               PendingGoalCancels<Handle,Capacity> &pending) {
    std::lock_guard<std::mutex> lock(mutex_);
    return goal && active_==goal && pending.request(goal);
  }

 private:
  std::mutex mutex_;
  std::shared_ptr<Handle> active_;
};

class GuardedPort final : public ActionPort {
 public:
  std::shared_ptr<Controller> controller;
  dummy_ros_control::CommandIdentity identity{};
  dummy_ros_control::GoalUuid uuid{};
  std::uint64_t motion_generation=0, started_us=0;
  bool executing=false, failed=false, publishing=false;
  bool direct_hold=false, direct_hold_started=false;
  unsigned refresh_diag_count=0U;
  std::size_t active_count=0;
  Trajectory trajectory;
  Position hold_position{};
  void prime(const Position &position,std::uint32_t epoch,std::uint64_t generation) override {
    failed=false; publishing=false;
    executing=false; identity.motion_epoch=epoch; motion_generation=generation; hold_position=position;
    dummy_ros_control::set_command_identity(identity);
    if(controller) {
      direct_hold=false;
      controller->set_update_sink(nullptr,nullptr);
      if(!controller->prime_position(position)) { failed=true; return; }
      controller->set_update_sink(dummy_ros_control::controller_update_sink,nullptr);
    } else {
      const auto published = dummy_ros_control::publish_control_update(position.data(),active_count,0,nullptr);
      std::fprintf(stderr, "GATE_PRIME_DIAG published=%llu boot=%llu session=%llu ref=%u motion=%u ticket=%llu\n",
                   static_cast<unsigned long long>(published),
                   static_cast<unsigned long long>(identity.boot),
                   static_cast<unsigned long long>(identity.session), identity.reference_epoch,
                   identity.motion_epoch, static_cast<unsigned long long>(identity.ticket));
      if(published==0U) { failed=true; return; }
      direct_hold=true; direct_hold_started=true;
    }
    publishing=true;
  }
  void refresh_direct_hold() {
    if(controller && controller->get_lifecycle_state().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      controller->set_update_sink(nullptr,nullptr);
      controller.reset();
      if(executing) { failed=true; publishing=false; direct_hold=false; }
      else if(publishing) {
        dummy_ros_control::set_command_identity(identity);
        direct_hold=true; direct_hold_started=true;
      }
    }
    if(direct_hold && publishing && !controller) {
      const auto published=dummy_ros_control::publish_control_update(
          hold_position.data(),active_count,0,nullptr);
      if(refresh_diag_count++<3U)
        std::fprintf(stderr,"GATE_HOLD_REFRESH_DIAG published=%llu motion=%u ticket=%llu\n",
                     static_cast<unsigned long long>(published),identity.motion_epoch,
                     static_cast<unsigned long long>(identity.ticket));
      if(published==0U) failed=true;
    }
  }
  void adopt_controller(std::shared_ptr<Controller> active) {
    if(controller || !active ||
       active->get_lifecycle_state().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    if(direct_hold) {
      if(!active->prime_position(hold_position)) { failed=true; return; }
      active->set_update_sink(dummy_ros_control::controller_update_sink,nullptr);
    }
    controller=std::move(active);
    direct_hold=false;
    failed=false;
  }
  bool control_ready() const {
    return publishing && !failed && (direct_hold ||
      (controller && controller->get_lifecycle_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  }
  void start(const Trajectory &value,std::uint64_t generation) override {
    auto message=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    message->joint_names=value.joints;
    for(const auto &point:value.points) {
      trajectory_msgs::msg::JointTrajectoryPoint item;
      item.positions=point.positions; item.velocities=point.velocities; item.accelerations=point.accelerations;
      item.time_from_start=rclcpp::Duration::from_seconds(point.time); message->points.push_back(std::move(item));
    }
    const dummy_ros_control::GoalCredential credential{uuid,dummy_ros_control::trajectory_digest(*message)};
    if(!controller || !controller->register_goal(credential) || !controller->submit_registered_goal(credential,message)) {
      failed=true; return;
    }
    trajectory=value; motion_generation=generation; started_us=management_now_us(); executing=true;
  }
  void disable() override {
    // Keep the last generation publishable while Complete waits for accepted_seq.
    // Called on the same control loop after update/write, so no new sample can race this freeze.
    if(controller) controller->set_update_sink(nullptr,nullptr);
    publishing=false; executing=false;
    direct_hold=false;
  }
  bool freeze_hold() override { disable(); return true; }
  void cancel() override {
    dummy_ros_control::clear_command_identity();
    if(controller) controller->cancel_execution();
  }
};

bool enable_gate_with_port_authority(ActionGate &gate, GuardedPort &port,
                                     const Authorization &authorization,
                                     std::uint64_t now) {
  const auto &identity=authorization.identity;
  if(identity.expected_boot_epoch()!=port.identity.boot ||
     identity.expected_session_id()!=port.identity.session ||
     identity.expected_reference_epoch()!=port.identity.reference_epoch ||
     !gate.enable(authorization,now)) return false;
  port.identity.motion_epoch=identity.expected_motion_epoch();
  return true;
}

class DeviceActionGate final : public rclcpp::Node {
 public:
  DeviceActionGate():Node("device_action_gate") {
    parameters_.endpoint=declare_parameter<std::string>("endpoint","");
    const auto device_id=declare_parameter<std::string>("device_id","");
    parameters_.device=hex_id<16>(device_id);
    parameters_.client=hex_id<16>(declare_parameter<std::string>("client_instance_id",""));
    const auto profile=declare_parameter<std::int64_t>("profile_id",0);
    const auto mask=declare_parameter<std::int64_t>("configured_mask",0);
    const auto session=declare_parameter<std::int64_t>("session_id",0);
    const auto permit_us=declare_parameter<std::int64_t>("permit_remaining_us",10000000);
    const auto renewal_us=declare_parameter<std::int64_t>("requested_session_lease_us",0);
    const auto state_max_age=declare_parameter<std::int64_t>("state_max_age_us",250000);
    if(profile<=0 || profile>UINT32_MAX || (mask!=1 && mask!=63) || session<=0 || permit_us<=0 ||
       renewal_us<0 || renewal_us>3600000000LL ||
       state_max_age<=0 || state_max_age>1000000)
      throw std::invalid_argument("profile/mask/session/permit duration required");
    state_max_age_us_=static_cast<std::uint64_t>(state_max_age);
    continuity_=ObservationContinuity(state_max_age_us_,2000000U,500000U);
    requested_lease_us_=static_cast<std::uint64_t>(renewal_us);
    parameters_.profile=profile; parameters_.mask=mask;
    controller_name_=declare_parameter<std::string>("guarded_controller","arm_controller");
    const auto bundle=declare_parameter<std::string>("contract_bundle","");
    const auto profile_name=declare_parameter<std::string>("profile_name","");
    if(bundle.empty() || profile_name.empty()) throw std::invalid_argument("contract bundle and profile_name are required");
    if(profile_name=="f405_6dof_bench") {
      NoLoadDevActivationRequest activation;
      for(const auto &[name,value] : std::initializer_list<std::pair<const char*,std::string*>>{
            {"activation_path",&activation.activation_path},
            {"activation_sha256",&activation.expected_activation_sha256},
            {"source_bundle_root",&activation.source_bundle_root},
            {"prebuild_bundle_manifest_path",&activation.prebuild_bundle_manifest_path},
            {"compiled_policy_manifest_path",&activation.compiled_policy_manifest_path},
            {"firmware_image_path",&activation.firmware_image_path},
            {"host_executable_path",&activation.host_executable_path},
            {"map_file_path",&activation.map_file_path},
            {"resource_report_path",&activation.resource_report_path},
            {"d_regression_report_path",&activation.d_regression_report_path},
            {"d_regression_provenance_path",&activation.d_regression_provenance_path},
            {"hil_report_path",&activation.hil_report_path},
            {"hil_raw_index_path",&activation.hil_raw_index_path},
            {"hil_evidence_root",&activation.hil_evidence_root},
            {"elf_path",&activation.elf_path},
            {"resource_base_root",&activation.resource_base_root},
            {"usb_serial",&activation.usb_serial}}) {
        *value=declare_parameter<std::string>(name,"");
        if(value->empty()) throw std::invalid_argument(std::string("NO_LOAD_DEV requires ")+name);
      }
      activation.device_id=device_id;
      activation.endpoint=parameters_.endpoint;
      device_profile_=load_device_profile(bundle,profile_name,parameters_.profile,parameters_.mask,activation);
      if(!device_profile_.functional_development_basis)
        throw std::invalid_argument("NO_LOAD_DEV_FUNCTIONAL basis required");
    } else {
      device_profile_=load_device_profile(bundle,profile_name,parameters_.profile,parameters_.mask);
    }
    if(!validated_requested_motion_permit(permit_us,device_profile_))
      throw std::invalid_argument("permit duration exceeds verified profile limit");
    profile_=device_profile_.joints;
    if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
       !limited_goal_budget_.configure(device_profile_.goal_limit,
         device_profile_.limited_boot_epoch,static_cast<std::uint64_t>(session)))
      throw std::invalid_argument("invalid limited functional goal budget");
    const auto timeline_path=declare_parameter<std::string>("goal_timeline_path","");
    if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
       timeline_path!=device_profile_.limited_goal_timeline_path)
      throw std::invalid_argument("limited functional invocation timeline mismatch");
    if(!timeline_path.empty()) {
      timeline_=std::make_unique<GoalTimeline>(timeline_path);
      if(timeline_->failed()) {
        if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited)
          throw std::invalid_argument("limited functional invocation already used or failed");
        evidence_incomplete_=true;
      }
    }
    evidence_failure_timer_=create_wall_timer(std::chrono::seconds(1),[this] {
      if(evidence_incomplete_.load() && !evidence_failure_logged_.exchange(true))
        RCLCPP_ERROR(get_logger(),"EVIDENCE_INCOMPLETE timeline_failed=%d raw_failure=%u",
          timeline_ && timeline_->failed(),
          transport_ ? static_cast<unsigned>(transport_->raw_evidence_failure()) : 0U);
    });
    port_.active_count=profile_.joints.size();
    transport_=std::make_unique<DpManagementTransport>(parameters_,DpManagementTransport::Connection::kDeviceWorker);
    if(!transport_->start()) throw std::invalid_argument("invalid device transport parameters");
    std::random_device random;
    const auto request_nonce=(static_cast<std::uint64_t>(random())<<32U) | random();
    client_=std::make_unique<WireManagementClient>(*transport_,static_cast<std::uint64_t>(session),
                                                   request_nonce);
    ManageRequest bootstrap_identity{};
    std::uint8_t device[16]{}, client_id[16]{};
    std::copy(parameters_.device.begin(),parameters_.device.end(),device);
    std::copy(parameters_.client.begin(),parameters_.client.end(),client_id);
    bootstrap_identity.set_device_id(device); bootstrap_identity.set_client_instance_id(client_id);
    bootstrap_identity.set_expected_session_id(static_cast<std::uint64_t>(session));
    bootstrap_=std::make_unique<BootstrapCoordinator>(
      *client_,bootstrap_identity,device_profile_.policy_hash,permit_us,
      device_profile_.functional_development_basis ? 30000000U : 0U);
    gate_=std::make_unique<ActionGate>(*client_,port_,profile_,2000000);
    enable_service_=create_service<std_srvs::srv::Trigger>("~/enable",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success=!evidence_incomplete_.load() && release_stage_.load()==0 && enable_available_.load() &&
          !enable_requested_.exchange(true);
        response->message=response->success ? "Enable queued; wait for confirmed HOLD before submitting motion" :
          "Enable unavailable phase="+std::to_string(static_cast<unsigned>(bootstrap_->phase()))+
          " transport="+(transport_->ready()?"ready":"not_ready")+
          " boot="+std::to_string(latest_bootstrap_.boot_epoch)+
          " raw="+(latest_bootstrap_.raw_ready?"ready":"not_ready");
      });
    bootstrap_status_service_=create_service<std_srvs::srv::Trigger>("~/bootstrap_status",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto admission=cancel_admission_.try_control();
        if(!admission) { response->success=false; response->message="BOOTSTRAP_STATUS_UNAVAILABLE"; return; }
        if(fault_cleanup_confirmed_.load() && !evidence_incomplete_.load() && transport_->raw_evidence_failure()==0U) {
          response->success=true;response->message="FAULT_CLEANUP_CONFIRMED";return;
        }
        const int release=release_stage_.load();
        if(evidence_incomplete_.load()) {
          response->success=false;
          response->message=release==3 && observation_ready_.load() ?
            "EVIDENCE_INCOMPLETE;RELEASED_IDLE" :
            (release==2 ? "EVIDENCE_INCOMPLETE;RELEASE_PENDING" :
             "EVIDENCE_INCOMPLETE;RELEASE_UNCONFIRMED");
          return;
        }
        if(release!=0) {
          response->success=release==3 && observation_ready_.load();
          response->message=release==3 ?
            (response->success ? "RELEASED_IDLE" : "RELEASE_OBSERVATION_UNAVAILABLE") :
            (release==4 ? "RELEASE_UNCONFIRMED" : "RELEASE_PENDING");
          return;
        }
        const bool operation_complete=management_request_.load()==0 && bootstrap_ && bootstrap_->operation_complete();
        const bool renewal_pending=renewal_reserved_.load();
        if(bootstrap_->phase()!=BootstrapPhase::kFault &&
           take_withdrawn_lease_renewal(explicit_renewal_status_)) {
          response->success=false; response->message="LEASE_RENEWAL_WITHDRAWN_NOT_SENT"; return;
        }
        const bool renewal_complete=take_confirmed_lease_renewal(
          explicit_renewal_status_,observation_ready_.load() && !renewal_pending &&
          bootstrap_->phase()!=BootstrapPhase::kFault,
          operation_complete,
          gate_->phase()==Phase::kHold || gate_->phase()==Phase::kRunning,
          bootstrap_->last_operation());
        response->success=observation_ready_ && !renewal_pending &&
          (renewal_complete || (enabled_ ? ready_.load() : operation_complete)) &&
          bootstrap_->phase()!=BootstrapPhase::kFault;
        if(renewal_pending) response->message="LEASE_RENEWAL_PENDING";
        else if(bootstrap_->phase()==BootstrapPhase::kFault)
          response->message=bootstrap_status_message(bootstrap_->phase(),bootstrap_->error(),
            false,enabled_,gate_->ready(),operation_complete);
        else if(renewal_complete) response->message="LEASE_RENEWED";
        else response->message=bootstrap_status_message(bootstrap_->phase(),bootstrap_->error(),
          false,enabled_,gate_->ready(),operation_complete);
      });
    last_goal_rejection_service_=create_service<std_srvs::srv::Trigger>("~/last_goal_rejection",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if(release_stage_.load()!=0 || evidence_incomplete_.load() ||
           bootstrap_fault_snapshot_.load()) {
          response->message="NO_RECENT_RENEWAL_REJECTION";
          return;
        }
        const auto message=goal_rejection_receipt_.message(GoalRejectionReceipt::Clock::now());
        response->success=message.has_value();
        response->message=message.value_or("NO_RECENT_RENEWAL_REJECTION");
      });
    const auto add_operation_service=[this](const char *name,ManageOperation operation) {
      management_services_.push_back(create_service<std_srvs::srv::Trigger>(name,
        [this,operation](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          const bool available=operation==ManageOperation::kSetPermit ?
            permit_operation_available_.load() : explicit_operation_available_.load();
          response->success=release_stage_.load()==0 && available &&
            queue_management_request(management_request_,busy_,renewal_reserved_,
                                     static_cast<int>(wire_value(operation)),available);
          response->message=response->success ? "Operation queued; poll bootstrap_status for final result" :
            "Operation unavailable or another management request is pending";
        }));
    };
    add_operation_service("~/select_control_mode",ManageOperation::kSelectMode);
    add_operation_service("~/grant_session",ManageOperation::kGrantSession);
    add_operation_service("~/capture_bench_reference",ManageOperation::kCaptureReference);
    add_operation_service("~/set_bench_permit",ManageOperation::kSetPermit);
    management_services_.push_back(create_service<std_srvs::srv::Trigger>("~/renew_session_lease",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto admission=cancel_admission_.try_control();
        response->success=admission && release_stage_.load()==0 && renew_available_.load() &&
          reserve_lease_renewal(true);
        response->message=response->success ? "Lease renewal queued; poll bootstrap_status" :
          "Lease renewal unavailable or another operation is active";
      }));
    management_services_.push_back(create_service<std_srvs::srv::Trigger>("~/enable_hold",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success=!evidence_incomplete_.load() && release_stage_.load()==0 &&
          single_enable_available_.load() &&
          !single_enable_requested_.exchange(true);
        response->message=response->success ? "EnableHold queued; poll bootstrap_status for confirmed HOLD" :
          "EnableHold requires a successfully completed SetBenchPermit";
      }));
    management_services_.push_back(create_service<std_srvs::srv::Trigger>("~/disable_and_release",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if(!queue_disable_and_release(release_available_,management_request_,busy_,release_stage_)) {
          response->message="RELEASE_UNAVAILABLE"; return;
        }
        admission_prerequisites_=false;
        ready_=false;
        response->success=false;
        response->message="RELEASE_QUEUED";
      }));
    server_=rclcpp_action::create_server<Follow>(this,"/arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const Follow::Goal> goal) {
        if(!validated_device_trajectory(*goal,profile_)) {
          goal_rejection_receipt_.record(uuid,GoalRejectionReason::kInvalid,
            renewal_reserved_.load(),false,release_stage_.load(),evidence_incomplete_.load(),
            bootstrap_fault_snapshot_.load(),GoalRejectionReceipt::Clock::now());
          RCLCPP_WARN(get_logger(),"DEVICE_GOAL_REJECTED_VALIDATION"); return rclcpp_action::GoalResponse::REJECT;
        }
        if(!ready_.load()) {
          goal_rejection_receipt_.record(uuid,GoalRejectionReason::kNotReady,
            renewal_reserved_.load(),admission_prerequisites_.load(),release_stage_.load(),
            evidence_incomplete_.load(),bootstrap_fault_snapshot_.load(),
            GoalRejectionReceipt::Clock::now());
          std::fprintf(stderr,"GATE_GOAL_REJECTED_DIAG reason=not_ready busy=%u observation=%u renewal=%u release=%d evidence=%u\n",
            busy_.load() ? 1U : 0U,observation_ready_.load() ? 1U : 0U,
            renewal_reserved_.load() ? 1U : 0U,release_stage_.load(),
            evidence_incomplete_.load() ? 1U : 0U);
          return rclcpp_action::GoalResponse::REJECT;
        }
        if(busy_.exchange(true)) {
          goal_rejection_receipt_.record(uuid,GoalRejectionReason::kBusy,
            renewal_reserved_.load(),false,release_stage_.load(),evidence_incomplete_.load(),
            bootstrap_fault_snapshot_.load(),GoalRejectionReceipt::Clock::now());
          std::fprintf(stderr,"GATE_GOAL_REJECTED_DIAG reason=busy\n");
          return rclcpp_action::GoalResponse::REJECT;
        }
        // 取得唯一busy后复核ready，阻止读旧ready的callback跨最后预算消费。
        if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited && !ready_.load()) {
          busy_=false;
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        return cancel_admission_.request(handle,cancel_requests_) ? rclcpp_action::CancelResponse::ACCEPT :
          rclcpp_action::CancelResponse::REJECT;
      },
      [this](std::shared_ptr<GoalHandle> handle) { std::lock_guard<std::mutex> lock(goal_mutex_); pending_=std::move(handle); });
  }
  ~DeviceActionGate() override { port_.disable(); port_.cancel(); transport_->stop(); if(timeline_) timeline_->close(); }
  void attach(controller_manager::ControllerManager &manager) {
    if(std::atomic_load(&discovered_controller_)) return;
    for(const auto &spec:manager.get_loaded_controllers()) {
      if(spec.info.name!=controller_name_) continue;
      auto controller=std::dynamic_pointer_cast<Controller>(spec.c);
      if(controller && controller->get_lifecycle_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        std::atomic_store(&discovered_controller_,std::move(controller));
    }
  }
  void cycle() noexcept {
    auto admission=cancel_admission_.try_control();
    if(!admission) return;
    try {
      cycle_impl(admission);
      bootstrap_fault_snapshot_=bootstrap_->phase()==BootstrapPhase::kFault;
    }
    catch(const std::exception &error) {
      RCLCPP_ERROR(get_logger(),"DEVICE_GATE_FAULT: %s",error.what());
      bootstrap_fault_snapshot_=true;
      admission_prerequisites_=false;
      gate_->abort("INTERNAL_BRIDGE_FAILURE",management_now_us()); ready_=false;
    }
    if(!active_ || gate_->terminal()!=Terminal::kNone) admission.close();
  }
  void prepare_update() noexcept {
    auto admission=cancel_admission_.try_control();
    if(!admission) return;
    try {
      const auto now=management_now_us();
      if(cancel_requests_.take_for(active_)) gate_->cancel(now);
      if(!hold_tolerance_bound_ && transport_->ready())
        hold_tolerance_bound_=transport_->set_post_complete_hold_tolerance(profile_.goal_tolerance);
      if((timeline_ && timeline_->failed()) || transport_->raw_evidence_failure()!=0U)
        evidence_incomplete_=true;
      if(evidence_incomplete_.load()) { admission_prerequisites_=false; ready_=false; }
      port_.adopt_controller(std::atomic_load(&discovered_controller_));
      if(port_.failed) { admission_prerequisites_=false; ready_=false; }
      const bool observation_ready=read_observation(now);
      if(!observation_ready) admission_prerequisites_=false;
      observation_ready_=observation_ready;
      if(renewal_running_.load() && !observation_ready)
        bootstrap_->invalidate("RUN_LEASE_PRODUCER_LOST");
      if(evidence_incomplete_.load()) {
        admission_prerequisites_=false;
        ready_=false;
        if(active_ && gate_->terminal()==Terminal::kNone)
          gate_->abort("EVIDENCE_INCOMPLETE",now);
      }
      if(!observation_ready_) {
        admission_prerequisites_=false;
        ready_=false; enable_available_=false; renew_available_=false;
        explicit_operation_available_=false; permit_operation_available_=false;
        single_enable_available_=false;
        if(observation_lost_) {
          if(bootstrap_) bootstrap_->invalidate("OBSERVATION_UNAVAILABLE");
          if(enabled_) { gate_->invalidate_authorization(now); port_.cancel(); }
        }
      }
      gate_->tick(now);
      if(bootstrap_ && bootstrap_tick_required(observation_ready_,observation_lost_,
          release_transition_pending_,release_stage_.load(),bootstrap_->phase(),bootstrap_->last_operation(),
          bootstrap_->release_request_inflight()))
        bootstrap_tick(now);
      const bool fault=bootstrap_->phase()==BootstrapPhase::kFault;
      bootstrap_fault_snapshot_=fault;
      if(fault) admission_prerequisites_=false;
    } catch(const std::exception &error) {
      RCLCPP_ERROR(get_logger(),"DEVICE_GATE_OBSERVATION: %s",error.what());
      bootstrap_fault_snapshot_=true;
      admission_prerequisites_=false;
      gate_->abort("INTERNAL_BRIDGE_FAILURE",management_now_us()); ready_=false;
    }
    if(!active_ || gate_->terminal()!=Terminal::kNone) admission.close();
  }
 private:
  bool reserve_lease_renewal(bool explicit_request=false) noexcept {
    if(explicit_request && explicit_renewal_status_.load()<0) return false;
    const bool running=run_renew_available_.load();
    if(!queue_lease_renewal(management_request_,busy_,renewal_reserved_,running)) return false;
    renewal_owns_busy_=!running;
    renewal_running_=running;
    renewal_started_=false;
    if(explicit_request) explicit_renewal_status_=1;
    else if(explicit_renewal_status_.load()>=0) explicit_renewal_status_=0;
    ready_=false;
    return true;
  }
  bool read_observation(std::uint64_t now) {
    observation_lost_=false;
    release_transition_pending_=false;
    const auto observed=transport_->observe();
    const auto wait_for_observation=[this,now](const DeviceObservation &value,bool coherent) {
      const auto result=continuity_.evaluate(value,now,enabled_,coherent);
      observation_lost_=result==ObservationContinuity::Result::kLost;
      return result==ObservationContinuity::Result::kReady;
    };
    if(!observed || observed->state.empty() || observed->status.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"DEVICE_BOOTSTRAP_INPUT_REJECTED_EMPTY");
      return wait_for_observation(observed.value_or(DeviceObservation{}),false);
    }
    wire::DummyDeviceStatusV2 status; wire::DummyJointStateV2 state;
    if(wire::DummyDeviceStatusV2Codec::decode(observed->status.data(),observed->status.size(),&status)!=wire::CodecResult::kOk ||
       wire::DummyJointStateV2Codec::decode(observed->state.data(),observed->state.size(),&state)!=wire::CodecResult::kOk) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"DEVICE_BOOTSTRAP_INPUT_REJECTED_CODEC");
      observation_lost_=true; return false;
    }
    if (active_ && gate_->terminal()==Terminal::kNone &&
        observed->rejected_ack_received_us>last_typed_reject_seen_us_ &&
        observed->rejected_ack_sequence!=0U &&
        observed->rejected_ack_boot_epoch==port_.identity.boot &&
        observed->rejected_ack_session_id==port_.identity.session &&
        observed->rejected_ack_reference_epoch==port_.identity.reference_epoch &&
        observed->rejected_ack_motion_epoch==port_.identity.motion_epoch &&
        observed->rejected_ack_received_us>=goal_accepted_us_) {
      last_typed_reject_seen_us_=observed->rejected_ack_received_us;
      gate_->abort("DEVICE_COMMAND_REJECTED_DURING_ACK",now);
    }
    const auto reject=[this](const char *reason) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"DEVICE_BOOTSTRAP_INPUT_REJECTED_%s",reason);
      observation_lost_=true;
      return false;
    };
    DeviceStatusIdentity status_identity{};
    std::copy(std::begin(status.contract_sha256()),std::end(status.contract_sha256()),status_identity.contract_hash.begin());
    std::copy(std::begin(status.model_sha256()),std::end(status.model_sha256()),status_identity.model_hash.begin());
    std::copy(std::begin(status.profile_sha256()),std::end(status.profile_sha256()),status_identity.profile_hash.begin());
    std::copy(std::begin(status.compiled_policy_sha256()),std::end(status.compiled_policy_sha256()),status_identity.policy_hash.begin());
    std::copy(std::begin(status.firmware_sha256()),std::end(status.firmware_sha256()),status_identity.firmware_hash.begin());
    if(!std::equal(parameters_.device.begin(),parameters_.device.end(),status.device_id()) ||
       status.profile_id()!=parameters_.profile || status.configured_mask()!=parameters_.mask ||
       !status_matches_bundle_identity(device_profile_,status_identity)) return reject("HASH");
    if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
       status.boot_epoch()!=device_profile_.limited_boot_epoch) return reject("LIMITED_BOOT");
    if(status.boot_epoch()==0 || !status.motion_supported() ||
       status.core_state()==wire_value(WireCoreState::kFault)) return reject("MOTION");
    if ((status.session_id()!=0U && status.control_mode()==0U) ||
       (status.reference_epoch()!=0U && status.reference_kind()==0U) ||
       (status.real_motion_permitted() != (status.permit_remaining_us()!=0U))) return reject("AUTHORITY");
    const bool releasing=release_stage_.load()!=0;
    if(enabled_ && (status.boot_epoch()!=port_.identity.boot ||
       (!releasing && (status.session_id()!=port_.identity.session ||
                       status.reference_epoch()!=port_.identity.reference_epoch))))
      return reject("AUTHORITY_CHANGED");
    if(latest_bootstrap_.boot_epoch!=0U && status.boot_epoch()!=latest_bootstrap_.boot_epoch)
      return reject("BOOT_CHANGED");
    if(state.boot_epoch()!=status.boot_epoch() || state.profile_id()!=parameters_.profile ||
       state.configured_mask()!=parameters_.mask ||
       !std::equal(parameters_.device.begin(),parameters_.device.end(),state.device_id()))
      return reject("STATE_IDENTITY");
    const bool state_fresh=observed->state_received_us!=0U &&
      (now<=observed->state_received_us || now-observed->state_received_us<=state_max_age_us_);
    const bool state_fault=state.core_state()==wire_value(WireCoreState::kFault);
    if(state_fresh && state_fault) return reject("STATE_FAULT");
    if(enabled_ && !releasing && state_fresh &&
       !state_authority_is_valid(
         {state.boot_epoch(),state.session_id(),state.reference_epoch(),state.motion_epoch()},
         {port_.identity.boot,port_.identity.session,port_.identity.reference_epoch,
          port_.identity.motion_epoch},state_fault,gate_->ready() || port_.executing))
      return reject("STATE_AUTHORITY_CHANGED");
    const bool released_idle=releasing && released_idle_observation(status,state);
    const bool release_status_fresh=observed->status_received_us!=0U &&
      (now<=observed->status_received_us ||
       now-observed->status_received_us<=state_max_age_us_);
    const auto transition=release_transition_origin_.observation(
      {status.boot_epoch(),status.session_id(),status.reference_epoch(),status.motion_epoch()},
      {state.boot_epoch(),state.session_id(),state.reference_epoch(),state.motion_epoch()},
      status.management_epoch(),state.management_epoch(),
      status.core_state(),state.core_state(),
      release_stage_.load());
    if (release_idle_transition_pending(transition,*observed,now,state_max_age_us_) &&
        !status.real_motion_permitted() && status.permit_remaining_us()==0U &&
        state.freshness_ticket()!=0U &&
        (state.raw_position_valid_mask() & parameters_.mask)==parameters_.mask) {
      release_transition_pending_=true;
      return false;
    }
    const bool unconfirmed_idle=releasing &&
      status.core_state()==wire_value(WireCoreState::kUnreferencedIdle) &&
      state.core_state()==wire_value(WireCoreState::kUnreferencedIdle) && !released_idle;
    const bool coherent=!unconfirmed_idle && (!releasing || release_status_fresh) &&
      (!releasing || state.management_epoch()==status.management_epoch()) &&
      state.freshness_ticket()!=0U &&
      (released_idle || (state.raw_position_valid_mask() & parameters_.mask)==parameters_.mask) &&
      (releasing ? (state.session_id()==status.session_id() &&
                    state.reference_epoch()==status.reference_epoch()) :
                   (enabled_ || bootstrap_authority_coherent(status,state)));
    if(!wait_for_observation(*observed,coherent)) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"DEVICE_BOOTSTRAP_INPUT_WAITING_FOR_COHERENT_OBSERVATION");
      return false;
    }
    if(enabled_ && !releasing) {
      port_.identity.boot=status.boot_epoch(); port_.identity.session=status.session_id();
      port_.identity.reference_epoch=status.reference_epoch();
      if(port_.identity.ticket!=state.freshness_ticket()) {
        port_.identity.ticket=state.freshness_ticket();
        if(port_.publishing) dummy_ros_control::set_command_identity(port_.identity);
      }
      gate_->observe_state(observed->state.data(),observed->state.size(),now);
      if(const auto worker=transport_->hold_drain_snapshot()) gate_->observe_hold_drain(*worker,now);
    }
    latest_bootstrap_={status.boot_epoch(),status.session_id(),status.reference_epoch(),state.motion_epoch(),
      state.management_epoch(),state.freshness_ticket(),parameters_.profile,parameters_.mask,true,
      status.motion_supported(),status.control_mode(),status.reference_kind(),status.real_motion_permitted(),
      status.permit_remaining_us(),status.core_state(),state.core_state()};
    latest_bootstrap_.run_id=status.run_id();
    std::copy(std::begin(status.firmware_sha256()),std::end(status.firmware_sha256()),latest_bootstrap_.firmware_sha256.begin());
    std::copy(std::begin(status.contract_sha256()),std::end(status.contract_sha256()),latest_bootstrap_.contract_sha256.begin());
    std::copy(std::begin(status.model_sha256()),std::end(status.model_sha256()),latest_bootstrap_.model_sha256.begin());
    std::copy(std::begin(status.profile_sha256()),std::end(status.profile_sha256()),latest_bootstrap_.profile_sha256.begin());
    timeline_identity_={status.boot_epoch(),status.session_id(),status.reference_epoch(),
                        state.motion_epoch(),status.run_id()};
    timeline_refresh_closure_.reset();
    const bool completed_hold=status.run_id()==timeline_run_id_ &&
      status.attempt_kind()==1U && status.parent_run_id()==0U &&
      status.final_state()==6U && status.motion_complete() &&
      !status.evidence_lost() &&
      status.core_state()==wire_value(WireCoreState::kHold) &&
      state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
      transport_->post_complete_begin_ready() &&
      (!transport_->raw_evidence_capture_enabled() ||
       raw_terminal_matches(transport_->raw_terminal_flushed(),
         status.boot_epoch(),status.session_id(),status.run_id()));
    const TimelineIdentity canceled_run{goal_boot_,goal_session_,
      goal_before_identity_.reference_epoch,timeline_run_motion_epoch_,timeline_run_id_};
    const auto cancel_terminal=transport_->raw_terminal_flushed();
    const auto cancel_successor=transport_->raw_first_page_written();
    const auto cancel_receipt=gate_->confirmed_cancel_receipt();
    const bool raw_capture=transport_->raw_evidence_capture_enabled();
    const auto raw_failure=transport_->raw_evidence_failure();
    const bool canceled_hold=timeline_cancel_hold_confirmed(status,state,
      {goal_boot_,goal_session_,timeline_identity_.reference_epoch,
       timeline_run_motion_epoch_,timeline_run_id_},
      cancel_terminal,parameters_.mask);
    const bool cancel_gate_ready=gate_->terminal()==Terminal::kCanceled &&
      active_ && active_->is_canceling() &&
      goal_run_id_!=0U && goal_run_id_==timeline_run_id_;
    const bool canceled_refresh_hold=cancel_gate_ready &&
      timeline_cancel_refresh_hold_confirmed(status,state,canceled_run,
        cancel_terminal,cancel_successor,cancel_receipt,raw_capture,
        raw_failure,parameters_.mask);
    const TimelineIdentity completed_run{goal_boot_,goal_session_,
      goal_before_identity_.reference_epoch,timeline_run_motion_epoch_,timeline_run_id_};
    const bool completed_refresh_hold=goal_run_id_!=0U &&
      goal_run_id_==timeline_run_id_ &&
      timeline_completed_refresh_hold_confirmed(status,state,completed_run,
        cancel_terminal,cancel_successor,raw_capture,raw_failure,parameters_.mask);
    if(!cancel_refresh_diagnostic_logged_ && timeline_ && timeline_goal_open_ &&
       timeline_after_ && timeline_run_id_!=UINT64_MAX &&
       goal_run_id_!=0U && goal_run_id_==timeline_run_id_ &&
       active_ && active_->is_canceling() &&
       status.run_id()==timeline_run_id_+1U &&
       status.core_state()==wire_value(WireCoreState::kHold) &&
       !canceled_refresh_hold) {
      cancel_refresh_diagnostic_logged_=true;
      std::fprintf(stderr,
        "CANCEL_REFRESH_DIAG run=%llu boot=%llu session=%llu ref=%u epoch=%u goal_run=%llu "
        "status_run=%llu status_boot=%llu status_session=%llu status_ref=%u status_epoch=%u "
        "status_core=%u status_kind=%u status_parent=%llu status_final=%u status_complete=%u status_lost=%u "
        "state_boot=%llu state_session=%llu state_ref=%u state_epoch=%u state_mgmt=%u "
        "status_mgmt=%u state_core=%u hold_ready=%u submitted=%u sent=%u "
        "terminal_run=%llu terminal_boot=%llu terminal_session=%llu terminal_kind=%u terminal_parent=%llu "
        "terminal_final=%u terminal_failure_source=%u terminal_failure_mask=%u terminal_lost=%u "
        "terminal_complete=%u terminal_release=%u terminal_chain=%u safety_valid=%u safety_epoch=%u terminal_ok=%u "
        "page_run=%llu page_boot=%llu page_session=%llu page_epoch=%u page_seq=%u page_count=%u "
        "page_kind=%u page_category=%u page_expected=%u page_submitted=%u page_sent=%u page_failed=%u "
        "page_final=%u page_lost=%u page_chain=%u page_six=%u page_strict=%u page_ok=%u "
        "receipt_request=%llu receipt_operation=%llu receipt_ok=%u raw_capture=%u raw_failure=%u "
        "status_fresh=%u state_fresh=%u coherent=%u status_received_us=%llu state_received_us=%llu now_us=%llu "
        "gate_terminal=%u gate_phase=%u active=%u active_canceling=%u gate_ready=%u axis_mask=%u\n",
        static_cast<unsigned long long>(canceled_run.run_id),
        static_cast<unsigned long long>(canceled_run.boot),
        static_cast<unsigned long long>(canceled_run.session),canceled_run.reference_epoch,
        canceled_run.motion_epoch,static_cast<unsigned long long>(goal_run_id_),
        static_cast<unsigned long long>(status.run_id()),
        static_cast<unsigned long long>(status.boot_epoch()),
        static_cast<unsigned long long>(status.session_id()),status.reference_epoch(),
        status.motion_epoch(),status.core_state(),status.attempt_kind(),
        static_cast<unsigned long long>(status.parent_run_id()),status.final_state(),
        status.motion_complete(),status.evidence_lost(),
        static_cast<unsigned long long>(state.boot_epoch()),
        static_cast<unsigned long long>(state.session_id()),state.reference_epoch(),
        state.motion_epoch(),state.management_epoch(),status.management_epoch(),
        state.core_state(),state.hold_ready(),state.can_submitted_mask(),state.can_sent_mask(),
        static_cast<unsigned long long>(cancel_terminal.run_id),
        static_cast<unsigned long long>(cancel_terminal.boot_epoch),
        static_cast<unsigned long long>(cancel_terminal.session_id),cancel_terminal.attempt_kind,
        static_cast<unsigned long long>(cancel_terminal.parent_run_id),cancel_terminal.final_state,
        cancel_terminal.first_failure_source,cancel_terminal.first_failure_axis_mask,
        cancel_terminal.evidence_lost,cancel_terminal.motion_complete,
        cancel_terminal.release_committed,cancel_terminal.chain_valid,
        cancel_terminal.safety_hold_valid,cancel_terminal.safety_hold_motion_epoch,
        clean_cancel_terminal(cancel_terminal,canceled_run),
        static_cast<unsigned long long>(cancel_successor.run_id),
        static_cast<unsigned long long>(cancel_successor.boot_epoch),
        static_cast<unsigned long long>(cancel_successor.session_id),
        cancel_successor.motion_epoch,cancel_successor.evidence_seq,
        cancel_successor.group_count,cancel_successor.attempt_kind,cancel_successor.category,
        cancel_successor.expected_mask,cancel_successor.submitted_mask,cancel_successor.sent_mask,
        cancel_successor.failed_mask,cancel_successor.final,cancel_successor.evidence_lost,
        cancel_successor.chain_valid,cancel_successor.full_six,
        cancel_successor.strict_refresh_successor,
        verified_refresh_successor(cancel_successor,
          {status.boot_epoch(),status.session_id(),status.reference_epoch(),
           status.motion_epoch(),status.run_id()}),
        static_cast<unsigned long long>(cancel_receipt ? cancel_receipt->request_id : 0U),
        static_cast<unsigned long long>(cancel_receipt ? cancel_receipt->operation_id : 0U),
        cancel_receipt.has_value(),raw_capture,raw_failure,release_status_fresh,state_fresh,
        refresh_observation_coherent(status,state),
        static_cast<unsigned long long>(observed->status_received_us),
        static_cast<unsigned long long>(observed->state_received_us),
        static_cast<unsigned long long>(now),static_cast<unsigned>(gate_->terminal()),
        static_cast<unsigned>(gate_->phase()),active_!=nullptr,
        active_ && active_->is_canceling(),cancel_gate_ready,parameters_.mask);
    }
    if(timeline_ && timeline_goal_open_ && timeline_after_ &&
       release_status_fresh && state_fresh &&
       refresh_observation_coherent(status,state) &&
       (completed_hold || completed_refresh_hold || canceled_hold || canceled_refresh_hold)) {
      timeline_goal_hold_confirmed_=true;
      timeline_goal_hold_identity_=(completed_refresh_hold || canceled_refresh_hold) ?
        TimelineIdentity{goal_boot_,goal_session_,status.reference_epoch(),
          status.motion_epoch(),timeline_run_id_} : timeline_identity_;
    }
    if(timeline_ && timeline_goal_open_ && timeline_after_ &&
       status.run_id()!=timeline_run_id_) {
      const bool raw_capture_enabled=transport_->raw_evidence_capture_enabled();
      const auto raw_terminal=transport_->raw_terminal_flushed();
      const auto successor_page=transport_->raw_first_page_written();
      const bool raw_terminal_matched=raw_capture_enabled &&
        raw_terminal_matches(raw_terminal,goal_boot_,goal_session_,goal_run_id_);
      const bool successor_page_verified=raw_capture_enabled &&
        verified_refresh_successor(successor_page,timeline_identity_);
      const bool status_hold=release_status_fresh &&
        status.core_state()==wire_value(WireCoreState::kHold);
      const bool observation_coherent=refresh_observation_coherent(status,state);
      const bool state_hold=state_fresh &&
        state.core_state()==wire_value(WireCoreState::kHold) &&
        state.hold_ready() && observation_coherent;
      const bool refresh_status=status.attempt_kind()==1U &&
        status.parent_run_id()==0U && status.final_state()==0U &&
        !status.evidence_lost();
      timeline_refresh_closure_=timeline_refresh_closure_identity(
        timeline_identity_,timeline_goal_hold_identity_,
        timeline_goal_hold_confirmed_,raw_terminal_matched,successor_page_verified,
        status_hold,state_hold,refresh_status);
      if(!timeline_refresh_closure_) {
        if(timeline_refresh_wait_started_us_==0U)
          timeline_refresh_wait_started_us_=observed->status_received_us;
        const TimelineIdentity candidate{status.boot_epoch(),status.session_id(),
          status.reference_epoch(),status.motion_epoch(),status.run_id()};
        const bool refresh_status=status.attempt_kind()==1U && status.parent_run_id()==0U &&
          status.final_state()==0U && !status.evidence_lost() &&
          status.core_state()==wire_value(WireCoreState::kHold);
        if(timeline_refresh_wait_allowed(candidate,timeline_goal_hold_identity_,
             timeline_goal_hold_confirmed_,refresh_status,timeline_refresh_wait_started_us_,
             now,state_max_age_us_)) {
          // Keep the completed Motion identity while the new Status awaits State.
          // Raw terminal and coherent Hold are still mandatory to close the timeline.
          timeline_identity_=timeline_goal_hold_identity_;
          timeline_hold_observed_=false;
          return false;
        }
        if(!evidence_incomplete_.exchange(true)) {
          const auto wait_age=now>=timeline_refresh_wait_started_us_ ?
            now-timeline_refresh_wait_started_us_ : 0U;
          std::fprintf(stderr,
            "REFRESH_CLOSURE_DIAG closure_failure=%s wait_failure=%s prior_hold=%u "
            "raw_capture=%u raw_terminal_matched=%u raw_failure=%u "
            "successor_page_present=%u successor_page_verified=%u page_run=%llu page_boot=%llu "
            "page_session=%llu page_epoch=%u page_seq=%u page_count=%u page_kind=%u "
            "page_category=%u page_expected=%u page_submitted=%u page_sent=%u page_failed=%u "
            "page_final=%u page_lost=%u page_strict=%u page_chain=%u page_full_six=%u "
            "status_fresh=%u state_fresh=%u coherent=%u status_core=%u state_core=%u "
            "status_hold=%u state_hold=%u refresh_status=%u status_kind=%u status_parent=%llu "
            "status_final=%u status_lost=%u status_group_count=%u state_hold_ready=%u "
            "status_run=%llu status_boot=%llu status_session=%llu status_ref=%u status_epoch=%u "
            "status_mgmt=%u state_boot=%llu state_session=%llu state_ref=%u state_epoch=%u "
            "state_mgmt=%u hold_run=%llu hold_boot=%llu hold_session=%llu hold_ref=%u hold_epoch=%u "
            "goal_run=%llu goal_boot=%llu goal_session=%llu terminal_run=%llu terminal_boot=%llu "
            "terminal_session=%llu status_received_us=%llu state_received_us=%llu "
            "first_seen_us=%llu now_us=%llu wait_age_us=%llu wait_budget_us=%llu\n",
            refresh_closure_failure(timeline_identity_,timeline_goal_hold_identity_,
              timeline_goal_hold_confirmed_,raw_terminal_matched,successor_page_verified,
              status_hold,state_hold,refresh_status),
            refresh_wait_failure(candidate,timeline_goal_hold_identity_,
              timeline_goal_hold_confirmed_,refresh_status,timeline_refresh_wait_started_us_,
              now,state_max_age_us_),
            timeline_goal_hold_confirmed_,raw_capture_enabled,raw_terminal_matched,
            static_cast<unsigned>(transport_->raw_evidence_failure()),
            successor_page.run_id!=0U,successor_page_verified,
            static_cast<unsigned long long>(successor_page.run_id),
            static_cast<unsigned long long>(successor_page.boot_epoch),
            static_cast<unsigned long long>(successor_page.session_id),
            successor_page.motion_epoch,successor_page.evidence_seq,successor_page.group_count,
            successor_page.attempt_kind,successor_page.category,successor_page.expected_mask,
            successor_page.submitted_mask,successor_page.sent_mask,successor_page.failed_mask,
            successor_page.final,successor_page.evidence_lost,
            successor_page.strict_refresh_successor,successor_page.chain_valid,
            successor_page.full_six,release_status_fresh,state_fresh,observation_coherent,
            status.core_state(),state.core_state(),
            status_hold,state_hold,refresh_status,status.attempt_kind(),
            static_cast<unsigned long long>(status.parent_run_id()),status.final_state(),
            status.evidence_lost(),status.group_count(),state.hold_ready(),
            static_cast<unsigned long long>(status.run_id()),
            static_cast<unsigned long long>(status.boot_epoch()),
            static_cast<unsigned long long>(status.session_id()),status.reference_epoch(),
            status.motion_epoch(),status.management_epoch(),
            static_cast<unsigned long long>(state.boot_epoch()),
            static_cast<unsigned long long>(state.session_id()),state.reference_epoch(),
            state.motion_epoch(),state.management_epoch(),
            static_cast<unsigned long long>(timeline_goal_hold_identity_.run_id),
            static_cast<unsigned long long>(timeline_goal_hold_identity_.boot),
            static_cast<unsigned long long>(timeline_goal_hold_identity_.session),
            timeline_goal_hold_identity_.reference_epoch,timeline_goal_hold_identity_.motion_epoch,
            static_cast<unsigned long long>(goal_run_id_),
            static_cast<unsigned long long>(goal_boot_),
            static_cast<unsigned long long>(goal_session_),
            static_cast<unsigned long long>(raw_terminal.run_id),
            static_cast<unsigned long long>(raw_terminal.boot_epoch),
            static_cast<unsigned long long>(raw_terminal.session_id),
            static_cast<unsigned long long>(observed->status_received_us),
            static_cast<unsigned long long>(observed->state_received_us),
            static_cast<unsigned long long>(timeline_refresh_wait_started_us_),
            static_cast<unsigned long long>(now),static_cast<unsigned long long>(wait_age),
            static_cast<unsigned long long>(state_max_age_us_));
        }
      } else timeline_refresh_wait_started_us_=0U;
    }
    const bool begin_observed=trusted_begin_observation(
      goal_before_identity_,timeline_identity_,
      state.core_state()==wire_value(WireCoreState::kRunning),
      gate_->phase()==Phase::kRunning);
    if(active_ && goal_run_id_==0U && begin_observed)
      goal_run_id_=timeline_identity_.run_id;
    timeline_status_core_state_=status.core_state();
    timeline_state_core_state_=state.core_state();
    timeline_hold_observed_=timeline_goal_hold_confirmed_ && state_fresh &&
      release_status_fresh &&
      refresh_observation_coherent(status,state) &&
      status.core_state()==wire_value(WireCoreState::kHold) &&
      state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
      (status.run_id()==timeline_run_id_ || timeline_refresh_closure_.has_value());
    if(timeline_ && timeline_goal_open_) {
      if(!timeline_after_ && begin_observed) {
        record_timeline(TimelineEvent::Kind::kAfterBegin,now);
        timeline_run_id_=timeline_identity_.run_id;
        timeline_run_motion_epoch_=timeline_identity_.motion_epoch;
        timeline_after_=true;
      }
      if(timeline_after_ && !timeline_terminal_ &&
         status.run_id()==timeline_run_id_ &&
         state.state_seq()!=timeline_feedback_state_seq_) {
        auto event=timeline_event(TimelineEvent::Kind::kFeedback,observed->state_received_us);
        event.feedback_seq=std::array<std::uint32_t,6>{};
        event.feedback_age_us=std::array<std::uint32_t,6>{};
        std::copy(std::begin(state.feedback_seq()),std::end(state.feedback_seq()),event.feedback_seq.begin());
        std::copy(std::begin(state.can_rx_age_us()),std::end(state.can_rx_age_us()),event.feedback_age_us.begin());
        if(!timeline_->record(event)) evidence_incomplete_=true;
        timeline_feedback_state_seq_=state.state_seq();
      }
      if(timeline_terminal_ && !timeline_hold_ && timeline_hold_observed_ &&
         (timeline_identity_.run_id==timeline_run_id_ || timeline_refresh_closure_) &&
         timeline_identity_.motion_epoch>=timeline_run_motion_epoch_) {
        record_timeline(TimelineEvent::Kind::kHold,now);
        timeline_hold_=true; timeline_goal_open_=false;
      }
    }
    if(timeline_ && release_stage_.load()==3 && !timeline_release_recorded_ &&
       released_idle_observation(status,state)) {
      const auto receipt=bootstrap_ ? bootstrap_->confirmed_release() : std::nullopt;
      if(receipt) {
        auto event=timeline_event(TimelineEvent::Kind::kRelease,now);
        event.release_request_id=receipt->request_id;
        event.release_operation_id=receipt->operation_id;
        timeline_release_recorded_=timeline_->record(event);
      }
      if(!timeline_release_recorded_) evidence_incomplete_=true;
    }
    if(bootstrap_) bootstrap_->observe(latest_bootstrap_,now);
    const bool port_control_ready=port_.control_ready();
    const auto gate_phase=gate_->phase();
    const auto gate_terminal=gate_->terminal();
    const bool transport_ready=transport_->ready();
    const bool authority_coherent=bootstrap_authority_coherent(status,state);
    const RunLeaseEligibility run_lease_inputs{
      enabled_ && status.core_state()==wire_value(WireCoreState::kRunning),
      state.core_state()==wire_value(WireCoreState::kRunning),
      authority_coherent,
      active_ && goal_run_id_!=0U && status.run_id()==goal_run_id_,
      port_.executing && port_control_ready && gate_phase==Phase::kRunning &&
        gate_terminal==Terminal::kNone,
      last_control_update_us_!=0U && now>=last_control_update_us_ &&
        now-last_control_update_us_<=state_max_age_us_,
      transport_ready && status.real_motion_permitted() &&
        state_fresh && release_status_fresh};
    // 完成冻结可与同轮RUN续租预约相遇；不为Completing授予新续租权限。
    ManageRequest completion_live;
    completion_live.set_operation(wire_value(ManageOperation::kComplete));
    completion_live.set_expected_boot_epoch(status.boot_epoch());
    completion_live.set_expected_session_id(status.session_id());
    completion_live.set_expected_reference_epoch(status.reference_epoch());
    completion_live.set_expected_motion_epoch(state.motion_epoch());
    completion_live.set_expected_management_epoch(state.management_epoch());
    completion_live.set_expected_profile_id(status.profile_id());
    completion_live.set_configured_mask(status.configured_mask());
    completion_live.set_device_id(status.device_id());
    std::uint8_t completion_client[16]{};
    std::copy(parameters_.client.begin(),parameters_.client.end(),completion_client);
    completion_live.set_client_instance_id(completion_client);
    const bool completion_ack=run_lease_inputs.status_running && run_lease_inputs.state_running &&
      run_lease_inputs.status_state_coherent && run_lease_inputs.same_goal_run &&
      run_lease_inputs.producer_recent && run_lease_inputs.transport_ready &&
      status.can_submitted_mask()==parameters_.mask && status.can_sent_mask()==parameters_.mask &&
      status.can_failed_mask()==0U && state.can_submitted_mask()==parameters_.mask &&
      state.can_sent_mask()==parameters_.mask && state.can_failed_mask()==0U &&
      state.position_valid_mask()==parameters_.mask && state.velocity_valid_mask()==parameters_.mask &&
      state.raw_position_valid_mask()==parameters_.mask &&
      gate_->bounded_completion_intent(completion_live,state.committed_producer_seq(),now);
    auto cancel_live=completion_live;
    cancel_live.set_operation(wire_value(ManageOperation::kCancel));
    const bool cancel_intent=run_lease_inputs.status_running && run_lease_inputs.state_running &&
      run_lease_inputs.status_state_coherent && run_lease_inputs.same_goal_run &&
      run_lease_inputs.producer_recent && run_lease_inputs.transport_ready &&
      gate_->bounded_cancel_intent(cancel_live,now);
    (void)withdraw_queued_running_renewal(management_request_,renewal_reserved_,renewal_running_,
      renewal_started_,renewal_owns_busy_.load(),bootstrap_->renewal_request_inflight(),
      completion_ack || cancel_intent,explicit_renewal_status_);
    // 已接纳续租期间，冻结末样本可有界替代producer_active，其他新鲜度/身份条件不变。
    const auto renewal_continuation_inputs=run_lease_continuation_inputs(run_lease_inputs,
        renewal_running_.load(),gate_->completion_waiting_for_renewal(now) ||
          gate_->cancel_waiting_for_renewal(cancel_live,now));
    const bool run_lease_active=run_lease_eligible(renewal_continuation_inputs);
    // 新续租仍仅允许原始Running生产条件，不由completion defer开启。
    run_renew_available_=run_lease_eligible(run_lease_inputs);
    if(renewal_running_.load() && !run_lease_active) {
      if(!run_lease_diag_reported_) {
        run_lease_diag_reported_=true;
        std::fprintf(stderr,
          "RUN_LEASE_DIAG first_false=%s now_us=%llu age_budget_us=%llu "
          "status_running=%d state_running=%d status_state_coherent=%d same_goal_run=%d "
          "producer_active=%d producer_recent=%d transport_ready_input=%d "
          "last_control_update_age_us=%llu status_age_us=%llu state_age_us=%llu "
          "last_control_update_us=%llu status_received_us=%llu state_received_us=%llu "
          "enabled=%d status_core=%u state_core=%u authority_coherent=%d "
          "active=%d goal_run=%llu status_run=%llu "
          "status_boot=%llu state_boot=%llu status_session=%llu state_session=%llu "
          "status_ref=%u state_ref=%u status_motion=%u state_motion=%u "
          "status_mgmt=%u state_mgmt=%u "
          "port_executing=%d port_control_ready=%d gate_phase=%u gate_terminal=%u "
          "transport_ready=%d real_motion_permitted=%d status_fresh=%d state_fresh=%d\n",
          run_lease_first_failed(run_lease_inputs),
          static_cast<unsigned long long>(now),
          static_cast<unsigned long long>(state_max_age_us_),
          run_lease_inputs.status_running,run_lease_inputs.state_running,
          run_lease_inputs.status_state_coherent,run_lease_inputs.same_goal_run,
          run_lease_inputs.producer_active,run_lease_inputs.producer_recent,
          run_lease_inputs.transport_ready,
          static_cast<unsigned long long>(run_lease_age_us(now,last_control_update_us_)),
          static_cast<unsigned long long>(run_lease_age_us(now,observed->status_received_us)),
          static_cast<unsigned long long>(run_lease_age_us(now,observed->state_received_us)),
          static_cast<unsigned long long>(last_control_update_us_),
          static_cast<unsigned long long>(observed->status_received_us),
          static_cast<unsigned long long>(observed->state_received_us),
          enabled_,static_cast<unsigned>(status.core_state()),
          static_cast<unsigned>(state.core_state()),authority_coherent,
          active_!=nullptr,static_cast<unsigned long long>(goal_run_id_),
          static_cast<unsigned long long>(status.run_id()),
          static_cast<unsigned long long>(status.boot_epoch()),
          static_cast<unsigned long long>(state.boot_epoch()),
          static_cast<unsigned long long>(status.session_id()),
          static_cast<unsigned long long>(state.session_id()),
          static_cast<unsigned>(status.reference_epoch()),
          static_cast<unsigned>(state.reference_epoch()),
          static_cast<unsigned>(status.motion_epoch()),
          static_cast<unsigned>(state.motion_epoch()),
          static_cast<unsigned>(status.management_epoch()),
          static_cast<unsigned>(state.management_epoch()),
          port_.executing,port_control_ready,static_cast<unsigned>(gate_phase),
          static_cast<unsigned>(gate_terminal),transport_ready,
          status.real_motion_permitted(),release_status_fresh,state_fresh);
      }
      bootstrap_->invalidate("RUN_LEASE_PRODUCER_LOST");
    }
    const int requested=management_request_.exchange(0);
    if(requested!=0) {
      explicit_operation_available_=false;
      const auto operation=static_cast<ManageOperation>(requested);
      int queued_release=1;
      if(operation!=ManageOperation::kDisableAndRelease ||
         release_stage_.compare_exchange_strong(queued_release,2)) {
        if(operation==ManageOperation::kDisableAndRelease) {
          port_.disable(); port_.cancel();
        }
        const bool renewal_binding_ready=!renewal_running_.load() || run_lease_active;
        const bool started=operation==ManageOperation::kDisableAndRelease ?
          begin_release(now) :
          operation==ManageOperation::kRenewLease ?
            (renewal_binding_ready && bootstrap_->start_renew_lease(
              latest_bootstrap_,requested_lease_us_,now)) :
            bootstrap_->start_operation(latest_bootstrap_,operation,now);
        if(!started) {
          RCLCPP_WARN(get_logger(),"EXPLICIT_MANAGEMENT_OPERATION_REJECTED operation=%d",requested);
          if(operation==ManageOperation::kRenewLease) {
            bootstrap_->invalidate("LEASE_RENEWAL_REJECTED");
            clear_failed_lease_renewal(management_request_,busy_,renewal_reserved_,
                                       renewal_started_,explicit_renewal_status_,
                                       renewal_owns_busy_.load());
            renewal_running_=false;
            renew_available_=false; admission_prerequisites_=false; ready_=false;
          }
          if(operation==ManageOperation::kDisableAndRelease) release_stage_=4;
        } else if(operation==ManageOperation::kRenewLease) renewal_started_=true;
        else if(operation==ManageOperation::kSetPermit)
          permit_refresh_latch_.begin(latest_bootstrap_.boot_epoch,latest_bootstrap_.session_id);
      }
    }
    port_.identity.boot=status.boot_epoch(); port_.identity.session=status.session_id();
    port_.identity.reference_epoch=status.reference_epoch();
    if(port_.identity.ticket!=state.freshness_ticket()) {
      port_.identity.ticket=state.freshness_ticket();
      if(port_.publishing) dummy_ros_control::set_command_identity(port_.identity);
    }
    enable_available_=!releasing && !enabled_ && bootstrap_ && bootstrap_->phase()==BootstrapPhase::kIdle && transport_->ready();
    explicit_operation_available_=!releasing && !enabled_ && bootstrap_ &&
      (bootstrap_->phase()==BootstrapPhase::kIdle || bootstrap_->operation_complete()) && transport_->ready();
    permit_operation_available_=!releasing && (explicit_operation_available_.load() ||
      (enabled_ && gate_->permit_refresh_allowed() && bootstrap_ &&
       (bootstrap_->ready() || bootstrap_->operation_complete()) && transport_->ready()));
    single_enable_available_=!releasing && !enabled_ && bootstrap_ && bootstrap_->operation_complete() &&
      bootstrap_->last_operation()==ManageOperation::kSetPermit && transport_->ready();
    if(enable_available_ && enable_requested_.exchange(false)) bootstrap_->start(latest_bootstrap_,now);
    const bool lease_hold=enabled_ &&
      status.core_state()==wire_value(WireCoreState::kHold) &&
      state.core_state()==wire_value(WireCoreState::kHold) && state.hold_ready() &&
      status.session_id()==state.session_id() &&
      status.reference_epoch()==state.reference_epoch() &&
      status.motion_epoch()==state.motion_epoch() &&
      status.management_epoch()==state.management_epoch();
    renew_available_=!releasing && !evidence_incomplete_.load() &&
      requested_lease_us_!=0U &&
      ((lease_hold && release_status_fresh &&
        transport_->post_complete_begin_ready() &&
        (gate_->ready() || gate_->permit_refresh_allowed()) && !busy_.load()) ||
       run_lease_eligible(run_lease_inputs)) && management_request_.load()==0 &&
      bootstrap_ && bootstrap_->renewal_due(now,requested_lease_us_) && transport_->ready();
    if(renew_available_.load() && reserve_lease_renewal()) renew_available_=false;
    release_available_=release_operation_available({
      releasing,release_status_fresh,state.management_epoch()==status.management_epoch(),
      device_profile_.functional_development_basis,enabled_,gate_->ready(),
      gate_->permit_refresh_allowed(),busy_.load(),management_request_.load()!=0,
      bootstrap_ && (bootstrap_->ready() || bootstrap_->operation_complete()),
      status.core_state()==wire_value(WireCoreState::kHold),
      state.core_state()==wire_value(WireCoreState::kHold),transport_->ready()});
    return true;
  }
  bool begin_release(std::uint64_t now) {
    const auto snapshot=latest_bootstrap_;
    if(!bootstrap_->start_release(snapshot,now)) return false;
    release_transition_origin_.capture(
      {snapshot.boot_epoch,snapshot.session_id,snapshot.reference_epoch,snapshot.motion_epoch},
      snapshot.management_epoch);
    return true;
  }
  void bootstrap_tick(std::uint64_t now) {
    if(!bootstrap_) return;
    if(release_stage_.load()!=0) {
      if(bootstrap_->phase()!=BootstrapPhase::kFault) bootstrap_->tick(now);
      if(bootstrap_->released()) {
        int started=2;
        if(release_stage_.compare_exchange_strong(started,3)) {
          enabled_=false;
          transport_->allow_inactive_prime(false);
        }
      } else if(bootstrap_->phase()==BootstrapPhase::kFault) release_stage_=4;
      return;
    }
    if(bootstrap_->phase()==BootstrapPhase::kFault && renewal_reserved_) {
      clear_failed_lease_renewal(management_request_,busy_,renewal_reserved_,
                                 renewal_started_,explicit_renewal_status_,
                                 renewal_owns_busy_.load());
      renewal_running_=false;
    }
    if(bootstrap_->phase()==BootstrapPhase::kFault) {
      report_bootstrap_fault(now);
      bootstrap_fault_snapshot_=true; admission_prerequisites_=false;
      ready_=false; renew_available_=false;
      if(enabled_) gate_->invalidate_authorization(now);
      return;
    }
    if(bootstrap_->phase()==BootstrapPhase::kIdle) return;
    bootstrap_->tick(now);
    if(renewal_reserved_ && renewal_started_ &&
       (bootstrap_->operation_complete() || bootstrap_->phase()==BootstrapPhase::kFault)) {
      if(bootstrap_->operation_complete()) {
        int pending=1;
        explicit_renewal_status_.compare_exchange_strong(pending,2);
      } else explicit_renewal_status_=0;
      renewal_reserved_=false;
      renewal_started_=false;
      if(renewal_owns_busy_.load()) busy_=false;
      renewal_running_=false;
    }
    if(bootstrap_->phase()==BootstrapPhase::kFault) {
      report_bootstrap_fault(now);
      bootstrap_fault_snapshot_=true; admission_prerequisites_=false;
      ready_=false; renew_available_=false;
      if(enabled_) gate_->invalidate_authorization(now);
      return;
    }
    if(enabled_ && permit_refresh_latch_.eligible(latest_bootstrap_.boot_epoch,
                                                 latest_bootstrap_.session_id) &&
       bootstrap_->operation_complete() &&
       bootstrap_->last_operation()==ManageOperation::kSetPermit) {
      if(const auto authorization=bootstrap_->authorization())
        if(gate_->renew_permit(*authorization))
          permit_refresh_latch_.applied();
    }
    if(!enabled_ && !evidence_incomplete_.load() && single_enable_requested_.exchange(false)) {
      const auto authorization=bootstrap_->authorization();
      if(!authorization || !transport_->allow_inactive_prime(true) ||
         !enable_gate_with_port_authority(*gate_,port_,*authorization,now)) {
        transport_->allow_inactive_prime(false); admission_prerequisites_=false; ready_=false; return;
      }
      enabled_=true;
    }
    if(!enabled_ && !evidence_incomplete_.load() && bootstrap_->ready()) {
      const auto authorization=bootstrap_->authorization();
      if(!authorization || !transport_->allow_inactive_prime(true) ||
         !enable_gate_with_port_authority(*gate_,port_,*authorization,now)) {
        transport_->allow_inactive_prime(false); admission_prerequisites_=false; ready_=false; return;
      }
      enabled_=true;
    }
    if(enabled_ && port_.direct_hold_started && !port_.direct_hold)
      transport_->allow_inactive_prime(false);
  }
  void report_bootstrap_fault(std::uint64_t now) {
    if(bootstrap_fault_reported_) return;
    bootstrap_fault_reported_=true;
    std::fprintf(stderr,
      "BOOTSTRAP_FAULT_DIAG now=%llu error=%s boot=%llu session=%llu observation_lost=%d renewal_reserved=%d gate_phase=%u\n",
      static_cast<unsigned long long>(now),bootstrap_->error(),
      static_cast<unsigned long long>(latest_bootstrap_.boot_epoch),
      static_cast<unsigned long long>(latest_bootstrap_.session_id),
      observation_lost_,renewal_reserved_.load(),static_cast<unsigned>(gate_->phase()));
  }
  void check_execution(std::uint64_t now) {
    if(!port_.executing || !port_.controller) return;
    const auto sample=port_.controller->execution_sample(); if(!sample) return;
    const auto elapsed=(now>port_.started_us ? now-port_.started_us : 0)*1e-6;
    if(active_ && active_->is_active()) {
      auto feedback=std::make_shared<Follow::Feedback>();
      feedback->header.stamp=this->now(); feedback->joint_names=profile_.joints;
      const auto count=profile_.joints.size();
      feedback->actual.positions.assign(sample->actual.begin(),sample->actual.begin()+count);
      feedback->desired.positions.assign(sample->desired.begin(),sample->desired.begin()+count);
      for(std::size_t i=0;i<count;++i) feedback->error.positions.push_back(sample->desired[i]-sample->actual[i]);
      feedback->actual.time_from_start=rclcpp::Duration::from_seconds(elapsed);
      feedback->desired.time_from_start=feedback->actual.time_from_start;
      feedback->error.time_from_start=feedback->actual.time_from_start;
      active_->publish_feedback(feedback);
    }
    const auto &trajectory=port_.trajectory;
    bool terminal=true;
    for(std::size_t i=0;i<trajectory.joints.size();++i) {
      const double error=std::abs(sample->actual[i]-sample->desired[i]);
      if(!std::isfinite(error) || error>trajectory.path_tolerance[i]) {
        gate_->inner_result(port_.motion_generation,Terminal::kAborted,"PATH_TOLERANCE_VIOLATED",now); return;
      }
      terminal=terminal && std::abs(sample->actual[i]-trajectory.points.back().positions[i])<=trajectory.goal_tolerance[i] &&
          std::abs(sample->desired[i]-trajectory.points.back().positions[i])<=1e-9;
    }
    if(elapsed<trajectory.points.back().time) return;
    if(terminal) gate_->inner_result(port_.motion_generation,Terminal::kSucceeded,"",now);
    else if(elapsed>trajectory.points.back().time+trajectory.goal_time_tolerance+0.25)
      gate_->inner_result(port_.motion_generation,Terminal::kAborted,"GOAL_TOLERANCE_VIOLATED",now);
  }
  void cycle_impl(GoalCancelAdmission<GoalHandle>::Control &admission) {
    static unsigned cycle_diag_count=0U;
    if(port_.direct_hold_started && cycle_diag_count++<3U)
      std::fprintf(stderr,"GATE_CYCLE_DIAG direct_hold=%d publishing=%d failed=%d enabled=%d observation_ready=%d gate_ready=%d error=%s\n",
                   port_.direct_hold,port_.publishing,port_.failed,enabled_,
                   observation_ready_.load(),gate_->ready(),gate_->error().c_str());
    const auto now=management_now_us();
    if(cancel_requests_.take_for(active_)) gate_->cancel(now);
    if((timeline_ && timeline_->failed()) || transport_->raw_evidence_failure()!=0U)
      evidence_incomplete_=true;
    if(evidence_incomplete_.load()) {
      if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited) { limited_goal_budget_.fail(); ready_=false; }
      admission_prerequisites_=false;
      ready_=false;
      if(active_ && gate_->terminal()==Terminal::kNone)
        gate_->abort("EVIDENCE_INCOMPLETE",now);
    }
    if(const auto update=dummy_ros_control::consume_control_update(last_update_)) {
      last_update_=update->generation;
      if(!gate_->record_update(port_.motion_generation,update->identity.motion_epoch,
          dummy_ros_control::wire_producer_sequence(update->generation),update->position,
          update->generation)) {
        gate_->abort("CONTROL_UPDATE_REJECTED",now);
      } else last_control_update_us_=now;
    }
    if(const auto drain=transport_->hold_drain_snapshot())
      gate_->observe_hold_drain(*drain,now);
    { std::unique_lock<std::mutex> lock(goal_mutex_,std::try_to_lock);
      if(lock && pending_) {
        active_=std::move(pending_); port_.uuid=active_->get_goal_id();
        if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
           (!transport_->raw_evidence_capture_enabled() ||
            !limited_goal_budget_.consume(port_.identity.boot,port_.identity.session))) {
          limited_goal_budget_.fail();
          auto rejected=std::make_shared<Follow::Result>();
          rejected->error_code=Follow::Result::INVALID_GOAL;
          rejected->error_string="LIMITED_FUNCTIONAL_BUDGET_EXHAUSTED_OR_IDENTITY";
          active_->abort(rejected); active_.reset(); busy_=false; ready_=false;
          return;
        }
        if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
           !limited_goal_budget_.available()) ready_=false;
        goal_accepted_us_=now;
        goal_before_identity_=timeline_identity_;
        goal_boot_=timeline_identity_.boot;
        goal_session_=timeline_identity_.session;
        goal_run_id_=0U;
        timeline_terminal_started_us_=0U;
        if(timeline_) {
          timeline_uuid_=port_.uuid;
          timeline_digest_=dummy_ros_control::trajectory_digest(active_->get_goal()->trajectory);
          timeline_goal_open_=true; timeline_after_=timeline_terminal_=timeline_hold_=false;
          fault_cleanup_confirmed_=false;timeline_fault_cleanup_=false;
          timeline_goal_hold_confirmed_=false;
          cancel_refresh_diagnostic_logged_=false;
          timeline_refresh_wait_started_us_=0U;
          timeline_refresh_closure_.reset();
          record_timeline(TimelineEvent::Kind::kAccepted,now);
        }
        const auto trajectory=validated_device_trajectory(*active_->get_goal(),profile_);
        if(evidence_incomplete_.load() || !trajectory || !gate_->submit(*trajectory,now)) {
          auto result=std::make_shared<Follow::Result>(); result->error_code=Follow::Result::INVALID_GOAL;
          result->error_string=evidence_incomplete_.load() ? "EVIDENCE_INCOMPLETE" : "DEVICE_NOT_READY";
          if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited) { limited_goal_budget_.fail(); ready_=false; }
          active_->abort(result); active_.reset(); busy_=false;
        } else {
          admission.open(active_);
          if(timeline_) record_timeline(TimelineEvent::Kind::kBeforeBegin,now);
        }
      }
    }
    port_.refresh_direct_hold();
    if(port_.failed) gate_->abort("GUARDED_REGISTRATION_FAILED",now);
    check_execution(now);
    if(active_ && gate_->terminal()!=Terminal::kNone) {
      auto result=std::make_shared<Follow::Result>(); result->error_string=gate_->error();
      if(timeline_ && !timeline_terminal_) {
        const auto kind=gate_->terminal()==Terminal::kSucceeded ? TimelineEvent::Kind::kTerminalSuccess :
          gate_->terminal()==Terminal::kCanceled ? TimelineEvent::Kind::kTerminalCanceled :
          TimelineEvent::Kind::kTerminalAborted;
        record_timeline(kind,now);
        timeline_terminal_=true;
        if(timeline_hold_observed_ &&
           (timeline_identity_.run_id==timeline_run_id_ || timeline_refresh_closure_) &&
           timeline_identity_.motion_epoch>=timeline_run_motion_epoch_) {
          record_timeline(TimelineEvent::Kind::kHold,now);
          timeline_hold_=true; timeline_goal_open_=false;
        }
      }
      // 自治fault清理只读专属原raw快照；不把Fault授权清零当可运行/可续租状态。
      if(gate_->terminal()==Terminal::kAborted && !fault_cleanup_confirmed_.load() && timeline_after_) {
        const auto receipt=transport_->raw_terminal_flushed();
        const auto cleanup=transport_->raw_cleanup_observation();
        const auto sampled_after_read=management_now_us();
        wire::DummyDeviceStatusV2 cleanup_status{};
        const bool status_decoded=wire::DummyDeviceStatusV2Codec::decode(cleanup.status.data(),cleanup.status.size(),&cleanup_status)==wire::CodecResult::kOk;
        DeviceStatusIdentity cleanup_identity{};
        if(status_decoded) {
          std::copy(std::begin(cleanup_status.contract_sha256()),std::end(cleanup_status.contract_sha256()),cleanup_identity.contract_hash.begin());
          std::copy(std::begin(cleanup_status.model_sha256()),std::end(cleanup_status.model_sha256()),cleanup_identity.model_hash.begin());
          std::copy(std::begin(cleanup_status.profile_sha256()),std::end(cleanup_status.profile_sha256()),cleanup_identity.profile_hash.begin());
          std::copy(std::begin(cleanup_status.compiled_policy_sha256()),std::end(cleanup_status.compiled_policy_sha256()),cleanup_identity.policy_hash.begin());
          std::copy(std::begin(cleanup_status.firmware_sha256()),std::end(cleanup_status.firmware_sha256()),cleanup_identity.firmware_hash.begin());
        }
        if(transport_->raw_evidence_capture_enabled() && transport_->raw_evidence_failure()==0U && status_decoded &&
           status_matches_bundle_identity(device_profile_,cleanup_identity) &&
           raw_fault_cleanup_matches(receipt,transport_->raw_first_page_written(),cleanup,goal_boot_,goal_session_,goal_run_id_,timeline_run_motion_epoch_,
             parameters_.profile,parameters_.mask,parameters_.device,sampled_after_read,state_max_age_us_)) {
          auto event=timeline_event(TimelineEvent::Kind::kFaultCleanup,sampled_after_read);
          event.identity={goal_boot_,goal_session_,port_.identity.reference_epoch,timeline_run_motion_epoch_,goal_run_id_};
          event.status_core_state=9U;event.state_core_state=9U;
          event.fault_cleanup={receipt.boot_epoch,receipt.session_id,receipt.run_id,receipt.attempt_motion_epoch,
            timeline_run_motion_epoch_,receipt.fault_stop_motion_epoch,receipt.first_failure_source,
            receipt.first_failure_axis_mask,receipt.final_state,receipt.terminal_chain,
            receipt.full_fault_groups_valid,receipt.chain_valid,receipt.release_committed,
            receipt.evidence_lost,receipt.motion_complete,true};
          if(!timeline_ || timeline_->record(event)) {
            fault_cleanup_confirmed_=true;timeline_fault_cleanup_=true;timeline_goal_open_=false;
            admission_prerequisites_=false;ready_=false;enable_available_=false;renew_available_=false;
          } else evidence_incomplete_=true;
        }
      }
      const bool raw_capture=transport_->raw_evidence_capture_enabled();
      const auto raw_terminal=transport_->raw_terminal_flushed();
      const bool raw_terminal_ready=raw_terminal_matches(
        raw_terminal,goal_boot_,goal_session_,goal_run_id_);
      const auto decision=fault_cleanup_confirmed_.load() && timeline_fault_cleanup_ && gate_->terminal()==Terminal::kAborted && (!timeline_ || timeline_->drained())
        ? timeline_success_decision(false,evidence_incomplete_.load() || (timeline_ && timeline_->failed()),
            false,!timeline_ || timeline_->drained(),true,transport_->raw_evidence_failure()!=0U,raw_terminal_ready)
        : raw_capture ? timeline_terminal_decision(
        gate_->terminal(),timeline_!=nullptr,
        evidence_incomplete_.load() || (timeline_ && timeline_->failed()),
        timeline_hold_,!timeline_ || timeline_->drained(),
        transport_->raw_evidence_failure()!=0U,raw_terminal_ready) :
        timeline_success_decision(timeline_!=nullptr,
          evidence_incomplete_.load() || (timeline_ && timeline_->failed()),
          timeline_hold_,!timeline_ || timeline_->drained());
      if(decision==TimelineSuccessDecision::kWait &&
         result->error_string!="DEVICE_COMMAND_REJECTED_DURING_ACK") {
        if(timeline_terminal_started_us_==0U) timeline_terminal_started_us_=now;
        if(now-timeline_terminal_started_us_<=2000000U) {
          admission_prerequisites_=false;
          ready_=false;
          return;
        }
        evidence_incomplete_=true;
      }
      if(result->error_string=="DEVICE_COMMAND_REJECTED_DURING_ACK") {
        result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
        active_->abort(result);
      } else if(evidence_incomplete_.load() || decision==TimelineSuccessDecision::kEvidenceIncomplete) {
        evidence_incomplete_=true;
        result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string=evidence_incomplete_result_error(result->error_string);
        active_->abort(result);
      } else if(fault_cleanup_confirmed_.load() && gate_->terminal()==Terminal::kAborted) {
        const auto final_cleanup=transport_->raw_cleanup_observation();
        if(transport_->raw_evidence_failure()!=0U || !raw_fault_cleanup_matches(
            transport_->raw_terminal_flushed(),transport_->raw_first_page_written(),final_cleanup,
            goal_boot_,goal_session_,goal_run_id_,timeline_run_motion_epoch_,parameters_.profile,
            parameters_.mask,parameters_.device,management_now_us(),state_max_age_us_)) {
          fault_cleanup_confirmed_=false;evidence_incomplete_=true;result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
          result->error_string=evidence_incomplete_result_error(result->error_string);active_->abort(result);
          active_.reset();busy_=false;return;
        }
        result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string=result->error_string.empty()?"FAULT_CLEANUP_CONFIRMED":result->error_string+";FAULT_CLEANUP_CONFIRMED";
        active_->abort(result);
      } else if(gate_->terminal()==Terminal::kSucceeded) {
        // Recheck the asynchronous writer immediately at the ROS result boundary.
        const auto final_raw_terminal=transport_->raw_terminal_flushed();
        if(transport_->raw_evidence_failure()!=0U || (timeline_ && timeline_->failed()) ||
           (raw_capture && !raw_terminal_matches(
             final_raw_terminal,goal_boot_,goal_session_,goal_run_id_))) {
          evidence_incomplete_=true;
          result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
          result->error_string=evidence_incomplete_result_error(result->error_string);
          active_->abort(result);
        } else active_->succeed(result);
      }
      else if(transport_->raw_evidence_failure()!=0U || (timeline_ && timeline_->failed()) ||
              (raw_capture && !raw_terminal_matches(transport_->raw_terminal_flushed(),
                goal_boot_,goal_session_,goal_run_id_))) {
        evidence_incomplete_=true;
        result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string=evidence_incomplete_result_error(result->error_string);
        active_->abort(result);
      } else if(gate_->terminal()==Terminal::kCanceled && active_->is_canceling()) active_->canceled(result);
      else { result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED; active_->abort(result); }
      if(device_profile_.no_load_basis==NoLoadBasis::kFunctionalLimited &&
         (gate_->terminal()!=Terminal::kSucceeded || evidence_incomplete_.load()))
        { limited_goal_budget_.fail(); ready_=false; }
      active_.reset(); busy_=false;
    }
    if(!fault_cleanup_confirmed_.load()) request_evidence_release(now);
    const AdmissionPrerequisites prerequisites{
      enabled_,observation_ready_.load(),gate_->ready(),port_.control_ready(),
      hold_tolerance_bound_,transport_->post_complete_begin_ready()};
    const bool fault=bootstrap_->phase()==BootstrapPhase::kFault;
    bootstrap_fault_snapshot_=fault;
    admission_prerequisites_=prerequisites.ready() && !fault;
    ready_=admission_prerequisites_.load() && !renewal_reserved_.load() &&
      release_stage_.load()==0 && !evidence_incomplete_.load() &&
      (device_profile_.no_load_basis!=NoLoadBasis::kFunctionalLimited || (limited_goal_budget_.available() && transport_->raw_evidence_capture_enabled()));
  }
  void request_evidence_release(std::uint64_t now) noexcept {
    if(!evidence_incomplete_.load() || !device_profile_.functional_development_basis || active_ ||
       release_stage_.load()!=0 || !enabled_ || !observation_ready_.load() ||
       !timeline_hold_observed_ || management_request_.load()!=0 ||
       (gate_->terminal()!=Terminal::kNone && gate_->error()=="STOP_FAILED") ||
       !bootstrap_ || !(bootstrap_->ready() || bootstrap_->operation_complete())) return;
    release_stage_=2;
    admission_prerequisites_=false;
    ready_=false;
    port_.disable(); port_.cancel();
    if(!begin_release(now)) release_stage_=4;
  }
  TimelineEvent timeline_event(TimelineEvent::Kind kind,std::uint64_t now) const noexcept {
    TimelineEvent event{};
    event.kind=kind; event.uuid=timeline_uuid_; event.trajectory_sha256=timeline_digest_;
    event.identity=timeline_refresh_closure_ &&
      (kind==TimelineEvent::Kind::kTerminalSuccess ||
       kind==TimelineEvent::Kind::kTerminalCanceled ||
       kind==TimelineEvent::Kind::kTerminalAborted ||
       kind==TimelineEvent::Kind::kHold) ?
      *timeline_refresh_closure_ : timeline_identity_;
    event.steady_us=now; event.axis_mask=parameters_.mask;
    event.status_core_state=timeline_status_core_state_;
    event.state_core_state=timeline_state_core_state_;
    if(kind==TimelineEvent::Kind::kTerminalCanceled && active_ && active_->is_canceling() &&
       timeline_after_ && goal_run_id_!=0U && goal_run_id_==timeline_run_id_ &&
       event.identity.boot==goal_boot_ && event.identity.session==goal_session_ &&
       event.identity.run_id==goal_run_id_) {
      if(const auto receipt=gate_->confirmed_cancel_receipt()) {
        event.cancel_request_id=receipt->request_id;
        event.cancel_operation_id=receipt->operation_id;
      }
    }
    return event;
  }
  bool record_timeline(TimelineEvent::Kind kind,std::uint64_t now) noexcept {
    if(!timeline_) return true;
    if(timeline_->record(timeline_event(kind,now))) return true;
    evidence_incomplete_=true;
    return false;
  }
  DpManagementParameters parameters_;
  JointProfile profile_;
  DeviceProfile device_profile_;
  std::string controller_name_;
  GuardedPort port_;
  std::shared_ptr<Controller> discovered_controller_;
  std::unique_ptr<DpManagementTransport> transport_;
  std::unique_ptr<WireManagementClient> client_;
  std::unique_ptr<BootstrapCoordinator> bootstrap_;
  std::unique_ptr<ActionGate> gate_;
  rclcpp_action::Server<Follow>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bootstrap_status_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr last_goal_rejection_service_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> management_services_;
  std::atomic<bool> ready_{false}, busy_{false};
  PendingGoalCancels<GoalHandle> cancel_requests_;
  GoalCancelAdmission<GoalHandle> cancel_admission_;
  std::atomic<bool> admission_prerequisites_{false}, bootstrap_fault_snapshot_{false};
  std::atomic<bool> enable_requested_{false}, enable_available_{false};
  std::atomic<bool> single_enable_requested_{false}, single_enable_available_{false};
  std::atomic<bool> explicit_operation_available_{false};
  std::atomic<bool> permit_operation_available_{false};
  std::atomic<bool> renew_available_{false}, renewal_reserved_{false}, renewal_started_{false};
  std::atomic<bool> run_renew_available_{false}, renewal_running_{false};
  std::atomic<bool> renewal_owns_busy_{true};
  PermitRefreshLatch permit_refresh_latch_;
  std::atomic<int> explicit_renewal_status_{0};
  std::atomic<bool> release_available_{false};
  std::atomic<int> release_stage_{0};
  std::atomic<int> management_request_{0};
  std::uint64_t state_max_age_us_=250000U;
  ObservationContinuity continuity_{250000U,2000000U,500000U};
  bool observation_lost_=false;
  bool release_transition_pending_=false;
  ReleaseTransitionOrigin release_transition_origin_{};
  bool bootstrap_fault_reported_=false;
  bool run_lease_diag_reported_=false;
  std::atomic<bool> observation_ready_{false};
  std::uint64_t requested_lease_us_=0U;
  std::mutex goal_mutex_;
  GoalRejectionReceipt goal_rejection_receipt_;
  std::shared_ptr<GoalHandle> pending_, active_;
  std::uint64_t last_update_=0;
  std::uint64_t last_control_update_us_=0;
  std::uint64_t last_typed_reject_seen_us_=0;
  LimitedGoalBudget limited_goal_budget_;
  std::uint64_t goal_accepted_us_=0;
  bool enabled_=false;
  bool hold_tolerance_bound_=false;
  BootstrapSnapshot latest_bootstrap_{};
  std::unique_ptr<GoalTimeline> timeline_;
  std::atomic<bool> evidence_incomplete_{false};
  rclcpp::TimerBase::SharedPtr evidence_failure_timer_;
  std::atomic<bool> evidence_failure_logged_{false};
  TimelineIdentity timeline_identity_{};
  TimelineIdentity timeline_goal_hold_identity_{};
  std::optional<TimelineIdentity> timeline_refresh_closure_;
  TimelineIdentity goal_before_identity_{};
  dummy_ros_control::GoalUuid timeline_uuid_{};
  dummy_ros_control::TrajectoryDigest timeline_digest_{};
  std::uint64_t goal_boot_=0, goal_session_=0, goal_run_id_=0;
  std::uint64_t timeline_terminal_started_us_=0;
  std::uint64_t timeline_refresh_wait_started_us_=0;
  std::uint64_t timeline_run_id_=0;
  std::uint32_t timeline_run_motion_epoch_=0;
  std::uint16_t timeline_status_core_state_=0, timeline_state_core_state_=0;
  std::uint32_t timeline_feedback_state_seq_=0;
  bool timeline_goal_open_=false, timeline_after_=false, timeline_terminal_=false;
  bool timeline_goal_hold_confirmed_=false;
  bool cancel_refresh_diagnostic_logged_=false;
  std::atomic<bool> fault_cleanup_confirmed_{false};
  bool timeline_fault_cleanup_=false;
  bool timeline_hold_=false, timeline_hold_observed_=false, timeline_release_recorded_=false;
};
}  // namespace dummy_motion_client

#ifndef DUMMY_ACTION_GATE_TEST
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto executor=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto gate=std::make_shared<dummy_motion_client::DeviceActionGate>();
    auto manager_options=controller_manager::get_cm_node_options();
    manager_options.arguments(std::vector<std::string>(argv+1,argv+argc));
    auto manager=std::make_shared<controller_manager::ControllerManager>(executor,"controller_manager","",manager_options);
    executor->add_node(manager); executor->add_node(gate);
    // Controller list discovery takes the manager's non-RT lock. Keep it off
    // the update thread: loading waits for that thread to adopt the new list.
    auto discovery=gate->create_wall_timer(std::chrono::milliseconds(20),[&]{gate->attach(*manager);});
    std::thread control([&] {
      const auto period=std::chrono::nanoseconds(1000000000ULL/std::max(1U,manager->get_update_rate()));
      auto next=std::chrono::steady_clock::now();
      while(rclcpp::ok()) {
        next+=period;
        // Refresh authority before producing a sample. Once the post-update
        // terminal check freezes that sample, a later ticket cannot retire it.
        gate->prepare_update();
        if(manager->is_resource_manager_initialized()) {
          const auto time=manager->now(); const rclcpp::Duration duration(period);
          manager->read(time,duration); manager->update(time,duration); manager->write(time,duration);
        }
        gate->cycle();
        std::this_thread::sleep_until(next);
      }
    });
    executor->spin(); rclcpp::shutdown(); control.join();
    discovery.reset();
    executor->remove_node(gate); gate.reset(); manager.reset();
  } catch(const std::exception &error) {
    fprintf(stderr,"device_action_gate: %s\n",error.what()); rclcpp::shutdown(); return 1;
  }
  return 0;
}
#endif
