#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/deadline.hpp"
#include "dummy_ros_control/diagnostic_scope.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>
namespace dummy_motion_client {
namespace {
constexpr double kTerminalVelocityZeroToleranceRadS=1e-9;
bool sequence_newer(std::uint32_t candidate,std::uint32_t current) {
  const auto distance=static_cast<std::uint32_t>(candidate-current);
  return distance!=0U && distance<0x80000000U;
}
bool sequence_at_or_after(std::uint32_t candidate,std::uint32_t current) {
  return static_cast<std::uint32_t>(candidate-current)<0x80000000U;
}
bool finite(const std::vector<double> &values) {
  return std::all_of(values.begin(),values.end(),[](double v) { return std::isfinite(v); });
}
bool tolerance(const std::vector<double> &values, const std::vector<double> &limits) {
  if (values.empty()) return true;
  if (values.size()!=limits.size() || !finite(values)) return false;
  for (std::size_t i=0;i<values.size();++i)
    if (values[i]<0 || values[i]>limits[i]) return false;
  return true;
}
}
std::optional<Trajectory> validate_trajectory(const Trajectory &input, const JointProfile &profile,
                                              const Position *measured_start) {
  const auto count=profile.joints.size();
  if (count==0 || count>6 || input.joints.size()!=count || input.points.empty() ||
      profile.lower.size()!=count || profile.upper.size()!=count || profile.velocity.size()!=count ||
      profile.acceleration.size()!=count || profile.path_tolerance.size()!=count || profile.goal_tolerance.size()!=count ||
      !finite(profile.lower) || !finite(profile.upper) || !finite(profile.velocity) ||
      !finite(profile.acceleration) || !finite(profile.path_tolerance) || !finite(profile.goal_tolerance) ||
      !std::isfinite(input.goal_time_tolerance) || input.goal_time_tolerance<0) return {};
  const std::unordered_set<std::string> names(input.joints.begin(),input.joints.end());
  if (names.size()!=count || std::unordered_set<std::string>(profile.joints.begin(),profile.joints.end())!=names)
    return {};
  auto output=input; output.joints=profile.joints;
  std::vector<std::size_t> order;
  for (const auto &name:profile.joints)
    order.push_back(std::find(input.joints.begin(),input.joints.end(),name)-input.joints.begin());
  auto reorder=[&order](const std::vector<double> &values) {
    std::vector<double> result; for (auto index:order) result.push_back(values[index]); return result;
  };
  if ((!input.path_tolerance.empty() && input.path_tolerance.size()!=count) ||
      (!input.goal_tolerance.empty() && input.goal_tolerance.size()!=count)) return {};
  if (!input.path_tolerance.empty()) output.path_tolerance=reorder(input.path_tolerance);
  if (!input.goal_tolerance.empty()) output.goal_tolerance=reorder(input.goal_tolerance);
  if (!tolerance(output.path_tolerance,profile.path_tolerance) ||
      !tolerance(output.goal_tolerance,profile.goal_tolerance)) return {};
  if (output.path_tolerance.empty()) output.path_tolerance=profile.path_tolerance;
  if (output.goal_tolerance.empty()) output.goal_tolerance=profile.goal_tolerance;
  for (std::size_t i=0;i<count;++i) {
    if (output.path_tolerance[i]==0) output.path_tolerance[i]=profile.path_tolerance[i];
    if (output.goal_tolerance[i]==0) output.goal_tolerance[i]=profile.goal_tolerance[i];
  }
  double previous=-1;
  for (auto &point:output.points) {
    if (point.positions.size()!=count || (!point.velocities.empty() && point.velocities.size()!=count) ||
        (!point.accelerations.empty() && point.accelerations.size()!=count) || !finite(point.positions) ||
        !finite(point.velocities) || !finite(point.accelerations) || !std::isfinite(point.time) ||
        point.time<0 || point.time<=previous) return {};
    previous=point.time; point.positions=reorder(point.positions);
    if (!point.velocities.empty()) point.velocities=reorder(point.velocities);
    if (!point.accelerations.empty()) point.accelerations=reorder(point.accelerations);
    for (std::size_t i=0;i<count;++i) {
      if (profile.lower[i]>profile.upper[i] || profile.velocity[i]<=0 || profile.acceleration[i]<=0 ||
          profile.path_tolerance[i]<=0 || profile.goal_tolerance[i]<=0 ||
          point.positions[i]<profile.lower[i] || point.positions[i]>profile.upper[i] ||
          (!point.velocities.empty() && std::abs(point.velocities[i])>profile.velocity[i]) ||
          (!point.accelerations.empty() && std::abs(point.accelerations[i])>profile.acceleration[i])) return {};
    }
  }
  for (double velocity:output.points.back().velocities)
    if (std::abs(velocity)>kTerminalVelocityZeroToleranceRadS) return {};
  const bool position_only=std::all_of(output.points.begin(),output.points.end(),[](const auto &point) {
    return point.velocities.empty() && point.accelerations.empty();
  });
  if (position_only) {
    std::vector<double> previous_position;
    double previous_time=0, previous_duration=0;
    std::vector<double> previous_velocity;
    if (measured_start) {
      previous_position.assign(measured_start->begin(),measured_start->begin()+count);
      for (std::size_t i=0;i<count;++i)
        if (!std::isfinite(previous_position[i]) || previous_position[i]<profile.lower[i] ||
            previous_position[i]>profile.upper[i]) return {};
    }
    const std::size_t first_segment=measured_start ? 0 : 1;
    if (!measured_start) {
      previous_position=output.points.front().positions;
      previous_time=output.points.front().time;
    }
    for (std::size_t point_index=first_segment;point_index<output.points.size();++point_index) {
      const auto &point=output.points[point_index];
      const double duration=point.time-previous_time;
      if (duration<=0) return {};
      std::vector<double> velocity(count);
      for (std::size_t i=0;i<count;++i) {
        velocity[i]=(point.positions[i]-previous_position[i])/duration;
        if (std::abs(velocity[i])>profile.velocity[i] ||
            (!previous_velocity.empty() &&
             std::abs(velocity[i]-previous_velocity[i])/std::min(duration,previous_duration)>profile.acceleration[i]))
          return {};
      }
      previous_position=point.positions; previous_time=point.time;
      previous_duration=duration; previous_velocity=std::move(velocity);
    }
  }
  return output;
}
ActionGate::ActionGate(ManagementClient &client, ActionPort &port, JointProfile profile, std::uint64_t timeout)
    : client_(client),port_(port),profile_(std::move(profile)),timeout_(timeout) {}
bool ActionGate::issue(ManageOperation operation,std::uint64_t now,std::uint64_t existing_deadline) {
  auto request=authorization_.identity; request.set_expected_motion_epoch(epoch_);
  request.set_operation(static_cast<std::uint16_t>(operation));
  if (operation==ManageOperation::kComplete) {
    request.set_terminal_producer_seq(frozen_seq_);
    double positions[6]{};
    std::copy_n(frozen_q_.begin(),profile_.joints.size(),positions);
    request.set_terminal_position_rad(positions);
  }
  operation_=operation;
  deadline_=existing_deadline ? existing_deadline : saturating_deadline(now,timeout_);
  request_=client_.submit(request,now);
  management_evidence_={}; state_evidence_={};
  cancel_request_id_=(operation==ManageOperation::kCancel && cancel_terminal_) ? request_ : 0U;
  if (!request_ && operation==ManageOperation::kComplete) {
    completing_after_renewal_=client_.matching_inflight_renewal(request);
    if(completing_after_renewal_!=0U) { deferred_complete_request_=request; return true; }
  }
  if (!request_ && operation==ManageOperation::kCancel && cancel_terminal_ && error_.empty()) {
    canceling_after_renewal_=client_.matching_inflight_renewal(request);
    if(canceling_after_renewal_!=0U) { deferred_cancel_request_=request; return true; }
  }
  if (!request_) { fail(operation==ManageOperation::kCancel ? "STOP_FAILED" : "TRANSPORT_UNAVAILABLE"); return false; }
  return true;
}
std::optional<CancelReceipt> ActionGate::confirmed_cancel_receipt() const noexcept {
  const auto &reply=management_evidence_;
  if(terminal_!=Terminal::kCanceled || phase_!=Phase::kHold || request_!=0U ||
     operation_!=ManageOperation::kCancel || !cancel_terminal_ || cancel_request_id_==0U ||
     reply.request_id!=cancel_request_id_ || reply.operation_id==0U ||
     !reply.accepted || !reply.result_valid || !reply.succeeded ||
     reply.transaction_phase!=wire_value(WireTransactionPhase::kCommitted) ||
     reply.result_code!=wire_value(WireResult::kOk) || reply.motion_epoch!=epoch_ ||
     reply.session_id!=authorization_.identity.expected_session_id() ||
     reply.reference_epoch!=authorization_.identity.expected_reference_epoch())
    return std::nullopt;
  return CancelReceipt{cancel_request_id_,reply.operation_id};
}
bool ActionGate::enable(const Authorization &authorization,std::uint64_t now) {
  if (phase_!=Phase::kIdle || request_ || !authorization.session || !authorization.reference ||
      !authorization.permit || authorization.identity.expected_session_id()==0) return false;
  authorization_=authorization;
  epoch_=authorization.identity.expected_motion_epoch();
  return issue(ManageOperation::kEnable,now);
}
bool ActionGate::renew_permit(const Authorization &authorization) {
  const auto &current=authorization_.identity;
  const auto &next=authorization.identity;
  if (phase_!=Phase::kHold || request_!=0U || !authorization.session ||
      !authorization.reference || !authorization.permit ||
      current.expected_boot_epoch()!=next.expected_boot_epoch() ||
      current.expected_session_id()!=next.expected_session_id() ||
      current.expected_reference_epoch()!=next.expected_reference_epoch() ||
      current.expected_profile_id()!=next.expected_profile_id() ||
      current.configured_mask()!=next.configured_mask() ||
      !std::equal(std::begin(current.device_id()),std::end(current.device_id()),std::begin(next.device_id())) ||
      !std::equal(std::begin(current.client_instance_id()),std::end(current.client_instance_id()),
                  std::begin(next.client_instance_id()))) return false;
  authorization_=authorization;
  return true;
}
bool ActionGate::submit(const Trajectory &trajectory,std::uint64_t now) {
  if (!ready() || !authorization_.session || !authorization_.reference || !authorization_.permit) return false;
  const auto validated=validate_trajectory(trajectory,profile_); if (!validated) return false;
  trajectory_=*validated; terminal_=Terminal::kNone; error_.clear(); frozen_=false; frozen_seq_=0;
  if (!port_.freeze_hold()) return issue(ManageOperation::kBegin,now);
  frozen_seq_=last_seq_; frozen_generation_=last_producer_generation_;
  drained_generation_=0; drained_committed_seq_=0;
  drain_worker_ready_=false; drain_state_fresh_=false; hold_drain_pending_=true;
  if(auto* journal=dummy_ros_control::current_host_diagnostics) {
        journal->detail(
               "GATE_HOLD_DRAIN_FREEZE_DIAG frozen=%u generation=%llu committed=%u state=%u epoch=%u submitted=%u sent=%u\n",
               frozen_seq_,static_cast<unsigned long long>(frozen_generation_),
               hold_committed_seq_,state_seq_,epoch_,hold_submitted_mask_,hold_sent_mask_);
  } else std::fprintf(stderr,
               "GATE_HOLD_DRAIN_FREEZE_DIAG frozen=%u generation=%llu committed=%u state=%u epoch=%u submitted=%u sent=%u\n",
               frozen_seq_,static_cast<unsigned long long>(frozen_generation_),
               hold_committed_seq_,state_seq_,epoch_,hold_submitted_mask_,hold_sent_mask_);
  deadline_=saturating_deadline(now,timeout_);
  return true;
}
bool ActionGate::prime(const ManageReply &reply,Phase phase,std::uint64_t now) {
  port_.disable(); ++generation_; epoch_=reply.motion_epoch; phase_=phase;
  sampled_=false; last_seq_=0; last_producer_generation_=0;
  prime_seq_=0; frozen_=false;
  prime_producer_generation_=0; hold_state_observed_us_=0;
  hold_committed_seq_=0; hold_submitted_mask_=hold_sent_mask_=0;
  prime_q_=phase==Phase::kPrimeMotion ? reply.position : reply.hold_position;
  if (phase==Phase::kPrimeMotion) {
    const auto validated=validate_trajectory(trajectory_,profile_,&prime_q_);
    if (!validated) { stop(false,"TRAJECTORY_LIMIT_VIOLATION",now); return false; }
    trajectory_=*validated;
  }
  if(auto* journal=dummy_ros_control::current_host_diagnostics) {
        journal->detail( "GATE_PRIME_DIAG phase=%u epoch=%u generation=%llu q0=%.9f\n",
               static_cast<unsigned>(phase), epoch_,
               static_cast<unsigned long long>(generation_), prime_q_[0]);
  } else std::fprintf(stderr, "GATE_PRIME_DIAG phase=%u epoch=%u generation=%llu q0=%.9f\n",
               static_cast<unsigned>(phase), epoch_,
               static_cast<unsigned long long>(generation_), prime_q_[0]);
  port_.prime(prime_q_,epoch_,generation_);
  return true;
}
bool ActionGate::handshake(const ManageReply &reply) const {
  return reply.snapshot_valid && reply.motion_epoch==epoch_ && sampled_ &&
      reply.accepted_seq!=0U && sequence_at_or_after(reply.accepted_seq,prime_seq_) &&
      reply.result_valid && reply.succeeded;
}
void ActionGate::observe(const ManageReply &reply,std::uint64_t now) {
  if (reply.request_id!=request_) return;
  management_evidence_=reply;
  if (state_evidence_.snapshot_valid && !reply.snapshot_valid) {
    auto combined=state_evidence_;
    combined.result_valid=reply.result_valid; combined.succeeded=reply.succeeded;
    combined.rejected=reply.rejected; combined.queued=false;
    advance(combined,now);
  } else advance(reply,now);
  try_completion_hold(now);
}
bool ActionGate::observe_state(const std::uint8_t *bytes,std::size_t size,std::uint64_t now) {
  namespace wire=dp::protocol::dummy_robot_v2;
  wire::DummyJointStateV2 state;
  if (wire::DummyJointStateV2Codec::decode(bytes,size,&state)!=wire::CodecResult::kOk ||
      state.schema_major()!=2 ||
      (!state_seen_ && state.state_seq()==0U) ||
      (state_seen_ && !sequence_newer(state.state_seq(),state_seq_)) ||
      state.boot_epoch()!=authorization_.identity.expected_boot_epoch() ||
      state.session_id()!=authorization_.identity.expected_session_id() ||
      state.reference_epoch()!=authorization_.identity.expected_reference_epoch() ||
      state.profile_id()!=authorization_.identity.expected_profile_id() ||
      state.configured_mask()!=authorization_.identity.configured_mask() ||
      (state.position_valid_mask() & state.configured_mask())!=state.configured_mask() ||
      !std::equal(std::begin(state.device_id()),std::end(state.device_id()),std::begin(authorization_.identity.device_id()))) {
    return false;
  }
  const bool hold_valid=state.hold_ready() && state.configured_mask()!=0 &&
      (state.position_valid_mask() & state.configured_mask())==state.configured_mask();
  for (std::size_t axis=0;axis<6;++axis) {
    if (!(state.configured_mask() & (1U<<axis))) continue;
    if (!std::isfinite(state.position_rad()[axis])) return false;
  }
  state_seq_=state.state_seq(); state_seen_=true;
  hold_state_observed_us_=0;
  authorization_.identity.set_expected_management_epoch(state.management_epoch());
  authorization_.identity.set_freshness_ticket(state.freshness_ticket());
  if (state.motion_epoch()==epoch_ &&
      state.core_state()==wire_value(WireCoreState::kHold) && hold_valid) {
    hold_committed_seq_=state.committed_producer_seq();
    hold_submitted_mask_=state.can_submitted_mask();
    hold_sent_mask_=state.can_sent_mask();
    hold_state_observed_us_=now;
    if (hold_drain_pending_) drain_state_fresh_=true;
  }
  if (hold_drain_pending_ && state.core_state()==wire_value(WireCoreState::kFault)) {
    fail("DEVICE_FAULT");
    return true;
  }
  if (hold_drain_pending_ && deadline_ && now>=deadline_) {
    stop(false,"HOLD_DRAIN_TIMEOUT",now);
    return true;
  }
  if (hold_drain_pending_ && drain_worker_ready_ && drain_state_fresh_ &&
      state.motion_epoch()==epoch_ &&
      state.core_state()==wire_value(WireCoreState::kHold) && hold_valid &&
      state.can_submitted_mask()==state.configured_mask() &&
      state.can_sent_mask()==state.configured_mask() &&
      state.committed_producer_seq()!=0U &&
      sequence_at_or_after(state.committed_producer_seq(),drained_committed_seq_)) {
    const auto begin_deadline=deadline_;
    hold_drain_pending_=false;
    issue(ManageOperation::kBegin,now,begin_deadline);
    return true;
  }
  if (!request_) return true;
  if (state.core_state()==wire_value(WireCoreState::kFault)) { fail("DEVICE_FAULT"); return true; }
  ManageReply evidence; evidence.request_id=request_; evidence.snapshot_valid=true;
  evidence.motion_epoch=state.motion_epoch(); evidence.accepted_seq=state.committed_producer_seq();
  const bool enabling_hold=state.core_state()==wire_value(WireCoreState::kEnablingHold);
  evidence.hold_ready=state.hold_ready() && hold_valid;
  evidence.hold_available=enabling_hold ||
      (hold_valid && state.core_state()==wire_value(WireCoreState::kHold));
  // A fresh, identity-checked state transition is core acceptance evidence; QUEUED alone is not.
  evidence.accepted=(evidence.motion_epoch>epoch_ &&
      ((operation_==ManageOperation::kBegin && state.core_state()==wire_value(WireCoreState::kRunning)) ||
       (operation_!=ManageOperation::kBegin && evidence.hold_available))) ||
      (evidence.motion_epoch==epoch_ && phase_!=Phase::kIdle && phase_!=Phase::kHold);
  std::copy(std::begin(state.position_rad()),std::end(state.position_rad()),evidence.position.begin());
  evidence.hold_position=evidence.position;
  evidence.result_valid=management_evidence_.result_valid; evidence.succeeded=management_evidence_.succeeded;
  state_evidence_=evidence; advance(evidence,now);
  // prime() may have advanced the epoch while handling this very State.
  if(state.motion_epoch()==epoch_ && state.core_state()==wire_value(WireCoreState::kHold) && hold_valid) {
    hold_committed_seq_=state.committed_producer_seq();
    hold_submitted_mask_=state.can_submitted_mask(); hold_sent_mask_=state.can_sent_mask();
    hold_state_observed_us_=now;
  }
  try_completion_hold(now); return true;
}
void ActionGate::observe_hold_drain(const HoldDrainEvidence &evidence,std::uint64_t now) {
  completion_worker_=evidence;
  try_completion_hold(now);
  if (!hold_drain_pending_ || !evidence.processed_generation || !evidence.committed_seq ||
      evidence.processed_generation<frozen_generation_ ||
      evidence.boot_epoch!=authorization_.identity.expected_boot_epoch() ||
      evidence.session_id!=authorization_.identity.expected_session_id() ||
      evidence.reference_epoch!=authorization_.identity.expected_reference_epoch() ||
      evidence.motion_epoch!=epoch_ || (deadline_ && now>=deadline_)) return;
  if (!drain_worker_ready_)
    if(auto* journal=dummy_ros_control::current_host_diagnostics) {
        journal->detail(
                 "GATE_HOLD_DRAIN_WORKER_DIAG frozen_generation=%llu processed=%llu target=%u epoch=%u\n",
                 static_cast<unsigned long long>(frozen_generation_),
                 static_cast<unsigned long long>(evidence.processed_generation),
                 evidence.committed_seq,epoch_);
  } else std::fprintf(stderr,
                 "GATE_HOLD_DRAIN_WORKER_DIAG frozen_generation=%llu processed=%llu target=%u epoch=%u\n",
                 static_cast<unsigned long long>(frozen_generation_),
                 static_cast<unsigned long long>(evidence.processed_generation),
                 evidence.committed_seq,epoch_);
  drained_generation_=evidence.processed_generation;
  drained_committed_seq_=evidence.committed_seq;
  drain_worker_ready_=true;
}
void ActionGate::try_completion_hold(std::uint64_t now) {
  const auto &worker=completion_worker_;
  const auto mask=authorization_.identity.configured_mask();
  if(phase_!=Phase::kPrimeCompletionHold || operation_!=ManageOperation::kComplete ||
     !request_ || !sampled_ || !prime_producer_generation_ ||
     (deadline_ && now>=deadline_) || !management_evidence_.result_valid ||
     !management_evidence_.succeeded || management_evidence_.request_id!=request_ ||
     management_evidence_.motion_epoch!=epoch_ ||
     !worker.quiescent || (worker.disposition!=1U && worker.disposition!=2U) ||
     worker.processed_generation<prime_producer_generation_ || !worker.committed_seq ||
     worker.boot_epoch!=authorization_.identity.expected_boot_epoch() ||
     worker.session_id!=authorization_.identity.expected_session_id() ||
     worker.reference_epoch!=authorization_.identity.expected_reference_epoch() ||
     worker.motion_epoch!=epoch_ || !worker.observed_us || !worker.max_state_age_us ||
     now<worker.observed_us || now-worker.observed_us>worker.max_state_age_us ||
     !hold_state_observed_us_ || now<hold_state_observed_us_ ||
     now-hold_state_observed_us_>worker.max_state_age_us ||
     !state_evidence_.snapshot_valid || state_evidence_.motion_epoch!=epoch_ ||
     !state_evidence_.hold_ready || !mask || hold_submitted_mask_!=mask || hold_sent_mask_!=mask ||
     !sequence_at_or_after(hold_committed_seq_,worker.committed_seq)) return;
  for(std::size_t axis=0;axis<prime_q_.size();++axis) {
    if(!(mask & (1U<<axis))) continue;
    const auto tolerance=worker.tolerance[axis];
    if(!std::isfinite(tolerance) || tolerance<0.0 || !std::isfinite(worker.position[axis]) ||
       std::abs(worker.position[axis]-prime_q_[axis])>tolerance ||
       std::abs(state_evidence_.hold_position[axis]-prime_q_[axis])>tolerance) return;
  }
  terminal_=Terminal::kSucceeded; phase_=Phase::kHold; request_=0; deadline_=0;
}
void ActionGate::advance(const ManageReply &reply,std::uint64_t now) {
  if (!request_ || reply.request_id!=request_ || phase_==Phase::kFault) return;
  if (reply.rejected || (reply.result_valid && !reply.succeeded)) {
    if (phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold) fail("STOP_FAILED");
    else if (operation_==ManageOperation::kEnable) fail("ENABLE_FAILED");
    else stop(false,"OPERATION_FAILED",now);
    return;
  }
  if (reply.queued || !reply.accepted || !reply.snapshot_valid) return;
  if (phase_==Phase::kCompleting && frozen_ && operation_==ManageOperation::kBegin) {
    if (reply.motion_epoch==epoch_ && reply.accepted_seq==frozen_seq_) issue(ManageOperation::kComplete,now);
    return;
  }
  if (operation_==ManageOperation::kEnable && phase_==Phase::kIdle && reply.motion_epoch>epoch_ && reply.hold_available) {
    if (!prime(reply,Phase::kPrimeHold,now)) return;
  } else if (operation_==ManageOperation::kBegin && phase_==Phase::kHold && reply.motion_epoch>epoch_) {
    if (!prime(reply,Phase::kPrimeMotion,now)) return;
    authorization_.permit=false;
  } else if (operation_==ManageOperation::kComplete && phase_==Phase::kCompleting && reply.motion_epoch>epoch_ && reply.hold_available) {
    if (!prime(reply,Phase::kPrimeCompletionHold,now)) return;
  } else if (operation_==ManageOperation::kCancel && phase_==Phase::kStopping && reply.motion_epoch>epoch_ && reply.hold_available) {
    if (!prime(reply,Phase::kPrimeStopHold,now)) return;
  }
  if (!handshake(reply)) return;
  if (phase_==Phase::kPrimeMotion) {
    phase_=Phase::kRunning; port_.start(trajectory_,generation_); deadline_=0;
  } else if ((phase_==Phase::kPrimeHold || phase_==Phase::kPrimeCompletionHold || phase_==Phase::kPrimeStopHold) && reply.hold_ready) {
    if (phase_==Phase::kPrimeCompletionHold) terminal_=Terminal::kSucceeded;
    if (phase_==Phase::kPrimeStopHold) terminal_=cancel_terminal_ ? Terminal::kCanceled : Terminal::kAborted;
    phase_=Phase::kHold; request_=0; deadline_=0;
  }
}
bool ActionGate::deferred_complete_matches_authority() const noexcept {
  const auto &r=deferred_complete_request_; const auto &a=authorization_.identity;
  return authorization_.session && authorization_.reference &&
      r.expected_boot_epoch()==a.expected_boot_epoch() &&
      r.expected_session_id()==a.expected_session_id() &&
      r.expected_reference_epoch()==a.expected_reference_epoch() &&
      r.expected_motion_epoch()==epoch_ && r.expected_management_epoch()==a.expected_management_epoch() &&
      r.expected_profile_id()==a.expected_profile_id() && r.configured_mask()==a.configured_mask() &&
      std::equal(std::begin(r.device_id()),std::end(r.device_id()),std::begin(a.device_id())) &&
      std::equal(std::begin(r.client_instance_id()),std::end(r.client_instance_id()),std::begin(a.client_instance_id()));
}
bool ActionGate::bounded_completion_intent(const ManageRequest &live, std::uint32_t committed_seq,
                                          std::uint64_t now) const noexcept {
  // 只证明本goal已冻结且设备确认的完成意图，不授权新采样或续租。
  const auto &identity=authorization_.identity;
  if(phase_!=Phase::kCompleting || terminal_!=Terminal::kNone || !frozen_ ||
     !frozen_seq_ || frozen_seq_!=last_seq_ || !committed_seq ||
     !sequence_at_or_after(committed_seq,frozen_seq_) || !deadline_ || now>=deadline_ ||
     !authorization_.session || !authorization_.reference || trajectory_.points.empty() ||
     (operation_!=ManageOperation::kBegin && operation_!=ManageOperation::kComplete) ||
     (operation_==ManageOperation::kBegin && !request_) ||
     (operation_==ManageOperation::kComplete && !request_ && !completing_after_renewal_) ||
     !live.expected_boot_epoch() || !live.expected_session_id() ||
     !live.expected_reference_epoch() || !live.expected_profile_id() || !live.configured_mask() ||
     live.expected_boot_epoch()!=identity.expected_boot_epoch() ||
     live.expected_session_id()!=identity.expected_session_id() ||
     live.expected_reference_epoch()!=identity.expected_reference_epoch() ||
     live.expected_motion_epoch()!=epoch_ ||
     live.expected_management_epoch()!=identity.expected_management_epoch() ||
     live.expected_profile_id()!=identity.expected_profile_id() ||
     live.configured_mask()!=identity.configured_mask() ||
     !std::equal(std::begin(live.device_id()),std::end(live.device_id()),std::begin(identity.device_id())) ||
     !std::equal(std::begin(live.client_instance_id()),std::end(live.client_instance_id()),std::begin(identity.client_instance_id())))
    return false;
  const auto &target=trajectory_.points.back().positions;
  if(target.size()!=profile_.joints.size() || trajectory_.goal_tolerance.size()!=target.size()) return false;
  for(std::size_t axis=0;axis<target.size();++axis) {
    if(!std::isfinite(frozen_q_[axis]) || !std::isfinite(last_q_[axis]) ||
       !std::isfinite(target[axis]) || !std::isfinite(trajectory_.goal_tolerance[axis]) ||
       trajectory_.goal_tolerance[axis]<0.0 || frozen_q_[axis]!=last_q_[axis] ||
       std::abs(frozen_q_[axis]-target[axis])>trajectory_.goal_tolerance[axis]) return false;
  }
  return true;
}
bool ActionGate::bounded_cancel_intent(const ManageRequest &live, std::uint64_t now) const noexcept {
  // 只识别当前已提交、仍在原期限内的取消事务，供撤回尚未发送的
  // RUN续租预约；不授权新的续租或延长停止期限。
  const auto &identity=authorization_.identity;
  return phase_==Phase::kStopping && terminal_==Terminal::kNone && frozen_ &&
      cancel_terminal_ && error_.empty() && operation_==ManageOperation::kCancel &&
      request_!=0U && cancel_request_id_==request_ && deadline_!=0U && now<deadline_ &&
      authorization_.session && authorization_.reference &&
      live.operation()==wire_value(ManageOperation::kCancel) &&
      live.expected_boot_epoch()!=0U && live.expected_session_id()!=0U &&
      live.expected_reference_epoch()!=0U && live.expected_profile_id()!=0U &&
      live.configured_mask()!=0U &&
      live.expected_boot_epoch()==identity.expected_boot_epoch() &&
      live.expected_session_id()==identity.expected_session_id() &&
      live.expected_reference_epoch()==identity.expected_reference_epoch() &&
      live.expected_motion_epoch()==epoch_ &&
      live.expected_management_epoch()==identity.expected_management_epoch() &&
      live.expected_profile_id()==identity.expected_profile_id() &&
      live.configured_mask()==identity.configured_mask() &&
      std::equal(std::begin(live.device_id()),std::end(live.device_id()),std::begin(identity.device_id())) &&
      std::equal(std::begin(live.client_instance_id()),std::end(live.client_instance_id()),std::begin(identity.client_instance_id()));
}
bool ActionGate::completion_waiting_for_renewal(std::uint64_t now) const noexcept {
  return phase_==Phase::kCompleting && terminal_==Terminal::kNone &&
      operation_==ManageOperation::kComplete && request_==0U && frozen_ && frozen_seq_!=0U &&
      completing_after_renewal_!=0U && deadline_!=0U && now<deadline_ &&
      deferred_complete_matches_authority() &&
      client_.matching_inflight_renewal(deferred_complete_request_)==completing_after_renewal_;
}
bool ActionGate::deferred_cancel_matches_authority() const noexcept {
  const auto &r=deferred_cancel_request_; const auto &a=authorization_.identity;
  return r.operation()==wire_value(ManageOperation::kCancel) && authorization_.session && authorization_.reference &&
      r.expected_boot_epoch()==a.expected_boot_epoch() &&
      r.expected_session_id()==a.expected_session_id() &&
      r.expected_reference_epoch()==a.expected_reference_epoch() &&
      r.expected_motion_epoch()==epoch_ && r.expected_management_epoch()==a.expected_management_epoch() &&
      r.expected_profile_id()==a.expected_profile_id() && r.configured_mask()==a.configured_mask() &&
      std::equal(std::begin(r.device_id()),std::end(r.device_id()),std::begin(a.device_id())) &&
      std::equal(std::begin(r.client_instance_id()),std::end(r.client_instance_id()),std::begin(a.client_instance_id()));
}
bool ActionGate::cancel_waiting_for_renewal(const ManageRequest &live, std::uint64_t now) const noexcept {
  return phase_==Phase::kStopping && terminal_==Terminal::kNone && frozen_ &&
      cancel_terminal_ && error_.empty() && operation_==ManageOperation::kCancel &&
      request_==0U && cancel_request_id_==0U && canceling_after_renewal_!=0U &&
      deadline_!=0U && now<deadline_ && authorization_.session && authorization_.reference &&
      deferred_cancel_matches_authority() &&
      live.operation()==wire_value(ManageOperation::kCancel) &&
      live.expected_boot_epoch()==deferred_cancel_request_.expected_boot_epoch() &&
      live.expected_session_id()==deferred_cancel_request_.expected_session_id() &&
      live.expected_reference_epoch()==deferred_cancel_request_.expected_reference_epoch() &&
      live.expected_motion_epoch()==deferred_cancel_request_.expected_motion_epoch() &&
      live.expected_management_epoch()==deferred_cancel_request_.expected_management_epoch() &&
      live.expected_profile_id()==deferred_cancel_request_.expected_profile_id() &&
      live.configured_mask()==deferred_cancel_request_.configured_mask() &&
      std::equal(std::begin(live.device_id()),std::end(live.device_id()),std::begin(deferred_cancel_request_.device_id())) &&
      std::equal(std::begin(live.client_instance_id()),std::end(live.client_instance_id()),std::begin(deferred_cancel_request_.client_instance_id())) &&
      client_.matching_inflight_renewal(deferred_cancel_request_)==canceling_after_renewal_;
}
void ActionGate::tick(std::uint64_t now) {
  if(canceling_after_renewal_!=0U) {
    if(phase_!=Phase::kStopping || terminal_!=Terminal::kNone || !frozen_ || !cancel_terminal_ ||
       !error_.empty() || operation_!=ManageOperation::kCancel || request_!=0U || cancel_request_id_!=0U ||
       deadline_==0U || !deferred_cancel_matches_authority()) {
      canceling_after_renewal_=0U; fail("STOP_FAILED"); return;
    }

    if(deadline_ && now>=deadline_) { canceling_after_renewal_=0U; fail("STOP_FAILED"); return; }
    const auto renewal=client_.renewal_completion(canceling_after_renewal_);
    if(renewal==RenewalCompletion::kPending) return;
    canceling_after_renewal_=0U;
    if(renewal!=RenewalCompletion::kCommitted || !deferred_cancel_matches_authority()) { fail("STOP_FAILED"); return; }
    deferred_cancel_request_.set_freshness_ticket(authorization_.identity.freshness_ticket());
    request_=client_.submit(deferred_cancel_request_,now);cancel_request_id_=request_;
    if(!request_) { fail("STOP_FAILED"); return; }
  }
  if(completing_after_renewal_!=0U) {
    if(deadline_ && now>=deadline_) {
      completing_after_renewal_=0U; stop(false,"RESULT_UNCONFIRMED",now); return;
    }
    const auto renewal=client_.renewal_completion(completing_after_renewal_);
    if(renewal==RenewalCompletion::kPending) return;
    completing_after_renewal_=0U;
    if(renewal!=RenewalCompletion::kCommitted || !deferred_complete_matches_authority()) {
      stop(false,"RESULT_UNCONFIRMED",now); return;
    }
    deferred_complete_request_.set_freshness_ticket(authorization_.identity.freshness_ticket());
    request_=client_.submit(deferred_complete_request_,now);
    if(!request_) { fail("TRANSPORT_UNAVAILABLE"); return; }
  }
  try_completion_hold(now);
  if (hold_drain_pending_ && deadline_ && now>=deadline_) {
    if(auto* journal=dummy_ros_control::current_host_diagnostics) {
    const auto observed=journal->context().first;
    const auto& identity=authorization_.identity;
    const dummy_ros_control::DiagnosticIdentity expected{identity.expected_boot_epoch(),identity.expected_session_id(),identity.expected_reference_epoch(),epoch_,last_seq_,identity.expected_management_epoch(),authorization_.session && authorization_.reference};
    journal->reject(dummy_ros_control::DiagnosticReason::kActionHoldDrainTimeout,observed,expected,now,"HOLD_DRAIN_TIMEOUT",0U,{frozen_seq_,frozen_generation_,last_seq_,drained_generation_,drained_committed_seq_,hold_committed_seq_,state_seq_,epoch_,hold_submitted_mask_,hold_sent_mask_});
    journal->detail(
                 "GATE_HOLD_DRAIN_TIMEOUT_DIAG frozen=%u generation=%llu local=%u processed=%llu target=%u committed=%u state=%u epoch=%u submitted=%u sent=%u\n",
                 frozen_seq_,static_cast<unsigned long long>(frozen_generation_),last_seq_,
                 static_cast<unsigned long long>(drained_generation_),drained_committed_seq_,
                 hold_committed_seq_,state_seq_,epoch_,
                 hold_submitted_mask_,hold_sent_mask_);
  } else std::fprintf(stderr,
                 "GATE_HOLD_DRAIN_TIMEOUT_DIAG frozen=%u generation=%llu local=%u processed=%llu target=%u committed=%u state=%u epoch=%u submitted=%u sent=%u\n",
                 frozen_seq_,static_cast<unsigned long long>(frozen_generation_),last_seq_,
                 static_cast<unsigned long long>(drained_generation_),drained_committed_seq_,
                 hold_committed_seq_,state_seq_,epoch_,
                 hold_submitted_mask_,hold_sent_mask_);
    hold_drain_pending_=false;
    stop(false,"HOLD_DRAIN_TIMEOUT",now);
    return;
  }
  if (request_ && deadline_ && now>=deadline_) {
    if (phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold) fail("STOP_FAILED");
    else if (operation_==ManageOperation::kEnable) fail("FIRST_SAMPLE_TIMEOUT");
    else stop(false,"RESULT_UNCONFIRMED",now);
    return;
  }
  if (!request_) return;
  try { if (const auto reply=client_.poll(request_)) observe(*reply,now); }
  catch (const std::exception &) {
    if (phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold) fail("STOP_FAILED");
    else if (phase_==Phase::kRunning || phase_==Phase::kCompleting || phase_==Phase::kPrimeMotion)
      stop(false,"RESULT_UNCONFIRMED",now);
    else if (operation_==ManageOperation::kEnable) fail("ENABLE_FAILED");
    else fail("RESULT_UNCONFIRMED");
  }
}
bool ActionGate::record_update(std::uint64_t generation,std::uint32_t epoch,std::uint32_t seq,
                               const Position &q,std::uint64_t producer_generation) {
  if (generation!=generation_ || epoch!=epoch_ || !seq ||
      (last_seq_ && !sequence_newer(seq,last_seq_)) || frozen_ ||
      phase_==Phase::kStopping || phase_==Phase::kFault || phase_==Phase::kIdle || phase_==Phase::kCompleting)
    return false;
  for (std::size_t i=0;i<profile_.joints.size();++i) {
    if (!std::isfinite(q[i]) ||
        (phase_==Phase::kRunning && (q[i]<profile_.lower[i] || q[i]>profile_.upper[i]))) return false;
    if ((phase_!=Phase::kRunning || !sampled_) && std::abs(q[i]-prime_q_[i])>1e-9) return false;
  }
  if (!sampled_) {
    prime_seq_=seq;
    prime_producer_generation_=producer_generation ? producer_generation : seq;
  }
  last_q_=q; last_seq_=seq;
  last_producer_generation_=producer_generation ? producer_generation : seq;
  sampled_=true; return true;
}
void ActionGate::inner_result(std::uint64_t generation,Terminal result,const std::string &error,std::uint64_t now) {
  if (generation!=generation_ || phase_!=Phase::kRunning || terminal_!=Terminal::kNone) return;
  if (result!=Terminal::kSucceeded) { stop(false,error.empty()?"INNER_ABORT":error,now); return; }
  if (!last_seq_) { stop(false,"NO_TERMINAL_SAMPLE",now); return; }
  for (std::size_t i=0;i<profile_.joints.size();++i)
    if (std::abs(last_q_[i]-trajectory_.points.back().positions[i])>trajectory_.goal_tolerance[i]) {
      stop(false,"BAD_TERMINAL_POSITION",now); return;
    }
  port_.disable(); frozen_=true; frozen_seq_=last_seq_; frozen_q_=last_q_;
  phase_=Phase::kCompleting; deadline_=saturating_deadline(now,timeout_);
}
void ActionGate::stop(bool canceled,const std::string &error,std::uint64_t now) {
  canceling_after_renewal_=0U; deferred_cancel_request_={};
  completing_after_renewal_=0U;
  if (phase_==Phase::kFault || phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold ||
      (ready() && terminal_!=Terminal::kNone)) return;
  if(auto* journal=dummy_ros_control::current_host_diagnostics) {
    const auto observed=journal->context().first;
    const auto& identity=authorization_.identity;
    const dummy_ros_control::DiagnosticIdentity expected{identity.expected_boot_epoch(),identity.expected_session_id(),identity.expected_reference_epoch(),epoch_,last_seq_,identity.expected_management_epoch(),authorization_.session && authorization_.reference};
    journal->reject(dummy_ros_control::DiagnosticReason::kActionStop,observed,expected,now,error.c_str(),0U,{static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),request_,now},!canceled || !error.empty());
    journal->detail("GATE_STOP_DIAG phase=%u operation=%u reason=%s request=%llu now=%llu\n",
               static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),error.c_str(),
               static_cast<unsigned long long>(request_),static_cast<unsigned long long>(now));
  } else std::fprintf(stderr,"GATE_STOP_DIAG phase=%u operation=%u reason=%s request=%llu now=%llu\n",
               static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),error.c_str(),
               static_cast<unsigned long long>(request_),static_cast<unsigned long long>(now));
  port_.disable(); ++generation_; port_.cancel(); cancel_terminal_=canceled;
  error_=error; phase_=Phase::kStopping; frozen_=true; hold_drain_pending_=false;
  issue(ManageOperation::kCancel,now);
}
void ActionGate::cancel(std::uint64_t now) { if (request_ || hold_drain_pending_ || completing_after_renewal_!=0U) stop(true,"",now); }
void ActionGate::abort(const std::string &error,std::uint64_t now) {
  if(canceling_after_renewal_!=0U && request_==0U && cancel_request_id_==0U && phase_==Phase::kStopping) { fail(error.empty()?"RESULT_UNCONFIRMED":error); return; }
  if (request_ || hold_drain_pending_ || completing_after_renewal_!=0U) stop(false,error,now);
}
void ActionGate::invalidate_authorization(std::uint64_t now) {
  canceling_after_renewal_=0U; deferred_cancel_request_={};
  if (phase_==Phase::kFault && !authorization_.session) return;
  if (request_ || hold_drain_pending_ || completing_after_renewal_!=0U) stop(false,"SESSION_CHANGED",now);
  authorization_.session=false; port_.disable(); ++generation_;
  fail("SESSION_CHANGED");
}
void ActionGate::fail(const std::string &error) {
  canceling_after_renewal_=0U; deferred_cancel_request_={};
  completing_after_renewal_=0U;
  if(phase_!=Phase::kFault || error_!=error)
    if(auto* journal=dummy_ros_control::current_host_diagnostics) {
    const auto observed=journal->context().first;
    const auto& identity=authorization_.identity;
    const dummy_ros_control::DiagnosticIdentity expected{identity.expected_boot_epoch(),identity.expected_session_id(),identity.expected_reference_epoch(),epoch_,last_seq_,identity.expected_management_epoch(),authorization_.session && authorization_.reference};
    journal->reject(dummy_ros_control::DiagnosticReason::kActionFail,observed,expected,dummy_ros_control::host_diagnostic_now_us(),error.c_str(),0U,{static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),request_});
    journal->detail("GATE_FAIL_DIAG phase=%u operation=%u reason=%s request=%llu\n",
                 static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),error.c_str(),
                 static_cast<unsigned long long>(request_));
  } else std::fprintf(stderr,"GATE_FAIL_DIAG phase=%u operation=%u reason=%s request=%llu\n",
                 static_cast<unsigned>(phase_),static_cast<unsigned>(operation_),error.c_str(),
                 static_cast<unsigned long long>(request_));
  port_.disable(); port_.cancel(); ++generation_; phase_=Phase::kFault; terminal_=Terminal::kAborted;
  if (error_.empty() || error=="STOP_FAILED") error_=error;
  request_=0; deadline_=0; hold_drain_pending_=false;
}
}  // namespace dummy_motion_client
