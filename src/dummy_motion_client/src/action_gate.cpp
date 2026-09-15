#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/deadline.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>
namespace dummy_motion_client {
namespace {
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
  for (double velocity:output.points.back().velocities) if (velocity!=0) return {};
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
bool ActionGate::issue(ManageOperation operation,std::uint64_t now) {
  auto request=authorization_.identity; request.set_expected_motion_epoch(epoch_);
  request.set_operation(static_cast<std::uint16_t>(operation));
  if (operation==ManageOperation::kComplete) {
    request.set_terminal_producer_seq(frozen_seq_);
    double positions[6]; std::copy(frozen_q_.begin(),frozen_q_.end(),positions);
    request.set_terminal_position_rad(positions);
  }
  operation_=operation; deadline_=saturating_deadline(now,timeout_); request_=client_.submit(request,now);
  management_evidence_={}; state_evidence_={};
  if (!request_) { fail(operation==ManageOperation::kCancel ? "STOP_FAILED" : "TRANSPORT_UNAVAILABLE"); return false; }
  return true;
}
bool ActionGate::enable(const Authorization &authorization,std::uint64_t now) {
  if (phase_!=Phase::kIdle || request_ || !authorization.session || !authorization.reference ||
      !authorization.permit || authorization.identity.expected_session_id()==0) return false;
  authorization_=authorization; return issue(ManageOperation::kEnable,now);
}
bool ActionGate::submit(const Trajectory &trajectory,std::uint64_t now) {
  if (!ready() || !authorization_.session || !authorization_.reference || !authorization_.permit) return false;
  const auto validated=validate_trajectory(trajectory,profile_); if (!validated) return false;
  trajectory_=*validated; terminal_=Terminal::kNone; error_.clear(); frozen_=false; frozen_seq_=0;
  return issue(ManageOperation::kBegin,now);
}
bool ActionGate::prime(const ManageReply &reply,Phase phase,std::uint64_t now) {
  port_.disable(); ++generation_; epoch_=reply.motion_epoch; phase_=phase;
  sampled_=false; last_seq_=0; prime_seq_=0; frozen_=false;
  prime_q_=phase==Phase::kPrimeMotion ? reply.position : reply.hold_position;
  if (phase==Phase::kPrimeMotion) {
    const auto validated=validate_trajectory(trajectory_,profile_,&prime_q_);
    if (!validated) { stop(false,"TRAJECTORY_LIMIT_VIOLATION",now); return false; }
    trajectory_=*validated;
  }
  port_.prime(prime_q_,epoch_,generation_);
  return true;
}
bool ActionGate::handshake(const ManageReply &reply) const {
  return reply.snapshot_valid && reply.motion_epoch==epoch_ && sampled_ &&
      reply.accepted_seq>=prime_seq_ && reply.result_valid && reply.succeeded;
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
}
bool ActionGate::observe_state(const std::uint8_t *bytes,std::size_t size,std::uint64_t now) {
  namespace wire=dp::protocol::dummy_robot_v1;
  wire::DummyJointStateV1 state;
  if (wire::DummyJointStateV1Codec::decode(bytes,size,&state)!=wire::CodecResult::kOk ||
      state.schema_major()!=1 || state.state_seq()<=state_seq_ ||
      state.boot_epoch()!=authorization_.identity.expected_boot_epoch() ||
      state.session_id()!=authorization_.identity.expected_session_id() ||
      state.reference_epoch()!=authorization_.identity.expected_reference_epoch() ||
      state.profile_id()!=authorization_.identity.expected_profile_id() ||
      state.configured_mask()!=authorization_.identity.configured_mask() ||
      (state.position_valid_mask() & state.configured_mask())!=state.configured_mask() ||
      !std::equal(std::begin(state.device_id()),std::end(state.device_id()),std::begin(authorization_.identity.device_id())))
    return false;
  const bool hold_valid=state.configured_mask()!=0 &&
      (state.hold_valid_mask() & state.configured_mask())==state.configured_mask();
  for (std::size_t axis=0;axis<6;++axis) {
    if (!(state.configured_mask() & (1U<<axis))) continue;
    if (!std::isfinite(state.position_rad()[axis]) ||
        (hold_valid && !std::isfinite(state.hold_position_rad()[axis]))) return false;
  }
  state_seq_=state.state_seq();
  authorization_.identity.set_expected_management_epoch(state.management_epoch());
  authorization_.identity.set_freshness_ticket(state.freshness_ticket());
  if (!request_) return true;
  if (state.core_state()==wire_value(WireCoreState::kFault)) { fail("DEVICE_FAULT"); return true; }
  ManageReply evidence; evidence.request_id=request_; evidence.snapshot_valid=true;
  evidence.motion_epoch=state.motion_epoch(); evidence.accepted_seq=state.accepted_seq_valid()?state.accepted_producer_seq():0;
  evidence.hold_ready=state.hold_ready() && hold_valid;
  evidence.hold_available=hold_valid && (state.core_state()==wire_value(WireCoreState::kEnablingHold) ||
      state.core_state()==wire_value(WireCoreState::kHold));
  // A fresh, identity-checked state transition is core acceptance evidence; QUEUED alone is not.
  evidence.accepted=(evidence.motion_epoch>epoch_ &&
      ((operation_==ManageOperation::kBegin && state.core_state()==wire_value(WireCoreState::kRunning)) ||
       (operation_!=ManageOperation::kBegin && evidence.hold_available))) ||
      (evidence.motion_epoch==epoch_ && phase_!=Phase::kIdle && phase_!=Phase::kHold);
  std::copy(std::begin(state.position_rad()),std::end(state.position_rad()),evidence.position.begin());
  std::copy(std::begin(state.hold_position_rad()),std::end(state.hold_position_rad()),evidence.hold_position.begin());
  evidence.result_valid=management_evidence_.result_valid; evidence.succeeded=management_evidence_.succeeded;
  state_evidence_=evidence; advance(evidence,now); return true;
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
void ActionGate::tick(std::uint64_t now) {
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
bool ActionGate::record_update(std::uint64_t generation,std::uint32_t epoch,std::uint32_t seq,const Position &q) {
  if (generation!=generation_ || epoch!=epoch_ || !seq || seq<=last_seq_ || frozen_ ||
      phase_==Phase::kStopping || phase_==Phase::kFault || phase_==Phase::kIdle || phase_==Phase::kCompleting)
    return false;
  for (std::size_t i=0;i<profile_.joints.size();++i) {
    if (!std::isfinite(q[i]) || q[i]<profile_.lower[i] || q[i]>profile_.upper[i]) return false;
    if ((phase_!=Phase::kRunning || !sampled_) && std::abs(q[i]-prime_q_[i])>1e-9) return false;
  }
  if (!sampled_) prime_seq_=seq;
  last_q_=q; last_seq_=seq; sampled_=true; return true;
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
  if (phase_==Phase::kFault || phase_==Phase::kStopping || phase_==Phase::kPrimeStopHold ||
      (ready() && terminal_!=Terminal::kNone)) return;
  port_.disable(); ++generation_; port_.cancel(); cancel_terminal_=canceled;
  error_=error; phase_=Phase::kStopping; frozen_=true;
  issue(ManageOperation::kCancel,now);
}
void ActionGate::cancel(std::uint64_t now) { if (request_) stop(true,"",now); }
void ActionGate::abort(const std::string &error,std::uint64_t now) { if (request_) stop(false,error,now); }
void ActionGate::invalidate_authorization(std::uint64_t now) {
  if (request_) stop(false,"SESSION_CHANGED",now);
  authorization_.session=false; port_.disable(); ++generation_;
  fail("SESSION_CHANGED");
}
void ActionGate::fail(const std::string &error) {
  port_.disable(); port_.cancel(); ++generation_; phase_=Phase::kFault; terminal_=Terminal::kAborted;
  if (error_.empty() || error=="STOP_FAILED") error_=error;
  request_=0; deadline_=0;
}
}  // namespace dummy_motion_client
