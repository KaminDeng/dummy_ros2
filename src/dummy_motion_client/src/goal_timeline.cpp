#include "dummy_motion_client/goal_timeline.hpp"

#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <utility>

namespace dummy_motion_client {
namespace {
const char *name(TimelineEvent::Kind kind) {
  switch(kind) {
    case TimelineEvent::Kind::kAccepted: return "accepted";
    case TimelineEvent::Kind::kBeforeBegin: return "before_begin";
    case TimelineEvent::Kind::kAfterBegin: return "after_begin";
    case TimelineEvent::Kind::kFeedback: return "feedback";
    case TimelineEvent::Kind::kTerminalSuccess: return "terminal_success";
    case TimelineEvent::Kind::kTerminalCanceled: return "terminal_canceled";
    case TimelineEvent::Kind::kTerminalAborted: return "terminal_aborted";
    case TimelineEvent::Kind::kHold: return "pre_release_hold";
    case TimelineEvent::Kind::kRelease: return "post_release_status";
  }
  return "invalid";
}
template<std::size_t N> void hex(std::ostream &output,const std::array<std::uint8_t,N> &bytes) {
  output << std::hex << std::setfill('0');
  for(const auto byte:bytes) output << std::setw(2) << static_cast<unsigned>(byte);
  output << std::dec;
}
void row(std::ostream &output,const TimelineEvent &event) {
  output << name(event.kind) << ',';
  hex(output,event.uuid); output << ',';
  hex(output,event.trajectory_sha256);
  const auto &id=event.identity;
  output << ',' << event.steady_us << ',' << id.boot << ',' << id.session << ','
         << id.reference_epoch << ',' << id.motion_epoch << ',' << id.run_id << ','
         << static_cast<unsigned>(event.axis_mask) << ',' << event.status_core_state << ','
         << event.state_core_state;
  for(const auto value:event.feedback_seq) output << ',' << value;
  for(const auto value:event.feedback_age_us) output << ',' << value;
  output << ',' << (event.kind==TimelineEvent::Kind::kRelease ? event.release_request_id : 0U)
         << ',' << (event.kind==TimelineEvent::Kind::kRelease ? event.release_operation_id : 0U)
         << ',' << (event.kind==TimelineEvent::Kind::kTerminalCanceled ? event.cancel_request_id : 0U)
         << ',' << (event.kind==TimelineEvent::Kind::kTerminalCanceled ? event.cancel_operation_id : 0U);
  output << '\n';
}
}  // namespace

GoalTimeline::GoalTimeline(const std::string &path,std::size_t capacity)
    :capacity_(capacity),ring_(capacity) {
  if(path.empty() || capacity_==0) { failed_=true; return; }
  const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
  if(fd<0) { failed_=true; return; }
  output_=::fdopen(fd,"w");
  if(!output_) { ::close(fd); failed_=true; return; }
  if(std::fputs("event,goal_uuid,trajectory_sha256,steady_us,boot,session,reference_epoch,motion_epoch,run_id,axis_mask"
                ",status_core_state,state_core_state,feedback_seq_0,feedback_seq_1,feedback_seq_2,feedback_seq_3,feedback_seq_4,feedback_seq_5"
                ",feedback_age_us_0,feedback_age_us_1,feedback_age_us_2,feedback_age_us_3,feedback_age_us_4,feedback_age_us_5"
                ",release_request_id,release_operation_id,cancel_request_id,cancel_operation_id\n",output_)<0) {
    failed_=true; return;
  }
  if(std::fflush(output_)!=0) { failed_=true; return; }
  writer_=std::thread([this]{ write_loop(); });
}

GoalTimeline::~GoalTimeline() { close(); }

bool GoalTimeline::same_authority(const TimelineIdentity &a,const TimelineIdentity &b) noexcept {
  return a.boot==b.boot && a.session==b.session && a.reference_epoch==b.reference_epoch;
}

bool GoalTimeline::record(TimelineEvent event) noexcept {
  if(failed_.load()) return false;
  const bool canceled=event.kind==TimelineEvent::Kind::kTerminalCanceled;
  if((event.cancel_request_id!=0U || event.cancel_operation_id!=0U) != canceled ||
     (canceled && (event.cancel_request_id==0U || event.cancel_operation_id==0U))) {
    failed_=true;
    return false;
  }
  const auto head=head_.load(std::memory_order_relaxed);
  if(closing_.load() || head-tail_.load(std::memory_order_acquire)>=capacity_) {
    failed_=true; return false;
  }
  const auto kind=event.kind;
  bool valid=false;
  if(kind==TimelineEvent::Kind::kAccepted) {
    valid=!active_ && !release_recorded_ && event.identity.boot!=0 && event.identity.session!=0;
    if(valid) { current_=event; active_=true; before_=after_=feedback_=terminal_=false; }
  } else if(active_ && event.uuid==current_.uuid) {
    event.trajectory_sha256=current_.trajectory_sha256;
    if(kind==TimelineEvent::Kind::kBeforeBegin) {
      valid=!before_ && same_authority(current_.identity,event.identity);
      if(valid) { before_=true; before_identity_=event.identity; }
    } else if(kind==TimelineEvent::Kind::kAfterBegin) {
      valid=before_ && !after_ && same_authority(before_identity_,event.identity) &&
        event.identity.run_id!=0 && event.identity.run_id>=before_identity_.run_id &&
        (event.identity.run_id!=before_identity_.run_id ||
         trusted_begin_observation(before_identity_,event.identity,true,true));
      if(valid) { after_=true; run_identity_=event.identity; }
    } else if(kind==TimelineEvent::Kind::kFeedback || kind==TimelineEvent::Kind::kTerminalSuccess ||
              kind==TimelineEvent::Kind::kTerminalCanceled || kind==TimelineEvent::Kind::kTerminalAborted ||
              kind==TimelineEvent::Kind::kHold) {
      valid=after_ && same_authority(run_identity_,event.identity) &&
        run_identity_.run_id==event.identity.run_id && !terminal_;
      if(kind==TimelineEvent::Kind::kHold) {
        valid=after_ && terminal_ && same_authority(run_identity_,event.identity) &&
          run_identity_.run_id==event.identity.run_id;
        if(valid) { active_=false; ++goals_; }
      } else if(kind==TimelineEvent::Kind::kFeedback) {
        if(valid) feedback_=true;
      } else {
        valid=valid && feedback_;
        if(valid) terminal_=true;
      }
    } else if(kind==TimelineEvent::Kind::kRelease) {
      valid=false;
    }
  } else if(kind==TimelineEvent::Kind::kRelease && !active_ && goals_!=0 &&
            !release_recorded_ && event.uuid==current_.uuid) {
    event.trajectory_sha256=current_.trajectory_sha256;
    valid=event.release_request_id!=0U && event.release_operation_id!=0U &&
      (same_authority(run_identity_,event.identity) ||
       (run_identity_.boot==event.identity.boot && event.identity.session==0));
    if(valid) release_recorded_=true;
  }
  if(!valid) { failed_=true; return false; }
  ring_[head%capacity_]=std::move(event);
  head_.store(head+1,std::memory_order_release);
  cv_.notify_one();
  return true;
}

void GoalTimeline::write_loop() {
  for(;;) {
    TimelineEvent event;
    auto tail=tail_.load(std::memory_order_relaxed);
    if(tail==head_.load(std::memory_order_acquire)) {
      std::unique_lock<std::mutex> lock(wait_mutex_);
      cv_.wait_for(lock,std::chrono::milliseconds(10),[this]{ return closing_.load() ||
        tail_.load(std::memory_order_relaxed)!=head_.load(std::memory_order_acquire); });
      tail=tail_.load(std::memory_order_relaxed);
      if(tail==head_.load(std::memory_order_acquire)) {
        if(closing_.load()) break;
        continue;
      }
    }
    event=ring_[tail%capacity_];
    std::ostringstream line;
    row(line,event);
    const auto bytes=line.str();
    if(std::fwrite(bytes.data(),1,bytes.size(),output_)!=bytes.size() ||
       std::fflush(output_)!=0) failed_=true;
    tail_.store(tail+1,std::memory_order_release);
  }
}

void GoalTimeline::close() {
  if(closing_.exchange(true)) return;
  if(active_ || goals_==0 || !release_recorded_) failed_=true;
  cv_.notify_one();
  if(writer_.joinable()) writer_.join();
  if(output_) {
    if(!failed_ && std::fprintf(output_,"COMPLETE,%zu\n",goals_)<0) failed_=true;
    if(std::fflush(output_)!=0) failed_=true;
    if(std::fclose(output_)!=0) failed_=true;
    output_=nullptr;
  }
  complete_=!failed_;
}
}  // namespace dummy_motion_client
