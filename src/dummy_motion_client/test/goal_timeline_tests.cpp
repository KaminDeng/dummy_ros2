#include "dummy_motion_client/goal_timeline.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <csignal>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstdlib>

using dummy_motion_client::GoalTimeline;
using dummy_motion_client::TimelineEvent;
using dummy_motion_client::TimelineIdentity;
using dummy_motion_client::TimelineSuccessDecision;
using dummy_motion_client::timeline_success_decision;

static std::string read_all(const std::filesystem::path &path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

static TimelineEvent event(TimelineEvent::Kind kind, unsigned goal, std::uint64_t run) {
  TimelineEvent value{};
  value.kind=kind;
  value.uuid[0]=static_cast<std::uint8_t>(goal);
  value.identity=TimelineIdentity{7,8,9,10,run};
  value.steady_us=1000+goal;
  value.axis_mask=63;
  if(kind==TimelineEvent::Kind::kRelease) {
    value.release_request_id=101U;
    value.release_operation_id=201U;
  }
  return value;
}

static void goal(GoalTimeline &timeline, unsigned number) {
  assert(timeline.record(event(TimelineEvent::Kind::kAccepted,number,number-1)));
  assert(timeline.record(event(TimelineEvent::Kind::kBeforeBegin,number,number-1)));
  assert(timeline.record(event(TimelineEvent::Kind::kAfterBegin,number,number)));
  auto feedback=event(TimelineEvent::Kind::kFeedback,number,number);
  feedback.feedback_seq={1,2,3,4,5,6};
  feedback.feedback_age_us={10,11,12,13,14,15};
  assert(timeline.record(feedback));
  assert(timeline.record(event(TimelineEvent::Kind::kTerminalSuccess,number,number)));
  assert(timeline.record(event(TimelineEvent::Kind::kHold,number,number)));
}

int main() {
  const TimelineIdentity preallocated{7,8,9,1,1};
  const TimelineIdentity began{7,8,9,2,1};
  assert(!dummy_motion_client::trusted_begin_observation(preallocated,preallocated,true,true));
  assert(!dummy_motion_client::trusted_begin_observation(preallocated,began,false,true));
  assert(!dummy_motion_client::trusted_begin_observation(preallocated,began,true,false));
  assert(dummy_motion_client::trusted_begin_observation(preallocated,began,true,true));
  auto wrong_boot=began;
  wrong_boot.boot=11;
  assert(!dummy_motion_client::trusted_begin_observation(preallocated,wrong_boot,true,true));
  auto prior_run=began;
  prior_run.run_id=2;
  auto later_start=preallocated;
  later_start.run_id=3;
  assert(!dummy_motion_client::trusted_begin_observation(later_start,prior_run,true,true));
  assert(timeline_success_decision(false,false,false,false)==TimelineSuccessDecision::kSucceed);
  assert(timeline_success_decision(true,false,false,true)==TimelineSuccessDecision::kWait);
  assert(timeline_success_decision(true,false,true,false)==TimelineSuccessDecision::kWait);
  assert(timeline_success_decision(true,false,true,true)==TimelineSuccessDecision::kSucceed);
  assert(timeline_success_decision(true,true,true,true)==TimelineSuccessDecision::kEvidenceIncomplete);
  assert(timeline_success_decision(false,false,false,false,true,false,false)==TimelineSuccessDecision::kWait);
  assert(timeline_success_decision(false,false,false,false,true,false,true)==TimelineSuccessDecision::kSucceed);
  assert(timeline_success_decision(false,false,false,false,true,true,true)==TimelineSuccessDecision::kEvidenceIncomplete);
  std::atomic<bool> injected_failure{false};
  const auto before_success=[&] {
    return timeline_success_decision(false,false,false,false,true,
                                     injected_failure.load(std::memory_order_acquire),true);
  };
  assert(before_success()==TimelineSuccessDecision::kSucceed);
  injected_failure.store(true,std::memory_order_release);
  assert(before_success()==TimelineSuccessDecision::kEvidenceIncomplete);
  const auto pattern=(std::filesystem::temp_directory_path()/"dummy_goal_timeline_XXXXXX").string();
  std::vector<char> template_path(pattern.begin(),pattern.end());
  template_path.push_back('\0');
  const auto directory=::mkdtemp(template_path.data());
  assert(directory);
  const std::filesystem::path root=directory;
  const auto valid=root/"valid.csv";
  {
    GoalTimeline timeline(valid.string());
    for(unsigned i=1;i<=50;++i) goal(timeline,i);
    auto released=event(TimelineEvent::Kind::kRelease,50,0);
    released.identity.session=0;
    assert(timeline.record(released));
    timeline.close();
    assert(timeline.complete());
  }
  const auto body=read_all(valid);
  assert(body.find("COMPLETE,50")!=std::string::npos);
  assert(body.find("feedback,01000000000000000000000000000000")!=std::string::npos);
  assert(body.find("pre_release_hold,")!=std::string::npos);
  assert(body.find("post_release_status,")!=std::string::npos);
  assert(body.find("release_request_id,release_operation_id,cancel_request_id,cancel_operation_id\n")!=std::string::npos);
  assert(body.find(",101,201,0,0\nCOMPLETE,50")!=std::string::npos);
  assert(body.find(",0,0\n")!=std::string::npos);
  {
    GoalTimeline duplicate(valid.string());
    assert(!duplicate.record(event(TimelineEvent::Kind::kAccepted,1,0)));
    duplicate.close();
    assert(!duplicate.complete());
  }
  assert(read_all(valid)==body);

  const auto preallocated_path=root/"preallocated_run.csv";
  {
    GoalTimeline timeline(preallocated_path.string());
    auto accepted=event(TimelineEvent::Kind::kAccepted,1,1);
    accepted.identity.motion_epoch=1;
    assert(timeline.record(accepted));
    accepted.kind=TimelineEvent::Kind::kBeforeBegin;
    assert(timeline.record(accepted));
    auto after=event(TimelineEvent::Kind::kAfterBegin,1,1);
    after.identity.motion_epoch=2;
    assert(timeline.record(after));
    after.kind=TimelineEvent::Kind::kFeedback;
    assert(timeline.record(after));
    after.kind=TimelineEvent::Kind::kTerminalSuccess;
    assert(timeline.record(after));
    after.kind=TimelineEvent::Kind::kHold;
    assert(timeline.record(after));
    auto released=event(TimelineEvent::Kind::kRelease,1,0);
    released.identity.session=0;
    assert(timeline.record(released));
    timeline.close();
    assert(timeline.complete());
  }

  const auto canceled_path=root/"canceled.csv";
  {
    GoalTimeline timeline(canceled_path.string());
    assert(timeline.record(event(TimelineEvent::Kind::kAccepted,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kBeforeBegin,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kAfterBegin,1,1)));
    assert(timeline.record(event(TimelineEvent::Kind::kFeedback,1,1)));
    auto canceled=event(TimelineEvent::Kind::kTerminalCanceled,1,1);
    canceled.cancel_request_id=301U;
    canceled.cancel_operation_id=401U;
    assert(timeline.record(canceled));
    assert(timeline.record(event(TimelineEvent::Kind::kHold,1,1)));
    assert(timeline.record(event(TimelineEvent::Kind::kRelease,1,1)));
    timeline.close();
    assert(timeline.complete());
  }
  const auto canceled_body=read_all(canceled_path);
  assert(canceled_body.find("cancel_request_id,cancel_operation_id\n")!=std::string::npos);
  assert(canceled_body.find(",0,0,301,401\n")!=std::string::npos);
  const auto missing_cancel_path=root/"missing_cancel_key.csv";
  {
    GoalTimeline timeline(missing_cancel_path.string());
    assert(timeline.record(event(TimelineEvent::Kind::kAccepted,2,1)));
    assert(timeline.record(event(TimelineEvent::Kind::kBeforeBegin,2,1)));
    assert(timeline.record(event(TimelineEvent::Kind::kAfterBegin,2,2)));
    assert(timeline.record(event(TimelineEvent::Kind::kFeedback,2,2)));
    assert(!timeline.record(event(TimelineEvent::Kind::kTerminalCanceled,2,2)));
    timeline.close();
    assert(!timeline.complete());
  }
  const auto stale_cancel_path=root/"stale_cancel_key.csv";
  {
    GoalTimeline timeline(stale_cancel_path.string());
    assert(timeline.record(event(TimelineEvent::Kind::kAccepted,3,2)));
    assert(timeline.record(event(TimelineEvent::Kind::kBeforeBegin,3,2)));
    assert(timeline.record(event(TimelineEvent::Kind::kAfterBegin,3,3)));
    assert(timeline.record(event(TimelineEvent::Kind::kFeedback,3,3)));
    auto success=event(TimelineEvent::Kind::kTerminalSuccess,3,3);
    success.cancel_request_id=301U;
    success.cancel_operation_id=401U;
    assert(!timeline.record(success));
    timeline.close();
    assert(!timeline.complete());
  }

  const auto mixed=root/"mixed.csv";
  {
    GoalTimeline timeline(mixed.string());
    assert(timeline.record(event(TimelineEvent::Kind::kAccepted,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kBeforeBegin,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kAfterBegin,1,1)));
    assert(timeline.record(event(TimelineEvent::Kind::kFeedback,1,1)));
    auto wrong=event(TimelineEvent::Kind::kTerminalSuccess,1,2);
    wrong.identity.boot=11;
    assert(!timeline.record(wrong));
    timeline.close();
    assert(!timeline.complete());
  }
  assert(read_all(mixed).find("COMPLETE")==std::string::npos);

  const auto cross_run=root/"cross_run.csv";
  {
    GoalTimeline timeline(cross_run.string());
    assert(timeline.record(event(TimelineEvent::Kind::kAccepted,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kBeforeBegin,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kAfterBegin,1,1)));
    assert(timeline.record(event(TimelineEvent::Kind::kFeedback,1,1)));
    assert(!timeline.record(event(TimelineEvent::Kind::kTerminalSuccess,1,2)));
    timeline.close();
    assert(!timeline.complete());
  }

  const auto missing=root/"missing_hold.csv";
  {
    GoalTimeline timeline(missing.string());
    assert(timeline.record(event(TimelineEvent::Kind::kAccepted,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kBeforeBegin,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kAfterBegin,1,1)));
    assert(timeline.record(event(TimelineEvent::Kind::kFeedback,1,1)));
    assert(timeline.record(event(TimelineEvent::Kind::kTerminalSuccess,1,1)));
    timeline.close();
    assert(!timeline.complete());
  }
  assert(read_all(missing).find("COMPLETE")==std::string::npos);

  const auto no_release=root/"no_release.csv";
  {
    GoalTimeline timeline(no_release.string());
    goal(timeline,1);
    timeline.close();
    assert(!timeline.complete());
  }
  assert(read_all(no_release).find("COMPLETE")==std::string::npos);

  const auto missing_key=root/"missing_release_key.csv";
  {
    GoalTimeline timeline(missing_key.string());
    goal(timeline,1);
    auto released=event(TimelineEvent::Kind::kRelease,1,0);
    released.identity.session=0;
    released.release_operation_id=0;
    assert(!timeline.record(released));
    timeline.close();
    assert(!timeline.complete());
  }
  assert(read_all(missing_key).find("COMPLETE")==std::string::npos);

  const auto wrong_goal=root/"wrong_release_goal.csv";
  {
    GoalTimeline timeline(wrong_goal.string());
    goal(timeline,1);
    auto released=event(TimelineEvent::Kind::kRelease,2,0);
    released.identity.session=0;
    assert(!timeline.record(released));
    timeline.close();
    assert(!timeline.complete());
  }

  const auto no_feedback=root/"no_feedback.csv";
  {
    GoalTimeline timeline(no_feedback.string());
    assert(timeline.record(event(TimelineEvent::Kind::kAccepted,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kBeforeBegin,1,0)));
    assert(timeline.record(event(TimelineEvent::Kind::kAfterBegin,1,1)));
    assert(!timeline.record(event(TimelineEvent::Kind::kTerminalSuccess,1,1)));
    timeline.close();
    assert(!timeline.complete());
  }

  const auto idle=root/"idle_writer.csv";
  {
    GoalTimeline timeline(idle.string());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    goal(timeline,1);
    auto released=event(TimelineEvent::Kind::kRelease,1,0);
    released.identity.session=0;
    assert(timeline.record(released));
    while(!timeline.drained() && !timeline.failed())
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(timeline.drained());
    timeline.close();
    assert(timeline.complete());
  }
  assert(read_all(idle).find("COMPLETE,1")!=std::string::npos);

  const auto disk_failure=root/"disk_failure.csv";
  const auto child=::fork();
  assert(child>=0);
  if(child==0) {
    std::signal(SIGXFSZ,SIG_IGN);
    GoalTimeline timeline(disk_failure.string());
    const rlimit limit{300,300};
    assert(::setrlimit(RLIMIT_FSIZE,&limit)==0);
    assert(timeline.record(event(TimelineEvent::Kind::kAccepted,1,0)));
    for(unsigned i=0;i<100 && !timeline.failed();++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(timeline.failed());
    assert(!timeline.drained());
    timeline.close();
    assert(!timeline.complete());
    _exit(0);
  }
  int child_status=0;
  assert(::waitpid(child,&child_status,0)==child);
  assert(WIFEXITED(child_status) && WEXITSTATUS(child_status)==0);
  assert(read_all(disk_failure).find("COMPLETE")==std::string::npos);

  const auto saturated=root/"saturated.csv";
  {
    GoalTimeline timeline(saturated.string(),1);
    assert(!timeline.failed());
    bool accepted=timeline.record(event(TimelineEvent::Kind::kAccepted,1,0));
    if(accepted) accepted=timeline.record(event(TimelineEvent::Kind::kBeforeBegin,1,0));
    if(accepted) accepted=timeline.record(event(TimelineEvent::Kind::kAfterBegin,1,1));
    for(unsigned i=0;i<10000 && accepted;++i)
      accepted=timeline.record(event(TimelineEvent::Kind::kFeedback,1,1));
    assert(!accepted && timeline.failed());
    timeline.close();
    assert(!timeline.complete());
  }
  assert(read_all(saturated).find("COMPLETE")==std::string::npos);

  GoalTimeline broken("/this/path/does/not/exist/goal.csv");
  assert(!broken.record(event(TimelineEvent::Kind::kAccepted,1,0)));
  broken.close();
  assert(!broken.complete());
}
