/**
 * @file goal_timeline.cpp
 * @brief 目标CSV事件的因果校验、SPSC缓冲与独立写线程实现。
 * @details 单生产者推进事件状态机；writer正常以fflush完成每行后发布tail；写失败仍消费并锁failed。既有失败不清除，close只在完整目标及释放/认可故障清理存在时输出COMPLETE。
 */
#include <algorithm>
#include "dummy_motion_client/goal_timeline.hpp"

#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <utility>

namespace dummy_motion_client {
namespace {
/**
 * @brief 将事件类型映射为稳定CSV标签；未识别枚举输出invalid。
 */
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
    case TimelineEvent::Kind::kFaultCleanup: return "fault_cleanup_confirmed";
  }
  return "invalid";
}
/**
 * @brief 输出每字节两位十六进制，再恢复十进制；用于UUID与轨迹摘要。
 */
template<std::size_t N> void hex(std::ostream &output,const std::array<std::uint8_t,N> &bytes) {
  output << std::hex << std::setfill('0');
  for(const auto byte:bytes) output << std::setw(2) << static_cast<unsigned>(byte);
  output << std::dec;
}
/**
 * @brief 序列化单事件；只在对应类型列输出release/cancel编号，其余写0。
 * @details FaultCleanup行额外附加attempt/active/stop代次、故障源和32个链字节；基础CSV头不增加这些扩展列。
 */
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
  if(event.kind==TimelineEvent::Kind::kFaultCleanup) {
    const auto &p=event.fault_cleanup;
    output << ',' << p.attempt_epoch << ',' << p.active_epoch << ',' << p.stop_epoch << ',' << p.failure_source;
    for(auto b:p.terminal_chain) output << ',' << static_cast<unsigned>(b);
  }
  output << '\n';
}
}  // namespace

/**
 * @brief 创建全新文件、写表头并flush后启动线程；任何文件步骤失败都会锁failed。
 * @details open使用O_EXCL防覆盖既有证据，0600限制权限；fdopen失败时自行close原fd。
 */
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

/**
 * @brief RAII收尾，可能阻塞等writer排空和文件关闭。
 */
GoalTimeline::~GoalTimeline() { close(); }

/**
 * @brief 只检查boot/session/reference等值，run与motion的约束由事件分支承担。
 */
bool GoalTimeline::same_authority(const TimelineIdentity &a,const TimelineIdentity &b) noexcept {
  return a.boot==b.boot && a.session==b.session && a.reference_epoch==b.reference_epoch;
}

/**
 * @brief 先验证取消收据成对出现和容量，再按当前目标阶段接纳事件。
 * @details 状态机字段只有此生产者读写；ring由head的release发布给writer。false会保留失败，不能靠后续合法事件洗成完整证据。
 */
bool GoalTimeline::record(TimelineEvent event) noexcept {
  if(failed_.load()) return false;
  /**
   * @brief 取消终态必须同时携带request和operation；非取消事件禁止携带这两个编号。
   */
  const bool canceled=event.kind==TimelineEvent::Kind::kTerminalCanceled;
  if((event.cancel_request_id!=0U || event.cancel_operation_id!=0U) != canceled ||
     (canceled && (event.cancel_request_id==0U || event.cancel_operation_id==0U))) {
    failed_=true;
    return false;
  }
  /**
   * @brief 不阻塞等writer；达到capacity就锁失败，绝不覆盖未写旧事件。
   */
  const auto head=head_.load(std::memory_order_relaxed);
  if(closing_.load() || head-tail_.load(std::memory_order_acquire)>=capacity_) {
    failed_=true; return false;
  }
  const auto kind=event.kind;
  bool valid=false;
  /**
   * @brief 一个目标收尾进入保持后可接纳下个目标；最终Release/FaultCleanup之后不再新建目标。
   */
  if(kind==TimelineEvent::Kind::kAccepted) {
    valid=!active_ && !release_recorded_ && !fault_cleanup_recorded_ && event.identity.boot!=0 && event.identity.session!=0;
    if(valid) { current_=event; active_=true; before_=after_=feedback_=terminal_=false; }
  } else if(active_ && event.uuid==current_.uuid) {
    event.trajectory_sha256=current_.trajectory_sha256;
    /**
     * @brief 同UUID下保留原轨迹摘要；BeforeBegin绑定accepted授权，AfterBegin建立本run身份。
     */
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
              kind==TimelineEvent::Kind::kHold || kind==TimelineEvent::Kind::kFaultCleanup) {
      valid=after_ && same_authority(run_identity_,event.identity) &&
        run_identity_.run_id==event.identity.run_id && !terminal_;
      /**
       * @brief 故障清理是Aborted后的专用闭合路径：只认六轴指定故障、连续代次、有效末链与已确认释放；不伪装成功运动。
       */
      if(kind==TimelineEvent::Kind::kFaultCleanup) {
        const auto &p=event.fault_cleanup;
        valid=after_ && terminal_ && terminal_aborted_ && same_authority(run_identity_,event.identity) &&
          run_identity_.run_id==event.identity.run_id && event.identity.motion_epoch==run_identity_.motion_epoch &&
          p.boot==run_identity_.boot && p.session==run_identity_.session && p.run==run_identity_.run_id &&
          p.active_epoch==run_identity_.motion_epoch && p.attempt_epoch!=0U && p.active_epoch>=p.attempt_epoch &&
          p.active_epoch-p.attempt_epoch<=1U && p.active_epoch!=UINT32_MAX && p.stop_epoch==p.active_epoch+1U &&
          p.final_state==4U && (p.failure_source==0x1003U || p.failure_source==0x100aU) && p.axis_mask==63U && p.full_groups_valid && p.chain_valid &&
          p.release_committed && !p.lost && !p.motion_complete && p.current_fault_clear_verified &&
          std::any_of(p.terminal_chain.begin(),p.terminal_chain.end(),[](auto b){return b!=0U;}) &&
          event.status_core_state==9U && event.state_core_state==9U && event.axis_mask==63U &&
          event.release_request_id==0U && event.release_operation_id==0U;
        if(valid) {active_=false;fault_cleanup_recorded_=true;++goals_;}
      /**
       * @brief 终态后的保持关闭当前目标；普通终态必须已有反馈，Hold本身不新增物理到位检查。
       */
      } else if(kind==TimelineEvent::Kind::kHold) {
        valid=after_ && terminal_ && same_authority(run_identity_,event.identity) &&
          run_identity_.run_id==event.identity.run_id;
        if(valid) { active_=false; ++goals_; }
      } else if(kind==TimelineEvent::Kind::kFeedback) {
        if(valid) feedback_=true;
      } else {
        valid=valid && feedback_;
        if(valid) {terminal_=true;terminal_aborted_=kind==TimelineEvent::Kind::kTerminalAborted;}
      }
    } else if(kind==TimelineEvent::Kind::kRelease) {
      valid=false;
    }
  /**
   * @brief 只有目标不再active且至少已关闭一个目标时接纳最终Release；必须有非零事务编号。
   */
  } else if(kind==TimelineEvent::Kind::kRelease && !active_ && goals_!=0 &&
            !release_recorded_ && event.uuid==current_.uuid) {
    event.trajectory_sha256=current_.trajectory_sha256;
    valid=event.release_request_id!=0U && event.release_operation_id!=0U &&
      (same_authority(run_identity_,event.identity) ||
       (run_identity_.boot==event.identity.boot && event.identity.session==0));
    if(valid) release_recorded_=true;
  }
  if(!valid) { failed_=true; return false; }
  /**
   * @brief 写槽完成后release发布head，writer用acquire观察；notify仅用于加快唤醒。
   */
  ring_[head%capacity_]=std::move(event);
  head_.store(head+1,std::memory_order_release);
  cv_.notify_one();
  return true;
}

/**
 * @brief writer轮询/有界等待事件，按顺序写行和flush，再release发布tail。
 * @details 写入失败仍推进消费，failed保持；关闭请求到来后，已入队事件仍全部消费。
 */
void GoalTimeline::write_loop() {
  for(;;) {
    TimelineEvent event;
    auto tail=tail_.load(std::memory_order_relaxed);
    /**
     * @brief 空环最多等待10ms再检查；谓词同时识别关停与新数据，避免依赖一次notify。
     */
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
    /**
     * @brief 先按值取出槽，再构造CSV；正常写入并fflush后推进tail；fwrite失败会短路刷新且仍消费，failed阻止drained成功。
     */
    event=ring_[tail%capacity_];
    std::ostringstream line;
    row(line,event);
    const auto bytes=line.str();
    if(std::fwrite(bytes.data(),1,bytes.size(),output_)!=bytes.size() ||
       std::fflush(output_)!=0) failed_=true;
    tail_.store(tail+1,std::memory_order_release);
  }
}

/**
 * @brief 幂等关停：只由第一次exchange调用者检查收尾、join、写COMPLETE并关文件。
 * @details 无目标、仍active或缺最终释放/认可故障清理均失败。COMPLETE反映本记录器检查，不代替raw链、设备或机械验收。
 */
void GoalTimeline::close() {
  if(closing_.exchange(true)) return;
  if(active_ || goals_==0 || (!release_recorded_ && !fault_cleanup_recorded_)) failed_=true;
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
