/**
 * @file goal_timeline.hpp
 * @brief 目标事件时间线、Begin因果判据与异步CSV记录器接口。
 * @details record由单一生产者调用，writer独占文件写入；正常路径每行fflush后推进tail；写入失败仍消费，但failed阻止drained成功。drained只说明当前队列写完，不等同close完整性，更不表示fsync持久化或机械动作已验证。
 */
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

/**
 * @brief 事件绑定的boot/session/reference/motion/run身份；各字段必须按实际事件填写。
 */
struct TimelineIdentity {
  std::uint64_t boot=0, session=0;
  std::uint32_t reference_epoch=0, motion_epoch=0;
  std::uint64_t run_id=0;
};
/**
 * @brief 识别同boot/session/reference下、运动代次恰加1且run非退后的已确认Begin。
 * @param state_running 调用方已确认State处于运行。
 * @param begin_confirmed 调用方已确认Begin事务。
 * @return 同时满足上层事实和字段关系才为true；本函数不读取设备，也不要求reference非零。
 */
inline bool trusted_begin_observation(const TimelineIdentity &before,
                                      const TimelineIdentity &after,
                                      bool state_running,bool begin_confirmed) noexcept {
  return state_running && begin_confirmed && before.boot!=0U && before.session!=0U &&
    before.boot==after.boot && before.session==after.session &&
    before.reference_epoch==after.reference_epoch && before.motion_epoch!=UINT32_MAX &&
    after.motion_epoch==before.motion_epoch+1U && after.run_id!=0U &&
    after.run_id>=before.run_id;
}

/**
 * @brief 故障收尾记录所需证明的汇总；record仍会验证白名单故障、六轴、链和代次关系。
 * @details terminal_chain为末页链摘要；若lost或motion_complete为true，则不属于当前故障清理的认可路径。current_fault_clear_verified表示当前故障事实已按调用方证据核对，不是这里发出ClearFault命令。
 */
struct FaultCleanupTimelineProof {
  std::uint64_t boot=0,session=0,run=0;
  std::uint32_t attempt_epoch=0,active_epoch=0,stop_epoch=0;
  std::uint16_t failure_source=0;
  std::uint8_t axis_mask=0,final_state=0;
  std::array<std::uint8_t,32> terminal_chain{};
  bool full_groups_valid=false,chain_valid=false,release_committed=false;
  bool lost=false,motion_complete=false,current_fault_clear_verified=false;
};
/**
 * @brief 单条按值入队的事件；feedback_age_us为六轴微秒龄期，steady_us为Host单调时间。
 * @details uuid关联当前目标，trajectory_sha256在后继事件中由accepted记录覆盖。Release和Canceled各有独立request/operation编号，不能混用。
 */
struct TimelineEvent {
  /**
   * @brief 接纳→Begin前后→反馈→终态→保持/故障清理，以及最终释放；实际允许顺序由record检查。
   */
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

/**
 * @brief 对外成功判定：继续等写入、允许成功、或证据链已失败。
 */
enum class TimelineSuccessDecision { kWait, kSucceed, kEvidenceIncomplete };
/**
 * @brief 合并可选时间线和raw证据写入条件，不提前报成功。
 * @details 任一failed即EvidenceIncomplete，即便对应configured为false；已配置时间线须有hold且writer排空，已配置raw须terminal已flush。均满足才Succeed；未配置时不自行验证运动。
 */
inline TimelineSuccessDecision timeline_success_decision(bool timeline_configured,bool timeline_failed,
                                                         bool hold_recorded,bool writer_drained,
                                                         bool raw_configured=false,bool raw_failed=false,
                                                         bool raw_terminal_flushed=false) noexcept {
  if(timeline_failed || raw_failed) return TimelineSuccessDecision::kEvidenceIncomplete;
  if((timeline_configured && (!hold_recorded || !writer_drained)) ||
     (raw_configured && !raw_terminal_flushed)) return TimelineSuccessDecision::kWait;
  return TimelineSuccessDecision::kSucceed;
}

/**
 * @brief 有界单生产者/单消费者记录器；非法事件、环满或IO失败永久置failed。
 * @details 创建O_EXCL文件防覆盖；析构调用close并join。禁止复制以避免多个owner共享FILE、线程和索引。
 */
class GoalTimeline final {
 public:
  /**
   * @brief 以0600权限新建CSV并启动writer；空路径、零容量或IO失败会留下failed对象。
   * @param path 必须尚不存在的目标文件。
   * @param capacity 环中可容纳的最大事件数，默认2048；构造会分配存储并可能因分配/线程创建抛异常。
   */
  explicit GoalTimeline(const std::string &path, std::size_t capacity=2048);
  /**
   * @brief 结束记录并等待writer退出；不应在实时控制回调中析构。
   */
  ~GoalTimeline();
  /**
   * @brief 禁止复制构造，保持单一文件和线程所有权。
   */
  GoalTimeline(const GoalTimeline &)=delete;
  /**
   * @brief 禁止复制赋值，避免异步writer引用被替换。
   */
  GoalTimeline &operator=(const GoalTimeline &)=delete;
  /**
   * @brief 单生产者校验事件顺序/身份后按值入环，唤醒writer。
   * @return 已入队为true；不是该行已写入。非法顺序、关闭或满环为false并锁failed，不覆盖旧事件。
   */
  bool record(TimelineEvent event) noexcept;
  /**
   * @brief 单次关停、排空并join；检查至少一个目标已以保持或认可故障清理收尾，且最终有释放或故障清理。
   * @details 成功才写COMPLETE行并设置complete；fflush/fclose错误会置failed，不调用fsync。调用方应先停止生产，避免与record并发。
   */
  void close();
  /**
   * @brief close结束且没有失败时为true；运行中队列排空不会设置它。
   */
  bool complete() const noexcept { return complete_.load(); }
  /**
   * @brief 查询永久失败锁，涵盖事件、容量及写入/关闭错误。
   */
  bool failed() const noexcept { return failed_.load(); }
  /**
   * @brief acquire观察tail==head且未失败；消费者正常写入/fflush后发布tail，IO失败时仍消费但锁failed。
   * @details 只是当前快照，后续record仍可新增事件；不代表最终release或COMPLETE存在。
   */
  bool drained() const noexcept {
    const auto tail=tail_.load(std::memory_order_acquire);
    return tail==head_.load(std::memory_order_acquire) &&
      !failed_.load(std::memory_order_acquire);
  }
 private:
  /**
   * @brief 独立writer按环序逐行写CSV并fflush，关闭后继续处理余项直至空环。
   */
  void write_loop();
  /**
   * @brief 只比较boot/session/reference；motion_epoch和run必须由各事件分支另外检查。
   */
  static bool same_authority(const TimelineIdentity &a,const TimelineIdentity &b) noexcept;
  /**
   * @brief 构造初始化、writer写入、close在join后关闭；失败对象也可能已持有文件。
   */
  std::FILE *output_=nullptr;
  /**
   * @brief 固定环容量；head-tail是单调累计数的差，不预留一个空槽。
   */
  const std::size_t capacity_;
  /**
   * @brief 仅用于writer等待，不保护record的状态机字段或使其可多生产者调用。
   */
  std::mutex wait_mutex_;
  std::condition_variable cv_;
  /**
   * @brief 构造时分配的事件环；head/tail的release/acquire转交槽所有权。
   */
  std::vector<TimelineEvent> ring_;
  /**
   * @brief head由生产者发布，tail由writer在落到stdio流后发布。
   */
  std::atomic<std::size_t> head_{0}, tail_{0};
  /**
   * @brief 唯一写线程；close必须join后才关闭output。
   */
  std::thread writer_;
  /**
   * @brief 跨线程失败锁及完整关闭标记；complete仅close最终设置。
   */
  std::atomic<bool> failed_{false}, complete_{false};
  /**
   * @brief 关停请求；置位后record不再接纳，writer仍排空已有队列。
   */
  std::atomic<bool> closing_{false};
  /**
   * @brief 单生产者事件状态：当前目标及Begin前、Begin后、反馈是否已记录。
   */
  bool active_=false, before_=false, after_=false, feedback_=false;
  /**
   * @brief 终态、最终释放及故障清理标志；用于拒绝缺证据或重复关闭。
   */
  bool terminal_=false, release_recorded_=false, fault_cleanup_recorded_=false, terminal_aborted_=false;
  /**
   * @brief 已以Hold或认可FaultCleanup关闭的目标数，不是仅接纳目标数。
   */
  std::size_t goals_=0;
  /**
   * @brief 当前/最近accepted事件，后继事件沿用其UUID和轨迹摘要。
   */
  TimelineEvent current_{};
  /**
   * @brief Begin前授权身份和Begin后run身份，用于跨阶段关联。
   */
  TimelineIdentity before_identity_{}, run_identity_{};
};

}  // namespace dummy_motion_client
