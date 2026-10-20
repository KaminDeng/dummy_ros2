/**
 * @file native_wait_policy.hpp
 * @brief Native Action 单 worker 的有界等待、取消双证据及故障锁。
 * @details 只有 phase_ 是原子；deadline_、cancel_ack_、terminal_ 必须由同一 action worker 顺序访问。原子 admission 不提供整个对象的并发安全。
 */
#pragma once
#include <atomic>
#include "dummy_motion_client/deadline.hpp"
namespace dummy_motion_client {
/**
 * @brief 用 Idle 原子占槽避免多个动作同时进入；未确认退出会锁 Fault。
 */
class NativeWaitPolicy {
 public:
  /**
   * @brief Admitted 仅占槽；Waiting 等终态；Canceling 要同时等调用方取消响应标志与终态；Fault 不自动解锁。
   */
  enum class Phase { kIdle, kAdmitted, kWaiting, kCanceling, kFault };
  /**
   * @brief CAS 将 Idle 变为 Admitted；同一时刻只有一个竞争者成功。
   */
  bool try_acquire() {
    auto expected=Phase::kIdle;
    return phase_.compare_exchange_strong(expected,Phase::kAdmitted);
  }
  /**
   * @brief 非 Idle 都占用入口，包括已锁 Fault 的情况。
   */
  bool busy() const { return phase_.load()!=Phase::kIdle; }
  /**
   * @brief 查询原子阶段是否为 Fault，不读取非原子等待字段。
   */
  bool faulted() const { return phase_.load()==Phase::kFault; }
  /**
   * @brief 单 worker 清空上次双证据，设置饱和截止时间并进入 Waiting。
   * @details 除 Fault 外不检查原阶段；调用方应先成功取得 admission。now/budget 单位微秒。
   */
  void begin(std::uint64_t now,std::uint64_t budget) {
    if (faulted()) return;
    cancel_ack_=false; terminal_=false;
    deadline_=saturating_deadline(now,budget); phase_.store(Phase::kWaiting);
  }
  /**
   * @brief 首次进入 Canceling 时设置取消预算，重复取消保留原截止时间。
   * @details 不清已有 terminal/ack，因此观察到的有效终态可以先于取消回执到达；Fault 时不改动。
   */
  void cancel(std::uint64_t now,std::uint64_t budget) {
    if (faulted() || phase_.load()==Phase::kCanceling) return;
    deadline_=saturating_deadline(now,budget); phase_.store(Phase::kCanceling);
  }
  /**
   * @brief 记录调用方报告的取消响应标志；单独不足以完成取消。
   * @details 当前Native入口只核对响应非空，不检查取消接受码/目标列表；本类也不额外验证。
   */
  void cancel_acknowledged() { if (!faulted()) cancel_ack_=true; }
  /**
   * @brief 记录已有终态；调用方负责验证该终态属于当前动作。
   */
  void terminal_confirmed() { if (!faulted()) terminal_=true; }
  /**
   * @brief Canceling 阶段同时具备调用方取消响应标志与终态才返回 true。
   */
  bool cancel_complete() const { return phase_.load()==Phase::kCanceling && cancel_ack_ && terminal_; }
  /**
   * @brief now 达到或超过 deadline 时为超时；不检查当前阶段或自动进入 Fault。
   */
  bool expired(std::uint64_t now) const { return now>=deadline_; }
  /**
   * @brief 无条件锁 Fault；后续 begin/cancel/finish 都不能把它恢复到 Idle。
   */
  void latch_fault() { phase_.store(Phase::kFault); }
  /**
   * @brief 根据既有证据释放占槽；未确认收尾则 Fault。
   * @details Admitted 尚未启动可直接退回 Idle；Waiting 必须有 terminal，Canceling 必须双证据。已经 Idle 时再次调用不属于合法收尾，当前实现会锁 Fault。
   */
  void finish() {
    const auto phase=phase_.load();
    if (phase==Phase::kFault) return;
    const bool confirmed=phase==Phase::kAdmitted ||
        (phase==Phase::kWaiting && terminal_) || cancel_complete();
    phase_.store(confirmed?Phase::kIdle:Phase::kFault);
  }
 private:
  /**
   * @brief 可跨 admission/worker 查询的原子阶段，其余字段仍为单 worker 所有。
   */
  std::atomic<Phase> phase_{Phase::kIdle};
  // Deadline methods run on the single action worker; admission is atomic.
  std::uint64_t deadline_=0;
  bool cancel_ack_=false, terminal_=false;
};
}  // namespace dummy_motion_client
