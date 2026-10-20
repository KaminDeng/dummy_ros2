/**
 * @file bounded_publish_queue.hpp
 * @brief 在实时转交区间与通信worker之间保存有限条控制代记录。
 * 预期单生产者/单消费者：GuardedZenbeddedSystem 用 transfer_busy 将控制线程和worker的入队尝试串行化，worker独占出队。队满立即拒绝并计数，绝不阻塞等待网络或覆盖尚未消费的槽。
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "dummy_ros_control/command_producer.hpp"

namespace dummy_ros_control {

/**
 * @brief 按固定容量分配的SPSC发布队列。
 * @tparam Capacity 编译期槽数，调用方必须保证大于零；本工程使用4槽。槽record不是原子，occupied的release/acquire发布与回收负责交接可见性。
 */
template <std::size_t Capacity>
class BoundedPublishQueue {
 public:
  /**
   * @brief 尝试复制一条完整控制记录到下一空槽。
   * @param record 身份、代号与六轴rad目标的值副本。
   * @return true入队；占用槽未回收时false并递增drops。
   */
  bool try_push(const CommandGenerationRecord &record) noexcept {
    const auto head = head_.load(std::memory_order_relaxed);
    auto &slot = slots_[head % Capacity];
    if (slot.occupied.load(std::memory_order_acquire)) {
      drops_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    slot.record = record;
    slot.occupied.store(true, std::memory_order_release);
    head_.store(head + 1, std::memory_order_relaxed);
    return true;
  }

  /**
   * @brief 尝试取出队首并归还槽。
   * @param record 非空输出指针；成功时复制记录。
   * @return false表示空队列或空指针，不改输出。
   */
  bool try_pop(CommandGenerationRecord *record) noexcept {
    if (!record) return false;
    const auto tail = tail_.load(std::memory_order_relaxed);
    auto &slot = slots_[tail % Capacity];
    if (!slot.occupied.load(std::memory_order_acquire)) return false;
    *record = slot.record;
    slot.occupied.store(false, std::memory_order_release);
    tail_.store(tail + 1, std::memory_order_relaxed);
    return true;
  }

  /**
   * @brief 读取累计入队拒绝数。
   * @return 次数；仅计槽占用导致的拒绝，不是设备丢包数。
   */
  std::uint64_t drops() const noexcept { return drops_.load(std::memory_order_relaxed); }

 private:
  /**
   * @brief 一个值记录及其发布标志；生产者写record后置occupied，消费者复制后清标志。
   */
  struct Slot {
    CommandGenerationRecord record{};
    std::atomic<bool> occupied{false};
  };
  std::array<Slot, Capacity> slots_{};
  /**
   * @brief head/tail是宿主累计入出队位置，按Capacity取模定位；drops为整个队列对象生命周期累计次数。
   */
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
  std::atomic<std::uint64_t> drops_{0};
};

}  // namespace dummy_ros_control
