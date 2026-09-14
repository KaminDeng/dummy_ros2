#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "dummy_ros_control/command_producer.hpp"

namespace dummy_ros_control {

template <std::size_t Capacity>
class BoundedPublishQueue {
 public:
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

  std::uint64_t drops() const noexcept { return drops_.load(std::memory_order_relaxed); }

 private:
  struct Slot {
    CommandGenerationRecord record{};
    std::atomic<bool> occupied{false};
  };
  std::array<Slot, Capacity> slots_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
  std::atomic<std::uint64_t> drops_{0};
};

}  // namespace dummy_ros_control
