#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <optional>

namespace dummy_ros_control {

struct CommandIdentity {
  std::uint64_t boot = 0;
  std::uint64_t session = 0;
  std::uint32_t reference_epoch = 0;
  std::uint32_t motion_epoch = 0;
  std::uint64_t ticket = 0;
};

struct CommandGenerationRecord {
  std::uint64_t generation = 0;
  CommandIdentity identity{};
  std::array<double, 6> position{};
};

class CommandProducer {
 public:
  std::uint64_t record_update(const CommandIdentity &identity,
                              const std::array<double, 6> &position) noexcept {
    const auto generation = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
    auto &slot = slots_[generation % slots_.size()];
    slot.ready.store(0, std::memory_order_relaxed);
    slot.boot.store(identity.boot, std::memory_order_relaxed);
    slot.session.store(identity.session, std::memory_order_relaxed);
    slot.reference_epoch.store(identity.reference_epoch, std::memory_order_relaxed);
    slot.motion_epoch.store(identity.motion_epoch, std::memory_order_relaxed);
    slot.ticket.store(identity.ticket, std::memory_order_relaxed);
    for (std::size_t index = 0; index < position.size(); ++index) {
      slot.position_bits[index].store(to_bits(position[index]), std::memory_order_relaxed);
    }
    slot.ready.store(generation, std::memory_order_release);
    published_generation_.store(generation, std::memory_order_release);
    return generation;
  }

  std::optional<CommandGenerationRecord> consume_after(std::uint64_t last_generation) const noexcept {
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
      const auto generation = published_generation_.load(std::memory_order_acquire);
      if (generation == 0 || generation <= last_generation) return std::nullopt;
      const auto &slot = slots_[generation % slots_.size()];
      if (slot.ready.load(std::memory_order_acquire) != generation) continue;
      CommandGenerationRecord record{};
      record.generation = generation;
      record.identity.boot = slot.boot.load(std::memory_order_relaxed);
      record.identity.session = slot.session.load(std::memory_order_relaxed);
      record.identity.reference_epoch = slot.reference_epoch.load(std::memory_order_relaxed);
      record.identity.motion_epoch = slot.motion_epoch.load(std::memory_order_relaxed);
      record.identity.ticket = slot.ticket.load(std::memory_order_relaxed);
      for (std::size_t index = 0; index < record.position.size(); ++index) {
        record.position[index] = from_bits(slot.position_bits[index].load(std::memory_order_relaxed));
      }
      if (slot.ready.load(std::memory_order_acquire) == generation) return record;
    }
    return std::nullopt;
  }

 private:
  static std::uint64_t to_bits(double value) noexcept {
    std::uint64_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
  }
  static double from_bits(std::uint64_t bits) noexcept {
    double value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }
  struct Slot {
    std::atomic<std::uint64_t> ready{0};
    std::atomic<std::uint64_t> boot{0};
    std::atomic<std::uint64_t> session{0};
    std::atomic<std::uint32_t> reference_epoch{0};
    std::atomic<std::uint32_t> motion_epoch{0};
    std::atomic<std::uint64_t> ticket{0};
    std::array<std::atomic<std::uint64_t>, 6> position_bits{};
  };
  std::array<Slot, 2> slots_{};
  std::atomic<std::uint64_t> next_generation_{0};
  std::atomic<std::uint64_t> published_generation_{0};
};

}  // namespace dummy_ros_control
