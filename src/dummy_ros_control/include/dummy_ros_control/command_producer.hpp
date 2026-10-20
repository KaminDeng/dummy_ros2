/**
 * @file command_producer.hpp
 * @brief 以两槽原子快照保存最近一次控制更新及其授权代号。
 * 这是最新值快照，不是逐点可靠FIFO。六轴position单位rad，未使用轴可为NaN；generation是宿主64位控制更新代数，线producer_seq只取低32位，不能把宿主代数当作设备完成回执。
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <optional>

namespace dummy_ros_control {

// Wire serial arithmetic is modulo 2^32. Zero is reserved by the device CAN
// scheduler; the worker drops that one generation at wrap and resumes at one.
/**
 * @brief 投影宿主控制代数到模2^32线序号。
 * @param generation 宿主64位递增代号。
 * @return 低32位，可能为0；实际worker须丢弃设备保留的0序号。
 */
inline std::uint32_t wire_producer_sequence(std::uint64_t generation) noexcept {
  return static_cast<std::uint32_t>(generation);
}

/**
 * @brief 单次命令绑定的boot/session/reference/motion及新鲜票据；epoch/ID无角度单位，ticket由设备状态提供。
 */
struct CommandIdentity {
  std::uint64_t boot = 0;
  std::uint64_t session = 0;
  std::uint32_t reference_epoch = 0;
  std::uint32_t motion_epoch = 0;
  std::uint64_t ticket = 0;
};

/**
 * @brief 控制更新快照：generation为宿主代数，position为六轴rad，authority_generation用于阻断旧授权快照复用。
 */
struct CommandGenerationRecord {
  std::uint64_t generation = 0;
  CommandIdentity identity{};
  std::array<double, 6> position{};
  std::uint64_t authority_generation = 0U;
};

/**
 * @brief 只保留最新发布代的两槽快照生产器。
 * 调用方管理控制更新生产关系；reader最多三次尝试，不等待writer。position以double位模式原子存取，避免并发读取非原子double；此类不校验限位、身份资格或实际送达。
 */
class CommandProducer {
 public:
  /**
   * @brief 写入身份和目标并最后发布完整代号。
   * @param identity 本次命令身份值。
   * @param position 六轴rad目标快照。
   * @param authority_generation command_channel核对过的授权世代，默认0不代表有效授权。
   * @return 本次新宿主generation；不表示已入传输队列或设备接纳。
   */
  std::uint64_t record_update(const CommandIdentity &identity,
                              const std::array<double, 6> &position,
                              std::uint64_t authority_generation = 0U) noexcept {
    const auto generation = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
    auto &slot = slots_[generation % slots_.size()];
    slot.ready.store(0, std::memory_order_relaxed);
    slot.boot.store(identity.boot, std::memory_order_relaxed);
    slot.session.store(identity.session, std::memory_order_relaxed);
    slot.reference_epoch.store(identity.reference_epoch, std::memory_order_relaxed);
    slot.motion_epoch.store(identity.motion_epoch, std::memory_order_relaxed);
    slot.ticket.store(identity.ticket, std::memory_order_relaxed);
    slot.authority_generation.store(authority_generation, std::memory_order_relaxed);
    for (std::size_t index = 0; index < position.size(); ++index) {
      slot.position_bits[index].store(to_bits(position[index]), std::memory_order_relaxed);
    }
    slot.ready.store(generation, std::memory_order_release);
    published_generation_.store(generation, std::memory_order_release);
    return generation;
  }

  /**
   * @brief 有界读取比指定代数更新的完整最新快照。
   * 首尾核对slot.ready，避免读取被重用槽的混合代；较旧且尚未消费的中间代可能被最新值替代。
   * @param last_generation 调用方已处理的宿主水位，旧代不重复返回。
   * @return optional记录；无更新或三次遇到写入竞争时为空。
   */
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
      record.authority_generation = slot.authority_generation.load(std::memory_order_relaxed);
      for (std::size_t index = 0; index < record.position.size(); ++index) {
        record.position[index] = from_bits(slot.position_bits[index].load(std::memory_order_relaxed));
      }
      if (slot.ready.load(std::memory_order_acquire) == generation) return record;
    }
    return std::nullopt;
  }

 private:
  /**
   * @brief 按位复制double到64位整数容器。
   * @param value 输入double，不做数值转换。
   * @return 相同位模式的uint64。
   */
  static std::uint64_t to_bits(double value) noexcept {
    std::uint64_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
  }
  /**
   * @brief 恢复先前保存的double位模式。
   * @param bits 64位原始浮点位模式。
   * @return double值，NaN等位型也保持。
   */
  static double from_bits(std::uint64_t bits) noexcept {
    double value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }
  /**
   * @brief 身份字段、double位模式与ready均原子；ready=0表示该槽正在重写，非零值绑定具体generation。
   */
  struct Slot {
    std::atomic<std::uint64_t> ready{0};
    std::atomic<std::uint64_t> boot{0};
    std::atomic<std::uint64_t> session{0};
    std::atomic<std::uint32_t> reference_epoch{0};
    std::atomic<std::uint32_t> motion_epoch{0};
    std::atomic<std::uint64_t> ticket{0};
    std::atomic<std::uint64_t> authority_generation{0};
    std::array<std::atomic<std::uint64_t>, 6> position_bits{};
  };
  /**
   * @brief 两槽轮换并只公布published_generation；next_generation分配新宿主代数，不是队列深度。
   */
  std::array<Slot, 2> slots_{};
  std::atomic<std::uint64_t> next_generation_{0};
  std::atomic<std::uint64_t> published_generation_{0};
};

}  // namespace dummy_ros_control
