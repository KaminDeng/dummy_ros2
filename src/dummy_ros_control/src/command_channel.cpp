/**
 * @file command_channel.cpp
 * @brief 实现进程内控制授权世代和最新命令快照。
 * 身份写入/撤销由identity_writer_mutex串行；实时publish仅读原子并写CommandProducer两槽。ROS更新时间参数不参与设备票据时效，设备新鲜票据与本地authority_generation分别防旧状态和旧授权复用。
 */
#include "dummy_ros_control/command_channel.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>

namespace dummy_ros_control {
namespace {
/**
 * @brief 本进程内唯一producer；authority_epoch撤销/切换时递增，published_authority_epoch标记已完整发布身份，invalidated_generation给消费下限。
 */
CommandProducer producer;
std::mutex identity_writer_mutex;
std::atomic<std::uint64_t> authority_epoch{1U};
std::atomic<std::uint64_t> published_authority_epoch{0U};
std::atomic<std::uint64_t> boot{0};
std::atomic<std::uint64_t> session{0};
std::atomic<std::uint32_t> reference_epoch{0};
std::atomic<std::uint32_t> motion_epoch{0};
std::atomic<std::uint64_t> ticket{0};
std::atomic<std::uint64_t> invalidated_generation{0};
}  // namespace

bool advance_control_sequence_floor(std::uint32_t floor) noexcept {
  return producer.advance_generation_floor(floor);
}

/**
 * @brief 发布或刷新当前控制授权身份。
 * 同一授权只更新ticket；身份变化递增authority世代。此接口含writer mutex，不能泛称所有命令通道操作均无锁。
 * @param identity boot/session/reference/motion及非零设备票据。
 */
void set_command_identity(const CommandIdentity &identity) noexcept {
  std::lock_guard<std::mutex> writer(identity_writer_mutex);
  /**
   * @brief 同boot/session/reference/motion且当前世代仍有效时只刷新ticket，避免每个State票据更新都使控制generation失效。
   */
  const bool same_authority =
      boot.load(std::memory_order_relaxed) == identity.boot &&
      session.load(std::memory_order_relaxed) == identity.session &&
      reference_epoch.load(std::memory_order_relaxed) == identity.reference_epoch &&
      motion_epoch.load(std::memory_order_relaxed) == identity.motion_epoch &&
      published_authority_epoch.load(std::memory_order_acquire) ==
          authority_epoch.load(std::memory_order_acquire);
  if (same_authority && identity.boot != 0U && identity.session != 0U && identity.ticket != 0U) {
    ticket.store(identity.ticket, std::memory_order_release);
    return;
  }
  /**
   * @brief 先抬世代使RT读者拒绝混合身份，再逐字段写；最后仅对boot/session/ticket全非零的身份发布新世代。
   */
  const auto epoch = authority_epoch.fetch_add(1U, std::memory_order_acq_rel) + 1U;
  boot.store(identity.boot, std::memory_order_relaxed);
  session.store(identity.session, std::memory_order_relaxed);
  reference_epoch.store(identity.reference_epoch, std::memory_order_relaxed);
  motion_epoch.store(identity.motion_epoch, std::memory_order_relaxed);
  ticket.store(identity.ticket, std::memory_order_relaxed);
  if (identity.boot != 0U && identity.session != 0U && identity.ticket != 0U)
    published_authority_epoch.store(epoch, std::memory_order_release);
}

/**
 * @brief 无条件撤销当前本地授权世代。
 * 保留底层身份数值但使其世代不再匹配；当前已发布记录的代号成为失效水位，不发送Stop。
 */
void clear_command_identity() noexcept {
  std::lock_guard<std::mutex> writer(identity_writer_mutex);
  authority_epoch.fetch_add(1U, std::memory_order_acq_rel);
  const auto record = producer.consume_after(0U);
  if (record) invalidated_generation.store(record->generation, std::memory_order_release);
}

/**
 * @brief 仅在被退役身份仍为当前身份时撤销。
 * 防止迟到旧会话清理撤销新授权。
 * @param expected 预期退役的boot/session/reference/motion；ticket不参与比较。
 * @return true已撤销，false当前已是另一身份。
 */
bool clear_command_identity_if(const CommandIdentity &expected) noexcept {
  std::lock_guard<std::mutex> writer(identity_writer_mutex);
  if (boot.load(std::memory_order_relaxed) != expected.boot ||
      session.load(std::memory_order_relaxed) != expected.session ||
      reference_epoch.load(std::memory_order_relaxed) != expected.reference_epoch ||
      motion_epoch.load(std::memory_order_relaxed) != expected.motion_epoch) {
    return false;
  }
  authority_epoch.fetch_add(1U, std::memory_order_acq_rel);
  const auto record = producer.consume_after(0U);
  if (record) invalidated_generation.store(record->generation, std::memory_order_release);
  return true;
}

/**
 * @brief 保存1轴或6轴有限目标的最近控制快照。
 * 前后核对authority世代，实时路径不等待网络；返回非零不意味着worker已发送。
 * @param positions 调用期间有效的位置数组，rad。
 * @param count 只能1或6；未用轴填NaN。
 * @param update_time_ns 控制器ROS更新时间ns；当前实现不使用它判断新鲜度。
 * @param context 回调兼容上下文；当前实现未使用。
 * @return 宿主generation；非法输入或授权更新竞争返回0。
 */
std::uint64_t publish_control_update(const double *positions, std::size_t count,
                                     std::int64_t, void *) noexcept {
  const auto epoch = published_authority_epoch.load(std::memory_order_acquire);
  if (!positions || (count != 1U && count != 6U) ||
      epoch != authority_epoch.load(std::memory_order_acquire)) return 0U;
  const CommandIdentity identity{
      boot.load(std::memory_order_relaxed), session.load(std::memory_order_relaxed),
      reference_epoch.load(std::memory_order_relaxed), motion_epoch.load(std::memory_order_relaxed),
      ticket.load(std::memory_order_relaxed)};
  if (epoch != authority_epoch.load(std::memory_order_acquire)) return 0U;
  std::array<double, 6> position{};
  position.fill(std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < count; ++index) {
    if (!std::isfinite(positions[index])) return 0U;
    position[index] = positions[index];
  }
  /**
   * @brief 写快照后再次检查授权世代；并发撤销时即使槽存在也返回0，消费者还须command_record_current拒绝该世代。
   */
  const auto generation = producer.record_update(identity, position, epoch);
  return epoch == authority_epoch.load(std::memory_order_acquire) ? generation : 0U;
}

/**
 * @brief 适配JTC的void控制更新回调。
 * 忽略publish_control_update的返回值；失效记录由后续消费者水位处理。
 * @param positions 有限rad位置数组。
 * @param count 1或6轴。
 * @param update_time_ns ROS控制更新时间ns，原样转交。
 * @param context 用户回调上下文，原样转交。
 */
void controller_update_sink(const double *positions, std::size_t count,
                            std::int64_t update_time_ns, void *context) noexcept {
  (void)publish_control_update(positions, count, update_time_ns, context);
}

/**
 * @brief 读取当前授权下高于消费/失效水位的最新更新。
 * 读取最新快照不消耗网络带宽；硬件层须再尝试有界队列。
 * @param last_generation 上次成功转交的宿主generation。
 * @return optional记录，旧授权或没有可见更新时为空。
 */
std::optional<CommandGenerationRecord> consume_control_update(
    std::uint64_t last_generation) noexcept {
  const auto floor = invalidated_generation.load(std::memory_order_acquire);
  const auto record = producer.consume_after(last_generation > floor ? last_generation : floor);
  return record && command_record_current(*record) ? record : std::nullopt;
}

/**
 * @brief 检查记录绑定的授权世代仍被发布且有效。
 * 不替代worker的ticket时效、设备身份、限位及commit检查。
 * @param record 待转交或待发送控制记录。
 * @return bool；要求非零authority_generation并同时等于当前/已发布世代。
 */
bool command_record_current(const CommandGenerationRecord &record) noexcept {
  return record.authority_generation != 0U &&
         record.authority_generation == authority_epoch.load(std::memory_order_acquire) &&
         record.authority_generation == published_authority_epoch.load(std::memory_order_acquire);
}
}  // namespace dummy_ros_control
