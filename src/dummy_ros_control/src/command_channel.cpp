#include "dummy_ros_control/command_channel.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>

namespace dummy_ros_control {
namespace {
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

void set_command_identity(const CommandIdentity &identity) noexcept {
  std::lock_guard<std::mutex> writer(identity_writer_mutex);
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
  const auto epoch = authority_epoch.fetch_add(1U, std::memory_order_acq_rel) + 1U;
  boot.store(identity.boot, std::memory_order_relaxed);
  session.store(identity.session, std::memory_order_relaxed);
  reference_epoch.store(identity.reference_epoch, std::memory_order_relaxed);
  motion_epoch.store(identity.motion_epoch, std::memory_order_relaxed);
  ticket.store(identity.ticket, std::memory_order_relaxed);
  if (identity.boot != 0U && identity.session != 0U && identity.ticket != 0U)
    published_authority_epoch.store(epoch, std::memory_order_release);
}

void clear_command_identity() noexcept {
  std::lock_guard<std::mutex> writer(identity_writer_mutex);
  authority_epoch.fetch_add(1U, std::memory_order_acq_rel);
  const auto record = producer.consume_after(0U);
  if (record) invalidated_generation.store(record->generation, std::memory_order_release);
}

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
  const auto generation = producer.record_update(identity, position, epoch);
  return epoch == authority_epoch.load(std::memory_order_acquire) ? generation : 0U;
}

void controller_update_sink(const double *positions, std::size_t count,
                            std::int64_t update_time_ns, void *context) noexcept {
  (void)publish_control_update(positions, count, update_time_ns, context);
}

std::optional<CommandGenerationRecord> consume_control_update(
    std::uint64_t last_generation) noexcept {
  const auto floor = invalidated_generation.load(std::memory_order_acquire);
  const auto record = producer.consume_after(last_generation > floor ? last_generation : floor);
  return record && command_record_current(*record) ? record : std::nullopt;
}

bool command_record_current(const CommandGenerationRecord &record) noexcept {
  return record.authority_generation != 0U &&
         record.authority_generation == authority_epoch.load(std::memory_order_acquire) &&
         record.authority_generation == published_authority_epoch.load(std::memory_order_acquire);
}
}  // namespace dummy_ros_control
