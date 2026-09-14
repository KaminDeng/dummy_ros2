#include "dummy_ros_control/command_channel.hpp"

#include <array>
#include <atomic>

namespace dummy_ros_control {
namespace {
CommandProducer producer;
std::atomic<bool> identity_valid{false};
std::atomic<std::uint64_t> boot{0};
std::atomic<std::uint64_t> session{0};
std::atomic<std::uint32_t> reference_epoch{0};
std::atomic<std::uint32_t> motion_epoch{0};
std::atomic<std::uint64_t> ticket{0};
}  // namespace

void set_command_identity(const CommandIdentity &identity) noexcept {
  identity_valid.store(false, std::memory_order_release);
  boot.store(identity.boot, std::memory_order_relaxed);
  session.store(identity.session, std::memory_order_relaxed);
  reference_epoch.store(identity.reference_epoch, std::memory_order_relaxed);
  motion_epoch.store(identity.motion_epoch, std::memory_order_relaxed);
  ticket.store(identity.ticket, std::memory_order_relaxed);
  identity_valid.store(
      identity.boot != 0U && identity.session != 0U && identity.ticket != 0U,
      std::memory_order_release);
}

void clear_command_identity() noexcept {
  identity_valid.store(false, std::memory_order_release);
}

std::uint64_t publish_control_update(const double *positions, std::size_t count,
                                     std::int64_t, void *) noexcept {
  if (!positions || count != 6U || !identity_valid.load(std::memory_order_acquire)) return 0U;
  const CommandIdentity identity{
      boot.load(std::memory_order_relaxed), session.load(std::memory_order_relaxed),
      reference_epoch.load(std::memory_order_relaxed), motion_epoch.load(std::memory_order_relaxed),
      ticket.load(std::memory_order_relaxed)};
  if (!identity_valid.load(std::memory_order_acquire)) return 0U;
  std::array<double, 6> position{};
  for (std::size_t index = 0; index < position.size(); ++index) position[index] = positions[index];
  return producer.record_update(identity, position);
}

void controller_update_sink(const double *positions, std::size_t count,
                            std::int64_t update_time_ns, void *context) noexcept {
  (void)publish_control_update(positions, count, update_time_ns, context);
}

std::optional<CommandGenerationRecord> consume_control_update(
    std::uint64_t last_generation) noexcept {
  return producer.consume_after(last_generation);
}
}  // namespace dummy_ros_control
