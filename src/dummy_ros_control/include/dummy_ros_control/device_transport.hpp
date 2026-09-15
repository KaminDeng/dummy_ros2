#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

namespace dummy_ros_control {
class GuardedZenbeddedSystem;
struct DeviceManagementChannel;

struct TransportParameters {
  std::string endpoint;
  std::array<std::uint8_t, 16> device_id{};
  std::array<std::uint8_t, 16> client_instance_id{};
  std::uint32_t profile_id = 0U;
  std::uint8_t configured_mask = 0U;
  std::uint64_t max_state_age_us = 100000U;
};

bool parse_transport_parameters(
    const std::unordered_map<std::string, std::string> &values,
    std::size_t joint_count, TransportParameters *output);

struct TransportStatistics {
  std::uint64_t state_messages = 0U;
  std::uint64_t status_messages = 0U;
  std::uint64_t rejected_states = 0U;
  std::uint64_t commands_published = 0U;
  std::uint64_t rejected_commands = 0U;
  std::uint64_t invalidations = 0U;
  std::uint64_t blocked_stop_requests = 0U;
  bool ready = false;
  bool initialization_failed = false;
};

// Context, node, publishers, subscriptions and executor live exclusively on
// the worker thread. start/stop are non-RT lifecycle operations.
class DeviceTransport final {
 public:
  DeviceTransport(GuardedZenbeddedSystem &system, TransportParameters parameters);
  ~DeviceTransport();
  DeviceTransport(const DeviceTransport &) = delete;
  DeviceTransport &operator=(const DeviceTransport &) = delete;
  bool start();
  void stop();
  TransportStatistics statistics() const noexcept;
  static std::shared_ptr<DeviceManagementChannel> management_channel(const TransportParameters &) noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
std::uint64_t transport_steady_now_us() noexcept;
inline std::uint64_t saturating_state_deadline(std::uint64_t produced_at_us,
                                              std::uint64_t max_age_us) noexcept {
  return produced_at_us > UINT64_MAX - max_age_us ? UINT64_MAX : produced_at_us + max_age_us;
}
}  // namespace dummy_ros_control
