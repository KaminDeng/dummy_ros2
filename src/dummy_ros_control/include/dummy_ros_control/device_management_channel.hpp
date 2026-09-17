#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include "dummy_ros_control/device_transport.hpp"

namespace dummy_ros_control {
// One pending request and one latest observation: capacity is independent of traffic rate.
struct DeviceManagementRequest {
  std::array<std::uint8_t,225> request{};
  std::array<std::uint8_t,105> response{};
  std::atomic<bool> canceled{false}, failed{false}, complete{false};
};
struct DeviceManagementObservation {
  std::array<std::uint8_t,301> state{};
  std::array<std::uint8_t,257> status{};
  std::uint64_t state_received_us=0, status_received_us=0;
};
struct DeviceManagementChannel {
  explicit DeviceManagementChannel(TransportParameters value):parameters(std::move(value)) {}
  const TransportParameters parameters;
  std::mutex mutex;
  std::atomic<bool> ready{false};
  std::atomic<bool> inactive_prime_allowed{false};
  std::shared_ptr<DeviceManagementRequest> queued;
  DeviceManagementObservation observation;
};
}  // namespace dummy_ros_control
