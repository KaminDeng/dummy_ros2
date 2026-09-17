#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <memory>

#include "dummy_ros_control/bounded_publish_queue.hpp"
#include "dummy_ros_control/command_producer.hpp"
#include "dummy_ros_control/state_validator.hpp"
#include "hardware_interface/system_interface.hpp"

namespace dummy_ros_control {
class DeviceTransport;
struct TransportStatistics;

struct JointStateSnapshot {
  StateEnvelope envelope{};
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
};

class GuardedZenbeddedSystem final : public hardware_interface::SystemInterface {
 public:
  GuardedZenbeddedSystem();
  ~GuardedZenbeddedSystem() override;
  CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::return_type read(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  bool ingest_state(const JointStateSnapshot &snapshot,
                    const StateExpectation &expectation) noexcept;
  bool ingest_wire_state(const std::uint8_t *bytes, std::size_t size,
                         const StateExpectation &expectation) noexcept;
  bool copy_state(JointStateSnapshot *snapshot) const noexcept;
  std::uint64_t submit_control_update(
      const CommandIdentity &identity,
      const std::array<double, 6> &position) noexcept;
  bool take_publish_record(CommandGenerationRecord *record) noexcept;
  bool transfer_control_update() noexcept;
  std::uint64_t publish_drops() const noexcept;
  bool take_stop_request() noexcept;
  bool stop_request_pending() const noexcept {
    return stop_requested_.load(std::memory_order_acquire);
  }
  TransportStatistics transport_statistics() const noexcept;
  void invalidate_transport_state(const CommandIdentity *retired_authority = nullptr) noexcept;
  void invalidate_authority_state(const CommandIdentity &retired_authority) noexcept;
  void set_state_deadline(std::uint64_t deadline_us) noexcept;
  bool commands_active() const noexcept { return active_.load(std::memory_order_acquire); }
  static constexpr bool active_joint_count_supported(std::size_t count) noexcept {
    return count == 1U || count == 6U;
  }

 private:
  static constexpr std::array<const char *, 6> kJointNames{
      "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
  mutable std::mutex state_mutex_;
  JointStateSnapshot state_{};
  std::atomic<bool> state_valid_{false};
  // Epoch changes suspend command authorization but retain still-fresh measured positions for read().
  std::atomic<bool> read_state_valid_{false};
  std::atomic<bool> active_{false};
  std::atomic<std::uint64_t> state_deadline_us_{0U};
  std::atomic<bool> stop_requested_{false};
  std::size_t active_joint_count_ = 0;
  std::uint64_t consumed_generation_ = 0;
  BoundedPublishQueue<4> publish_queue_;
  std::unique_ptr<DeviceTransport> transport_;
  std::array<hardware_interface::StateInterface::SharedPtr, 6> position_handles_{};
  std::array<hardware_interface::StateInterface::SharedPtr, 6> velocity_handles_{};
  std::array<hardware_interface::CommandInterface::SharedPtr, 6> command_handles_{};
};

}  // namespace dummy_ros_control
