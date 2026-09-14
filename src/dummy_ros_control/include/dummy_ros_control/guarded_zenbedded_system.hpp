#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>

#include "dummy_ros_control/bounded_publish_queue.hpp"
#include "dummy_ros_control/command_producer.hpp"
#include "dummy_ros_control/state_validator.hpp"
#include "hardware_interface/system_interface.hpp"

namespace dummy_ros_control {

struct JointStateSnapshot {
  StateEnvelope envelope{};
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
};

class GuardedZenbeddedSystem final : public hardware_interface::SystemInterface {
 public:
  CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params) override;
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
  static constexpr bool active_joint_count_supported(std::size_t count) noexcept {
    return count == 1U || count == 6U;
  }

 private:
  static constexpr std::array<const char *, 6> kJointNames{
      "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
  mutable std::mutex state_mutex_;
  JointStateSnapshot state_{};
  bool state_valid_ = false;
  bool active_ = false;
  std::atomic<bool> stop_requested_{false};
  std::size_t active_joint_count_ = 0;
  std::uint64_t consumed_generation_ = 0;
  BoundedPublishQueue<4> publish_queue_;
};

}  // namespace dummy_ros_control
