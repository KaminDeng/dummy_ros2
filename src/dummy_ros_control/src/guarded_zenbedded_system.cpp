#include "dummy_ros_control/guarded_zenbedded_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "dp_dummy_robot_v1.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dummy_ros_control {

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS ||
      !active_joint_count_supported(params.hardware_info.joints.size())) return CallbackReturn::ERROR;
  active_joint_count_ = params.hardware_info.joints.size();
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    const auto &joint = params.hardware_info.joints[index];
    if (joint.name != kJointNames[index] || joint.command_interfaces.size() != 1U ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_activate(
    const rclcpp_lifecycle::State &) {
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !state_valid_) return CallbackReturn::FAILURE;
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    set_command<double>(std::string(kJointNames[index]) + "/position", state_.position[index]);
  }
  active_ = true;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_deactivate(
    const rclcpp_lifecycle::State &) {
  active_ = false;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_error(
    const rclcpp_lifecycle::State &) {
  active_ = false;
  stop_requested_.store(true, std::memory_order_release);
  clear_command_identity();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GuardedZenbeddedSystem::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !state_valid_) return hardware_interface::return_type::ERROR;
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    set_state<double>(std::string(kJointNames[index]) + "/position", state_.position[index]);
    set_state<double>(std::string(kJointNames[index]) + "/velocity", state_.velocity[index]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GuardedZenbeddedSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &) {
  if (!active_) return hardware_interface::return_type::OK;
  transfer_control_update();
  return hardware_interface::return_type::OK;
}

bool GuardedZenbeddedSystem::transfer_control_update() noexcept {
  const auto record = consume_control_update(consumed_generation_);
  if (!record.has_value()) return false;
  consumed_generation_ = record->generation;
  return publish_queue_.try_push(record.value());
}

bool GuardedZenbeddedSystem::ingest_state(
    const JointStateSnapshot &snapshot, const StateExpectation &expectation) noexcept {
  if (validate_state(snapshot.envelope, expectation) != StateValidity::kValid ||
      !std::all_of(snapshot.position.begin(), snapshot.position.end(), [](double value) { return std::isfinite(value); }))
    return false;
  for (std::size_t index = 0; index < snapshot.velocity.size(); ++index) {
    if ((snapshot.envelope.velocity_valid_mask & (1U << index)) != 0U &&
        !std::isfinite(snapshot.velocity[index])) return false;
  }
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  state_ = snapshot;
  state_valid_ = true;
  return true;
}

bool GuardedZenbeddedSystem::ingest_wire_state(
    const std::uint8_t *bytes, std::size_t size,
    const StateExpectation &expectation) noexcept {
  namespace wire = dp::protocol::dummy_robot_v1;
  wire::DummyJointStateV1 decoded;
  if (wire::DummyJointStateV1Codec::decode(bytes, size, &decoded) != wire::CodecResult::kOk)
    return false;
  JointStateSnapshot snapshot{};
  snapshot.envelope.boot = decoded.boot_epoch();
  snapshot.envelope.profile = decoded.profile_id();
  snapshot.envelope.state_seq = decoded.state_seq();
  snapshot.envelope.produced_at_us = decoded.produced_at_us();
  snapshot.envelope.configured_mask = decoded.configured_mask();
  snapshot.envelope.position_valid_mask = decoded.position_valid_mask();
  snapshot.envelope.velocity_valid_mask = decoded.velocity_valid_mask();
  for (std::size_t index = 0; index < snapshot.position.size(); ++index) {
    snapshot.position[index] = decoded.position_rad()[index];
    snapshot.velocity[index] =
        (decoded.velocity_valid_mask() & (1U << index)) != 0U
            ? decoded.estimated_velocity_rad_s()[index]
            : std::numeric_limits<double>::quiet_NaN();
  }
  return ingest_state(snapshot, expectation);
}

bool GuardedZenbeddedSystem::copy_state(JointStateSnapshot *snapshot) const noexcept {
  if (!snapshot) return false;
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !state_valid_) return false;
  *snapshot = state_;
  return true;
}

std::uint64_t GuardedZenbeddedSystem::submit_control_update(
    const CommandIdentity &identity, const std::array<double, 6> &position) noexcept {
  set_command_identity(identity);
  return publish_control_update(position.data(), position.size(), 0, nullptr);
}

bool GuardedZenbeddedSystem::take_publish_record(CommandGenerationRecord *record) noexcept {
  return publish_queue_.try_pop(record);
}

std::uint64_t GuardedZenbeddedSystem::publish_drops() const noexcept {
  return publish_queue_.drops();
}

bool GuardedZenbeddedSystem::take_stop_request() noexcept {
  return stop_requested_.exchange(false, std::memory_order_acq_rel);
}

}  // namespace dummy_ros_control

PLUGINLIB_EXPORT_CLASS(
    dummy_ros_control::GuardedZenbeddedSystem,
    hardware_interface::SystemInterface)
