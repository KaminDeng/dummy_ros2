#include "dummy_ros_control/guarded_zenbedded_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "dp_dummy_robot_v2.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dummy_ros_control {
GuardedZenbeddedSystem::GuardedZenbeddedSystem() = default;
GuardedZenbeddedSystem::~GuardedZenbeddedSystem() = default;

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
  TransportParameters transport_parameters{};
  if (!parse_transport_parameters(params.hardware_info.hardware_parameters,
                                  active_joint_count_, &transport_parameters)) return CallbackReturn::ERROR;
  transport_ = std::make_unique<DeviceTransport>(*this, transport_parameters);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_configure(
    const rclcpp_lifecycle::State &) {
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    position_handles_[index] = get_state_interface_handle(std::string(kJointNames[index]) + "/position");
    velocity_handles_[index] = get_state_interface_handle(std::string(kJointNames[index]) + "/velocity");
    command_handles_[index] = get_command_interface_handle(std::string(kJointNames[index]) + "/position");
  }
  return transport_ && transport_->start() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_cleanup(
    const rclcpp_lifecycle::State &) {
  if (transport_) transport_->stop();
  invalidate_transport_state();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_activate(
    const rclcpp_lifecycle::State &) {
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !state_valid_ ||
      (state_deadline_us_ != 0U && transport_steady_now_us() > state_deadline_us_)) return CallbackReturn::FAILURE;
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    if (!set_command<double>(command_handles_[index], state_.position[index], false)) return CallbackReturn::FAILURE;
  }
  active_ = true;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_deactivate(
    const rclcpp_lifecycle::State &) {
  active_ = false;
  auto none = StopRequestKind::kNone;
  stop_requested_.compare_exchange_strong(none, StopRequestKind::kDeactivation,
                                          std::memory_order_acq_rel);
  clear_command_identity();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GuardedZenbeddedSystem::on_error(
    const rclcpp_lifecycle::State &) {
  active_ = false;
  stop_requested_.store(StopRequestKind::kSafety, std::memory_order_release);
  clear_command_identity();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GuardedZenbeddedSystem::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  // controller_manager polls inactive hardware before activation. Fresh state
  // remains mandatory in on_activate(); an inactive poll must not force ERROR.
  if (!active_) return hardware_interface::return_type::OK;
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!read_state_valid_ ||
      (state_deadline_us_ != 0U && transport_steady_now_us() > state_deadline_us_)) return hardware_interface::return_type::ERROR;
  // A receive in progress is not stale state. Keep the last exported handles;
  // neither the payload timestamp nor its deadline is advanced by this path.
  if(!lock.owns_lock()) return hardware_interface::return_type::OK;
  for (std::size_t index = 0; index < active_joint_count_; ++index) {
    if (!set_state<double>(position_handles_[index], state_.position[index], false) ||
        !set_state<double>(velocity_handles_[index], state_.velocity[index], false))
      return hardware_interface::return_type::ERROR;
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
  if (!publish_queue_.try_push(record.value())) return false;
  consumed_generation_ = record->generation;
  return true;
}

bool GuardedZenbeddedSystem::ingest_state(
    const JointStateSnapshot &snapshot, const StateExpectation &expectation) noexcept {
  if (validate_state(snapshot.envelope, expectation) != StateValidity::kValid) return false;
  for (std::size_t index = 0; index < snapshot.velocity.size(); ++index) {
    if ((snapshot.envelope.configured_mask & (1U << index)) != 0U &&
        !std::isfinite(snapshot.position[index])) return false;
    if ((snapshot.envelope.velocity_valid_mask & (1U << index)) != 0U &&
        !std::isfinite(snapshot.velocity[index])) return false;
  }
  std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  state_ = snapshot;
  state_valid_ = true;
  read_state_valid_ = true;
  return true;
}

bool GuardedZenbeddedSystem::ingest_wire_state(
    const std::uint8_t *bytes, std::size_t size,
    const StateExpectation &expectation) noexcept {
  namespace wire = dp::protocol::dummy_robot_v2;
  wire::DummyJointStateV2 decoded;
  if (wire::DummyJointStateV2Codec::decode(bytes, size, &decoded) != wire::CodecResult::kOk)
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
  if (!lock.owns_lock() || !state_valid_ ||
      (state_deadline_us_ != 0U && transport_steady_now_us() > state_deadline_us_)) return false;
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
  return take_stop_request_kind() != StopRequestKind::kNone;
}

GuardedZenbeddedSystem::StopRequestKind GuardedZenbeddedSystem::take_stop_request_kind() noexcept {
  return stop_requested_.exchange(StopRequestKind::kNone, std::memory_order_acq_rel);
}

void GuardedZenbeddedSystem::request_transport_stop() noexcept {
  active_.store(false, std::memory_order_release);
  stop_requested_.store(StopRequestKind::kSafety, std::memory_order_release);
  clear_command_identity();
}

TransportStatistics GuardedZenbeddedSystem::transport_statistics() const noexcept {
  return transport_ ? transport_->statistics() : TransportStatistics{};
}

void GuardedZenbeddedSystem::invalidate_transport_state(
    const CommandIdentity *retired_authority) noexcept {
  state_valid_.store(false, std::memory_order_release);
  read_state_valid_.store(false, std::memory_order_release);
  active_.store(false, std::memory_order_release);
  if (retired_authority != nullptr) clear_command_identity_if(*retired_authority);
  else clear_command_identity();
}

void GuardedZenbeddedSystem::invalidate_authority_state(
    const CommandIdentity &retired_authority) noexcept {
  state_valid_.store(false, std::memory_order_release);
  clear_command_identity_if(retired_authority);
}

void GuardedZenbeddedSystem::set_state_deadline(std::uint64_t deadline_us) noexcept {
  state_deadline_us_.store(deadline_us, std::memory_order_release);
}

}  // namespace dummy_ros_control

PLUGINLIB_EXPORT_CLASS(
    dummy_ros_control::GuardedZenbeddedSystem,
    hardware_interface::SystemInterface)
