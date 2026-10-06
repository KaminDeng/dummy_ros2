#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"

#include "pluginlib/class_list_macros.hpp"
#include <openssl/sha.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace dummy_ros_control {
namespace {
void append_u32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  for (int shift = 0; shift < 32; shift += 8) bytes.push_back(static_cast<std::uint8_t>(value >> shift));
}

void append_stamp(std::vector<std::uint8_t> &bytes, std::int32_t sec, std::uint32_t nanosec) {
  if (nanosec >= 1000000000U) throw std::invalid_argument("trajectory nanosec outside range");
  append_u32(bytes, static_cast<std::uint32_t>(sec));
  append_u32(bytes, nanosec);
}

void append_length(std::vector<std::uint8_t> &bytes, std::size_t length) {
  if (length > std::numeric_limits<std::uint32_t>::max())
    throw std::invalid_argument("trajectory sequence outside uint32 range");
  append_u32(bytes, static_cast<std::uint32_t>(length));
}

void append_string(std::vector<std::uint8_t> &bytes, const std::string &value) {
  append_length(bytes, value.size());
  bytes.insert(bytes.end(), value.begin(), value.end());
}

void append_doubles(std::vector<std::uint8_t> &bytes, const std::vector<double> &values) {
  append_length(bytes, values.size());
  for (double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("trajectory values must be finite");
    const double normalized = value == 0.0 ? 0.0 : value;
    std::uint64_t bits = 0;
    std::memcpy(&bits, &normalized, sizeof(bits));
    for (int shift = 0; shift < 64; shift += 8)
      bytes.push_back(static_cast<std::uint8_t>(bits >> shift));
  }
}
}  // namespace

TrajectoryDigest trajectory_digest(const trajectory_msgs::msg::JointTrajectory &trajectory) {
  std::vector<std::uint8_t> bytes{'D','P','S','D','K','-','J','T','-','1',0};
  append_stamp(bytes, trajectory.header.stamp.sec, trajectory.header.stamp.nanosec);
  append_string(bytes, trajectory.header.frame_id);
  append_length(bytes, trajectory.joint_names.size());
  for (const auto &name : trajectory.joint_names) append_string(bytes, name);
  append_length(bytes, trajectory.points.size());
  for (const auto &point : trajectory.points) {
    append_doubles(bytes, point.positions);
    append_doubles(bytes, point.velocities);
    append_doubles(bytes, point.accelerations);
    append_doubles(bytes, point.effort);
    append_stamp(bytes, point.time_from_start.sec, point.time_from_start.nanosec);
  }
  TrajectoryDigest digest{};
  SHA256(bytes.data(), bytes.size(), digest.data());
  return digest;
}

controller_interface::return_type GuardedJointTrajectoryController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  const auto prime=*prime_buffer_.readFromRT();
  auto result=controller_interface::return_type::OK;
  if(prime) {
    if(dof_!=1 && dof_!=6) return controller_interface::return_type::ERROR;
    for(std::size_t index=0;index<dof_;++index) {
      state_desired_.positions[index]=(*prime)[index];
      state_current_.positions[index]=joint_state_interface_[0][index].get().get_optional().value_or(NAN);
      if(!joint_command_interface_[0][index].get().set_value((*prime)[index]))
        return controller_interface::return_type::ERROR;
    }
  } else result = JointTrajectoryController::update(time, period);
  sample_sequence_.fetch_add(1,std::memory_order_acq_rel);
  for(std::size_t index=0;index<dof_ && index<6;++index) {
    sample_actual_[index].store(state_current_.positions[index],std::memory_order_relaxed);
    sample_desired_[index].store(state_desired_.positions[index],std::memory_order_relaxed);
  }
  sample_sequence_.fetch_add(1,std::memory_order_release);
  const auto sink = update_sink_.load(std::memory_order_acquire);
  if (result == controller_interface::return_type::OK && sink != nullptr &&
      state_desired_.positions.size() == dof_) {
    sink(state_desired_.positions.data(), state_desired_.positions.size(),
         time.nanoseconds(), update_sink_context_.load(std::memory_order_relaxed));
  }
  return result;
}

controller_interface::CallbackReturn GuardedJointTrajectoryController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto result = JointTrajectoryController::on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) return result;
  joint_command_subscriber_.reset();
  action_server_.reset();
  subscriber_is_active_.store(false);
  // The action gate arms this sink only after it primes the hold target.
  set_update_sink(nullptr, nullptr);
  return result;
}

controller_interface::CallbackReturn GuardedJointTrajectoryController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  const bool owned_command_sink=update_sink_.load(std::memory_order_acquire)!=nullptr;
  set_update_sink(nullptr, nullptr);
  if(owned_command_sink) clear_command_identity();
  return JointTrajectoryController::on_deactivate(previous_state);
}

bool GuardedJointTrajectoryController::register_goal(const GoalCredential &credential) {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  const auto registered = gate_.register_goal(credential);
  if (!registered.has_value()) return false;
  gate_ = registered.value();
  return true;
}

bool GuardedJointTrajectoryController::submit_registered_goal(
    const GoalCredential &credential,
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> &trajectory) {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  if (!gate_.permits(credential) || !trajectory) return false;
  try {
    if (trajectory_digest(*trajectory) != credential.digest) return false;
  } catch (const std::invalid_argument &) {
    return false;
  }
  if (!validate_trajectory_msg(*trajectory)) return false;
  auto ordered = std::make_shared<trajectory_msgs::msg::JointTrajectory>(*trajectory);
  if (ordered->joint_names.size() != command_joint_names_.size()) return false;
  sort_to_local_joint_order(ordered);
  add_new_trajectory_msg(ordered);
  prime_buffer_.writeFromNonRT(nullptr);
  gate_ = gate_.clear();
  return true;
}

bool GuardedJointTrajectoryController::prime_position(const std::array<double,6> &position) {
  if(dof_!=1 && dof_!=6) return false;
  for(std::size_t i=0;i<dof_;++i) if(!std::isfinite(position[i])) return false;
  prime_buffer_.writeFromNonRT(std::make_shared<std::array<double,6>>(position));
  return true;
}
void GuardedJointTrajectoryController::cancel_execution() {
  clear_registration();
  if(const auto sample=execution_sample()) prime_position(sample->actual);
}
std::optional<GuardedExecutionSample> GuardedJointTrajectoryController::execution_sample() const noexcept {
  const auto sequence=sample_sequence_.load(std::memory_order_acquire);
  if(sequence==0 || sequence%2) return {};
  GuardedExecutionSample sample; sample.sequence=sequence;
  for(std::size_t i=0;i<6;++i) {
    sample.actual[i]=sample_actual_[i].load(std::memory_order_relaxed);
    sample.desired[i]=sample_desired_[i].load(std::memory_order_relaxed);
  }
  if(sample_sequence_.load(std::memory_order_acquire)!=sequence) return {};
  return sample;
}

void GuardedJointTrajectoryController::clear_registration() {
  std::lock_guard<std::mutex> lock(gate_mutex_);
  gate_ = gate_.clear();
}

void GuardedJointTrajectoryController::set_update_sink(
    UpdateSink sink, void *context) noexcept {
  update_sink_.store(nullptr, std::memory_order_release);
  update_sink_context_.store(context, std::memory_order_relaxed);
  update_sink_.store(sink, std::memory_order_release);
}

}  // namespace dummy_ros_control

PLUGINLIB_EXPORT_CLASS(
    dummy_ros_control::GuardedJointTrajectoryController,
    controller_interface::ControllerInterface)
