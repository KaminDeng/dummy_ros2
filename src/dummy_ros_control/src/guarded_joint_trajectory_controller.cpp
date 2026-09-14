#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace dummy_ros_control {

controller_interface::return_type GuardedJointTrajectoryController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  const auto result = JointTrajectoryController::update(time, period);
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
  set_update_sink(controller_update_sink, nullptr);
  return result;
}

controller_interface::CallbackReturn GuardedJointTrajectoryController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  set_update_sink(nullptr, nullptr);
  clear_command_identity();
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
  if (!gate_.permits(credential) || !trajectory || !validate_trajectory_msg(*trajectory)) return false;
  auto ordered = std::make_shared<trajectory_msgs::msg::JointTrajectory>(*trajectory);
  if (ordered->joint_names.size() != command_joint_names_.size()) return false;
  sort_to_local_joint_order(ordered);
  add_new_trajectory_msg(ordered);
  gate_ = gate_.clear();
  return true;
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
