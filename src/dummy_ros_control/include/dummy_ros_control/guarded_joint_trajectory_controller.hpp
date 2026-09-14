#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>

#include "dummy_ros_control/goal_registration_gate.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace dummy_ros_control {

class GuardedJointTrajectoryController
    : public joint_trajectory_controller::JointTrajectoryController {
 public:
  using UpdateSink = void (*)(const double *positions, std::size_t count,
                              std::int64_t update_time_ns, void *context);
  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  bool register_goal(const GoalCredential &credential);
  bool submit_registered_goal(
      const GoalCredential &credential,
      const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> &trajectory);
  void clear_registration();
  void set_update_sink(UpdateSink sink, void *context) noexcept;

 private:
  std::mutex gate_mutex_;
  GoalRegistrationGate gate_;
  std::atomic<UpdateSink> update_sink_{nullptr};
  std::atomic<void *> update_sink_context_{nullptr};
};

}  // namespace dummy_ros_control
