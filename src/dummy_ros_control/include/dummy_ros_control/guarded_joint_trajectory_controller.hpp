#pragma once

#include <memory>
#include <mutex>

#include "dummy_ros_control/goal_registration_gate.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace dummy_ros_control {

class GuardedJointTrajectoryController
    : public joint_trajectory_controller::JointTrajectoryController {
 public:
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  bool register_goal(const GoalCredential &credential);
  bool submit_registered_goal(
      const GoalCredential &credential,
      const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> &trajectory);
  void clear_registration();

 private:
  std::mutex gate_mutex_;
  GoalRegistrationGate gate_;
};

}  // namespace dummy_ros_control
