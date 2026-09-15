#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>

#include "dummy_ros_control/goal_registration_gate.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace dummy_ros_control {
TrajectoryDigest trajectory_digest(const trajectory_msgs::msg::JointTrajectory &trajectory);
struct GuardedExecutionSample {
  std::array<double,6> actual{}, desired{};
  std::uint64_t sequence=0;
};

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
  bool prime_position(const std::array<double,6> &position);
  void cancel_execution();
  std::optional<GuardedExecutionSample> execution_sample() const noexcept;

 private:
  std::mutex gate_mutex_;
  GoalRegistrationGate gate_;
  std::atomic<UpdateSink> update_sink_{nullptr};
  std::atomic<void *> update_sink_context_{nullptr};
  realtime_tools::RealtimeBuffer<std::shared_ptr<std::array<double,6>>> prime_buffer_;
  std::atomic<std::uint64_t> sample_sequence_{0};
  std::array<std::atomic<double>,6> sample_actual_{}, sample_desired_{};
};

}  // namespace dummy_ros_control
