#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace dummy_motion_client {
using Follow = control_msgs::action::FollowJointTrajectory;
using ServerHandle = rclcpp_action::ServerGoalHandle<Follow>;
using ClientHandle = rclcpp_action::ClientGoalHandle<Follow>;
using namespace std::chrono_literals;

class NativeActionGate final : public rclcpp::Node {
 public:
  NativeActionGate() : Node("native_action_gate") {
    client_ = rclcpp_action::create_client<Follow>(this, "/_internal_arm_controller/follow_joint_trajectory");
    server_ = rclcpp_action::create_server<Follow>(
        this, "/arm_controller/follow_joint_trajectory",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> goal) {
          if (!valid(*goal)) return rclcpp_action::GoalResponse::REJECT;
          bool expected = false;
          return busy_.compare_exchange_strong(expected, true)
                     ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                     : rclcpp_action::GoalResponse::REJECT;
        },
        [](std::shared_ptr<ServerHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<ServerHandle> handle) {
          std::lock_guard<std::mutex> lock(worker_mutex_);
          if (worker_.joinable()) worker_.join();
          worker_ = std::thread([this, handle = std::move(handle)] { execute(handle); });
        });
  }

  ~NativeActionGate() override {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) worker_.join();
  }

 private:
  static bool valid(const Follow::Goal &goal) {
    static const std::unordered_set<std::string> expected{
        "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
    static const std::unordered_map<std::string, std::pair<double, double>> limits{
        {"Joint1", {-2.9670597283903604, 2.9670597283903604}},
        {"Joint2", {-1.2740903539558606, 1.5707963267948966}},
        {"Joint3", {0.6108652381980153, 3.141592653589793}},
        {"Joint4", {-3.141592653589793, 3.141592653589793}},
        {"Joint5", {-2.0943951023931953, 2.0943951023931953}},
        {"Joint6", {-12.566370614359172, 12.566370614359172}},
    };
    const auto &trajectory = goal.trajectory;
    if (trajectory.joint_names.size() != expected.size() ||
        std::unordered_set<std::string>(trajectory.joint_names.begin(), trajectory.joint_names.end()) != expected ||
        trajectory.points.empty()) return false;
    std::int64_t previous_ns = -1;
    for (const auto &point : trajectory.points) {
      if (point.positions.size() != expected.size()) return false;
      if (!point.velocities.empty() && point.velocities.size() != expected.size()) return false;
      if (!point.accelerations.empty() && point.accelerations.size() != expected.size()) return false;
      for (std::size_t index = 0; index < point.positions.size(); ++index) {
        const auto value = point.positions[index];
        const auto bounds = limits.at(trajectory.joint_names[index]);
        if (!std::isfinite(value) || value < bounds.first || value > bounds.second) return false;
      }
      for (const auto value : point.velocities) if (!std::isfinite(value)) return false;
      for (const auto value : point.accelerations) if (!std::isfinite(value)) return false;
      const auto current_ns = rclcpp::Duration(point.time_from_start).nanoseconds();
      if (current_ns < 0 || current_ns <= previous_ns) return false;
      previous_ns = current_ns;
    }
    const auto &terminal_velocity = trajectory.points.back().velocities;
    return terminal_velocity.empty() ||
           std::all_of(terminal_velocity.begin(), terminal_velocity.end(), [](double value) { return value == 0.0; });
  }

  void finish(const std::shared_ptr<ServerHandle> &outer, const std::shared_ptr<Follow::Result> &result,
              rclcpp_action::ResultCode code) {
    if (code == rclcpp_action::ResultCode::SUCCEEDED) outer->succeed(result);
    else if (code == rclcpp_action::ResultCode::CANCELED) outer->canceled(result);
    else outer->abort(result);
    busy_.store(false);
  }

  void execute(const std::shared_ptr<ServerHandle> &outer) {
    auto failure = std::make_shared<Follow::Result>();
    failure->error_code = Follow::Result::INVALID_GOAL;
    if (!client_->wait_for_action_server(10s)) {
      finish(outer, failure, rclcpp_action::ResultCode::ABORTED);
      return;
    }
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.feedback_callback = [outer](ClientHandle::SharedPtr,
                                        const std::shared_ptr<const Follow::Feedback> feedback) {
      outer->publish_feedback(std::make_shared<Follow::Feedback>(*feedback));
    };
    auto sent = client_->async_send_goal(*outer->get_goal(), options);
    if (sent.wait_for(10s) != std::future_status::ready) {
      finish(outer, failure, rclcpp_action::ResultCode::ABORTED);
      return;
    }
    const auto internal = sent.get();
    if (!internal) {
      finish(outer, failure, rclcpp_action::ResultCode::ABORTED);
      return;
    }
    auto result = client_->async_get_result(internal);
    bool cancel_sent = false;
    while (rclcpp::ok() && result.wait_for(20ms) != std::future_status::ready) {
      if (outer->is_canceling() && !cancel_sent) {
        client_->async_cancel_goal(internal);
        cancel_sent = true;
      }
    }
    if (!rclcpp::ok()) {
      busy_.store(false);
      return;
    }
    const auto wrapped = result.get();
    finish(outer, wrapped.result, wrapped.code);
  }

  std::atomic<bool> busy_{false};
  rclcpp_action::Client<Follow>::SharedPtr client_;
  rclcpp_action::Server<Follow>::SharedPtr server_;
  std::mutex worker_mutex_;
  std::thread worker_;
};
}  // namespace dummy_motion_client

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<dummy_motion_client::NativeActionGate>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
