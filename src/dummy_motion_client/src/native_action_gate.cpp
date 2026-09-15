#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dummy_motion_client/native_goal_validation.hpp"
#include "dummy_motion_client/native_wait_policy.hpp"

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
          if (!valid_native_goal(*goal)) return rclcpp_action::GoalResponse::REJECT;
          return wait_policy_.try_acquire()
                     ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                     : rclcpp_action::GoalResponse::REJECT;
        },
        [](std::shared_ptr<ServerHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<ServerHandle> handle) {
          std::lock_guard<std::mutex> lock(worker_mutex_);
          if (worker_.joinable()) worker_.join();
          worker_ = std::thread([this, handle = std::move(handle)] {
            try { execute(handle); }
            catch (const std::exception &error) {
              wait_policy_.latch_fault();
              auto failure=std::make_shared<Follow::Result>();
              failure->error_code=Follow::Result::INVALID_GOAL;
              failure->error_string=std::string("INTERNAL_RESULT_UNCONFIRMED: ")+error.what();
              finish(handle,failure,rclcpp_action::ResultCode::ABORTED);
            }
          });
        });
  }

  ~NativeActionGate() override {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) worker_.join();
  }

 private:
  void finish(const std::shared_ptr<ServerHandle> &outer, const std::shared_ptr<Follow::Result> &result,
              rclcpp_action::ResultCode code) {
    try {
      if (code == rclcpp_action::ResultCode::SUCCEEDED) outer->succeed(result);
      else if (code == rclcpp_action::ResultCode::CANCELED) outer->canceled(result);
      else outer->abort(result);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Action terminal delivery failed: %s", error.what());
    }
    wait_policy_.finish();
  }

  static std::uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static std::uint64_t duration_us(const builtin_interfaces::msg::Duration &duration) {
    return static_cast<std::uint64_t>(duration.sec)*1000000U+duration.nanosec/1000U;
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
      if (outer->is_active()) outer->publish_feedback(std::make_shared<Follow::Feedback>(*feedback));
    };
    options.goal_response_callback = [this,outer](ClientHandle::SharedPtr internal) {
      if (internal && !outer->is_active()) client_->async_cancel_goal(internal);
    };
    auto sent = client_->async_send_goal(*outer->get_goal(), options);
    if (sent.wait_for(10s) != std::future_status::ready) {
      wait_policy_.latch_fault();
      failure->error_string="INTERNAL_GOAL_ACCEPTANCE_UNCONFIRMED";
      finish(outer, failure, rclcpp_action::ResultCode::ABORTED);
      return;
    }
    const auto internal = sent.get();
    if (!internal) {
      finish(outer, failure, rclcpp_action::ResultCode::ABORTED);
      return;
    }
    auto result = client_->async_get_result(internal);
    using CancelResponse=rclcpp_action::Client<Follow>::CancelResponse;
    std::optional<std::shared_future<CancelResponse::SharedPtr>> cancel_response;
    bool result_timed_out=false;
    const auto goal=outer->get_goal();
    const auto budget=saturating_deadline(duration_us(goal->trajectory.points.back().time_from_start),
        saturating_deadline(duration_us(goal->goal_time_tolerance),10000000U));
    wait_policy_.begin(now_us(),budget);
    while (rclcpp::ok()) {
      if (!cancel_response && (outer->is_canceling() || wait_policy_.expired(now_us()))) {
        result_timed_out=!outer->is_canceling();
        cancel_response=client_->async_cancel_goal(internal);
        wait_policy_.cancel(now_us(),2000000U);
      }
      const bool result_ready=result.wait_for(20ms)==std::future_status::ready;
      const bool terminal_confirmed=result_ready &&
          (result.get().code==rclcpp_action::ResultCode::SUCCEEDED ||
           result.get().code==rclcpp_action::ResultCode::CANCELED ||
           result.get().code==rclcpp_action::ResultCode::ABORTED);
      if (terminal_confirmed) wait_policy_.terminal_confirmed();
      if (!cancel_response && terminal_confirmed) {
        const auto wrapped=result.get(); finish(outer,wrapped.result,wrapped.code); return;
      }
      if (!cancel_response && result_ready) {
        result_timed_out=true;
        cancel_response=client_->async_cancel_goal(internal);
        wait_policy_.cancel(now_us(),2000000U);
      }
      if (!cancel_response) continue;
      if (cancel_response->wait_for(0ms)==std::future_status::ready && cancel_response->get())
        wait_policy_.cancel_acknowledged();
      if (wait_policy_.cancel_complete()) {
        const auto wrapped=result.get();
        if (result_timed_out) {
          failure->error_string="INTERNAL_RESULT_TIMEOUT";
          finish(outer,failure,rclcpp_action::ResultCode::ABORTED);
        } else finish(outer,wrapped.result,wrapped.code);
        return;
      }
      if (wait_policy_.expired(now_us())) {
        wait_policy_.latch_fault();
        failure->error_string="INTERNAL_CANCEL_RESULT_UNCONFIRMED: restart gate before another goal";
        finish(outer,failure,rclcpp_action::ResultCode::ABORTED);
        return;
      }
    }
    wait_policy_.latch_fault();
  }

  NativeWaitPolicy wait_policy_;
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
