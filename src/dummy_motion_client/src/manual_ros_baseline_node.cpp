// 自动基线只编排公开 ROS 接口；设备管理与许可始终属于 gate 的唯一 worker。
#include "dummy_motion_client/manual_ros_baseline.hpp"
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include <algorithm>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <csignal>
#include <dpsdk_interfaces/msg/dummy_device_status_v2.hpp>
#include <dpsdk_interfaces/msg/dummy_joint_state_v2.hpp>
#include <dummy_ref/manual_reference_policy.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <set>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

namespace dummy_motion_client {
using Trigger = std_srvs::srv::Trigger;
using Fjt = control_msgs::action::FollowJointTrajectory;
using Status = dpsdk_interfaces::msg::DummyDeviceStatusV2;
using State = dpsdk_interfaces::msg::DummyJointStateV2;
using Plan = moveit_msgs::srv::GetMotionPlan;
using Scene = moveit_msgs::srv::GetPlanningScene;
using Apply = moveit_msgs::srv::ApplyPlanningScene;
using Validity = moveit_msgs::srv::GetStateValidity;
using Hardware = controller_manager_msgs::srv::SetHardwareComponentState;
using Switch = controller_manager_msgs::srv::SwitchController;
using Controllers = controller_manager_msgs::srv::ListControllers;
using GoalHandle = rclcpp_action::ClientGoalHandle<Fjt>;
// 标准Action客户端只暴露本次原随机UUID；协议和请求仍由rclcpp_action处理。
class ReceiptActionClient final : public rclcpp_action::Client<Fjt> {
public:
  using rclcpp_action::Client<Fjt>::Client;
  std::string sent_uuid;
protected:
  rclcpp_action::GoalUUID generate_goal_id() override {
    const auto id = rclcpp_action::ClientBase::generate_goal_id();
    std::ostringstream out;
    for (const auto byte : id)
      out << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
    sent_uuid = out.str();
    return id;
  }
};
static std::uint64_t steady_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
class ManualRosBaselineNode final : public rclcpp::Node {
public:
  ManualRosBaselineNode() : Node("manual_ros_baseline") {
    BaselineOptions options;
    const auto total_seconds = declare_parameter("total_timeout_s", 840);
    if (total_seconds < 1 || total_seconds > 840)
      configuration_error_ +=
          "total_timeout_s必须为1..840，为900秒permit保留清理余量; ";
    options.total_timeout_us =
        std::uint64_t(std::clamp<std::int64_t>(total_seconds, 1, 840)) *
        1000000;
    flow_ = ManualRosBaseline(options);
    task_timeout_us_ = options.total_timeout_us;
    automatic_ = declare_parameter("auto_calibrate_and_test", false);
    initial_pose_ =
        declare_parameter("initial_pose", std::string("manual_limits"));
    names_ = declare_parameter("joint_names", std::vector<std::string>{});
    lower_ = read_array("lower_rad");
    upper_ = read_array("upper_rad");
    rest_ = read_array("rest_rad");
    velocity_ = read_array("velocity_limits_rad_s");
    acceleration_ = read_array("acceleration_limits_rad_s2");
    age_us_ = std::uint64_t(std::max<std::int64_t>(
                  1, declare_parameter("feedback_max_age_ms", 250))) *
              1000;
    // Status与Native消费同一公共SDK预算；State/逐轴/CAN仍独立250ms。
    constexpr auto status_limit_ms = static_cast<std::int64_t>(
        dummy_ref::host::kManualReferenceStatusMaxAgeUs / 1000);
    const auto status_age_ms =
        declare_parameter("status_max_age_ms", status_limit_ms);
    if (status_age_ms < 1 || status_age_ms > status_limit_ms)
      configuration_error_ +=
          "status_max_age_ms必须为1.." + std::to_string(status_limit_ms) + "; ";
    status_age_us_ = std::uint64_t(std::clamp<std::int64_t>(status_age_ms, 1,
                                                            status_limit_ms)) *
                     1000;
    report_path_ = declare_parameter("report_path", std::string{});
    timeline_path_ = declare_parameter("goal_timeline_path", std::string{});
    raw_path_ = declare_parameter("raw_evidence_path", std::string{});
    group_ = declare_parameter("planning_group", std::string("dummy_arm"));
    frame_ = declare_parameter("planning_frame", std::string("base_link"));
    cylinder_frame_ = declare_parameter("cylinder_frame", frame_);
    cylinder_pose_ = declare_parameter("cylinder_pose", std::vector<double>{});
    cylinder_dimensions_ =
        declare_parameter("cylinder_dimensions", std::vector<double>{});
    // 与Jazzy create_client相同的waitable注册/移除生命周期。
    std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_waitables = get_node_waitables_interface();
    action_ = std::shared_ptr<ReceiptActionClient>(new ReceiptActionClient(
        get_node_base_interface(), get_node_graph_interface(),
        get_node_logging_interface(), "/arm_controller/follow_joint_trajectory",
        rcl_action_client_get_default_options()),
        [weak_waitables](ReceiptActionClient *client) {
          if (auto waitables = weak_waitables.lock()) {
            std::shared_ptr<ReceiptActionClient> temporary(client, [](auto *) {});
            waitables->remove_waitable(temporary, nullptr);
          }
          delete client;
        });
    get_node_waitables_interface()->add_waitable(action_, nullptr);
    plan_ = create_client<Plan>("/plan_kinematic_path");
    validity_ = create_client<Validity>("/check_state_validity");
    scene_ = create_client<Scene>("/get_planning_scene");
    apply_ = create_client<Apply>("/apply_planning_scene");
    hardware_ = create_client<Hardware>(
        "/controller_manager/set_hardware_component_state");
    controllers_ =
        create_client<Switch>("/controller_manager/switch_controller");
    controller_list_ = create_client<Controllers>("/controller_manager/list_controllers");
    for (const auto &name :
         {"calibrate_manual_reference", "manual_reference_status",
          "enable_hold", "bootstrap_status", "disable_and_release", "last_goal_rejection", "set_bench_permit"})
      services_[name] =
          create_client<Trigger>(std::string("/device_action_gate/") + name);
    start_service_ =
        create_service<Trigger>("/device_action_gate/auto_calibrate_and_test",
                                [this](const Trigger::Request::SharedPtr,
                                       Trigger::Response::SharedPtr response) {
                                  response->success = start();
                                  response->message = response->success
                                                          ? "BASELINE_ACCEPTED"
                                                          : description();
                                });
    manual_start_service_ = create_service<Trigger>(
        "/device_action_gate/prepare_manual_control",
        [this](const Trigger::Request::SharedPtr,
               Trigger::Response::SharedPtr response) {
          response->success = start(true);
          response->message =
              response->success ? "MANUAL_CONTROL_ACCEPTED" : description();
        });
    manual_status_service_ = create_service<Trigger>(
        "/device_action_gate/manual_control_status",
        [this](const Trigger::Request::SharedPtr,
               Trigger::Response::SharedPtr response) {
          response->success =
              manual_mode_ &&
              ((manual_ready_ && flow_.stage() == BaselineStage::kSingleAxis &&
                (moving_ready() || waiting_for_next_permit()) && state_.session_id == manual_session_ &&
                state_.reference_epoch == expected_reference_) ||
               (manual_external_release_ && released_ && !released_fault_ && !flow_.active()));
          response->message = description();
        });
    status_service_ = create_service<Trigger>(
        "/device_action_gate/baseline_status",
        [this](const Trigger::Request::SharedPtr,
               Trigger::Response::SharedPtr response) {
          response->success =
              !manual_mode_ && flow_.stage() == BaselineStage::kPassed;
          response->message = description();
        });
    status_sub_ =
        create_subscription<Status>("/device_action_gate/device_status", 10,
                                    [this](Status::SharedPtr value) {
                                      status_ = *value;
                                      status_at_ = steady_us();
                                    });
    state_sub_ = create_subscription<State>(
        "/device_action_gate/device_state", 10, [this](State::SharedPtr value) {
          if (state_at_ && value->boot_epoch == state_.boot_epoch &&
              std::int32_t(value->state_seq - state_.state_seq) <= 0)
            return;
          const auto received = steady_us();
          for (unsigned i = 0; i < 6; ++i)
            if (!state_at_ || value->boot_epoch != state_.boot_epoch ||
                value->feedback_seq[i] != state_.feedback_seq[i])
              axis_at_[i] = received;
          if (state_at_ && value->boot_epoch == state_.boot_epoch) {
            bool still = true;
            for (unsigned i = 0; i < 6; ++i) {
              if (value->feedback_seq[i] == state_.feedback_seq[i])
                still = false;
              if (std::abs(value->raw_motor_position_rev[i] -
                           state_.raw_motor_position_rev[i]) > 0.0005)
                still = false;
            }
            stationary_samples_ = still ? stationary_samples_ + 1 : 0;
          } else
            stationary_samples_ = 0;
          state_ = *value;
          state_at_ = steady_us();
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
      try {
        tick();
      } catch (const std::exception &e) {
        fail(std::string("自动基线异常: ") + e.what());
      }
    });
  }
  bool finished() const { return !flow_.active(); }
  void shutdown_cleanup() {
    if (flow_.active())
      fail("节点关闭");
  }

private:
  std::array<double, 6> read_array(const std::string &name) {
    auto values = declare_parameter(name, std::vector<double>{});
    std::array<double, 6> out{};
    if (values.size() != 6) {
      configuration_error_ += "缺少六轴参数 " + name + "; ";
      return out;
    }
    std::copy(values.begin(), values.end(), out.begin());
    return out;
  }
  bool fresh() const {
    const auto now = steady_us();
    for (unsigned i = 0; i < 6; ++i)
      if (!axis_at_[i] || now - axis_at_[i] > age_us_ ||
          state_.can_rx_age_us[i] > age_us_)
        return false;
    return state_at_ && status_at_ && now - state_at_ <= age_us_ &&
           now - status_at_ <= status_age_us_ &&
           state_.device_id == status_.device_id && state_.boot_epoch &&
           state_.boot_epoch == status_.boot_epoch &&
           (!boot_ ||
            (boot_ == state_.boot_epoch && device_ == state_.device_id));
  }
  bool moving_ready() const {
    return controllers_active_ && fresh() && state_.position_valid_mask == 63 &&
           state_.velocity_valid_mask == 63 && state_.session_id &&
           state_.profile_id == status_.profile_id &&
           state_.session_id == status_.session_id && state_.reference_epoch &&
           state_.reference_epoch == status_.reference_epoch &&
           status_.real_motion_permitted &&
           status_.permit_remaining_us > age_us_ && !status_.evidence_lost;
  }
  bool waiting_for_next_permit() const {
    return controllers_active_ && fresh() && state_.core_state == 4 &&
            status_.core_state == 4 && state_.hold_ready &&
            state_.position_valid_mask == 63 && state_.velocity_valid_mask == 63 &&
            state_.session_id == manual_session_ && status_.session_id == manual_session_ &&
            state_.profile_id == status_.profile_id &&
            state_.reference_epoch == expected_reference_ &&
            status_.reference_epoch == expected_reference_ &&
            !status_.real_motion_permitted && status_.permit_remaining_us == 0 &&
            !status_.evidence_lost;
  }
  std::string description() const {
    if (manual_mode_) {
      if (manual_external_release_ && released_ && !released_fault_ && !flow_.active())
        return "MANUAL_CONTROL_RELEASED boot=" + std::to_string(boot_);
      if (flow_.stage() == BaselineStage::kFailed)
        return "MANUAL_CONTROL_FAILED: " + flow_.failure() +
               " cleanup=" + (released_ ? (released_fault_ ? "RELEASED_FAULT" : "RELEASED_IDLE") : "UNCONFIRMED");
      if (manual_ready_ && flow_.stage() == BaselineStage::kSingleAxis)
        return "MANUAL_CONTROL_READY boot=" + std::to_string(boot_) +
               " session=" + std::to_string(manual_session_);
      return std::string("MANUAL_CONTROL_") +
             baseline_stage_name(flow_.stage()) + " " + detail_;
    }
    if (flow_.stage() == BaselineStage::kPassed)
      return "BASELINE_PASSED boot=" + std::to_string(boot_) +
             " report=" + report_path_;
    if (flow_.stage() == BaselineStage::kFailed)
      return "BASELINE_FAILED: " + flow_.failure() + " cleanup=" +
             (!management_requested_
                  ? "NOT_ACQUIRED"
                  : (released_ ? (released_fault_ ? "RELEASED_FAULT" : "RELEASED_IDLE") : "UNCONFIRMED"));
    return std::string("BASELINE_") + baseline_stage_name(flow_.stage()) + " " +
           detail_;
  }
  bool preexisting_owner() const {
    return state_.session_id != 0 || status_.session_id != 0 ||
           status_.real_motion_permitted || status_.permit_remaining_us != 0;
  }
  bool start(bool manual = false) {
    if (fresh() && preexisting_owner()) {
      detail_ = "已有控制owner/permit，自动任务拒绝接管";
      return false;
    }
    const auto started_at = steady_us();
    released_fault_=false;
    if (!flow_.start(started_at))
      return false;
    controllers_active_ = false;
    next_permit_requested_ = false;
    pending_controller_activation_.clear();
    task_started_us_ = started_at;
    manual_mode_ = manual;
    step_started_ = steady_us();
    entry_ = BaselineStage::kReadOnly;
    if (initial_pose_ != "manual_limits")
      fail("缺少当前双限位显式前提");
    if (names_.size() != 6)
      configuration_error_ += "joint_names必须六轴; ";
    if (report_path_.empty() || timeline_path_.empty() || raw_path_.empty())
      configuration_error_ += "缺少报告/timeline/raw路径; ";
    if (!configuration_error_.empty())
      fail(configuration_error_);
    if (flow_.stage() != BaselineStage::kRelease) {
      if (std::filesystem::exists(report_path_))
        fail("报告路径已存在，拒绝覆盖");
      else {
        report_.open(report_path_);
        if (!report_)
          fail("报告路径不可写");
      }
    }
    record("START", "反馈证据只代表设备编码器，不代替独立外部位移测量");
    return true;
  }
  void record(const std::string &event, const std::string &text) {
    detail_ = text;
    RCLCPP_INFO(get_logger(), "%s: %s", event.c_str(), text.c_str());
    if (report_) {
      report_ << steady_us() << ' ' << event << " boot=" << state_.boot_epoch
              << " session=" << state_.session_id << " seq=" << state_.state_seq
              << " operation=" << status_.operation_id
              << " run=" << status_.run_id << ' ' << text << '\n';
      report_.flush();
      if (!report_)
        fail("报告写入失败");
    }
  }
  void fail(const std::string &reason) {
    flow_.fail(reason, steady_us());
    detail_ = reason;
  }
  template <class Service, class Callback>
  bool request(typename rclcpp::Client<Service>::SharedPtr client,
               typename Service::Request::SharedPtr req, Callback callback) {
    if (busy_ || !client->service_is_ready())
      return false;
    busy_ = true;
    goal_deadline_us_ = 0;
    const auto ticket = ++ticket_;
    pending_since_ = steady_us();
    client->async_send_request(
        req, [this, ticket,
              callback](typename rclcpp::Client<Service>::SharedFuture future) {
          if (ticket != ticket_)
            return;
          busy_ = false;
          try {
            callback(*future.get());
          } catch (const std::exception &e) {
            fail(e.what());
          }
        });
    return true;
  }
  bool trigger(const std::string &name,
               std::function<void(const Trigger::Response &)> callback) {
    return request<Trigger>(services_.at(name),
                            std::make_shared<Trigger::Request>(),
                            std::move(callback));
  }
  void advance() { flow_.complete(true, "CONFIRMED", steady_us()); }
  void tick() {
    if (automatic_ && !automatic_attempted_) {
      automatic_attempted_ = true;
      start();
    }
    if (!flow_.active())
      return;
    const auto now = steady_us();
    // 手动Ready后由本owner的反馈、许可和动作期限监督；准备/自动基线才消费一次性总期限。
    if (!(manual_mode_ && manual_ready_ &&
          flow_.stage() == BaselineStage::kSingleAxis))
      flow_.tick(now);
    if (flow_.stage() != BaselineStage::kRelease && boot_ && !fresh()) {
      // 只等待本节点DDS缓存追上；等待期间不发新动作。USB/逐轴CAN的250ms保护仍由唯一设备worker执行。
      bool local_delivery_wait = manual_ready_ && state_at_ && status_at_ &&
          state_.boot_epoch == boot_ && status_.boot_epoch == boot_ &&
          state_.device_id == device_ && status_.device_id == device_ &&
          state_.session_id == manual_session_ && status_.session_id == manual_session_ &&
          state_.reference_epoch == expected_reference_ && status_.reference_epoch == expected_reference_ &&
          state_.profile_id == status_.profile_id && !status_.evidence_lost &&
          state_.position_valid_mask == 63 && state_.velocity_valid_mask == 63 &&
          state_.core_state != 9 && status_.core_state != 9 &&
          now - state_at_ <= age_us_ + 2000000 &&
          now - status_at_ <= status_age_us_ + 2000000;
      for (unsigned i = 0; i < 6; ++i)
        local_delivery_wait = local_delivery_wait && axis_at_[i] &&
            now - axis_at_[i] <= age_us_ + 2000000 && state_.can_rx_age_us[i] <= age_us_;
      if (local_delivery_wait)
        return;
      // 首次失鲜保留独立接收锚/轴推进/CAN年龄，不能把DDS停发误称设备源停发。
      std::string detail="state_age_us="+std::to_string(state_at_?now-state_at_:UINT64_MAX)+
        " status_age_us="+std::to_string(status_at_?now-status_at_:UINT64_MAX)+
        " state_seq="+std::to_string(state_.state_seq)+" boot="+std::to_string(state_.boot_epoch)+
        " status_boot="+std::to_string(status_.boot_epoch)+" expected_boot="+std::to_string(boot_)+
        " device_match="+std::to_string(state_.device_id==status_.device_id && device_==state_.device_id);
      for(unsigned i=0;i<6;++i)detail+=" axis"+std::to_string(i)+"_progress_age_us="+
        std::to_string(axis_at_[i]?now-axis_at_[i]:UINT64_MAX)+" can_age_us="+std::to_string(state_.can_rx_age_us[i]);
      record("FRESHNESS_FAILED",detail);
      fail("设备身份改变或源反馈停止进展");
    }
    if (busy_ &&
        (goal_ && goal_deadline_us_ ? now >= goal_deadline_us_
                                    : now - pending_since_ > 15000000)) {
      busy_ = false;
      ++ticket_;
      fail("ROS异步调用期限超时");
    }
    if (flow_.stage() != entry_) {
      entry_ = flow_.stage();
      substep_ = 0;
      step_started_ = now;
      if (entry_ == BaselineStage::kRelease) {
        // A separately requested D10 has already produced fresh dual released
        // Idle. Consume the gate's original-page/terminal confirmation below;
        // do not send another D10 or infer success from the dual states alone.
        if (manual_external_release_)
          substep_ = 1;
        busy_ = false;
        ++ticket_;
        release_seq_ = state_.state_seq;
        release_retry_at_ = 0;
        if (!reference_bound_ &&
            state_.reference_epoch == status_.reference_epoch)
          expected_reference_ = state_.reference_epoch;
        if (goal_) {
          action_->async_cancel_goal(goal_);
        }
      }
      record("STAGE", baseline_stage_name(entry_));
    }
    if (!manual_ready_ && entry_ != BaselineStage::kRelease &&
        now - step_started_ > 180000000) {
      fail("单项期限超时");
      return;
    }
    if (goal_ && entry_ != BaselineStage::kRelease) {
      if (fresh() && std::int32_t(state_.state_seq - goal_seq_) > 0) {
        for (unsigned i = 0; i < 6; ++i)
          if (std::abs(state_.position_rad[i] - before_[i]) > 0.0005)
            feedback_moved_ = true;
        if (state_.core_state == 5 && status_.core_state == 5 && status_.run_id)
          goal_running_ = true;
      }
      if (cancel_test_ && goal_running_ && now - goal_sent_at_ > 600000 &&
          !cancel_requested_) {
        cancel_requested_ = true;
        const auto cancel_ticket = ticket_;
        action_->async_cancel_goal(goal_, [this, cancel_ticket](auto response) {
          if (cancel_ticket != ticket_)
            return;
          cancel_ack_ = response && response->return_code == 0 &&
                        !response->goals_canceling.empty();
          if (!cancel_ack_)
            fail("目标取消未受理");
        });
      }
    }
    if (renewal_phase_ && entry_ != BaselineStage::kRelease) {
      if (now >= renewal_until_ || now >= renewal_goal_deadline_) {
        fail("续租拒绝回执等待期限超时");
        return;
      }
      if (!busy_)
        retry_after_renewal();
      return;
    }
    if (next_motion_waiting_ && entry_ != BaselineStage::kRelease) {
      if (now >= next_motion_until_) {
        fail("下一运动许可/ready等待期限超时");
        return;
      }
      if (!busy_)
        await_next_motion_ready();
      return;
    }
    if (busy_)
      return;
    switch (entry_) {
    case BaselineStage::kCalibrating:
      calibrate();
      break;
    case BaselineStage::kHold:
      hold();
      break;
    case BaselineStage::kSingleAxis:
      if (manual_mode_) {
        if (!manual_ready_) {
          manual_ready_ = true;
          manual_session_ = state_.session_id;
          record("MANUAL_READY", description());
        }
        const bool released_idle =
            fresh() && !state_.session_id && !status_.session_id &&
            !status_.real_motion_permitted &&
            status_.permit_remaining_us == 0 && status_.release_committed &&
            state_.core_state == 2 && status_.core_state == 2 &&
            state_.reference_epoch == expected_reference_ &&
            status_.reference_epoch == expected_reference_;
        if (released_idle) {
          manual_external_release_ = true;
          fail("手动控制已释放");
        } else if ((!moving_ready() && !waiting_for_next_permit()) || state_.session_id != manual_session_ ||
                   state_.reference_epoch != expected_reference_) {
          if (state_.core_state == 9 || status_.core_state == 9 ||
              state_.reference_epoch != expected_reference_ ||
              status_.reference_epoch != expected_reference_ ||
              (state_.session_id && state_.session_id != manual_session_) ||
              (status_.session_id && status_.session_id != manual_session_))
            fail("手动控制owner/参考失效");
          else if (!manual_owner_lost_at_)
            manual_owner_lost_at_ = now;
          else if (now - manual_owner_lost_at_ > 2000000)
            fail("手动控制许可或owner失效");
        } else
          manual_owner_lost_at_ = 0;
      } else
        single_axis();
      break;
    case BaselineStage::kMultiAxis:
      multi_axis();
      break;
    case BaselineStage::kPlanRestRoundTrip:
      rest_trip();
      break;
    case BaselineStage::kObstacleCheck:
      obstacle();
      break;
    case BaselineStage::kCancelCheck:
      cancel_check();
      break;
    case BaselineStage::kRelease:
      release();
      break;
    default:
      break;
    }
  }
  void calibrate() {
    // 初始静止只用于提交资格运动的准入；17退让期间允许真实位置变化。
    if (!management_requested_ &&
        (!fresh() || state_.raw_position_valid_mask != 63 || stationary_samples_ < 3)) {
      if (steady_us() - step_started_ > 5000000)
        fail("没有独立新鲜六轴静止反馈");
      return;
    }
    if (!management_requested_ && preexisting_owner()) {
      fail("已有控制owner/permit，自动任务拒绝接管");
      return;
    }
    if (!boot_) {
      boot_ = state_.boot_epoch;
      device_ = state_.device_id;
      expected_reference_ = state_.reference_epoch;
      reference_bound_ = expected_reference_ != 0;
    }
    if (substep_ == 0) {
      const bool submitted =
          trigger("calibrate_manual_reference", [this](const auto &response) {
            if (!response.success) {
              fail("标定未受理: " + response.message);
              return;
            }
            ++substep_;
            record("CALIBRATION_ACCEPTED", response.message);
          });
      if (submitted)
        management_requested_ = true;
    } else
      trigger("manual_reference_status", [this](const auto &response) {
        if (response.success &&
            response.message.find("READY") != std::string::npos) {
          if (!fresh() || state_.position_valid_mask != 63 ||
              !state_.reference_epoch)
            return;
          if (reference_bound_ &&
              state_.reference_epoch != expected_reference_) {
            fail("已资格参考在复用流程中改变");
            return;
          }
          expected_reference_ = state_.reference_epoch;
          reference_bound_ = true;
          record("CALIBRATION_CONFIRMED", response.message);
          advance();
        } else if (response.message.find("FAULT") != std::string::npos ||
                   response.message.find("FAILED") != std::string::npos)
          fail("标定失败: " + response.message);
      });
  }
  void hold() {
    if (rest_aligning_) {
      if (result_waiting_) {
        if (confirm_goal()) {
          rest_aligning_ = false;
          origin_ = state_.position_rad;
          record("REST_ALIGNMENT_CONFIRMED", "公开MoveIt/FJT归位后开始1度基线");
          advance();
        }
      } else if (plan_ready_) {
        plan_ready_ = false;
        send(planned_trajectory_, planned_target_, true);
      } else {
        request_plan(rest_);
      }
      return;
    }
    if (substep_ == 0) {
      auto req = std::make_shared<Hardware::Request>();
      req->name = "dummy_ref_real_6dof";
      req->target_state.id = 3;
      request<Hardware>(hardware_, req, [this](const auto &response) {
        if (!response.ok || response.state.id != 3)
          fail("硬件激活失败");
        else
          ++substep_;
      });
    } else if (substep_ == 1) {
      request<Controllers>(controller_list_, std::make_shared<Controllers::Request>(),
                           [this](const auto &response) {
        std::vector<std::string> inactive;
        for (const auto &name : {"joint_state_broadcaster", "arm_controller"}) {
          unsigned matches = 0;
          std::string state;
          for (const auto &controller : response.controller) {
            if (controller.name == name) { ++matches; state = controller.state; }
          }
          // 等spawner配置完成；原flow/step绝对截止不因轮询延期。
          if (matches != 1 || (state != "inactive" && state != "active")) return;
          if (state == "inactive") inactive.emplace_back(name);
        }
        pending_controller_activation_ = std::move(inactive);
        ++substep_;
      });
    } else if (substep_ == 2) {
      if (pending_controller_activation_.empty()) { ++substep_; return; }
      auto req = std::make_shared<Switch::Request>();
      // 已active者不重复激活；所有尚inactive者在同一次STRICT事务中激活。
      req->activate_controllers = pending_controller_activation_;
      req->strictness = Switch::Request::STRICT;
      req->activate_asap = false;
      req->timeout.sec = 10;
      request<Switch>(controllers_, req, [this](const auto &response) {
        if (!response.ok) fail("控制器严格激活失败: " + response.message);
        else ++substep_;
      });
    } else if (substep_ == 3) {
      request<Controllers>(controller_list_, std::make_shared<Controllers::Request>(),
                           [this](const auto &response) {
        for (const auto &name : {"joint_state_broadcaster", "arm_controller"}) {
          unsigned matches = 0;
          for (const auto &controller : response.controller)
            if (controller.name == name && controller.state == "active") ++matches;
          if (matches != 1) return;
        }
        controllers_active_ = true;
        ++substep_;
      });
    } else if (substep_ == 4)
      trigger("enable_hold", [this](const auto &response) {
        if (!response.success)
          fail("保持未受理: " + response.message);
        else
          ++substep_;
      });
    else
      trigger("bootstrap_status", [this](const auto &response) {
        if (response.success &&
            response.message.find("HOLD") != std::string::npos &&
            moving_ready() && state_.hold_ready && state_.core_state == 4 &&
            status_.core_state == 4) {
          for (unsigned i = 0; i < 6; ++i) {
            lower_[i] = std::max(lower_[i], status_.position_min_rad[i]);
            upper_[i] = std::min(upper_[i], status_.position_max_rad[i]);
            velocity_[i] =
                std::min(velocity_[i], status_.velocity_max_rad_s[i]);
            acceleration_[i] =
                std::min(acceleration_[i], status_.acceleration_max_rad_s[i]);
            if (velocity_[i] <= 0 || acceleration_[i] <= 0 ||
                lower_[i] >= upper_[i]) {
              fail("设备与部署限制无有效交集");
              return;
            }
          }
          origin_ = state_.position_rad;
          record("HOLD_CONFIRMED", response.message);
          // SDK/REF只退让危险侧，保留参考时的安全J2/J3可能不同于模型REST。
          // 自动基线先经公开规划/执行归位；手动准备不增加任何自动运动。
          bool needs_rest_alignment = false;
          for (unsigned i = 0; i < 6; ++i)
            needs_rest_alignment =
                needs_rest_alignment || std::abs(origin_[i] - rest_[i]) > 0.001;
          if (!manual_mode_ && needs_rest_alignment) {
            if (!valid_baseline_rest_alignment(
                    origin_, rest_, lower_, upper_,
                    dummy_ref::host::kManualReferenceWorkingMarginRad)) {
              fail("资格位置与模型REST差异超出有限归位范围");
              return;
            }
            rest_aligning_ = true;
            record("REST_ALIGNMENT_REQUESTED",
                   "仅J2/J3在SDK工作余量内沿受限走廊归位");
            return;
          }
          advance();
        } else if (response.message.find("FAULT") != std::string::npos)
          fail("保持失败: " + response.message);
      });
  }
  trajectory_msgs::msg::JointTrajectory
  make_trajectory(const std::array<double, 6> &target,
                  double minimum_seconds = 0) {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(target.begin(), target.end());
    double seconds = minimum_seconds;
    for (unsigned i = 0; i < 6; ++i) {
      const double v = std::min(velocity_[i], 0.03490658503988659),
                   a = std::min(acceleration_[i], 0.1);
      if (v <= 0 || a <= 0)
        throw std::runtime_error("无有效部署动态上限");
      const double delta = std::abs(target[i] - state_.position_rad[i]);
      seconds =
          std::max(seconds, std::max(2 * delta / v, 3 * std::sqrt(delta / a)));
    }
    seconds = std::max(seconds, 2.0);
    point.time_from_start = rclcpp::Duration::from_seconds(seconds);
    point.velocities.assign(6, 0);
    point.accelerations.assign(6, 0);
    trajectory.points.push_back(point);
    return trajectory;
  }
  bool
  validate_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory,
                      const std::array<double, 6> &target, bool planned) {
    if (trajectory.joint_names != names_ || trajectory.points.empty() ||
        !moving_ready())
      return false;
    auto previous = state_.position_rad;
    std::array<double, 6> previous_velocity{}, previous_acceleration{};
    double prior_time = 0;
    unsigned index = 0;
    for (const auto &point : trajectory.points) {
      if (point.positions.size() != 6 ||
          (!point.velocities.empty() && point.velocities.size() != 6) ||
          (!point.accelerations.empty() && point.accelerations.size() != 6))
        return false;
      if (rest_aligning_ &&
          (point.velocities.size() != 6 || point.accelerations.size() != 6 ||
           (index == 0 &&
            rclcpp::Duration(point.time_from_start).nanoseconds() != 0)))
        return false;
      const double t = rclcpp::Duration(point.time_from_start).seconds();
      if (!std::isfinite(t) || t <= prior_time) {
        // MoveIt 首点可为 t=0，仅允许与当前实测位置吻合的静止首点。
        if (!(planned && index == 0 && t == 0))
          return false;
      }
      for (unsigned i = 0; i < 6; ++i) {
        const double position = point.positions[i];
        if (!std::isfinite(position) || position < lower_[i] ||
            position > upper_[i] ||
            (rest_aligning_
                 ? (position < std::min(origin_[i], rest_[i]) - 0.0001 ||
                    position > std::max(origin_[i], rest_[i]) + 0.0001)
                 : std::abs(position - origin_[i]) >
                       0.017453292519943295 + 0.0001))
          return false;
        const double v = std::min(velocity_[i], 0.03490658503988659),
                     a = std::min(acceleration_[i], 0.1);
        if (t == 0 && std::abs(position - previous[i]) > 0.0005)
          return false;
        if (t > prior_time &&
            std::abs(position - previous[i]) / (t - prior_time) > v * 1.001)
          return false;
        if (!point.velocities.empty() &&
            (!std::isfinite(point.velocities[i]) ||
             std::abs(point.velocities[i]) > v * 1.001))
          return false;
        if (!point.accelerations.empty() &&
            (!std::isfinite(point.accelerations[i]) ||
             std::abs(point.accelerations[i]) > a * 1.001))
          return false;
        if (rest_aligning_ && t > prior_time) {
          const auto bounds = baseline_quintic_bounds(
              previous[i], previous_velocity[i], previous_acceleration[i],
              position, point.velocities[i], point.accelerations[i],
              t - prior_time);
          if (!bounds.valid ||
              bounds.lower < std::min(origin_[i], rest_[i]) - 0.0001 ||
              bounds.upper > std::max(origin_[i], rest_[i]) + 0.0001 ||
              bounds.max_velocity > v * 1.001 ||
              bounds.max_acceleration > a * 1.001)
            return false;
        }
        previous[i] = position;
        if (rest_aligning_) {
          previous_velocity[i] = point.velocities[i];
          previous_acceleration[i] = point.accelerations[i];
        }
      }
      prior_time = t;
      ++index;
    }
    for (unsigned i = 0; i < 6; ++i)
      if (std::abs(previous[i] - target[i]) > 0.0005)
        return false;
    return true;
  }
  std::string
  trajectory_digest(const trajectory_msgs::msg::JointTrajectory &trajectory) {
    const auto hash = dummy_ros_control::trajectory_digest(trajectory);
    std::ostringstream out;
    for (auto b : hash)
      out << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
    return out.str();
  }
  void retry_after_renewal() {
    if (renewal_phase_ == 1) {
      trigger("last_goal_rejection", [this](const auto &response) {
        if (!response.success || response.message != "RENEWAL_PENDING:" + rejected_uuid_) {
          fail("公开FJT拒绝目标，无匹配续租UUID回执");
          return;
        }
        renewal_phase_ = 2;
        record("RENEWAL_REJECTION_BOUND", "uuid=" + rejected_uuid_);
      });
      return;
    }
    const auto same_identity = [this]() {
      return moving_ready() && state_.device_id == renewal_state_.device_id &&
          status_.device_id == renewal_status_.device_id &&
          state_.profile_id == renewal_state_.profile_id && status_.profile_id == renewal_status_.profile_id &&
          state_.boot_epoch == renewal_state_.boot_epoch && status_.boot_epoch == renewal_status_.boot_epoch &&
          state_.session_id == renewal_state_.session_id && status_.session_id == renewal_status_.session_id &&
          state_.reference_epoch == renewal_state_.reference_epoch && status_.reference_epoch == renewal_status_.reference_epoch &&
          state_.motion_epoch == renewal_state_.motion_epoch && status_.motion_epoch == renewal_status_.motion_epoch &&
          std::uint32_t(state_.management_epoch - renewal_state_.management_epoch) <= 1 &&
          std::uint32_t(status_.management_epoch - renewal_status_.management_epoch) <= 1;
    };
    if (!same_identity()) {
      fail("续租等待期间身份/许可/epoch漂移");
      return;
    }
    if (renewal_phase_ == 3) {
      renewal_phase_ = 0;
      send(renewal_trajectory_, renewal_target_, renewal_planned_, renewal_cancel_, true);
      return;
    }
    trigger("bootstrap_status", [this](const auto &response) {
      if (!response.success ||
          (response.message.find("HOLD") == std::string::npos && response.message.find("LEASE_RENEWED") == std::string::npos))
        return;
      if (!moving_ready() || !state_.hold_ready || state_.core_state != 4 || status_.core_state != 4 ||
          state_.management_epoch != status_.management_epoch ||
          std::int32_t(state_.state_seq - renewal_state_.state_seq) <= 0 ||
          status_at_ <= renewal_status_at_)
        return;
      // 下次tick重新核验完整身份，再按原目标及首次发送截止重发一次。
      renewal_phase_ = 3;
    });
  }
  void send(const trajectory_msgs::msg::JointTrajectory &trajectory,
            const std::array<double, 6> &target, bool planned = false,
            bool cancel = false, bool renewal_retry = false) {
    if (!validate_trajectory(trajectory, target, planned)) {
      fail("整轨迹/起点/动态/一度范围核对失败");
      return;
    }
    if (!action_->action_server_is_ready())
      return;
    Fjt::Goal request;
    request.trajectory = trajectory;
    request.goal_time_tolerance.sec = 3;
    const auto trajectory_ns =
        rclcpp::Duration(trajectory.points.back().time_from_start)
            .nanoseconds();
    std::uint64_t execution_budget = 0;
    const auto budget_now = steady_us();
    const std::uint64_t step_remaining =
        180000000ULL -
        std::min<std::uint64_t>(budget_now - step_started_, 180000000ULL);
    const auto task_remaining =
        task_timeout_us_ -
        std::min(budget_now - task_started_us_, task_timeout_us_);
    if (trajectory_ns <= 0 ||
        !dummy_ref::host::manual_reference_move_budget(
            static_cast<std::uint64_t>((trajectory_ns + 999) / 1000), 175000000,
            3000000, 0, 2000000, status_.permit_remaining_us,
            std::min(step_remaining, task_remaining), &execution_budget)) {
      fail("完整轨迹与到位余量超出许可或单项期限");
      return;
    }
    for (const auto &name : names_) {
      control_msgs::msg::JointTolerance tolerance;
      tolerance.name = name;
      tolerance.position = 0.001;
      request.goal_tolerance.push_back(tolerance);
    }
    before_ = state_.position_rad;
    target_ = target;
    goal_seq_ = state_.state_seq;
    feedback_moved_ = false;
    goal_running_ = false;
    cancel_requested_ = false;
    cancel_ack_ = false;
    cancel_test_ = cancel;
    result_waiting_ = false;
    busy_ = true;
    pending_since_ = steady_us();
    const auto ticket = ++ticket_;
    if (!renewal_retry) {
      goal_sent_at_ = pending_since_;
      goal_deadline_us_ = goal_sent_at_ + execution_budget;
      renewal_goal_deadline_ = goal_deadline_us_;
      renewal_until_ = goal_sent_at_ + 2000000;
      renewal_state_ = state_;
      renewal_status_ = status_;
      renewal_status_at_ = status_at_;
      renewal_trajectory_ = trajectory;
      renewal_target_ = target;
      renewal_planned_ = planned;
      renewal_cancel_ = cancel;
    } else {
      goal_deadline_us_ = renewal_goal_deadline_;
      record("RENEWAL_RETRY_SENT", "原目标/首次发送deadline不变");
    }
    const auto digest = trajectory_digest(trajectory);
    rclcpp_action::Client<Fjt>::SendGoalOptions opts;
    opts.goal_response_callback = [this, ticket,
                                   digest, renewal_retry](GoalHandle::SharedPtr handle) {
      if (ticket != ticket_)
        return;
      if (!handle) {
        busy_ = false;
        if (renewal_retry) {
          fail("公开FJT续租后再次拒绝目标");
          return;
        }
        rejected_uuid_ = action_->sent_uuid;
        renewal_phase_ = 1;
        return;
      }
      goal_ = handle;
      std::ostringstream uuid;
      for (auto b : handle->get_goal_id())
        uuid << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
      record("GOAL_ACCEPTED",
             "uuid=" + uuid.str() + " trajectory_sha256=" + digest);
    };
    opts.result_callback = [this,
                            ticket](const GoalHandle::WrappedResult &result) {
      if (ticket != ticket_)
        return;
      record("GOAL_RESULT", "result_code=" + std::to_string(static_cast<int>(result.code)) +
             " error_code=" + (result.result ? std::to_string(result.result->error_code) : "missing") +
             " error_string=" + (result.result ? result.result->error_string : "missing") +
             " cancel_requested=" + std::to_string(cancel_requested_) +
             " goal_running=" + std::to_string(goal_running_));
      busy_ = false;
      goal_.reset();
      goal_deadline_us_ = 0;
      if (cancel_test_) {
        if (result.code != rclcpp_action::ResultCode::CANCELED ||
            !cancel_requested_ || !goal_running_) {
          fail("未确认运行中的目标取消终态");
          return;
        }
        record("CANCEL_TERMINAL", "公开gate返回CANCELED，继续核对停车反馈");
        result_waiting_ = true;
        result_seq_ = state_.state_seq;
      } else {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
            !result.result ||
            result.result->error_code != Fjt::Result::SUCCESSFUL) {
          fail("公开FJT非成功终态");
          return;
        }
        record("GOAL_TERMINAL", "公开gate返回SUCCEEDED，继续核对到位反馈");
        result_waiting_ = true;
        result_seq_ = goal_seq_;
      }
    };
    action_->async_send_goal(request, opts);
  }
  void await_next_motion_ready() {
    if (!fresh() || state_.session_id != arrival_state_.session_id ||
        status_.session_id != arrival_status_.session_id ||
        state_.profile_id != arrival_state_.profile_id || status_.profile_id != arrival_status_.profile_id ||
        state_.reference_epoch != arrival_state_.reference_epoch || status_.reference_epoch != arrival_status_.reference_epoch ||
        state_.motion_epoch != arrival_state_.motion_epoch || status_.motion_epoch != arrival_status_.motion_epoch ||
        std::uint32_t(state_.management_epoch - arrival_state_.management_epoch) > 2 ||
        std::uint32_t(status_.management_epoch - arrival_status_.management_epoch) > 2 ||
        state_.core_state != 4 || status_.core_state != 4 || !state_.hold_ready || status_.evidence_lost) {
      fail("等待下一许可时原Hold身份/epoch改变");
      return;
    }
    trigger("bootstrap_status", [this](const auto &response) {
      if (response.success && response.message == "ENABLED_HOLD_PERMIT_REQUIRED") {
        if (!next_permit_requested_) {
          next_permit_requested_ = true;
          next_permit_management_ = state_.management_epoch;
          const bool sent = trigger("set_bench_permit", [this](const auto &permit) {
            record("NEXT_PERMIT_REQUEST", permit.message);
            if (!permit.success && permit.message.find("QUEUED") == std::string::npos &&
                permit.message.find("queued") == std::string::npos)
              fail("下一运动显式许可请求未受理: " + permit.message);
          });
          if (!sent)
            next_permit_requested_ = false;
        }
        return;
      }
      if (!response.success ||
          (response.message.find("HOLD") == std::string::npos && response.message.find("LEASE_RENEWED") == std::string::npos))
        return;
      // 显式请求只排管理操作；仍须新管理水位、真实许可及双源反馈确认，queued不是Ready。
      if ((!next_permit_requested_ || std::int32_t(state_.management_epoch - next_permit_management_) > 0) && moving_ready() && state_.core_state == 4 && status_.core_state == 4 && state_.hold_ready &&
          state_.management_epoch == status_.management_epoch &&
          state_.motion_epoch == arrival_state_.motion_epoch && status_.motion_epoch == arrival_status_.motion_epoch &&
          state_.session_id == arrival_state_.session_id && status_.session_id == arrival_status_.session_id &&
          state_.profile_id == arrival_state_.profile_id && status_.profile_id == arrival_status_.profile_id &&
          std::uint32_t(state_.management_epoch - arrival_state_.management_epoch) <= 2 &&
          std::uint32_t(status_.management_epoch - arrival_status_.management_epoch) <= 2 &&
          state_.can_sent_mask == 63 && status_.can_sent_mask == 63 &&
          !state_.can_failed_mask && !status_.can_failed_mask &&
          state_.reference_epoch == arrival_state_.reference_epoch && status_.reference_epoch == arrival_status_.reference_epoch &&
          std::int32_t(state_.state_seq - arrival_state_.state_seq) > 0 && status_at_ > arrival_status_at_) {
        next_motion_waiting_ = false;
        record("NEXT_MOTION_READY", "Gate真实剩余许可及fresh双Hold/bootstrap确认");
      }
    });
  }
  bool confirm_goal() {
    // Cancel响应与Action终态回调允许任意先后；两者齐备后才确认停车。
    if (cancel_test_ && !cancel_ack_)
      return false;
    // CompleteMotion正常消费permit；已完成轨迹的真实到位不能要求下一次运动许可。
    // Begin推进一次，Complete握手封口或Cancel推进一次，最终均为发送前+2。
    const auto completed_motion = renewal_state_.motion_epoch + 2U;
    if (result_waiting_ && fresh() &&
        (state_.session_id != renewal_state_.session_id || status_.session_id != renewal_status_.session_id ||
         state_.reference_epoch != renewal_state_.reference_epoch || status_.reference_epoch != renewal_status_.reference_epoch)) {
      fail("到位确认时原owner/reference改变");
      return false;
    }
    if (!result_waiting_ || !fresh() || state_.position_valid_mask != 63 ||
        state_.velocity_valid_mask != 63 || status_.evidence_lost ||
        state_.device_id != renewal_state_.device_id || status_.device_id != renewal_status_.device_id ||
        state_.profile_id != renewal_state_.profile_id || status_.profile_id != renewal_status_.profile_id ||
        state_.session_id != renewal_state_.session_id || status_.session_id != renewal_status_.session_id ||
        state_.reference_epoch != renewal_state_.reference_epoch || status_.reference_epoch != renewal_status_.reference_epoch ||
        state_.motion_epoch != completed_motion || status_.motion_epoch != completed_motion ||
        std::int32_t(state_.state_seq - result_seq_) <= 0 ||
        !state_.hold_ready || state_.core_state != 4 ||
        status_.core_state != 4 || state_.can_sent_mask != 63 ||
        status_.can_sent_mask != 63 || state_.can_failed_mask ||
        status_.can_failed_mask)
      return false;
    bool moved = false;
    std::ostringstream evidence;
    evidence << std::setprecision(10);
    for (unsigned i = 0; i < 6; ++i) {
      if (std::abs(state_.estimated_velocity_rad_s[i]) > 0.002)
        return false;
      if (!cancel_test_ &&
          std::abs(state_.position_rad[i] - target_[i]) > 0.001)
        return false;
      if (std::abs(state_.position_rad[i] - before_[i]) > 0.0005)
        moved = true;
      evidence << " axis=" << i << " target=" << target_[i]
               << " actual=" << state_.position_rad[i]
               << " delta=" << state_.position_rad[i] - before_[i]
               << " error=" << state_.position_rad[i] - target_[i];
    }
    if (!cancel_test_ && !moved && !feedback_moved_) {
      fail("Action成功但没有源反馈位移证据");
      return false;
    }
    record(cancel_test_ ? "CANCEL_STOP_CONFIRMED" : "ARRIVAL_CONFIRMED",
           evidence.str());
    result_waiting_ = false;
    next_motion_waiting_ = true;
    next_permit_requested_ = false;
    next_motion_until_ = steady_us() + 15000000;
    arrival_state_ = state_;
    arrival_status_ = status_;
    arrival_status_at_ = status_at_;
    return true;
  }
  void single_axis() {
    if (result_waiting_) {
      if (confirm_goal())
        ++substep_;
      return;
    }
    if (substep_ >= 12) {
      advance();
      return;
    }
    if (substep_ % 2) {
      send(make_trajectory(origin_), origin_);
      return;
    }
    auto target = bounded_baseline_target(origin_, lower_, upper_, velocity_,
                                          acceleration_, substep_ / 2);
    if (!target.valid) {
      fail("单轴没有安全可行空间");
      return;
    }
    send(make_trajectory(target.positions), target.positions);
  }
  void multi_axis() {
    if (result_waiting_) {
      if (confirm_goal())
        ++substep_;
      return;
    }
    if (substep_ >= 2) {
      advance();
      return;
    }
    if (substep_ == 1) {
      send(make_trajectory(origin_), origin_);
      return;
    }
    auto target = bounded_baseline_target(origin_, lower_, upper_, velocity_,
                                          acceleration_, -1);
    if (!target.valid) {
      fail("多轴没有安全可行空间");
      return;
    }
    send(make_trajectory(target.positions), target.positions);
  }
  bool unique_planning_services() {
    std::map<std::string, unsigned> counts{{"/plan_kinematic_path", 0},
                                           {"/get_planning_scene", 0},
                                           {"/apply_planning_scene", 0},
                                           {"/check_state_validity", 0}};
    // 同FQN的move_group内部节点不代表重复server；server数量由DDS图计数。
    for (const auto &service : counts)
      if (count_services(service.first) != 1)
        return false;
    std::map<std::string, std::string> providers;
    const auto node_names = get_node_names();
    const std::set<std::string> unique_nodes(node_names.begin(), node_names.end());
    for (const auto &full : unique_nodes) {
      const auto slash = full.find_last_of('/');
      const auto name = full.substr(slash + 1);
      const auto ns = slash == 0 ? std::string("/") : full.substr(0, slash);
      for (const auto &service :
           get_service_names_and_types_by_node(name, ns)) {
        const auto it = counts.find(service.first);
        if (it != counts.end()) {
          ++it->second;
          providers[service.first] = full;
        }
      }
    }
    const auto &planning_provider = providers["/plan_kinematic_path"];
    for (const auto &count : counts) {
      if (count.second != 1)
        return false;
      const auto &provider = providers[count.first];
      if (provider == planning_provider)
        continue;
      // MoveIt PlanningSceneMonitor在同namespace的private helper提供get_scene。
      // 仅允许当前planner FQN加非空十进制实例后缀，其他公开服务不可分流。
      const auto private_prefix = planning_provider + "_private_";
      if (count.first != "/get_planning_scene" ||
          provider.compare(0, private_prefix.size(), private_prefix) != 0 ||
          provider.size() == private_prefix.size() ||
          !std::all_of(provider.begin() + private_prefix.size(), provider.end(),
                       [](char ch) { return ch >= '0' && ch <= '9'; }))
        return false;
    }
    return true;
  }
  void request_plan(const std::array<double, 6> &target,
                    bool expected_rejection = false, bool validation_only = false) {
    if (!unique_planning_services()) {
      fail("规划/场景服务缺失或存在重复实例");
      return;
    }
    if (!moving_ready()) {
      fail("规划前反馈/许可不满足");
      return;
    }
    auto req = std::make_shared<Plan::Request>();
    auto &plan = req->motion_plan_request;
    plan.group_name = group_;
    plan.num_planning_attempts = 1;
    plan.allowed_planning_time = 5;
    double scaling = 1;
    for (unsigned i = 0; i < 6; ++i)
      scaling = std::min(scaling, std::min(0.03490658503988659 / velocity_[i],
                                           0.1 / acceleration_[i]));
    plan.max_velocity_scaling_factor = std::min(0.1, scaling);
    plan.max_acceleration_scaling_factor = std::min(0.1, scaling);
    plan.start_state.joint_state.name = names_;
    plan.start_state.joint_state.position.assign(state_.position_rad.begin(),
                                                 state_.position_rad.end());
    moveit_msgs::msg::Constraints constraint;
    for (unsigned i = 0; i < 6; ++i) {
      moveit_msgs::msg::JointConstraint joint;
      joint.joint_name = names_[i];
      joint.position = target[i];
      joint.tolerance_above = 0.0001;
      joint.tolerance_below = 0.0001;
      joint.weight = 1;
      constraint.joint_constraints.push_back(joint);
    }
    plan.goal_constraints.push_back(constraint);
    if (validation_only) {
      if (!obstacle_probe_request_)
        obstacle_probe_request_ = std::make_shared<Plan::Request>(*req);
      // A/B/A复用完全相同的字段（包括静态start），不依赖CDR padding或重采编码器噪声。
      req = std::make_shared<Plan::Request>(*obstacle_probe_request_);
      const auto &start = req->motion_plan_request.start_state.joint_state.position;
      for (unsigned i = 0; i < 6; ++i)
        if (std::abs(state_.position_rad[i] - start[i]) > 0.001) {
          fail("圆柱只读规划探针期间实际起点漂移");
          return;
        }
    }
    request<Plan>(
        plan_, req, [this, target, expected_rejection, validation_only](const auto &response) {
          const auto &result = response.motion_plan_response;
          if (expected_rejection) {
            if (result.error_code.val ==
                moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
              fail("圆柱目标应碰撞但规划成功，场景/拒绝证据不足 MoveIt error=" +
                   std::to_string(result.error_code.val));
              return;
            }
            if (validation_only && (!obstacle_clear_before_ || !obstacle_contact_confirmed_ || !cylinder_owned_)) {
              fail("圆柱拒绝规划缺少同请求无障碍成功或独立同ID接触证据");
              return;
            }
            // generic FAILURE仅在独立接触+clear前成功后暂记，必须再以移除后成功完成A/B/A。
            if (!(validation_only && result.error_code.val == moveit_msgs::msg::MoveItErrorCodes::FAILURE) &&
                result.error_code.val != moveit_msgs::msg::MoveItErrorCodes::
                                             START_STATE_IN_COLLISION &&
                result.error_code.val !=
                    moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION &&
                result.error_code.val !=
                    moveit_msgs::msg::MoveItErrorCodes::GOAL_STATE_INVALID &&
                result.error_code.val !=
                    moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN &&
                result.error_code.val !=
                    moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED) {
              fail("圆柱规划失败原因并非碰撞路径拒绝 MoveIt error=" +
                   std::to_string(result.error_code.val));
              return;
            }
            record("OBSTACLE_PLAN_REJECTED",
                   "MoveIt error=" + std::to_string(result.error_code.val));
            obstacle_blocked_plan_ = true;
            ++substep_;
            return;
          }
          if (validation_only) {
            const auto &trajectory = result.trajectory.joint_trajectory;
            bool valid = result.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
                         trajectory.joint_names == names_ && trajectory.points.size() >= 2 &&
                         result.trajectory.multi_dof_joint_trajectory.points.empty();
            for (const auto &point : trajectory.points)
              valid = valid && point.positions.size() == 6 &&
                      std::all_of(point.positions.begin(), point.positions.end(), [](double value) { return std::isfinite(value); });
            if (valid) {
              const auto &start = obstacle_probe_request_->motion_plan_request.start_state.joint_state.position;
              for (unsigned i = 0; i < 6; ++i)
                valid = valid && std::abs(trajectory.points.front().positions[i] - start[i]) <= 0.0005 &&
                        std::abs(trajectory.points.back().positions[i] - target[i]) <= 0.0001;
            }
            if (!valid) {
              fail("圆柱无障碍规划探针没有成功完整路径 MoveIt error=" + std::to_string(result.error_code.val));
              return;
            }
            if (substep_ == 1 && !obstacle_clear_before_) {
              obstacle_clear_before_ = true;
              record("CYLINDER_CLEAR_PLAN_BEFORE", "冻结请求SUCCESS；仅Probe不执行");
            } else if (substep_ == 7 && obstacle_clear_before_ && obstacle_contact_confirmed_ && obstacle_blocked_plan_ && !cylinder_owned_) {
              record("CYLINDER_CLEAR_PLAN_AFTER", "移除圆柱后同一冻结请求SUCCESS；仅Probe不执行");
              record("CYLINDER_ABA_CONFIRMED", "同一静态start/target请求：clear SUCCESS→同ID碰撞拒绝→clear SUCCESS");
              advance();
            } else {
              fail("圆柱A/B/A探针步骤或证据不完整");
            }
            return;
          }
          if (result.error_code.val !=
                  moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
              !result.trajectory.multi_dof_joint_trajectory.points.empty()) {
            fail("REST往返无合法完整路径");
            return;
          }
          auto trajectory = result.trajectory.joint_trajectory;
          if (trajectory.points.size() < 2) {
            fail("规划没有完整路点");
            return;
          }
          // 只降低已规划路径速度；按整段时间等比例缩放，保留所有路点。
          double scale = 1;
          auto previous = state_.position_rad;
          std::array<double, 6> previous_velocity{}, previous_acceleration{};
          double previous_t = 0;
          for (const auto &p : trajectory.points) {
            if (p.positions.size() != 6) {
              fail("规划轴数错误");
              return;
            }
            if (rest_aligning_ &&
                (p.velocities.size() != 6 || p.accelerations.size() != 6)) {
              fail("REST归位需要完整五次样条导数");
              return;
            }
            const double time = rclcpp::Duration(p.time_from_start).seconds(),
                         dt = time - previous_t;
            for (unsigned i = 0; i < 6; ++i) {
              const double v = std::min(velocity_[i], 0.03490658503988659),
                           a = std::min(acceleration_[i], 0.1);
              if (dt > 0)
                scale = std::max(
                    scale, 2 * std::abs(p.positions[i] - previous[i]) / dt / v);
              if (p.velocities.size() == 6)
                scale = std::max(scale, std::abs(p.velocities[i]) / v);
              if (p.accelerations.size() == 6)
                scale = std::max(scale,
                                 std::sqrt(std::abs(p.accelerations[i]) / a));
              if (rest_aligning_ && dt > 0) {
                const auto bounds = baseline_quintic_bounds(
                    previous[i], previous_velocity[i], previous_acceleration[i],
                    p.positions[i], p.velocities[i], p.accelerations[i], dt);
                if (!bounds.valid) {
                  fail("REST归位样条无效");
                  return;
                }
                scale = std::max(
                    scale, std::max(bounds.max_velocity / v,
                                    std::sqrt(bounds.max_acceleration / a)));
              }
              if (rest_aligning_) {
                previous_velocity[i] = p.velocities[i];
                previous_acceleration[i] = p.accelerations[i];
              }
            }
            std::copy(p.positions.begin(), p.positions.end(), previous.begin());
            previous_t = time;
          }
          for (auto &p : trajectory.points) {
            p.time_from_start = rclcpp::Duration::from_seconds(
                rclcpp::Duration(p.time_from_start).seconds() * scale);
            for (auto &v : p.velocities)
              v /= scale;
            for (auto &a : p.accelerations)
              a /= (scale * scale);
          }
          planned_trajectory_ = trajectory;
          planned_target_ = target;
          plan_ready_ = true;
          record("PLAN_CONFIRMED",
                 "完整路点数=" + std::to_string(trajectory.points.size()));
        });
  }
  void rest_trip() {
    for (unsigned i = 0; i < 6; ++i)
      if (std::abs(rest_[i] - origin_[i]) > 0.001) {
        fail("标定终点不在绑定REST");
        return;
      }
    if (result_waiting_) {
      if (confirm_goal())
        ++substep_;
      return;
    }
    if (substep_ >= 2) {
      advance();
      return;
    }
    if (plan_ready_) {
      plan_ready_ = false;
      send(planned_trajectory_, planned_target_, true);
      return;
    }
    auto target = bounded_baseline_target(origin_, lower_, upper_, velocity_,
                                          acceleration_, 0);
    if (!target.valid) {
      fail("REST往返无安全目标");
      return;
    }
    request_plan(substep_ == 0 ? target.positions : rest_);
  }
  moveit_msgs::msg::CollisionObject cylinder(bool remove = false) const {
    moveit_msgs::msg::CollisionObject object;
    object.id = "manual_ros_baseline_cylinder";
    object.header.frame_id = cylinder_frame_;
    object.operation = remove ? object.REMOVE : object.ADD;
    if (remove)
      return object;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.assign(cylinder_dimensions_.begin(),
                                cylinder_dimensions_.end());
    geometry_msgs::msg::Pose pose;
    pose.position.x = cylinder_pose_[0];
    pose.position.y = cylinder_pose_[1];
    pose.position.z = cylinder_pose_[2];
    pose.orientation.x = cylinder_pose_[3];
    pose.orientation.y = cylinder_pose_[4];
    pose.orientation.z = cylinder_pose_[5];
    pose.orientation.w = cylinder_pose_[6];
    // CollisionObject.pose相对header；primitive_pose相对object。
    object.pose = pose;
    geometry_msgs::msg::Pose local_pose;
    local_pose.orientation.w = 1.0;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(local_pose);
    return object;
  }
  void obstacle() {
    if (cylinder_pose_.size() != 7 || cylinder_dimensions_.size() != 2 ||
        cylinder_dimensions_[0] <= 0 || cylinder_dimensions_[1] <= 0) {
      fail("缺少绑定圆柱场景参数");
      return;
    }
    if (substep_ == 0 || substep_ == 2 || substep_ == 6) {
      auto req = std::make_shared<Scene::Request>();
      req->components.components =
          moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
      request<Scene>(scene_, req, [this](const auto &response) {
        const auto &objects = response.scene.world.collision_objects;
        auto it = std::find_if(
            objects.begin(), objects.end(), [](const auto &object) {
              return object.id == "manual_ros_baseline_cylinder";
            });
        if (substep_ == 0 || substep_ == 6) {
          if (it != objects.end()) {
            fail("圆柱场景未清理或同名对象已存在");
            return;
          }
          if (substep_ == 6) {
            ++substep_; // 场景确已移除，再以同一请求证明可规划恢复。
            return;
          }
          ++substep_;
          return;
        }
        if (it == objects.end() || it->header.frame_id != cylinder_frame_ ||
            it->primitives.size() != 1 ||
            it->primitives[0].type !=
                shape_msgs::msg::SolidPrimitive::CYLINDER ||
            (it->primitives[0].dimensions.size() !=
                 cylinder_dimensions_.size() ||
             !std::equal(it->primitives[0].dimensions.begin(),
                         it->primitives[0].dimensions.end(),
                         cylinder_dimensions_.begin())) ||
            it->primitive_poses.size() != 1 || !it->meshes.empty() || !it->planes.empty()) {
          fail("圆柱场景读回不一致");
          return;
        }
        const auto expected = cylinder();
        const auto transform = [](const geometry_msgs::msg::Pose &pose,
                                  Eigen::Isometry3d &result) {
          const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
          Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x,
                                         pose.orientation.y, pose.orientation.z);
          if (!position.allFinite() || !orientation.coeffs().allFinite() ||
              std::abs(orientation.squaredNorm() - 1.0) > 1e-6)
            return false;
          result = Eigen::Isometry3d::Identity();
          result.translation() = position;
          result.linear() = orientation.normalized().toRotationMatrix();
          return true;
        };
        Eigen::Isometry3d actual_object, actual_primitive, expected_object, expected_primitive;
        if (!transform(it->pose, actual_object) ||
            !transform(it->primitive_poses[0], actual_primitive) ||
            !transform(expected.pose, expected_object) ||
            !transform(expected.primitive_poses[0], expected_primitive)) {
          fail("圆柱位姿包含无效变换");
          return;
        }
        // MoveIt可改变object/local分解，实际header-frame变换必须相同。
        const Eigen::Isometry3d actual_world = actual_object * actual_primitive;
        const Eigen::Isometry3d expected_world = expected_object * expected_primitive;
        if ((actual_world.translation() - expected_world.translation()).cwiseAbs().maxCoeff() > 1e-8 ||
            (actual_world.linear() - expected_world.linear()).cwiseAbs().maxCoeff() > 1e-8) {
          fail("圆柱位姿读回不一致");
          return;
        }
        record("SCENE_ROUNDTRIP_CONFIRMED", "id/frame/圆柱尺寸/位姿完全一致");
        ++substep_;
      });
    } else if (substep_ == 1 || substep_ == 5) {
      if (substep_ == 1 && !obstacle_clear_before_) {
        // 模型REST固定夹具；此目标只作Validity/只读规划Probe，不发FJT。
        const auto target = bounded_baseline_target(rest_, lower_, upper_, velocity_, acceleration_, 0);
        if (!target.valid) {
          fail("圆柱用例没有安全目标");
          return;
        }
        obstacle_target_ = target.positions;
        request_plan(obstacle_target_, false, true);
        return;
      }
      auto req = std::make_shared<Apply::Request>();
      req->scene.is_diff = true;
      req->scene.world.collision_objects.push_back(cylinder(substep_ == 5));
      request<Apply>(apply_, req, [this](const auto &response) {
        if (!response.success)
          fail("圆柱场景修改失败");
        else {
          cylinder_owned_ = substep_ == 1;
          ++substep_;
        }
      });
    } else if (substep_ == 3) {
      if (!obstacle_start_valid_) {
        if (!moving_ready()) {
          fail("圆柱起点验证时反馈或许可不满足");
          return;
        }
        auto req = std::make_shared<Validity::Request>();
        req->group_name = group_;
        req->robot_state.joint_state.name = names_;
        req->robot_state.joint_state.position =
            obstacle_probe_request_->motion_plan_request.start_state.joint_state.position;
        request<Validity>(validity_, req, [this](const auto &response) {
          if (!response.valid) {
            fail("新增圆柱遮挡当前REST起点");
            return;
          }
          obstacle_start_valid_ = true;
          record("CYLINDER_START_CLEAR",
                 "GetStateValidity确认当前REST起点不碰撞");
        });
        return;
      }
      auto req = std::make_shared<Validity::Request>();
      req->group_name = group_;
      req->robot_state.joint_state.name = names_;
      req->robot_state.joint_state.position.assign(obstacle_target_.begin(),
                                                   obstacle_target_.end());
      request<Validity>(validity_, req, [this](const auto &response) {
        bool cylinder_contact = false;
        for (const auto &contact : response.contacts)
          if (contact.contact_body_1 == "manual_ros_baseline_cylinder" ||
              contact.contact_body_2 == "manual_ros_baseline_cylinder")
            cylinder_contact = true;
        if (response.valid || !cylinder_contact) {
          fail("绑定圆柱没有产生可归因的目标碰撞，拒绝假通过");
          return;
        }
        obstacle_contact_confirmed_ = true;
        record("CYLINDER_COLLISION_CONFIRMED",
               "GetStateValidity含同ID圆柱接触");
        ++substep_;
      });
    } else if (substep_ == 4)
      request_plan(obstacle_target_, true, true);
    else if (substep_ == 7)
      request_plan(obstacle_target_, false, true);
  }
  void cancel_check() {
    if (result_waiting_) {
      if (confirm_goal())
        advance();
      return;
    }
    auto target = bounded_baseline_target(state_.position_rad, lower_, upper_,
                                          velocity_, acceleration_, 0);
    if (!target.valid) {
      fail("取消用例没有安全目标");
      return;
    }
    send(make_trajectory(target.positions, 8.0), target.positions, false, true);
  }
  static bool nonempty_file(const std::string &path) {
    std::error_code error;
    return !path.empty() && std::filesystem::is_regular_file(path, error) &&
           std::filesystem::file_size(path, error) > 0 && !error;
  }
  void release() {
    if (!management_requested_) {
      flow_.release_confirmed(false);
      record("NO_OWNERSHIP", "本次未提交管理请求，不取消其他owner，不发送D10");
      record("TERMINAL", description());
      return;
    }
    if (steady_us() - step_started_ > 20000000) {
      flow_.release_confirmed(false);
      record("TERMINAL", description());
      return;
    }
    if (substep_ == 0) {
      if (steady_us() < release_retry_at_)
        return;
      // 服务暂不准入时保留清理意图；重试不重置本阶段20秒总截止。
      // gate 先停止当前公开目标再发D10；QUEUED不等于释放已完成。
      trigger("disable_and_release", [this](const auto &response) {
        record("RELEASE_REQUEST", response.message);
        if (response.message == "RELEASE_UNAVAILABLE") {
          release_retry_at_ = steady_us() + 200000;
          return;
        }
        if (response.message == "RELEASE_QUEUED" ||
            response.message == "EVIDENCE_INCOMPLETE;RELEASE_PENDING" ||
            response.message == "EVIDENCE_INCOMPLETE;RELEASED_IDLE" ||
            response.message == "EVIDENCE_INCOMPLETE;RELEASED_FAULT" ||
            (response.success && response.message == "RELEASED_IDLE"))
          ++substep_;
        else {
          flow_.release_confirmed(false);
          record("TERMINAL", description());
        }
      });
      return;
    }
    if (substep_ == 1) {
      trigger("bootstrap_status", [this](const auto &response) {
        const bool dual_idle =
            fresh() && state_.state_seq != release_seq_ && !state_.session_id &&
            !status_.session_id && !status_.real_motion_permitted &&
            !status_.permit_remaining_us && !state_.hold_ready &&
            state_.reference_epoch == expected_reference_ &&
            status_.reference_epoch == expected_reference_ &&
            state_.core_state == (expected_reference_ ? 2 : 1) &&
            state_.core_state == status_.core_state;
        const bool dual_fault = fresh() && state_.state_seq!=release_seq_ &&
            state_.core_state==9 && status_.core_state==9 && !state_.session_id && !status_.session_id &&
            state_.profile_id==status_.profile_id && state_.motion_epoch==status_.motion_epoch &&
            state_.management_epoch==status_.management_epoch && state_.operation_id==status_.operation_id &&
            state_.reference_epoch==expected_reference_ &&
            status_.reference_epoch==expected_reference_ && !state_.hold_ready &&
            state_.raw_position_valid_mask==63 && !state_.can_failed_mask && !status_.can_failed_mask &&
            !status_.real_motion_permitted && !status_.permit_remaining_us && status_.release_committed &&
            status_.cleanup_state==3 && !status_.evidence_lost;
        if(response.message=="EVIDENCE_INCOMPLETE;RELEASED_FAULT" && dual_fault) {
          released_=true;released_fault_=true;
          flow_.fail("设备故障已失能释放，运动证据仍不完整",steady_us());
          record("RELEASE_CONFIRMED",response.message);++substep_;
          return;
        }
        if (((response.success && response.message=="RELEASED_IDLE") ||
             response.message=="EVIDENCE_INCOMPLETE;RELEASED_IDLE") && dual_idle && status_.release_committed) {
          if(!response.success)flow_.fail("失能释放已确认，运动证据仍不完整",steady_us());
          // release_committed仅由gate原页/双状态确认后发布；上层不可由零填充状态替代。
          released_ = true;
          record("RELEASE_CONFIRMED", response.message);
          ++substep_;
        }
      });
      return;
    }
    if (substep_ == 2 && cylinder_owned_) {
      auto req = std::make_shared<Apply::Request>();
      req->scene.is_diff = true;
      req->scene.world.collision_objects.push_back(cylinder(true));
      request<Apply>(apply_, req, [this](const auto &response) {
        if (!response.success)
          fail("清理圆柱场景失败");
        cylinder_owned_ = false;
        ++substep_;
      });
      return;
    }
    if (!nonempty_file(timeline_path_) || !nonempty_file(raw_path_))
      fail("缺少gate原始证据或goal_timeline");
    flow_.release_confirmed(released_);
    record("EVIDENCE", "goal_timeline=" + timeline_path_ + " raw=" + raw_path_);
    record("TERMINAL", description());
  }
  ManualRosBaseline flow_;
  BaselineStage entry_{BaselineStage::kReadOnly};
  bool manual_mode_{}, manual_ready_{}, manual_external_release_{},
      rest_aligning_{};
  std::uint64_t manual_session_{}, manual_owner_lost_at_{};
  bool management_requested_{}, obstacle_start_valid_{}, reference_bound_{};
  bool obstacle_clear_before_{}, obstacle_contact_confirmed_{}, obstacle_blocked_plan_{};
  Plan::Request::SharedPtr obstacle_probe_request_;
  bool automatic_{}, automatic_attempted_{}, busy_{}, released_{},
      cylinder_owned_{}, released_fault_{};
  bool plan_ready_{}, result_waiting_{}, feedback_moved_{}, goal_running_{},
      cancel_test_{}, cancel_requested_{}, cancel_ack_{};
  std::uint64_t boot_{}, state_at_{}, status_at_{}, age_us_{250000},
      status_age_us_{dummy_ref::host::kManualReferenceStatusMaxAgeUs},
      step_started_{}, pending_since_{}, ticket_{}, goal_sent_at_{},
      goal_deadline_us_{}, task_started_us_{}, task_timeout_us_{};
  std::array<std::uint64_t, 6> axis_at_{};
  std::uint64_t release_retry_at_{};
  std::uint32_t expected_reference_{}, release_seq_{}, goal_seq_{},
      result_seq_{};
  unsigned substep_{}, stationary_samples_{};
  std::array<std::uint8_t, 16> device_{};
  Status status_{};
  State state_{};
  std::array<double, 6> lower_{}, upper_{}, rest_{}, velocity_{},
      acceleration_{}, origin_{}, before_{}, target_{}, planned_target_{},
      obstacle_target_{};
  std::vector<std::string> names_;
  std::vector<double> cylinder_pose_, cylinder_dimensions_;
  std::string initial_pose_, configuration_error_, report_path_, timeline_path_,
      raw_path_, group_, frame_, cylinder_frame_, detail_;
  std::ofstream report_;
  trajectory_msgs::msg::JointTrajectory planned_trajectory_;
  GoalHandle::SharedPtr goal_;
  std::shared_ptr<ReceiptActionClient> action_;
  bool next_motion_waiting_{}, next_permit_requested_{};
  std::uint32_t next_permit_management_{};
  std::uint64_t next_motion_until_{}, arrival_status_at_{};
  State arrival_state_{};
  Status arrival_status_{};
  unsigned renewal_phase_{};
  std::uint64_t renewal_until_{}, renewal_goal_deadline_{}, renewal_status_at_{};
  State renewal_state_{};
  Status renewal_status_{};
  trajectory_msgs::msg::JointTrajectory renewal_trajectory_;
  std::array<double, 6> renewal_target_{};
  bool renewal_planned_{}, renewal_cancel_{};
  std::string rejected_uuid_;
  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> services_;
  rclcpp::Client<Hardware>::SharedPtr hardware_;
  rclcpp::Client<Switch>::SharedPtr controllers_;
  rclcpp::Client<Controllers>::SharedPtr controller_list_;
  bool controllers_active_ = false;
  std::vector<std::string> pending_controller_activation_;
  rclcpp::Client<Plan>::SharedPtr plan_;
  rclcpp::Client<Scene>::SharedPtr scene_;
  rclcpp::Client<Apply>::SharedPtr apply_;
  rclcpp::Client<Validity>::SharedPtr validity_;
  rclcpp::Service<Trigger>::SharedPtr start_service_, status_service_,
      manual_start_service_, manual_status_service_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  rclcpp::Subscription<State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace dummy_motion_client
namespace {
volatile std::sig_atomic_t baseline_stop_requested = 0;
void request_baseline_stop(int) { baseline_stop_requested = 1; }
} // namespace
int main(int argc, char **argv) {
  // SIGINT采用有界异步finally：保持executor存活直至释放确认或清理期限。
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<dummy_motion_client::ManualRosBaselineNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::signal(SIGINT, request_baseline_stop);
  std::signal(SIGTERM, request_baseline_stop);
  bool stopping = false;
  while (rclcpp::ok()) {
    if (baseline_stop_requested && !stopping) {
      stopping = true;
      node->shutdown_cleanup();
    }
    executor.spin_some();
    if (stopping && node->finished())
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::shutdown();
  return 0;
}
