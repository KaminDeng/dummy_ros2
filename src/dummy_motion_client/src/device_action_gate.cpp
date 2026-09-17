#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include "controller_manager/controller_manager.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_motion_client/device_profile.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_motion_client/native_goal_validation.hpp"
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace dummy_motion_client {
namespace wire=dp::protocol::dummy_robot_v2;
using Follow=control_msgs::action::FollowJointTrajectory;
using GoalHandle=rclcpp_action::ServerGoalHandle<Follow>;
using Controller=dummy_ros_control::GuardedJointTrajectoryController;

template<std::size_t Size> std::array<std::uint8_t,Size> hex_id(const std::string &text) {
  if(text.size()!=Size*2) throw std::invalid_argument("invalid hexadecimal identity length");
  std::array<std::uint8_t,Size> bytes{};
  for(std::size_t i=0;i<Size;++i) {
    unsigned value=0; const auto result=std::from_chars(text.data()+i*2,text.data()+i*2+2,value,16);
    if(result.ec!=std::errc{} || result.ptr!=text.data()+i*2+2) throw std::invalid_argument("invalid hexadecimal identity");
    bytes[i]=value;
  }
  return bytes;
}

class GuardedPort final : public ActionPort {
 public:
  std::shared_ptr<Controller> controller;
  dummy_ros_control::CommandIdentity identity{};
  dummy_ros_control::GoalUuid uuid{};
  std::uint64_t motion_generation=0, started_us=0;
  bool executing=false, failed=false, publishing=false;
  bool direct_hold=false, direct_hold_started=false;
  std::size_t active_count=0;
  Trajectory trajectory;
  Position hold_position{};
  void prime(const Position &position,std::uint32_t epoch,std::uint64_t generation) override {
    executing=false; identity.motion_epoch=epoch; motion_generation=generation; hold_position=position;
    dummy_ros_control::set_command_identity(identity);
    if(controller) {
      direct_hold=false;
      if(!controller->prime_position(position)) { failed=true; return; }
      controller->set_update_sink(dummy_ros_control::controller_update_sink,nullptr);
    } else {
      const auto published = dummy_ros_control::publish_control_update(position.data(),active_count,0,nullptr);
      if(published==0U) { failed=true; return; }
      direct_hold=true; direct_hold_started=true;
    }
    publishing=true;
  }
  void refresh_direct_hold() {
    if(direct_hold && publishing)
      (void)dummy_ros_control::publish_control_update(hold_position.data(),active_count,0,nullptr);
  }
  void start(const Trajectory &value,std::uint64_t generation) override {
    auto message=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    message->joint_names=value.joints;
    for(const auto &point:value.points) {
      trajectory_msgs::msg::JointTrajectoryPoint item;
      item.positions=point.positions; item.velocities=point.velocities; item.accelerations=point.accelerations;
      item.time_from_start=rclcpp::Duration::from_seconds(point.time); message->points.push_back(std::move(item));
    }
    const dummy_ros_control::GoalCredential credential{uuid,dummy_ros_control::trajectory_digest(*message)};
    if(!controller || !controller->register_goal(credential) || !controller->submit_registered_goal(credential,message)) {
      failed=true; return;
    }
    trajectory=value; motion_generation=generation; started_us=management_now_us(); executing=true;
  }
  void disable() override {
    // Keep the last generation publishable while Complete waits for accepted_seq.
    // Called on the same control loop after update/write, so no new sample can race this freeze.
    if(controller) controller->set_update_sink(nullptr,nullptr);
    publishing=false; executing=false;
    direct_hold=false;
  }
  void cancel() override {
    dummy_ros_control::clear_command_identity();
    if(controller) controller->cancel_execution();
  }
};

class DeviceActionGate final : public rclcpp::Node {
 public:
  DeviceActionGate():Node("device_action_gate") {
    parameters_.endpoint=declare_parameter<std::string>("endpoint","");
    parameters_.device=hex_id<16>(declare_parameter<std::string>("device_id",""));
    parameters_.client=hex_id<16>(declare_parameter<std::string>("client_instance_id",""));
    const auto profile=declare_parameter<std::int64_t>("profile_id",0);
    const auto mask=declare_parameter<std::int64_t>("configured_mask",0);
    const auto session=declare_parameter<std::int64_t>("session_id",0);
    const auto permit_text=declare_parameter<std::string>("permit_sha256","");
    const auto permit_us=declare_parameter<std::int64_t>("permit_remaining_us",10000000);
    const auto state_max_age=declare_parameter<std::int64_t>("state_max_age_us",250000);
    if(profile<=0 || profile>UINT32_MAX || (mask!=1 && mask!=63) || session<=0 || permit_us<=0 || permit_us>60000000 ||
       state_max_age<=0 || state_max_age>1000000)
      throw std::invalid_argument("profile/mask/session/permit duration required");
    state_max_age_us_=static_cast<std::uint64_t>(state_max_age);
    parameters_.profile=profile; parameters_.mask=mask;
    controller_name_=declare_parameter<std::string>("guarded_controller","arm_controller");
    const auto bundle=declare_parameter<std::string>("contract_bundle","");
    const auto profile_name=declare_parameter<std::string>("profile_name","");
    if(bundle.empty() || profile_name.empty()) throw std::invalid_argument("contract bundle and profile_name are required");
    device_profile_=load_device_profile(bundle,profile_name,parameters_.profile,parameters_.mask);
    profile_=device_profile_.joints;
    port_.active_count=profile_.joints.size();
    transport_=std::make_unique<DpManagementTransport>(parameters_,DpManagementTransport::Connection::kDeviceWorker);
    if(!transport_->start()) throw std::invalid_argument("invalid device transport parameters");
    client_=std::make_unique<WireManagementClient>(*transport_);
    ManageRequest bootstrap_identity{};
    std::uint8_t device[16]{}, client_id[16]{};
    std::copy(parameters_.device.begin(),parameters_.device.end(),device);
    std::copy(parameters_.client.begin(),parameters_.client.end(),client_id);
    bootstrap_identity.set_device_id(device); bootstrap_identity.set_client_instance_id(client_id);
    bootstrap_identity.set_expected_session_id(static_cast<std::uint64_t>(session));
    const auto permit=hex_id<32>(permit_text);
    bootstrap_=std::make_unique<BootstrapCoordinator>(
      *client_,bootstrap_identity,permit,device_profile_.profile_hash,permit_us);
    gate_=std::make_unique<ActionGate>(*client_,port_,profile_,2000000);
    enable_service_=create_service<std_srvs::srv::Trigger>("~/enable",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success=enable_available_.load() && !enable_requested_.exchange(true);
        response->message=response->success ? "Enable queued; wait for confirmed HOLD before submitting motion" :
          "Enable unavailable phase="+std::to_string(static_cast<unsigned>(bootstrap_->phase()))+
          " transport="+(transport_->ready()?"ready":"not_ready")+
          " boot="+std::to_string(latest_bootstrap_.boot_epoch)+
          " raw="+(latest_bootstrap_.raw_ready?"ready":"not_ready");
      });
    bootstrap_status_service_=create_service<std_srvs::srv::Trigger>("~/bootstrap_status",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const bool operation_complete=management_request_.load()==0 && bootstrap_ && bootstrap_->operation_complete();
        response->success=enabled_ ? gate_->ready() : operation_complete;
        response->message=enabled_ && gate_->ready() ? "ENABLED_HOLD" :
          (operation_complete ? "OPERATION_COMPLETE" :
          (bootstrap_ && bootstrap_->phase()==BootstrapPhase::kFault ? bootstrap_->error() : "BOOTSTRAP_PENDING"));
      });
    const auto add_operation_service=[this](const char *name,ManageOperation operation) {
      management_services_.push_back(create_service<std_srvs::srv::Trigger>(name,
        [this,operation](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          int expected=0;
          const bool available=operation==ManageOperation::kSetPermit ?
            permit_operation_available_.load() : explicit_operation_available_.load();
          response->success=available &&
            management_request_.compare_exchange_strong(expected,static_cast<int>(wire_value(operation)));
          response->message=response->success ? "Operation queued; poll bootstrap_status for final result" :
            "Operation unavailable or another management request is pending";
        }));
    };
    add_operation_service("~/select_control_mode",ManageOperation::kSelectMode);
    add_operation_service("~/grant_session",ManageOperation::kGrantSession);
    add_operation_service("~/capture_bench_reference",ManageOperation::kCaptureReference);
    add_operation_service("~/set_bench_permit",ManageOperation::kSetPermit);
    management_services_.push_back(create_service<std_srvs::srv::Trigger>("~/enable_hold",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success=single_enable_available_.load() && !single_enable_requested_.exchange(true);
        response->message=response->success ? "EnableHold queued; poll bootstrap_status for confirmed HOLD" :
          "EnableHold requires a successfully completed SetBenchPermit";
      }));
    server_=rclcpp_action::create_server<Follow>(this,"/arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Follow::Goal> goal) {
        if(!validated_device_trajectory(*goal,profile_)) {
          RCLCPP_WARN(get_logger(),"DEVICE_GOAL_REJECTED_VALIDATION"); return rclcpp_action::GoalResponse::REJECT;
        }
        if(!ready_.load() || busy_.exchange(true)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle>) { cancel_requested_=true; return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<GoalHandle> handle) { std::lock_guard<std::mutex> lock(goal_mutex_); pending_=std::move(handle); });
  }
  ~DeviceActionGate() override { port_.disable(); port_.cancel(); transport_->stop(); }
  void attach(controller_manager::ControllerManager &manager) {
    if(std::atomic_load(&discovered_controller_)) return;
    for(const auto &spec:manager.get_loaded_controllers()) {
      if(spec.info.name!=controller_name_) continue;
      auto controller=std::dynamic_pointer_cast<Controller>(spec.c);
      if(controller && controller->get_lifecycle_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        std::atomic_store(&discovered_controller_,std::move(controller));
    }
  }
  void cycle() noexcept {
    try { cycle_impl(); }
    catch(const std::exception &error) {
      RCLCPP_ERROR(get_logger(),"DEVICE_GATE_FAULT: %s",error.what());
      gate_->abort("INTERNAL_BRIDGE_FAILURE",management_now_us()); ready_=false;
    }
  }
  void prepare_update() noexcept {
    try {
      const auto now=management_now_us();
      if(!port_.controller) port_.controller=std::atomic_load(&discovered_controller_);
      if(!read_observation(now)) {
        ready_=false; enable_available_=false;
        if(enabled_) { gate_->invalidate_authorization(now); port_.cancel(); }
      }
      gate_->tick(now);
      bootstrap_tick(now);
    } catch(const std::exception &error) {
      RCLCPP_ERROR(get_logger(),"DEVICE_GATE_OBSERVATION: %s",error.what());
      gate_->abort("INTERNAL_BRIDGE_FAILURE",management_now_us()); ready_=false;
    }
  }
 private:
  bool read_observation(std::uint64_t now) {
    const auto observed=transport_->observe();
    if(!observed || observed->state.empty() || observed->status.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"DEVICE_BOOTSTRAP_INPUT_REJECTED_EMPTY"); return false;
    }
    if(!observation_is_fresh(*observed,now,state_max_age_us_)) {
      const auto state_age=now>observed->state_received_us ? now-observed->state_received_us : 0U;
      const auto status_age=now>observed->status_received_us ? now-observed->status_received_us : 0U;
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
        "DEVICE_BOOTSTRAP_INPUT_REJECTED_STALE state_age_us=%llu status_age_us=%llu",
        static_cast<unsigned long long>(state_age),static_cast<unsigned long long>(status_age)); return false;
    }
    wire::DummyDeviceStatusV2 status; wire::DummyJointStateV2 state;
    if(wire::DummyDeviceStatusV2Codec::decode(observed->status.data(),observed->status.size(),&status)!=wire::CodecResult::kOk ||
       wire::DummyJointStateV2Codec::decode(observed->state.data(),observed->state.size(),&state)!=wire::CodecResult::kOk) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"DEVICE_BOOTSTRAP_INPUT_REJECTED_CODEC"); return false;
    }
    const auto reject=[this](const char *reason) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"DEVICE_BOOTSTRAP_INPUT_REJECTED_%s",reason);
      return false;
    };
    if(!std::equal(device_profile_.contract_hash.begin(),device_profile_.contract_hash.end(),status.contract_sha256()) ||
       !std::equal(device_profile_.model_hash.begin(),device_profile_.model_hash.end(),status.model_sha256()) ||
       !std::equal(device_profile_.profile_hash.begin(),device_profile_.profile_hash.end(),status.profile_sha256())) return reject("HASH");
    if(status.boot_epoch()==0 || !status.motion_supported() ||
       status.core_state()==wire_value(WireCoreState::kFault)) return reject("MOTION");
    if(state.boot_epoch()!=status.boot_epoch() || state.session_id()!=status.session_id() ||
       state.profile_id()!=status.profile_id() || state.configured_mask()!=status.configured_mask() ||
       state.reference_epoch()!=status.reference_epoch() || state.freshness_ticket()==0) {
      return reject("STATE");
    }
    if((state.raw_position_valid_mask() & parameters_.mask)!=parameters_.mask) return reject("RAW");
    if ((status.session_id()!=0U && status.control_mode()==0U) ||
       (status.reference_epoch()!=0U && status.reference_kind()==0U) ||
       (status.real_motion_permitted() != (status.permit_remaining_us()!=0U))) return reject("AUTHORITY");
    latest_bootstrap_={status.boot_epoch(),status.session_id(),status.reference_epoch(),status.motion_epoch(),
      status.management_epoch(),state.freshness_ticket(),parameters_.profile,parameters_.mask,true,
      status.motion_supported(),status.control_mode(),status.reference_kind(),status.real_motion_permitted(),
      status.permit_remaining_us()};
    if(bootstrap_) bootstrap_->observe(latest_bootstrap_);
    const int requested=management_request_.exchange(0);
    if(requested!=0) {
      explicit_operation_available_=false;
      const auto operation=static_cast<ManageOperation>(requested);
      if(!bootstrap_->start_operation(latest_bootstrap_,operation,now))
        RCLCPP_WARN(get_logger(),"EXPLICIT_MANAGEMENT_OPERATION_REJECTED operation=%d",requested);
    }
    if(enabled_ && (status.boot_epoch()!=port_.identity.boot || status.session_id()!=port_.identity.session ||
       status.reference_epoch()!=port_.identity.reference_epoch)) return false;
    port_.identity.boot=status.boot_epoch(); port_.identity.session=status.session_id();
    port_.identity.reference_epoch=status.reference_epoch();
    if(port_.identity.ticket!=state.freshness_ticket()) {
      port_.identity.ticket=state.freshness_ticket();
      if(port_.publishing) dummy_ros_control::set_command_identity(port_.identity);
    }
    enable_available_=!enabled_ && bootstrap_ && bootstrap_->phase()==BootstrapPhase::kIdle && transport_->ready();
    explicit_operation_available_=!enabled_ && bootstrap_ &&
      (bootstrap_->phase()==BootstrapPhase::kIdle || bootstrap_->operation_complete()) && transport_->ready();
    permit_operation_available_=explicit_operation_available_.load() ||
      (enabled_ && gate_->permit_refresh_allowed() && bootstrap_ &&
       (bootstrap_->ready() || bootstrap_->operation_complete()) && transport_->ready());
    single_enable_available_=!enabled_ && bootstrap_ && bootstrap_->operation_complete() &&
      bootstrap_->last_operation()==ManageOperation::kSetPermit && transport_->ready();
    if(enable_available_ && enable_requested_.exchange(false)) bootstrap_->start(latest_bootstrap_,now);
    if(enabled_) gate_->observe_state(observed->state.data(),observed->state.size(),now);
    return true;
  }
  void bootstrap_tick(std::uint64_t now) {
    if(!bootstrap_ || bootstrap_->phase()==BootstrapPhase::kIdle || bootstrap_->phase()==BootstrapPhase::kFault) return;
    bootstrap_->tick(now);
    if(enabled_ && bootstrap_->operation_complete() &&
       bootstrap_->last_operation()==ManageOperation::kSetPermit) {
      if(const auto authorization=bootstrap_->authorization())
        (void)gate_->renew_permit(*authorization);
    }
    if(!enabled_ && single_enable_requested_.exchange(false)) {
      const auto authorization=bootstrap_->authorization();
      if(!authorization || !transport_->allow_inactive_prime(true) || !gate_->enable(*authorization,now)) {
        transport_->allow_inactive_prime(false); ready_=false; return;
      }
      enabled_=true;
    }
    if(!enabled_ && bootstrap_->ready()) {
      const auto authorization=bootstrap_->authorization();
      if(!authorization || !transport_->allow_inactive_prime(true) || !gate_->enable(*authorization,now)) {
        transport_->allow_inactive_prime(false); ready_=false; return;
      }
      enabled_=true;
    }
    if(enabled_ && port_.direct_hold_started && !port_.direct_hold)
      transport_->allow_inactive_prime(false);
  }
  void check_execution(std::uint64_t now) {
    if(!port_.executing || !port_.controller) return;
    const auto sample=port_.controller->execution_sample(); if(!sample) return;
    const auto elapsed=(now>port_.started_us ? now-port_.started_us : 0)*1e-6;
    if(active_ && active_->is_active()) {
      auto feedback=std::make_shared<Follow::Feedback>();
      feedback->header.stamp=this->now(); feedback->joint_names=profile_.joints;
      const auto count=profile_.joints.size();
      feedback->actual.positions.assign(sample->actual.begin(),sample->actual.begin()+count);
      feedback->desired.positions.assign(sample->desired.begin(),sample->desired.begin()+count);
      for(std::size_t i=0;i<count;++i) feedback->error.positions.push_back(sample->desired[i]-sample->actual[i]);
      feedback->actual.time_from_start=rclcpp::Duration::from_seconds(elapsed);
      feedback->desired.time_from_start=feedback->actual.time_from_start;
      feedback->error.time_from_start=feedback->actual.time_from_start;
      active_->publish_feedback(feedback);
    }
    const auto &trajectory=port_.trajectory;
    bool terminal=true;
    for(std::size_t i=0;i<trajectory.joints.size();++i) {
      const double error=std::abs(sample->actual[i]-sample->desired[i]);
      if(!std::isfinite(error) || error>trajectory.path_tolerance[i]) {
        gate_->inner_result(port_.motion_generation,Terminal::kAborted,"PATH_TOLERANCE_VIOLATED",now); return;
      }
      terminal=terminal && std::abs(sample->actual[i]-trajectory.points.back().positions[i])<=trajectory.goal_tolerance[i] &&
          std::abs(sample->desired[i]-trajectory.points.back().positions[i])<=1e-9;
    }
    if(elapsed<trajectory.points.back().time) return;
    if(terminal) gate_->inner_result(port_.motion_generation,Terminal::kSucceeded,"",now);
    else if(elapsed>trajectory.points.back().time+trajectory.goal_time_tolerance+0.25)
      gate_->inner_result(port_.motion_generation,Terminal::kAborted,"GOAL_TOLERANCE_VIOLATED",now);
  }
  void cycle_impl() {
    const auto now=management_now_us();
    if(const auto update=dummy_ros_control::consume_control_update(last_update_)) {
      last_update_=update->generation;
      if(!gate_->record_update(port_.motion_generation,update->identity.motion_epoch,
          dummy_ros_control::wire_producer_sequence(update->generation),update->position)) {
        gate_->abort("CONTROL_UPDATE_REJECTED",now);
      }
    }
    port_.refresh_direct_hold();
    { std::unique_lock<std::mutex> lock(goal_mutex_,std::try_to_lock);
      if(lock && pending_) {
        active_=std::move(pending_); port_.uuid=active_->get_goal_id();
        const auto trajectory=validated_device_trajectory(*active_->get_goal(),profile_);
        if(!trajectory || !gate_->submit(*trajectory,now)) {
          auto result=std::make_shared<Follow::Result>(); result->error_code=Follow::Result::INVALID_GOAL;
          result->error_string="DEVICE_NOT_READY"; active_->abort(result); active_.reset(); busy_=false;
        }
      }
    }
    if(cancel_requested_.exchange(false)) gate_->cancel(now);
    if(port_.failed) { port_.failed=false; gate_->abort("GUARDED_REGISTRATION_FAILED",now); }
    check_execution(now);
    if(active_ && gate_->terminal()!=Terminal::kNone) {
      auto result=std::make_shared<Follow::Result>(); result->error_string=gate_->error();
      if(gate_->terminal()==Terminal::kSucceeded) active_->succeed(result);
      else if(gate_->terminal()==Terminal::kCanceled && active_->is_canceling()) active_->canceled(result);
      else { result->error_code=Follow::Result::PATH_TOLERANCE_VIOLATED; active_->abort(result); }
      active_.reset(); busy_=false;
    }
    ready_=enabled_ && gate_->ready();
  }
  DpManagementParameters parameters_;
  JointProfile profile_;
  DeviceProfile device_profile_;
  std::string controller_name_;
  GuardedPort port_;
  std::shared_ptr<Controller> discovered_controller_;
  std::unique_ptr<DpManagementTransport> transport_;
  std::unique_ptr<WireManagementClient> client_;
  std::unique_ptr<BootstrapCoordinator> bootstrap_;
  std::unique_ptr<ActionGate> gate_;
  rclcpp_action::Server<Follow>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bootstrap_status_service_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> management_services_;
  std::atomic<bool> ready_{false}, busy_{false}, cancel_requested_{false};
  std::atomic<bool> enable_requested_{false}, enable_available_{false};
  std::atomic<bool> single_enable_requested_{false}, single_enable_available_{false};
  std::atomic<bool> explicit_operation_available_{false};
  std::atomic<bool> permit_operation_available_{false};
  std::atomic<int> management_request_{0};
  std::uint64_t state_max_age_us_=250000U;
  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandle> pending_, active_;
  std::uint64_t last_update_=0;
  bool enabled_=false;
  BootstrapSnapshot latest_bootstrap_{};
};
}  // namespace dummy_motion_client

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto executor=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto gate=std::make_shared<dummy_motion_client::DeviceActionGate>();
    auto manager_options=controller_manager::get_cm_node_options();
    manager_options.arguments(std::vector<std::string>(argv+1,argv+argc));
    auto manager=std::make_shared<controller_manager::ControllerManager>(executor,"controller_manager","",manager_options);
    executor->add_node(manager); executor->add_node(gate);
    // Controller list discovery takes the manager's non-RT lock. Keep it off
    // the update thread: loading waits for that thread to adopt the new list.
    auto discovery=gate->create_wall_timer(std::chrono::milliseconds(20),[&]{gate->attach(*manager);});
    std::thread control([&] {
      const auto period=std::chrono::nanoseconds(1000000000ULL/std::max(1U,manager->get_update_rate()));
      auto next=std::chrono::steady_clock::now();
      while(rclcpp::ok()) {
        next+=period;
        // Refresh authority before producing a sample. Once the post-update
        // terminal check freezes that sample, a later ticket cannot retire it.
        gate->prepare_update();
        if(manager->is_resource_manager_initialized()) {
          const auto time=manager->now(); const rclcpp::Duration duration(period);
          manager->read(time,duration); manager->update(time,duration); manager->write(time,duration);
        }
        gate->cycle();
        std::this_thread::sleep_until(next);
      }
    });
    executor->spin(); rclcpp::shutdown(); control.join();
    discovery.reset();
    executor->remove_node(gate); gate.reset(); executor->remove_node(manager); manager.reset();
  } catch(const std::exception &error) {
    fprintf(stderr,"device_action_gate: %s\n",error.what()); rclcpp::shutdown(); return 1;
  }
  return 0;
}
