/** @file fixed_step_jtc_tests.cpp
 * @brief 用真实JTC与内存关节接口重放GUI11固定生产时基导数故障，不连接设备。
 */
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using Controller = dummy_ros_control::GuardedJointTrajectoryController;
struct Probe {
  Controller controller;
  double actual=2.9097823829665854, command=actual;
  hardware_interface::StateInterface state{"Joint3","position",&actual};
  hardware_interface::CommandInterface output{"Joint3","position",&command};
  std::int64_t now=1000000000LL;
  unsigned sinks=0;
  Probe() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("joints",std::vector<std::string>{"Joint3"}),
      rclcpp::Parameter("command_interfaces",std::vector<std::string>{"position"}),
      rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position"})});
    assert(controller.init("fixed_step_test",R"(<robot name="test"><link name="base"/><link name="tip"/><joint name="Joint3" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="0" upper="3.2" effort="1" velocity="1.0995574287564276"/></joint></robot>)",50,"",options)==controller_interface::return_type::OK);
    controller.configure();
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(output);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(state);
    controller.assign_interfaces(std::move(commands),std::move(states));
    controller.get_node()->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    controller.set_update_sink([](const double*,std::size_t,std::int64_t,void* context){
      ++static_cast<Probe*>(context)->sinks;
    },this);
  }
  ~Probe(){controller.set_update_sink(nullptr,nullptr);controller.release_interfaces();}
  void submit() {
    auto trajectory=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    trajectory->joint_names={"Joint3"};
    const double values[]={2.9097823829665854,2.9097823829665854,
                           2.9089839805697815,2.9065887733793696,2.902596761395349};
    for(unsigned i=0;i<5;++i){
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions={values[i]};point.time_from_start.nanosec=(i+1)*20000000U;
      trajectory->points.push_back(point);
    }
    dummy_ros_control::GoalCredential credential{};
    credential.uuid[0]=1U;credential.digest=dummy_ros_control::trajectory_digest(*trajectory);
    assert(controller.register_goal(credential));
    assert(controller.submit_registered_goal(credential,trajectory));
  }
  double tick(std::int64_t elapsed) {
    now+=elapsed;
    assert(controller.update(rclcpp::Time(now,RCL_ROS_TIME),rclcpp::Duration::from_nanoseconds(elapsed))==controller_interface::return_type::OK);
    return controller.execution_sample()->desired[0];
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  {
    Probe probe;probe.submit();
    probe.tick(20000000);probe.tick(20000000);
    const double q0=probe.tick(20000000);
    const double q1=probe.tick(19582279);
    const double q2=probe.tick(20384057);
    const double acceleration=std::abs(q2-2*q1+q0)/(.02*.02);
    std::cout.precision(17);
    std::cout<<"JTC q="<<q0<<","<<q1<<","<<q2<<" acceleration="<<acceleration<<std::endl;
    assert(acceleration<=4.0);
  }
  {
    Probe probe;
    std::array<double,6> hold{};hold[0]=probe.actual;
    assert(probe.controller.prime_position(hold));
    assert(probe.tick(20000000)==probe.actual);
    assert(probe.tick(19500000)==probe.actual);
    probe.submit();
    assert(probe.tick(20000000)==probe.actual);
    probe.tick(20000000);probe.tick(20000000);
    assert(probe.tick(20000000)<probe.actual);
    probe.controller.cancel_execution();
    assert(probe.tick(20000000)==probe.actual);
    probe.submit();
    assert(probe.tick(20000000)==probe.actual);
    assert(probe.tick(20000000)==probe.actual);
    assert(probe.tick(20000000)==probe.actual);
    assert(probe.tick(20000000)<probe.actual);
  }
  for (unsigned failure=0;failure<4;++failure) {
    Probe probe;probe.submit();probe.tick(20000000);
    const auto before=probe.sinks;
    const auto sample_before=probe.controller.execution_sample()->sequence;
    dummy_ros_control::set_command_identity({1U,2U,3U,4U,5U});
    const std::int64_t elapsed=failure==0 ? -1 : failure==1 ? 0 : failure==2 ? 40000000 : 20000000;
    probe.now += failure==3 ? -1 : elapsed;
    assert(probe.controller.update(rclcpp::Time(probe.now,RCL_ROS_TIME),
      rclcpp::Duration::from_nanoseconds(elapsed))==controller_interface::return_type::ERROR);
    assert(probe.sinks==before && probe.controller.execution_sample()->sequence==sample_before);
    const double position[1]{probe.actual};
    assert(dummy_ros_control::publish_control_update(position,1,0,nullptr)==0U);
    probe.now+=20000000;
    assert(probe.controller.update(rclcpp::Time(probe.now,RCL_ROS_TIME),
      rclcpp::Duration::from_nanoseconds(20000000))==controller_interface::return_type::ERROR);
    assert(probe.sinks==before);
  }
  rclcpp::shutdown();
}
