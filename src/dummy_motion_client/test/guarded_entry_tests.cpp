#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include <iostream>
#include "rclcpp_action/rclcpp_action.hpp"
class Controller : public dummy_ros_control::GuardedJointTrajectoryController {
 public:
  bool topic_open() const { return bool(joint_command_subscriber_); }
  bool action_open() const { return bool(action_server_); }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); Controller controller;
  const auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("joints",std::vector<std::string>{"Joint1"}),
    rclcpp::Parameter("command_interfaces",std::vector<std::string>{"position"}),
    rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"})});
  if (controller.init("r06_guarded","",100,"/r06",options)!=controller_interface::return_type::OK) return 1;
  if (controller.on_configure(rclcpp_lifecycle::State())!=controller_interface::CallbackReturn::SUCCESS) return 2;
  const dummy_ros_control::GoalCredential registered{{1U},{2U}}, wrong{{9U},{2U}};
  if (!controller.register_goal(registered)) return 3;
  const std::string mode=argc>1 ? argv[1] : "MOV13";
  if (mode=="MOV13") {
    auto attacker=std::make_shared<rclcpp::Node>("topic_attacker");
    auto publisher=attacker->create_publisher<trajectory_msgs::msg::JointTrajectory>("/r06/r06_guarded/joint_trajectory",10);
    for(unsigned i=0;i<20;++i) { publisher->publish(trajectory_msgs::msg::JointTrajectory{}); rclcpp::spin_some(attacker); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
    if(publisher->get_subscription_count()!=0) return 8;
    const bool topic_disabled=!controller.topic_open();
    auto empty=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    const bool empty_rejected=!controller.submit_registered_goal(registered,empty);
    if (!topic_disabled || !empty_rejected) return 4;
    std::cout << "{\"topic_subscriber_present\":" << (topic_disabled?"false":"true")
      << ",\"empty_trajectory_rejected\":" << (empty_rejected?"true":"false") << "}\n";
  } else if (mode=="MOV14") {
    auto attacker=std::make_shared<rclcpp::Node>("action_attacker");
    auto client=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(attacker,"/r06/r06_guarded/follow_joint_trajectory");
    if(client->wait_for_action_server(std::chrono::milliseconds(200))) return 9;
    auto trajectory=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    trajectory->joint_names={"Joint1"}; trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions={0}; point.time_from_start.sec=1; trajectory->points={point};
    const bool rejected=!controller.submit_registered_goal(wrong,trajectory);
    const bool internal_action_closed=!controller.action_open();
    if (!rejected || !internal_action_closed) return 5;
    std::cout << "{\"unregistered_goal_rejected\":" << (rejected?"true":"false")
      << ",\"internal_action_present\":" << (internal_action_closed?"false":"true") << "}\n";
  } else if(mode=="MOV16") {
    auto trajectory=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    trajectory->joint_names={"Joint1"}; trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions={0}; point.time_from_start.sec=1; trajectory->points={point};
    if(controller.submit_registered_goal(registered,trajectory)) return 10;
  } else return 6;
  rclcpp::shutdown();
}
