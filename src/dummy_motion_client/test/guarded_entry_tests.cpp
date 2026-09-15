#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include <iostream>
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
    const bool topic_disabled=!controller.topic_open();
    auto empty=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    const bool empty_rejected=!controller.submit_registered_goal(registered,empty);
    if (!topic_disabled || !empty_rejected) return 4;
    std::cout << "{\"topic_subscriber_present\":" << (topic_disabled?"false":"true")
      << ",\"empty_trajectory_rejected\":" << (empty_rejected?"true":"false") << "}\n";
  } else if (mode=="MOV14") {
    auto trajectory=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    trajectory->joint_names={"Joint1"}; trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions={0}; point.time_from_start.sec=1; trajectory->points={point};
    const bool rejected=!controller.submit_registered_goal(wrong,trajectory);
    const bool internal_action_closed=!controller.action_open();
    if (!rejected || !internal_action_closed) return 5;
    std::cout << "{\"unregistered_goal_rejected\":" << (rejected?"true":"false")
      << ",\"internal_action_present\":" << (internal_action_closed?"false":"true") << "}\n";
  } else return 6;
  rclcpp::shutdown();
}
