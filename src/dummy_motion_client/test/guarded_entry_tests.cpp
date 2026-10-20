/**
 * @file guarded_entry_tests.cpp
 * @brief 通过实际ROS图核Guarded控制器关闭旁路入口。
 *
 * 运行会init ROS并创建攻击者topic/Action客户端，但不连接硬件或执行有效运动；测试命名空间带PID避免并行串扰。MOV13/14/16各检查不同入口，不能把其中一个模式算全覆盖。
 */
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include <iostream>
#include <unistd.h>
#include "rclcpp_action/rclcpp_action.hpp"
/**
 * @brief 测试派生类只暴露父类topic订阅器与内部Action句柄是否存在，不直接绕开生产提交校验。
 */
class Controller : public dummy_ros_control::GuardedJointTrajectoryController {
 public:
  /**
   * @brief 查询内部joint_trajectory订阅器存在性。
   * @return bool句柄，不表示当前ROS图外部topic不存在。
   */
  bool topic_open() const { return bool(joint_command_subscriber_); }
  /**
   * @brief 查询内部Action server句柄存在性。
   * @return true仅表示内部旁路已创建；公开Gate Action是另一路径。
   */
  bool action_open() const { return bool(action_server_); }
};
/**
 * @brief 初始化J1控制器并按MOV模式验证旁路关闭。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * argv[1]默认MOV13：攻击空topic20次且订阅数0，注册空轨迹仍拒。MOV14：内部Action发现200ms不可用，错误credential提交拒绝；MOV16：仅configure未active时即使credential正确也不得执行。
 * base configure可能短暂发布内部server，故PID命名空间隔离；不声称测试了任意ROS攻击模型。
 */
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); Controller controller;
  const std::string mode=argc>1 ? argv[1] : "MOV13";
  // 并行 fixture 的 base on_configure 会短暂发布 Action server，隔离各测试命名空间。
  const std::string test_namespace="/r06_"+mode+"_"+std::to_string(getpid());
  const std::string controller_path=test_namespace+"/r06_guarded";
  const auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("joints",std::vector<std::string>{"Joint1"}),
    rclcpp::Parameter("command_interfaces",std::vector<std::string>{"position"}),
    rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"})});
  if (controller.init("r06_guarded","",100,test_namespace,options)!=controller_interface::return_type::OK) return 1;
  if (controller.on_configure(rclcpp_lifecycle::State())!=controller_interface::CallbackReturn::SUCCESS) return 2;
  const dummy_ros_control::GoalCredential registered{{1U},{2U}}, wrong{{9U},{2U}};
  if (!controller.register_goal(registered)) return 3;
  if (mode=="MOV13") {
    auto attacker=std::make_shared<rclcpp::Node>("topic_attacker");
    auto publisher=attacker->create_publisher<trajectory_msgs::msg::JointTrajectory>(controller_path+"/joint_trajectory",10);
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
    auto client=rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(attacker,controller_path+"/follow_joint_trajectory");
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
