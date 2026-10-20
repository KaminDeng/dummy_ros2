/**
 * @file guarded_execution_tests.cpp
 * @brief 用真实JTC更新与借用的内存接口核登记执行/prime/末代冻结。
 *
 * 运行会init/configure/activate ROS控制器，但关节状态和目标接口均为本进程double，循环position=command模拟理想跟随。
 * 不包含设备CAN/回执/真实轴动态，不能将终点0.35rad或JSON registered_jtc_execution读成物理执行证明。
 */
#include "dummy_ros_control/guarded_joint_trajectory_controller.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include <cmath>
#include <iostream>
/**
 * @brief 从actual接口0.25rad prime，推进JTC后执行登记的一秒目标。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 100Hz/10ms人工update；保持期先推进100拍，避免轨迹时钟停滞。注册UUID+语义摘要只许提交一次，110拍后command与最新generation均0.35rad。
 * 将sink设nullptr后不再生成更新，但末记录身份仍current供transport；clear后才deactivate/ROS shutdown。
 */
int main(int argc,char **argv) {
  using namespace dummy_ros_control;
  rclcpp::init(argc,argv);
  GuardedJointTrajectoryController controller;
  const auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("joints",std::vector<std::string>{"Joint1"}),
    rclcpp::Parameter("command_interfaces",std::vector<std::string>{"position"}),
    rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"})});
  const std::string urdf=R"(<robot name="probe"><link name="base"/><link name="tip"/><joint name="Joint1" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>)";
  if(controller.init("execution_probe",urdf,100,"/r06",options)!=controller_interface::return_type::OK) return 1;
  controller.get_node()->configure();
  double command=0.0,position=0.25,velocity=0.0;
  hardware_interface::CommandInterface command_interface("Joint1","position",&command);
  hardware_interface::StateInterface state_position("Joint1","position",&position),state_velocity("Joint1","velocity",&velocity);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(command_interface);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(state_position); states.emplace_back(state_velocity);
  controller.assign_interfaces(std::move(commands),std::move(states));
  controller.get_node()->activate();
  std::array<double,6> prime{}; prime[0]=0.25;
  set_command_identity({7,8,9,10,11});
  if(!controller.prime_position(prime)) return 2;
  controller.set_update_sink(controller_update_sink,nullptr);
  const auto origin=controller.get_node()->now();
  controller.update(origin,rclcpp::Duration::from_seconds(0.01));
  const auto sampled=consume_control_update(0);
  if(!sampled || sampled->position[0]!=0.25 || command!=0.25) return 3;
  // Priming/holding must advance the JTC clock while waiting for device acceptance.
  for(unsigned i=1;i<=100;++i)
    controller.update(origin+rclcpp::Duration::from_seconds(i*0.01),rclcpp::Duration::from_seconds(0.01));
  auto trajectory=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory->joint_names={"Joint1"}; trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions={0.35}; point.time_from_start.sec=1; trajectory->points={point};
  GoalCredential credential{{1},trajectory_digest(*trajectory)};
  if(!controller.register_goal(credential) || !controller.submit_registered_goal(credential,trajectory)) return 4;
  if(controller.submit_registered_goal(credential,trajectory)) return 5;
  for(unsigned i=1;i<=110;++i) {
    controller.update(origin+rclcpp::Duration::from_seconds((i+100)*0.01),rclcpp::Duration::from_seconds(0.01));
    position=command;
  }
  if(std::abs(command-0.35)>1e-9) { std::cerr<<"terminal command="<<command<<'\n'; return 6; }
  const auto final=consume_control_update(sampled->generation);
  if(!final || final->generation<=sampled->generation || std::abs(final->position[0]-0.35)>1e-9) return 7;
  // Freezing at update source keeps the terminal generation valid for the transport worker.
  controller.set_update_sink(nullptr,nullptr);
  controller.update(origin+rclcpp::Duration::from_seconds(2.2),rclcpp::Duration::from_seconds(0.01));
  if(consume_control_update(final->generation) || !command_record_current(*final)) return 8;
  clear_command_identity(); controller.get_node()->deactivate(); rclcpp::shutdown();
  std::cout<<"{\"prime_from_actual_update\":true,\"registered_jtc_execution\":true,\"terminal_frozen\":true}\n";
}
