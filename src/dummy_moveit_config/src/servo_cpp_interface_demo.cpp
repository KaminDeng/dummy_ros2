/**
 * @file servo_cpp_interface_demo.cpp
 * @brief 历史MoveIt Servo C++接口示例，保留原教程与BSD许可；当前本包CMake未为此源建target。
 * @details 它沿旧ServoParameters/Servo API编写，不能据源码存在推断已兼容当前安装版本。当前50ms定时器行被注释，本程序未自行注册publishCommands来自动点动。
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_cpp_interface_demo.cpp
 *      Project   : moveit2_tutorials
 *      Created   : 07/13/2020
 *      Author    : Adam Pettinger
 */
// adapt to Dummy by Muzhxiaowen, check out the details on bilibili.com

// ROS
#include <rclcpp/rclcpp.hpp>

// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

/**
 * @brief 示例日志分类，不是机器人设备标识或控制会话。
 */
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_tutorials.servo_demo_node.cpp");

// BEGIN_TUTORIAL

// Setup
// ^^^^^
// First we declare pointers to the node and publisher that will publish commands to Servo
/**
 * @brief 回调共享的节点、两个publisher与阶段计数；当前仅为教程示例全局状态，不提供多producer同步。
 */
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
size_t count_ = 0;

// BEGIN_SUB_TUTORIAL publishCommands
// Here is the timer callback for publishing commands. The C++ interface sends commands through internal ROS topics,
// just like if Servo was launched using ServoNode.
/**
 * @brief 若被调用，先发布100次Joint1点动，再持续发布base_link中的Twist。
 * @details 前段速度字段0.3，后段平移X=0.3/旋转Z=0.5；在unitless模式下还需Servo缩放。当前没有启用timer来调用本函数，因此不能把函数定义当作已执行这段动作。
 */
void publishCommands()
{
  // First we will publish 100 joint jogging commands. The :code:`joint_names` field allows you to specify individual
  // joints to move, at the velocity in the corresponding :code:`velocities` field. It is important that the message
  // contains a recent timestamp, or Servo will think the command is stale and will not move the robot.
  /**
   * @brief 初段每次成功走发布调用后自增计数；计数只表示回调路径，不表示电机完成100次运动。
   */
  if (count_ < 100)
  {
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp = node_->now();
    msg->joint_names.push_back("Joint1");
    msg->velocities.push_back(0.3);
    joint_cmd_pub_->publish(std::move(msg));
    ++count_;
  }

  // After a while, we switch to publishing twist commands. The provided frame is the frame in which the twist is
  // defined, not the robot frame that will follow the command. Again, we need a recent timestamp in the message
  /**
   * @brief 达100后不再增加count_，后续每次调用均走Twist分支；frame表示速度表达坐标，不是自动跟随的实体frame。
   */
  else
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.x = 0.3;
    msg->twist.angular.z = 0.5;
    twist_cmd_pub_->publish(std::move(msg));
  }
}
// END_SUB_TUTORIAL

// Next we will set up the node, planning_scene_monitor, and collision object
/**
 * @brief 建节点/场景监视器/publisher/Servo并spin，任何缺场景或参数失败直接返回。
 * @details 4秒暂停只是固定等待，不是RViz/TF就绪确认；启动Servo不等于已经建立DPSDK实机授权。
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  /**
   * @brief 原示例显式关闭intra-process通讯；这里不据此验证当前中间件QoS或延迟。
   */
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
  rclcpp::sleep_for(std::chrono::seconds(4));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
  // before initializing any collision objects
  /**
   * @brief 场景监视器借用节点时钟/TF并加载robot_description，必须成功获得PlanningScene才继续。
   */
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");

  // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
  if (planning_scene_monitor->getPlanningScene())
  {
    /**
     * @brief 订阅/joint_states并以25Hz发布场景、监听场景变化及提供查询服务；话题存在不等同设备反馈具备可信身份/新鲜度。
     */
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }

  // These are the publishers that will send commands to MoveIt Servo. Two command types are supported: JointJog
  // messages which will directly jog the robot in the joint space, and TwistStamped messages which will move the
  // specified link with the commanded Cartesian velocity. In this demo, we jog the end effector link.
  /**
   * @brief 示例指令走ROS内部话题，键盘节点也可向同一话题发消息；本处不核设备session/permit。
   */
  joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("servo_demo_node/delta_joint_cmds", 10);
  twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("servo_demo_node/delta_twist_cmds", 10);

  // Next we will create a collision object in the way of the arm. As the arm is servoed towards it, it will slow down
  // and stop before colliding
  /**
   * @brief 以link6_1_1表达一个world碰撞对象；不是attached object，frame坐标在处理时用于转换。
   * @details 盒体尺寸/位置单位米；发布场景diff只表达添加请求，不证明远端场景已接收或真实表面间隙。
   */
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "link6_1_1";
  collision_object.id = "box";

  // Make a box and put it in the way
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.1 };
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.2;
  box_pose.position.y = 0;
  box_pose.position.z = 0.0025;

  // Add the box as a collision object
  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Create the message to publish the collision object
  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object);
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw;

  // Publish the collision object to the planning scene
  /**
   * @brief 将一次PlanningScene diff发布到planning_scene；没有等待服务ACK或观察碰撞减速结果。
   */
  auto scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  scene_pub->publish(ps);

  // Initializing Servo
  // ^^^^^^^^^^^^^^^^^^
  // Servo requires a number of parameters to dictate its behavior. These can be read automatically by using the
  // :code:`makeServoParameters` helper function
  /**
   * @brief 旧API从节点参数构造Servo配置；失败返回，不能用缺参默认值继续假称就绪。
   */
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
  /**
   * @brief Servo借用同一节点、参数与场景监视器；对象在executor退出后按局部RAII释放。
   */
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

  // You can start Servo directly using the C++ interface. If launched using the alternative ServoNode, a ROS
  // service is used to start Servo. Before it is started, MoveIt Servo will not accept any commands or move the robot
  /**
   * @brief 启动旧Servo处理入口，仍须有有效输入、模型及后端控制器；不是设备EnableHold事务。
   */
  servo->start();

  // Sending Commands
  // ^^^^^^^^^^^^^^^^
  // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot
  /**
   * @brief 原50ms回调注册当前被注释；本文件自身没有其他地方调用publishCommands。
   */
  // rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms, publishCommands);

  // CALL_SUB_TUTORIAL publishCommands

  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
  /**
   * @brief 多线程executor处理并发ROS任务直至关闭；这里没有把退出映射成DPSDK失能/释放确认。
   */
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  // END_TUTORIAL

  rclcpp::shutdown();
  return 0;
}
