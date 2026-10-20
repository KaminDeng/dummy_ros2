/**
 * @file servo_keyboard_input.cpp
 * @brief 沿MoveIt教程适配的交互终端Servo输入器；保留原BSD许可和示例退出行为。
 * @details 本程序只发布Twist/JointJog，不建立DPSDK会话、许可或CAN完成证据。当前CMake会构建该target；话题、控制器和Servo配置仍须由所选演示装配匹配。
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/*      Title     : servo_keyboard_input.cpp
 *      Project   : moveit2_tutorials
 *      Created   : 05/31/2021
 *      Author    : Adam Pettinger
 */

// adapt to Dummy by Muzhxiaowen, check out the details on bilibili.com
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Define used keys
/**
 * @brief 按字节识别按键：箭头仅匹配常见转义序列末字节，非完整ANSI序列解析；同值普通字母也可能命中。
 */
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

// Some constants used in the Servo Teleop demo
/**
 * @brief 固定Servo话题和坐标系：Twist可选base_link/法兰，JointJog固定标base_link。
 * @details 队列深度10只是ROS发布缓存，不是可执行运动段预算，也不保证对端接纳。
 */
const std::string TWIST_TOPIC = "/servo_demo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_demo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "link6_1_1";
const std::string BASE_FRAME_ID = "base_link";

// A class for reading the key inputs from the terminal
/**
 * @brief stdin终端模式的示例封装；调用者显式shutdown恢复，不提供自动析构恢复。
 * @details 构造阶段不检查tcgetattr/tcsetattr结果，使用场景是交互TTY，不应按健壮的通用字节流读取器理解。
 */
class KeyboardReader
{
public:
  /**
   * @brief 保存stdin原termios，关闭规范行输入和回显，设置VEOL/VEOF后应用。
   * @details 全局input对象在main之前构造；这些操作本身不初始化ROS或触达机械臂。
   */
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  /**
   * @brief 尝试从stdin读一个字节，仅负返回抛异常。
   * @param c 可写目标字节；调用方保证有效。
   * @details EOF返回0时本方法不另报错、不保证写c，keyLoop仍沿原示例读取该变量，不能把EOF当成已确认的退出键。
   */
  void readOne(char* c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  /**
   * @brief 恢复构造时保存的终端设置；不停止Servo、释放DPSDK或核验机械静止。
   */
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  /**
   * @brief stdin文件描述符及原终端设置，仅本读取器拥有恢复所需副本。
   */
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
/**
 * @brief 把单字节键映射为两种Servo消息；键盘处理与ROS spin为两个执行路径。
 */
class KeyboardServo
{
public:
  /**
   * @brief 创建节点及两个publisher，默认Twist在base_link，关节点动方向为+1。
   */
  KeyboardServo();
  /**
   * @brief 主线程读取并发布键盘命令；Q返回0、read负返回−1。
   * @details 启动的spin线程被detach，没有join收尾；本例不能充当正式有界停机/对象寿命控制的模板。
   */
  int keyLoop();

private:
  /**
   * @brief 持续spin_some直到rclcpp关闭，当前循环没有额外sleep；不是设备控制周期线程。
   */
  void spin();

  /**
   * @brief 节点及publisher由KeyboardServo保存；发布与spin共享该节点，析构前没有显式等待detach线程。
   */
  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  /**
   * @brief 当前Twist参考frame与JointJog的±1方向值；R只翻转关节方向，不翻转笛卡尔方向。
   */
  std::string frame_to_publish_;
  double joint_vel_cmd_;
};

/**
 * @brief 构造servo_keyboard_input节点，创建两个固定话题publisher。
 * @details 默认配置command_in_type为unitless时，后端还会使用scale缩放消息值；此处的1不能直接当作已批准实机速度。
 */
KeyboardServo::KeyboardServo() : frame_to_publish_(BASE_FRAME_ID), joint_vel_cmd_(1)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
}

/**
 * @brief 全局终端读取器先于main构造；恢复只由下面的显式调用完成。
 */
KeyboardReader input;

/**
 * @brief SIGINT原示例路径：恢复终端、请求ROS关闭并exit；没有DPSDK失能回执或机械停止检查。
 * @param sig 信号号未参与分支。
 * @details 此处调用复杂库函数，注释不将它提升为已证明异步信号安全的正式关闭协议。
 */
void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

/**
 * @brief 初始化ROS/发布器并注册SIGINT，随后阻塞于键盘循环；常规返回路径再恢复终端和关闭ROS。
 * @return keyLoop返回码；不能据进程exit0推断设备失能或最后命令已执行。
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

/**
 * @brief 处理节点回调直到rclcpp::ok为false；没有抢占保证，也没有控制周期计时。
 */
void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

/**
 * @brief 每次读一字节，构造新消息、按键填字段，最后只发布被选中的一种消息。
 * @details 箭头为X/Y，点号/分号为Z；E/W切Twist参考，数字键选JointJog，R反向，Q退出。没有在此做碰撞、关节范围、设备许可或ACK校验。
 */
int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  /**
   * @brief 原示例启动独立spin并detach；Q退出时不等待该线程，避免把当前收尾解释成可证明无并发访问。
   */
  std::thread{ std::bind(&KeyboardServo::spin, this) }.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
  puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
  puts("'Q' to quit.");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Use read key-press
    switch (c)
    {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg->twist.linear.y = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg->twist.linear.y = 1.0;
        publish_twist = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg->twist.linear.x = 1.0;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg->twist.linear.x = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_PERIOD:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg->twist.linear.z = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_SEMICOLON:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg->twist.linear.z = 1.0;
        publish_twist = true;
        break;
      /**
       * @brief E切法兰坐标，W实际切base_link；虽然提示文字写world，这里的frame常量并非world。
       */
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "E");
        frame_to_publish_ = EEF_FRAME_ID;
        break;
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "W");
        frame_to_publish_ = BASE_FRAME_ID;
        break;
      /**
       * @brief 数字1到6仅填对应Joint名和±1速度值；是否按物理速度或无量纲缩放由接收Servo配置决定。
       */
      case KEYCODE_1:
        RCLCPP_DEBUG(nh_->get_logger(), "1");
        joint_msg->joint_names.push_back("Joint1");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_2:
        RCLCPP_DEBUG(nh_->get_logger(), "2");
        joint_msg->joint_names.push_back("Joint2");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_3:
        RCLCPP_DEBUG(nh_->get_logger(), "3");
        joint_msg->joint_names.push_back("Joint3");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_4:
        RCLCPP_DEBUG(nh_->get_logger(), "4");
        joint_msg->joint_names.push_back("Joint4");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_5:
        RCLCPP_DEBUG(nh_->get_logger(), "5");
        joint_msg->joint_names.push_back("Joint5");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_6:
        RCLCPP_DEBUG(nh_->get_logger(), "6");
        joint_msg->joint_names.push_back("Joint6");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      /**
       * @brief 历史第7轴分支仍会发布Joint7，即便日志说Nothing；当前六轴模型无该关节，本程序没有在此过滤。
       */
      case KEYCODE_7:
        RCLCPP_INFO(nh_->get_logger(), "Nothing on 7 key!!!");
        joint_msg->joint_names.push_back("Joint7");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(nh_->get_logger(), "R");
        joint_vel_cmd_ *= -1;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        return 0;
    }

    // If a key requiring a publish was pressed, publish the message now
    /**
     * @brief 发布前填节点时钟戳及frame，移动unique_ptr转交消息所有权；本地publish返回不证明Servo/CAN已经接受。
     */
    if (publish_twist)
    {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = frame_to_publish_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
    else if (publish_joint)
    {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }

  return 0;
}
