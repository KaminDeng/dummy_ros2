##
# @file display.launch.py
# @brief 纯显示入口：共享几何、RSP、可选滑块关节发布器和RViz，不连接DPSDK。

# Copyright 2026 Dummy ROS maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


##
# @brief 读取共享Xacro及显示配置，声明gui选项并装配显示节点。
# @return LaunchDescription；gui默认True，仅决定用滑块GUI还是普通joint_state_publisher。
def generate_launch_description():
    ##
    # 从ament索引定位描述包，不依赖开发者下载目录或报告目录。
    share_dir = get_package_share_directory('dummy-ros2_description')

    ##
    # 展开当前完整CAD模型；显示关节由下方发布器提供，不是来自机械臂实际编码器。
    xacro_file = os.path.join(share_dir, 'urdf', 'dummy-ros2.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    ##
    # LaunchConfiguration到运行时才求值，If/UnlessCondition使两个关节发布器二选一。
    show_gui = LaunchConfiguration('gui')

    ##
    # RSP用模型和joint_states生成连杆TF，自身不负责关节目标或实机反馈。
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    ##
    # gui关闭时的普通关节状态发布器，适合静态/软件显示；不能与真实关节状态源混用。
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    ##
    # gui开启时使用滑块改变显示姿态；这些滑块不会调用设备运动接口。
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    ##
    # 加载描述包display.rviz，仅显示模型/TF；RViz进程启动不意味着所有输入话题已就绪。
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
