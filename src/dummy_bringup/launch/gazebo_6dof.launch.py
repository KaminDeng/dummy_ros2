##
# @file gazebo_6dof.launch.py
# @brief 装配标准六轴 Gazebo、MoveIt 场景和公开 action 门控。
# 此后端使用 Gazebo 原生仿真，不创建真实设备传输。gui 与 rviz 默认关闭；场景和门控使用 use_sim_time=True，与 Gazebo /clock 对齐，仿真暂停时 ROS 仿真时间也会停止。
"""Canonical six-axis Gazebo entrypoint with no device transport."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


##
# @brief 包含 Gazebo/MoveIt 入口并添加公共场景与动作门控。
# gui/rviz 参数转给 dummy_moveit_config/gz_moveit.launch.py。scene_publisher 从本包 config/scene.yaml 向 MoveIt 应用场景差量；native_action_gate 提供公开轨迹门控，创建节点不等于目标已被接受或轨迹已完成。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
def generate_launch_description():
    moveit_share = get_package_share_directory("dummy_moveit_config")
    bringup_share = get_package_share_directory("dummy_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, "launch", "gz_moveit.launch.py")
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
            Node(
                package="dummy_bringup",
                ##
                # 场景发布器负责 MoveIt 几何；Gazebo 世界几何由被包含入口配置，两者须共享公共场景定义。
                executable="scene_publisher.py",
                parameters=[
                    {
                        "scene_file": os.path.join(bringup_share, "config", "scene.yaml"),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="dummy_motion_client",
                executable="native_action_gate",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
