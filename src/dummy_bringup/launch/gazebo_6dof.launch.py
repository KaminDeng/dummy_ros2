"""Canonical six-axis Gazebo entrypoint with no device transport."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
