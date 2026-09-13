"""Launch Gazebo Sim and the MoveIt planning stack without a duplicate FakeSystem."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "dummy-ros2", package_name="dummy_moveit_config"
    ).to_moveit_configs()
    share_dir = get_package_share_directory("dummy_moveit_config")
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, "launch", "gz_sim.launch.py"))
    )
    move_group_launch = generate_move_group_launch(moveit_config)
    rviz_launch = generate_moveit_rviz_launch(moveit_config)
    static_tf_launch = generate_static_virtual_joint_tfs_launch(moveit_config)

    return LaunchDescription(
        [
            simulation_launch,
            *static_tf_launch.entities,
            *move_group_launch.entities,
            *rviz_launch.entities,
        ]
    )
