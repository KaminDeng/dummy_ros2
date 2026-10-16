"""仅启动与手动面板同模型的 MoveIt；不接 USB、不启动轨迹执行。"""
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_nodes(context):
    cfg = json.loads(Path(LaunchConfiguration('config').perform(context)).read_text())
    moveit = (MoveItConfigsBuilder('dummy-ros2', package_name='dummy_moveit_config')
              .robot_description(file_path=cfg['description_xacro'])
              .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    # 不沿用旧模型速度/加速度或旧 J5 限位。规划工作域与实机面板一致。
    limits = moveit.joint_limits['robot_description_planning']['joint_limits']
    for joint in cfg['joints']:
        limits[joint['name']] = {
            'has_position_limits': True,
            'min_position': math.radians(joint['limits_deg'][0]),
            'max_position': math.radians(joint['limits_deg'][1]),
            'has_velocity_limits': True,
            'max_velocity': math.radians(cfg['motion_speed_max_deg_s']),
            'has_acceleration_limits': True,
            'max_acceleration': cfg['motion_acceleration_max_rad_s2'],
        }
    return [Node(package='moveit_ros_move_group', executable='move_group',
                 name='manual_move_group', output='screen',
                 parameters=[moveit.to_dict(), {
                     'allow_trajectory_execution': False,
                     'publish_robot_description_semantic': True,
                     'publish_planning_scene': True,
                     'publish_geometry_updates': True,
                     'publish_state_updates': True,
                     'publish_transforms_updates': True,
                 }])]


def generate_launch_description():
    default = str(Path(get_package_share_directory('dummy_bringup')) / 'config/manual_joint_panel.json')
    return LaunchDescription([DeclareLaunchArgument('config', default_value=default),
                              OpaqueFunction(function=build_nodes)])
