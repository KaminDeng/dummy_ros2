"""仅启动与手动面板同模型的 MoveIt；不接 USB、不启动轨迹执行。"""
import importlib.util
import math
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config_helpers():
    path = Path(get_package_prefix('dummy_bringup')) / 'lib/dummy_bringup/manual_panel_config.py'
    spec = importlib.util.spec_from_file_location('manual_panel_config', path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper


def _moveit_builder():
    from moveit_configs_utils import MoveItConfigsBuilder
    return MoveItConfigsBuilder


def build_nodes(context):
    bringup = Path(get_package_share_directory('dummy_bringup'))
    backend = LaunchConfiguration('backend').perform(context)
    helper = _config_helpers()
    path = helper.resolve_config_path(LaunchConfiguration('config').perform(context),
                                      backend, bringup / 'config/manual_joint_panel.json')
    cfg = helper.load_panel_config(path, backend)
    moveit = (_moveit_builder()('dummy-ros2', package_name='dummy_moveit_config')
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
                     # 与Native的20ms控制周期一致，避免100ms粗采样的插值人为抬高加速度。
                     'ompl.totg.resample_dt': 0.02,
                     # 默认0.1rad的圆滑偏差会切入贴近障碍物的已规划路径。
                     'ompl.totg.path_tolerance': 0.01,
                     # 常规路径快速搜索；验证发现漏采样碰撞后改用精细配置。
                     'ompl.dummy_arm.default_planner_config': 'RRTConnectFast',
                     'ompl.dummy_arm.planner_configs': ['RRTConnectFast','RRTConnectPrecise'],
                     'ompl.planner_configs.RRTConnectFast.type': 'geometric::RRTConnect',
                     'ompl.planner_configs.RRTConnectFast.range': 0.0,
                     'ompl.planner_configs.RRTConnectFast.longest_valid_segment_fraction': 0.01,
                     'ompl.planner_configs.RRTConnectPrecise.type': 'geometric::RRTConnect',
                     'ompl.planner_configs.RRTConnectPrecise.range': 0.0,
                     'ompl.planner_configs.RRTConnectPrecise.longest_valid_segment_fraction': 0.0005,
                     'allow_trajectory_execution': False,
                     'publish_robot_description_semantic': True,
                     'publish_planning_scene': True,
                     'publish_geometry_updates': True,
                     'publish_state_updates': True,
                     'publish_transforms_updates': True,
                 }])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value=''),
                              DeclareLaunchArgument('backend', default_value='real', choices=['real', 'simulator']),
                              OpaqueFunction(function=build_nodes)])
