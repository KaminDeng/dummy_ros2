"""六轴实机交互面板与RViz；单USB owner；目标虚影使用独立topic/TF，不驱动实物。"""
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import json


def build_nodes(context):
    bringup = Path(get_package_share_directory('dummy_bringup'))
    config_path = LaunchConfiguration('config').perform(context)
    config = json.loads(Path(config_path).read_text())
    # 安装目录的ament_python data副本可能落后；三个显示端使用同一明确源文件。
    model_path = str(Path(config['description_xacro']).resolve(strict=True))
    urdf = xacro.process_file(model_path).toxml()
    moveit_arg = LaunchConfiguration('moveit').perform(context).lower()
    if moveit_arg not in ('true', 'false'):
        raise ValueError('moveit 参数只接受 true / false')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='target_preview', name='target_robot_state_publisher',
             parameters=[{'robot_description': urdf, 'frame_prefix': 'target_preview/'}],
             remappings=[('joint_states', '/target_joint_states')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='target_preview_base',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'target_preview/base_link']),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(bringup / 'config/manual_joint_panel.rviz')]),
        Node(package='dummy_bringup', executable='manual_joint_panel.py',
             arguments=['--config', config_path, '--report', LaunchConfiguration('report'),
                        '--description-xacro', model_path] + (['--moveit'] if moveit_arg == 'true' else []), output='screen'),
    ]


def generate_launch_description():
    bringup = Path(get_package_share_directory('dummy_bringup'))
    report = '<home>/kamin_work/00-code/dummy_ros2/build_dummy/reports/manual-panel-' + datetime.now().strftime('%Y%m%d-%H%M%S')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(bringup / 'config/manual_joint_panel.json')),
        DeclareLaunchArgument('report', default_value=report),
        DeclareLaunchArgument('moveit', default_value='true', description='启用末端规划页及仅规划MoveIt服务'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/manual_moveit_planning.launch.py')),
            condition=IfCondition(LaunchConfiguration('moveit')),
            launch_arguments={'config': LaunchConfiguration('config')}.items()),
        OpaqueFunction(function=build_nodes),
    ])
