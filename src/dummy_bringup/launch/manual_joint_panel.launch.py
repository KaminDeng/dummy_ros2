"""六轴实机交互面板与RViz；单USB owner；目标虚影使用独立topic/TF，不驱动实物。"""
from datetime import datetime
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _config_helpers():
    path = Path(get_package_prefix('dummy_bringup')) / 'lib/dummy_bringup/manual_panel_config.py'
    spec = importlib.util.spec_from_file_location('manual_panel_config', path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper


def build_nodes(context):
    bringup = Path(get_package_share_directory('dummy_bringup'))
    backend = LaunchConfiguration('backend').perform(context)
    helper = _config_helpers()
    config_path = helper.resolve_config_path(LaunchConfiguration('config').perform(context),
                                             backend, bringup / 'config/manual_joint_panel.json')
    config = helper.load_panel_config(config_path, backend)
    # 安装目录的ament_python data副本可能落后；三个显示端使用同一明确源文件。
    model_path = str(Path(config['description_xacro']).resolve(strict=True))
    urdf = xacro.process_file(model_path).toxml()
    moveit_arg = LaunchConfiguration('moveit').perform(context).lower()
    if moveit_arg not in ('true', 'false'):
        raise ValueError('moveit 参数只接受 true / false')
    actions = []
    if moveit_arg == 'true':
        try:
            get_package_share_directory('dummy_moveit_config')
        except PackageNotFoundError as exc:
            raise RuntimeError('未安装 dummy_moveit_config；请在 ROS 工作区执行 '
                               'bash scripts/build_manual_panel.sh 并重新 source 安装区；'
                               '仅使用关节面板可传 moveit:=false') from exc
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/manual_moveit_planning.launch.py')),
            launch_arguments={'config': str(config_path), 'backend': backend}.items()))
    return actions + [
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
             arguments=['--config', str(config_path), '--report', LaunchConfiguration('report'),
                        '--description-xacro', model_path, '--backend', LaunchConfiguration('backend')] + (['--moveit'] if moveit_arg == 'true' else []), output='screen'),
    ]


def generate_launch_description():
    report = str(Path.home() / '.ros/dummy_reports/manual-panel-') + datetime.now().strftime('%Y%m%d-%H%M%S')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='', description='留空使用固定部署配置；模拟器可使用包内模板'),
        DeclareLaunchArgument('report', default_value=report),
        DeclareLaunchArgument('backend', default_value='real', choices=['real', 'simulator'], description='真机或纯软件模拟器'),
        DeclareLaunchArgument('moveit', default_value='true', description='启用末端规划页及仅规划MoveIt服务'),
        OpaqueFunction(function=build_nodes),
    ])
