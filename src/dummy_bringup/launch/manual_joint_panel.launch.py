##
# @file manual_joint_panel.launch.py
# @brief 为手动面板启动一致模型的测量显示、独立目标虚影和可选只规划服务。
# 真机控制仍由面板内唯一 Native worker 管理；launch 创建界面和显示节点不会自动标定、使能或授权电机。backend=simulator 使用软件模型，backend=real 要求固定部署配置；两个后端不互相继承设备身份。
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


##
# @brief 从已安装可执行目录加载部署配置辅助模块。
# 按 ament 安装前缀定位，不假定源码脚本路径或从最新构建目录搜索；导入失败交给 launch 报告。
# @return module：lib/dummy_bringup/manual_panel_config.py 对应的 Python 模块。
def _config_helpers():
    path = Path(get_package_prefix('dummy_bringup')) / 'lib/dummy_bringup/manual_panel_config.py'
    spec = importlib.util.spec_from_file_location('manual_panel_config', path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper


##
# @brief 在上下文求值后校验后端配置并构建面板与显示动作。
# 所有显示/规划端绑定配置中同一个严格解析的 xacro，避免安装副本落后。moveit 仅接受 true/false；开启而未安装配置包时给出具体错误，不静默退化。使用默认墙上时钟，没有在此启用 use_sim_time。
# @param context 包含 config/backend/moveit/report 参数的 launch 上下文。
# @return list：可选只规划入口、测量 RSP、目标 RSP/静态 TF、RViz 和面板节点。
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
             ##
             # 实际描述由面板测量关节话题驱动；独立 target_preview 命名空间与 frame_prefix 仅显示未执行目标，不覆盖实际 TF。
             parameters=[{'robot_description': urdf}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='target_preview', name='target_robot_state_publisher',
             parameters=[{'robot_description': urdf, 'frame_prefix': 'target_preview/'}],
             remappings=[('joint_states', '/target_joint_states')]),
        ##
        # 静态 TF 把 base_link 与 target_preview/base_link 的显示原点对齐；不改变设备参考、不发送运动。
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='target_preview_base',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'target_preview/base_link']),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(bringup / 'config/manual_joint_panel.rviz')]),
        ##
        # 面板进程接收同一配置和模型路径；报告记录、只读连接、授权和设备完成确认均在面板/Native 状态机中处理。
        Node(package='dummy_bringup', executable='manual_joint_panel.py',
             arguments=['--config', str(config_path), '--report', LaunchConfiguration('report'),
                        '--description-xacro', model_path, '--backend', LaunchConfiguration('backend')] + (['--moveit'] if moveit_arg == 'true' else []), output='screen'),
    ]


##
# @brief 声明配置、报告、后端和 MoveIt 开关。
# config 为空时 real 指向用户固定 current 部署，simulator 可用包内模板；report 默认 ~/.ros/dummy_reports 下按本地日期时间命名。moveit 默认 true，可 false 单用关节页；实际校验在 OpaqueFunction 内执行。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
def generate_launch_description():
    report = str(Path.home() / '.ros/dummy_reports/manual-panel-') + datetime.now().strftime('%Y%m%d-%H%M%S')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='', description='留空使用固定部署配置；模拟器可使用包内模板'),
        DeclareLaunchArgument('report', default_value=report),
        DeclareLaunchArgument('backend', default_value='real', choices=['real', 'simulator'], description='真机或纯软件模拟器'),
        DeclareLaunchArgument('moveit', default_value='true', description='启用末端规划页及仅规划MoveIt服务'),
        OpaqueFunction(function=build_nodes),
    ])
