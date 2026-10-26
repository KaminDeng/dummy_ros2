##
# @file manual_joint_panel.launch.py
# @brief 为手动面板启动一致模型的测量显示、独立目标虚影和可选只规划服务。
# real由Native拥有通信，ros由GUI管理唯一C++控制链；普通启动默认只读。simulator使用软件模型，三个后端不互相继承设备授权。
"""六轴实机交互面板与RViz；单USB owner；目标虚影使用独立topic/TF，不驱动实物。"""
from datetime import datetime
import importlib.util
import hashlib
import json
import tempfile
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
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
    planning_config_path = config_path
    sdk_arg = context.launch_configurations.get('sdk_prefix', '')
    # 公共显示/规划由父launch持有；配置从任一初始目标都可切换ROS。
    if backend == 'ros' or sdk_arg:
        sdk = str(Path(sdk_arg).resolve(strict=True))
        deployment_config = (config_path if backend != 'simulator' else
                             helper.resolve_config_path('', 'ros', bringup / 'config/manual_joint_panel.json'))
        root = Path(deployment_config).resolve(strict=True).parent
        pin = hashlib.sha256((root / 'deployment.json').read_bytes()).hexdigest()
        launch_module_path = bringup / 'launch/manual_ros_control.launch.py'
        spec = importlib.util.spec_from_file_location('manual_ros_control_launch', launch_module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _, cfg, _, _, _ = module.read_bound_deployment(root, pin, sdk)
        if backend == 'simulator':
            # 模拟显示仍使用调用方选定的模型；连接ROS时再用固定部署的身份。
            simulated = helper.load_panel_config(config_path, 'simulator')
            cfg.update(description_xacro=simulated['description_xacro'])
        generated = Path(tempfile.mkdtemp(prefix='dummy-manual-panel-ros-'))
        controllers = generated / 'controllers.yaml'
        controllers.write_text(yaml.safe_dump(module.controller_parameters()))
        report = str(Path(LaunchConfiguration('report').perform(context)).resolve())
        Path(report).parent.mkdir(parents=True, exist_ok=True)
        cfg.update(sdk_prefix=sdk, ros_controller_config=str(controllers),
                   deployment_source_config=str(Path(deployment_config).resolve(strict=True)),
                   operator_confirmed_double_limits=LaunchConfiguration('operator_confirmed_double_limits', default='false').perform(context) == 'true',
                   auto_calibrate_and_test=LaunchConfiguration('auto_calibrate_and_test', default='false').perform(context) == 'true',
                   ros_launch_command=['ros2', 'launch', 'dummy_bringup', 'manual_ros_control.launch.py',
                     'deployment_root:='+str(root), 'deployment_sha256:='+pin, 'sdk_prefix:='+sdk,
                     'auto_calibrate_and_test:=false', 'initial_pose:=manual_limits', 'reuse_display_planning:=true',
                     'operator_confirmed_double_limits:='+LaunchConfiguration('operator_confirmed_double_limits', default='false').perform(context),
                     'report_path:='+report+'-baseline.log', 'raw_evidence_path:='+report+'-raw.jsonl',
                     'goal_timeline_path:='+report+'-timeline.jsonl',
                     'cylinder_pose:='+LaunchConfiguration('cylinder_pose').perform(context),
                     'cylinder_dimensions:='+LaunchConfiguration('cylinder_dimensions').perform(context)])
        config_path = generated / 'panel-config.json'
        config_path.write_text(json.dumps(cfg))
    if backend == 'real' and config_path != planning_config_path:
        config = helper.load_panel_config(planning_config_path, 'real')
        config.update(cfg)
    else:
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
            launch_arguments={'config': str(config_path if backend == 'ros' else planning_config_path), 'backend': backend}.items()))
    actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                        parameters=[{'robot_description': urdf}]))
    # 面板退出后只关闭本launch拥有的显示/规划进程，不终止其他会话。
    panel = Node(package='dummy_bringup', executable='manual_joint_panel.py',
             arguments=['--config', str(config_path), '--report', LaunchConfiguration('report'),
                        '--description-xacro', model_path, '--backend', LaunchConfiguration('backend')] + (['--moveit'] if moveit_arg == 'true' else []) +
                       (['--auto-calibrate-and-test'] if LaunchConfiguration('auto_calibrate_and_test', default='false').perform(context) == 'true' else []) +
                       ['--initial-pose', LaunchConfiguration('initial_pose')], output='screen')
    panel_exit = RegisterEventHandler(OnProcessExit(
        target_action=lambda action: action is panel,
        on_exit=[EmitEvent(event=Shutdown(reason='手动面板已退出，关闭本会话显示与规划'))]))
    return actions + [
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
        panel_exit,
        panel,
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
        DeclareLaunchArgument('backend', default_value='real', choices=['real', 'ros', 'simulator'], description='真机或纯软件模拟器'),
        DeclareLaunchArgument('moveit', default_value='true', description='启用末端规划页及仅规划MoveIt服务'),
        DeclareLaunchArgument('sdk_prefix', default_value=EnvironmentVariable('DPSDK_HOST_SDK_PREFIX', default_value=''), description='ROS后端明确绑定的既有SDK安装前缀'),
        DeclareLaunchArgument('auto_calibrate_and_test', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('initial_pose', default_value='manual_limits', choices=['manual_limits']),
        DeclareLaunchArgument('operator_confirmed_double_limits', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('cylinder_pose', default_value='[]'),
        DeclareLaunchArgument('cylinder_dimensions', default_value='[]'),
        OpaqueFunction(function=build_nodes),
    ])
