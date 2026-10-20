##
# @file manual_moveit_planning.launch.py
# @brief 启动与手动面板同模型、同工作范围的只规划 MoveIt。
# 不创建 USB 通信或轨迹控制器；allow_trajectory_execution=False 关闭 MoveIt 执行入口。规划服务的成功结果仍须由面板验证完整实际命令曲线、场景版本和设备身份，再显式提交运动。
"""仅启动与手动面板同模型的 MoveIt；不接 USB、不启动轨迹执行。"""
import importlib.util
import math
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


##
# @brief 加载安装区的面板配置路径/完整性校验模块。
# 与面板主入口采用同一配置解析逻辑；real 要求明确部署产物，simulator 可用模板。
# @return module：明确安装在 lib/dummy_bringup 的 manual_panel_config。
def _config_helpers():
    path = Path(get_package_prefix('dummy_bringup')) / 'lib/dummy_bringup/manual_panel_config.py'
    spec = importlib.util.spec_from_file_location('manual_panel_config', path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper


##
# @brief 延迟导入 MoveIt 配置构建器。
# 让导入 launch 描述与创建 MoveIt 配置分开，便于独立安装布局与离线配置测试。
# @return type：MoveItConfigsBuilder 类，供 build_nodes 实例化。
def _moveit_builder():
    from moveit_configs_utils import MoveItConfigsBuilder
    return MoveItConfigsBuilder


##
# @brief 把面板限制转换为 ROS 单位并创建只规划 move_group。
# 每轴位置范围 °→rad，速度 °/s→rad/s，加速度本来就是 rad/s²。用部署值覆盖旧模型限制，再设 TOTG 20 ms 重采样与 0.01 rad 路径容差，以及 Fast/Precise 两套 RRTConnect 几何采样配置；不自动授权或执行电机运动。
# @param context 包含 config/backend 的 launch 上下文。
# @return list[Node]：名为 manual_move_group 的 move_group 节点动作。
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
    ##
    # 此处覆盖规划工作域与动态上限，不修改部署 JSON 或设备策略；规划通过不代表实际起点、许可或碰撞场景仍然有效。
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
                     ##
                     # 只开放 FK/IK、路径和状态有效性等计算及场景发布；运动由面板显式请求给 Native，不通过 MoveIt 执行控制器。
                     'allow_trajectory_execution': False,
                     'publish_robot_description_semantic': True,
                     'publish_planning_scene': True,
                     'publish_geometry_updates': True,
                     'publish_state_updates': True,
                     'publish_transforms_updates': True,
                 }])]


##
# @brief 声明配置及后端，并延迟创建仅规划节点。
# config 默认空；backend 默认 real 且只接受 real/simulator。此入口没有设置 use_sim_time，沿用节点默认墙上时钟。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value=''),
                              DeclareLaunchArgument('backend', default_value='real', choices=['real', 'simulator']),
                              OpaqueFunction(function=build_nodes)])
