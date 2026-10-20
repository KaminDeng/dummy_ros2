##
# @file sil_j1.launch.py
# @brief 装配 J1 软件在环 peer、受保护 ROS 门控和单轴模型。
# 固定测试身份、profile_id=2、configured_mask=1 区分于六轴 SIL。该入口会启动软件 peer，不连接真实 USB；会话许可与控制组件激活仍由运行时校验，启动描述本身不构成运动授权。
"""Production J1 SIL: committed DPSDK peer plus guarded ROS device gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


##
# @brief 构造单轴 SIL 的最小 URDF 与受保护接口。
# 位置限位 ±0.5 rad、速度 0.25 rad/s；固定设备/客户端身份和 profile 2、mask 1 与门控一致。无真实连杆碰撞几何，不应当作六轴模型使用。
# @param endpoint 软件 peer 的传输端点。
# @return str：Joint1 的 URDF，位置命令及位置/速度状态接口。
def _robot_description(endpoint):
    return f'''<robot name="dummy_sil_j1"><link name="base_link"/><link name="link1"/>
<joint name="Joint1" type="revolute"><parent link="base_link"/><child link="link1"/>
<axis xyz="0 0 1"/><limit lower="-0.5" upper="0.5" velocity="0.25" effort="1"/></joint>
<ros2_control name="dummy" type="system"><hardware>
<plugin>dummy_ros_control/GuardedZenbeddedSystem</plugin><param name="endpoint">{endpoint}</param>
<param name="device_id">01000000000000000000000000000000</param>
<param name="client_instance_id">02000000000000000000000000000000</param>
<param name="profile_id">2</param><param name="configured_mask">1</param>
<param name="max_state_age_us">1000000</param></hardware>
<joint name="Joint1"><command_interface name="position"/><state_interface name="position"/>
<state_interface name="velocity"/></joint></ros2_control></robot>'''


##
# @brief 创建绑定单轴测试身份的 gate 和状态发布器动作。
# session=11、状态年龄上限 1000000 µs、测试许可 60000000 µs；profile_name=sil_j1。dummy 初始 inactive 是传入的生命周期配置，不把进程创建等同控制组件已激活。
# @param context 含 endpoint/contract_bundle 的 launch 上下文。
# @return list[Node]：device_action_gate 与 robot_state_publisher。
def _nodes(context):
    endpoint = LaunchConfiguration("endpoint").perform(context)
    parameters = {
        "endpoint": endpoint, "device_id": "01000000000000000000000000000000",
        "client_instance_id": "02000000000000000000000000000000", "profile_id": 2,
        "configured_mask": 1, "session_id": 11, "state_max_age_us": 1000000,
        "permit_sha256": "01" + "00" * 31, "permit_remaining_us": 60000000,
        "contract_bundle": LaunchConfiguration("contract_bundle"), "profile_name": "sil_j1",
    }
    return [
        Node(package="dummy_motion_client", executable="device_action_gate", output="screen",
             parameters=[parameters], arguments=["--ros-args", "-p",
                 "controller_manager:hardware_components_initial_state.inactive:=['dummy']"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": _robot_description(endpoint)}]),
    ]


##
# @brief 声明单轴 SIL 路径/端点并创建软件 peer 和 ROS 动作。
# 默认端点 tcp/127.0.0.1:17447，避免与六轴默认端口混用；从 DPSDK_ROOT 拼接明确 peer 和契约路径，命令携带 profile 2、mask 1。未设置 use_sim_time，使用默认墙上时钟。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
def generate_launch_description():
    endpoint = LaunchConfiguration("endpoint")
    executable = LaunchConfiguration("dpsdk_executable")
    return LaunchDescription([
        DeclareLaunchArgument("endpoint", default_value="tcp/127.0.0.1:17447"),
        DeclareLaunchArgument("dpsdk_executable", default_value=PathJoinSubstitution([
            EnvironmentVariable("DPSDK_ROOT"), "build", "posix-zenohc", "applications",
            "dummy_ref_control", "dummy_ref_d08_device_peer"])),
        DeclareLaunchArgument("contract_bundle", default_value=PathJoinSubstitution([
            EnvironmentVariable("DPSDK_ROOT"), "contracts", "dummy_robot_v2"])),
        ExecuteProcess(cmd=[executable, "--endpoint", endpoint, "--client-id",
                            "02000000000000000000000000000000", "--profile-id", "2",
                            "--configured-mask", "1"], output="screen"),
        OpaqueFunction(function=_nodes),
    ])
