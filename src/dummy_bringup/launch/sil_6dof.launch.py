##
# @file sil_6dof.launch.py
# @brief 装配六轴软件在环 peer、受保护 ROS 门控和最小状态模型。
# SIL 使用固定测试身份、profile_id=1、mask=63 和本机测试端点，不等于真实设备部署。Node/ExecuteProcess 只是启动动作；会话、许可和硬件组件生命周期仍由运行时门控校验，不能把 launch 创建视为已取得电机控制。
"""Production six-axis SIL: committed DPSDK peer plus guarded ROS gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


##
# @brief 生成六轴 SIL 的最小关节链和受保护 ros2_control 配置。
# 关节角 rad、速度 rad/s；J3 采用 [0.61,2.53]，其他轴 [-3.14,3.14]。固定设备/客户端 ID 与 gate 一致；max_state_age_us=1000000。此零几何链用于接口装配，不替代真实 CAD 模型或碰撞规划。
# @param endpoint 本次软件 peer 的显式传输端点字符串。
# @return str：包含六轴位置/速度状态与位置命令接口的 URDF。
def _robot_description(endpoint):
    links = ['<link name="base_link"/>'] + [f'<link name="link{i}"/>' for i in range(1, 7)]
    joints, controls = [], []
    for axis in range(1, 7):
        parent = "base_link" if axis == 1 else f"link{axis - 1}"
        lower, upper = ("0.61", "2.53") if axis == 3 else ("-3.14", "3.14")
        joints.append(f'<joint name="Joint{axis}" type="revolute"><parent link="{parent}"/>'
                      f'<child link="link{axis}"/><axis xyz="0 0 1"/>'
                      f'<limit lower="{lower}" upper="{upper}" velocity="0.5" effort="1"/></joint>')
        controls.append(f'<joint name="Joint{axis}"><command_interface name="position"/>'
                        '<state_interface name="position"/><state_interface name="velocity"/></joint>')
    ##
    # GuardedZenbeddedSystem 名称描述受保护的 SDK 传输插件；这里绑定的是 SIL peer，不是 Gazebo 仿真组件。
    hardware = f'''<ros2_control name="dummy" type="system"><hardware>
<plugin>dummy_ros_control/GuardedZenbeddedSystem</plugin><param name="endpoint">{endpoint}</param>
<param name="device_id">01000000000000000000000000000000</param>
<param name="client_instance_id">02000000000000000000000000000000</param>
<param name="profile_id">1</param><param name="configured_mask">63</param>
<param name="max_state_age_us">1000000</param></hardware>'''
    return '<robot name="dummy_sil_6dof">' + ''.join(links + joints) + hardware + ''.join(controls) + '</ros2_control></robot>'


##
# @brief 将同一 SIL 端点及固定身份传给门控和状态发布器。
# session_id=11、测试 permit 剩余 60000000 µs，状态年龄上限 1000000 µs；profile_name=sil_6dof。向 controller_manager 传初始 dummy inactive 配置，创建节点不宣称其已激活或具有真实设备授权。
# @param context 含 endpoint 与 contract_bundle 的运行上下文。
# @return list[Node]：device_action_gate 和 robot_state_publisher。
def _nodes(context):
    endpoint = LaunchConfiguration("endpoint").perform(context)
    parameters = {
        "endpoint": endpoint, "device_id": "01000000000000000000000000000000",
        "client_instance_id": "02000000000000000000000000000000", "profile_id": 1,
        "configured_mask": 63, "session_id": 11, "state_max_age_us": 1000000,
        "permit_sha256": "01" + "00" * 31, "permit_remaining_us": 60000000,
        "contract_bundle": LaunchConfiguration("contract_bundle"), "profile_name": "sil_6dof",
    }
    return [
        Node(package="dummy_motion_client", executable="device_action_gate", output="screen",
             parameters=[parameters], arguments=["--ros-args", "-p",
                 "controller_manager:hardware_components_initial_state.inactive:=['dummy']"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": _robot_description(endpoint)}]),
    ]


##
# @brief 声明软件 peer/契约路径并启动 peer 与 ROS 侧动作。
# 端点默认 tcp/127.0.0.1:17446；可执行文件和契约目录默认由 DPSDK_ROOT 拼接明确路径，不搜索最新构建。peer 命令与门控共享客户端 ID、profile 1 和六轴掩码 63；未设置 use_sim_time，沿用墙上时钟。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
def generate_launch_description():
    endpoint = LaunchConfiguration("endpoint")
    executable = LaunchConfiguration("dpsdk_executable")
    return LaunchDescription([
        DeclareLaunchArgument("endpoint", default_value="tcp/127.0.0.1:17446"),
        DeclareLaunchArgument("dpsdk_executable", default_value=PathJoinSubstitution([
            EnvironmentVariable("DPSDK_ROOT"), "build", "posix-zenohc", "applications",
            "dummy_ref_control", "dummy_ref_d08_device_peer"])),
        DeclareLaunchArgument("contract_bundle", default_value=PathJoinSubstitution([
            EnvironmentVariable("DPSDK_HOST_SDK_PREFIX"), "share", "dummy_ref", "robot_contract"])),
        ExecuteProcess(cmd=[executable, "--endpoint", endpoint, "--client-id",
                            "02000000000000000000000000000000", "--profile-id", "1",
                            "--configured-mask", "63"], output="screen"),
        OpaqueFunction(function=_nodes),
    ])
