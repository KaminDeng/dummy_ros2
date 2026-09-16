"""Production J1 SIL: committed DPSDK peer plus guarded ROS device gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


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


def generate_launch_description():
    endpoint = LaunchConfiguration("endpoint")
    executable = LaunchConfiguration("dpsdk_executable")
    return LaunchDescription([
        DeclareLaunchArgument("endpoint", default_value="tcp/127.0.0.1:17447"),
        DeclareLaunchArgument("dpsdk_executable", default_value=PathJoinSubstitution([
            EnvironmentVariable("DPSDK_ROOT"), "build", "posix-zenohc", "applications",
            "dummy_ref_control", "dummy_ref_d08_device_peer"])),
        DeclareLaunchArgument("contract_bundle", default_value=PathJoinSubstitution([
            EnvironmentVariable("DPSDK_ROOT"), "contracts", "dummy_robot_v1"])),
        ExecuteProcess(cmd=[executable, "--endpoint", endpoint, "--client-id",
                            "02000000000000000000000000000000", "--profile-id", "2",
                            "--configured-mask", "1"], output="screen"),
        OpaqueFunction(function=_nodes),
    ])
