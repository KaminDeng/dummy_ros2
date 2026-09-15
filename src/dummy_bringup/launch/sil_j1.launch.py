"""J1 device SIL.  It never opens tty, J-Link, USB, or SocketCAN."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    endpoint = LaunchConfiguration("endpoint")
    dpsdk_executable = LaunchConfiguration("dpsdk_executable")
    ros_device_executable = LaunchConfiguration("ros_device_executable")
    device_id = LaunchConfiguration("device_id")
    serial = LaunchConfiguration("serial")
    profile = LaunchConfiguration("profile")
    return LaunchDescription([
        DeclareLaunchArgument("device_id", default_value="sil-j1-00000000000000000000000001"),
        DeclareLaunchArgument("serial", default_value="virtual-can-sil-j1"),
        DeclareLaunchArgument("profile", default_value="sil_j1"),
        DeclareLaunchArgument("endpoint", default_value="tcp/127.0.0.1:17447"),
        DeclareLaunchArgument(
            "dpsdk_executable",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("DPSDK_ROOT"), "build", "posix-zenohc", "applications",
                "dummy_ref_control", "dummy_ref_posix_node",
            ]),
        ),
        DeclareLaunchArgument(
            "ros_device_executable",
            description="Required ROS device bridge or committed GuardedSystem probe executable",
        ),
        ExecuteProcess(cmd=[dpsdk_executable, "--listen", endpoint], output="screen"),
        ExecuteProcess(cmd=[
            ros_device_executable, "--device-id", device_id, "--endpoint", endpoint,
            "--serial", serial, "--profile", profile, "--read-only", "false",
        ], output="screen"),
    ])
