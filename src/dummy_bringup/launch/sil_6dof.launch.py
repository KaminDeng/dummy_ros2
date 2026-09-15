"""Six-axis device SIL: a committed DPSDK peer plus a ROS device-side probe."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Start the two independently-owned halves of the device SIL contract."""
    dpsdk_executable = LaunchConfiguration("dpsdk_executable")
    ros_device_executable = LaunchConfiguration("ros_device_executable")
    device_id = LaunchConfiguration("device_id")
    serial = LaunchConfiguration("serial")
    profile = LaunchConfiguration("profile")
    endpoint = LaunchConfiguration("endpoint")
    return LaunchDescription([
        DeclareLaunchArgument("device_id", default_value="sil-6dof-000000000000000000000001"),
        DeclareLaunchArgument("serial", default_value="virtual-can-sil-6dof"),
        DeclareLaunchArgument("profile", default_value="sil_6dof"),
        DeclareLaunchArgument("endpoint", default_value="tcp/127.0.0.1:17446"),
        DeclareLaunchArgument(
            "dpsdk_executable",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("DPSDK_ROOT"), "build", "posix-zenohc", "applications",
                "dummy_ref_control", "dummy_ref_posix_node",
            ]),
        ),
        # R06 owns the production bridge.  This explicit override keeps the launch
        # runnable with its committed GuardedSystem probe until that bridge exists.
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
