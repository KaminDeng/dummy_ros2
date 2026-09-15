"""J1 SIL backend. This launch never opens tty, J-Link, or SocketCAN."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    endpoint = LaunchConfiguration("endpoint")
    executable = PathJoinSubstitution([
        EnvironmentVariable("DPSDK_ROOT"), "build", "posix-zenohc", "applications",
        "dummy_ref_control", "dummy_ref_posix_node",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("endpoint", default_value="tcp/127.0.0.1:17447"),
        ExecuteProcess(cmd=[executable, "--listen", endpoint], output="screen"),
    ])
