"""Read-only-by-default real J1 declaration; motion requires a separate human gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device_id", description="Required 32-hex Dummy device identity"),
        DeclareLaunchArgument("endpoint", description="Required explicit Zenoh endpoint"),
        DeclareLaunchArgument("serial", description="Required board serial identity"),
        DeclareLaunchArgument("profile", default_value="real_j1"),
        DeclareLaunchArgument("read_only", default_value="true"),
        LogInfo(msg=["real_j1 configured read_only=", LaunchConfiguration("read_only")]),
    ])
