"""Optional read-only Dabai input; disabled without importing the sensor overlay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _enabled(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized not in {"true", "false"}:
        raise ValueError("use_camera must be true or false")
    return normalized == "true"


def _camera_actions(context):
    if not _enabled(LaunchConfiguration("use_camera").perform(context)):
        return []

    from ament_index_python.packages import get_package_share_directory

    sensor_share = get_package_share_directory("dabai_sensor_bringup")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{sensor_share}/launch/sensor.launch.py"),
            launch_arguments={
                "source": LaunchConfiguration("camera_source"),
                "allow_hardware": LaunchConfiguration("allow_camera_hardware"),
                "serial_number": LaunchConfiguration("camera_serial_number"),
                "bag_path": LaunchConfiguration("camera_bag_path"),
                "capture_manifest": LaunchConfiguration("camera_capture_manifest"),
            }.items(),
        ),
        Node(
            package="dabai_sensor_examples",
            executable="readonly_consumer",
            namespace="/sensors/dabai/examples",
            output="screen",
            remappings=[
                ("status", "/sensors/dabai/status"),
                ("rgbd/frame", "/sensors/dabai/rgbd/frame"),
                ("diagnostics", "/sensors/dabai/examples/diagnostics"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="false"),
        DeclareLaunchArgument("camera_source", default_value="fixture"),
        DeclareLaunchArgument("allow_camera_hardware", default_value="false"),
        DeclareLaunchArgument("camera_serial_number", default_value=""),
        DeclareLaunchArgument("camera_bag_path", default_value=""),
        DeclareLaunchArgument("camera_capture_manifest", default_value=""),
        OpaqueFunction(function=_camera_actions),
    ])
