"""Explicit, read-only real J1 entry; it never authorizes or activates motion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def _start_read_only_chain(context):
    required = ("device_id", "endpoint", "serial", "profile", "read_only_device_executable")
    values = {name: LaunchConfiguration(name).perform(context).strip() for name in required}
    missing = [name for name, value in values.items() if not value]
    if missing:
        reason = "BLOCKED real_j1 requires explicit " + ", ".join(missing)
        return [EmitEvent(event=Shutdown(reason=reason))]
    if LaunchConfiguration("read_only").perform(context).strip().lower() != "true":
        reason = "BLOCKED real_j1 only supports read_only:=true"
        return [EmitEvent(event=Shutdown(reason=reason))]
    return [ExecuteProcess(
        cmd=[
            LaunchConfiguration("read_only_device_executable"),
            "--device-id", LaunchConfiguration("device_id"),
            "--endpoint", LaunchConfiguration("endpoint"),
            "--serial", LaunchConfiguration("serial"),
            "--profile", LaunchConfiguration("profile"),
            "--read-only", LaunchConfiguration("read_only"),
        ], output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device_id", description="Required 32-hex Dummy device identity"),
        DeclareLaunchArgument("endpoint", description="Required explicit Zenoh endpoint"),
        DeclareLaunchArgument("serial", description="Required board serial identity"),
        DeclareLaunchArgument("profile", default_value="real_j1"),
        DeclareLaunchArgument("read_only", default_value="true"),
        DeclareLaunchArgument(
            "read_only_device_executable",
            description="Required read-only state/management bridge executable",
        ),
        OpaqueFunction(function=_start_read_only_chain),
    ])
