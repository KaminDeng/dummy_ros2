##
# @file demo_sensor_input.launch.py
# @brief 以默认关闭、延迟导入方式接入可选只读 RGB-D 输入。
# use_camera=false 时不查询传感器包，基础工作区无需传感器 overlay。开启后才包含传感器入口并启动只读消费示例；是否允许实际相机由独立参数转交给传感器入口判断。
"""Optional read-only Dabai input; disabled without importing the sensor overlay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


##
# @brief 严格解析相机开关而不接受含糊真值。
# @exception ValueError 除 true/false 外的值，包括 1，不被接受。
# @param value launch 参数文本，允许首尾空白和大小写差异。
# @return bool：规范化 true 为 True，false 为 False。
def _enabled(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized not in {"true", "false"}:
        raise ValueError("use_camera must be true or false")
    return normalized == "true"


##
# @brief 按运行时开关延迟解析传感器 overlay 并创建只读动作。
# source、allow_hardware、serial_number、bag_path、capture_manifest 显式传递；示例位于 /sensors/dabai/examples，输入 remap 到公共状态/图像帧话题，诊断保持示例命名空间。
# @param context launch 运行上下文，用于展开 use_camera。
# @return list：关闭时空列表；开启时传感器 Include 与 readonly_consumer 节点。
def _camera_actions(context):
    if not _enabled(LaunchConfiguration("use_camera").perform(context)):
        return []

    ##
    # 包查询严格放在开关检查之后，关闭可选输入时不会因未安装传感器包而失败。
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


##
# @brief 声明默认关闭相机及可选来源/身份参数。
# camera_source 默认 fixture；允许硬件参数默认 false。串号、录包路径和捕获清单为空，具体来源校验由包含的传感器 launch 负责；OpaqueFunction 直到启动上下文形成后才查包。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
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
