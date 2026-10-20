##
# @file real_j1.launch.py
# @brief 要求明确身份和可执行文件的真机 J1 只读入口。
# 只传递状态/管理桥所需参数，不在此激活运动。参数存在性和 read_only=true 是启动前的最低检查；设备 ID 的 32 位十六进制语义、实际板卡身份和只读行为仍须由指定桥实现验证。
"""Explicit, read-only real J1 entry; it never authorizes or activates motion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


##
# @brief 在所有必需值非空且只读开关明确为真时创建桥进程动作。
# 启动参数使用原 LaunchConfiguration 交给进程；此函数只 strip 后检查缺失，不对端点格式或设备 ID 做完整身份认证。不会自动创建电机运动授权。
# @param context launch 上下文，提供 device_id/endpoint/serial/profile/read_only 和显式可执行文件。
# @return list：合法时一个 ExecuteProcess；缺参数或非只读则一个 Shutdown 事件。
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


##
# @brief 声明必须显式绑定的 J1 状态桥参数。
# device_id、endpoint、serial、read_only_device_executable 无默认值；profile 默认 real_j1、read_only 默认 true。OpaqueFunction 在参数展开后决定启动或阻断。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
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
