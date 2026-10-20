##
# @file mock_6dof.launch.py
# @brief 保留 mock 六轴入口并转交标准 Gazebo 后端。
# 该别名包含 gazebo_6dof.launch.py，使用 Gazebo 原生仿真和公开门控；不是另一个 SDK 设备模拟链，也不建立真实 USB 连接。
"""Pure Gazebo six-axis backend without a DPSDK dependency."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


##
# @brief 从 ament 安装共享目录包含标准六轴 Gazebo 启动描述。
# 本文件没有再声明 gui/rviz 参数；默认值和时钟/场景装配由被包含的 gazebo_6dof 入口负责。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
def generate_launch_description():
    launch_file = Path(get_package_share_directory("dummy_bringup")) / "launch/gazebo_6dof.launch.py"
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)))])
