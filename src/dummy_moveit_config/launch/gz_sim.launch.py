##
# @file gz_sim.launch.py
# @brief Gazebo Sim入口：展开共享URDF、转换SDF、启动server/clock桥并装配机器人和控制器。
"""Launch the dummy arm in Gazebo Sim Harmonic using gz_ros2_control."""

import os
import subprocess
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


##
# @brief 在launch运行上下文中解析gui并生成Gazebo相关动作。
# @param context 可解析LaunchConfiguration的launch上下文。
# @return Gazebo、RSP、clock桥、模型create及两个spawner动作列表。展开/转换异常直接传播，不伪造可用模型。
def _launch_gazebo_sim(context):
    ##
    # 解析已安装配置包并展开gz_sim Xacro，后者选择GazeboSimSystem及固定根。
    share_dir = get_package_share_directory("dummy_moveit_config")
    xacro_file = os.path.join(share_dir, "config", "dummy-ros2.gz_sim.xacro")
    robot_urdf = xacro.process_file(xacro_file).toxml()

    ##
    # 固定临时目录中的同名URDF/SDF会被重写；这是运行期中间产物，不是新的模型来源，多个同名实例不能靠此路径自动隔离。
    runtime_dir = os.path.join(tempfile.gettempdir(), "dummy-ros2-gz-sim")
    os.makedirs(runtime_dir, exist_ok=True)
    urdf_path = os.path.join(runtime_dir, "dummy-ros2.urdf")
    sdf_path = os.path.join(runtime_dir, "dummy-ros2.sdf")
    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_urdf)

    ##
    # 调用gz sdf把URDF转换成SDF；check=True使非零退出立即失败，当前调用没有额外timeout。
    conversion = subprocess.run(
        ["gz", "sdf", "-p", urdf_path],
        check=True,
        capture_output=True,
        text=True,
    )
    with open(sdf_path, "w", encoding="utf-8") as sdf_file:
        sdf_file.write(conversion.stdout)

    ##
    # 使用包内empty.sdf；gui只有字符串忽略大小写等于true才启GUI，否则加-s仅运行server。
    world_file = os.path.join(share_dir, "worlds", "empty.sdf")
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    gui_args = "" if gui else " -s"
    ##
    # 通过ros_gz_sim启动仿真，-r请求运行世界；不启动真实DPSDK节点。
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r{gui_args} -v 4 {world_file}"}.items(),
    )

    ##
    # 所有仿真节点采用use_sim_time，clock桥提供ROS时钟。create/spawner列在动作树中，不构成额外事件就绪屏障；spawner自身负责服务等待。
    return [
        gz_launch,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_urdf, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            parameters=[
                {
                    "use_sim_time": True,
                    "world": "default",
                    "file": sdf_path,
                    "name": "dummy-ros2",
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                }
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["_internal_arm_controller", "--controller-manager", "/controller_manager"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ]


##
# @brief 声明gui默认true，延迟到OpaqueFunction执行时才展开模型/调用转换器。
# @return 含参数声明与延迟装配回调的LaunchDescription。
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo Sim GUI in addition to the server.",
            ),
            OpaqueFunction(function=_launch_gazebo_sim),
        ]
    )
