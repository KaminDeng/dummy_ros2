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


def _launch_gazebo_sim(context):
    share_dir = get_package_share_directory("dummy_moveit_config")
    xacro_file = os.path.join(share_dir, "config", "dummy-ros2.gz_sim.xacro")
    robot_urdf = xacro.process_file(xacro_file).toxml()

    runtime_dir = os.path.join(tempfile.gettempdir(), "dummy-ros2-gz-sim")
    os.makedirs(runtime_dir, exist_ok=True)
    urdf_path = os.path.join(runtime_dir, "dummy-ros2.urdf")
    sdf_path = os.path.join(runtime_dir, "dummy-ros2.sdf")
    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(robot_urdf)

    conversion = subprocess.run(
        ["gz", "sdf", "-p", urdf_path],
        check=True,
        capture_output=True,
        text=True,
    )
    with open(sdf_path, "w", encoding="utf-8") as sdf_file:
        sdf_file.write(conversion.stdout)

    world_file = os.path.join(share_dir, "worlds", "empty.sdf")
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    gui_args = "" if gui else " -s"
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

    return [
        gz_launch,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_urdf}],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            parameters=[
                {
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
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dummy_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]


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
