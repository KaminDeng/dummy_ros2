"""Pure Gazebo six-axis backend without a DPSDK dependency."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    launch_file = Path(get_package_share_directory("dummy_bringup")) / "launch/gazebo_6dof.launch.py"
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)))])
