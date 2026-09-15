"""Launch-test the canonical Gazebo action path."""

from pathlib import Path
import json
import subprocess
import tempfile
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing


def generate_test_description():
    launch_file = Path(get_package_share_directory("dummy_bringup")) / "launch/gazebo_6dof.launch.py"
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
        launch_testing.actions.ReadyToTest(),
    ])


class GazeboRuntime(unittest.TestCase):
    def test_public_action_moves_and_recovers_after_cancel(self):
        probe = Path(get_package_prefix("dummy_bringup")) / "lib/dummy_bringup/action_gate_probe.py"
        with tempfile.TemporaryDirectory(prefix="r02_launch_") as temporary:
            report = Path(temporary) / "action.json"
            result = subprocess.run(
                ["/usr/bin/python3", str(probe), "--report", str(report)],
                capture_output=True, text=True, check=False, timeout=60,
            )
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertTrue(report.is_file())
            observations = json.loads(report.read_text())
            self.assertTrue(observations["first_goal_accepted"])
            self.assertFalse(observations["second_goal_accepted"])
            self.assertEqual(5, observations["first_goal_terminal_status"])
            self.assertTrue(observations["third_goal_accepted"])
            self.assertEqual(4, observations["third_goal_terminal_status"])
            self.assertFalse(observations["out_of_bounds_goal_accepted"])
            self.assertGreater(observations["joint_state_sample_count"], 2)
            self.assertGreater(observations["joint1_position_change_rad"], 0.02)
