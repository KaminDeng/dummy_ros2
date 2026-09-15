"""Launch-test MoveIt planning and synchronized scene behavior."""

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


class PlanExecuteRuntime(unittest.TestCase):
    def test_planning_probe_observes_collision_and_partial_cartesian_path(self):
        probe = Path(get_package_prefix("dummy_bringup")) / "lib/dummy_bringup/planning_probe.py"
        with tempfile.TemporaryDirectory(prefix="r05_launch_") as temporary:
            report = Path(temporary) / "planning.json"
            result = subprocess.run(
                ["/usr/bin/python3", str(probe), "--report", str(report)],
                capture_output=True, text=True, check=False, timeout=90,
            )
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertTrue(report.is_file())
            observations = json.loads(report.read_text())
            self.assertEqual(1, observations["valid_joint_plan_error_code"])
            self.assertGreater(observations["valid_joint_trajectory_points"], 1)
            self.assertLess(observations["unreachable_cartesian_fraction"], 1.0)
            self.assertTrue(observations["barrier_collision_state_found"])
            self.assertNotEqual(1, observations["barrier_collision_goal_error_code"])
            self.assertTrue(observations["planning_scene_matches_canonical"])
            self.assertTrue(observations["world_scene_revision_matches"])
            self.assertFalse(observations["trajectory_executed"])
