"""R02 only reports runtime behavior from a corresponding live probe."""

from pathlib import Path
import subprocess
import sys
from unittest.mock import patch
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


class R02Acceptance(unittest.TestCase):
    def evidence(self, case_id: str) -> dict:
        return run_case("R02", case_id, {
            "case_id": case_id, "hardware": "none", "test_only": True,
        })

    def test_mod05_uses_the_joint3_range_probe(self):
        result = self.evidence("MOD05")
        self.assertEqual("PASS", result["status"], result["reason"])
        self.assertIn("all_zero_pose_is_rejected", result["observations"]["case_target"])

    def test_runtime_cases_are_not_claimed_when_the_live_gazebo_runtime_is_absent(self):
        from dummy_test_support.adapters import r02

        with patch.object(r02, "_run_runtime_probe", return_value={"_runtime_unavailable": "test runtime absent"}):
            for case_id in ("FRE11", "ROS01", "ROS02", "ROS03", "ROS11"):
                result = self.evidence(case_id)
                self.assertEqual("NOT_RUN", result["status"])
                self.assertFalse(result["executed"])
                self.assertTrue(result["reason"])

    def test_each_runtime_case_requires_its_own_complete_probe_observations(self):
        from dummy_test_support.adapters import r02

        native_identity = {
            "hardware_components": [{"name": "GazeboSimSystem", "type": ""}],
            "hardware_component_count": 1,
            "gazebo_hardware_component_count": 1,
            "native_gazebo_backend_only": True,
            "dummy_ros_control_device_present": False,
            "dpsdk_device_dependencies_present": False,
        }
        observations = {
            "FRE11": {
                **native_identity,
                "gazebo_world_control_available": True,
                "clock_samples_before_pause": 2,
                "clock_advanced_before_pause": True,
                "clock_samples_while_paused": 5,
                "clock_advanced_while_paused": False,
                "clock_stopped_while_paused": True,
                "clock_advanced_after_resume": True,
                "gazebo_pause_requested": True,
                "gazebo_resume_requested": True,
            },
            "ROS01": {
                **native_identity,
                "controller_manager_available": True,
                "loaded_controllers": [{"name": "joint_state_broadcaster", "state": "active"}],
                "active_controller_count": 1,
                "joint_state_sample_count": 2,
                "public_action_available": True,
                "gazebo_hardware_component_present": True,
            },
            "ROS02": {
                **native_identity,
                "public_action_available": True,
                "goal_accepted": True,
                "goal_terminal_status": 4,
                "joint_state_time_series": [{"time_sec": 0.0, "joint1_rad": 0.0}, {"time_sec": 0.5, "joint1_rad": 0.05}, {"time_sec": 1.0, "joint1_rad": 0.1}],
                "actual_joint1_change_rad": 0.1,
                "final_joint1_abs_error_rad": 0.01,
            },
            "ROS03": {
                **native_identity,
                "no_command_sent": True,
                "joint_state_time_series": [{"time_sec": 0.0, "joint1_rad": 0.0}, {"time_sec": 0.5, "joint1_rad": 0.0}, {"time_sec": 1.0, "joint1_rad": 0.0}],
                "max_joint_drift_rad": 0.0,
                "joint_axis_drift_rad": {f"Joint{index}": 0.0 for index in range(1, 7)},
                "protobuf_defaulted_axes": [],
                "base_pose_time_series": [{"time_sec": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}, {"time_sec": 0.5, "x": 0.0, "y": 0.0, "z": 0.0}, {"time_sec": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}],
                "max_base_translation_drift_m": 0.0,
                "base_is_fixed": True,
            },
            "ROS11": {
                **native_identity,
                "controller_manager_available": True,
                "loaded_controllers": [{"name": "_internal_arm_controller", "type": "joint_trajectory_controller/JointTrajectoryController", "state": "active"}],
                "active_controller_names": ["_internal_arm_controller"],
                "locked_controller_name": "_internal_arm_controller",
                "locked_controller_type": "joint_trajectory_controller/JointTrajectoryController",
                "locked_configuration_accepted": True,
            },
        }
        with patch.object(
            r02, "_run_runtime_probe", side_effect=lambda case_id: observations[case_id], create=True,
        ):
            for case_id in observations:
                result = self.evidence(case_id)
                self.assertEqual("PASS", result["status"], result["reason"])
                self.assertEqual(case_id, result["observations"]["case_id"])
                self.assertEqual(observations[case_id], result["observations"]["probe_observations"])

        incomplete = dict(observations["ROS02"])
        incomplete.pop("final_joint1_abs_error_rad")
        with patch.object(r02, "_run_runtime_probe", return_value=incomplete, create=True):
            result = self.evidence("ROS02")
        self.assertEqual("FAIL", result["status"])
        self.assertTrue(result["executed"])

        non_native = dict(observations["ROS11"])
        non_native["native_gazebo_backend_only"] = False
        with patch.object(r02, "_run_runtime_probe", return_value=non_native, create=True):
            result = self.evidence("ROS11")
        self.assertEqual("FAIL", result["status"])

    def test_ros01_recognizes_the_live_gazebo_component_name_when_type_is_empty(self):
        probe = TEST_ROOT.parents[2] / "src/dummy_bringup/scripts/gazebo_runtime_probe.py"
        source = probe.read_text()
        self.assertIn("_is_gazebo_component", source)
        self.assertIn('item["name"]', source)

    def test_ros03_reads_fixed_base_pose_from_the_gazebo_pose_topic(self):
        probe = TEST_ROOT.parents[2] / "src/dummy_bringup/scripts/gazebo_runtime_probe.py"
        source = probe.read_text()
        self.assertIn('"gz", "topic", "-e", "-n", "1"', source)
        self.assertIn("/pose/info", source)

    def test_fre11_distinguishes_frozen_clock_values_from_repeated_clock_messages(self):
        probe = TEST_ROOT.parents[2] / "src/dummy_bringup/scripts/gazebo_runtime_probe.py"
        source = probe.read_text()
        self.assertIn("clock_advanced_while_paused", source)
        self.assertIn("_wait_for_clock_quiescence", source)

    def test_sources_bind_the_native_gazebo_launch_and_model_inputs(self):
        from dummy_test_support.adapters import r02

        required = {
            "src/dummy_moveit_config/launch/gz_moveit.launch.py",
            "src/dummy_moveit_config/launch/gz_sim.launch.py",
            "src/dummy_moveit_config/config/dummy-ros2.gz_sim.xacro",
            "src/dummy_moveit_config/config/dummy-ros2.urdf.xacro",
            "src/dummy-ros2_description/urdf/dummy-ros2.xacro",
            "src/dummy_moveit_config/worlds/empty.sdf",
            "src/dummy_motion_client/src/native_action_gate.cpp",
            "src/dummy_bringup/CMakeLists.txt",
            "src/dummy_bringup/package.xml",
        }
        self.assertTrue(required.issubset(r02.SOURCES))

    def test_pose_parser_uses_one_complete_base_link_block_and_requires_all_axes(self):
        probe = TEST_ROOT.parents[2] / "src/dummy_bringup/scripts/gazebo_runtime_probe.py"
        code = """
import importlib.util
import sys

spec = importlib.util.spec_from_file_location('r02_probe', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
payload = '''pose {
  name: "other_base_link"
  position { x: 99 y: 99 z: 99 }
}
pose {
  name: "base_link"
  position {
    x: 1.25
    y: -2.5
    z: 3.75
  }
}
pose {
  name: "base_link_visual"
  position { x: 7 y: 8 z: 9 }
}
'''
assert module._parse_base_link_pose(payload) == {
    'x': 1.25, 'y': -2.5, 'z': 3.75, 'protobuf_defaulted_axes': [],
}
assert module._parse_base_link_pose(
    'pose { name: "base_link" position { z: 3.5 } }'
) == {'x': 0.0, 'y': 0.0, 'z': 3.5, 'protobuf_defaulted_axes': ['x', 'y']}
for invalid in (
    'pose { name: "base_link" }',
    'pose { name: "base_link" position { x: not-a-number y: 2 z: 3 } }',
    'pose { name: "base_link" position { x: 1 y: 2 z: 3 } }\\n'
    'pose { name: "base_link" position { x: 4 y: 5 z: 6 } }',
):
    try:
        module._parse_base_link_pose(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError(invalid)
"""
        result = subprocess.run(
            ["/usr/bin/python3", "-c", code, str(probe)],
            cwd=TEST_ROOT.parents[2], capture_output=True, text=True, check=False,
        )
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
