"""Launch-level contract checks for the two device SIL entries and real J1 gate."""

from pathlib import Path
import subprocess
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import yaml


def generate_test_description():
    return LaunchDescription([
        ExecuteProcess(cmd=["/bin/sh", "-c", "while :; do sleep 1; done"]),
        launch_testing.actions.ReadyToTest(),
    ])


class BackendContractLaunchTest(unittest.TestCase):
    def test_backend_profiles_are_explicit_and_motion_is_not_automatic(self):
        package_root = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((package_root / "config/backends.yaml").read_text())
        self.assertEqual(False, config["security"]["auto_grant"])
        self.assertEqual(False, config["security"]["auto_capture_reference"])
        self.assertEqual(False, config["security"]["auto_enable"])
        self.assertEqual(["Joint1"], config["backends"]["sil_j1"]["active_axes"])
        self.assertEqual(["Joint1"], config["backends"]["real_j1"]["active_axes"])
        for name in ("sil_6dof", "sil_j1"):
            entry = config["backends"][name]
            for key in ("device_id", "serial", "profile", "endpoint"):
                self.assertTrue(entry[key])
            source = (package_root / "launch" / f"{name}.launch.py").read_text()
            self.assertIn("ros_device_executable", source)
            self.assertIn("Required ROS device bridge", source)

    def test_real_j1_has_an_executable_read_only_gate(self):
        source = (Path(__file__).resolve().parents[1] / "launch/real_j1.launch.py").read_text()
        self.assertIn("read_only_device_executable", source)
        self.assertIn("ExecuteProcess", source)
        self.assertIn("BLOCKED real_j1", source)
        self.assertNotIn("Grant", source)
        self.assertNotIn("Capture", source)
        self.assertNotIn("Enable", source)

    def test_launch_expands_identity_and_mode_into_bridge_command(self):
        package_root = Path(__file__).resolve().parents[1]
        expected = (
            "--device-id bridge-device", "--endpoint tcp/127.0.0.1:17555",
            "--serial bridge-serial", "--profile bridge-profile", "--read-only true",
        )
        for name in ("sil_6dof", "sil_j1", "real_j1"):
            launch_file = package_root / "launch" / f"{name}.launch.py"
            command = [
                "ros2", "launch", str(launch_file),
                "device_id:=bridge-device", "endpoint:=tcp/127.0.0.1:17555",
                "serial:=bridge-serial", "profile:=bridge-profile",
                "read_only:=true", "dpsdk_executable:=/usr/bin/true",
            ]
            if name == "real_j1":
                command.append("read_only_device_executable:=/usr/bin/echo")
            else:
                command.append("ros_device_executable:=/usr/bin/echo")
            completed = subprocess.run(
                command, capture_output=True, text=True, check=False, timeout=10,
            )
            output = completed.stdout + completed.stderr
            self.assertEqual(0, completed.returncode, output)
            expected_mode = "--read-only true" if name == "real_j1" else "--read-only false"
            for value in (*expected[:-1], expected_mode):
                self.assertIn(value, output, output)

    def test_missing_bridge_arguments_fail_closed(self):
        package_root = Path(__file__).resolve().parents[1]
        cases = (
            ("sil_j1", ("dpsdk_executable:=/usr/bin/true",), "ros_device_executable"),
            ("real_j1", (), "device_id"),
        )
        for name, arguments, required in cases:
            completed = subprocess.run(
                ["ros2", "launch", str(package_root / "launch" / f"{name}.launch.py"), *arguments],
                capture_output=True, text=True, check=False, timeout=10,
            )
            output = completed.stdout + completed.stderr
            self.assertNotEqual(0, completed.returncode, output)
            self.assertIn(f"required argument '{required}'", output, output)
