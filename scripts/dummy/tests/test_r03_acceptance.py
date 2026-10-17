#!/usr/bin/env python3
"""Build and execute the ROS-owned R03 codec and goal-gate probes."""
import os

from pathlib import Path
import subprocess
import tempfile
import json
import hashlib
import unittest


ROOT = Path(__file__).resolve().parents[3]
DPSDK = Path(os.environ.get("DPSDK_ROOT", str(Path(__file__).resolve().parents[3].parent / "dpsdk_consolidated")))
C03_INCLUDE = ROOT / "contracts/dummy_robot_v2/codec"


class R03AcceptanceTest(unittest.TestCase):
    def test_c05_wire_release_is_pinned_to_the_r03_handoff(self):
        manifest = json.loads((ROOT / "contracts/dummy_robot_v2/manifest.json").read_text())
        self.assertEqual("dummy-wire-v2", manifest["release_id"])
        self.assertEqual("wire", manifest["stage"])
        self.assertEqual("2f9ebf1", manifest["r03_commit"])

    def test_c03_prototype_is_pinned_and_not_mislabeled_wire_v1(self):
        prefix = "contracts/dummy_robot_v2/c03_prototype/generated"
        listing = subprocess.run(
            ["git", "ls-tree", "-r", "--name-only", "2f9ebf1", prefix],
            cwd=ROOT, capture_output=True, text=True, check=True,
        ).stdout.splitlines()
        self.assertTrue(listing)
        manifest = json.loads((ROOT / "contracts/dummy_robot_v2/manifest.json").read_text())
        self.assertEqual("dummy-wire-v2", manifest["release_id"])
        self.assertFalse(any(entry["path"].startswith("c03_prototype/") for entry in manifest["payloads"]))
        for relative in listing:
            expected = subprocess.run(
                ["git", "show", f"2f9ebf1:{relative}"], cwd=ROOT,
                capture_output=True, check=True,
            ).stdout
            self.assertEqual(expected, (ROOT / relative).read_bytes(), relative)

    def compile_and_run(self, source: Path) -> None:
        self.assertTrue(source.is_file(), f"missing R03 SUT: {source}")
        with tempfile.TemporaryDirectory(prefix="dummy_r03_") as temporary:
            binary = Path(temporary) / source.stem
            completed = subprocess.run(
                [
                    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "src/dummy_ros_control/include"),
                    "-I", str(C03_INCLUDE),
                    "-I", str(DPSDK / "libraries/dp_comm/include"),
                    str(source), "-o", str(binary),
                ],
                cwd=ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(0, completed.returncode, completed.stderr)
            executed = subprocess.run([str(binary)], check=False, capture_output=True, text=True)
            self.assertEqual(0, executed.returncode, executed.stderr)

    def test_gen01_gen03_and_rpc_payloads_use_actual_generated_codecs(self):
        self.compile_and_run(ROOT / "src/dummy_ros_control/test/wire_probe_tests.cpp")

    def test_rpc12_preserves_caller_owner_and_session_snapshot(self):
        self.compile_and_run(ROOT / "src/dummy_ros_control/test/wire_probe_tests.cpp")

    def test_mov14_unregistered_goal_and_wrong_digest_are_rejected(self):
        self.compile_and_run(ROOT / "src/dummy_ros_control/test/goal_gate_tests.cpp")

    def test_rpc01_and_rpc08_use_real_dp_comm_transport_and_handles(self):
        probe = ROOT / "scripts/dummy/r03_rpc_probe.py"
        self.assertTrue(probe.is_file(), "R03 RPC orchestrator is missing")
        with tempfile.TemporaryDirectory(prefix="dummy_r03_rpc_") as temporary:
            report = Path(temporary) / "report.json"
            completed = subprocess.run(
                ["/usr/bin/python3", str(probe), "--report", str(report)],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=35,
            )
            self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)
            observed = json.loads(report.read_text())
            self.assertTrue(observed["interprocess_service_passed"])
            self.assertTrue(observed["queued_not_accepted"])
            self.assertTrue(observed["large_uint64_preserved"])
            self.assertEqual(3, observed["request_handle_tests_passed"])
        destructor_probe = ROOT / "build_dummy/dummy_ros_control/request_handle_destructor_tests"
        self.assertTrue(destructor_probe.is_file(), "RequestHandle destructor probe was not built")
        completed = subprocess.run(
            [str(destructor_probe)], cwd=ROOT, capture_output=True, text=True, check=False
        )
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)

    def test_guarded_jtc_plugin_closes_base_action_and_topic_inputs(self):
        header = ROOT / "src/dummy_ros_control/include/dummy_ros_control/guarded_joint_trajectory_controller.hpp"
        source = ROOT / "src/dummy_ros_control/src/guarded_joint_trajectory_controller.cpp"
        plugins = ROOT / "src/dummy_ros_control/dummy_ros_control_plugins.xml"
        self.assertTrue(header.is_file())
        self.assertTrue(source.is_file())
        self.assertTrue(plugins.is_file())
        implementation = source.read_text()
        self.assertIn("joint_command_subscriber_.reset()", implementation)
        self.assertIn("action_server_.reset()", implementation)
        self.assertIn("add_new_trajectory_msg", implementation)
        self.assertIn("GuardedJointTrajectoryController", plugins.read_text())
        loader = ROOT / "build_dummy/dummy_ros_control/guarded_plugin_load_tests"
        self.assertTrue(loader.is_file(), "pluginlib load probe was not built")
        environment = dict(__import__("os").environ)
        prefix = ROOT / "install_dummy/dummy_ros_control"
        environment["AMENT_PREFIX_PATH"] = f"{prefix}:/opt/ros/jazzy"
        environment["LD_LIBRARY_PATH"] = f"{prefix}/lib:/opt/ros/jazzy/lib"
        completed = subprocess.run(
            [str(loader)], cwd=ROOT, env=environment,
            capture_output=True, text=True, check=False
        )
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)


if __name__ == "__main__":
    unittest.main()
