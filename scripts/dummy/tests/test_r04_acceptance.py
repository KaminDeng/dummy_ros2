"""Compile and execute R04 nonblocking producer/state acceptance tests."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]


class R04AcceptanceTest(unittest.TestCase):
    def compile_run(self, source_name: str) -> None:
        source = ROOT / "src/dummy_ros_control/test" / source_name
        self.assertTrue(source.is_file(), f"missing R04 test SUT: {source}")
        with tempfile.TemporaryDirectory(prefix="r04_") as temporary:
            binary = Path(temporary) / source.stem
            completed = subprocess.run(
                ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
                 "-I", str(ROOT / "src/dummy_ros_control/include"), str(source), "-o", str(binary)],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            self.assertEqual(0, completed.returncode, completed.stderr)
            executed = subprocess.run([str(binary)], capture_output=True, text=True, check=False)
            self.assertEqual(0, executed.returncode, executed.stdout + executed.stderr)

    def test_fre04_fre05_and_fre06_generation_identity(self):
        self.compile_run("producer_tests.cpp")
        binary = ROOT / "build_dummy/dummy_ros_control/command_channel_tests"
        self.assertTrue(binary.is_file(), "shared command channel test was not built")
        completed = subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False)
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)

    def test_rpc09_fre09_fre10_and_ros07_state_validation(self):
        self.compile_run("stale_state_tests.cpp")

    def test_ros09_bounded_publisher_backpressure(self):
        for name in ("nonblocking_tests", "dp_comm_worker_tests"):
            binary = ROOT / "build_dummy/dummy_ros_control" / name
            self.assertTrue(binary.is_file(), f"{name} was not built")
            completed = subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False)
            self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)

    def test_guarded_system_is_exported_without_implicit_authorization(self):
        header = ROOT / "src/dummy_ros_control/include/dummy_ros_control/guarded_zenbedded_system.hpp"
        source = ROOT / "src/dummy_ros_control/src/guarded_zenbedded_system.cpp"
        plugins = ROOT / "src/dummy_ros_control/dummy_ros_control_plugins.xml"
        self.assertTrue(header.is_file())
        self.assertTrue(source.is_file())
        implementation = source.read_text()
        self.assertIn("hardware_interface::SystemInterface", header.read_text())
        self.assertIn("GuardedZenbeddedSystem", plugins.read_text())
        for forbidden in ("Grant", "Capture", "EnableHold", "async_request", "spin_some"):
            self.assertNotIn(forbidden, implementation)
        controller = (
            ROOT / "src/dummy_ros_control/src/guarded_joint_trajectory_controller.cpp"
        ).read_text()
        self.assertIn("JointTrajectoryController::update", controller)
        self.assertIn("update_sink_", controller)
        write_body = implementation.split("GuardedZenbeddedSystem::write", 1)[1].split(
            "GuardedZenbeddedSystem::ingest_state", 1
        )[0]
        self.assertNotIn("record_update", write_body)
        binary = ROOT / "build_dummy/dummy_ros_control/wire_state_ingest_tests"
        self.assertTrue(binary.is_file(), "wire-state ingest test was not built")
        completed = subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False)
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)


if __name__ == "__main__":
    unittest.main()
