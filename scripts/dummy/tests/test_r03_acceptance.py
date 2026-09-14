#!/usr/bin/env python3
"""Build and execute the ROS-owned R03 codec and goal-gate probes."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
DPSDK = Path("../dpsdk")


class R03AcceptanceTest(unittest.TestCase):
    def compile_and_run(self, source: Path) -> None:
        self.assertTrue(source.is_file(), f"missing R03 SUT: {source}")
        with tempfile.TemporaryDirectory(prefix="dummy_r03_") as temporary:
            binary = Path(temporary) / source.stem
            completed = subprocess.run(
                [
                    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "src/dummy_ros_control/include"),
                    "-I", str(DPSDK / "libraries/dp_protocol/generated/include"),
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

    def test_mov14_unregistered_goal_and_wrong_digest_are_rejected(self):
        self.compile_and_run(ROOT / "src/dummy_ros_control/test/goal_gate_tests.cpp")


if __name__ == "__main__":
    unittest.main()
