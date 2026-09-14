"""Compile and execute the R06 action lifecycle coordinator tests."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]


class R06AcceptanceTest(unittest.TestCase):
    def compile_run(self, source_name: str) -> None:
        source = ROOT / "src/dummy_motion_client/test" / source_name
        self.assertTrue(source.is_file(), f"missing R06 SUT: {source}")
        with tempfile.TemporaryDirectory(prefix="r06_") as temporary:
            binary = Path(temporary) / source.stem
            completed = subprocess.run(
                ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                 "-I", str(ROOT / "src/dummy_motion_client/include"), str(source), "-o", str(binary)],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            self.assertEqual(0, completed.returncode, completed.stderr)
            executed = subprocess.run([str(binary)], capture_output=True, text=True, check=False)
            self.assertEqual(0, executed.returncode, executed.stdout + executed.stderr)

    def test_enable_begin_complete_and_second_goal(self):
        self.compile_run("session_action_tests.cpp")

    def test_cancel_abort_and_late_result_barriers(self):
        self.compile_run("cancel_barrier_tests.cpp")

    def test_management_receipt_and_operation_result_are_distinct(self):
        self.compile_run("management_client_tests.cpp")


if __name__ == "__main__":
    unittest.main()
