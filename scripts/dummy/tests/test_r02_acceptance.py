"""R02 only reports runtime behavior from a corresponding live probe."""

from pathlib import Path
import sys
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

    def test_runtime_cases_are_not_claimed_from_static_launch_checks(self):
        for case_id in ("FRE11", "ROS01", "ROS02", "ROS03", "ROS11"):
            result = self.evidence(case_id)
            self.assertEqual("NOT_RUN", result["status"])
            self.assertFalse(result["executed"])
            self.assertTrue(result["reason"])


if __name__ == "__main__":
    unittest.main()
