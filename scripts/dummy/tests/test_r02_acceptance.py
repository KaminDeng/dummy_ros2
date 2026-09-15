"""R02 native Gazebo case acceptance with honest runtime absence."""

import json
from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


CASES = json.loads((TEST_ROOT / "fixtures/R02.json").read_text())["cases"]
RUNTIME_CASES = {"ROS01", "ROS02"}


class R02Acceptance(unittest.TestCase):
    pass


def _make_case(case_id: str):
    def test(self):
        result = run_case("R02", case_id, {
            "case_id": case_id, "hardware": "none", "test_only": True,
        })
        self.assertEqual(case_id, result["case_id"])
        self.assertTrue(result["sut_source_sha256"])
        self.assertTrue(result["trace_sha256"])
        if case_id in RUNTIME_CASES and result["status"] == "NOT_RUN":
            self.assertFalse(result["executed"])
            self.assertTrue(result["reason"])
        else:
            self.assertTrue(result["executed"])
            self.assertEqual("PASS", result["status"], result["reason"])

    return test


for _case_id in CASES:
    setattr(R02Acceptance, f"test_{_case_id.lower()}", _make_case(_case_id))


if __name__ == "__main__":
    unittest.main()
