"""R08 release evidence acceptance."""

from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


class R08Acceptance(unittest.TestCase):
    def test_rel01(self):
        observed = run_case("R08", "REL01", {
            "case_id": "REL01", "hardware": "none", "test_only": True,
        })
        self.assertIs(observed["executed"], True)
        self.assertTrue(observed["sut_source_sha256"])
        self.assertTrue(observed["trace_sha256"])
        values = observed["observations"]
        self.assertTrue(values["source_lock_present"])
        self.assertFalse(values["runtime_not_run_as_pass"])
        self.assertTrue(values["ros_gazebo_record_separate"])
        self.assertTrue(values["requirement_observed"])


if __name__ == "__main__":
    unittest.main()
