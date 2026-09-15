"""R06 device action lifecycle acceptance against ROS and the shared DPSDK core."""

import json
from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


ROOT = Path(__file__).resolve().parents[3]
CASES = json.loads((ROOT / "scripts/dummy/tests/fixtures/R06.json").read_text())["cases"]


class R06Acceptance(unittest.TestCase):
    pass


def _make_case(case_id: str):
    def test(self):
        observed = run_case("R06", case_id, {
            "case_id": case_id,
            "hardware": "none",
            "test_only": True,
        })
        self.assertIs(observed["executed"], True)
        self.assertTrue(observed["sut_source_sha256"])
        self.assertTrue(observed["trace_sha256"])
        values = observed["observations"]
        self.assertEqual(case_id, values["case_id"])
        self.assertTrue(values["requirement_observed"])
        self.assertNotIn("case_passed", values)

    return test


for _case_id in CASES:
    setattr(R06Acceptance, f"test_{_case_id.lower()}", _make_case(_case_id))


if __name__ == "__main__":
    unittest.main()
