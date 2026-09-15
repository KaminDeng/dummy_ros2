"""R06 device action lifecycle acceptance against ROS and the shared DPSDK core."""

import json
from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


ROOT = Path(__file__).resolve().parents[3]
FIXTURE = json.loads((ROOT / "scripts/dummy/tests/fixtures/R06.json").read_text())
CASES = FIXTURE["cases"]


class R06Acceptance(unittest.TestCase):
    pass


def _make_case(case_id: str):
    def test(self):
        observed = run_case("R06", case_id, {
            "case_id": case_id,
            "hardware": "none",
            "test_only": True,
        })
        self.assertTrue(observed["sut_source_sha256"])
        self.assertTrue(observed["trace_sha256"])
        values = observed["observations"]
        self.assertEqual(case_id, values["case_id"])
        self.assertNotIn("case_passed", values)
        if case_id != "ROS06":
            self.assertIs(observed["executed"], False)
            self.assertEqual("NOT_RUN", observed["status"])
            self.assertIs(values["requirement_observed"], False)
            self.assertIn("no cross-process", values["evidence_scope"])
            self.assertTrue(observed["reason"])
            component = observed["component_evidence"]
            self.assertIs(component["executed"], True)
            values = observed["component_observations"]
        elif not observed["executed"]:
            self.assertEqual("NOT_RUN", observed["status"])
            self.assertIs(values["requirement_observed"], False)
            self.assertTrue(observed["reason"])
            return
        else:
            self.assertIs(observed["executed"], True)
            self.assertEqual("PASS", observed["status"])
            self.assertTrue(values["requirement_observed"])
        for key, expected in FIXTURE["expected_observations"][case_id].items():
            self.assertIn(key, values, f"{case_id} must report {key}")
            self.assertEqual(expected, values[key], f"{case_id}: {key}")
        if case_id == "ROS06":
            self.assertGreater(values["joint_state_sample_count"], 2)
            self.assertGreater(values["joint1_position_change_rad"], 0.02)

    return test


for _case_id in CASES:
    setattr(R06Acceptance, f"test_{_case_id.lower()}", _make_case(_case_id))


if __name__ == "__main__":
    unittest.main()
