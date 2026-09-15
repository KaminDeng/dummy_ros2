"""R01 case-specific model acceptance."""

import json
from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


CASES = json.loads((TEST_ROOT / "fixtures/R01.json").read_text())["cases"]


class R01Acceptance(unittest.TestCase):
    pass


def _make_case(case_id: str):
    def test(self):
        result = run_case("R01", case_id, {
            "case_id": case_id, "hardware": "none", "test_only": True,
        })
        self.assertEqual(case_id, result["case_id"])
        self.assertTrue(result["executed"])
        self.assertEqual("PASS", result["status"], result["reason"])
        self.assertTrue(result["sut_source_sha256"])
        self.assertTrue(result["trace_sha256"])
        self.assertEqual(case_id, result["observations"]["case_id"])

    return test


for _case_id in CASES:
    setattr(R01Acceptance, f"test_{_case_id.lower()}", _make_case(_case_id))


if __name__ == "__main__":
    unittest.main()
