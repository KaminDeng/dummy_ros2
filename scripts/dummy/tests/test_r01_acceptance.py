"""R01 adapter evidence is tied to the real model and FK probes."""

from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


class R01Acceptance(unittest.TestCase):
    def evidence(self, case_id: str) -> dict:
        return run_case("R01", case_id, {
            "case_id": case_id, "hardware": "none", "test_only": True,
        })

    def test_mod01_uses_the_joint1_anchor_round_trip_probe(self):
        result = self.evidence("MOD01")
        self.assertEqual("PASS", result["status"], result["reason"])
        self.assertIn("joint1_bench_mapping", result["observations"]["case_target"])

    def test_mod02_uses_reference_chain_fk_samples(self):
        result = self.evidence("MOD02")
        self.assertEqual("PASS", result["status"], result["reason"])
        self.assertIn("fixed_and_random_evidence_match_kdl", result["observations"]["case_target"])

    def test_mod03_uses_the_actual_legacy_chain_divergence_probe(self):
        result = self.evidence("MOD03")
        self.assertEqual("PASS", result["status"], result["reason"])
        self.assertIn("literal_legacy_axis3_row_diverges", result["observations"]["case_target"])

    def test_mod04_and_mod06_are_not_claimed_without_their_specific_probe(self):
        for case_id in ("MOD04", "MOD06"):
            result = self.evidence(case_id)
            self.assertEqual("NOT_RUN", result["status"])
            self.assertFalse(result["executed"])


if __name__ == "__main__":
    unittest.main()
