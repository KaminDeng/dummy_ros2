"""R07 backend and SIL fault acceptance."""

import json
from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


ROOT = Path(__file__).resolve().parents[3]
CASES = json.loads((ROOT / "scripts/dummy/tests/fixtures/R07.json").read_text())["cases"]


class R07Acceptance(unittest.TestCase):
    pass


def _make_case(case_id: str):
    def test(self):
        observed = run_case("R07", case_id, {
            "case_id": case_id, "hardware": "none", "test_only": True,
        })
        self.assertEqual("PASS", observed["status"])
        self.assertIsInstance(observed["reason"], str)
        self.assertTrue(observed["sut_source_sha256"])
        self.assertTrue(observed["trace_sha256"])
        values = observed["observations"]
        self.assertEqual(case_id, values["case_id"])
        self.assertFalse(values.get("real_hardware_accessed", False))
        if observed["status"] == "PASS":
            self.assertIs(observed["executed"], True)
            self.assertTrue(values["requirement_observed"])
        else:
            self.assertIs(observed["executed"], False)
            self.assertTrue(observed["reason"])
            self.assertTrue(values["blocked"])

        required = {
            "B02": ("posix_peer_started", "ros_device_chain_started"),
            "AUTH04": ("auto_management_calls", "posix_peer_started"),
            "FRE11": ("steady_clock_source", "simulation_clock_isolated"),
            "SEN05": ("robot_core", "virtual_can", "configured_mask"),
            "SEN06": ("robot_core", "virtual_can", "configured_mask"),
            "ROS07": ("stale_read_result",),
            "ROS08": ("management_while_inactive",),
            "ROS10": ("controller_state_interfaces", "controller_command_interfaces"),
        }[case_id]
        for field in required:
            self.assertIn(field, values)
        if case_id in {"B02", "AUTH04", "SEN05", "SEN06"}:
            self.assertIn(values["upstream_subtest_status"], {"PASS", "FAIL"})
            self.assertTrue(values["upstream_trace_sha256"])
        if case_id == "AUTH04":
            self.assertEqual(0, values["auto_management_calls"])
        if case_id == "FRE11" and observed["status"] == "PASS":
            self.assertEqual("steady", values["steady_clock_source"])
            self.assertIs(values["simulation_clock_isolated"], True)
        if case_id == "ROS07" and observed["status"] == "PASS":
            self.assertEqual("ERROR_WHEN_ACTIVE", values["stale_read_result"])
            self.assertEqual("OK", values["inactive_poll_result"])
        if case_id == "ROS10" and observed["status"] == "PASS":
            self.assertEqual(["Joint1/position", "Joint1/velocity"], values["controller_state_interfaces"])
            self.assertEqual(["Joint1/position"], values["controller_command_interfaces"])

    return test


for _case_id in CASES:
    setattr(R07Acceptance, f"test_{_case_id.lower()}", _make_case(_case_id))


if __name__ == "__main__":
    unittest.main()
