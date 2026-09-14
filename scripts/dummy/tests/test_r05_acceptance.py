"""Runtime acceptance for the actual MoveIt planning services."""

import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
PROBE = ROOT / "src/dummy_bringup/scripts/planning_probe.py"
ACTION_PROBE = ROOT / "src/dummy_bringup/scripts/action_gate_probe.py"


class R05PlanningAcceptance(unittest.TestCase):
    def test_public_action_gate_rejects_busy_then_allows_goal_after_cancel(self):
        self.assertTrue(ACTION_PROBE.is_file(), "R05 action gate probe is missing")
        with tempfile.TemporaryDirectory(prefix="r05_action_") as temporary:
            report = Path(temporary) / "observations.json"
            result = subprocess.run(
                ["/usr/bin/python3", str(ACTION_PROBE), "--report", str(report)],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=45,
            )
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            observed = json.loads(report.read_text())
            self.assertTrue(observed["first_goal_accepted"])
            self.assertFalse(observed["second_goal_accepted"])
            self.assertEqual(5, observed["first_goal_terminal_status"])
            self.assertTrue(observed["third_goal_accepted"])
            self.assertEqual(4, observed["third_goal_terminal_status"])
            self.assertFalse(observed["out_of_bounds_goal_accepted"])

    def test_contract_bounds_preserve_joint6_multiturn_and_reject_joint3_zero(self):
        self.assertTrue(PROBE.is_file(), "R05 MoveIt service probe is missing")
        sys.path.insert(0, str(PROBE.parent))
        from planning_probe import validate_joint_target

        self.assertTrue(validate_joint_target((0, 0, math.pi / 2, 0, 0, 3 * math.pi)))
        self.assertFalse(validate_joint_target((0, 0, math.pi / 2, 0, 0, 5 * math.pi)))
        self.assertFalse(validate_joint_target((0, 0, 0, 0, 0, 0)))
        self.assertFalse(validate_joint_target((math.nan, 0, math.pi / 2, 0, 0, 0)))

    def test_real_moveit_plans_and_rejects_invalid_paths(self):
        self.assertTrue(PROBE.is_file(), "R05 MoveIt service probe is missing")
        with tempfile.TemporaryDirectory(prefix="r05_planning_") as temporary:
            report = Path(temporary) / "observations.json"
            result = subprocess.run(
                ["/usr/bin/python3", str(PROBE), "--report", str(report)],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=45,
            )
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertTrue(report.is_file(), "probe produced no runtime evidence")
            observed = json.loads(report.read_text())
            self.assertGreater(observed["valid_joint_trajectory_points"], 1)
            self.assertEqual(1, observed["valid_joint_plan_error_code"])
            self.assertTrue(observed["out_of_bounds_rejected_by_guard"])
            self.assertFalse(observed["out_of_bounds_service_sent"])
            self.assertLess(observed["unreachable_cartesian_fraction"], 1.0)
            self.assertFalse(observed["trajectory_executed"])
            self.assertTrue(observed["barrier_collision_state_found"])
            self.assertNotEqual(1, observed["barrier_collision_goal_error_code"])
            self.assertEqual(1, observed["pose_ik_error_code"])
            self.assertTrue(observed["pose_ik_solution_within_contract_limits"])


if __name__ == "__main__":
    unittest.main()
