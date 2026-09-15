"""R05 MoveIt/action adapter which only runs against a live action server."""

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import run_unittest_case


TARGETS = {
    "MOD06": "scripts.dummy.tests.test_r05_acceptance.R05PlanningAcceptance.test_contract_bounds_preserve_joint6_multiturn_and_reject_joint3_zero",
    "ROS02": "scripts.dummy.tests.test_r05_acceptance.R05PlanningAcceptance.test_public_action_gate_rejects_busy_then_allows_goal_after_cancel",
    "ROS03": "scripts.dummy.tests.test_r05_acceptance.R05PlanningAcceptance.test_real_moveit_plans_and_rejects_invalid_paths",
    "ROS04": "scripts.dummy.tests.test_r05_acceptance.R05PlanningAcceptance.test_real_moveit_plans_and_rejects_invalid_paths",
    "ROS05": "scripts.dummy.tests.test_r05_acceptance.R05PlanningAcceptance.test_real_moveit_plans_and_rejects_invalid_paths",
    "ROS06": "scripts.dummy.tests.test_r05_acceptance.R05PlanningAcceptance.test_public_action_gate_rejects_busy_then_allows_goal_after_cancel",
    "ROS11": "scripts.dummy.tests.test_r05_acceptance.R05PlanningAcceptance.test_real_moveit_plans_and_rejects_invalid_paths",
}
SOURCES = (
    "scripts/dummy/tests/test_r05_acceptance.py",
    "src/dummy_bringup/scripts/action_gate_probe.py",
    "src/dummy_bringup/scripts/planning_probe.py",
)


def run_r05_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R05 case: {case_id}") from error
    return run_unittest_case(
        case_id, stimulus, target, SOURCES, runtime_action="/arm_controller/follow_joint_trajectory"
    )
