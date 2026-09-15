"""R05 MoveIt/action adapter which only runs against a live action server."""

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import not_run_case, run_action_probe_case, run_unittest_case


PROBES = {
    "ROS02": "src/dummy_bringup/scripts/action_gate_probe.py",
    "ROS03": "src/dummy_bringup/scripts/planning_probe.py",
    "ROS04": "src/dummy_bringup/scripts/planning_probe.py",
    "ROS05": "src/dummy_bringup/scripts/planning_probe.py",
    "ROS06": "src/dummy_bringup/scripts/action_gate_probe.py",
}
SOURCES = (
    "scripts/dummy/tests/test_r05_acceptance.py",
    "src/dummy_bringup/scripts/action_gate_probe.py",
    "src/dummy_bringup/scripts/planning_probe.py",
    "src/dummy_moveit_config/test/test_r05_action_gate_contract.py",
    "src/dummy_moveit_config/test/test_model_alignment.py",
)


def run_r05_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if case_id == "MOD06":
        return run_unittest_case(
            case_id, stimulus,
            "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_mesh_bounds_match_transformed_collision_boxes",
            SOURCES, sut_kind="core_binary",
        )
    if case_id == "ROS11":
        return not_run_case(
            case_id, stimulus, SOURCES,
            "no installed-controller load probe verifies the locked-version cancellation configuration",
        )
    validators = {
        "ROS02": lambda observed: (
            observed.get("first_goal_accepted") is True
            and observed.get("third_goal_accepted") is True
            and observed.get("joint_feedback_changed") is True
        ),
        "ROS03": lambda observed: (
            observed.get("valid_joint_trajectory_points", 0) > 1
            and observed.get("valid_joint_plan_error_code") == 1
            and observed.get("trajectory_executed") is False
        ),
        "ROS04": lambda observed: (
            observed.get("planning_scene_matches_canonical") is True
            and observed.get("world_scene_revision_matches") is True
            and observed.get("barrier_collision_state_found") is True
        ),
        "ROS05": lambda observed: (
            observed.get("unreachable_cartesian_fraction", 1.0) < 1.0
            and observed.get("trajectory_executed") is False
        ),
        "ROS06": lambda observed: (
            observed.get("first_goal_accepted") is True
            and observed.get("second_goal_accepted") is False
            and observed.get("first_goal_terminal_status") == 5
            and observed.get("third_goal_accepted") is True
        ),
    }
    try:
        return run_action_probe_case(
            case_id, stimulus, PROBES[case_id], SOURCES, validator=validators[case_id],
            runtime_action="/arm_controller/follow_joint_trajectory",
        )
    except KeyError as error:
        raise ValueError(f"unsupported R05 case: {case_id}") from error
