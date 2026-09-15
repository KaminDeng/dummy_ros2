"""R02 Gazebo adapter gated by the real public trajectory action."""

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import not_run_case, run_unittest_case


TARGETS = {
    "MOD05": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_all_zero_pose_is_rejected_by_joint3_contract_range",
}
SOURCES = (
    "src/dummy_bringup/launch/gazebo_6dof.launch.py",
    "src/dummy_bringup/test/test_gazebo_runtime.launch.py",
    "src/dummy_moveit_config/test/test_r02_gazebo_contract.py",
    "src/dummy_bringup/scripts/action_gate_probe.py",
)


def run_r02_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    unavailable = {
        "FRE11": "no pause/resume Gazebo clock probe also observes independent device watchdog behavior",
        "ROS01": "no case-specific headless Gazebo probe observes spawn, controller count, and device isolation",
        "ROS02": "no case-specific Gazebo trajectory probe observes time-varying joint feedback and tracking error",
        "ROS03": "no case-specific no-command Gazebo probe observes fixed-base gravity stability",
        "ROS11": "no case-specific controller load probe validates this installed JTC configuration",
    }
    if case_id in unavailable:
        return not_run_case(case_id, stimulus, SOURCES, unavailable[case_id], sut_kind="gazebo_runtime")
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R02 case: {case_id}") from error
    return run_unittest_case(case_id, stimulus, target, SOURCES)
