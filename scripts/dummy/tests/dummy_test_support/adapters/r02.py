"""R02 Gazebo adapter gated by the real public trajectory action."""

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import run_action_probe_case, run_unittest_case


TARGETS = {
    "MOD05": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_all_zero_pose_is_rejected_by_joint3_contract_range",
    "FRE11": "src.dummy_moveit_config.test.test_r02_gazebo_contract.R02GazeboContractTest.test_gazebo_nodes_use_sim_time_and_do_not_start_device_transport",
    "ROS03": "src.dummy_moveit_config.test.test_r02_gazebo_contract.R02GazeboContractTest.test_collision_geometry_uses_gazebo_supported_positive_boxes",
    "ROS11": "src.dummy_moveit_config.test.test_r02_gazebo_contract.R02GazeboContractTest.test_bringup_installs_canonical_gazebo_entrypoint",
    "ROS01": "src/dummy_bringup/scripts/action_gate_probe.py",
    "ROS02": "src/dummy_bringup/scripts/action_gate_probe.py",
}
SOURCES = (
    "src/dummy_bringup/launch/gazebo_6dof.launch.py",
    "src/dummy_bringup/test/test_gazebo_runtime.launch.py",
    "src/dummy_moveit_config/test/test_r02_gazebo_contract.py",
    "src/dummy_bringup/scripts/action_gate_probe.py",
)


def run_r02_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if case_id in {"ROS01", "ROS02"}:
        return run_action_probe_case(case_id, stimulus, TARGETS[case_id], SOURCES)
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R02 case: {case_id}") from error
    return run_unittest_case(case_id, stimulus, target, SOURCES)
