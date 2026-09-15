"""R01 model-contract adapter using the real model validation tests."""

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import run_unittest_case


TARGETS = {
    "GEN07": "scripts.dummy.tests.test_r01_contract.R01ContractTest.test_model_bundle_is_complete_and_hash_verified",
    "MOD01": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_urdf_limits_match_contract_without_wrapping_joint6",
    "MOD02": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_r01_tests_are_registered_with_ament",
    "MOD03": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_initial_positions_are_complete_and_within_contract_limits",
    "MOD04": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_joint1_bench_mapping_applies_anchor_once_and_round_trips",
    "MOD05": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_all_zero_pose_is_rejected_by_joint3_contract_range",
    "MOD06": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_all_srdf_group_states_are_complete_and_within_limits",
}
SOURCES = (
    "scripts/dummy/tests/test_r01_contract.py",
    "src/dummy_moveit_config/test/test_model_alignment.py",
    "contracts/dummy_robot_v1/model/model.yaml",
)


def run_r01_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R01 case: {case_id}") from error
    return run_unittest_case(case_id, stimulus, target, SOURCES)
