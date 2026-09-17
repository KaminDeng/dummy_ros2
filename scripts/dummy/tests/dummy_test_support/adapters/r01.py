"""R01 model-contract adapter using the real model validation tests."""

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import run_unittest_case


TARGETS = {
    "GEN07": "scripts.dummy.tests.test_r01_contract.R01ContractTest.test_wire_v2_mirror_matches_current_dpsdk_release",
    "MOD01": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_joint1_bench_mapping_applies_anchor_once_and_round_trips",
    "MOD02": "src.dummy_moveit_config.test.test_reference_chain_fk.ReferenceChainFkTest.test_fixed_and_random_evidence_match_kdl",
    "MOD03": "src.dummy_moveit_config.test.test_reference_chain_fk.ReferenceChainFkTest.test_literal_legacy_axis3_row_diverges_from_actual_fk",
    "MOD04": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_joint6_preserves_three_pi_and_rejects_five_pi_without_wrapping",
    "MOD05": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_all_zero_pose_is_rejected_by_joint3_contract_range",
    "MOD06": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_mesh_bounds_match_transformed_collision_boxes",
}
SOURCES = (
    "scripts/dummy/tests/test_r01_contract.py",
    "src/dummy_moveit_config/test/test_model_alignment.py",
    "src/dummy_moveit_config/test/test_reference_chain_fk.py",
    "contracts/dummy_robot_v2/model/model.yaml",
)


def run_r01_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R01 case: {case_id}") from error
    return run_unittest_case(case_id, stimulus, target, SOURCES)
