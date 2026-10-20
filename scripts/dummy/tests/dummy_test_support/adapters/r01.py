"""R01 model-contract adapter using the real model validation tests."""
##
# @file r01.py
# @brief 把R01模型/合同case映射到具体unittest目标的适配器。
#
# 实际启动目标测试进程，MOD02/03固定/usr/bin/python3，其余用当前解释器；证据来自这些离线模型/FK断言，不是新机械臂测量。TARGETS含MOD04/MOD06具体目标，当前实现不会因为旧验收期望而主动返回NOT_RUN。

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


##
# @brief 查TARGETS后调用run_unittest_case；未知case转ValueError，选定解释器仅改变子测试运行环境。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 该suite所选执行路径返回的证据字典，状态/范围不自动升级。
def run_r01_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R01 case: {case_id}") from error
    python_executable = "/usr/bin/python3" if case_id in {"MOD02", "MOD03"} else None
    return run_unittest_case(
        case_id, stimulus, target, SOURCES, python_executable=python_executable)
