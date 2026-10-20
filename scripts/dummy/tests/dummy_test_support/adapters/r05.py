"""R05 MoveIt/action adapter which only runs against a live action server."""
##
# @file r05.py
# @brief MoveIt/公开Action实际探针与模型组件混合的R05适配器。
#
# MOD06运行离线mesh目标；ROS02/03/11复用R02；ROS04/05/06先核公开Action唯一server再运行规划或Action脚本。规划探针要求未执行轨迹，Action探针会提交/取消目标；hardware=none不等于无ROS请求。

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import run_action_probe_case, run_unittest_case
from dummy_test_support.adapters.r02 import run_r02_case


PROBES = {
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


##
# @brief 选对应probe/validator；ROS04核场景与障碍，ROS05核部分笛卡尔且未执行，ROS06核忙拒绝/取消/恢复及J1运动。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 该suite所选执行路径返回的证据字典，状态/范围不自动升级。
def run_r05_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if case_id == "MOD06":
        return run_unittest_case(
            case_id, stimulus,
            "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_mesh_bounds_match_transformed_collision_boxes",
            SOURCES, sut_kind="core_binary",
        )
    if case_id in {"ROS02", "ROS03", "ROS11"}:
        return run_r02_case(case_id, stimulus)
    validators = {
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
            and observed.get("third_goal_terminal_status") == 4
            and observed.get("out_of_bounds_goal_accepted") is False
            and observed.get("joint_state_sample_count", 0) > 2
            and observed.get("joint1_position_change_rad", 0.0) > 0.02
        ),
    }
    try:
        return run_action_probe_case(
            case_id, stimulus, PROBES[case_id], SOURCES, validator=validators[case_id],
            runtime_action="/arm_controller/follow_joint_trajectory",
        )
    except KeyError as error:
        raise ValueError(f"unsupported R05 case: {case_id}") from error
