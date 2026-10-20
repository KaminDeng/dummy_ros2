"""R03 wire and RPC adapter using the compiled probes."""
##
# @file r03.py
# @brief R03 codec/目标门/RPC case到验收目标的映射。
#
# 经run_unittest_case启动真实Python目标；目标有g++临时编译/组件执行、跨进程dp_comm与插件加载，不全是静态检查。sut_kind默认document_utility是标签默认值，不能把它当实际执行层级保证。

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import run_unittest_case


TARGETS = {
    "GEN01": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_gen01_gen03_and_rpc_payloads_use_actual_generated_codecs",
    "GEN02": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_gen01_gen03_and_rpc_payloads_use_actual_generated_codecs",
    "GEN03": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_gen01_gen03_and_rpc_payloads_use_actual_generated_codecs",
    "RPC01": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_rpc01_and_rpc08_use_real_dp_comm_transport_and_handles",
    "RPC08": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_rpc01_and_rpc08_use_real_dp_comm_transport_and_handles",
    "MOV14": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_mov14_unregistered_goal_and_wrong_digest_are_rejected",
    "RPC12": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_rpc12_preserves_caller_owner_and_session_snapshot",
}
SOURCES = (
    "scripts/dummy/tests/test_r03_acceptance.py",
    "scripts/dummy/r03_rpc_probe.py",
    "src/dummy_ros_control/test/wire_probe_tests.cpp",
    "src/dummy_ros_control/test/goal_gate_tests.cpp",
)


##
# @brief 查映射后运行选定验收方法；多个case可能复用同一方法，不能把映射条数当独立探针次数。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 该suite所选执行路径返回的证据字典，状态/范围不自动升级。
def run_r03_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R03 case: {case_id}") from error
    return run_unittest_case(case_id, stimulus, target, SOURCES)
