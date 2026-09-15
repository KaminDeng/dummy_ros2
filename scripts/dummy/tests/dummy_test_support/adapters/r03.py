"""R03 wire and RPC adapter using the compiled probes."""

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import not_run_case, run_unittest_case


TARGETS = {
    "GEN01": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_gen01_gen03_and_rpc_payloads_use_actual_generated_codecs",
    "GEN02": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_gen01_gen03_and_rpc_payloads_use_actual_generated_codecs",
    "GEN03": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_gen01_gen03_and_rpc_payloads_use_actual_generated_codecs",
    "RPC01": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_rpc01_and_rpc08_use_real_dp_comm_transport_and_handles",
    "RPC08": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_rpc01_and_rpc08_use_real_dp_comm_transport_and_handles",
    "MOV14": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_mov14_unregistered_goal_and_wrong_digest_are_rejected",
    "RPC12": "scripts.dummy.tests.test_r03_acceptance.R03AcceptanceTest.test_guarded_jtc_plugin_closes_base_action_and_topic_inputs",
}
SOURCES = (
    "scripts/dummy/tests/test_r03_acceptance.py",
    "scripts/dummy/r03_rpc_probe.py",
    "src/dummy_ros_control/test/wire_probe_tests.cpp",
    "src/dummy_ros_control/test/goal_gate_tests.cpp",
)


def run_r03_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if case_id == "RPC12":
        return not_run_case(
            case_id, stimulus, SOURCES,
            "the available codec probe lacks a case-specific caller/owner/session/q snapshot and stale-result probe",
            sut_kind="codec_generator",
        )
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R03 case: {case_id}") from error
    return run_unittest_case(case_id, stimulus, target, SOURCES)
