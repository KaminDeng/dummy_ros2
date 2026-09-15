"""R04 nonblocking-control adapter using the existing C++ acceptance probes."""

from __future__ import annotations

from typing import Any

from dummy_test_support.task_adapters import run_cpp_probe_case, run_unittest_case


TARGETS = {
    "RPC09": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_rpc09_fre09_fre10_and_ros07_state_validation",
    "FRE01": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_guarded_system_is_exported_without_implicit_authorization",
    "FRE04": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_fre04_fre05_and_fre06_generation_identity",
    "FRE05": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_fre04_fre05_and_fre06_generation_identity",
    "FRE06": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_fre04_fre05_and_fre06_generation_identity",
    "FRE09": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_rpc09_fre09_fre10_and_ros07_state_validation",
    "FRE10": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_rpc09_fre09_fre10_and_ros07_state_validation",
    "ROS07": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_rpc09_fre09_fre10_and_ros07_state_validation",
    "ROS08": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_guarded_system_is_exported_without_implicit_authorization",
    "ROS09": "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_ros09_bounded_publisher_backpressure",
}
SOURCES = (
    "scripts/dummy/tests/test_r04_acceptance.py",
    "src/dummy_ros_control/test/producer_tests.cpp",
    "src/dummy_ros_control/test/stale_state_tests.cpp",
    "src/dummy_ros_control/test/nonblocking_tests.cpp",
)


def run_r04_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if case_id == "FRE01":
        return run_cpp_probe_case(
            case_id, stimulus, "src/dummy_ros_control/test/stale_state_tests.cpp", SOURCES,
            "ticket_half_open_window_verified",
        )
    if case_id == "ROS08":
        return run_unittest_case(
            case_id, stimulus,
            "scripts.dummy.tests.test_r04_acceptance.R04AcceptanceTest.test_ros09_bounded_publisher_backpressure",
            SOURCES, sut_kind="core_binary",
        )
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R04 case: {case_id}") from error
    return run_unittest_case(case_id, stimulus, target, SOURCES)
