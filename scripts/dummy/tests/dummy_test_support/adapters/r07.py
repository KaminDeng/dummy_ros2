"""Runtime evidence adapter for R07 device SIL backends.

This adapter intentionally reports a missing ROS bridge as ``NOT_RUN``. It does
not infer controller or management behavior from launch-file text.
"""

from __future__ import annotations

from hashlib import sha256
import importlib.util
import os
from pathlib import Path
import subprocess
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[5]
DPSDK_ROOT = Path(os.environ.get("DPSDK_ROOT", "../dpsdk"))
D06_CASES = {"B02": "B02", "AUTH04": "AUTH04"}
D07_CASES = {"SEN05", "SEN06"}
BACKEND_SOURCES = (
    ROOT / "src/dummy_bringup/config/backends.yaml",
    ROOT / "src/dummy_bringup/launch/sil_6dof.launch.py",
    ROOT / "src/dummy_bringup/launch/sil_j1.launch.py",
    ROOT / "src/dummy_bringup/launch/real_j1.launch.py",
    ROOT / "src/dummy_ros_control/test/guarded_lifecycle_probe.cpp",
    ROOT / "src/dummy_ros_control/test/wire_state_ingest_tests.cpp",
    ROOT / "src/dummy_ros_control/test/nonblocking_tests.cpp",
)


def _load(name: str):
    path = DPSDK_ROOT / "tools/dummy/tests/dummy_test_support/adapters" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"dpsdk_{name}_r07", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load upstream adapter: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _source_hash() -> str:
    digest = sha256()
    for path in BACKEND_SOURCES:
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(path.read_bytes() if path.is_file() else b"<missing>")
    return digest.hexdigest()


def _result(
    case_id: str, *, status: str, reason: str, trace: str, observations: dict[str, Any],
) -> dict[str, Any]:
    values = {
        "case_id": case_id,
        "requirement_observed": status == "PASS",
        "real_hardware_accessed": False,
        "blocked": status == "NOT_RUN",
        **observations,
    }
    return {
        "case_id": case_id,
        "executed": status == "PASS",
        "status": status,
        "reason": reason,
        "sut_source_sha256": _source_hash(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": values,
    }


def _run_ctest(name: str) -> tuple[bool, str]:
    completed = subprocess.run(
        ["ctest", "--test-dir", str(ROOT / "build_dummy/dummy_ros_control"),
         "-R", f"^{name}$", "--output-on-failure"],
        cwd=ROOT, capture_output=True, text=True, check=False, timeout=30,
    )
    return completed.returncode == 0, completed.stdout + completed.stderr


def _upstream(case_id: str) -> dict[str, Any]:
    suite = "d07" if case_id in D07_CASES else "d06"
    upstream_case = case_id if suite == "d07" else D06_CASES[case_id]
    return _load(suite).run_case(
        upstream_case, {"case_id": upstream_case, "hardware": "none", "test_only": True},
    )


def _sil_transport_case(case_id: str) -> dict[str, Any]:
    upstream = _upstream(case_id)
    upstream_observations = dict(upstream["observations"])
    upstream_status = "PASS" if (
        upstream.get("executed") is True and upstream_observations.get("requirement_observed", True)
    ) else "FAIL"
    return _blocked_case(
        case_id,
        "BLOCKED: the production bridge and D08 long-lived peer exist, but D08 status omits runtime contract/model/profile hashes required for one trusted ROS device chain",
        observations={
            "posix_peer_started": False,
            "ros_device_chain_started": False,
            "auto_management_calls": 0,
            "upstream_case_id": D06_CASES[case_id],
            "upstream_subtest_status": upstream_status,
            "upstream_trace_sha256": upstream["trace_sha256"],
            "upstream_transport_observations": upstream.get("transport_observations", {}),
        },
    )


def _d07_case(case_id: str) -> dict[str, Any]:
    upstream = _upstream(case_id)
    observations = dict(upstream["observations"])
    configured_mask = 1 if case_id == "SEN05" else 63
    upstream_status = "PASS" if (
        upstream.get("executed") is True and observations.get("requirement_observed", True)
    ) else "FAIL"
    return _blocked_case(
        case_id,
        "BLOCKED: DPSDK 2716594 provides J1/six-axis long-lived transport, but its runtime status omits contract/model/profile hashes and the D07 fault scenario is not the same ROS bridge process",
        observations={
            "robot_core": "DPSDK RobotCore",
            "virtual_can": True,
            "configured_mask": configured_mask,
            "upstream_subtest_status": upstream_status,
            "upstream_trace_sha256": upstream["trace_sha256"],
            "upstream_observations": observations,
        },
    )


def _blocked_case(case_id: str, reason: str, observations: dict[str, Any]) -> dict[str, Any]:
    return _result(
        case_id, status="NOT_RUN", reason=reason, trace=reason, observations=observations,
    )


def _backend_case(case_id: str) -> dict[str, Any]:
    config = yaml.safe_load((ROOT / "src/dummy_bringup/config/backends.yaml").read_text())
    if case_id == "FRE11":
        return _blocked_case(
            case_id,
            "BLOCKED: the production bridge exists, but the committed D08 peer omits runtime hash/authorization evidence required before its steady watchdog can be combined with ROS observations",
            {"steady_clock_source": config["backends"]["sil_j1"]["clock"],
             "simulation_clock_isolated": False},
        )
    if case_id == "ROS07":
        stale_test, trace = _run_ctest("wire_state_ingest_tests")
        return _blocked_case(
            case_id,
            "BLOCKED: the committed probe rejects stale wire state but does not expose a live "
            "GuardedSystem read() call after that rejection",
            {"stale_read_result": "NOT_RUN", "stale_wire_rejected": stale_test,
             "guarded_probe_trace_sha256": sha256(trace.encode()).hexdigest()},
        )
    if case_id == "ROS08":
        inactive_test, trace = _run_ctest("nonblocking_tests")
        return _blocked_case(
            case_id,
            "BLOCKED: the bridge supports inactive management bootstrap, but the D08 peer is rejected before mutation because required runtime hash evidence is absent",
            {"management_while_inactive": "NOT_RUN", "management_access_unit_probe": inactive_test,
             "guarded_probe_trace_sha256": sha256(trace.encode()).hexdigest()},
        )
    if case_id == "ROS10":
        plugin_loaded, trace = _run_ctest("guarded_plugin_load_tests")
        return _blocked_case(
            case_id,
            "BLOCKED: the embedded controller manager exposes J1 interfaces in component E2E, but the real D08 peer fails strict runtime identity before this can become real-peer evidence",
            {"controller_state_interfaces": [], "controller_command_interfaces": [],
             "guarded_plugin_loaded": plugin_loaded,
             "guarded_probe_trace_sha256": sha256(trace.encode()).hexdigest()},
        )
    raise ValueError(f"unsupported backend case: {case_id}")


def run_r07_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if stimulus != {"case_id": case_id, "hardware": "none", "test_only": True}:
        raise ValueError("R07 stimulus is invalid")
    if case_id in D06_CASES:
        return _sil_transport_case(case_id)
    if case_id in D07_CASES:
        return _d07_case(case_id)
    return _backend_case(case_id)
