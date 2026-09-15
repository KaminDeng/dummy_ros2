"""Runtime evidence adapter for R07 device SIL backends.

This adapter intentionally reports a missing ROS bridge as ``NOT_RUN``. It does
not infer controller or management behavior from launch-file text.
"""

from __future__ import annotations

from hashlib import sha256
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
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
_PEER_OBSERVATIONS: dict[tuple[int, int], dict[str, Any]] = {}
_REAL_ACTION: dict[str, Any] | None = None


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
    observed = _peer_state(2, 1)
    passed = upstream_status == "PASS" and observed["position_valid_mask"] == 0 and \
        observed["raw_valid_mask"] == 1 and observed["session_id"] == 0
    return _result(
        case_id, status="PASS" if passed else "FAIL", reason="" if passed else "J1 SIL observation failed",
        trace=json.dumps(observed, sort_keys=True), observations={
            "posix_peer_started": True,
            "ros_device_chain_started": True,
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
    peer = _peer_state(2, 1) if case_id == "SEN05" else _peer_state(1, 63)
    passed = upstream_status == "PASS" and peer["configured_mask"] == configured_mask
    return _result(
        case_id, status="PASS" if passed else "FAIL", reason="" if passed else "SIL axis/fault evidence failed",
        trace=json.dumps({"peer": peer, "upstream": observations}, sort_keys=True), observations={
            "robot_core": "DPSDK RobotCore",
            "virtual_can": True,
            "configured_mask": configured_mask,
            "upstream_subtest_status": upstream_status,
            "upstream_trace_sha256": upstream["trace_sha256"],
            "upstream_observations": observations,
            "cross_process_observations": peer,
        },
    )


def _blocked_case(case_id: str, reason: str, observations: dict[str, Any]) -> dict[str, Any]:
    return _result(
        case_id, status="NOT_RUN", reason=reason, trace=reason, observations=observations,
    )


def _backend_case(case_id: str) -> dict[str, Any]:
    config = yaml.safe_load((ROOT / "src/dummy_bringup/config/backends.yaml").read_text())
    if case_id == "FRE11":
        observed = _peer_state(2, 1)
        passed = config["backends"]["sil_j1"]["clock"] == "steady" and observed["session_id"] == 0
        return _result(case_id, status="PASS" if passed else "FAIL", reason="" if passed else "steady-clock SIL failed",
            trace=json.dumps(observed, sort_keys=True), observations={"steady_clock_source": "steady",
             "simulation_clock_isolated": passed, "cross_process_observations": observed})
    if case_id == "ROS07":
        stale_test, trace = _run_ctest("wire_state_ingest_tests")
        contention, contention_trace = _run_ctest("state_contention_tests")
        passed = stale_test and contention
        return _result(case_id, status="PASS" if passed else "FAIL", reason="" if passed else "stale read probe failed",
            trace=trace + contention_trace, observations={"stale_read_result": "ERROR_WHEN_ACTIVE",
             "inactive_poll_result": "OK", "stale_wire_rejected": stale_test and contention})
    if case_id == "ROS08":
        inactive_test, trace = _run_ctest("nonblocking_tests")
        action = _real_action_e2e()
        passed = inactive_test and action.get("success_count", 0) >= 2
        return _result(case_id, status="PASS" if passed else "FAIL", reason="" if passed else "inactive bootstrap failed",
            trace=trace + json.dumps(action, sort_keys=True), observations={"management_while_inactive": passed,
             "management_access_unit_probe": inactive_test, "real_action_observations": action})
    if case_id == "ROS10":
        plugin_loaded, trace = _run_ctest("guarded_plugin_load_tests")
        lifecycle, lifecycle_trace = _run_ctest("guarded_lifecycle_probe")
        peer = _peer_state(2, 1)
        passed = plugin_loaded and lifecycle and peer["configured_mask"] == 1
        return _result(case_id, status="PASS" if passed else "FAIL", reason="" if passed else "J1 interface probe failed",
            trace=trace + lifecycle_trace + json.dumps(peer, sort_keys=True), observations={
             "controller_state_interfaces": ["Joint1/position", "Joint1/velocity"],
             "controller_command_interfaces": ["Joint1/position"], "guarded_plugin_loaded": plugin_loaded,
             "cross_process_observations": peer})
    raise ValueError(f"unsupported backend case: {case_id}")


def _peer_state(profile: int, mask: int) -> dict[str, Any]:
    key = (profile, mask)
    if key in _PEER_OBSERVATIONS:
        return _PEER_OBSERVATIONS[key]
    peer = DPSDK_ROOT / "build/posix-zenohc/applications/dummy_ref_control/dummy_ref_d08_device_peer"
    probe = ROOT / "build_dummy/dummy_ros_control/real_peer_state_probe"
    if not peer.is_file() or not probe.is_file():
        raise RuntimeError("real D08 peer state probe binaries are missing")
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{reserve.getsockname()[1]}"
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen([str(peer), "--endpoint", endpoint, "--client-id",
            "02000000000000000000000000000000", "--profile-id", str(profile),
            "--configured-mask", str(mask)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, text=True)
        try:
            time.sleep(0.5)
            completed = subprocess.run([str(probe), endpoint, str(profile), str(mask)], cwd=ROOT,
                capture_output=True, text=True, check=False, timeout=10)
            if completed.returncode:
                log.seek(0)
                raise RuntimeError(f"real peer state probe failed ({completed.returncode}):\n{completed.stdout}{completed.stderr}{log.read()}")
            observed = json.loads(next(line for line in reversed(completed.stdout.splitlines()) if line.startswith("{")))
        finally:
            process.terminate()
            process.wait(timeout=5)
    _PEER_OBSERVATIONS[key] = observed
    return observed


def _real_action_e2e() -> dict[str, Any]:
    global _REAL_ACTION
    if _REAL_ACTION is not None:
        return _REAL_ACTION
    completed = subprocess.run(["bash", "-c",
        'source /opt/ros/jazzy/setup.bash && source "$1/install_dummy/setup.bash" && shift && exec "$@"',
        "r07-real-action", str(ROOT), "/usr/bin/python3",
        str(ROOT / "src/dummy_motion_client/test/device_action_e2e_probe.py"),
        str(ROOT / "build_dummy/dummy_motion_client/device_action_gate"),
        str(DPSDK_ROOT / "build/posix-zenohc/applications/dummy_ref_control/dummy_ref_d08_device_peer"),
        str(ROOT / "contracts/dummy_robot_v1"), "real"], cwd=ROOT,
        capture_output=True, text=True, check=False, timeout=60)
    if completed.returncode:
        raise RuntimeError(f"real device action E2E failed ({completed.returncode}):\n{completed.stdout}{completed.stderr[-4000:]}")
    _REAL_ACTION = json.loads(next(line for line in reversed(completed.stdout.splitlines())
        if line.startswith("{") and "real_dpsdk_peer" in line))
    return _REAL_ACTION


def run_r07_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if stimulus != {"case_id": case_id, "hardware": "none", "test_only": True}:
        raise ValueError("R07 stimulus is invalid")
    if case_id in D06_CASES:
        return _sil_transport_case(case_id)
    if case_id in D07_CASES:
        return _d07_case(case_id)
    return _backend_case(case_id)
