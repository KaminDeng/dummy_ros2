"""R06 adapter combining the shared DPSDK core with ROS-owned gates."""

from __future__ import annotations

from hashlib import sha256
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[5]
DPSDK_ROOT = Path(os.environ.get("DPSDK_ROOT") or ROOT.parent / "dpsdk").resolve()
CORE_CASES = {
    "EN01", "EN02", "EN03", "EN04", "EN05", "MOV01", "MOV02", "MOV03",
    "MOV04", "MOV05", "MOV06", "MOV07", "MOV16", "MOV17", "STOP01",
    "STOP02", "STOP09",
}
LOCAL_CASES = {
    "FRE05": "update_generation_tests",
    "MOV13": "guarded_entry_tests",
    "MOV14": "guarded_entry_tests",
    "MOV15": "native_action_gate_tests",
}
ACTION_CASES = {"ROS06"}
ENGINE_FIELDS = {
    "MOV03": ("engine_busy_rejected", "busy_rejected"),
    "MOV04": ("engine_completion_barrier", "completion_barrier"),
    "MOV16": ("engine_prime_before_result", "prime_before_result"),
    "MOV17": ("engine_frozen_seq", "complete_seq"),
    "STOP01": ("engine_cancel_barrier", "cancel_barrier"),
    "STOP02": ("engine_stop_timeout_aborted", "stop_timeout_aborted"),
    "STOP09": ("engine_late_generation_accepted", "late_generation_accepted"),
}
_ACTION_OBSERVATIONS: dict[str, Any] | None = None
_DEVICE_E2E_OBSERVATIONS: dict[str, Any] | None = None


def _load_upstream_adapter(name: str):
    path = DPSDK_ROOT / "tools/dummy/tests/dummy_test_support/adapters" / f"{name}.py"
    if not path.is_file():
        raise RuntimeError(f"DPSDK_ROOT must contain the published {name} adapter: {path}")
    spec = importlib.util.spec_from_file_location(f"dpsdk_{name}_adapter", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load upstream adapter: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _core_case(case_id: str) -> dict[str, Any]:
    suite = "d04" if case_id == "AUTH02" else "d05"
    upstream = _load_upstream_adapter(suite).run_case(
        case_id, {"case_id": case_id, "hardware": "none", "test_only": True}
    )
    values = dict(upstream["observations"])
    passed = bool(values.pop("case_passed", True))
    values["case_id"] = case_id
    values["requirement_observed"] = passed and bool(values.get("requirement_observed", True))
    values["evidence_scope"] = "RobotCore component; no cross-process device action"
    if case_id in ENGINE_FIELDS or case_id == "EN05":
        binary = "wire_management_tests" if case_id == "EN05" else "device_action_engine_tests"
        extra = _component(binary, [])
        if case_id == "EN05":
            values["wire_handshake_executed"] = extra["observations"]["two_stream_handshake"]
        else:
            destination, source = ENGINE_FIELDS[case_id]
            values[destination] = extra["observations"][source]
        if case_id == "STOP09":
            values["engine_abort_error_preserved"] = extra["observations"]["abort_error_preserved"]
        upstream = {**upstream,
                    "sut_source_sha256": sha256((upstream["sut_source_sha256"] + extra["sut_source_sha256"]).encode()).hexdigest(),
                    "trace_sha256": sha256((upstream["trace_sha256"] + extra["trace_sha256"]).encode()).hexdigest()}
    if case_id == "MOV04":
        completion = _component("complete_motion_tests", [])
        values.update(completion["observations"])
        upstream = {**upstream,
                    "sut_source_sha256": sha256((upstream["sut_source_sha256"] + completion["sut_source_sha256"]).encode()).hexdigest(),
                    "trace_sha256": sha256((upstream["trace_sha256"] + completion["trace_sha256"]).encode()).hexdigest()}
    return {**upstream, "observations": values}


def _component(name: str, arguments: list[str]) -> dict[str, Any]:
    codec = DPSDK_ROOT / "contracts/dummy_robot_v1/codec/dp_dummy_robot_v1.hpp"
    if not codec.is_file():
        raise RuntimeError(f"DPSDK_ROOT must contain the published manage_v1 codec: {codec}")
    binary = ROOT / "build_dummy/dummy_motion_client" / name
    if not binary.is_file():
        raise RuntimeError(f"R06 ROS SUT binary is missing: {binary}")
    run = subprocess.run(
        ["bash", "-c", 'source /opt/ros/jazzy/setup.bash && source "$1/install_dummy/setup.bash" && shift && exec "$@"',
         "r06-component", str(ROOT), str(binary), *arguments],
        cwd=ROOT, capture_output=True, text=True, check=False, timeout=20)
    if run.returncode:
        raise RuntimeError(f"{binary.name} failed ({run.returncode}):\n{run.stdout}{run.stderr}")
    sources = [*sorted((ROOT / "src/dummy_motion_client").rglob("*.cpp")),
               *sorted((ROOT / "src/dummy_motion_client").rglob("*.hpp")),
               ROOT / "src/dummy_ros_control/src/guarded_joint_trajectory_controller.cpp",
               ROOT / "src/dummy_ros_control/src/command_channel.cpp",
               codec]
    digest = sha256()
    for source in sources:
        if source.is_file():
            digest.update(source.read_bytes())
    trace = run.stdout + run.stderr + f"returncode={run.returncode}"
    return {
        "executed": True,
        "sut_source_sha256": digest.hexdigest(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": json.loads(next(line for line in reversed(run.stdout.splitlines()) if line.startswith("{"))),
    }


def _local_case(case_id: str) -> dict[str, Any]:
    result = _component(LOCAL_CASES[case_id], [case_id] if case_id in {"MOV13", "MOV14"} else [])
    return {**result, "observations": {**result["observations"], "case_id": case_id,
            "component": LOCAL_CASES[case_id], "requirement_observed": True,
            "evidence_scope": "ROS component; no cross-process device action"}}


def _action_case(case_id: str) -> dict[str, Any]:
    global _ACTION_OBSERVATIONS
    probe = ROOT / "src/dummy_bringup/scripts/action_gate_probe.py"
    if _ACTION_OBSERVATIONS is None:
        # The shared probe's final target is -0.1 rad. Establish a different
        # starting pose through the same public action so repeat runs still
        # measure real motion; never write simulator joint state directly.
        prepare = subprocess.run(
            ["/usr/bin/python3", "-c", """
import importlib.util, sys
spec = importlib.util.spec_from_file_location('r06_native_probe', sys.argv[1])
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
probe.rclpy.init()
node = probe.Node('r06_native_precondition')
client = probe.ActionClient(node, probe.FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
try:
    if not client.wait_for_server(timeout_sec=10):
        raise RuntimeError('public action server unavailable')
    handle = probe.wait(node, client.send_goal_async(probe.goal(0.1, 2)))
    if not handle.accepted or probe.wait(node, handle.get_result_async()).status != 4:
        raise RuntimeError('native precondition action failed')
finally:
    client.destroy()
    node.destroy_node()
    probe.rclpy.shutdown()
""", str(probe)], cwd=ROOT, capture_output=True, text=True, check=False, timeout=30)
        if prepare.returncode:
            if "public action server unavailable" in prepare.stderr:
                trace = prepare.stdout + prepare.stderr
                return {
                    "case_id": case_id, "executed": False, "status": "NOT_RUN",
                    "sut_kind": "ros_action", "reason": "Live native public action server unavailable",
                    "sut_source_sha256": sha256(probe.read_bytes()).hexdigest(),
                    "trace_sha256": sha256(trace.encode()).hexdigest(), "_trace_text": trace,
                    "observations": {"case_id": case_id, "requirement_observed": False,
                                     "evidence_scope": "native ROS action runtime unavailable"},
                }
            raise RuntimeError(f"native action precondition failed:\n{prepare.stdout}{prepare.stderr}")
        with tempfile.TemporaryDirectory(prefix="r06_action_") as temporary:
            report = Path(temporary) / "observations.json"
            run = subprocess.run(
                ["/usr/bin/python3", str(probe), "--report", str(report)], cwd=ROOT,
                capture_output=True, text=True, check=False, timeout=45,
            )
            if run.returncode or not report.is_file():
                raise RuntimeError(f"live action probe failed ({run.returncode}):\n{run.stdout}{run.stderr}")
            _ACTION_OBSERVATIONS = json.loads(report.read_text())
    values = _ACTION_OBSERVATIONS
    observed = (
        values["out_of_bounds_goal_accepted"] is False
        if case_id == "MOV15"
        else values["first_goal_accepted"] and not values["second_goal_accepted"]
        and values["third_goal_accepted"]
    )
    trace = json.dumps(values, sort_keys=True)
    return {
        "case_id": case_id,
        "executed": True,
        "status": "PASS" if observed else "FAIL", "sut_kind": "ros_action",
        "reason": "" if observed else "Native action lifecycle observation failed",
        "sut_source_sha256": sha256(probe.read_bytes() +
            (ROOT / "src/dummy_motion_client/src/native_action_gate.cpp").read_bytes() +
            (ROOT / "src/dummy_motion_client/src/native_goal_validation.cpp").read_bytes()).hexdigest(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": {
            **values,
            "case_id": case_id,
            "requirement_observed": bool(observed),
            "action_trace": values,
            "first_goal_accepted": values["first_goal_accepted"],
            "second_goal_accepted": values["second_goal_accepted"],
            "third_goal_accepted": values["third_goal_accepted"],
            "evidence_scope": "live native ROS action; no DPSDK device session",
        },
    }


def _device_not_run(case_id: str, component: dict[str, Any]) -> dict[str, Any]:
    e2e = _device_e2e_component()
    values = component["observations"]
    return {
        "case_id": case_id, "executed": False, "status": "NOT_RUN", "sut_kind": "ros_action",
        "reason": "Production bridge passes a C05 synthetic peer only; committed DPSDK operation enums drift from wire-v1 and no committed authorized positive peer exists",
        "sut_source_sha256": component["sut_source_sha256"],
        "trace_sha256": component["trace_sha256"],
        "component_evidence": component,
        "component_observations": values,
        "synthetic_e2e_observations": e2e,
        "observations": {"case_id": case_id, "requirement_observed": False,
                         "evidence_scope": values["evidence_scope"]},
    }


def _device_e2e_component() -> dict[str, Any]:
    global _DEVICE_E2E_OBSERVATIONS
    if _DEVICE_E2E_OBSERVATIONS is not None:
        return _DEVICE_E2E_OBSERVATIONS
    script = ROOT / "src/dummy_motion_client/test/device_action_e2e_probe.py"
    gate = ROOT / "build_dummy/dummy_motion_client/device_action_gate"
    peer = ROOT / "build_dummy/dummy_motion_client/c05_action_peer"
    bundle = ROOT / "contracts/dummy_robot_v1"
    completed = subprocess.run(
        ["/usr/bin/python3", str(script), str(gate), str(peer), str(bundle)],
        cwd=ROOT, capture_output=True, text=True, check=False, timeout=60,
    )
    if completed.returncode != 0:
        _DEVICE_E2E_OBSERVATIONS = {
            "evidence_scope": "component_only", "executed": False,
            "reason": (completed.stdout + completed.stderr)[-2000:],
        }
        return _DEVICE_E2E_OBSERVATIONS
    for line in reversed(completed.stdout.splitlines()):
        try:
            observed = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(observed, dict) and observed.get("evidence_scope") == "component_only":
            _DEVICE_E2E_OBSERVATIONS = {**observed, "executed": True}
            return _DEVICE_E2E_OBSERVATIONS
    _DEVICE_E2E_OBSERVATIONS = {
        "evidence_scope": "component_only", "executed": False,
        "reason": "synthetic E2E emitted no machine observation",
    }
    return _DEVICE_E2E_OBSERVATIONS


def run_r06_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("R06 stimulus is invalid")
    if case_id == "AUTH02" or case_id in CORE_CASES:
        return _device_not_run(case_id, _core_case(case_id))
    if case_id in LOCAL_CASES:
        return _device_not_run(case_id, _local_case(case_id))
    if case_id in ACTION_CASES:
        return _action_case(case_id)
    raise ValueError(f"unsupported R06 case: {case_id}")
