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
DPSDK_ROOT = Path(os.environ.get("DPSDK_ROOT", "../dpsdk"))
CORE_CASES = {
    "EN01", "EN02", "EN03", "EN04", "EN05", "MOV01", "MOV02", "MOV03",
    "MOV04", "MOV05", "MOV06", "MOV07", "MOV16", "MOV17", "STOP01",
    "STOP02", "STOP09",
}
LOCAL_CASES = {
    "FRE05": "command_channel_tests",
    "MOV13": "goal_gate_tests",
    "MOV14": "goal_gate_tests",
}
ACTION_CASES = {"MOV15", "ROS06"}
_ACTION_OBSERVATIONS: dict[str, Any] | None = None


def _load_upstream_adapter(name: str):
    path = DPSDK_ROOT / "tools/dummy/tests/dummy_test_support/adapters" / f"{name}.py"
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
    return {**upstream, "observations": values}


def _local_case(case_id: str) -> dict[str, Any]:
    binary = ROOT / "build_dummy/dummy_ros_control" / LOCAL_CASES[case_id]
    if not binary.is_file():
        raise RuntimeError(f"R06 ROS SUT binary is missing: {binary}")
    run = subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False)
    if run.returncode:
        raise RuntimeError(f"{binary.name} failed ({run.returncode}):\n{run.stdout}{run.stderr}")
    sources = [
        ROOT / "src/dummy_ros_control/test" / f"{LOCAL_CASES[case_id].removesuffix('_tests')}.cpp",
        ROOT / "src/dummy_ros_control/include/dummy_ros_control/goal_registration_gate.hpp",
        ROOT / "src/dummy_ros_control/src/command_channel.cpp",
    ]
    digest = sha256()
    for source in sources:
        if source.is_file():
            digest.update(source.read_bytes())
    trace = run.stdout + run.stderr + f"returncode={run.returncode}"
    return {
        "executed": True,
        "sut_source_sha256": digest.hexdigest(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": {
            "case_id": case_id,
            "component": binary.name,
            "return_code": run.returncode,
            "requirement_observed": True,
        },
    }


def _action_case(case_id: str) -> dict[str, Any]:
    global _ACTION_OBSERVATIONS
    probe = ROOT / "src/dummy_bringup/scripts/action_gate_probe.py"
    if _ACTION_OBSERVATIONS is None:
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
        "executed": True,
        "sut_source_sha256": sha256((probe.read_bytes() + trace.encode())).hexdigest(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": {
            "case_id": case_id,
            "requirement_observed": bool(observed),
            "action_trace": values,
        },
    }


def run_r06_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("R06 stimulus is invalid")
    if case_id == "AUTH02" or case_id in CORE_CASES:
        return _core_case(case_id)
    if case_id in LOCAL_CASES:
        return _local_case(case_id)
    if case_id in ACTION_CASES:
        return _action_case(case_id)
    raise ValueError(f"unsupported R06 case: {case_id}")
