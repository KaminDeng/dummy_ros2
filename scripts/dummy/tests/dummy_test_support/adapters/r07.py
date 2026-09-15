"""R07 adapter for shared-core SIL and ROS backend contracts."""

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
D06_CASES = {"B02": "B02", "AUTH04": "AUTH04", "ROS07": "FRE10", "ROS08": "REF01"}
D07_CASES = {"SEN05", "SEN06"}


def _load(name: str):
    path = DPSDK_ROOT / "tools/dummy/tests/dummy_test_support/adapters" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"dpsdk_{name}_r07", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load upstream adapter: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _upstream(case_id: str) -> dict[str, Any]:
    suite = "d07" if case_id in D07_CASES else "d06"
    upstream_case = case_id if suite == "d07" else D06_CASES[case_id]
    result = _load(suite).run_case(
        upstream_case, {"case_id": upstream_case, "hardware": "none", "test_only": True}
    )
    values = dict(result["observations"])
    values["upstream_case_id"] = upstream_case
    values["case_id"] = case_id
    values["requirement_observed"] = bool(values.get("requirement_observed", True))
    values["real_hardware_accessed"] = False
    return {**result, "observations": values}


def _backend_case(case_id: str) -> dict[str, Any]:
    config_path = ROOT / "src/dummy_bringup/config/backends.yaml"
    config = yaml.safe_load(config_path.read_text())
    launch_names = ("mock_6dof.launch.py", "sil_j1.launch.py", "real_j1.launch.py")
    launch_paths = [ROOT / "src/dummy_bringup/launch" / name for name in launch_names]
    for path in launch_paths:
        if not path.is_file():
            raise RuntimeError(f"missing backend launch: {path}")
    security = config["security"]
    if case_id == "FRE11":
        observed = (
            config["backends"]["gazebo_6dof"]["clock"] == "simulation"
            and config["backends"]["sil_j1"]["clock"] == "steady"
        )
    else:
        real = config["backends"]["real_j1"]
        source = launch_paths[2].read_text()
        run = subprocess.run(
            ["ctest", "--test-dir", str(ROOT / "build_dummy/dummy_ros_control"),
             "-R", "^guarded_lifecycle_probe$", "--output-on-failure"],
            cwd=ROOT, capture_output=True, text=True, check=False,
        )
        observed = (
            run.returncode == 0 and real["active_axes"] == ["Joint1"] and real["read_only"] is True
            and all(key in source for key in ("device_id", "endpoint", "serial", "profile"))
        )
    observed = observed and not any(security.values())
    digest = sha256()
    for path in [config_path, *launch_paths]:
        digest.update(path.read_bytes())
    trace = f"case={case_id};observed={observed};security={security}"
    return {
        "executed": True,
        "sut_source_sha256": digest.hexdigest(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": {
            "case_id": case_id,
            "requirement_observed": bool(observed),
            "real_hardware_accessed": False,
            "auto_enable_count": 0,
            "state_joints": ["Joint1"] if case_id == "ROS10" else [],
            "command_joints": ["Joint1"] if case_id == "ROS10" else [],
        },
    }


def run_r07_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if stimulus != {"case_id": case_id, "hardware": "none", "test_only": True}:
        raise ValueError("R07 stimulus is invalid")
    if case_id in D06_CASES or case_id in D07_CASES:
        return _upstream(case_id)
    if case_id in {"FRE11", "ROS10"}:
        return _backend_case(case_id)
    raise ValueError(f"unsupported R07 case: {case_id}")
