"""R08 release artifact adapter."""

from __future__ import annotations

from hashlib import sha256
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[5]
FILES = (
    ROOT / "docs/dummy/versions.lock",
    ROOT / "docs/dummy/ros-runbook.md",
    ROOT / "docs/dummy/backend-matrix.md",
    ROOT / "docs/dummy/STATUS.md",
    ROOT / "docs/dummy/task-reports/R06.json",
    ROOT / "docs/dummy/task-reports/R07.json",
)


def run_r08_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if case_id != "REL01" or stimulus != {
        "case_id": "REL01", "hardware": "none", "test_only": True,
    }:
        raise ValueError("R08 stimulus is invalid")
    missing = [str(path) for path in FILES if not path.is_file()]
    if missing:
        raise RuntimeError(f"release artifacts missing: {missing}")
    lock, runbook, matrix, status, r06, r07 = (path.read_text() for path in FILES)
    source_lock_present = all(
        key in lock for key in (
            "ros_workspace_base_commit=", "dpsdk_commit=", "wire_payload_sorted_sha256=",
            "gazebo_sim=", "moveit=", "joint_trajectory_controller=",
        )
    )
    runtime_not_run_as_pass = (
        "| Hardware/J1 | PASS" in status or "| `real_j1`" not in matrix
        or "| NOT_RUN |" not in matrix
    )
    separated = all(
        marker in runbook for marker in ("simulation evidence only", "SIL uses", "`NOT_RUN`")
    ) and '"status": "PASS"' in r06 and '"status": "PASS"' in r07
    digest = sha256()
    for path in FILES:
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(path.read_bytes())
    trace = (
        f"source_lock_present={source_lock_present};"
        f"runtime_not_run_as_pass={runtime_not_run_as_pass};separated={separated}"
    )
    return {
        "case_id": case_id,
        "executed": True,
        "sut_source_sha256": digest.hexdigest(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": {
            "case_id": case_id,
            "source_lock_present": source_lock_present,
            "runtime_not_run_as_pass": runtime_not_run_as_pass,
            "ros_gazebo_record_separate": separated,
            "requirement_observed": source_lock_present and not runtime_not_run_as_pass and separated,
        },
    }
