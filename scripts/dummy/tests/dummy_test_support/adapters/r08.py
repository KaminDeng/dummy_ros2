"""R08 release artifact adapter."""

from __future__ import annotations

from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
from typing import Any


ROOT = Path(__file__).resolve().parents[5]
DPSDK_ROOT = Path(os.environ.get("DPSDK_ROOT", ROOT.parent / "dpsdk")).resolve()
FILES = (
    ROOT / "docs/dummy/versions.lock",
    ROOT / "docs/dummy/ros-runbook.md",
    ROOT / "docs/dummy/backend-matrix.md",
    ROOT / "docs/dummy/STATUS.md",
    ROOT / "docs/dummy/task-reports/R06.json",
    ROOT / "docs/dummy/task-reports/R07.json",
)


def _lock_values(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in text.splitlines():
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key] = value
    return result


def _git(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(DPSDK_ROOT), *arguments], capture_output=True, text=True, check=False,
    )


def _runtime_reports_current() -> bool:
    reports = [ROOT / "build_dummy/reports/R06.json", ROOT / "build_dummy/reports/R07.json"]
    sources = [ROOT / "contracts/dummy_robot_v2/manifest.json"]
    for directory in (ROOT / "src/dummy_motion_client", ROOT / "src/dummy_ros_control"):
        sources.extend(path for path in directory.rglob("*") if path.is_file())
    newest_source = max(path.stat().st_mtime_ns for path in sources)
    for report in reports:
        if not report.is_file() or report.stat().st_mtime_ns < newest_source:
            return False
        try:
            payload = json.loads(report.read_text())
        except (OSError, json.JSONDecodeError):
            return False
        cases = payload.get("cases")
        if payload.get("status") != "PASS" or not isinstance(cases, list) or not cases or any(
            case.get("status") != "PASS" or case.get("executed") is not True for case in cases
        ):
            return False
    return True


def _dpsdk_peer_current() -> bool:
    peer = DPSDK_ROOT / "build/posix-zenohc/applications/dummy_ref_control/dummy_ref_d08_device_peer"
    if not peer.is_file():
        return False
    inputs = [DPSDK_ROOT / "contracts/dummy_robot_v2/manifest.json"]
    for directory in (
        DPSDK_ROOT / "applications/dummy_ref_control",
        DPSDK_ROOT / "libraries/dp_protocol/generated",
    ):
        inputs.extend(path for path in directory.rglob("*") if path.is_file())
    inputs.append(DPSDK_ROOT / "config/products/posix_demo.cmake")
    return peer.stat().st_mtime_ns >= max(path.stat().st_mtime_ns for path in inputs)


def run_r08_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if case_id != "REL01" or stimulus != {
        "case_id": "REL01", "hardware": "none", "test_only": True,
    }:
        raise ValueError("R08 stimulus is invalid")
    missing = [str(path) for path in FILES if not path.is_file()]
    if missing:
        raise RuntimeError(f"release artifacts missing: {missing}")
    lock, runbook, matrix, status, r06, r07 = (path.read_text() for path in FILES)
    locks = _lock_values(lock)
    source_lock_present = all(
        key in lock for key in (
            "ros_workspace_base_commit=", "dpsdk_commit=", "wire_payload_sorted_sha256=",
            "gazebo_sim=", "moveit=", "joint_trajectory_controller=",
        )
    )
    ros_base = locks.get("ros_workspace_base_commit", "")
    source_lock_present = source_lock_present and subprocess.run(
        ["git", "-C", str(ROOT), "merge-base", "--is-ancestor", ros_base, "HEAD"],
        capture_output=True, check=False,
    ).returncode == 0
    runtime_not_run_as_pass = (
        "| Hardware/J1 | PASS" in status or "| 实体硬件/J1 | PASS" in status
        or "| `real_j1`" not in matrix
        or "| NOT_RUN |" not in matrix
    )
    simulation_boundary = "simulation evidence only" in runbook or "只产生仿真证据" in runbook
    sil_boundary = "SIL uses" in runbook or "两个 SIL 入口仅使用" in runbook
    separated = simulation_boundary and sil_boundary and "`NOT_RUN`" in runbook
    ros_manifest = json.loads((ROOT / "contracts/dummy_robot_v2/manifest.json").read_text())
    dpsdk_manifest = json.loads((DPSDK_ROOT / "contracts/dummy_robot_v2/manifest.json").read_text())
    wire_lock_matches_bundle = (
        locks.get("wire_release") == ros_manifest.get("release_id") == "dummy-wire-v2"
        and locks.get("wire_payload_sorted_sha256") == ros_manifest.get("payload_sorted_sha256")
        and ros_manifest == dpsdk_manifest
    )
    head = _git("rev-parse", "HEAD")
    dpsdk_commit_matches = head.returncode == 0 and locks.get("dpsdk_commit") == head.stdout.strip()
    unstaged = _git("diff", "--quiet", "--", "contracts/dummy_robot_v2")
    staged = _git("diff", "--cached", "--quiet", "--", "contracts/dummy_robot_v2")
    dpsdk_contract_clean = unstaged.returncode == 0 and staged.returncode == 0
    dpsdk_peer_current = _dpsdk_peer_current()
    runtime_reports_current = _runtime_reports_current()
    digest = sha256()
    for path in FILES:
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(path.read_bytes())
    trace = (
        f"source_lock_present={source_lock_present};"
        f"runtime_not_run_as_pass={runtime_not_run_as_pass};separated={separated};"
        f"wire_lock_matches_bundle={wire_lock_matches_bundle};"
        f"dpsdk_commit_matches={dpsdk_commit_matches};dpsdk_contract_clean={dpsdk_contract_clean};"
        f"dpsdk_peer_current={dpsdk_peer_current};runtime_reports_current={runtime_reports_current}"
    )
    passed = source_lock_present and not runtime_not_run_as_pass and separated and \
        wire_lock_matches_bundle and dpsdk_commit_matches and dpsdk_contract_clean and \
        dpsdk_peer_current and runtime_reports_current
    return {
        "case_id": case_id,
        "executed": True,
        "status": "PASS" if passed else "FAIL",
        "reason": "" if passed else "release lock or evidence separation validation failed",
        "sut_source_sha256": digest.hexdigest(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": {
            "case_id": case_id,
            "source_lock_present": source_lock_present,
            "runtime_not_run_as_pass": runtime_not_run_as_pass,
            "ros_gazebo_record_separate": separated,
            "wire_lock_matches_bundle": wire_lock_matches_bundle,
            "dpsdk_commit_matches": dpsdk_commit_matches,
            "dpsdk_contract_clean": dpsdk_contract_clean,
            "dpsdk_peer_current": dpsdk_peer_current,
            "runtime_reports_current": runtime_reports_current,
            "requirement_observed": passed,
        },
    }
