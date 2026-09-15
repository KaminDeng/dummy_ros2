"""Dispatch test cases to their suite-specific adapters."""

from __future__ import annotations

from hashlib import sha256
import json
import re
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[4]


def _source_evidence(source_paths: tuple[str, ...]) -> tuple[str, list[str]]:
    digest = sha256()
    missing: list[str] = []
    for relative in source_paths:
        path = ROOT / relative
        digest.update(relative.encode())
        if path.is_file():
            digest.update(path.read_bytes())
        else:
            digest.update(b"<missing>")
            missing.append(relative)
    return digest.hexdigest(), missing


def _missing_source_result(
    case_id: str, target: str, source_hash: str, missing: list[str],
) -> dict[str, Any]:
    output = f"case={case_id};target={target};missing_sources={','.join(missing)}"
    return {
        "case_id": case_id, "executed": False, "status": "FAIL", "reason": output,
        "sut_source_sha256": source_hash, "trace_sha256": sha256(output.encode()).hexdigest(),
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{target}",
            "missing_source_paths": missing, "subtest_output_sha256": sha256(output.encode()).hexdigest(),
        },
    }


def run_unittest_case(
    case_id: str,
    stimulus: dict[str, Any],
    target: str,
    source_paths: tuple[str, ...],
    runtime_action: str | None = None,
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, target, source_hash, missing)
    if runtime_action is not None:
        try:
            availability = subprocess.run(
                ["ros2", "action", "info", runtime_action, "-t"], cwd=ROOT,
                capture_output=True, text=True, check=False, timeout=10,
            )
            available = availability.returncode == 0 and "Action servers: 1" in availability.stdout
            availability_trace = availability.stdout + availability.stderr
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            available = False
            availability_trace = str(error)
        if not available:
            reason = f"required live action server unavailable: {runtime_action}"
            trace = f"case={case_id};{reason};{availability_trace}"
            return {
                "case_id": case_id, "executed": False, "status": "NOT_RUN", "reason": reason,
                "sut_source_sha256": source_hash, "trace_sha256": sha256(trace.encode()).hexdigest(),
                "observations": {
                    "case_id": case_id, "case_target": f"{case_id}:{target}",
                    "runtime_action": runtime_action, "runtime_available": False,
                    "subtest_output_sha256": sha256(availability_trace.encode()).hexdigest(),
                },
            }
    completed = subprocess.run(
        [sys.executable, "-m", "unittest", target, "-v"], cwd=ROOT,
        capture_output=True, text=True, check=False, timeout=90,
    )
    output = completed.stdout + completed.stderr
    match = re.search(r"Ran (\d+) tests?", output)
    count = int(match.group(1)) if match else 0
    passed = completed.returncode == 0 and count > 0
    trace = f"case={case_id};target={target};returncode={completed.returncode}\n{output}"
    return {
        "case_id": case_id, "executed": count > 0, "status": "PASS" if passed else "FAIL",
        "reason": "" if passed else output[-4000:], "sut_source_sha256": source_hash,
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{target}",
            "return_code": completed.returncode, "executed_test_count": count,
            "subtest_output_sha256": sha256(output.encode()).hexdigest(), "requirement_observed": passed,
        },
    }


def run_action_probe_case(
    case_id: str, stimulus: dict[str, Any], probe_path: str, source_paths: tuple[str, ...],
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, probe_path, source_hash, missing)
    with tempfile.TemporaryDirectory(prefix=f"{case_id.lower()}_probe_") as temporary:
        report = Path(temporary) / "observations.json"
        completed = subprocess.run(
            [sys.executable, str(ROOT / probe_path), "--report", str(report)], cwd=ROOT,
            capture_output=True, text=True, check=False, timeout=90,
        )
        output = completed.stdout + completed.stderr
        observed = json.loads(report.read_text()) if report.is_file() else {}
    trace = f"case={case_id};probe={probe_path};returncode={completed.returncode}\n{output}"
    if completed.returncode == 3:
        status, executed, reason = "NOT_RUN", False, output[-4000:] or "required live runtime unavailable"
    elif completed.returncode == 0 and observed:
        status, executed, reason = "PASS", True, ""
    else:
        status, executed, reason = "FAIL", True, output[-4000:] or "probe produced no observations"
    return {
        "case_id": case_id, "executed": executed, "status": status, "reason": reason,
        "sut_source_sha256": source_hash, "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{probe_path}",
            "probe_return_code": completed.returncode,
            "subtest_output_sha256": sha256(output.encode()).hexdigest(),
            "probe_observations": observed,
        },
    }


from dummy_test_support.adapters.r00 import run_r00_case
from dummy_test_support.adapters.r01 import run_r01_case
from dummy_test_support.adapters.r02 import run_r02_case
from dummy_test_support.adapters.r03 import run_r03_case
from dummy_test_support.adapters.r04 import run_r04_case
from dummy_test_support.adapters.r05 import run_r05_case
from dummy_test_support.adapters.r06 import run_r06_case
from dummy_test_support.adapters.r07 import run_r07_case
from dummy_test_support.adapters.r08 import run_r08_case


def run_case(suite: str, case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if suite == "R00":
        return run_r00_case(case_id, stimulus)
    if suite == "R01":
        return run_r01_case(case_id, stimulus)
    if suite == "R02":
        return run_r02_case(case_id, stimulus)
    if suite == "R03":
        return run_r03_case(case_id, stimulus)
    if suite == "R04":
        return run_r04_case(case_id, stimulus)
    if suite == "R05":
        return run_r05_case(case_id, stimulus)
    if suite == "R06":
        return run_r06_case(case_id, stimulus)
    if suite == "R07":
        return run_r07_case(case_id, stimulus)
    if suite == "R08":
        return run_r08_case(case_id, stimulus)
    raise ValueError(f"unsupported suite: {suite}")
