"""Dispatch test cases to their suite-specific adapters."""

from __future__ import annotations

from hashlib import sha256
import json
import os
import re
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any, Callable
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[4]
EvidenceValidator = Callable[[dict[str, Any]], bool | tuple[bool, str]]


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
        "_trace_text": output,
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{target}",
            "missing_source_paths": missing, "subtest_output_sha256": sha256(output.encode()).hexdigest(),
        },
    }


def not_run_case(
    case_id: str, stimulus: dict[str, Any], source_paths: tuple[str, ...], reason: str,
    *, sut_kind: str = "document_utility",
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, "not-run", source_hash, missing)
    trace = f"case={case_id};status=NOT_RUN;reason={reason}"
    return {
        "case_id": case_id, "executed": False, "status": "NOT_RUN", "reason": reason,
        "sut_source_sha256": source_hash, "trace_sha256": sha256(trace.encode()).hexdigest(),
        "sut_kind": sut_kind, "_trace_text": trace,
        "observations": {"case_id": case_id, "case_specific_probe_available": False},
    }


def run_unittest_case(
    case_id: str,
    stimulus: dict[str, Any],
    target: str,
    source_paths: tuple[str, ...],
    runtime_action: str | None = None,
    sut_kind: str = "document_utility",
    python_executable: str | None = None,
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
                "sut_kind": sut_kind, "_trace_text": trace,
                "observations": {
                    "case_id": case_id, "case_target": f"{case_id}:{target}",
                    "runtime_action": runtime_action, "runtime_available": False,
                    "subtest_output_sha256": sha256(availability_trace.encode()).hexdigest(),
                },
            }
    try:
        completed = subprocess.run(
            [python_executable or sys.executable, "-m", "unittest", target, "-v"], cwd=ROOT,
            capture_output=True, text=True, check=False, timeout=90,
        )
        output = completed.stdout + completed.stderr
    except subprocess.TimeoutExpired as error:
        output = (error.stdout or "") + (error.stderr or "")
        completed = subprocess.CompletedProcess(error.cmd, 124, output, "")
    match = re.search(r"Ran (\d+) tests?", output)
    count = int(match.group(1)) if match else 0
    passed = completed.returncode == 0 and count > 0
    trace = f"case={case_id};target={target};returncode={completed.returncode}\n{output}"
    return {
        "case_id": case_id, "executed": count > 0, "status": "PASS" if passed else "FAIL",
        "reason": "" if passed else output[-4000:], "sut_source_sha256": source_hash,
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "sut_kind": sut_kind, "_trace_text": trace,
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{target}",
            "return_code": completed.returncode, "executed_test_count": count,
            "subtest_output_sha256": sha256(output.encode()).hexdigest(), "requirement_observed": passed,
        },
    }


def run_action_probe_case(
    case_id: str,
    stimulus: dict[str, Any],
    probe_path: str,
    source_paths: tuple[str, ...],
    *,
    validator: EvidenceValidator,
    sut_kind: str = "ros_launch",
    runtime_action: str | None = None,
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, probe_path, source_hash, missing)
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
                "sut_kind": sut_kind, "_trace_text": trace,
                "observations": {
                    "case_id": case_id, "case_target": f"{case_id}:{probe_path}",
                    "runtime_action": runtime_action, "runtime_available": False,
                    "subtest_output_sha256": sha256(availability_trace.encode()).hexdigest(),
                },
            }
    with tempfile.TemporaryDirectory(prefix=f"{case_id.lower()}_probe_") as temporary:
        report = Path(temporary) / "observations.json"
        try:
            completed = subprocess.run(
                [sys.executable, str(ROOT / probe_path), "--report", str(report)], cwd=ROOT,
                capture_output=True, text=True, check=False, timeout=90,
            )
            output = completed.stdout + completed.stderr
        except subprocess.TimeoutExpired as error:
            output = (error.stdout or "") + (error.stderr or "")
            completed = subprocess.CompletedProcess(error.cmd, 124, output, "")
        try:
            observed = json.loads(report.read_text()) if report.is_file() else {}
        except json.JSONDecodeError as error:
            observed = {}
            output += f"\ninvalid probe JSON: {error}"
    trace = f"case={case_id};probe={probe_path};returncode={completed.returncode}\n{output}"
    if completed.returncode == 3:
        status, executed, reason = "NOT_RUN", False, output[-4000:] or "required live runtime unavailable"
    elif completed.returncode == 0 and isinstance(observed, dict):
        validation = validator(observed)
        valid, validation_reason = (
            validation if isinstance(validation, tuple) else (validation, "case validator rejected probe observations")
        )
        status, executed, reason = (
            ("PASS", True, "") if valid else ("FAIL", True, validation_reason)
        )
    else:
        status, executed, reason = "FAIL", True, output[-4000:] or "probe produced no observations"
    return {
        "case_id": case_id, "executed": executed, "status": status, "reason": reason,
        "sut_source_sha256": source_hash, "trace_sha256": sha256(trace.encode()).hexdigest(),
        "sut_kind": sut_kind, "_trace_text": trace,
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{probe_path}",
            "probe_return_code": completed.returncode,
            "subtest_output_sha256": sha256(output.encode()).hexdigest(),
            "probe_observations": observed,
        },
    }


def run_cpp_probe_case(
    case_id: str,
    stimulus: dict[str, Any],
    source_path: str,
    source_paths: tuple[str, ...],
    observation_name: str,
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, source_path, source_hash, missing)
    with tempfile.TemporaryDirectory(prefix=f"{case_id.lower()}_cpp_") as temporary:
        binary = Path(temporary) / "probe"
        compile_command = [
            "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
            "-I", str(ROOT / "src/dummy_ros_control/include"), str(ROOT / source_path), "-o", str(binary),
        ]
        try:
            compiled = subprocess.run(
                compile_command, cwd=ROOT, capture_output=True, text=True, check=False, timeout=90,
            )
            executed = (
                subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False, timeout=90)
                if compiled.returncode == 0 else None
            )
            output = compiled.stdout + compiled.stderr
            if executed is not None:
                output += executed.stdout + executed.stderr
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            compiled = subprocess.CompletedProcess(compile_command, 124, "", str(error))
            executed = None
            output = str(error)
    passed = compiled.returncode == 0 and executed is not None and executed.returncode == 0
    blocked = compiled.returncode != 0 and "fatal error:" in output
    trace = f"case={case_id};compile={' '.join(compile_command)}\n{output}"
    return {
        "case_id": case_id, "executed": executed is not None,
        "status": "PASS" if passed else ("BLOCKED" if blocked else "FAIL"),
        "reason": "" if passed else output[-4000:], "sut_source_sha256": source_hash,
        "trace_sha256": sha256(trace.encode()).hexdigest(), "sut_kind": "core_binary", "_trace_text": trace,
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{source_path}",
            "compile_return_code": compiled.returncode,
            "probe_return_code": None if executed is None else executed.returncode,
            observation_name: passed,
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


def _input_digest(stimulus: dict[str, Any]) -> str:
    encoded = json.dumps(stimulus, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode()
    return sha256(encoded).hexdigest()


def _materialize_evidence(result: dict[str, Any], stimulus: dict[str, Any]) -> dict[str, Any]:
    evidence = dict(result)
    evidence["schema"] = 1
    evidence["sut_kind"] = evidence.get("sut_kind", "document_utility")
    evidence["input_digest"] = _input_digest(stimulus)
    trace_text = str(evidence.pop("_trace_text", ""))
    if not trace_text:
        trace_text = json.dumps(evidence, ensure_ascii=True, sort_keys=True, default=str)
    trace_dir = Path(os.environ.get("DUMMY_EVIDENCE_DIR", ROOT / "build_dummy/reports/evidence"))
    trace_dir.mkdir(parents=True, exist_ok=True)
    trace_path = trace_dir / f"{evidence['case_id'].lower()}-{uuid4().hex}.trace"
    trace_path.write_text(trace_text + "\n")
    evidence["trace_path"] = str(trace_path.resolve())
    evidence["trace_sha256"] = sha256(trace_path.read_bytes()).hexdigest()
    return evidence


def run_case(suite: str, case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if suite == "R00":
        result = run_r00_case(case_id, stimulus)
    elif suite == "R01":
        result = run_r01_case(case_id, stimulus)
    elif suite == "R02":
        result = run_r02_case(case_id, stimulus)
    elif suite == "R03":
        result = run_r03_case(case_id, stimulus)
    elif suite == "R04":
        result = run_r04_case(case_id, stimulus)
    elif suite == "R05":
        result = run_r05_case(case_id, stimulus)
    elif suite == "R06":
        result = run_r06_case(case_id, stimulus)
    elif suite == "R07":
        result = run_r07_case(case_id, stimulus)
    elif suite == "R08":
        result = run_r08_case(case_id, stimulus)
    else:
        raise ValueError(f"unsupported suite: {suite}")
    return _materialize_evidence(result, stimulus)
