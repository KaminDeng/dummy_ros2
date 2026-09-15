#!/usr/bin/env python3
"""Strict suite runner for the Dummy ROS project."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as element_tree


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
TEST_ROOT = WORKSPACE_ROOT / "scripts" / "dummy" / "tests"
sys.path.insert(0, str(TEST_ROOT))

from dummy_test_support.task_adapters import run_case  # noqa: E402


EXIT_PASS = 0
EXIT_UNKNOWN_SUITE = 2
EXIT_NOT_RUN = 3
EXIT_UNEXECUTED = 4
SUITES = {
    "R00": {
        "fixture": "R00.json",
    },
    "R01": {
        "fixture": "R01.json",
    },
    "R02": {
        "fixture": "R02.json",
    },
    "R03": {
        "fixture": "R03.json",
    },
    "R04": {
        "fixture": "R04.json",
    },
    "R05": {
        "fixture": "R05.json",
    },
    "R06": {
        "fixture": "R06.json",
        "modules": ("scripts/dummy/tests/test_r06_acceptance.py",),
    },
    "R07": {
        "fixture": "R07.json",
        "modules": ("scripts/dummy/tests/test_r07_acceptance.py",),
        "timeout": 180,
    },
    "R08": {
        "fixture": "R08.json",
        "modules": ("scripts/dummy/tests/test_r08_acceptance.py",),
    },
}


def write_junit(report: dict[str, object], path: Path) -> None:
    cases = report["cases"]
    suite = element_tree.Element("testsuite", name=str(report["suite"]), tests=str(len(cases)))
    for result in cases:
        case = element_tree.SubElement(suite, "testcase", name=result["case_id"])
        if result["status"] == "FAIL":
            element_tree.SubElement(case, "failure", message=result["reason"] or result["status"])
        elif result["status"] in {"NOT_RUN", "SKIP"}:
            element_tree.SubElement(case, "skipped", message=result["reason"] or result["status"])
    path.write_text(element_tree.tostring(suite, encoding="unicode") + "\n")


def write_log(report: dict[str, object], path: Path) -> None:
    lines = [f"suite={report['suite']}", f"hardware={report['hardware']}"]
    for result in report["cases"]:
        lines.append(
            f"case={result['case_id']} status={result['status']} trace={result['trace_sha256']}"
        )
        if result["reason"]:
            lines.append(f"reason={result['reason']}")
    path.write_text("\n".join(lines) + "\n")


def _invalid_result(case_id: str, reason: str) -> dict[str, object]:
    trace = hashlib.sha256(reason.encode()).hexdigest()
    return {
        "case_id": case_id,
        "executed": True,
        "status": "FAIL",
        "reason": reason,
        "sut_source_sha256": hashlib.sha256(case_id.encode()).hexdigest(),
        "trace_sha256": trace,
        "observations": {"runner_validation_error": reason},
    }


def _validate_case_result(case_id: str, result: object) -> dict[str, object]:
    if not isinstance(result, dict):
        return _invalid_result(case_id, "adapter did not return an evidence object")
    required = {
        "case_id", "executed", "status", "reason", "sut_source_sha256",
        "trace_sha256", "observations",
    }
    missing = sorted(required.difference(result))
    if missing:
        return _invalid_result(case_id, f"adapter evidence missing fields: {', '.join(missing)}")
    if result["case_id"] != case_id:
        return _invalid_result(case_id, "adapter evidence case_id does not match requested fixture case")
    if not isinstance(result["executed"], bool):
        return _invalid_result(case_id, "adapter evidence executed must be boolean")
    if result["status"] not in {"PASS", "FAIL", "NOT_RUN", "SKIP"}:
        return _invalid_result(case_id, f"adapter evidence status is invalid: {result['status']!r}")
    if not isinstance(result["reason"], str) or not isinstance(result["observations"], dict):
        return _invalid_result(case_id, "adapter evidence reason/observations has invalid type")
    for field in ("sut_source_sha256", "trace_sha256"):
        value = result[field]
        if not isinstance(value, str) or len(value) != 64:
            return _invalid_result(case_id, f"adapter evidence {field} is not a SHA-256 digest")
    return result


def _suite_status(results: list[dict[str, object]]) -> tuple[str, int]:
    if not results:
        return "UNEXECUTED", EXIT_UNEXECUTED
    if any(result["status"] == "FAIL" for result in results):
        return "FAIL", 1
    if any(result["status"] == "NOT_RUN" for result in results):
        return "NOT_RUN", EXIT_NOT_RUN
    if any(not result["executed"] or result["status"] == "SKIP" for result in results):
        return "UNEXECUTED", EXIT_UNEXECUTED
    if all(result["status"] == "PASS" and result["executed"] for result in results):
        return "PASS", EXIT_PASS
    return "UNEXECUTED", EXIT_UNEXECUTED


def run_registered_suite(
    suite: str, hardware: str, selected_case: str | None = None,
) -> tuple[dict[str, object], int]:
    definition = SUITES[suite]
    fixture = json.loads((TEST_ROOT / "fixtures" / definition["fixture"]).read_text())
    cases = fixture.get("cases")
    if not isinstance(cases, list):
        results = [_invalid_result("<fixture>", "fixture cases must be a list")]
    else:
        if selected_case is not None:
            if selected_case not in cases:
                return {
                    "suite": suite, "hardware": hardware, "status": "UNKNOWN_CASE",
                    "executed_test_count": 0, "cases": [],
                }, EXIT_UNKNOWN_SUITE
            cases = [selected_case]
        results = []
        for case_id in cases:
            if not isinstance(case_id, str):
                results.append(_invalid_result("<fixture>", "fixture case identifiers must be strings"))
                continue
            try:
                result = run_case(suite, case_id, {
                    "case_id": case_id, "hardware": hardware, "test_only": True,
                })
            except Exception as error:  # Keep a report even when an adapter fails unexpectedly.
                result = _invalid_result(case_id, f"adapter raised {type(error).__name__}: {error}")
            results.append(_validate_case_result(case_id, result))
    status, exit_code = _suite_status(results)
    return {
        "suite": suite, "hardware": hardware, "status": status,
        "executed_test_count": sum(bool(result["executed"]) for result in results),
        "cases": results,
    }, exit_code


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", required=True)
    parser.add_argument("--case")
    parser.add_argument("--hardware", default="none", choices=("none",))
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.suite not in SUITES:
        return EXIT_UNKNOWN_SUITE
    if arguments.suite != "R00" and arguments.case is not None:
        return EXIT_UNKNOWN_SUITE
    report, exit_code = run_registered_suite(
        arguments.suite, arguments.hardware, arguments.case,
    )
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    arguments.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    write_junit(report, arguments.report.with_suffix(".junit.xml"))
    write_log(report, arguments.report.with_suffix(".log"))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
