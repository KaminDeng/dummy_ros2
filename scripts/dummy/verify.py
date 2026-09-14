#!/usr/bin/env python3
"""Strict suite runner for the Dummy ROS project."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest
import xml.etree.ElementTree as element_tree


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
TEST_ROOT = WORKSPACE_ROOT / "scripts" / "dummy" / "tests"
sys.path.insert(0, str(TEST_ROOT))

from dummy_test_support.task_adapters import run_case  # noqa: E402


EXIT_PASS = 0
EXIT_UNKNOWN_SUITE = 2
EXIT_NOT_RUN = 3
EXIT_UNEXECUTED = 4
R00_CASES = ("AUD01", "AUD02", "AUD03", "ROS01")
SUITES = {
    "R01": {
        "fixture": "R01.json",
        "modules": (
            "scripts/dummy/tests/test_r01_contract.py",
            "src/dummy_moveit_config/test/test_model_alignment.py",
            "src/dummy_moveit_config/test/test_reference_chain_fk.py",
        ),
    },
    "R03": {
        "fixture": "R03.json",
        "modules": ("scripts/dummy/tests/test_r03_acceptance.py",),
    },
    "R04": {
        "fixture": "R04.json",
        "modules": ("scripts/dummy/tests/test_r04_acceptance.py",),
    },
    "R05": {
        "fixture": "R05.json",
        "modules": ("scripts/dummy/tests/test_r05_acceptance.py",),
        "runtime_action": "/arm_controller/follow_joint_trajectory",
    },
}


def write_junit(report: dict[str, object], path: Path) -> None:
    cases = report["cases"]
    suite = element_tree.Element("testsuite", name=str(report["suite"]), tests=str(len(cases)))
    for result in cases:
        case = element_tree.SubElement(suite, "testcase", name=result["case_id"])
        if result["status"] != "PASS":
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


def source_digest(paths: tuple[str, ...]) -> str:
    digest = hashlib.sha256()
    for relative in paths:
        path = WORKSPACE_ROOT / relative
        digest.update(relative.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def runtime_action_available(name: str) -> bool:
    completed = subprocess.run(
        ["ros2", "action", "info", name, "-t"],
        cwd=WORKSPACE_ROOT, capture_output=True, text=True, check=False, timeout=10,
    )
    return completed.returncode == 0 and "Action servers: 1" in completed.stdout


def run_registered_suite(suite: str, hardware: str) -> tuple[dict[str, object], int]:
    definition = SUITES[suite]
    fixture = json.loads((TEST_ROOT / "fixtures" / definition["fixture"]).read_text())
    modules = definition["modules"]
    if "runtime_action" in definition and not runtime_action_available(definition["runtime_action"]):
        reason = f"required live action server unavailable: {definition['runtime_action']}"
        results = [
            {
                "case_id": case_id,
                "executed": False,
                "status": "NOT_RUN",
                "reason": reason,
                "sut_source_sha256": source_digest(modules),
                "trace_sha256": hashlib.sha256(reason.encode()).hexdigest(),
                "observations": {},
            }
            for case_id in fixture["cases"]
        ]
        return {
            "suite": suite, "hardware": hardware, "status": "NOT_RUN",
            "executed_test_count": 0, "cases": results,
        }, EXIT_NOT_RUN

    command = ["/usr/bin/python3", "-m", "unittest", *modules, "-v"]
    completed = subprocess.run(
        command, cwd=WORKSPACE_ROOT, capture_output=True, text=True, check=False, timeout=45,
    )
    output = completed.stdout + completed.stderr
    match = re.search(r"Ran (\d+) tests?", output)
    count = int(match.group(1)) if match else 0
    passed = completed.returncode == 0 and count > 0 and "FAILED" not in output
    status = "PASS" if passed else "FAIL"
    trace = hashlib.sha256(output.encode()).hexdigest()
    results = [
        {
            "case_id": case_id,
            "executed": count > 0,
            "status": status,
            "reason": "" if passed else output[-4000:],
            "sut_source_sha256": source_digest(modules),
            "trace_sha256": trace,
            "observations": {"aggregate_test_count": count},
        }
        for case_id in fixture["cases"]
    ]
    return {
        "suite": suite, "hardware": hardware, "status": status,
        "executed_test_count": count, "command": command, "cases": results,
    }, EXIT_PASS if passed else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", required=True)
    parser.add_argument("--case")
    parser.add_argument("--hardware", default="none", choices=("none",))
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.suite != "R00" and arguments.suite not in SUITES:
        return EXIT_UNKNOWN_SUITE

    if arguments.suite == "R00":
        if arguments.case is not None and arguments.case not in R00_CASES:
            return EXIT_UNKNOWN_SUITE
        selected_cases = (arguments.case,) if arguments.case else R00_CASES
        results = [run_case("R00", case_id, {"hardware": arguments.hardware}) for case_id in selected_cases]
        report = {"suite": "R00", "hardware": arguments.hardware, "cases": results}
        exit_code = 1 if any(result["status"] == "FAIL" for result in results) else (
            EXIT_NOT_RUN if any(result["status"] == "NOT_RUN" for result in results) else (
                EXIT_UNEXECUTED if not results else EXIT_PASS
            )
        )
    else:
        if arguments.case is not None:
            return EXIT_UNKNOWN_SUITE
        report, exit_code = run_registered_suite(arguments.suite, arguments.hardware)
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    arguments.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    write_junit(report, arguments.report.with_suffix(".junit.xml"))
    write_log(report, arguments.report.with_suffix(".log"))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
