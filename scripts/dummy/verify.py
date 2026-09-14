#!/usr/bin/env python3
"""Strict suite runner for the Dummy ROS project."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", required=True)
    parser.add_argument("--case", choices=R00_CASES)
    parser.add_argument("--hardware", default="none", choices=("none",))
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.suite != "R00":
        return EXIT_UNKNOWN_SUITE

    selected_cases = (arguments.case,) if arguments.case else R00_CASES
    results = [run_case("R00", case_id, {"hardware": arguments.hardware}) for case_id in selected_cases]
    report = {"suite": "R00", "hardware": arguments.hardware, "cases": results}
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    arguments.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    write_junit(report, arguments.report.with_suffix(".junit.xml"))
    write_log(report, arguments.report.with_suffix(".log"))
    if any(result["status"] == "FAIL" for result in results):
        return 1
    if any(result["status"] == "NOT_RUN" for result in results):
        return EXIT_NOT_RUN
    if not results:
        return EXIT_UNEXECUTED
    return EXIT_PASS


if __name__ == "__main__":
    raise SystemExit(main())
