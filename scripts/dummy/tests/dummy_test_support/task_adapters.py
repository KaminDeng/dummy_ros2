"""Dispatch test cases to their suite-specific adapters."""

from __future__ import annotations

from typing import Any

from dummy_test_support.adapters.r00 import run_r00_case
from dummy_test_support.adapters.r06 import run_r06_case
from dummy_test_support.adapters.r07 import run_r07_case
from dummy_test_support.adapters.r08 import run_r08_case


def run_case(suite: str, case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if suite == "R00":
        return run_r00_case(case_id, stimulus)
    if suite == "R06":
        return run_r06_case(case_id, stimulus)
    if suite == "R07":
        return run_r07_case(case_id, stimulus)
    if suite == "R08":
        return run_r08_case(case_id, stimulus)
    raise ValueError(f"unsupported suite: {suite}")
