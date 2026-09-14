"""Dispatch test cases to their suite-specific adapters."""

from __future__ import annotations

from typing import Any

from dummy_test_support.adapters.r00 import run_r00_case


def run_case(suite: str, case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if suite != "R00":
        raise ValueError(f"unsupported suite: {suite}")
    return run_r00_case(case_id, stimulus)
