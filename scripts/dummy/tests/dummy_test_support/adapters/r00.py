"""Real local operations used by R00 acceptance cases."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any


WORKSPACE_ROOT = Path(__file__).resolve().parents[5]
DPSDK_ROOT = Path(
    os.environ.get("DPSDK_ROOT", os.environ.get("DUMMY_DPSDK_ROOT", WORKSPACE_ROOT.parent / "dpsdk"))
)
SOURCE_FILES = (
    WORKSPACE_ROOT / "src/dummy_moveit_config/launch/gz_moveit.launch.py",
    WORKSPACE_ROOT / "src/dummy_moveit_config/config/dummy-ros2.gz_sim.xacro",
    WORKSPACE_ROOT / "src/dummy-ros2_description/urdf/dummy-ros2.xacro",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_hash() -> str:
    digest = hashlib.sha256()
    for path in SOURCE_FILES:
        digest.update(str(path.relative_to(WORKSPACE_ROOT)).encode("utf-8"))
        digest.update(_sha256(path).encode("ascii"))
    return digest.hexdigest()


def _valid_dpsdk_root(path: Path) -> bool:
    return (
        (path / "tools/ros2/zenbedded.repos").is_file()
        and (path / "applications/zenbedded_control").is_dir()
        and (path / "CMakeLists.txt").is_file()
    )


def _classified_exit(return_code: int, discovered: int, skipped: int) -> int:
    if discovered == 0 or discovered == skipped:
        return 4
    if return_code != 0:
        return return_code
    return 0


def _test_exit(command: list[str]) -> int:
    try:
        return subprocess.run(command, check=False, capture_output=True, text=True).returncode
    except FileNotFoundError:
        return 127


def _run_temporary_unittest(source: str) -> tuple[int, int, int, str]:
    with tempfile.TemporaryDirectory(prefix="r00_aud02_") as temporary:
        test_file = Path(temporary) / "test_probe.py"
        test_file.write_text(source)
        completed = subprocess.run(
            [sys.executable, "-m", "unittest", "discover", "-s", temporary, "-p", "test_*.py", "-v"],
            capture_output=True, text=True, check=False,
        )
    output = completed.stdout + completed.stderr
    match = re.search(r"Ran (\d+) tests?", output)
    skipped = re.search(r"skipped=(\d+)", output)
    return completed.returncode, int(match.group(1)) if match else 0, int(skipped.group(1)) if skipped else 0, output


class _HardwareNoneGuard:
    """Reject hardware access before a serial, CAN, or J-Link API is invoked."""

    def __init__(self) -> None:
        self.denied_probes: tuple[str, ...] = ()
        self.opened_probes: tuple[str, ...] = ()

    def request_probe(self, probe: str) -> None:
        self.denied_probes = (*self.denied_probes, probe)
        raise PermissionError(f"hardware probe denied in --hardware none: {probe}")


def _hardware_denial(stimulus: dict[str, Any]) -> dict[str, Any]:
    if stimulus.get("hardware") != "none":
        raise ValueError("R00 adapter only supports --hardware none")
    guard = _HardwareNoneGuard()
    for probe in ("ttyACM0", "socketcan", "jlink"):
        try:
            guard.request_probe(probe)
        except PermissionError:
            continue
    return {
        "attempted_probes": tuple(guard.denied_probes),
        "denied_probes": list(guard.denied_probes),
        "real_serial_open_count": guard.opened_probes.count("ttyACM0"),
        "socketcan_open_count": guard.opened_probes.count("socketcan"),
        "jlink_spawn_count": guard.opened_probes.count("jlink"),
        "denial_observed": len(guard.denied_probes) == 3,
    }


def run_r00_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    source_hash_before = _source_hash()
    observations: dict[str, Any]
    status = "PASS"
    reason = ""

    if case_id == "AUD01":
        invalid_root = DPSDK_ROOT / "applications"
        observations = {
            "dpsdk_root_valid": _valid_dpsdk_root(DPSDK_ROOT),
            "invalid_dpsdk_root_rejected": not _valid_dpsdk_root(invalid_root),
            "source_hashes_unchanged": source_hash_before == _source_hash(),
        }
        status = "PASS" if all(observations.values()) else "FAIL"
    elif case_id == "AUD02":
        failed_raw, failed_count, failed_skipped, failed_trace = _run_temporary_unittest(
            "import unittest\nclass Probe(unittest.TestCase):\n def test_failure(self): self.fail('expected AUD02 failure')\n"
        )
        empty_raw, empty_count, empty_skipped, empty_trace = _run_temporary_unittest("# deliberately no tests\n")
        skipped_raw, skipped_count, skipped_skipped, skipped_trace = _run_temporary_unittest(
            "import unittest\n@unittest.skip('expected AUD02 skip')\nclass Probe(unittest.TestCase):\n def test_skip(self): pass\n"
        )
        failed = _classified_exit(failed_raw, failed_count, failed_skipped)
        empty = _classified_exit(empty_raw, empty_count, empty_skipped)
        skipped = _classified_exit(skipped_raw, skipped_count, skipped_skipped)
        observations = {
            "failed_test_exit": failed,
            "empty_test_exit": empty,
            "all_skipped_test_exit": skipped,
            "temporary_unittest_processes": 3,
        }
        status = "PASS" if (
            failed == 1 and empty == 4 and skipped == 4
            and observations["temporary_unittest_processes"] == 3
        ) else "FAIL"
        reason = "" if status == "PASS" else "\n".join((failed_trace, empty_trace, skipped_trace))
    elif case_id == "AUD03":
        observations = _hardware_denial(stimulus)
    elif case_id == "ROS01":
        from dummy_test_support.adapters.r02 import run_r02_case

        return run_r02_case(case_id, stimulus)
    else:
        raise ValueError(f"unknown R00 case: {case_id}")

    trace = repr((case_id, observations, status, reason))
    return {
        "case_id": case_id,
        "executed": status != "NOT_RUN",
        "status": status,
        "reason": reason,
        "sut_source_sha256": source_hash_before,
        "trace_sha256": hashlib.sha256(trace.encode("utf-8")).hexdigest(),
        "sut_kind": "document_utility", "_trace_text": trace,
        "observations": observations,
    }
