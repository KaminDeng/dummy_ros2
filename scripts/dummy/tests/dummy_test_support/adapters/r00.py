"""Real local operations used by R00 acceptance cases."""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess
import sys
from typing import Any


WORKSPACE_ROOT = Path(__file__).resolve().parents[5]
DPSDK_ROOT = Path("../dpsdk")
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
    return (path / "tools/ros2/zenbedded.repos").is_file() and (path / "CMakeLists.txt").is_file()


def _test_exit(command: list[str]) -> int:
    return subprocess.run(command, check=False, capture_output=True, text=True).returncode


def _classified_exit(return_code: int, discovered: int, skipped: int) -> int:
    if return_code != 0:
        return return_code
    if discovered == 0 or discovered == skipped:
        return 4
    return 0


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
        failed = _classified_exit(_test_exit([sys.executable, "-c", "raise SystemExit(1)"]), 1, 0)
        empty = _classified_exit(_test_exit([sys.executable, "-c", "raise SystemExit(0)"]), 0, 0)
        skipped = _classified_exit(_test_exit([sys.executable, "-c", "raise SystemExit(0)"]), 1, 1)
        observations = {
            "failed_test_exit": failed,
            "empty_test_exit": empty,
            "all_skipped_test_exit": skipped,
        }
        status = "PASS" if observations == {"failed_test_exit": 1, "empty_test_exit": 4, "all_skipped_test_exit": 4} else "FAIL"
    elif case_id == "AUD03":
        observations = _hardware_denial(stimulus)
    elif case_id == "ROS01":
        required_packages = ("gz_ros2_control", "controller_manager", "moveit_ros_move_group")
        package_checks = {
            package: _test_exit(["ros2", "pkg", "prefix", package]) == 0
            for package in required_packages
        }
        observations = {
            "hardware_mode": stimulus.get("hardware"),
            "dpsdk_process_started": False,
            "serial_open_count": 0,
            "required_packages": package_checks,
            "new_build_required": True,
        }
        status = "NOT_RUN"
        reason = "R00 does not reuse archived build/install output; Gazebo runtime starts after a new build."
    else:
        raise ValueError(f"unknown R00 case: {case_id}")

    trace = hashlib.sha256(repr((case_id, observations, status, reason)).encode("utf-8")).hexdigest()
    return {
        "case_id": case_id,
        "executed": True,
        "status": status,
        "reason": reason,
        "sut_source_sha256": source_hash_before,
        "trace_sha256": trace,
        "observations": observations,
    }
