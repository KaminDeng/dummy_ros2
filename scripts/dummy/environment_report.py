#!/usr/bin/env python3
"""Collect a read-only, reproducible R00 environment baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
from typing import Any


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
DPSDK_ROOT = Path(os.environ.get("DPSDK_ROOT", WORKSPACE_ROOT.parent / "dpsdk"))
DUMMY_ROOT = Path(os.environ.get(
    "DUMMY_ROOT", WORKSPACE_ROOT.parent / "dummy_code/firmware/dummy-ref-core-fw"
))


def command_output(command: list[str]) -> dict[str, Any]:
    executable = shutil.which(command[0])
    if executable is None:
        return {"available": False, "command": command, "returncode": None, "output": ""}
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    return {
        "available": True,
        "command": command,
        "returncode": completed.returncode,
        "output": (completed.stdout + completed.stderr).strip(),
    }


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes() -> dict[str, str]:
    paths = (
        WORKSPACE_ROOT / "src/dummy_moveit_config/launch/gz_moveit.launch.py",
        WORKSPACE_ROOT / "src/dummy_moveit_config/config/dummy-ros2.gz_sim.xacro",
        WORKSPACE_ROOT / "src/dummy-ros2_description/urdf/dummy-ros2.xacro",
    )
    return {str(path.relative_to(WORKSPACE_ROOT)): sha256(path) for path in paths}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    report = {
        "workspace_root": str(WORKSPACE_ROOT),
        "dpsdk_root": str(DPSDK_ROOT),
        "dummy_root": str(DUMMY_ROOT),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "source_hashes": source_hashes(),
        "tools": {
            "ros2": command_output(["ros2", "--help"]),
            "gz": command_output(["gz", "--versions"]),
            "colcon": command_output(["colcon", "list"]),
        },
        "roots": {
            "workspace_exists": WORKSPACE_ROOT.is_dir(),
            "dpsdk_sentinel": (DPSDK_ROOT / "tools/ros2/zenbedded.repos").is_file(),
            "dummy_sentinel": (DUMMY_ROOT / "Robot/instances/dummy_robot.cpp").is_file(),
        },
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
