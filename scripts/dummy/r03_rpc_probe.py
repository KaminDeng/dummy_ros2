#!/usr/bin/python3
"""Run ROS-owner R03 evidence against real DPSDK dp_comm binaries."""

from __future__ import annotations
import os

import argparse
import json
from pathlib import Path
import subprocess
import time


DPSDK = Path(os.environ.get("DPSDK_ROOT", str(Path(__file__).resolve().parents[2].parent / "dpsdk_consolidated")))
BUILD = DPSDK / "build/posix-zenohc"
HOST_TESTS = BUILD / "libraries/dp_comm/dp_comm_ros_like_host_tests"
TRANSPORT_TESTS = BUILD / "libraries/dp_comm/dp_comm_ros_like_transport_tests"
INTERPROCESS = BUILD / "applications/dummy_ref_control/dummy_zenoh_interprocess_probe"
ROUTER = Path("/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd")


def checked(command: list[str], timeout: float = 15.0) -> subprocess.CompletedProcess[str]:
    completed = subprocess.run(
        command, cwd=DPSDK, capture_output=True, text=True, check=False, timeout=timeout
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n{completed.stdout}{completed.stderr}"
        )
    return completed


def run(report: Path) -> None:
    required = (HOST_TESTS, TRANSPORT_TESTS, INTERPROCESS, ROUTER)
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(f"missing R03 runtime binaries: {missing}")

    handle_filters = (
        "DpCommRosLike.TypedServiceClientDeliversResponseThroughExecutor:"
        "DpCommRosLike.CancelledTypedRequestIsNotDelivered"
    )
    checked([str(HOST_TESTS), f"--gtest_filter={handle_filters}"])
    checked([
        str(TRANSPORT_TESTS),
        "--gtest_filter=DpCommRosLikeTransport.CancelledRemoteCorrelationIsQuarantinedUntilDeadline",
    ])

    router = subprocess.Popen(
        [str(ROUTER)], cwd=DPSDK, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    server = None
    try:
        time.sleep(1.0)
        server = subprocess.Popen(
            [str(INTERPROCESS), "server"],
            cwd=DPSDK,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(1.0)
        client = checked([str(INTERPROCESS), "client"], timeout=10.0)
    finally:
        if server is not None:
            server.terminate()
            try:
                server.communicate(timeout=5.0)
            except subprocess.TimeoutExpired:
                server.kill()
                server.communicate(timeout=5.0)
        router.terminate()
        try:
            router.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            router.kill()
            router.wait(timeout=5.0)

    stdout = client.stdout
    observations = {
        "interprocess_service_passed": client.returncode == 0,
        "queued_not_accepted": "receipt=1 accepted=0" in stdout,
        "large_uint64_preserved": (
            "request_id=9007199254740993" in stdout
            and "boot_epoch=9007199254740995" in stdout
        ),
        "request_handle_tests_passed": 3,
        "client_stdout": stdout.strip(),
    }
    if not all(
        observations[key]
        for key in ("interprocess_service_passed", "queued_not_accepted", "large_uint64_preserved")
    ):
        raise RuntimeError(f"interprocess observations incomplete: {observations}")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(observations, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    try:
        run(arguments.report)
        return 0
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
