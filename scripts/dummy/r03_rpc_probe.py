#!/usr/bin/python3
##
# @file r03_rpc_probe.py
# @brief 执行宿主dp_comm二进制与跨进程RPC的R03证据入口。
#
# 执行会运行三个选定GTest、启动rmw_zenohd router与服务端/客户端进程并写JSON，具有真实进程/网络副作用；不是只读文件静态检查。
# 这里的real DPSDK binaries指实际宿主通信实现，不等于真实电机/CAN证据。报告只覆盖请求句柄/相关隔离和指定消息观察，不认证机械资格。
"""Run ROS-owner R03 evidence against real DPSDK dp_comm binaries."""

from __future__ import annotations
import os

import argparse
import json
from pathlib import Path
import subprocess
import time


##
# @brief DPSDK_ROOT覆盖历史dpsdk_consolidated默认根；二进制固定在build/posix-zenohc。
# router路径固定Jazzy rmw_zenoh_cpp，源码SHA/编译revision不在本入口核对，文件存在不证明二进制新鲜。
DPSDK = Path(os.environ.get("DPSDK_ROOT", str(Path(__file__).resolve().parents[2].parent / "dpsdk_consolidated")))
BUILD = DPSDK / "build/posix-zenohc"
HOST_TESTS = BUILD / "libraries/dp_comm/dp_comm_ros_like_host_tests"
TRANSPORT_TESTS = BUILD / "libraries/dp_comm/dp_comm_ros_like_transport_tests"
INTERPROCESS = BUILD / "applications/dummy_ref_control/dummy_zenoh_interprocess_probe"
ROUTER = Path("/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd")


##
# @brief 在SDK根运行命令，限时捕获输出并拒绝非零。
# @param command argv列表，调用者提供显式二进制/过滤器。
# @param timeout 秒，默认15；跨进程client使用10。
# @return 成功CompletedProcess；非零抛RuntimeError附stdout/stderr，超时/启动错误传播。
def checked(command: list[str], timeout: float = 15.0) -> subprocess.CompletedProcess[str]:
    completed = subprocess.run(
        command, cwd=DPSDK, capture_output=True, text=True, check=False, timeout=timeout
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n{completed.stdout}{completed.stderr}"
        )
    return completed


##
# @brief 运行R03软件通信回归并归档选定观察。
# @param report 输出JSON，成功才创建父目录/覆盖报告。
# @return None；缺二进制、测试/观察失败或超时抛异常，不产出成功报告。
# 先跑2个句柄case与1个远程correlation隔离case；随后启动router、服务端各等1s，client完成后finally terminate，5s不退则kill。
# 固定等待不是服务ready握手；只检查client退出与stdout中Queued非Accepted、大uint64字段，server退出码并未作为独立成功断言。request_handle_tests_passed=3是固定计数，不是解析实际GTest执行数量。
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


##
# @brief 解析报告路径并把预期进程/IO失败转换为退出1。
# @return run成功0；OSError/RuntimeError/TimeoutExpired打印错误并返回1。
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
