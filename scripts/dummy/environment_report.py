#!/usr/bin/env python3
##
# @file environment_report.py
# @brief 收集R00宿主环境、工具输出与三份源码原始摘要。
#
# 入口会执行dpkg-query、gz --versions、ros2 --help和colcon list等本地命令，并写指定JSON；不启动launch或硬件动作。
# DPSDK/DUMMY根可由环境覆盖，缺省仍使用历史同级目录名；根存在/哨兵只记录bool，不验证仓库版本或部署身份一致。
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


##
# @brief 从脚本路径推工作区；DPSDK_ROOT默认dpsdk_consolidated，DUMMY_ROOT默认旧固件根。
# 这些路径是报告输入位置而非SDK自动发现/认证结果，环境覆盖不会重定位ROS源码三文件。
WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
DPSDK_ROOT = Path(os.environ.get("DPSDK_ROOT", WORKSPACE_ROOT.parent / "dpsdk_consolidated"))
DUMMY_ROOT = Path(os.environ.get(
    "DUMMY_ROOT", WORKSPACE_ROOT.parent / "dummy_code/firmware/dummy-ref-core-fw"
))


##
# @brief 查找命令并捕获其实际退出码与合并输出。
# @param command 非空argv列表，第0项通过PATH查找；子进程继承当前环境/cwd。
# @return available/command/returncode/output字典；未找到available=false，其它启动异常传播。
# 不设timeout，命令可能阻塞；available=true仅表示可找到并执行，不代表returncode=0或工具功能完整。
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


##
# @brief 读取文件全部原始字节求摘要。
# @param path 存在的文件路径。
# @return 64字符小写SHA-256；读取异常传播，未解析或规范化YAML/XML。
def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


##
# @brief 通过dpkg-query读取宿主Debian包版本。
# @param name 包名，本入口使用Jazzy包名。
# @return 成功stdout版本；查询非零为NOT_AVAILABLE，dpkg-query本身无法启动则异常传播。
def package_version(name: str) -> str:
    completed = subprocess.run(
        ["dpkg-query", "-W", "-f=${Version}", name],
        check=False, capture_output=True, text=True,
    )
    return completed.stdout.strip() if completed.returncode == 0 else "NOT_AVAILABLE"


##
# @brief 固定记录三个launch/xacro源文件的原字节SHA。
# @return 工作区相对路径到摘要的dict；缺文件时失败，不以空值伪装完整报告。
def source_hashes() -> dict[str, str]:
    paths = (
        WORKSPACE_ROOT / "src/dummy_moveit_config/launch/gz_moveit.launch.py",
        WORKSPACE_ROOT / "src/dummy_moveit_config/config/dummy-ros2.gz_sim.xacro",
        WORKSPACE_ROOT / "src/dummy-ros2_description/urdf/dummy-ros2.xacro",
    )
    return {str(path.relative_to(WORKSPACE_ROOT)): sha256(path) for path in paths}


##
# @brief 采集环境并创建/覆盖报告JSON。
# @return 成功写入0；参数或命令/I/O异常由argparse/调用栈报告。
# ROS_DISTRO缺省jazzy只是报告回退值，不能证明环境已经source Jazzy。colcon list受调用cwd影响；各工具输出原样存档，不从中自动推出全部依赖可用。
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
        "versions": {
            "ros_distro": os.environ.get("ROS_DISTRO", "jazzy"),
            "moveit": package_version("ros-jazzy-moveit"),
            "joint_trajectory_controller": package_version(
                "ros-jazzy-joint-trajectory-controller"
            ),
            "rmw": package_version("ros-jazzy-rmw-fastrtps-cpp"),
            "gazebo": command_output(["gz", "--versions"])["output"] or "NOT_AVAILABLE",
        },
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
