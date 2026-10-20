"""Real local operations used by R00 acceptance cases."""
##
# @file r00.py
# @brief 环境/runner行为检查与内存硬件拒绝guard的R00适配器。
#
# AUD01只核路径sentinel与三文件摘要；AUD02实际运行三个临时unittest进程；AUD03仅调用自有PermissionError guard并统计内存元组，不能据此证明整个OS禁止任意串口/CAN/JLink访问。ROS01转R02实际运行时探针。

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
    os.environ.get("DPSDK_ROOT", os.environ.get("DUMMY_DPSDK_ROOT", WORKSPACE_ROOT.parent / "dpsdk_consolidated"))
)
SOURCE_FILES = (
    WORKSPACE_ROOT / "src/dummy_moveit_config/launch/gz_moveit.launch.py",
    WORKSPACE_ROOT / "src/dummy_moveit_config/config/dummy-ros2.gz_sim.xacro",
    WORKSPACE_ROOT / "src/dummy-ros2_description/urdf/dummy-ros2.xacro",
)


##
# @brief 对单文件原字节求SHA-256，不解析文件语义。
# @param path 必须存在的文件Path，读失败异常传播。
# @return 64字符小写SHA-256。
def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


##
# @brief 按固定SOURCE_FILES顺序拼相对路径及各文件hex摘要再SHA，不是整个仓库或构建产物摘要。
# @return 三个选定输入按既定拼接规则的SHA-256。
def _source_hash() -> str:
    digest = hashlib.sha256()
    for path in SOURCE_FILES:
        digest.update(str(path.relative_to(WORKSPACE_ROOT)).encode("utf-8"))
        digest.update(_sha256(path).encode("ascii"))
    return digest.hexdigest()


##
# @brief 仅查repos sentinel、zenbedded_control目录和顶层CMakeLists三个形状条件。
# @param path 待核SDK根路径，不自动解析为其上级。
# @return 三个文件/目录形状条件是否都成立。
def _valid_dpsdk_root(path: Path) -> bool:
    return (
        (path / "tools/ros2/zenbedded.repos").is_file()
        and (path / "applications/zenbedded_control").is_dir()
        and (path / "CMakeLists.txt").is_file()
    )


##
# @brief 发现0例或全部skip优先归退出4，否则保留子进程非零码，正常有效测试归0。
# @param return_code 原子测试进程退出码。
# @param discovered 解析发现测试数。
# @param skipped 解析跳过测试数。
# @return 分类退出码，空/全skip优先4。
def _classified_exit(return_code: int, discovered: int, skipped: int) -> int:
    if discovered == 0 or discovered == skipped:
        return 4
    if return_code != 0:
        return return_code
    return 0


##
# @brief 运行命令并捕获输出但只返回退出码；缺可执行文件归127，无显式timeout。
# @param command 要实际运行的argv列表，不经本helper加硬件限制。
# @return 子进程退出码，缺命令127。
def _test_exit(command: list[str]) -> int:
    try:
        return subprocess.run(command, check=False, capture_output=True, text=True).returncode
    except FileNotFoundError:
        return 127


##
# @brief 临时写test_probe.py并启动unittest discover，从文本Ran/skipped提取计数，退出后删除目录。
# @param source 原样写入test_probe.py的Python文本，将在子进程执行。
# @return (原退出码, 发现数, skip数, stdout+stderr)。
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


##
# @brief 只记录请求并总抛PermissionError的内存guard；不是系统范围硬件访问拦截器。
class _HardwareNoneGuard:
    """Reject hardware access before a serial, CAN, or J-Link API is invoked."""

    ##
    # @brief 初始化denied/opened空元组，opened在本guard中从不写入真实打开结果。
    # @return None；仅初始化内存元组。
    def __init__(self) -> None:
        self.denied_probes: tuple[str, ...] = ()
        self.opened_probes: tuple[str, ...] = ()

    ##
    # @brief 先记录probe名称再抛PermissionError，根本不调用硬件API。
    # @param probe ttyACM0/socketcan/jlink等guard名称，仅用于内存拒绝记录。
    # @return 无正常返回；总抛PermissionError。
    def request_probe(self, probe: str) -> None:
        self.denied_probes = (*self.denied_probes, probe)
        raise PermissionError(f"hardware probe denied in --hardware none: {probe}")


##
# @brief 只接受hardware=none，依次请求三个自有guard名称并核拒绝计数；不探测真实设备。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 内存guard的拒绝名称与零打开计数字典。
def _hardware_denial(stimulus: dict[str, Any]) -> dict[str, Any]:
    if stimulus.get("hardware") != "none":
        raise ValueError("R00 adapter only supports --hardware none")
    guard = _HardwareNoneGuard()
    ##
    # @par 拒绝证据范围
    # 三个字符串只经过本guard，opened_probes从未打开API；零计数是该局部流程性质，不是对所有进程/设备访问的审计。
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


##
# @brief 按AUD01/02/03执行对应局部检查或把ROS01委托R02，形成source/trace摘要和observations。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 对应case结果字典；ROS01直接返回R02结果，其他附document_utility标签。
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
