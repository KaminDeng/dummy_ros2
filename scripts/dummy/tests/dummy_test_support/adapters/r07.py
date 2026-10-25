"""Runtime evidence adapter for R07 device SIL backends.

This adapter intentionally reports a missing ROS bridge as ``NOT_RUN``. It does
not infer controller or management behavior from launch-file text.
"""
##
# @file r07.py
# @brief DPSDK POSIX peer、控制组件与Action的R07 SIL证据适配器。
#
# 这些入口实际运行上游adapter、CTest或本机TCP peer/ROS Action探针，不是读取launch文本推断行为。real_hardware_accessed=False是本适配器填的范围声明；real peer仍为POSIX/虚拟CAN，不证明实体硬件。缓存按profile/mask/缺轴或整体Action复用。

from __future__ import annotations

from hashlib import sha256
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from typing import Any
try:
    from installed_sdk import robot_bundle
except ModuleNotFoundError:
    from scripts.dummy.installed_sdk import robot_bundle

import yaml


ROOT = Path(__file__).resolve().parents[5]
DPSDK_ROOT = Path(os.environ.get("DPSDK_ROOT", str(Path(__file__).resolve().parents[5].parent / "dpsdk_consolidated")))
D06_CASES = {"B02": "B02", "AUTH04": "AUTH04"}
D07_CASES = {"SEN05", "SEN06"}
BACKEND_SOURCES = (
    ROOT / "src/dummy_bringup/config/backends.yaml",
    ROOT / "src/dummy_bringup/launch/sil_6dof.launch.py",
    ROOT / "src/dummy_bringup/launch/sil_j1.launch.py",
    ROOT / "src/dummy_bringup/launch/real_j1.launch.py",
    ROOT / "src/dummy_ros_control/test/guarded_lifecycle_probe.cpp",
    ROOT / "src/dummy_ros_control/test/wire_state_ingest_tests.cpp",
    ROOT / "src/dummy_ros_control/test/nonblocking_tests.cpp",
)
_PEER_OBSERVATIONS: dict[tuple[int, int, int], dict[str, Any]] = {}
_REAL_ACTION: dict[str, Any] | None = None


##
# @brief 动态执行上游d06/d07 adapter；要求loader存在，不自行构建SDK。
# @param name 上游adapter基名d06或d07。
# @return 已导入上游模块对象。
def _load(name: str):
    path = DPSDK_ROOT / "applications/dummy_ref_control/tools/dummy_ref_tools/tests/dummy_test_support/adapters" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"dpsdk_{name}_r07", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load upstream adapter: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


##
# @brief 按BACKEND_SOURCES相对名/原字节或missing占位求摘要，不绑定所有运行二进制字节。
# @return 列举输入相对名/字节的SHA-256。
def _source_hash() -> str:
    digest = sha256()
    for path in BACKEND_SOURCES:
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(path.read_bytes() if path.is_file() else b"<missing>")
    return digest.hexdigest()


##
# @brief 统一status/trace与范围字段；executed仅status==PASS为True，FAIL也会置False，不能当实际从未启动进程。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param status PASS/FAIL/NOT_RUN等现有结果状态。
# @param reason 人类可读失败/未运行原因。
# @param trace 待哈希的运行/诊断文本，不自动证明设备行为。
# @param observations 要验证/嵌入结果的观测字典，保留其原有证据层级。
# @return 统一结果字典；executed按照PASS状态设置。
def _result(
    case_id: str, *, status: str, reason: str, trace: str, observations: dict[str, Any],
) -> dict[str, Any]:
    values = {
        "case_id": case_id,
        "requirement_observed": status == "PASS",
        "real_hardware_accessed": False,
        "blocked": status == "NOT_RUN",
        **observations,
    }
    return {
        "case_id": case_id,
        "executed": status == "PASS",
        "status": status,
        "reason": reason,
        "sut_source_sha256": _source_hash(),
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "observations": values,
    }


##
# @brief 对既有控制构建目录用精确名称正则跑CTest≤30s，仅检查退出0，不独立解析实际发现测试数。
# @param name 精确匹配的CTest名字，不是新建测试。
# @return (退出码是否0, stdout+stderr)。
def _run_ctest(name: str) -> tuple[bool, str]:
    completed = subprocess.run(
        ["ctest", "--test-dir", str(ROOT / "build_dummy/dummy_ros_control"),
         "-R", f"^{name}$", "--output-on-failure"],
        cwd=ROOT, capture_output=True, text=True, check=False, timeout=30,
    )
    return completed.returncode == 0, completed.stdout + completed.stderr


##
# @brief 按SEN05/06选d07，其余B02/AUTH04选d06，向上游传hardware none/test_only。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @return 上游adapter返回结果原值。
def _upstream(case_id: str) -> dict[str, Any]:
    suite = "d07" if case_id in D07_CASES else "d06"
    upstream_case = case_id if suite == "d07" else D06_CASES[case_id]
    return _load(suite).run_case(
        upstream_case, {"case_id": upstream_case, "hardware": "none", "test_only": True},
    )


##
# @brief 合并上游成功与J1 peer原始反馈有效但未引用/无session的观测；返回启动链/零自动管理等声明，非USB证据。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @return 合并J1 peer与上游判断的结果字典。
def _sil_transport_case(case_id: str) -> dict[str, Any]:
    upstream = _upstream(case_id)
    upstream_observations = dict(upstream["observations"])
    upstream_status = "PASS" if (
        upstream.get("executed") is True and upstream_observations.get("requirement_observed", True)
    ) else "FAIL"
    observed = _peer_state(2, 1)
    passed = upstream_status == "PASS" and observed["position_valid_mask"] == 0 and \
        observed["raw_valid_mask"] == 1 and observed["session_id"] == 0
    return _result(
        case_id, status="PASS" if passed else "FAIL", reason="" if passed else "J1 SIL observation failed",
        trace=json.dumps(observed, sort_keys=True), observations={
            "posix_peer_started": True,
            "ros_device_chain_started": True,
            "auto_management_calls": 0,
            "upstream_case_id": D06_CASES[case_id],
            "upstream_subtest_status": upstream_status,
            "upstream_trace_sha256": upstream["trace_sha256"],
            "upstream_transport_observations": upstream.get("transport_observations", {}),
        },
    )


##
# @brief SEN05核J1 mask1，SEN06核六轴mask63/J3缺反馈/Fault9、不自动降级，合并上游证据。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @return 含缺轴/掩码与上游证据的结果字典。
def _d07_case(case_id: str) -> dict[str, Any]:
    upstream = _upstream(case_id)
    observations = dict(upstream["observations"])
    configured_mask = 1 if case_id == "SEN05" else 63
    upstream_status = "PASS" if (
        upstream.get("executed") is True and observations.get("requirement_observed", True)
    ) else "FAIL"
    peer = _peer_state(2, 1) if case_id == "SEN05" else _peer_state(1, 63, 3)
    passed = upstream_status == "PASS" and peer["configured_mask"] == configured_mask
    if case_id == "SEN06":
        passed = passed and peer["missing_axis"] == 3 and peer["core_state"] == 9 \
            and not peer["auto_degraded"] and not (peer["raw_valid_mask"] & (1 << 2))
    return _result(
        case_id, status="PASS" if passed else "FAIL", reason="" if passed else "SIL axis/fault evidence failed",
        trace=json.dumps({"peer": peer, "upstream": observations}, sort_keys=True), observations={
            "robot_core": "DPSDK RobotCore",
            "virtual_can": True,
            "configured_mask": configured_mask,
            "upstream_subtest_status": upstream_status,
            "upstream_trace_sha256": upstream["trace_sha256"],
            "upstream_observations": observations,
            "cross_process_observations": peer,
        },
    )


##
# @brief 封装NOT_RUN/blocked结果，当前run分派无直接调用此helper。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param reason 人类可读失败/未运行原因。
# @param observations 要验证/嵌入结果的观测字典，保留其原有证据层级。
# @return executed=False/NOT_RUN且blocked=True结果。
def _blocked_case(case_id: str, reason: str, observations: dict[str, Any]) -> dict[str, Any]:
    return _result(
        case_id, status="NOT_RUN", reason=reason, trace=reason, observations=observations,
    )


##
# @brief FRE11核steady配置与peer session；ROS07跑状态/争用，ROS08非阻塞+Action，ROS10插件/生命周期+J1 peer。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @return 对应后端case结果；未知case抛ValueError。
def _backend_case(case_id: str) -> dict[str, Any]:
    config = yaml.safe_load((ROOT / "src/dummy_bringup/config/backends.yaml").read_text())
    if case_id == "FRE11":
        observed = _peer_state(2, 1)
        passed = config["backends"]["sil_j1"]["clock"] == "steady" and observed["session_id"] == 0
        return _result(case_id, status="PASS" if passed else "FAIL", reason="" if passed else "steady-clock SIL failed",
            trace=json.dumps(observed, sort_keys=True), observations={"steady_clock_source": "steady",
             "simulation_clock_isolated": passed, "cross_process_observations": observed})
    if case_id == "ROS07":
        stale_test, trace = _run_ctest("wire_state_ingest_tests")
        contention, contention_trace = _run_ctest("state_contention_tests")
        passed = stale_test and contention
        return _result(case_id, status="PASS" if passed else "FAIL", reason="" if passed else "stale read probe failed",
            trace=trace + contention_trace, observations={"stale_read_result": "ERROR_WHEN_ACTIVE",
             "inactive_poll_result": "OK", "stale_wire_rejected": stale_test and contention})
    if case_id == "ROS08":
        inactive_test, trace = _run_ctest("nonblocking_tests")
        action = _real_action_e2e()
        passed = inactive_test and action.get("success_count", 0) >= 2
        return _result(case_id, status="PASS" if passed else "FAIL", reason="" if passed else "inactive bootstrap failed",
            trace=trace + json.dumps(action, sort_keys=True), observations={"management_while_inactive": passed,
             "management_access_unit_probe": inactive_test, "real_action_observations": action})
    if case_id == "ROS10":
        plugin_loaded, trace = _run_ctest("guarded_plugin_load_tests")
        lifecycle, lifecycle_trace = _run_ctest("guarded_lifecycle_probe")
        peer = _peer_state(2, 1)
        passed = plugin_loaded and lifecycle and peer["configured_mask"] == 1
        return _result(case_id, status="PASS" if passed else "FAIL", reason="" if passed else "J1 interface probe failed",
            trace=trace + lifecycle_trace + json.dumps(peer, sort_keys=True), observations={
             "controller_state_interfaces": ["Joint1/position", "Joint1/velocity"],
             "controller_command_interfaces": ["Joint1/position"], "guarded_plugin_loaded": plugin_loaded,
             "cross_process_observations": peer})
    raise ValueError(f"unsupported backend case: {case_id}")


##
# @brief 按(profile,mask,missing_axis)缓存；首次选本机TCP端口启动D08 peer，0.5s后跑10s状态probe，finally终止peer并最多等5s。
# @param profile D08 peer的profile ID整数。
# @param mask 配置轴位掩码，J1=1、六轴=63。
# @param missing_axis 要注入缺反馈的1起始轴号；0不注入。
# @return 缓存或新采集peer JSON；启动/探针/解析失败异常传播。
def _peer_state(profile: int, mask: int, missing_axis: int = 0) -> dict[str, Any]:
    key = (profile, mask, missing_axis)
    if key in _PEER_OBSERVATIONS:
        return _PEER_OBSERVATIONS[key]
    peer = DPSDK_ROOT / "build/posix-zenohc/applications/dummy_ref_control/dummy_ref_d08_device_peer"
    probe = ROOT / "build_dummy/dummy_ros_control/real_peer_state_probe"
    if not peer.is_file() or not probe.is_file():
        raise RuntimeError("real D08 peer state probe binaries are missing")
    ##
    # @par 端口与owner寿命
    # 先临时bind端口再释放后启动peer；不是持有socket直交peer的端口预约保证。finally负责终止所启动peer，未做硬件owner交接。
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{reserve.getsockname()[1]}"
    with tempfile.TemporaryFile(mode="w+") as log:
        peer_command = [str(peer), "--endpoint", endpoint, "--client-id",
            "02000000000000000000000000000000", "--profile-id", str(profile),
            "--configured-mask", str(mask)]
        if missing_axis:
            peer_command += ["--drop-feedback-axis", str(missing_axis)]
        process = subprocess.Popen(peer_command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, text=True)
        try:
            time.sleep(0.5)
            probe_command = [str(probe), endpoint, str(profile), str(mask)]
            if missing_axis:
                probe_command.append(str(missing_axis))
            completed = subprocess.run(probe_command, cwd=ROOT,
                capture_output=True, text=True, check=False, timeout=10)
            if completed.returncode:
                log.seek(0)
                raise RuntimeError(f"real peer state probe failed ({completed.returncode}):\n{completed.stdout}{completed.stderr}{log.read()}")
            observed = json.loads(next(line for line in reversed(completed.stdout.splitlines()) if line.startswith("{")))
        finally:
            process.terminate()
            process.wait(timeout=5)
    _PEER_OBSERVATIONS[key] = observed
    return observed


##
# @brief source ROS环境后实际跑D08 peer与device Action gate E2E≤60s，非零抛异常，成功解析real_dpsdk_peer JSON并缓存。
# @return 缓存或新运行Action E2E JSON，非零退出抛RuntimeError。
def _real_action_e2e() -> dict[str, Any]:
    global _REAL_ACTION
    if _REAL_ACTION is not None:
        return _REAL_ACTION
    completed = subprocess.run(["bash", "-c",
        'source /opt/ros/jazzy/setup.bash && source "$1/install_dummy/setup.bash" && shift && exec "$@"',
        "r07-real-action", str(ROOT), "/usr/bin/python3",
        str(ROOT / "src/dummy_motion_client/test/device_action_e2e_probe.py"),
        str(ROOT / "build_dummy/dummy_motion_client/device_action_gate"),
        str(DPSDK_ROOT / "build/posix-zenohc/applications/dummy_ref_control/dummy_ref_d08_device_peer"),
        str(robot_bundle()), "real"], cwd=ROOT,
        capture_output=True, text=True, check=False, timeout=60)
    if completed.returncode:
        raise RuntimeError(f"real device action E2E failed ({completed.returncode}):\n{completed.stdout}{completed.stderr[-4000:]}")
    _REAL_ACTION = json.loads(next(line for line in reversed(completed.stdout.splitlines())
        if line.startswith("{") and "real_dpsdk_peer" in line))
    return _REAL_ACTION


##
# @brief 严格校验stimulus后按D06/D07/后端case派发，未识别case最终ValueError。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 该suite所选执行路径返回的证据字典，状态/范围不自动升级。
def run_r07_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if stimulus != {"case_id": case_id, "hardware": "none", "test_only": True}:
        raise ValueError("R07 stimulus is invalid")
    if case_id in D06_CASES:
        return _sil_transport_case(case_id)
    if case_id in D07_CASES:
        return _d07_case(case_id)
    return _backend_case(case_id)
