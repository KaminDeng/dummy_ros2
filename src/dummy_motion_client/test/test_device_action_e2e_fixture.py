"""离线验证真实 DPSDK peer 的测试夹具路径。"""
##
# @file test_device_action_e2e_fixture.py
# @brief SIL fixture能力/路径与真实本机PTY桥的混合回归。
#
# 路径/hash/URDF多为离线检查，但PTY用例实际启动原生serial_pty_bridge、线程/管道与短等待，自检方法实际执行C05 peer --self-test；不启动设备Gate或实体USB。SIL输入为明确安装SDK机器人包，不能当正式设备activation。

import importlib.util
import ctypes
import io
import json
import hashlib
import shutil
import os
from pathlib import Path
import select
import subprocess
import tempfile
import threading
import time
import tty
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest


TEST_DIR = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("device_action_e2e_probe", TEST_DIR / "device_action_e2e_probe.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


##
# @brief 只核CMake引用两个可配置D08 peer变量、不硬编码旧v2-tests路径，不真正执行CTest。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_ctest_uses_v2_fixture_under_configured_dpsdk_root():
    cmake = (TEST_DIR.parent / "CMakeLists.txt").read_text()
    assert "build_dummy/v2-tests" not in cmake
    assert "${DUMMY_D08_J1_PEER_EXECUTABLE}" in cmake
    assert "${DUMMY_D08_SIX_AXIS_PEER_EXECUTABLE}" in cmake


##
# @brief 核CMake smoke入口、pin fixture与六轴URDF/profile1/mask63/±0.3rad，和canonical摘要不同是预期隔离。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_six_axis_smoke_uses_all_axes_and_sil_profile():
    cmake = (TEST_DIR.parent / "CMakeLists.txt").read_text()
    assert "device_action_six_axis_smoke_probe" in cmake
    assert "real_six_axis_smoke" in cmake
    assert "device_motion_six_axis_smoke_probe" in cmake
    bundle = probe.installed_bundle()
    dpsdk_manifest = probe.installed_bundle() / "manifest.json"
    capability = probe.real_peer_capability(bundle, "sil_6dof")
    description = ET.fromstring(probe.urdf("serial//dev/pts/1#baudrate=3000000", capability, 6))
    joint_names = [joint.attrib["name"] for joint in description.findall("./ros2_control/joint")]
    parameters = {entry.attrib["name"]: entry.text for entry in description.findall("./ros2_control/hardware/param")}

    assert joint_names == [f"Joint{axis}" for axis in range(1, 7)]
    assert parameters["profile_id"] == "1"
    assert parameters["configured_mask"] == "63"
    assert capability["profile_sha256"] != probe.real_peer_capability(bundle)["profile_sha256"]
    assert capability["contract_sha256"] == json.loads(dpsdk_manifest.read_text())["contract_sha256"]
    assert capability["position_min_rad"] == [-0.3] * 6
    assert capability["position_max_rad"] == [0.3] * 6
    points = probe.smooth_motion_points([0] * 6, probe.SIX_AXIS_TARGET)
    assert list(points[-1].positions) == list(probe.SIX_AXIS_TARGET)


##
# @brief 即旁边有其它profile的peer文件，缺指定路径仍FileNotFoundError，不搜索误用。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_missing_fixture_reports_expected_path_without_other_profile_fallback():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        other = root / "build_dummy/qualification/host-cadence/applications/dummy_ref_control/dummy_ref_d08_device_peer"
        other.parent.mkdir(parents=True)
        other.write_text("other profile")
        expected = root / "build_dummy/v2-tests/applications/dummy_ref_control/dummy_ref_d08_device_peer"

        with pytest.raises(FileNotFoundError, match="DPSDK D08 peer fixture") as error:
            probe.validate_peer_fixture(expected)

        assert str(expected) in str(error.value)


##
# @brief 存在文本文件但名字device_action_gate，先按peer名字拒绝。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_explicit_fixture_rejects_wrong_executable_name():
    with tempfile.TemporaryDirectory() as directory:
        other = Path(directory) / "device_action_gate"
        other.write_text("wrong executable")

        with pytest.raises(ValueError, match="dummy_ref_d08_device_peer"):
            probe.validate_peer_fixture(other)


##
# @brief 仅名字/文件/执行位满足即可过路径门，fixture内容不是可用真实peer二进制，不运行它。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_valid_executable_fixture_is_accepted():
    with tempfile.TemporaryDirectory() as directory:
        peer = Path(directory) / "dummy_ref_d08_device_peer"
        peer.write_text("fixture")
        peer.chmod(0o755)

        assert probe.validate_peer_fixture(peer) == peer


##
# @brief 正确名字mode644仍PermissionError，不延迟到Popen。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_non_executable_fixture_fails_before_launch():
    with tempfile.TemporaryDirectory() as directory:
        peer = Path(directory) / "dummy_ref_d08_device_peer"
        peer.write_text("fixture")
        peer.chmod(0o644)

        with pytest.raises(PermissionError, match="not executable"):
            probe.validate_peer_fixture(peer)


##
# @brief 显式domain187原样保留，不擅改隔离域。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_probe_honors_configured_ros_domain_id(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "187")
    assert probe.selected_ros_domain_id() == "187"


##
# @brief 无显式域时PID派生值在0..232，未证明与其它进程无冲突。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_probe_chooses_valid_domain_when_unset(monkeypatch):
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    assert 0 <= int(probe.selected_ros_domain_id()) <= 232


##
# @brief 真实两PTY/原生桥分别传host-to-peer和peer-to-host字节，非真实USB链路。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_forwards_bytes_in_both_directions():
    with probe.serial_peer_endpoints() as (host_endpoint, peer_endpoint):
        assert host_endpoint != peer_endpoint
        assert host_endpoint.startswith("serial//dev/pts/")
        assert peer_endpoint.startswith("serial//dev/pts/")
        assert host_endpoint.endswith("#baudrate=3000000")
        assert peer_endpoint.endswith("#baudrate=3000000")
        host = os.open(host_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        peer = os.open(peer_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        try:
            tty.setraw(host)
            tty.setraw(peer)
            os.write(host, b"host-to-peer")
            assert select.select([peer], [], [], 1)[0]
            assert os.read(peer, 12) == b"host-to-peer"
            os.write(peer, b"peer-to-host")
            assert select.select([host], [], [], 1)[0]
            assert os.read(host, 12) == b"peer-to-host"
        finally:
            os.close(host)
            os.close(peer)


##
# @brief 线程持续灌peer方向至背压时反向reverse仍在0.5s可读，核双向桥独立推进。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_keeps_reverse_direction_live_under_backpressure():
    with probe.serial_peer_endpoints() as (host_endpoint, peer_endpoint):
        host = os.open(host_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        peer = os.open(peer_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        stop = threading.Event()
        try:
            tty.setraw(host)
            tty.setraw(peer)
            os.set_blocking(peer, False)

            ##
            # @par 内部回调 flood
            # 后台循环向非阻塞PTY写4096字节，满时短sleep；stop Event结束，finally join并关fd。
            # 返回：None；stop后退出线程。
            def flood():
                while not stop.is_set():
                    try:
                        os.write(peer, b"x" * 4096)
                    except BlockingIOError:
                        time.sleep(0.001)

            writer = threading.Thread(target=flood, daemon=True)
            writer.start()
            time.sleep(0.1)
            os.write(host, b"reverse")
            assert select.select([peer], [], [], 0.5)[0]
            assert os.read(peer, 7) == b"reverse"
        finally:
            stop.set()
            writer.join(timeout=1)
            os.close(host)
            os.close(peer)


##
# @brief 真实桥转发后核字节数与单调入出时间及chunk at=ingress，不给wire序号或策略资格。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_reports_bidirectional_transfer_timing():
    diagnostics = {}
    with probe.serial_peer_endpoints(diagnostics) as (host_endpoint, peer_endpoint):
        host = os.open(host_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        peer = os.open(peer_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        try:
            tty.setraw(host)
            tty.setraw(peer)
            os.write(host, b"command")
            assert select.select([peer], [], [], 1)[0]
            assert os.read(peer, 7) == b"command"
            os.write(peer, b"state")
            assert select.select([host], [], [], 1)[0]
            assert os.read(host, 5) == b"state"
        finally:
            os.close(host)
            os.close(peer)

    assert diagnostics["host_to_peer"]["bytes"] >= 7
    assert diagnostics["peer_to_host"]["bytes"] >= 5
    assert diagnostics["host_to_peer"]["last_at_us"] > 0
    assert diagnostics["peer_to_host"]["last_at_us"] > 0
    for direction in ("host_to_peer", "peer_to_host"):
        chunk = diagnostics[direction]["recent_chunks"][-1]
        assert chunk["ingress_at_us"] > 0
        assert chunk["egress_at_us"] >= chunk["ingress_at_us"]
        assert chunk["at_us"] == chunk["ingress_at_us"]


##
# @brief fake endpoints/probe注入单peer事件后失败，必须写INCOMPLETE/strict false trace并保原断言，不运行ROS。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @param capsys pytest stdout/stderr捕获fixture。
# @param tmp_path pytest临时目录Path。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_six_axis_bootstrap_failure_emits_incomplete_latency_trace(monkeypatch, capsys, tmp_path):
    from contextlib import contextmanager

    ##
    # @par 内部回调 fake_endpoints
    # 只yield人工serial端点字符串，finally补clock声明，无PTY/进程。
    # 参数 diagnostics：可变桥统计字典，正常退出时更新；None新建内部dict。
    # 参数 _threshold：fake endpoints接收但忽略的gap阈值。
    # 返回：yield两个人工端点字符串，finally补clock声明。
    @contextmanager
    def fake_endpoints(diagnostics, _threshold):
        try:
            yield ("serial/host", "serial/peer")
        finally:
            diagnostics.update({"clock": "steady_monotonic_us"})

    ##
    # @par 内部回调 fail_probe
    # 向共享trace_capture加一peer标记再抛bootstrap未达Hold，模拟失败证据保留。
    # 参数 trace_capture：失败前要追加人工标记的共享list。
    # 参数 _args：fake probe忽略的位置参数。
    # 参数 _kwargs：fake probe忽略的关键字参数。
    # 返回：无正常返回；抛bootstrap未达Hold的AssertionError。
    def fail_probe(*_args, trace_capture, **_kwargs):
        trace_capture.append({"stage": "peer_state_publish", "sequence": 7,
                              "at_us": 1000, "clock": "steady_monotonic_us"})
        raise AssertionError("bootstrap did not reach HOLD")

    monkeypatch.setattr(probe, "fixture_capability", lambda *_args: {"state_publish_period_us": 50000})
    monkeypatch.setattr(probe, "serial_peer_endpoints", fake_endpoints)
    monkeypatch.setattr(probe, "run_probe", fail_probe)
    monkeypatch.setattr(probe.sys, "argv", ["probe", "/tmp/gate", "/tmp/peer",
                                                "/tmp/bundle", "six_axis_smoke"])
    monkeypatch.setenv("DUMMY_E2E_TRANSPORT", "serial")
    report_path = tmp_path / "c05-trace.json"
    monkeypatch.setenv("DUMMY_C05_TRACE_OUTPUT", str(report_path))

    with pytest.raises(AssertionError, match="bootstrap did not reach HOLD"):
        probe.main()

    report_line = next(line for line in capsys.readouterr().err.splitlines()
                       if line.startswith('{"bridge"'))
    report = json.loads(report_line)
    assert report["event"] == "c05_latency_trace"
    assert report["status"] == "INCOMPLETE"
    assert report["strict_50ms_pass"] is False
    assert report["state"][0]["sequence"] == 7
    assert json.loads(report_path.read_text()) == report


##
# @brief ctypes PyDLL usleep实际持GIL200ms，桥进程仍在此前转发，说明转发不依赖Python回调泵，不是ROS时延保证。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_forwards_while_probe_holds_python_interpreter():
    diagnostics = {}
    with probe.serial_peer_endpoints(diagnostics) as (host_endpoint, peer_endpoint):
        host = os.open(host_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        peer = os.open(peer_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        try:
            tty.setraw(host)
            tty.setraw(peer)
            time.sleep(0.05)
            os.write(host, b"during-probe")
            ctypes.PyDLL(None).usleep(200000)
            resumed_at_us = time.monotonic_ns() // 1000
            assert select.select([peer], [], [], 1)[0]
            assert os.read(peer, 12) == b"during-probe"
        finally:
            os.close(host)
            os.close(peer)

    assert diagnostics["host_to_peer"]["last_at_us"] < resumed_at_us - 100000


##
# @brief 只核桥文件名/存在，不执行新构建或验证SHA。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_uses_native_forwarder():
    executable = probe.serial_bridge_executable()
    assert executable.name == "serial_pty_bridge"
    assert executable.is_file()


##
# @brief 环境显式桥路径优先，纯路径选择。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_honors_cmake_bridge_path(monkeypatch):
    monkeypatch.setenv("DUMMY_SERIAL_BRIDGE_EXECUTABLE", "/tmp/offline-serial-bridge")
    assert probe.serial_bridge_executable() == Path("/tmp/offline-serial-bridge")


##
# @brief 真实开PTY但Popen替身失败，finally应关全部fd，测试自身补清理。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_closes_all_ptys_when_bridge_spawn_fails(monkeypatch):
    opened = []
    original = probe.pty.openpty

    ##
    # @par 内部回调 tracked_openpty
    # 调用真正openpty并记录两fd用于泄漏检查，不打开实体设备。
    # 返回：真实(master,slave) fd元组，已登记。
    def tracked_openpty():
        descriptors = original()
        opened.extend(descriptors)
        return descriptors

    monkeypatch.setattr(probe.pty, "openpty", tracked_openpty)
    monkeypatch.setattr(probe.subprocess, "Popen", lambda *args, **kwargs: (_ for _ in ()).throw(OSError("spawn failed")))
    try:
        with pytest.raises(OSError, match="spawn failed"):
            with probe.serial_peer_endpoints():
                pass
        assert all(not descriptor_open(descriptor) for descriptor in opened)
    finally:
        for descriptor in opened:
            if descriptor_open(descriptor):
                os.close(descriptor)


##
# @brief 第二openpty注入异常时首对fd也必须关闭。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_closes_first_pty_when_second_open_fails(monkeypatch):
    opened = []
    original = probe.pty.openpty

    ##
    # @par 内部回调 fail_second_open
    # 首调用真openpty并记录，后续总抛OSError，隔离部分创建失败。
    # 返回：首调用fd对，第二调用抛OSError。
    def fail_second_open():
        if opened:
            raise OSError("second PTY failed")
        descriptors = original()
        opened.extend(descriptors)
        return descriptors

    monkeypatch.setattr(probe.pty, "openpty", fail_second_open)
    try:
        with pytest.raises(OSError, match="second PTY failed"):
            with probe.serial_peer_endpoints():
                pass
        assert all(not descriptor_open(descriptor) for descriptor in opened)
    finally:
        for descriptor in opened:
            if descriptor_open(descriptor):
                os.close(descriptor)


##
# @brief 用fstat判断本地fd当前是否仍开，OSError归False；不读串口内容。
# @param descriptor 本地PTY fd整数，可能已关闭。
# @return fstat成功True、OSError False。
def descriptor_open(descriptor):
    try:
        os.fstat(descriptor)
        return True
    except OSError:
        return False


##
# @brief HungBridge模拟stop/terminate两次超时，必须kill且关PTY，异常明确bridge退出非零。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_kills_bridge_after_second_shutdown_timeout(monkeypatch):
    opened = []
    original = probe.pty.openpty

    ##
    # @par 内部回调 tracked_openpty
    # 调用真正openpty并记录两fd用于泄漏检查，不打开实体设备。
    # 返回：真实(master,slave) fd元组，已登记。
    def tracked_openpty():
        descriptors = original()
        opened.extend(descriptors)
        return descriptors

    ##
    # @par 内部类型 HungBridge
    # 超时/kill标志的桥进程替身，不启动真实桥。
    class HungBridge:
        stdout = io.StringIO("READY\n")
        returncode = None
        calls = 0
        killed = False

        ##
        # @par 内部回调 communicate
        # 前两次抛2s TimeoutExpired，第三次给空JSON输出；只模拟Popen等待。
        # 参数 kwargs：Popen.communicate关键字参数，本替身按调用次数注入超时。
        # 返回：第三次返回空JSON/空stderr，前两次抛TimeoutExpired。
        def communicate(self, **kwargs):
            self.calls += 1
            if self.calls < 3:
                raise subprocess.TimeoutExpired("serial bridge", 2)
            return "{}", ""

        ##
        # @par 内部回调 terminate
        # 空操作，模拟拒绝温和终止。
        # 返回：None，模拟未响应terminate。
        def terminate(self):
            pass

        ##
        # @par 内部回调 kill
        # 置killed及returncode−9，不向真实PID发信号。
        # 返回：None；仅设置killed和−9退出码。
        def kill(self):
            self.killed = True
            self.returncode = -9

    bridge = HungBridge()
    monkeypatch.setattr(probe.pty, "openpty", tracked_openpty)
    monkeypatch.setattr(probe.subprocess, "Popen", lambda *args, **kwargs: bridge)
    monkeypatch.setattr(probe.select, "select", lambda readable, *args: (readable, [], []))
    try:
        with pytest.raises(RuntimeError, match="serial fixture bridge exited"):
            with probe.serial_peer_endpoints():
                pass
        assert bridge.killed
        assert all(not descriptor_open(descriptor) for descriptor in opened)
    finally:
        for descriptor in opened:
            if descriptor_open(descriptor):
                os.close(descriptor)


##
# @brief 互异带符号目标使交换J1/J2可检出；只核末命令/反馈逐轴，不是全运动链。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_six_axis_evidence_rejects_swapped_signed_joints():
    target = probe.SIX_AXIS_TARGET
    assert len(target) == len(set(target)) == 6
    feedback = [SimpleNamespace(actual=SimpleNamespace(positions=target))]
    commands = [{"position": list(target)}]
    probe.validate_six_axis_evidence(feedback, commands, target)
    swapped = list(target)
    swapped[0], swapped[1] = swapped[1], swapped[0]
    with pytest.raises(AssertionError, match="axis 1"):
        probe.validate_six_axis_evidence(feedback, [{"position": swapped}], target)
    with pytest.raises(AssertionError, match="axis 1"):
        probe.validate_six_axis_evidence(
            [SimpleNamespace(actual=SimpleNamespace(positions=swapped))], commands, target)


##
# @brief 真实桥两字节间隔30ms，核≥20ms最大gap及前后chunk1/2，chunk编号不是wire序号。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_serial_fixture_identifies_chunks_around_maximum_gap():
    diagnostics = {}
    with probe.serial_peer_endpoints(diagnostics, gap_threshold_us=20000) as (_, peer_endpoint):
        peer = os.open(peer_endpoint.removeprefix("serial/").split("#", 1)[0], os.O_RDWR)
        try:
            tty.setraw(peer)
            os.write(peer, b"a")
            time.sleep(0.03)
            os.write(peer, b"b")
            time.sleep(0.08)
        finally:
            os.close(peer)

    observed = diagnostics["peer_to_host"]
    assert observed["max_gap_us"] >= 20000
    assert observed["max_gap_before_chunk"] == 1
    assert observed["max_gap_after_chunk"] == 2
    assert observed["max_gap_before_at_us"] < observed["max_gap_after_at_us"]
    assert observed["long_gaps"][0]["before_seq"] == 1
    assert observed["long_gaps"][0]["after_seq"] == 2


##
# @brief 安装SDK pin/profile/model给能力与URDF身份参数一致，policy hash故意等profile原SHA，非真实compiled审批。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_real_peer_capability_comes_from_verified_bundle():
    bundle = probe.installed_bundle()
    capability = probe.real_peer_capability(bundle)
    manifest = probe.json.loads((bundle / "manifest.json").read_text())
    payloads = {entry["path"]: entry["sha256"] for entry in manifest["payloads"]}
    profile = probe.yaml.safe_load((bundle / "profiles/sil_j1.yaml").read_text())
    model = probe.yaml.safe_load((bundle / "model/model.yaml").read_text())

    assert capability["contract_sha256"] == manifest["contract_sha256"]
    assert capability["model_sha256"] == payloads["model/model.yaml"]
    assert capability["profile_sha256"] == payloads["profiles/sil_j1.yaml"]
    assert capability["compiled_policy_sha256"] == capability["profile_sha256"]
    assert capability["control_period_us"] == profile["timing"]["control_period_us"]
    assert capability["state_publish_period_us"] == profile["timing"]["state_publish_period_us"]
    assert capability["position_min_rad"] == [joint["model_limits_rad"][0] for joint in model["joints"]]
    assert capability["position_max_rad"] == [joint["model_limits_rad"][1] for joint in model["joints"]]
    assert capability["velocity_max_rad_s"] == [profile["simulation"]["velocity_limit_rad_s"]] * 6
    assert capability["acceleration_max_rad_s2"] == [profile["simulation"]["acceleration_limit_rad_s2"]] * 6

    hardware = ET.fromstring(probe.urdf("tcp/127.0.0.1:12345", capability)).find("./ros2_control/hardware")
    params = {item.attrib["name"]: item.text for item in hardware.findall("param")}
    for key in ("contract_sha256", "model_sha256", "profile_sha256", "compiled_policy_sha256"):
        assert params[key] == capability[key]
    for key in ("control_period_us", "state_publish_period_us"):
        assert params[key] == str(capability[key])


##
# @brief 副本改注释并重算所有SHA自洽仍不等strict pin，必须untrusted拒绝，不改原fixture。
# @param tmp_path pytest临时目录Path。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_foreign_hash_valid_sil_bundle_is_not_a_trusted_offline_fixture(tmp_path):
    # 从精确SIL fixture构造hash自洽但不在既有strict pin内的独立负例。
    source = probe.installed_bundle()
    bundle = tmp_path / "foreign_bundle"
    shutil.copytree(source, bundle)
    payload = bundle / "profiles/sil_j1.yaml"
    payload.write_bytes(payload.read_bytes() + b"\n# foreign hash-valid fixture\n")
    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    for entry in manifest["payloads"]:
        entry["sha256"] = hashlib.sha256((bundle / entry["path"]).read_bytes()).hexdigest()
    listing = [f"{entry['path']}:{entry['sha256']}" for entry in manifest["payloads"]]
    assert listing == sorted(listing)
    manifest["payload_sorted_sha256"] = hashlib.sha256("\n".join(listing).encode()).hexdigest()
    assert manifest["payload_sorted_sha256"] != json.loads((probe.installed_bundle() / "manifest.json").read_text())["payload_sorted_sha256"]
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="untrusted SIL fixture"):
        probe.real_peer_capability(bundle)


##
# @brief 只伪造manifest聚合f串必须aggregate mismatch。
# @param tmp_path pytest临时目录Path。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_current_sil_fixture_rejects_forged_aggregate_hash(tmp_path):
    bundle = tmp_path / "forged_bundle"
    shutil.copytree(probe.installed_bundle(), bundle)
    path = bundle / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["payload_sorted_sha256"] = "f" * 64
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="aggregate hash mismatch"):
        probe.real_peer_capability(bundle)


##
# @brief profile原字节改而manifest未更新须payload SHA拒绝。
# @param tmp_path pytest临时目录Path。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_current_sil_fixture_rejects_tampered_payload(tmp_path):
    bundle = tmp_path / "tampered_bundle"
    shutil.copytree(probe.installed_bundle(), bundle)
    path = bundle / "profiles/sil_j1.yaml"
    path.write_bytes(path.read_bytes() + b"\n# unlisted tamper\n")
    with pytest.raises(ValueError, match="payload hash mismatch"):
        probe.real_peer_capability(bundle)


##
# @brief 真实运行指定C05 --self-test，核50ms窗口前有效/之后拒绝报告；未在此核恰deadline布尔。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_c05_ticket_expires_at_profile_window():
    executable = Path(os.environ["DUMMY_C05_PEER_EXECUTABLE"])
    control_library = Path(os.environ["DUMMY_ROS_CONTROL_LIBRARY_DIR"])
    completed = subprocess.run([str(executable), "--self-test"], capture_output=True,
                               text=True, check=True, env={**os.environ, "LD_LIBRARY_PATH":
                                   str(control_library) + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")})
    evidence = json.loads(completed.stdout)
    assert evidence["ticket_window_us"] == 50000
    assert evidence["before_deadline_valid"] is True
    assert evidence["after_deadline_rejected"] is True


##
# @brief 显式50ms候选只改State周期，原capability仍10ms，不改变控制节奏或正式合同。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_50ms_candidate_changes_only_runtime_state_period(monkeypatch):
    bundle = probe.installed_bundle()
    baseline = probe.real_peer_capability(bundle)
    monkeypatch.setenv("DUMMY_E2E_STATE_PERIOD_US", "50000")

    candidate = probe.fixture_capability(bundle)

    assert candidate == {**baseline, "state_publish_period_us": 50000}
    assert baseline["state_publish_period_us"] == 10000


##
# @brief 显式20ms同样只改运行State周期，保全部静态身份。
# @param monkeypatch pytest属性/环境替换fixture，退出作用域恢复。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_20ms_candidate_changes_only_runtime_state_period(monkeypatch):
    bundle = probe.installed_bundle()
    baseline = probe.real_peer_capability(bundle)
    monkeypatch.setenv("DUMMY_E2E_STATE_PERIOD_US", "20000")

    assert probe.fixture_capability(bundle) == {**baseline, "state_publish_period_us": 20000}


##
# @brief 10ms控制周期算100Hz，独立于运行State候选20/50ms。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_controller_update_rate_matches_sil_control_period():
    bundle = probe.installed_bundle()
    capability = probe.real_peer_capability(bundle)

    assert capability["control_period_us"] == 10000
    assert probe.controller_update_rate(capability) == 100


##
# @brief 真实--self-test输出PENDING码与当前合同枚举对应；方法历史v24名称不代表只接受旧布局。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_synthetic_peer_pending_result_matches_current_constant():
    root = TEST_DIR.parents[2]
    peer = Path(os.environ["DUMMY_C05_PEER_EXECUTABLE"])
    library_path = Path(os.environ["DUMMY_ROS_CONTROL_LIBRARY_DIR"])
    env = {**os.environ, "LD_LIBRARY_PATH": str(library_path) + os.pathsep +
           os.environ.get("LD_LIBRARY_PATH", "")}
    completed = subprocess.run([str(peer), "--self-test"], capture_output=True,
                               text=True, check=True, env=env)
    event = json.loads(completed.stdout)
    constants = json.loads((probe.installed_bundle() / "protocol/include/protocol_constants.json").read_text())

    assert event["pending_result_code"] == constants["resources"]["dpsdk_interfaces/msg/DummyProtocolV2"]["RESULT_CODE_PENDING"]


##
# @brief 对50ms输入点用control_dt10ms作保守差分刺激，核速度/加速度小于fixture上限；不是完整JTC重采样验证。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_smooth_goal_respects_sil_command_derivative_limits():
    bundle = probe.installed_bundle()
    capability = probe.real_peer_capability(bundle)
    control_dt = capability["control_period_us"] / 1000000
    points = probe.smooth_motion_points(0.0, 0.05, duration_s=5)
    previous_position = 0.0
    previous_velocity = 0.0
    for point in points:
        velocity = (point.positions[0] - previous_position) / control_dt
        acceleration = (velocity - previous_velocity) / control_dt
        assert abs(velocity) < capability["velocity_max_rad_s"][0]
        assert abs(acceleration) < capability["acceleration_max_rad_s2"][0]
        previous_position = point.positions[0]
        previous_velocity = velocity

    assert list(points[-1].positions) == [0.05]
    assert points[-1].time_from_start.sec == 5


##
# @brief 人工合并/插值某些样本的差分加速度仍有余量，不覆盖任意调度延迟。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_smooth_goal_keeps_derivative_margin_after_coalesced_ros_sample():
    bundle = probe.installed_bundle()
    capability = probe.real_peer_capability(bundle)
    control_dt = capability["control_period_us"] / 1000000
    points = probe.smooth_motion_points(0.0, 0.05)
    for index in range(1, len(points) - 3):
        prior = points[index - 1].positions[0]
        current = points[index].positions[0]
        later = points[index + 2].positions[0] + 0.3 * (
            points[index + 3].positions[0] - points[index + 2].positions[0])
        prior_velocity = (current - prior) / control_dt
        coalesced_velocity = (later - current) / (2 * control_dt)
        acceleration = (coalesced_velocity - prior_velocity) / (2 * control_dt)
        assert abs(acceleration) < capability["acceleration_max_rad_s2"][0]


##
# @brief 临时JSONL五条命令只保末两seq3/4，不判真实设备接纳。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_peer_command_tail_keeps_only_recent_accepted_commands():
    with tempfile.TemporaryDirectory() as directory:
        evidence = Path(directory) / "peer.jsonl"
        evidence.write_text("\n".join(json.dumps({"event": "command", "producer_seq": seq,
                                                     "position": [seq / 1000]}) for seq in range(5)) + "\n")

        tail = probe.peer_command_tail(evidence, limit=2)

        assert [event["producer_seq"] for event in tail] == [3, 4]


##
# @brief real_smoke目标0.02rad，real默认0.05rad，纯选择函数。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_smoke_goal_is_distinct_from_full_e2e_goal():
    assert probe.first_goal_target("real_smoke") == 0.02
    assert probe.first_goal_target("real") == 0.05
