"""真实六轴栈的进程所有权测试；不启动 ROS 或访问真实设备。"""
##
# @file test_run_real_6dof_session.py
# @brief 会话单owner锁、进程组清理、路径与CLI预算的本机测试。
#
# 多数Popen/ps/fuser/signal调用是替身，但tty fixture真实openpty，锁是真实文件锁，两个shutdown用例实际启动Python父子进程并发本机信号。生成脚本文本原样保留；没有ROS launch或实体USB/机械动作，进程退出也不是设备Stop raw确认。

import importlib.util
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time

import pytest


MODULE = Path(__file__).resolve().parents[1] / "run_real_6dof_session.py"
SPEC = importlib.util.spec_from_file_location("run_real_6dof_session", MODULE)
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


##
# @brief 固定PID及可注入wait结果的子进程替身，不创建系统进程。
class FakeProcess:
    pid = 424242

    ##
    # @brief 保存wait返回值/异常与wait_calls，PID固定424242是替身，不代表当前系统进程。
    # @param result FakeFuture/Process存储的原结果或待抛BaseException。
    # @return None；初始化本测试对象字段，不做真实连接。
    def __init__(self, result=0):
        self.result = result
        self.wait_calls = []

    ##
    # @brief 记录timeout后返回result或重抛注入BaseException，不真实等待。
    # @param timeout 本次允许等待秒数，替身不真实等待。
    # @return 存储result；若为BaseException则抛出，不真正等待。
    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        if isinstance(self.result, BaseException):
            raise self.result
        return self.result


##
# @brief 实际openpty创建本机伪终端，yield slave Path，finally关闭两个fd，不打开设备串口。
# @return yield真实本机PTY slave Path；退出关闭fd。
@pytest.fixture
def tty():
    master, slave = pty.openpty()
    try:
        yield Path(os.ttyname(slave))
    finally:
        os.close(slave)
        os.close(master)


##
# @brief 同临时锁文件的第二owner_lock必须SessionError，验证进程内争用，非全部系统设备权限隔离。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_lock_rejects_second_owner(tmp_path):
    lock_path = tmp_path / "owner.lock"
    with runner.owner_lock(lock_path):
        with pytest.raises(runner.SessionError, match="owner"):
            with runner.owner_lock(lock_path):
                pass


##
# @brief owner检查替身返回tty/process冲突，Popen不得触达。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param tty 真实本机PTY slave Path，绝非实体USB串口。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param conflict 人工外部owner类型tty或process。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("conflict", ["tty", "process"])
def test_preflight_rejects_external_owner_without_spawning(tmp_path, tty, monkeypatch, conflict):
    monkeypatch.setattr(runner, "inspect_owners", lambda path, owned_pgid=None: [conflict])
    monkeypatch.setattr(runner.subprocess, "Popen", lambda *a, **k: pytest.fail("spawned"))
    with pytest.raises(runner.SessionError, match="occupied"):
        runner.run_session(["unused"], tty, tmp_path / "owner.lock")


##
# @brief 真实PTY路径但fuser/pgrep输出是替身，核命令形状与owner字符串解析。
# @param tty 真实本机PTY slave Path，绝非实体USB串口。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_inspection_uses_fuser_and_pgrep_on_pty(tty, monkeypatch):
    calls = []

    ##
    # @par 内部回调 run
    # 记录fuser/pgrep argv并返回预置CompletedProcess，不实际查本机owner。
    # 参数 command：fuser/pgrep待调用argv，在回调内替代结果。
    # 参数 kwargs：原样记录/转交的关键字选项，不自行启动硬件。
    # 返回：预置CompletedProcess。
    def run(command, **kwargs):
        calls.append(command)
        if command[0] == "fuser":
            return subprocess.CompletedProcess(command, 0, "987\n", "")
        return subprocess.CompletedProcess(command, 0, "123 device_action_gate\n", "")

    monkeypatch.setattr(runner.subprocess, "run", run)
    assert runner.inspect_owners(tty) == ["tty:987", "process:123"]
    assert calls[0] == ["fuser", str(tty)]
    assert calls[1][:2] == ["pgrep", "-af"]


##
# @brief fake leader退出后子成员仍在，按SIGINT→SIGTERM清自己的组，启动须start_new_session，postflight排除本组。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param tty 真实本机PTY slave Path，绝非实体USB串口。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_leader_exits_but_group_child_requires_term(tmp_path, tty, monkeypatch):
    process = FakeProcess()
    spawned = []
    signals = []
    members = iter([[424243], [424243], [424243], []])
    inspections = []

    ##
    # @par 内部回调 inspect
    # 记录owned_pgid并返回空owner列表，仅测pre/post调用边界。
    # 参数 path：临时文件/目录Path或/proc占位；本回调不真实采设备。
    # 参数 owned_pgid：应排除的本次自有进程组ID或None，不代表其它组归本次拥有。
    # 返回：空owner列表。
    def inspect(path, owned_pgid=None):
        inspections.append(owned_pgid)
        return []

    monkeypatch.setattr(runner, "inspect_owners", inspect)
    monkeypatch.setattr(runner.subprocess, "Popen", lambda command, **kwargs:
                        spawned.append(kwargs) or process)
    monkeypatch.setattr(runner, "group_members", lambda pgid: next(members))
    monkeypatch.setattr(runner.os, "killpg", lambda pgid, sig: signals.append((pgid, sig)))
    monkeypatch.setattr(runner.time, "sleep", lambda seconds: None)
    assert runner.run_session(["fake"], tty, tmp_path / "owner.lock", grace_s=0) == 0
    assert spawned[0]["start_new_session"] is True
    assert signals == [(process.pid, signal.SIGINT), (process.pid, signal.SIGTERM)]
    assert inspections == [None, process.pid]


##
# @brief fake wait中断仍清本组一次SIGINT，然后重抛KeyboardInterrupt。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param tty 真实本机PTY slave Path，绝非实体USB串口。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_interrupt_still_cleans_own_group(tmp_path, tty, monkeypatch):
    process = FakeProcess(KeyboardInterrupt())
    signals = []
    monkeypatch.setattr(runner, "inspect_owners", lambda path, owned_pgid=None: [])
    monkeypatch.setattr(runner.subprocess, "Popen", lambda *a, **k: process)
    monkeypatch.setattr(runner, "group_members", lambda pgid: [424243] if not signals else [])
    monkeypatch.setattr(runner.os, "killpg", lambda pgid, sig: signals.append((pgid, sig)))
    with pytest.raises(KeyboardInterrupt):
        runner.run_session(["fake"], tty, tmp_path / "owner.lock")
    assert signals == [(process.pid, signal.SIGINT)]


##
# @brief 自有组空但postflight仍见外部owner，session不得返回成功。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param tty 真实本机PTY slave Path，绝非实体USB串口。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_postflight_owner_remains_is_failure(tmp_path, tty, monkeypatch):
    process = FakeProcess()
    inspections = iter([[], ["process:987"]])
    monkeypatch.setattr(runner, "inspect_owners", lambda path, owned_pgid=None: next(inspections))
    monkeypatch.setattr(runner.subprocess, "Popen", lambda *a, **k: process)
    monkeypatch.setattr(runner, "group_members", lambda pgid: [])
    with pytest.raises(runner.SessionError, match="postflight"):
        runner.run_session(["fake"], tty, tmp_path / "owner.lock")


##
# @brief 从deployment精确by-id端点导绝对TTY，默认ACM0拒绝，不真实读取设备。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_tty_is_derived_from_bound_deployment_endpoint(tmp_path):
    manifest = tmp_path / "deployment.json"
    device = "/dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_ABC-if00"
    manifest.write_text(json.dumps({"endpoint": f"serial/{device}#baudrate=115200"}))
    assert runner.deployment_tty(manifest) == Path(device)
    manifest.write_text(json.dumps({"endpoint": "serial//dev/ttyACM0#baudrate=115200"}))
    with pytest.raises(runner.SessionError, match="endpoint"):
        runner.deployment_tty(manifest)


##
# @brief fake组始终存活时按INT/TERM/KILL有限升级后报survived，不谎报清理完成。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_persistent_owned_group_fails_after_bounded_kill(monkeypatch):
    signals = []
    monkeypatch.setattr(runner, "group_members", lambda pgid: [424243])
    monkeypatch.setattr(runner.os, "killpg", lambda pgid, sig: signals.append((pgid, sig)))
    with pytest.raises(runner.SessionError, match="survived SIGKILL"):
        runner.stop_owned_group(424242, grace_s=0, kill_wait_s=0)
    assert signals == [(424242, signal.SIGINT), (424242, signal.SIGTERM),
                       (424242, signal.SIGKILL)]


##
# @brief 默认leader仍活先只kill其PID，不直接killpg；假wait空即结束，避免干扰子写footer。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_live_leader_receives_sigint_without_signaling_group(monkeypatch):
    leader_signals = []
    monkeypatch.setattr(runner, "group_members", lambda pgid: [pgid])
    monkeypatch.setattr(runner.os, "kill", lambda pid, sig: leader_signals.append((pid, sig)))
    monkeypatch.setattr(runner.os, "killpg", lambda *args: pytest.fail("signaled group"))
    monkeypatch.setattr(runner, "_wait_group_empty", lambda pgid, deadline: True)
    runner.stop_owned_group(424242)
    assert leader_signals == [(424242, signal.SIGINT)]


##
# @brief fake持续两成员，核六阶段stderr JSON、group目标及GateAlive声明与三信号，非真进程观测。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param capsys pytest stdout/stderr捕获fixture。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_shutdown_reports_phase_member_count_and_gate_state(monkeypatch, capsys):
    signals = []
    monkeypatch.setattr(runner, "group_members", lambda pgid: [pgid, pgid + 1])
    monkeypatch.setattr(runner, "gate_alive", lambda members: True, raising=False)
    monkeypatch.setattr(runner.os, "killpg", lambda pgid, sig: signals.append(sig))
    with pytest.raises(runner.SessionError, match="survived SIGKILL"):
        runner.stop_owned_group(424242, grace_s=0, term_wait_s=0, kill_wait_s=0,
                                interrupt_group=True)
    stages = [json.loads(line) for line in capsys.readouterr().err.splitlines()]
    assert [stage["phase"] for stage in stages] == [
        "sigint", "grace_expired", "sigterm", "term_expired", "sigkill", "kill_expired"]
    assert stages[0]["target"] == "group"
    assert all(stage["members"] == 2 and stage["gate_alive"] is True for stage in stages)
    assert signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]


##
# @brief 人工/proc cmdline只首可执行basename为device_action_gate才算，python参数含此名不足。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_gate_alive_checks_only_executable_name(monkeypatch):
    ##
    # @par 内部回调 cmdline
    # 按指定/proc路径返回人工可执行/参数字节，未读取真正/proc。
    # 参数 path：临时文件/目录Path或/proc占位；本回调不真实采设备。
    # 返回：人工/proc cmdline bytes。
    def cmdline(path):
        if str(path) == "/proc/424243/cmdline":
            return b"/opt/ros/device_action_gate\0--ros-args\0"
        return b"/usr/bin/python3\0device_action_gate\0"

    monkeypatch.setattr(runner.Path, "read_bytes", cmdline)
    assert runner.gate_alive([424242, 424243]) is True
    assert runner.gate_alive([424242]) is False


##
# @brief 人工ps输出滤Z僵尸与其它组，leader已退出仍保同组活跃子PID。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_group_members_ignore_exited_leader_and_zombies(monkeypatch):
    output = "424243 424242 S\n424244 424242 Z\n5 5 S\n"
    monkeypatch.setattr(runner.subprocess, "run", lambda *a, **k:
                        subprocess.CompletedProcess(a[0], 0, output, ""))
    assert runner.group_members(424242) == [424243]


##
# @brief fake run_session核默认真实launch argv/250ms声明、无额外capture参数且SIGTERM handler恢复，不运行launch。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param capsys pytest stdout/stderr捕获fixture。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_cli_uses_bound_tty_and_restores_sigterm(tmp_path, monkeypatch, capsys):
    deployment = tmp_path / "deployment.json"
    device = "/dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_ABC-if00"
    deployment.write_text(json.dumps({"endpoint": f"serial/{device}#baudrate=115200"}))
    calls = []
    monkeypatch.setattr(runner, "run_session", lambda *args, **kwargs:
                        calls.append(args) or 0)
    previous = signal.getsignal(signal.SIGTERM)
    assert runner.main(["--deployment-paths", str(deployment), "--activation-sha256", "a" * 64,
                        "--ros-limits-json", str(tmp_path / "limits.json"),
                        "--lock-file", str(tmp_path / "lock")]) == 0
    assert calls[0][0][:4] == ["ros2", "launch", "dummy_bringup", "real_6dof.launch.py"]
    assert not any(argument.startswith(("raw_evidence_path:=",
                                        "raw_evidence_report_root:=",
                                        "goal_timeline_path:=",
                                        "host_state_max_age_us:=")) for argument in calls[0][0])
    assert '"effective_us": 250000' in capsys.readouterr().out
    assert calls[0][1] == Path(device)
    assert signal.getsignal(signal.SIGTERM) == previous


##
# @brief raw/root/timeline三件路径一起传同一次fake会话，避免分owner采集。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_cli_passes_complete_capture_set_to_same_session(tmp_path, monkeypatch):
    report_root = tmp_path / "report"
    report_root.mkdir()
    raw = report_root / "raw.jsonl"
    timeline = report_root / "goals.jsonl"
    calls = []
    monkeypatch.setattr(runner, "run_session", lambda *args, **kwargs:
                        calls.append(args) or 0)
    monkeypatch.setattr(runner, "deployment_tty", lambda path: Path("/dev/fake"))

    assert runner.main(["--deployment-paths", str(tmp_path / "deployment.json"),
                        "--activation-sha256", "a" * 64,
                        "--ros-limits-json", str(tmp_path / "limits.json"),
                        "--raw-evidence-path", str(raw),
                        "--raw-evidence-report-root", str(report_root),
                        "--goal-timeline-path", str(timeline)]) == 0

    command = calls[0][0]
    assert f"raw_evidence_path:={raw}" in command
    assert f"raw_evidence_report_root:={report_root}" in command
    assert f"goal_timeline_path:={timeline}" in command
    assert calls[0][1] == Path("/dev/fake")


##
# @brief capture模式向会话传grace12s，断言参数装配而非实际关停耗时。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_cli_capture_interrupts_launch_leader_once_with_bounded_grace(tmp_path, monkeypatch):
    report_root = tmp_path / "report"
    report_root.mkdir()
    calls = []
    monkeypatch.setattr(runner, "run_session", lambda *args, **kwargs:
                        calls.append(kwargs) or 0)
    monkeypatch.setattr(runner, "deployment_tty", lambda path: Path("/dev/fake"))
    assert runner.main(["--deployment-paths", str(tmp_path / "deployment.json"),
                        "--activation-sha256", "a" * 64,
                        "--ros-limits-json", str(tmp_path / "limits.json"),
                        "--raw-evidence-path", str(report_root / "raw.bin"),
                        "--raw-evidence-report-root", str(report_root),
                        "--goal-timeline-path", str(report_root / "timeline.csv")]) == 0
    assert calls == [{"grace_s": 12}]


##
# @brief 只有显式500000µs诊断才传launch，并标UNQUALIFIED_DIAGNOSTIC。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param capsys pytest stdout/stderr捕获fixture。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_cli_explicit_diagnostic_state_age_reaches_launch(tmp_path, monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(runner, "run_session", lambda *args: calls.append(args) or 0)
    monkeypatch.setattr(runner, "deployment_tty", lambda path: Path("/dev/fake"))
    assert runner.main(["--deployment-paths", str(tmp_path / "deployment.json"),
                        "--activation-sha256", "a" * 64,
                        "--ros-limits-json", str(tmp_path / "limits.json"),
                        "--diagnostic-host-state-age-us", "500000"]) == 0
    assert "host_state_max_age_us:=500000" in calls[0][0]
    report = json.loads(capsys.readouterr().out.strip())
    assert report == {"event": "host_state_age", "effective_us": 500000,
                      "scope": "UNQUALIFIED_DIAGNOSTIC"}


##
# @brief 默认值250000不能冒充诊断选项，超限/零/文本也在会话前拒绝。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param value 信号编号或非法预算文本，具体含义见对应参数表。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("value", ["250000", "500001", "0", "abc"])
def test_cli_rejects_unapproved_diagnostic_state_age(tmp_path, monkeypatch, value):
    monkeypatch.setattr(runner, "run_session", lambda *args: pytest.fail("session started"))
    with pytest.raises(SystemExit):
        runner.main(["--deployment-paths", str(tmp_path / "deployment.json"),
                     "--activation-sha256", "a" * 64,
                     "--ros-limits-json", str(tmp_path / "limits.json"),
                     "--diagnostic-host-state-age-us", value])


##
# @brief 三件capture参数只给一件时退出1，连TTY检查都不触达。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param capture_args 三件证据采集参数中的不完整子集。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("capture_args", [
    ["--raw-evidence-path", "/tmp/raw.jsonl"],
    ["--raw-evidence-report-root", "/tmp"],
    ["--goal-timeline-path", "/tmp/goals.jsonl"],
])
def test_cli_rejects_partial_capture_before_session(tmp_path, monkeypatch, capture_args):
    monkeypatch.setattr(runner, "run_session", lambda *args: pytest.fail("session started"))
    monkeypatch.setattr(runner, "deployment_tty", lambda path: pytest.fail("tty inspected"))
    with pytest.raises(SystemExit) as error:
        runner.main(["--deployment-paths", str(tmp_path / "deployment.json"),
                     "--activation-sha256", "a" * 64,
                     "--ros-limits-json", str(tmp_path / "limits.json"),
                     *capture_args])
    assert error.value.code == 1


##
# @brief 相对/跨root/已存在/同文件/symlink路径在会话和TTY检查前拒绝。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param case 参数化故障选项字典/路径场景名，按本用例说明。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("case", ["relative", "outside", "existing", "same", "symlink"])
def test_cli_rejects_invalid_capture_paths_before_session(tmp_path, monkeypatch, case):
    root = tmp_path / "report"
    root.mkdir()
    raw = root / "raw.jsonl"
    timeline = root / "goals.jsonl"
    if case == "relative":
        raw = Path("raw.jsonl")
    elif case == "outside":
        raw = tmp_path / "raw.jsonl"
    elif case == "existing":
        raw.touch()
    elif case == "same":
        timeline = raw
    elif case == "symlink":
        linked = tmp_path / "linked"
        linked.symlink_to(root)
        raw = linked / "raw.jsonl"
    monkeypatch.setattr(runner, "run_session", lambda *args: pytest.fail("session started"))
    monkeypatch.setattr(runner, "deployment_tty", lambda path: pytest.fail("tty inspected"))
    with pytest.raises(SystemExit) as error:
        runner.main(["--deployment-paths", str(tmp_path / "deployment.json"),
                     "--activation-sha256", "a" * 64,
                     "--ros-limits-json", str(tmp_path / "limits.json"),
                     "--raw-evidence-path", str(raw),
                     "--raw-evidence-report-root", str(root),
                     "--goal-timeline-path", str(timeline)])
    assert error.value.code == 1


##
# @brief 组查询抛错仍仅对已知自有pgid TERM/KILL并保原ps错误，不搜索/杀外部owner。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_group_probe_failure_still_signals_only_owned_pgid(monkeypatch):
    signals = []

    ##
    # @par 内部回调 unavailable
    # 始终抛SessionError ps unavailable，模拟无法枚举组成员。
    # 参数 pgid：本次清理的进程组号，不扩展到外部owner。
    # 返回：无正常返回；抛ps unavailable的SessionError。
    def unavailable(pgid):
        raise runner.SessionError("ps unavailable")

    monkeypatch.setattr(runner, "group_members", unavailable)
    monkeypatch.setattr(runner.os, "killpg", lambda pgid, sig: signals.append((pgid, sig)))
    with pytest.raises(runner.SessionError, match="ps unavailable"):
        runner.stop_owned_group(424242, grace_s=0)
    assert signals == [(424242, signal.SIGTERM), (424242, signal.SIGKILL)]


##
# @brief 实际本机Python父子进程：响应INT可写COMPLETE，忽略INT/TERM则升级KILL；核<2s和组空，不涉及ROS/设备。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param ignores_shutdown 真实Python子进程是否忽略INT/TERM，用于本机升级信号用例。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("ignores_shutdown", [False, True], ids=["graceful", "escalated"])
def test_real_group_shutdown_writes_footer_or_escalates(tmp_path, ignores_shutdown):
    ready = tmp_path / "ready"
    footer = tmp_path / "footer"
    child_code = f"""
import signal
import time
from pathlib import Path

def interrupt(signum, frame):
    time.sleep(0.1)
    Path({str(footer)!r}).write_text('COMPLETE,1\\n')
    raise SystemExit(0)

signal.signal(signal.SIGINT, signal.SIG_IGN if {ignores_shutdown!r} else interrupt)
signal.signal(signal.SIGTERM, signal.SIG_IGN if {ignores_shutdown!r} else signal.SIG_DFL)
Path({str(ready)!r}).write_text('ready')
while True:
    time.sleep(0.1)
"""
    leader_code = f"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

child = subprocess.Popen([sys.executable, '-c', {child_code!r}],
                         stdin=subprocess.DEVNULL)
while not Path({str(ready)!r}).exists():
    time.sleep(0.01)

def interrupt(signum, frame):
    os.kill(child.pid, signal.SIGINT)
    if {ignores_shutdown!r}:
        raise SystemExit(0)
    child.wait()
    raise SystemExit(0)

signal.signal(signal.SIGINT, interrupt)
while True:
    time.sleep(0.1)
"""
    process = subprocess.Popen([sys.executable, "-c", leader_code],
                               stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert ready.exists(), "subprocess fixture did not start"
        started = time.monotonic()
        runner.stop_owned_group(process.pid, grace_s=0.5, term_wait_s=0.25,
                                kill_wait_s=0.5)
        assert time.monotonic() - started < 2
        process.wait(timeout=1)
        assert runner.group_members(process.pid) == []
        assert footer.exists() is not ignores_shutdown
        if not ignores_shutdown:
            assert footer.read_text() == "COMPLETE,1\n"
    finally:
        if runner.group_members(process.pid):
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=1)


##
# @brief 实际leader忽略INT且不转发时显式group interrupt能到child写footer，最后组空并核诊断目标group。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param capsys pytest stdout/stderr捕获fixture。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_group_interrupt_reaches_child_when_leader_does_not_relay(tmp_path, capsys):
    ready = tmp_path / "ready"
    footer = tmp_path / "footer"
    child_code = f"""
import signal
import time
from pathlib import Path

def interrupt(signum, frame):
    Path({str(footer)!r}).write_text('COMPLETE,1\\n')
    raise SystemExit(0)

signal.signal(signal.SIGINT, interrupt)
Path({str(ready)!r}).touch()
while True:
    time.sleep(0.1)
"""
    leader_code = f"""
import signal
import subprocess
import sys
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)
subprocess.Popen([sys.executable, '-c', {child_code!r}],
                 stdin=subprocess.DEVNULL)
while True:
    time.sleep(0.1)
"""
    process = subprocess.Popen([sys.executable, "-c", leader_code],
                               stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert ready.exists()
        runner.stop_owned_group(process.pid, grace_s=0.3, term_wait_s=0.2,
                                kill_wait_s=0.3, interrupt_group=True)
        process.wait(timeout=1)
        assert footer.read_text() == "COMPLETE,1\n"
        assert runner.group_members(process.pid) == []
        reports = [json.loads(line) for line in capsys.readouterr().err.splitlines()]
        assert reports[0]["phase"] == "sigint"
        assert reports[0]["target"] == "group"
        assert reports[0]["sent"] is True
    finally:
        if runner.group_members(process.pid):
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=1)


##
# @brief 300000µs独立Stop预算传launch，不添加host_state_max_age放宽，实际会话仍替身。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_explicit_approved_stop_budget_reaches_launch_without_freshness_change(tmp_path, monkeypatch):
    calls=[]
    monkeypatch.setattr(runner, "run_session", lambda *args: calls.append(args) or 0)
    monkeypatch.setattr(runner, "deployment_tty", lambda path: Path("/dev/fake"))
    assert runner.main(["--deployment-paths", str(tmp_path/"deployment.json"),
        "--activation-sha256", "a"*64, "--ros-limits-json", str(tmp_path/"limits.json"),
        "--stop-confirmation-timeout-us", "300000"]) == 0
    assert "stop_confirmation_timeout_us:=300000" in calls[0][0]
    assert not any(s.startswith("host_state_max_age_us:=") for s in calls[0][0])


##
# @brief 250000/300001/大值/负/浮点文本Stop预算均在run_session前拒绝。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param value 信号编号或非法预算文本，具体含义见对应参数表。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("value", ["250000", "300001", "1000000", "-1", "300000.0"])
def test_session_rejects_unapproved_total_stop_budget(tmp_path, monkeypatch, value):
    monkeypatch.setattr(runner, "run_session", lambda *args: pytest.fail("session started"))
    with pytest.raises(SystemExit):
        runner.main(["--deployment-paths", str(tmp_path/"deployment.json"),
            "--activation-sha256", "a"*64, "--ros-limits-json", str(tmp_path/"limits.json"),
            "--stop-confirmation-timeout-us", value])
