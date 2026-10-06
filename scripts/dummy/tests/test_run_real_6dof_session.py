"""真实六轴栈的进程所有权测试；不启动 ROS 或访问真实设备。"""

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


class FakeProcess:
    pid = 424242

    def __init__(self, result=0):
        self.result = result
        self.wait_calls = []

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        if isinstance(self.result, BaseException):
            raise self.result
        return self.result


@pytest.fixture
def tty():
    master, slave = pty.openpty()
    try:
        yield Path(os.ttyname(slave))
    finally:
        os.close(slave)
        os.close(master)


def test_lock_rejects_second_owner(tmp_path):
    lock_path = tmp_path / "owner.lock"
    with runner.owner_lock(lock_path):
        with pytest.raises(runner.SessionError, match="owner"):
            with runner.owner_lock(lock_path):
                pass


@pytest.mark.parametrize("conflict", ["tty", "process"])
def test_preflight_rejects_external_owner_without_spawning(tmp_path, tty, monkeypatch, conflict):
    monkeypatch.setattr(runner, "inspect_owners", lambda path, owned_pgid=None: [conflict])
    monkeypatch.setattr(runner.subprocess, "Popen", lambda *a, **k: pytest.fail("spawned"))
    with pytest.raises(runner.SessionError, match="occupied"):
        runner.run_session(["unused"], tty, tmp_path / "owner.lock")


def test_inspection_uses_fuser_and_pgrep_on_pty(tty, monkeypatch):
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        if command[0] == "fuser":
            return subprocess.CompletedProcess(command, 0, "987\n", "")
        return subprocess.CompletedProcess(command, 0, "123 device_action_gate\n", "")

    monkeypatch.setattr(runner.subprocess, "run", run)
    assert runner.inspect_owners(tty) == ["tty:987", "process:123"]
    assert calls[0] == ["fuser", str(tty)]
    assert calls[1][:2] == ["pgrep", "-af"]


def test_leader_exits_but_group_child_requires_term(tmp_path, tty, monkeypatch):
    process = FakeProcess()
    spawned = []
    signals = []
    members = iter([[424243], [424243], [424243], []])
    inspections = []

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


def test_postflight_owner_remains_is_failure(tmp_path, tty, monkeypatch):
    process = FakeProcess()
    inspections = iter([[], ["process:987"]])
    monkeypatch.setattr(runner, "inspect_owners", lambda path, owned_pgid=None: next(inspections))
    monkeypatch.setattr(runner.subprocess, "Popen", lambda *a, **k: process)
    monkeypatch.setattr(runner, "group_members", lambda pgid: [])
    with pytest.raises(runner.SessionError, match="postflight"):
        runner.run_session(["fake"], tty, tmp_path / "owner.lock")


def test_tty_is_derived_from_bound_deployment_endpoint(tmp_path):
    manifest = tmp_path / "deployment.json"
    device = "/dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_ABC-if00"
    manifest.write_text(json.dumps({"endpoint": f"serial/{device}#baudrate=115200"}))
    assert runner.deployment_tty(manifest) == Path(device)
    manifest.write_text(json.dumps({"endpoint": "serial//dev/ttyACM0#baudrate=115200"}))
    with pytest.raises(runner.SessionError, match="endpoint"):
        runner.deployment_tty(manifest)


def test_persistent_owned_group_fails_after_bounded_kill(monkeypatch):
    signals = []
    monkeypatch.setattr(runner, "group_members", lambda pgid: [424243])
    monkeypatch.setattr(runner.os, "killpg", lambda pgid, sig: signals.append((pgid, sig)))
    with pytest.raises(runner.SessionError, match="survived SIGKILL"):
        runner.stop_owned_group(424242, grace_s=0, kill_wait_s=0)
    assert signals == [(424242, signal.SIGINT), (424242, signal.SIGTERM),
                       (424242, signal.SIGKILL)]


def test_live_leader_receives_sigint_without_signaling_group(monkeypatch):
    leader_signals = []
    monkeypatch.setattr(runner, "group_members", lambda pgid: [pgid])
    monkeypatch.setattr(runner.os, "kill", lambda pid, sig: leader_signals.append((pid, sig)))
    monkeypatch.setattr(runner.os, "killpg", lambda *args: pytest.fail("signaled group"))
    monkeypatch.setattr(runner, "_wait_group_empty", lambda pgid, deadline: True)
    runner.stop_owned_group(424242)
    assert leader_signals == [(424242, signal.SIGINT)]


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


def test_gate_alive_checks_only_executable_name(monkeypatch):
    def cmdline(path):
        if str(path) == "/proc/424243/cmdline":
            return b"/opt/ros/device_action_gate\0--ros-args\0"
        return b"/usr/bin/python3\0device_action_gate\0"

    monkeypatch.setattr(runner.Path, "read_bytes", cmdline)
    assert runner.gate_alive([424242, 424243]) is True
    assert runner.gate_alive([424242]) is False


def test_group_members_ignore_exited_leader_and_zombies(monkeypatch):
    output = "424243 424242 S\n424244 424242 Z\n5 5 S\n"
    monkeypatch.setattr(runner.subprocess, "run", lambda *a, **k:
                        subprocess.CompletedProcess(a[0], 0, output, ""))
    assert runner.group_members(424242) == [424243]


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


def test_cli_capture_uses_bounded_group_interrupt_grace(tmp_path, monkeypatch):
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
    assert calls == [{"grace_s": 12, "interrupt_group": True}]


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


@pytest.mark.parametrize("value", ["250000", "500001", "0", "abc"])
def test_cli_rejects_unapproved_diagnostic_state_age(tmp_path, monkeypatch, value):
    monkeypatch.setattr(runner, "run_session", lambda *args: pytest.fail("session started"))
    with pytest.raises(SystemExit):
        runner.main(["--deployment-paths", str(tmp_path / "deployment.json"),
                     "--activation-sha256", "a" * 64,
                     "--ros-limits-json", str(tmp_path / "limits.json"),
                     "--diagnostic-host-state-age-us", value])


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


def test_group_probe_failure_still_signals_only_owned_pgid(monkeypatch):
    signals = []

    def unavailable(pgid):
        raise runner.SessionError("ps unavailable")

    monkeypatch.setattr(runner, "group_members", unavailable)
    monkeypatch.setattr(runner.os, "killpg", lambda pgid, sig: signals.append((pgid, sig)))
    with pytest.raises(runner.SessionError, match="ps unavailable"):
        runner.stop_owned_group(424242, grace_s=0)
    assert signals == [(424242, signal.SIGTERM), (424242, signal.SIGKILL)]


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
