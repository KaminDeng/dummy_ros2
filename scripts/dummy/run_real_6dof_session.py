#!/usr/bin/env python3
"""持有真实六轴 ROS launch 的本地独占会话和进程组。"""

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import sys
import time


class SessionError(RuntimeError):
    pass


OWNER_PATTERN = r"(^|[[:space:]/])(device_action_gate|real_6dof[.]launch[.]py)([[:space:]]|$)"
ENDPOINT_PATTERN = re.compile(
    r"serial/(\/dev/serial/by-id/usb-Dummy_Robotics_STM32_"
    r"Virtual_ComPort_in_FS_Mode_[A-Za-z0-9]+-if00)#baudrate=115200\Z"
)


def deployment_tty(paths_file):
    try:
        paths = json.loads(Path(paths_file).read_text())
    except (OSError, ValueError) as error:
        raise SessionError(f"deployment paths unreadable: {error}") from error
    endpoint = paths.get("endpoint") if isinstance(paths, dict) else None
    match = ENDPOINT_PATTERN.fullmatch(endpoint) if isinstance(endpoint, str) else None
    if match is None:
        raise SessionError("deployment endpoint must bind a STM32 by-id tty")
    return Path(match.group(1))


def _capture_destination(path, report_root):
    for label, value in (("capture path", path), ("report root", report_root)):
        if (not value.startswith("/") or
                any(part in ("", ".", "..") for part in value[1:].split("/")) or
                any(char in value for char in "\x00\n\r\t")):
            raise SessionError(f"{label}: canonical absolute path required")
    if len(path.encode("utf-8")) > 256:
        raise SessionError("capture path exceeds transport limit")
    target, root = Path(path), Path(report_root)
    if root == Path("/") or target == root or not target.is_relative_to(root):
        raise SessionError("capture path must be beneath report root")
    for candidate in (root, *root.parents, target.parent, *target.parent.parents):
        if candidate.is_symlink():
            raise SessionError(f"capture symlink component: {candidate}")
    if not root.is_dir() or not target.parent.is_dir():
        raise SessionError("capture report directory missing")
    if os.path.lexists(path):
        raise SessionError("capture destination already exists")


def capture_launch_arguments(raw_path, report_root, timeline_path):
    if not any((raw_path, report_root, timeline_path)):
        return []
    if not all((raw_path, report_root, timeline_path)):
        raise SessionError("capture requires raw evidence, report root, and goal timeline paths")
    _capture_destination(raw_path, report_root)
    _capture_destination(timeline_path, report_root)
    if raw_path == timeline_path:
        raise SessionError("goal timeline and raw evidence paths must differ")
    return [f"raw_evidence_path:={raw_path}",
            f"raw_evidence_report_root:={report_root}",
            f"goal_timeline_path:={timeline_path}"]


@contextmanager
def owner_lock(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        details = os.fstat(descriptor)
        if not stat.S_ISREG(details.st_mode) or details.st_uid != os.getuid():
            raise SessionError("owner lock must be a regular file owned by this user")
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise SessionError("another real_6dof owner holds the lock") from error
        yield
    finally:
        os.close(descriptor)


def _probe(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=3, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise SessionError(f"owner probe failed: {command[0]}: {error}") from error
    if result.returncode not in (0, 1):
        raise SessionError(f"owner probe failed: {command[0]} exit {result.returncode}")
    return result


def inspect_owners(tty_path, owned_pgid=None):
    """保守检查；owned_pgid 仅用于调用方关联日志，不豁免任何残留。"""
    tty = Path(tty_path)
    if not stat.S_ISCHR(tty.stat().st_mode):
        raise SessionError(f"tty is not a character device: {tty}")
    fuser = _probe(["fuser", str(tty)])
    owners = [f"tty:{pid}" for pid in fuser.stdout.split()]
    pgrep = _probe(["pgrep", "-af", OWNER_PATTERN])
    for line in pgrep.stdout.splitlines():
        match = re.match(r"^([0-9]+)\s+", line)
        if match:
            owners.append(f"process:{match.group(1)}")
        else:
            raise SessionError("malformed pgrep owner output")
    return owners


def group_members(pgid):
    """返回指定进程组内所有非僵尸 PID，不依赖 leader 是否仍存在。"""
    try:
        result = subprocess.run(["ps", "-e", "-o", "pid=,pgid=,stat="],
                                capture_output=True, text=True, timeout=3, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise SessionError(f"process group probe failed: {error}") from error
    if result.returncode:
        raise SessionError(f"process group probe failed: ps exit {result.returncode}")
    members = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) < 3:
            raise SessionError("malformed ps group output")
        try:
            pid, group = int(parts[0]), int(parts[1])
        except ValueError as error:
            raise SessionError("malformed ps group output") from error
        if group == pgid and not parts[2].startswith("Z"):
            members.append(pid)
    return members


def _signal_group(pgid, signum):
    try:
        os.killpg(pgid, signum)
    except ProcessLookupError:
        return False
    return True


def gate_alive(members):
    for pid in members:
        try:
            executable = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0", 1)[0]
        except OSError:
            continue
        if executable.rsplit(b"/", 1)[-1] == b"device_action_gate":
            return True
    return False


def _shutdown_phase(phase, members, *, target=None, sent=None):
    report = {"event": "owned_group_shutdown", "phase": phase,
              "members": len(members), "gate_alive": gate_alive(members)}
    if target is not None:
        report["target"] = target
        report["sent"] = sent
    print(json.dumps(report), file=sys.stderr, flush=True)


def _wait_group_empty(pgid, deadline):
    while group_members(pgid):
        if time.monotonic() >= deadline:
            return False
        time.sleep(min(0.05, max(0, deadline - time.monotonic())))
    return True


def stop_owned_group(pgid, grace_s=5, term_wait_s=2, kill_wait_s=2,
                     interrupt_group=False):
    if pgid <= 0 or min(grace_s, term_wait_s, kill_wait_s) < 0:
        raise ValueError("owned group and shutdown waits must be positive")
    sent_term = False
    sent_kill = False
    try:
        members = group_members(pgid)
        if not members:
            _shutdown_phase("already_empty", members)
            return
        if interrupt_group or pgid not in members:
            target = "group"
            sent = _signal_group(pgid, signal.SIGINT)
        else:
            target = "leader"
            try:
                os.kill(pgid, signal.SIGINT)
                sent = True
            except ProcessLookupError:
                target = "group"
                sent = _signal_group(pgid, signal.SIGINT)
        _shutdown_phase("sigint", members, target=target, sent=sent)
        if _wait_group_empty(pgid, time.monotonic() + grace_s):
            _shutdown_phase("grace_complete", [])
            return
        members = group_members(pgid)
        _shutdown_phase("grace_expired", members)
        if not members:
            return
        sent = _signal_group(pgid, signal.SIGTERM)
        sent_term = True
        _shutdown_phase("sigterm", members, target="group", sent=sent)
        if _wait_group_empty(pgid, time.monotonic() + term_wait_s):
            _shutdown_phase("term_complete", [])
            return
        members = group_members(pgid)
        _shutdown_phase("term_expired", members)
        if not members:
            return
        sent = _signal_group(pgid, signal.SIGKILL)
        sent_kill = True
        _shutdown_phase("sigkill", members, target="group", sent=sent)
        if not _wait_group_empty(pgid, time.monotonic() + kill_wait_s):
            _shutdown_phase("kill_expired", group_members(pgid))
            raise SessionError(f"owned process group {pgid} survived SIGKILL")
        _shutdown_phase("kill_complete", [])
    except SessionError:
        if not sent_kill:
            if not sent_term:
                _signal_group(pgid, signal.SIGTERM)
            _signal_group(pgid, signal.SIGKILL)
        raise


def run_session(command, tty_path, lock_path, *, grace_s=5, interrupt_group=False):
    if grace_s < 0:
        raise ValueError("grace_s must be nonnegative")
    with owner_lock(lock_path):
        existing = inspect_owners(tty_path)
        if existing:
            raise SessionError(f"real_6dof occupied before launch: {', '.join(existing)}")
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                                   start_new_session=True)
        pgid = process.pid
        try:
            return_code = process.wait()
        finally:
            try:
                stop_owned_group(pgid, grace_s=grace_s,
                                 interrupt_group=interrupt_group)
            finally:
                remaining = inspect_owners(tty_path, owned_pgid=pgid)
                if remaining:
                    raise SessionError(f"real_6dof postflight occupied: {', '.join(remaining)}")
        return return_code


def _interrupt(signum, frame):
    raise KeyboardInterrupt(f"signal {signum}")


def main(argv=None):
    parser = argparse.ArgumentParser(description="独占持有真实六轴 ROS launch")
    parser.add_argument("--deployment-paths", type=Path, required=True)
    parser.add_argument("--activation-sha256", required=True)
    parser.add_argument("--ros-limits-json", type=Path, required=True)
    parser.add_argument("--startup-mode", choices=("gate_only", "full"), default="gate_only")
    parser.add_argument("--raw-evidence-path", default="")
    parser.add_argument("--raw-evidence-report-root", default="")
    parser.add_argument("--goal-timeline-path", default="")
    parser.add_argument("--diagnostic-host-state-age-us", type=int, choices=(500000,))
    parser.add_argument("--lock-file", type=Path,
                        default=Path(f"/run/user/{os.getuid()}/dummy-real-6dof-session.lock"))
    args = parser.parse_args(argv)
    previous = signal.signal(signal.SIGTERM, _interrupt)
    try:
        capture_args = capture_launch_arguments(
            args.raw_evidence_path, args.raw_evidence_report_root, args.goal_timeline_path)
        host_state_age_us = args.diagnostic_host_state_age_us or 250000
        print(json.dumps({"event": "host_state_age", "effective_us": host_state_age_us,
                          "scope": "UNQUALIFIED_DIAGNOSTIC" if
                          args.diagnostic_host_state_age_us else "NO_LOAD_DEV_DEFAULT"}), flush=True)
        command = ["ros2", "launch", "dummy_bringup", "real_6dof.launch.py",
                   f"deployment_paths:={args.deployment_paths}",
                   f"activation_sha256:={args.activation_sha256}",
                   f"ros_limits_json:={args.ros_limits_json}",
                   f"startup_mode:={args.startup_mode}", *capture_args]
        if args.diagnostic_host_state_age_us:
            command.append(f"host_state_max_age_us:={host_state_age_us}")
        shutdown_options = {"grace_s": 12} if capture_args else {}
        return run_session(command, deployment_tty(args.deployment_paths), args.lock_file,
                           **shutdown_options)
    except (SessionError, OSError, KeyboardInterrupt) as error:
        parser.exit(1, f"real_6dof session failed: {error}\n")
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
