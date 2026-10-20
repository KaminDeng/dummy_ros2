#!/usr/bin/env python3
##
# @file run_real_6dof_session.py
# @brief 用本地锁和进程组拥有实际real_6dof ROS launch会话。
#
# 此CLI没有dry-run，会启动真实launch并可能让Gate打开指定STM32 USB；gate_only与full仅改变启动装配，不是离线/实机许可开关。
# owner锁、fuser/pgrep是本机协作锁及瞬时探测，不能保证所有非合作程序永久不抢设备。进程组退出也不单独证明电机已物理停止；Gate受控退出/原证据须另核。
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


##
# @brief 会话锁、路径、owner探测和组收尾失败；不把未确认状态转成成功退出。
class SessionError(RuntimeError):
    pass


##
# @brief 除tty占用外保守探测device_action_gate/real_6dof.launch.py命令名；不只看当前leader。
# ENDPOINT_PATTERN限定固定厂商格式by-id/115200，字符串匹配不等于udev/固件身份认证。
OWNER_PATTERN = r"(^|[[:space:]/])(device_action_gate|real_6dof[.]launch[.]py)([[:space:]]|$)"
ENDPOINT_PATTERN = re.compile(
    r"serial/(\/dev/serial/by-id/usb-Dummy_Robotics_STM32_"
    r"Virtual_ComPort_in_FS_Mode_[A-Za-z0-9]+-if00)#baudrate=115200\Z"
)


##
# @brief 从部署路径文件取STM32 by-id tty。
# @param paths_file 含endpoint的JSON路径。
# @return 匹配by-id的Path；只解析文本，存在/字符设备检查留后续inspect_owners。
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


##
# @brief 核全新采集目标为报告根内canonical绝对路径。
# @param path raw或timeline目标字符串，UTF-8长度≤256。
# @param report_root 已存在目录，不能为/。
# @return None；空/点/父分量、控制字符、symlink、缺目录或lexists目标拒绝。
# 检查与launch后实际打开分离，不能称抵御任意并发路径替换的原子保证。
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


##
# @brief 全无采集或整组校验raw/root/timeline三参数。
# @param raw_path 原始证据目标文本。
# @param report_root 报告根文本。
# @param timeline_path Gate时间线目标文本，不能与raw同路径。
# @return 三个launch赋值参数列表；全空为[]，部分缺失拒绝。
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


##
# @brief 持有当前用户普通文件的非阻塞flock排他锁。
# @param path 锁文件路径，创建父目录和0600文件，最终路径O_NOFOLLOW。
# @return contextmanager，在yield期间持fd锁；退出close释放。
# 校fstat为普通文件且uid为当前用户；不删锁文件，不锁实际tty，也不对不使用同锁的进程构成强制排他。
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


##
# @brief 限3s运行只读owner探测命令。
# @param command fuser/pgrep等argv。
# @return CompletedProcess，0/1均接受；其它退出或启动/超时SessionError。
def _probe(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=3, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise SessionError(f"owner probe failed: {command[0]}: {error}") from error
    if result.returncode not in (0, 1):
        raise SessionError(f"owner probe failed: {command[0]} exit {result.returncode}")
    return result


##
# @brief 核字符设备并保守汇总tty PID及匹配Gate/launch进程。
# @param tty_path 部署by-id路径，可解析到字符设备。
# @param owned_pgid 仅兼容日志参数，不豁免任何进程，当前函数不用其值。
# @return owner标签列表；非空调用者应拒绝启动/标收尾失败，探测不明抛异常。
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


##
# @brief 列举pgid内全部非僵尸PID。
# @param pgid 所拥有的进程组ID。
# @return PID列表；ps失败或格式不明拒绝，不依赖leader还在。
# 不把僵尸算存活活动成员；快照后进程可变化，后续轮询收敛。
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


##
# @brief 向显式所拥有pgid发信号。
# @param pgid 正进程组，调用者确保所有权。
# @param signum SIGINT/TERM/KILL等。
# @return 已发True，不存在False；权限等其它错误传播，不向其它进程组扫描发送。
def _signal_group(pgid, signum):
    try:
        os.killpg(pgid, signum)
    except ProcessLookupError:
        return False
    return True


##
# @brief 从给定PID的proc cmdline首项探测Gate名。
# @param members 已观测组成员PID。
# @return 至少一个可读可执行basename=device_action_gate为True；proc读错忽略，不验证完整身份。
def gate_alive(members):
    for pid in members:
        try:
            executable = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0", 1)[0]
        except OSError:
            continue
        if executable.rsplit(b"/", 1)[-1] == b"device_action_gate":
            return True
    return False


##
# @brief 把进程组收尾阶段输出JSON到stderr。
# @param phase 阶段标签。
# @param members 当前PID快照，报告数量及Gate名是否在场。
# @param target 可选leader/group。
# @param sent 可选信号实际发送bool。
# @return None；仅记进程观察，不生成设备Stop确认。
def _shutdown_phase(phase, members, *, target=None, sent=None):
    report = {"event": "owned_group_shutdown", "phase": phase,
              "members": len(members), "gate_alive": gate_alive(members)}
    if target is not None:
        report["target"] = target
        report["sent"] = sent
    print(json.dumps(report), file=sys.stderr, flush=True)


##
# @brief 在原Host截止内反复等待非僵尸组成员清空。
# @param pgid 所属组。
# @param deadline Host monotonic绝对秒。
# @return 已空True，到期False；每次最多sleep50ms。
def _wait_group_empty(pgid, deadline):
    while group_members(pgid):
        if time.monotonic() >= deadline:
            return False
        time.sleep(min(0.05, max(0, deadline - time.monotonic())))
    return True


##
# @brief 按INT宽限→TERM→KILL有界回收所属进程组。
# @param pgid 必须>0的拥有组ID。
# @param grace_s SIGINT后宽限秒，允许0，默认5。
# @param term_wait_s TERM后等待秒，允许0，默认2。
# @param kill_wait_s KILL后等待秒，允许0，默认2。
# @param interrupt_group true直接INT全组，默认优先leader让launch自行有序退出。
# @return None；KILL仍存活或探测失败SessionError。
# leader不存在就INT组；SessionError时尽力TERM/KILL但保留失败，不能将强杀当Gate完成受控release或证据已flush。
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


##
# @brief 锁内preflight后启动新session进程，结束时收整个组并postflight。
# @param command 实际launch argv。
# @param tty_path 部署tty供owner探测。
# @param lock_path 本地协作锁。
# @param grace_s 退出宽限秒。
# @param interrupt_group 是否INT全组而非leader。
# @return 子进程原退出码；收尾/残留owner异常可覆盖原退出并拒绝成功。
# start_new_session=True使process.pid为pgid；process.wait无运行总时限，外部信号/结束才触发finally。即使leader退了仍清其后代组。
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


##
# @brief 把SIGTERM转换异常以进入finally收尾。
# @param signum 收到的信号编号。
# @param frame Python信号帧，未使用。
# @return 不正常返回，抛KeyboardInterrupt。
def _interrupt(signum, frame):
    raise KeyboardInterrupt(f"signal {signum}")


##
# @brief 解析真实launch身份/捕获参数并拥有该会话。
# @param argv 可选CLI参数列表。
# @return run_session原退出码；SessionError/OSError/中断转退出1。
# 诊断Host年龄只允许500ms，默认250ms；停止确认覆盖只允许0或300ms，0沿launch默认。带捕获时退出宽限改12s以便Gate落盘，不改变设备许可。
# 激活SHA/limits传给launch校验，不在此假定已通过；SIGTERM handler finally恢复。
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
    parser.add_argument("--stop-confirmation-timeout-us", type=int, choices=(0, 300000), default=0)
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
        if args.stop_confirmation_timeout_us:
            command.append(f"stop_confirmation_timeout_us:={args.stop_confirmation_timeout_us}")
        shutdown_options = {"grace_s": 12} if capture_args else {}
        return run_session(command, deployment_tty(args.deployment_paths), args.lock_file,
                           **shutdown_options)
    except (SessionError, OSError, KeyboardInterrupt) as error:
        parser.exit(1, f"real_6dof session failed: {error}\n")
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
