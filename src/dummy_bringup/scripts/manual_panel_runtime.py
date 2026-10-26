#!/usr/bin/env python3
"""管理本窗口创建的 ROS 进程；设备释放证明只由 C++ 提供。"""
import subprocess
import os
import signal
import time
import tempfile
from pathlib import Path


class PanelRuntime:
    def __init__(self, spawn=None):
        self._owns_group = spawn is None
        self.spawn = spawn or (lambda argv: subprocess.Popen(argv, start_new_session=True))
        self.owned_pgid = None
        self.process = None
        self.state = 'Disconnected'
        self.error = ''
        self.launch_argv = []

    def start(self, argv=()):
        if self.state != 'Disconnected':
            raise RuntimeError('ROS 生命周期尚未结束')
        self.state = 'Starting'
        try:
            if argv:
                self.launch_argv = session_ros_launch_argv(argv)
                self.process = self.spawn(self.launch_argv)
                if self._owns_group:
                    # Popen created a new session; record ownership at creation,
                    # not by later discovery of similarly named ROS processes.
                    self.owned_pgid = self.process.pid
        except Exception as error:
            self.fail(str(error))
            raise

    def check_startup_timeout(self, elapsed, timeout):
        if self.state == 'Starting' and elapsed > timeout:
            self.fail('ROS 启动超时：未收到 C++ 权威状态')

    def read_only(self):
        if self.state == 'Starting':
            self.state = 'ReadOnly'

    def active(self):
        if self.state == 'ReadOnly':
            self.state = 'Active'

    def begin_release(self):
        self.state = 'Releasing'

    def fail(self, reason):
        self.error = str(reason)
        self.state = 'Error'

    def stop(self, released):
        if not released:
            self.fail('失能未确认：保留 ROS 拥有者，禁止切换后端')
            return False
        self.state = 'Stopping'
        try:
            if self.process is not None:
                if self.owned_pgid is not None:
                    deadline = time.monotonic() + 6.0
                    try:
                        os.killpg(self.owned_pgid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=max(.001, deadline-time.monotonic()))
                    while self._owned_group_alive():
                        if time.monotonic() >= deadline:
                            raise TimeoutError('自有ROS进程组尚未退出：保留归属，禁止切换后端')
                        time.sleep(.02)
                    self.owned_pgid = None
                else:
                    self.process.terminate()
                    self.process.wait(timeout=6)
                self.process = None
        except Exception as error:
            self.fail(str(error))
            return False
        self.state = 'Disconnected'
        return True

    def _owned_group_alive(self):
        # Linux /proc membership is checked only against our recorded PGID.
        # Reparented live children still count; exited zombies cannot own ROS IO.
        for stat in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = stat.read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == self.owned_pgid and fields[0] != 'Z':
                    return True
            except (OSError, ValueError, IndexError):
                continue
        return False


ROS_RUNTIME_KEYS = (
    'sdk_prefix', 'ros_controller_config', 'ros_launch_command',
    'ros_startup_timeout_s', 'ros_release_timeout_s',
    'deployment_source_config', 'operator_confirmed_double_limits',
)


def load_runtime_panel_config(config_path, backend, description_xacro=''):
    """真实产物仍在原release校验；窗口临时文件仅携带ROS进程装配字段。"""
    import json
    from pathlib import Path
    from manual_panel_config import load_panel_config
    path = Path(config_path)
    runtime = json.loads(path.read_text())
    original = runtime.get('deployment_source_config')
    if backend == 'real' and original:
        config = load_panel_config(original, 'real', description_xacro)
        config.update({key: runtime[key] for key in ROS_RUNTIME_KEYS if key in runtime})
        config['_config_path'] = str(path.resolve(strict=True))
        return config
    return load_panel_config(path, backend, description_xacro)


def session_ros_launch_argv(argv):
    """每次自有ROS启动使用新证据目录；只建目录，原件仍由C++以O_EXCL创建。"""
    argv = list(argv)
    if 'manual_ros_control.launch.py' not in argv:
        return argv
    keys = ('report_path', 'raw_evidence_path', 'goal_timeline_path')
    existing = dict(argument.split(':=', 1) for argument in argv if ':=' in argument)
    report = existing.get('report_path', '')
    root = Path(report).expanduser().resolve().parent if report else Path.home()/'.ros/dummy_reports'
    root.mkdir(parents=True, exist_ok=True)
    session = Path(tempfile.mkdtemp(prefix='ros-session-', dir=root)).resolve(strict=True)
    values = dict(zip(keys, (session/'baseline.log', session/'transport.raw', session/'goal-timeline.jsonl')))
    # 拒绝所有旧path参数的复用，包括配置中的空值；其他授权/身份argv保持原边界。
    argv = [argument for argument in argv if argument.split(':=', 1)[0] not in keys]
    return argv + [key+':='+str(values[key]) for key in keys]
