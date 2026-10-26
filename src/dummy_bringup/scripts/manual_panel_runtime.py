#!/usr/bin/env python3
"""管理本窗口创建的 ROS 进程；设备释放证明只由 C++ 提供。"""
import subprocess
import os
import signal
import stat
import time
import tempfile
from pathlib import Path


class PanelRuntime:
    def __init__(self, spawn=None, serial_endpoint=None, sysfs_root='/sys/dev/char'):
        self._owns_group = spawn is None
        self.spawn = spawn or (lambda argv: subprocess.Popen(argv, start_new_session=True))
        self.owned_pgid = None
        self.process = None
        self.state = 'Disconnected'
        self.error = ''
        self.launch_argv = []
        self.serial_endpoint = serial_endpoint
        self._serial_identity = None
        self._sysfs_root = Path(sysfs_root)

    def start(self, argv=(), serial_endpoint=None):
        if self.state != 'Disconnected':
            raise RuntimeError('ROS 生命周期尚未结束')
        if serial_endpoint is not None:
            self.serial_endpoint = serial_endpoint
        self._serial_identity = self._serial_device_identity() if argv else None
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
            self.fail('ROS 启动超时：未收到控制链状态')

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
        return self._stop_owned_process()

    def _stop_owned_process(self):
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

    def _serial_device_path(self):
        endpoint = self.serial_endpoint
        if not isinstance(endpoint, str) or not endpoint.startswith('serial/'):
            return None
        path = endpoint[len('serial/'):].split('#', 1)[0]
        return path if path.startswith('/dev/') else None

    @staticmethod
    def _char_identity(path):
        value = os.stat(path)
        return (value.st_dev, value.st_ino, value.st_rdev) if stat.S_ISCHR(value.st_mode) else None

    def _serial_device_identity(self):
        path = self._serial_device_path()
        try:
            return self._char_identity(path) if path else None
        except (OSError, ValueError):
            return None

    def _owned_group_pids(self):
        # 只按创建时记录的PGID识别成员，绝不查看其他进程的FD或按名字认领。
        members = []
        for source in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = source.read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == self.owned_pgid and fields[0] != 'Z':
                    members.append(int(source.parent.name))
            except (OSError, ValueError, IndexError):
                continue
        return members

    def _owned_group_alive(self):
        return bool(self._owned_group_pids())

    def _owned_serial_detached(self):
        original = self._serial_identity
        if not self._owns_group or self.process is None or self.owned_pgid is None or original is None:
            return False
        try:
            current = self._serial_device_identity()
            if current is None or current == original:
                return False
            driver = self._sysfs_root / f'{os.major(original[2])}:{os.minor(original[2])}'
            if driver.exists():
                return False
            found = False
            for pid in self._owned_group_pids():
                for fd in (Path('/proc') / str(pid) / 'fd').iterdir():
                    identity = self._char_identity(fd)
                    if identity == current:
                        return False
                    if identity == original:
                        if not os.readlink(fd).endswith(' (deleted)'):
                            return False
                        found = True
            return found
        except (OSError, ValueError):
            return False

    def retire_disconnected_owner(self):
        """仅退休已物理断开的本组TTY拥有者，不确认设备失能或动作成功。"""
        if not self._owned_serial_detached():
            return False
        original_error = self.error
        # 发SIGINT前再核一次；无法证明或已有新设备FD则保持原UNCONFIRMED。
        if not self._owned_serial_detached():
            return False
        stopped = self._stop_owned_process()
        if stopped:
            self.error = original_error
        return stopped


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
