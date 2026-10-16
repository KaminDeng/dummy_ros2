#!/usr/bin/python3
"""六轴空载交互面板：SDK单owner；ROS只发布实际关节状态，不在启动时运动。"""
import argparse
import json
import hashlib
import math
import struct
import signal
import sys
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import xacro


def valid_goal(degrees, limits):
    return len(degrees) == 6 and all(
        math.isfinite(v) and lo <= v <= hi for v, (lo, hi) in zip(degrees, limits))


class JointPanel(QtWidgets.QWidget):
    def __init__(self, config, report, decoder, node):
        super().__init__()
        self.cfg, self.report, self.decode, self.node = config, report, decoder, node
        self.publisher = node.create_publisher(JointState, '/joint_states', 10)
        self.target_publisher = node.create_publisher(JointState, '/target_joint_states', 10)
        self.description_publisher = node.create_publisher(String, '/measured_robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.description_xml = xacro.process_file(config['description_xacro']).toxml()
        self.preview_description_publisher = node.create_publisher(String, '/target_preview/colored_robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.preview_failed = None
        self.description_visible = None
        self.report.mkdir(parents=True, exist_ok=False)
        self.raw = self.report / 'transport.raw'
        self.offset = 24
        self.last_state = None
        self.last_state_ns = 0
        self.calibration = None
        self.authorized = False
        self.control_session = None
        self.pending_authorization = None
        self.state_boot = None
        self.state_sequence = 0
        self.retired_state_boots = set()
        self.restart_after_release = False
        self.readonly_handoff = None
        self.release_confirmed = False
        self.pending_home = False
        self.pending_home_generation = None
        self.closing = False
        self.worker_generation = 0
        self.connection_action = None
        self.manage_generation = 0
        self.zero_busy = False
        self.busy = False
        self.wait_home = False
        self.readonly_fault = False
        self.readonly_fault_identity = None
        self.readonly_wait_generation = None
        self.readonly_wait_identity = None
        self.stdout_buffer = ''
        self.anchor = None
        self.display_initialized = False
        self.previous_raw = None
        self.calibration_event_ns = 0
        self.limits = [j['limits_deg'] for j in config['joints']]
        self.setWindowTitle('Dummy 六轴机械臂 · 实机标定与角度控制')
        self.resize(920, 550)
        layout = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel('六轴实机控制  /  RViz 实时反馈')
        title.setStyleSheet('font-size:22px;font-weight:bold;margin:8px')
        layout.addWidget(title)
        self.status = QtWidgets.QLabel('正在连接；启动不会采零或运动')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        reference_note = ('零点：人工轻靠双限位 [0°, −73°, 180°, 0°, 0°, 0°] → 标定 → 低速退工作 Rest。\n'
                          if self.manual_limits_mode() else '零点：人工 Home [0°, 0°, 90°, 0°, 0°, 0°] → 缓慢 Rest。\n')
        note = QtWidgets.QLabel(reference_note +
                               '滑块仅设置目标，点击“执行六轴目标”才运动。同一设备boot内，失能和软件重连保留已完成零点；重启后重新标定。')
        note.setWordWrap(True)
        layout.addWidget(note)
        grid = QtWidgets.QGridLayout()
        for col, label in enumerate(['关节', '实际角度', '目标滑块', '目标角度（°）', '软件范围（°）']):
            grid.addWidget(QtWidgets.QLabel(label), 0, col)
        self.feedback_labels, self.inputs, self.sliders = [], [], []
        preview_pose = config.get('calibration_pose_deg') if self.manual_limits_mode() else None
        if not isinstance(preview_pose, (list, tuple)) or len(preview_pose) != 6 or any(type(q) not in (int, float) or not math.isfinite(q) for q in preview_pose):
            preview_pose = config.get('home_deg', [0, 0, 90, 0, 0, 0])
        for i, (joint, (lo, hi)) in enumerate(zip(config['joints'], self.limits)):
            actual = QtWidgets.QLabel('—')
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            # 仅首次标定预览可显示P；普通目标授权仍由self.limits严格检查。
            preview_lo, preview_hi = min(lo, preview_pose[i]), max(hi, preview_pose[i])
            slider.setRange(round(preview_lo * 100), round(preview_hi * 100))
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(2)
            spin.setRange(preview_lo, preview_hi)
            spin.setSingleStep(0.1)
            spin.setSuffix(' °')
            spin.setKeyboardTracking(False)
            spin.setValue(preview_pose[i])
            slider.setValue(round(spin.value() * 100))
            slider.valueChanged.connect(lambda v, box=spin: box.setValue(v / 100))
            spin.valueChanged.connect(lambda v, bar=slider: self.sync_slider(bar, v))
            grid.addWidget(QtWidgets.QLabel(joint['name']), i + 1, 0)
            grid.addWidget(actual, i + 1, 1)
            grid.addWidget(slider, i + 1, 2)
            grid.addWidget(spin, i + 1, 3)
            grid.addWidget(QtWidgets.QLabel(f'{lo:g} … {hi:g}'), i + 1, 4)
            self.feedback_labels.append(actual)
            self.inputs.append(spin)
            self.sliders.append(slider)
        grid.setColumnStretch(2, 1)
        layout.addLayout(grid)
        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel('普通目标速度（°/s）'))
        self.speed_input = QtWidgets.QDoubleSpinBox()
        self.speed_input.setDecimals(1)
        self.speed_input.setRange(0.1, config.get('motion_speed_max_deg_s', 17.2))
        self.speed_input.setSingleStep(0.1)
        self.speed_input.setValue(config.get('motion_speed_default_deg_s', 17.2))
        self.speed_input.setSuffix(' °/s')
        speed_row.addWidget(self.speed_input)
        speed_row.addWidget(QtWidgets.QLabel('加速度（rad/s²）'))
        self.acceleration_input = QtWidgets.QDoubleSpinBox()
        self.acceleration_input.setDecimals(2)
        self.acceleration_input.setRange(0.05, config.get('motion_acceleration_max_rad_s2', 1.0))
        self.acceleration_input.setSingleStep(0.05)
        self.acceleration_input.setValue(config.get('motion_acceleration_default_rad_s2', 0.4))
        speed_row.addWidget(self.acceleration_input)
        speed_row.addWidget(QtWidgets.QLabel('双限位退Rest固定≤2°/s、≤0.1rad/s²' if self.manual_limits_mode() else 'Home→Rest标定段保持原限速'))
        speed_row.addStretch()
        layout.addLayout(speed_row)
        row = QtWidgets.QHBoxLayout()
        self.zero = QtWidgets.QPushButton('① 人工轻靠双限位 → 标定 → 低速退工作Rest' if self.manual_limits_mode() else '① Home → Rest 标定零点')
        self.copy = QtWidgets.QPushButton('实际角度填入目标')
        self.move = QtWidgets.QPushButton('② 执行滑块 / 输入框中的目标')
        self.resume = QtWidgets.QPushButton('使能 / 恢复已有零点')
        self.recover = QtWidgets.QPushButton('低速回工作区')
        self.reconnect = QtWidgets.QPushButton('重新连接（只读）')
        self.clear_fault = QtWidgets.QPushButton('清除已处理故障')
        self.stop = QtWidgets.QPushButton('停止并失能（保留已完成零点）')
        self.stop.setStyleSheet('background:#b42318;color:white;font-weight:bold;padding:8px')
        for button in [self.zero, self.resume, self.recover, self.copy, self.move, self.stop]:
            row.addWidget(button)
        layout.addLayout(row)
        connection_row = QtWidgets.QHBoxLayout()
        connection_row.addWidget(self.reconnect)
        connection_row.addWidget(self.clear_fault)
        layout.addLayout(connection_row)
        self.reconnect.clicked.connect(self.request_reconnect)
        self.clear_fault.clicked.connect(self.request_clear_fault)
        self.zero.clicked.connect(self.start_zero)
        self.resume.clicked.connect(self.start_resume)
        self.recover.clicked.connect(self.start_recovery)
        self.move.clicked.connect(self.start_move)
        self.copy.clicked.connect(self.copy_actual)
        self.stop.clicked.connect(lambda: self.release('STOP'))
        preview_row = QtWidgets.QHBoxLayout()
        for title, pose in [('仅预览 Home 示意（不运动）', [0,0,90,0,0,0]), ('仅预览 Rest 目标（不运动）', config['rest_deg'])]:
            button = QtWidgets.QPushButton(title)
            button.clicked.connect(lambda _checked, q=pose: self.set_preview_pose(q))
            preview_row.addWidget(button)
        layout.addLayout(preview_row)
        self.target_hint = QtWidgets.QLabel('修改滑块或输入框仅设置目标；点击②执行才会驱动实物。')
        layout.addWidget(self.target_hint)
        for box in self.inputs:
            box.valueChanged.connect(lambda _v: self.target_hint.setText('目标已修改，尚未发送。请点击②执行滑块 / 输入框中的目标。'))
        if config.get('moveit_enabled', False):
            from manual_moveit_tab import MoveItTab
            self.tabs = QtWidgets.QTabWidget()
            joint_tab = QtWidgets.QWidget()
            joint_layout = QtWidgets.QVBoxLayout(joint_tab)
            for item in (grid, preview_row):
                layout.removeItem(item)
                joint_layout.addLayout(item)
            layout.removeWidget(self.target_hint)
            joint_layout.addWidget(self.target_hint)
            self.tabs.addTab(joint_tab, '关节控制')
            self.moveit_tab = MoveItTab(self)
            self.tabs.addTab(self.moveit_tab, '末端规划 · MoveIt')
            self.tabs.currentChanged.connect(self.planning_tab_changed)
            # 连接、使能与停止仍在公共区域，两页始终可见。
            layout.insertWidget(3, self.tabs)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        layout.addWidget(self.log)
        self.worker = QtCore.QProcess(self)
        self.worker.readyReadStandardOutput.connect(self.stdout_ready)
        self.worker.readyReadStandardError.connect(self.stderr_ready)
        self.worker.finished.connect(self.finished)
        self.manage_worker = QtCore.QProcess(self)
        self.manage_worker.readyReadStandardOutput.connect(self.manage_stdout_ready)
        self.manage_worker.readyReadStandardError.connect(self.manage_stderr_ready)
        self.manage_worker.finished.connect(self.manage_finished)
        self.manage_worker.errorOccurred.connect(self.manage_error)
        self.start_worker()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)
        self.tick()

    def planning_tab_changed(self, index):
        # 规划页只能走整条路径按钮，避免误把关节页的末点按钮当作规划执行。
        self.move.setVisible(index == 0)
        self.copy.setVisible(index == 0)
        if index == 1 and not self.moveit_tab.path and not self.moveit_tab.pending and not self.moveit_tab.executing:
            self.moveit_tab.capture_actual()

    def helper_active(self):
        helper = getattr(self, 'manage_worker', None)
        return helper is not None and helper.state() != QtCore.QProcess.NotRunning

    def fault_clear_available(self):
        return (self.fresh() and self.last_state['core_state'] == 9 and
                self.last_state['session_id'] == 0 and not self.authorized and
                not self.busy and not self.helper_active() and getattr(self, 'connection_action', None) is None)

    def request_reconnect(self):
        if self.closing or self.busy or self.helper_active() or getattr(self, 'connection_action', None) is not None:
            return
        self.queue_connection_action('reconnect')

    def request_clear_fault(self):
        if self.closing or not self.fault_clear_available():
            return
        answer = QtWidgets.QMessageBox.question(self, '清除已处理故障',
            '请确认故障原因已处理、机械臂有支撑，并已停止手动调整。\n'
            '此操作仅清故障并重新只读连接，不采零、不使能、不运动。\n'
            '后续仍需另行点击“使能 / 恢复已有零点”；近限位时该步骤会低速退工作Rest。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer == QtWidgets.QMessageBox.Yes and not self.closing and self.fault_clear_available():
            self.queue_connection_action('clear_fault')

    def queue_connection_action(self, action):
        if self.helper_active() or self.closing:
            return
        self.remember_readonly_handoff()
        self.connection_action = action
        self.authorized = False
        self.pending_authorization = None
        self.pending_home = False
        self.pending_home_generation = None
        self.restart_after_release = False
        self.busy = True
        if self.worker.state() == QtCore.QProcess.NotRunning:
            self.run_connection_action()
        elif not self.command('RELEASE'):
            self.connection_action = None
            self.readonly_handoff = None
            self.busy = False
            self.message('退出原连接失败，未启动第二个通信进程。')

    def run_connection_action(self):
        if self.closing or self.worker.state() != QtCore.QProcess.NotRunning or self.helper_active():
            return
        action, self.connection_action = self.connection_action, None
        if action == 'clear_fault':
            self.start_fault_helper()
        elif action == 'reconnect':
            self.busy = False
            self.start_worker()

    def start_fault_helper(self):
        # 只传身份白名单；绝不沿用home-rest场景或运动授权参数。
        identity_flags = {'--endpoint', '--device', '--client', '--profile', '--mask',
                          '--scope', '--firmware', '--contract', '--model', '--profile-sha256'}
        original = self.cfg['native_arguments']
        args = ['--execute', '--manage-probe', '--target', '0,0,0,0,0,0']
        for index, flag in enumerate(original):
            if flag in identity_flags:
                if index + 1 >= len(original):
                    self.busy = False
                    self.message('清故障身份配置不完整，未启动通信。')
                    return
                args.extend([flag, original[index + 1]])
        self.manage_generation += 1
        self.manage_prefix = self.report / f'clear-fault-{self.manage_generation}'
        args.extend(['--raw-transport-output', str(self.manage_prefix) + '.raw'])
        Path(str(self.manage_prefix) + '.argv.json').write_text(json.dumps([self.cfg['native_binary']] + args, indent=2))
        self.busy = True
        self.message('仅执行明确清故障与模式选择；结束后重新只读连接，不自动使能。')
        self.manage_worker.start(self.cfg['native_binary'], args)

    def manage_stdout_ready(self):
        self.manage_output(False)

    def manage_stderr_ready(self):
        self.manage_output(True)

    def manage_output(self, stderr):
        data = bytes(self.manage_worker.readAllStandardError() if stderr else self.manage_worker.readAllStandardOutput()).decode(errors='replace')
        with Path(str(self.manage_prefix) + ('.stderr' if stderr else '.stdout')).open('a') as output:
            output.write(data)
        if data.strip():
            self.message(data.strip()[-1200:])

    def manage_error(self, error):
        if error == QtCore.QProcess.FailedToStart:
            self.manage_finished(-1, 0)

    def manage_finished(self, code, _status):
        self.busy = self.zero_busy = False
        self.message(f'清故障进程结束，code={code}；仅重新读取设备状态，未自动使能。')
        if not self.closing:
            self.start_worker()

    def start_worker(self):
        if self.closing or self.helper_active() or self.worker.state() != QtCore.QProcess.NotRunning:
            return
        # 每次Native保留独立原件；重启只读，不自动申请owner或运动。
        self.raw = self.report / ('transport.raw' if self.worker_generation == 0 else f'transport-{self.worker_generation}.raw')
        self.offset = 24
        self.last_state = None
        self.last_state_ns = 0
        self.authorized = False
        self.pending_authorization = None
        self.stdout_buffer = ''
        self.wait_home = False
        self.readonly_fault = False
        self.readonly_fault_identity = None
        self.readonly_handoff = None
        self.readonly_wait_generation = None
        self.readonly_wait_identity = None
        self.release_confirmed = False
        if self.pending_home:
            self.zero_busy = self.busy = True
        args = list(self.cfg['native_arguments']) + ['--panel', '--raw-transport-output', str(self.raw)]
        suffix = '' if self.worker_generation == 0 else f'-{self.worker_generation}'
        (self.report / f'argv{suffix}.json').write_text(json.dumps([self.cfg['native_binary']] + args, indent=2))
        (self.report / f'native-software{suffix}.json').write_text(json.dumps({'native_binary_sha256': hashlib.sha256(Path(self.cfg['native_binary']).read_bytes()).hexdigest(), 'expanded_model_sha256': hashlib.sha256(self.description_xml.encode()).hexdigest()}, indent=2) + '\n')
        self.worker_generation += 1
        self.worker.start(self.cfg['native_binary'], args)

    def control_state_allowed(self, allowed=(1, 2, 4)):
        state = self.last_state
        return bool(state) and not getattr(self, 'readonly_fault', False) and state['core_state'] in allowed

    def enter_readonly_fault(self, identity=None):
        self.readonly_fault = True
        if identity is not None:
            self.readonly_fault_identity = identity
        self.wait_home = self.authorized = self.busy = self.zero_busy = False
        self.pending_authorization = None
        self.pending_home = False
        self.pending_home_generation = None
        if getattr(self, 'readonly_handoff', None) is None:
            self.restart_after_release = False
        self.readonly_wait_generation = None
        self.readonly_wait_identity = None

    def capture_rejection_is_current_readonly(self, parts):
        marker = getattr(self, 'readonly_wait_identity', None)
        if (not self.zero_busy or not self.fresh() or not self.control_state_allowed((1, 2)) or
                getattr(self, 'readonly_wait_generation', None) != self.worker_generation or marker is None):
            return False
        try:
            identity = (int(parts['boot'], 16), int(parts['reference']), int(parts['state_seq']))
        except (KeyError, TypeError, ValueError):
            return False
        state = self.last_state
        return (identity[:2] == marker[:2] and identity[2] >= marker[2] and
                state['boot_epoch'] == marker[0] and state['reference_epoch'] == marker[1] and
                state['state_seq'] >= marker[2] and state['session_id'] == 0)

    def manual_limits_mode(self):
        return self.cfg.get('calibration_mode') == 'manual_limits'

    def calibration_command(self):
        return 'LIMITS CONFIRM_MARGIN' if self.manual_limits_mode() else 'HOME CONFIRM_MARGIN'

    def manual_limits_retreat_target(self):
        if not self.manual_limits_mode() or not self.last_state:
            return None
        state = self.last_state
        if (state.get('schema_major'), state.get('schema_minor')) != (2, 5) or state.get('profile_id') != 201 or state['position_valid_mask'] != 63:
            return None
        target = []
        boundary = False
        for axis, (value, joint, (lower, upper)) in enumerate(zip(state['position_rad'], self.cfg['joints'], self.limits)):
            nominal_lower, nominal_upper = map(math.radians, joint['nominal_limits_deg'])
            # 只有原0.1°反馈带、≤.003rad起点投影可请求Rest；实际State不夹紧。
            if not math.isfinite(value) or value < nominal_lower - math.radians(.1) or value > nominal_upper + math.radians(.1):
                return None
            projected = min(nominal_upper, max(nominal_lower, value))
            if abs(projected - value) > .003:
                return None
            below = value < math.radians(lower) - .003
            above = value > math.radians(upper) + .003
            # 仅接受指定机械止挡侧；J2上侧/J3下侧与其它轴均不能借用该链。
            if (axis == 1 and above) or (axis == 2 and below) or (axis not in (1, 2) and (below or above)):
                return None
            # Native's protected Rest still uses the original 13deg-.003rad
            # trigger, independently of the ordinary goal input range.
            rest_margin = math.radians(13) - .003
            danger = ((axis == 1 and value < nominal_lower + rest_margin) or
                      (axis == 2 and value > nominal_upper - rest_margin))
            if danger:
                boundary = True
                target.append(self.cfg['rest_deg'][axis])
            else:
                target.append(math.degrees(value))
        return target if boundary else None

    def restore_ordinary_goal_ranges(self):
        if not self.manual_limits_mode():
            return
        # 允许输入名义范围；执行前仍按self.limits内收，不扩大实机目标域。
        nominal = [joint['nominal_limits_deg'] for joint in self.cfg['joints']]
        for spin, slider, (lower, upper) in zip(self.inputs, self.sliders, nominal):
            spin.setRange(lower, upper)
            slider.setRange(round(lower * 100), round(upper * 100))

    def optional_display_anchor(self, state):
        candidate = self.cfg.get('initial_display_anchor')
        configured_boot = self.cfg.get('initial_display_boot')
        if not isinstance(candidate, dict) or configured_boot is None or state['raw_position_valid_mask'] != 63:
            return None
        try:
            configured_boot = int(configured_boot, 16) if isinstance(configured_boot, str) else int(configured_boot)
            anchor_boot = candidate.get('boot')
            anchor_boot = int(anchor_boot, 16) if isinstance(anchor_boot, str) else int(anchor_boot)
            motor = candidate.get('motor_rev')
            home = candidate.get('home_rad')
            raw = state['raw_motor_position_rev']
            if configured_boot != state['boot_epoch'] or anchor_boot != configured_boot:
                return None
            if any(not isinstance(values, (list, tuple)) or len(values) != 6 for values in (motor, home, raw)):
                return None
            if any(type(value) not in (int, float) or not math.isfinite(value) for values in (motor, home, raw) for value in values):
                return None
            if any(abs(a - b) > 0.5 for a, b in zip(motor, raw)):
                return None
            return dict(candidate, boot=f'{anchor_boot:x}', motor_rev=list(motor), home_rad=list(home))
        except (TypeError, ValueError, OverflowError):
            return None

    def resume_position_issue(self):
        state = self.last_state
        if not state or state['position_valid_mask'] != 63:
            return '实际位置反馈未就绪。'
        # 与Native原MOVE起点一致：配置已内收13°，只保留原0.003rad反馈容差。
        for axis,(value,(lower,upper)) in enumerate(zip(state['position_rad'],self.limits)):
            if not math.isfinite(value) or value < math.radians(lower)-0.003 or value > math.radians(upper)+0.003:
                return f'J{axis+1}实际 {math.degrees(value):.3f}°，工作范围 [{lower:g}, {upper:g}]°。'
        return None

    def recovery_target(self):
        if (self.manual_limits_mode() or not self.cfg.get('inward_recovery_enabled',False) or not self.last_state or
                self.last_state['position_valid_mask'] != 63 or
                (self.last_state.get('schema_major'),self.last_state.get('schema_minor')) != (2,5) or
                self.last_state.get('profile_id') != 201 or
                (self.cfg.get('wire_schema_major',2),self.cfg.get('wire_schema_minor',5)) != (2,5)):
            return None
        target=[]
        danger=False
        for value,joint,(lower,upper) in zip(self.last_state['position_rad'],self.cfg['joints'],self.limits):
            nominal_lower,nominal_upper=joint['nominal_limits_deg']
            if not math.isfinite(value) or value<math.radians(nominal_lower) or value>math.radians(nominal_upper):
                return None
            if value<math.radians(lower)-0.003:
                target.append(nominal_lower+15.0);danger=True
            elif value>math.radians(upper)+0.003:
                target.append(nominal_upper-15.0);danger=True
            else:
                target.append(math.degrees(value))
        return target if danger else None

    def start_recovery(self):
        target=self.recovery_target()
        if not self.control_state_allowed((2, 4)) or not self.authorized or not self.calibration or not self.fresh() or self.busy or target is None:
            return
        answer=QtWidgets.QMessageBox.question(self,'低速回工作区',
            '仅将缺少停车余量的关节向内收至原限位内15°，其它轴保持当前实测位置。\n'
            '固定最高0.1°/s、加速度0.01rad/s²；请确认有支撑且内收路径畅通。\n'
            + '目标：' + ' / '.join(f'J{i+1} {q:.3f}°' for i,q in enumerate(target)),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.No)
        if answer==QtWidgets.QMessageBox.Yes and self.control_state_allowed((2, 4)) and self.authorized and self.fresh() and not self.busy and self.recovery_target() is not None:
            if self.command('RECOVER'):
                self.busy=True

    def start_resume(self):
        if not self.control_state_allowed((2,)) or not self.wait_home or not self.calibration or not self.fresh() or self.busy:
            return
        issue = self.resume_position_issue()
        retreat = self.manual_limits_retreat_target()
        if issue and retreat is None and self.recovery_target() is None:
            self.message('零点保留，实际位置不足停车余量，需支撑调整后恢复；无需重新归零。' + issue)
            return
        resume_text = ('复用本次boot零点，不重新采零。本次将缓慢退回工作Rest：仅危险的J2下侧/J3上侧移动，其余轴冻结当前实测值。\n'
                       '固定最高2°/s、加速度0.1rad/s²；完成原Rest验证后才恢复普通运动。\n'
                       if retreat is not None else
                       '复用设备本次boot的零点，并在当前实测位置使能保持。不会回到Home或Rest。\n'
                       + ('当前位置仅允许固定0.1°/s低速内收；使能后请点击“低速回工作区”。\n' if issue else ''))
        answer = QtWidgets.QMessageBox.question(self, '恢复控制',
            resume_text + '请确认机械臂有支撑、路径安全且可操作现场保护。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer == QtWidgets.QMessageBox.Yes:
            # 模态对话期间反馈仍会更新；确认后再次检查实际位置和连接，不能使用旧位置。
            issue = self.resume_position_issue()
            if not self.control_state_allowed((2,)) or not self.wait_home or not self.calibration or not self.fresh() or self.busy or (issue and self.manual_limits_retreat_target() is None and self.recovery_target() is None):
                self.message('零点保留，恢复条件已变化，未发送使能命令。' + (issue or '请等待新鲜静止反馈。'))
                return
            if self.command('RESUME'):
                self.busy = True
                self.wait_home = False

    @staticmethod
    def sync_slider(slider, value):
        blocker = QtCore.QSignalBlocker(slider)
        slider.setValue(round(value * 100))
        del blocker

    def show_actual_description(self, visible):
        if visible != self.description_visible:
            message = String()
            message.data = self.description_xml if visible else '<robot name="feedback_unavailable"><link name="base_link"/></robot>'
            self.description_publisher.publish(message)
            self.description_visible = visible

    def set_preview_failed(self, failed):
        if self.preview_failed != failed:
            from manual_moveit_target import colored_description
            message = String()
            message.data = colored_description(self.description_xml, failed)
            self.preview_description_publisher.publish(message)
            self.preview_failed = failed

    def set_preview_pose(self, degrees):
        for box, value in zip(self.inputs, degrees):
            box.setValue(value)
        self.message('仅更新目标虚影，未发送实机命令。')

    def message(self, text):
        self.log.appendPlainText(text)

    def command(self, text):
        if self.worker.state() != QtCore.QProcess.Running:
            self.message('通信进程已退出；需重新只读连接，零点以设备本次boot反馈为准。')
            return False
        self.worker.write((text + '\n').encode())
        with (self.report / 'commands.jsonl').open('a') as f:
            f.write(json.dumps({'host_ns': time.monotonic_ns(), 'command': text}) + '\n')
        return True

    def fresh(self):
        if self.last_state is None:
            return False
        age = time.monotonic_ns() - self.last_state_ns
        return (0 <= age <= 250_000_000 and
                max(self.last_state['can_rx_age_us']) * 1000 + age <= 250_000_000)

    def start_zero(self):
        if (not self.control_state_allowed() or not self.fresh() or self.busy or self.closing or self.pending_home or
                self.worker.state() != QtCore.QProcess.Running or
                not (self.wait_home or self.authorized)):
            return
        if self.manual_limits_mode():
            pose = self.cfg.get('calibration_pose_deg')
            if not isinstance(pose, (list, tuple)) or len(pose) != 6 or any(type(value) not in (int, float) or not math.isfinite(value) for value in pose):
                self.message('缺少完整人工双限位姿态配置，未发送标定请求。')
                return
            expected_pose = list(self.cfg.get('home_deg', [0, 0, 90, 0, 0, 0]))
            expected_pose[1] = self.cfg['joints'][1]['nominal_limits_deg'][0]
            expected_pose[2] = self.cfg['joints'][2]['nominal_limits_deg'][1]
            if list(pose) != expected_pose:
                self.message('人工双限位姿态与当前模型不一致，未发送标定请求。')
                return
            title = '确认人工双限位姿态和退回路径'
            confirmation = (f'现在是否已由人工轻靠J2/J3机械止挡，实物为 {list(pose)}°，其余四轴0°，有固定支撑？\n'
                            '不要用电机自动顶挡块。此姿态不是旧Home；请确认原工作Rest路径畅通。\n'
                            '点击“是”将替换当前参考并固定低速退工作Rest（≤2°/s、≤0.1rad/s²）。\n'
                            '若尚未人工摆到该限位姿态，请点“否”；不会把当前姿态冒充参考。')
        else:
            title = '确认人工 Home 和运动路径'
            confirmation = ('现在实物是否已人工摆到 Home [0°,0°,90°,0°,0°,0°]？\n'
                            f"有支撑、现场可操作保护，且到工作 Rest {self.cfg['rest_deg']}° 的路径畅通？\n"
                            '点击“是”将捕获 Home 零点并立即缓慢运动到 Rest。\n'
                            '当前若仍在 Rest，请点“否”，先手动恢复 Home。')
        answer = QtWidgets.QMessageBox.question(
            self, title, confirmation,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes or not self.control_state_allowed() or not self.fresh():
            return
        # 用户明确重新Home才可重采零；先确认旧owner已失能释放。
        self.zero_busy = self.busy = True
        if self.wait_home:
            if not self.command(self.calibration_command()):
                self.zero_busy = self.busy = False
                return
            self.wait_home = False
        else:
            self.pending_home = True
            self.pending_home_generation = self.worker_generation + 1
            self.restart_after_release = True
            self.authorized = False
            if not self.command('RELEASE'):
                self.pending_home = self.restart_after_release = False
                self.pending_home_generation = None
                self.zero_busy = self.busy = False
                return
        self.message('已确认当次人工双限位；等待失能释放后捕获新参考并低速退工作Rest。' if self.manual_limits_mode() else '已确认当次人工Home；等待失能释放后捕获新参考并缓慢Rest。')

    def start_move(self):
        if hasattr(self, 'moveit_tab') and self.moveit_tab.executing:
            return
        if not self.control_state_allowed((2, 4)) or not self.authorized or not self.calibration or not self.fresh() or self.busy:
            return
        requested = [spin.value() for spin in self.inputs]
        nominal = [joint['nominal_limits_deg'] for joint in self.cfg['joints']]
        if not valid_goal(requested, nominal):
            self.message('目标无效；要求六轴有限角度且全部处于名义硬范围。未发送运动命令。')
            return
        goal = [min(upper, max(lower, value))
                for value, (lower, upper) in zip(requested, self.limits)]
        if not valid_goal(goal, self.limits):
            self.message('有效目标不满足软件范围，未发送运动命令。')
            return
        projection = '；'.join(f'J{i + 1} {before:.2f}° → {after:.2f}°'
                              for i, (before, after) in enumerate(zip(requested, goal)) if before != after)
        if projection:
            self.set_preview_pose(goal)
            self.message('已按现有停车余量内收请求：' + projection + '。仅更新目标虚影，未发送运动命令。')
        text = ' / '.join(f'J{i + 1} {v:.2f}°' for i, v in enumerate(goal))
        if projection:
            text = '请求 → 有效目标：' + projection + '\n实际发送：' + text
        answer = QtWidgets.QMessageBox.question(
            self, '执行六轴目标', text + f'\n本次目标速度 {self.speed_input.value():.1f}°/s，加速度 {self.acceleration_input.value():.2f}rad/s²；确认实物路径畅通后执行。\n'
            '将按所选速度和加速度规划；较长运动使用原有会话续租。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer == QtWidgets.QMessageBox.Yes and self.control_state_allowed((2, 4)) and self.authorized and self.fresh() and self.command(f'MOVE_SPEED {self.speed_input.value():.1f} ACCEL {self.acceleration_input.value():.2f} ' + ' '.join(f'{v:.8f}' for v in goal)):
            self.busy = True

    def release(self, command):
        if hasattr(self, 'moveit_tab') and self.moveit_tab.executing:
            self.moveit_tab.cancel_execution('已请求停止；剩余规划路径作废。')
        self.remember_readonly_handoff()
        self.authorized = False
        self.pending_authorization = None
        self.busy = True
        self.restart_after_release = not self.closing
        self.command(command)
        self.message('已请求停止、失能并释放；等待设备确认。')

    def remember_readonly_handoff(self):
        # This proves only that a stopped process can hand over its USB reader.
        # It grants no command authority and never extends State freshness.
        state = self.last_state
        self.readonly_handoff = ((self.worker_generation, state['boot_epoch'], state['reference_epoch'])
                                 if state and self.fresh() and not self.authorized and
                                 state['core_state'] == 9 and state['session_id'] == 0 else None)

    def copy_actual(self):
        if not self.fresh() or not hasattr(self, 'actual_q'):
            return
        for box, radians in zip(self.inputs, self.actual_q):
            box.setValue(math.degrees(radians))
        self.message('目标已填入；未发送运动命令。')

    def stdout_ready(self):
        text = bytes(self.worker.readAllStandardOutput()).decode(errors='replace')
        with (self.report / 'native.stdout').open('a') as f:
            f.write(text)
        self.stdout_buffer += text
        while '\n' in self.stdout_buffer:
            line, self.stdout_buffer = self.stdout_buffer.split('\n', 1)
            if not line.startswith('home_rest_event='):
                if line:
                    self.message(line)
                continue
            parts = dict(item.split('=', 1) for item in line.split() if '=' in item)
            event = parts['home_rest_event']
            if event not in ('REST_SAMPLE', 'LEASE_RENEWED'):
                self.message(event)
            if event == 'READONLY_FAULT':
                self.enter_readonly_fault((int(parts['boot'], 16), int(parts['state_seq'])))
                self.message(('设备参考仍存在；' if int(parts['reference']) != 0 else '当前无有效设备参考；') + '当前故障禁止控制。保持只读显示，不自动清故障或重新归零。')
            elif event == 'WAIT_MANUAL_HOME':
                self.wait_home = not getattr(self, 'readonly_fault', False) and not (self.last_state and self.last_state['core_state'] == 9)
                if self.wait_home:
                    try:
                        self.readonly_wait_identity = (int(parts['boot'], 16), int(parts['reference']), int(parts['state_seq']))
                        self.readonly_wait_generation = self.worker_generation
                    except (KeyError, TypeError, ValueError):
                        self.readonly_wait_identity = None
                        self.readonly_wait_generation = None
            elif event == 'CAPTURE_MODE_REJECTED':
                if self.capture_rejection_is_current_readonly(parts):
                    self.authorized = False
                    self.pending_authorization = None
                    self.busy = self.zero_busy = self.pending_home = False
                    self.pending_home_generation = None
                    self.restart_after_release = False
                    self.wait_home = True
                    self.message('标定模式与设备不匹配，保持只读等待明确姿态确认；未自动重试。')
            elif event == ('LIMITS_CONFIRMED_NO_MOTION' if self.manual_limits_mode() else 'HOME_CONFIRMED_NO_MOTION') and self.zero_busy and self.control_state_allowed((1, 2)):
                self.command('REST CONFIRM_MARGIN')
            elif event in ('CALIBRATION_READY', 'CALIBRATION_RESUMED'):
                if getattr(self, 'readonly_fault', False) or (self.last_state and self.last_state['core_state'] == 9):
                    self.authorized = False
                    self.pending_authorization = None
                    continue
                self.calibration_event_ns = time.monotonic_ns()
                self.calibration = (int(parts['boot'], 16), int(parts['reference']))
                self.authorized = False
                self.control_session = int(parts['session'])
                # stdout事件不是反馈帧；等同tuple且达到事件State/epoch水位的新鲜反馈才开放MOVE。
                self.pending_authorization = dict(
                    boot_epoch=int(parts['boot'],16),session_id=int(parts['session']),
                    reference_epoch=int(parts['reference']),state_seq=int(parts['state_seq']),
                    motion_epoch=int(parts['motion']),management_epoch=int(parts['management']))
                self.zero_busy = self.busy = False
                self.restore_ordinary_goal_ranges()
                if event == 'CALIBRATION_READY':
                    for box, value in zip(self.inputs, self.cfg['rest_deg']):
                        box.setValue(value)
                self.message('设备零点有效，本控制会话已就绪；失能或换软件连接保留同boot已完成零点。')
            elif event == 'MOVE_COMPLETED':
                self.busy = False
                self.message('目标完成。连接正常时持续保持；停止、断连会释放控制；已完成零点由设备同boot保留。')
            elif event == 'RESUME_NOT_READY':
                self.authorized = False
                self.pending_authorization = None
                self.busy = False
                self.wait_home = self.control_state_allowed((1, 2))
            elif event.startswith('MOVE_REJECTED') or event in ('MOVE_NOOP', 'MOVE_NOT_READY'):
                self.busy = False
            elif event == 'RELEASE_CONFIRMED':
                self.authorized = False
                self.pending_authorization = None
                self.release_confirmed = True
                self.message('设备已确认失能与释放；已完成零点由设备同boot保留。')
            if hasattr(self, 'moveit_tab'):
                self.moveit_tab.native_event(event, parts)

    def stderr_ready(self):
        text = bytes(self.worker.readAllStandardError()).decode(errors='replace')
        with (self.report / 'native.stderr').open('a') as f:
            f.write(text)
        if text.strip():
            self.message(text.strip()[-1200:])

    def finished(self, code, _status):
        was_readonly = self.wait_home
        handoff = getattr(self, 'readonly_handoff', None)
        state = self.last_state
        fault_readonly = (handoff is not None and state is not None and not self.authorized and
                          handoff == (self.worker_generation, state['boot_epoch'], state['reference_epoch']) and
                          state['session_id'] == 0)
        self.readonly_handoff = None
        self.authorized = False
        self.pending_authorization = None
        self.wait_home = False
        self.busy = self.zero_busy = False
        self.message(f'通信进程已退出，code={code}；控制已结束，零点以设备当前boot反馈为准。原始证据：{self.raw}')
        if getattr(self, 'connection_action', None) is not None:
            if not self.closing and code in (0, 130) and (self.release_confirmed or (code == 0 and (was_readonly or fault_readonly))):
                QtCore.QTimer.singleShot(0, self.run_connection_action)
            else:
                self.connection_action = None
                self.message('原连接退出未证实释放，未启动其他通信进程；可检查后手动重新连接。')
            return
        restart = self.restart_after_release and (self.release_confirmed or ((was_readonly or fault_readonly) and code == 0)) and code in (0, 130) and not self.closing
        self.restart_after_release = False
        if restart:
            if fault_readonly:
                self.pending_home = False
                self.pending_home_generation = None
            QtCore.QTimer.singleShot(0, self.start_worker)
        else:
            self.pending_home = False
            self.pending_home_generation = None

    def accept_state(self, state, arrival_ns):
        boot, sequence = state['boot_epoch'], state['state_seq']
        if boot in self.retired_state_boots:
            return False
        if self.state_boot == boot and sequence <= self.state_sequence:
            return False
        pending = self.pending_authorization
        if pending and boot == pending['boot_epoch']:
            if sequence < pending['state_seq']:
                return False
            # 事件对应帧唯一；同水位但错tuple是迟到旧帧，不用它撤销事件后的控制准备。
            if sequence == pending['state_seq'] and any(
                    state[key] != pending[key] for key in ('session_id','reference_epoch','motion_epoch','management_epoch')):
                return False
        if self.state_boot is not None and self.state_boot != boot:
            self.retired_state_boots.add(self.state_boot)
            self.authorized = False
            self.pending_authorization = None
        self.state_boot, self.state_sequence = boot, sequence
        self.last_state, self.last_state_ns = state, arrival_ns
        return True

    def reconcile_control_state(self):
        if not self.fresh() or self.last_state is None:
            return
        state = self.last_state
        if state['core_state'] == 9:
            self.enter_readonly_fault((state['boot_epoch'], state['state_seq']))
        elif getattr(self, 'readonly_fault', False):
            marker = getattr(self, 'readonly_fault_identity', None)
            if marker is not None and ((state['boot_epoch'] == marker[0] and state['state_seq'] > marker[1]) or
                                       (state['boot_epoch'] != marker[0] and marker[0] in self.retired_state_boots)):
                self.readonly_fault = False
                self.readonly_fault_identity = None
        pending = self.pending_authorization
        if pending:
            if state['boot_epoch'] == pending['boot_epoch'] and state['state_seq'] < pending['state_seq']:
                return
            exact_reference = all(state[key] == pending[key] for key in ('boot_epoch','session_id','reference_epoch'))
            if exact_reference and state['motion_epoch'] >= pending['motion_epoch'] and state['management_epoch'] >= pending['management_epoch'] and state['position_valid_mask'] == 63:
                self.authorized = True
                self.pending_authorization = None
            elif state['boot_epoch'] != pending['boot_epoch'] or (state['state_seq'] > pending['state_seq'] and not exact_reference):
                self.authorized = False
                self.pending_authorization = None
        observed = (state['boot_epoch'],state['reference_epoch'])
        if state['reference_epoch'] != 0 and state['position_valid_mask'] == 63:
            if self.calibration and observed != self.calibration:
                self.authorized = False
            self.calibration = observed
        else:
            self.calibration = None
            self.authorized = False
        if self.authorized and state['session_id'] != self.control_session:
            self.authorized = False

    def read_state(self):
        if not self.raw.exists():
            return
        with self.raw.open('rb') as f:
            header = f.read(24)
            if len(header) != 24 or struct.unpack('<6I', header)[:4] != (0x52575034, 3, 4320, 4096):
                return
            f.seek(self.offset)
            for _ in range(128):
                record = f.read(4320)
                if len(record) != 4320:
                    break
                self.offset += 4320
                size, result = struct.unpack_from('<II', record, 48)
                interface = struct.unpack_from('<I', record, 76)[0]
                if (record[72] != 0 or result != 0 or interface != self.cfg['state_interface_id'] or not (4 <= size <= 4096)):
                    continue
                try:
                    # wire_layout决定2.4/2.5实际长度，禁止另维护301/350的尺寸副本。
                    state = self.decode('DummyJointStateV2', record[224:224 + size])
                except (ValueError, struct.error):
                    continue
                self.accept_state(state,struct.unpack_from('<Q',record,8)[0])

    def tick(self):
        if not rclpy.ok():
            self.timer.stop()
            self.close()
            return
        # 有界批量消费只读规划响应；避免逐帧一次回调使碰撞检查耗时近一分钟。
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.read_state()
        if self.preview_failed is None:
            self.set_preview_failed(False)
        # 独立目标topic/TF；仅预览，永不写入实机反馈topic或发MOVE。
        target = JointState()
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.name = [j['name'] for j in self.cfg['joints']]
        target.position = [math.radians(box.value()) for box in self.inputs]
        self.target_publisher.publish(target)
        fresh = self.fresh()
        state = self.last_state
        self.reconcile_control_state()
        quiet = fresh and state['velocity_valid_mask'] == 63 and state['raw_position_valid_mask'] == 63 and all(math.isfinite(v) and abs(v) <= 0.01 for v in state['estimated_velocity_rad_s'])
        if (self.pending_home and self.wait_home and quiet and not self.closing and
                self.pending_home_generation is not None and
                self.worker_generation >= self.pending_home_generation and self.control_state_allowed((1, 2))):
            if self.command(self.calibration_command()):
                self.pending_home = False
                self.pending_home_generation = None
                self.zero_busy = self.busy = True
                self.wait_home = False
        self.zero.setEnabled(self.control_state_allowed() and quiet and not self.busy and not self.pending_home and self.worker.state() == QtCore.QProcess.Running and
                             (self.wait_home or self.authorized) and state['core_state'] in (1, 2, 4))
        resume_issue = self.resume_position_issue() if fresh and self.calibration else None
        self.resume.setEnabled(self.control_state_allowed((2,)) and bool(self.calibration) and self.wait_home and quiet and not self.busy and (resume_issue is None or self.manual_limits_retreat_target() is not None or self.recovery_target() is not None))
        self.recover.setEnabled(self.control_state_allowed((2, 4)) and self.authorized and bool(self.calibration) and quiet and not self.busy and self.recovery_target() is not None)
        self.move.setEnabled(self.control_state_allowed((2, 4)) and self.resume_position_issue() is None and self.authorized and bool(self.calibration) and fresh and not self.busy and
                             state['position_valid_mask'] == 63 and state['velocity_valid_mask'] == 63 and
                             state['core_state'] in (2, 4) and
                             all(math.isfinite(v) and abs(v)<=0.01 for v in state['estimated_velocity_rad_s']))
        self.copy.setEnabled(fresh and state['core_state'] != 9 and not getattr(self, 'readonly_fault', False) and hasattr(self, 'actual_q'))
        self.stop.setEnabled(self.worker.state() == QtCore.QProcess.Running and not self.helper_active())
        if hasattr(self, 'moveit_tab') and self.moveit_tab.executing:
            self.busy = True
            for button in (self.move, self.zero, self.resume, self.recover, self.clear_fault):
                button.setEnabled(False)
        if hasattr(self, 'reconnect'):
            self.reconnect.setEnabled(not self.closing and not self.busy and not self.helper_active())
            self.clear_fault.setEnabled(not self.closing and self.fault_clear_available())
        if not fresh:
            self.show_actual_description(False)
            self.status.setText('反馈未就绪 / 已失鲜；运动入口禁用。零点状态以设备同boot反馈为准，当前不授权运动。')
            return
        if not self.display_initialized:
            self.display_initialized = True
            self.anchor = self.optional_display_anchor(state)
            if self.anchor is not None:
                (self.report / 'display-only-Rest-anchor.json').write_text(json.dumps(self.anchor, indent=2))
        if self.previous_raw and any(abs(a - b) > 1.0 for a, b in zip(self.previous_raw, state['raw_motor_position_rev'])):
            self.anchor = None
        self.previous_raw = state['raw_motor_position_rev']
        q = None
        scope = '未标定'
        if state['position_valid_mask'] == 63 and state['reference_epoch'] != 0:
            q = state['position_rad']
            scope = '当前设备参考'
        elif self.anchor and state['boot_epoch'] == int(self.anchor['boot'], 16) and state['raw_position_valid_mask'] == 63:
            q = [h + j['direction'] * (m - a) * 2 * math.pi / j['gear_ratio']
                 for h, j, m, a in zip(self.anchor['home_rad'], self.cfg['joints'],
                                      state['raw_motor_position_rev'], self.anchor['motor_rev'])]
            scope = '人工确认姿态的显示参考（不授权运动）'
        self.show_actual_description(q is not None and all(math.isfinite(v) for v in q))
        if q is not None and all(math.isfinite(v) for v in q):
            self.actual_q = q
            for label, radians in zip(self.feedback_labels, q):
                label.setText(f'{math.degrees(radians):.3f}°')
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = [j['name'] for j in self.cfg['joints']]
            msg.position = q
            self.publisher.publish(msg)
        if state['core_state'] == 9 or getattr(self, 'readonly_fault', False):
            reference_note = '设备参考仍存在（同boot）' if state['reference_epoch'] != 0 else '当前没有有效设备参考'
            self.status.setText(f'USB 实时反馈 · {reference_note} · 故障禁止控制，仅可停止或关闭只读连接；未自动清故障。 · boot={state["boot_epoch"]:016x}')
            return
        zero = '标定进行中' if self.zero_busy else ('设备参考存在（同boot）' if self.calibration else '未标定')
        zero += ' · 已使能控制' if self.authorized else ' · 未取得控制权限'
        speed_note = ''
        if self.calibration and not self.busy:
            fast = [f'J{i+1} {v:.4f}rad/s' for i,v in enumerate(state['estimated_velocity_rad_s']) if abs(v)>0.01]
            if fast: speed_note = ' · 等待静止：' + ', '.join(fast)
        resume_note = ''
        if resume_issue and self.manual_limits_retreat_target() is not None:
            resume_note = ' · 零点保留；明确恢复后仅危险的J2下侧/J3上侧以≤2°/s缓退工作Rest，其余轴冻结，不重新采零。'
        elif resume_issue and self.recovery_target() is not None:
            resume_note = ' · 零点保留；仅允许固定0.1°/s低速内收，需明确使能并点击低速回工作区。'
        elif resume_issue and self.wait_home and not self.authorized:
            resume_note = ' · 零点保留，实际位置不足停车余量，需支撑调整后恢复；无需重新归零。' + resume_issue
        self.status.setText(f'USB 实时反馈{speed_note} · {zero} · {scope}{resume_note} · boot={state["boot_epoch"]:016x}')

    def closeEvent(self, event):
        self.closing = True
        self.connection_action = None
        if self.helper_active():
            self.manage_worker.terminate()
            self.manage_worker.waitForFinished(6000)
            if self.helper_active():
                event.ignore()
                self.closing = False
                self.message('清故障进程尚未退出，窗口保持打开；未启动其他通信。')
                return
        self.pending_home = self.restart_after_release = False
        self.pending_home_generation = None
        if self.worker.state() == QtCore.QProcess.Running:
            self.release('RELEASE')
            deadline = time.monotonic() + 6
            while self.worker.state() == QtCore.QProcess.Running and time.monotonic() < deadline:
                self.worker.waitForFinished(20)
                QtWidgets.QApplication.processEvents()
            if self.worker.state() == QtCore.QProcess.Running:
                self.worker.terminate()
                self.worker.waitForFinished(6000)
            if self.worker.state() == QtCore.QProcess.Running:
                event.ignore()
                self.closing = False
                self.message('释放尚未确认，窗口保持打开；请检查现场保护。')
                return
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--description-xacro', required=True)
    parser.add_argument('--moveit', action='store_true', help='启用末端规划页；另启动仅规划 MoveIt 服务')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    config = json.loads(Path(args.config).read_text())
    config['description_xacro'] = args.description_xacro
    config['moveit_enabled'] = args.moveit
    sys.path.insert(0, config['decoder_directory'])
    from join_real_6dof_evidence import _decode
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    rclpy.init(args=[], signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node('dummy_manual_joint_panel')
    panel = JointPanel(config, Path(args.report), _decode, node)
    signal.signal(signal.SIGTERM, lambda *_: QtCore.QTimer.singleShot(0, panel.close))
    signal.signal(signal.SIGINT, lambda *_: QtCore.QTimer.singleShot(0, panel.close))
    panel.show()
    try:
        return app.exec_()
    finally:
        panel.timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
