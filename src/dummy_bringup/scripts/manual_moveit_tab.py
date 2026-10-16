"""末端位姿、规划与独立虚影播放；运动接入由原面板管理。"""
import math
import time
import hashlib
import json

from PyQt5 import QtCore, QtWidgets
from manual_moveit_planner import MoveItPlanner, validate_trajectory
from manual_moveit_execution import SegmentedExecution, collision_samples, execution_polyline
from manual_moveit_target import target_markers


class MoveItTab(QtWidgets.QWidget):
    def __init__(self, panel):
        super().__init__(panel)
        self.panel = panel
        self.planner = MoveItPlanner(panel.node, [j['name'] for j in panel.cfg['joints']], panel.limits)
        self.pending = None
        self.path = None
        self.identity = None
        self.revision = 0
        self.play_index = None
        self.execution = None
        self.completed_marker = None
        self.started_motion = None
        self.previewed = False
        self.checks = []
        self.samples = ()
        self.checked_count = 0
        self.target_failed = False
        self.preset_fk = None
        self.marker_publisher = None
        if panel.node is not None:
            from visualization_msgs.msg import MarkerArray
            from rclpy.qos import QoSProfile, DurabilityPolicy
            self.marker_publisher = panel.node.create_publisher(MarkerArray, '/manual_target_marker',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.preview_timer = QtCore.QTimer(self)
        self.preview_timer.setSingleShot(True)
        self.preview_timer.timeout.connect(self.start_plan)
        self.jog_timer = QtCore.QTimer(self)
        self.jog_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.jog_timer.setInterval(10)
        self.jog_timer.timeout.connect(self.advance_jog)
        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel('基座坐标系 base_link；修改与规划均不运动。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addWidget(QtWidgets.QLabel('XYZ滑块控制增减速度：每10ms更新，松开即停；停更100ms自动规划。预览不运动，执行需单独确认。'))
        grid = QtWidgets.QGridLayout()
        self.inputs = []
        self.jog_sliders = []
        for i, name in enumerate(('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')):
            box = QtWidgets.QDoubleSpinBox()
            # 两位小数会使靠近腕部奇异位姿的已知可达目标变成不可解目标。
            box.setDecimals(4)
            box.setRange(-2000 if i < 3 else -180, 2000 if i < 3 else 180)
            box.setSuffix(' mm' if i < 3 else ' °')
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self.invalidate)
            box.valueChanged.connect(self.target_changed)
            grid.addWidget(QtWidgets.QLabel(name), i, 0)
            grid.addWidget(box, i, 1)
            if i < 3:
                slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                slider.setRange(-100, 100)
                slider.setToolTip('偏离中心越远增减越快，每10ms更新；松开停止并回中，目标位置保留。')
                slider.sliderPressed.connect(lambda axis=i: self.begin_jog(axis))
                slider.valueChanged.connect(lambda value, axis=i: self.jog_axis(axis, value))
                slider.sliderReleased.connect(lambda axis=i: self.end_jog(axis))
                grid.addWidget(slider, i, 2, 1, 2)
                self.jog_sliders.append(slider)
            self.inputs.append(box)
        self.step = QtWidgets.QDoubleSpinBox()
        self.step.setRange(.1, 200.)
        self.step.setValue(panel.cfg.get('moveit_jog_speed_default_mm_s', 100.))
        self.step.setSuffix(' mm/s')
        grid.addWidget(QtWidgets.QLabel('滑块最大增减速度'), 6, 0)
        grid.addWidget(self.step, 6, 1)
        layout.addLayout(grid)
        row = QtWidgets.QHBoxLayout()
        self.capture = QtWidgets.QPushButton('采用实际末端位姿')
        self.play = QtWidgets.QPushButton('预览 / 停止（播放虚影）')
        self.execute = QtWidgets.QPushButton('执行规划路径')
        for button in (self.capture, self.play, self.execute):
            button.setEnabled(False)
            row.addWidget(button)
        layout.addLayout(row)
        self.capture.clicked.connect(self.capture_actual)
        self.play.clicked.connect(self.toggle_preview)
        self.execute.clicked.connect(self.start_execution)
        self.play_speed = QtWidgets.QDoubleSpinBox()
        self.play_speed.setRange(.05, 20.)
        self.play_speed.setDecimals(2)
        self.play_speed.setSingleStep(.25)
        self.play_speed.setValue(panel.cfg.get('moveit_preview_speed_default', 20.))
        self.play_speed.setSuffix(' ×')
        row.addWidget(QtWidgets.QLabel('预览倍速'))
        row.addWidget(self.play_speed)
        presets = QtWidgets.QHBoxLayout()
        self.preset_buttons = []
        for label, pose in (('规划至 Home', panel.cfg['home_deg']), ('规划至工作 Rest', panel.cfg['rest_deg'])):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _, q=pose: self.start_joint_plan(q))
            self.preset_buttons.append(button)
            presets.addWidget(button)
        layout.addLayout(presets)
        layout.addWidget(QtWidgets.QLabel('实机沿规划几何路径重新限速；同一直线采样点等价合并，弯曲路径保留路点。不保证动画原耗时。'))
        layout.addStretch()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(60)

    def invalidate(self, *_):
        if self.executing:
            self.cancel_execution('规划内容变化，已停止后续路径。', stop=True)
        self.revision += 1
        self.path = None
        self.play_index = None
        self.previewed = False
        self.set_target_failed(False)
        if self.preset_fk:
            self.preset_fk[0].cancel()
            self.preset_fk = None
        for future in self.checks:
            future.cancel()
        self.checks = []
        self.samples = ()

    def publish_target(self):
        if self.marker_publisher is not None and len(self.inputs) == 6:
            self.marker_publisher.publish(target_markers([box.value() for box in self.inputs], self.target_failed))

    def set_target_failed(self, failed):
        self.target_failed = failed
        if hasattr(self.panel, 'set_preview_failed'):
            self.panel.set_preview_failed(failed)
        self.publish_target()

    def begin_jog(self, axis):
        self.jog_timer.start()

    def jog_axis(self, axis, value):
        slider = self.jog_sliders[axis]
        if not slider.isSliderDown():
            self.inputs[axis].setValue(self.inputs[axis].value()+value*self.step.value()/100.*.01)
            self.end_jog(axis)

    def advance_jog(self):
        if self.executing:
            return
        for axis, slider in enumerate(self.jog_sliders):
            if slider.isSliderDown() and slider.value():
                self.inputs[axis].setValue(self.inputs[axis].value()+slider.value()*self.step.value()/100.*.01)

    def end_jog(self, axis):
        blocker = QtCore.QSignalBlocker(self.jog_sliders[axis])
        self.jog_sliders[axis].setValue(0)
        del blocker
        if not any(slider.isSliderDown() for slider in self.jog_sliders):
            self.jog_timer.stop()

    def target_changed(self, *_):
        self.publish_target()
        if not self.executing:
            self.preview_timer.start(100)

    def fill_pose(self, pose):
        q = pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        values = (pose.position.x*1000, pose.position.y*1000, pose.position.z*1000,
                  math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        for box, value in zip(self.inputs, values):
            blocker = QtCore.QSignalBlocker(box)
            box.setValue(value)
            del blocker
        self.publish_target()

    @property
    def executing(self):
        return self.execution is not None and not self.execution.done and not self.execution.aborted

    def execution_identity(self):
        s = self.panel.last_state
        return (s['boot_epoch'], s['reference_epoch'], s['session_id']) if self.panel.fresh() and s else None

    def ready(self):
        p = self.panel
        return (p.fresh() and p.control_state_allowed((4,)) and p.authorized and
                bool(p.calibration) and p.resume_position_issue() is None and
                p.last_state['velocity_valid_mask'] == 63 and
                all(math.isfinite(v) and abs(v) <= .01 for v in p.last_state['estimated_velocity_rad_s']))

    def record(self, event, **fields):
        with (self.panel.report / 'moveit-events.jsonl').open('a') as f:
            f.write(json.dumps(dict(event=event, host_ns=time.monotonic_ns(), **fields))+'\n')

    def current_identity(self):
        state = self.panel.last_state
        return (state['boot_epoch'], state['reference_epoch']) if state and self.panel.fresh() else None

    def begin(self, phase, future, start):
        self.pending = (phase, future, time.monotonic()+12., self.revision,
                        self.current_identity(), tuple(start))
        self.status.setText({'fk': '读取实际末端位姿…', 'ik': '求解目标关节角…', 'plan': '规划自碰撞有效路径…'}[phase])

    def capture_actual(self):
        if not self.panel.fresh() or self.pending or not hasattr(self.panel, 'actual_q'):
            return
        self.invalidate()
        self.preview_timer.stop()
        start = tuple(self.panel.actual_q)
        self.begin('fk', self.planner.request_fk(start), start)

    def start_plan(self):
        self.preview_timer.stop()
        if self.pending and self.pending[3] != self.revision:
            self.pending[1].cancel()
            self.pending = None
        if self.executing or not self.panel.fresh() or self.pending or not hasattr(self.panel, 'actual_q'):
            return
        if not self.planner.ik.service_is_ready() or not self.planner.plan.service_is_ready():
            self.status.setText('等待规划服务就绪，尚未发送运动。')
            self.preview_timer.start(100)
            return
        self.invalidate()
        start = tuple(self.panel.actual_q)
        values = [box.value() for box in self.inputs]
        self.begin('ik', self.planner.request_ik(start, values[:3], values[3:]), start)

    def start_joint_plan(self, degrees):
        if not self.panel.fresh() or self.pending or self.executing or not hasattr(self.panel, 'actual_q'):
            return
        self.invalidate()
        self.preview_timer.stop()
        start = tuple(self.panel.actual_q)
        self.panel.set_preview_pose(degrees)
        self.preset_fk = (self.planner.request_fk([math.radians(v) for v in degrees]), self.revision)
        self.begin('plan', self.planner.request_plan(start, [math.radians(v) for v in degrees]), start)

    def toggle_preview(self):
        self.play_index = None if self.play_index is not None else (0 if self.path else None)
        if self.play_index == 0:
            self.preview_started = time.monotonic()
            self.play_clock = self.preview_started
            self.play_elapsed = 0.
            self.status.setText('仅播放虚影；预览结束后需单独点击“执行规划路径”，不会自动运动。')

    def start_execution(self):
        if not self.path or self.checks or self.samples or self.panel.busy or not self.ready() or self.pending or self.executing:
            return
        revision, path, identity = self.revision, self.path, self.execution_identity()
        execution_path = execution_polyline(path)
        speed = self.panel.speed_input.value()
        acceleration = self.panel.acceleration_input.value()
        answer = QtWidgets.QMessageBox.question(self, '执行 MoveIt 完整路径',
            f'规划含 {len(path)} 个采样点，几何等价执行 {len(execution_path)-1} 段；速度上限 {speed:g}°/s、加速度 {acceleration:g}rad/s²。\n'
            '各路点会停车，实机耗时与规划动画不同。确认环境无其它物体、机械臂路径畅通后执行。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        if revision != self.revision or path != self.path or identity != self.execution_identity() or self.panel.busy or not self.ready():
            self.status.setText('确认期间状态变化，未执行。请重新规划。')
            return
        try:
            self.execution = SegmentedExecution(execution_path, identity)
            self.execution.check(self.panel.actual_q, identity, path[0])
            self.execution_speed, self.execution_acceleration = speed, acceleration
            self.started_motion = self.completed_marker = None
            self.panel.busy = True
            self.record('EXECUTION_CONFIRMED', identity=identity, points=path,
                        execution_points=execution_path, original_times_s=self.times,
                        speed_deg_s=speed, acceleration_rad_s2=acceleration)
            self.advance_execution()
        except ValueError as error:
            self.cancel_execution(str(error), stop=False)

    def cancel_execution(self, reason, stop=False):
        was_active = self.executing
        if self.execution:
            self.execution.abort()
        self.execution = None
        self.completed_marker = self.started_motion = None
        self.panel.busy = False
        self.status.setText(reason)
        if was_active:
            self.record('EXECUTION_ABORTED', reason=reason)
        if was_active and stop:
            self.panel.release('STOP')

    def native_event(self, event, parts):
        if not self.executing:
            return
        identity = (int(parts['boot'], 16), int(parts['reference']), int(parts['session']))
        if identity != self.execution.identity:
            self.cancel_execution('设备事件身份改变，停止整条路径。', stop=True)
            return
        if event == 'MOVE_STARTED':
            self.started_motion = int(parts['motion'])
        elif event == 'MOVE_COMPLETED' and self.started_motion is not None and int(parts['motion']) > self.started_motion:
            self.completed_marker = (int(parts['motion']), int(parts['state_seq']))
        elif event == 'MOVE_NOOP':
            self.completed_marker = (int(parts['motion']), int(parts['state_seq']))
        elif event.startswith('MOVE_REJECTED') or event in ('MOVE_NOT_READY', 'READONLY_FAULT', 'EXIT_CLEANUP', 'RELEASE_CONFIRMED', 'RELEASE_UNCONFIRMED'):
            self.cancel_execution('Native 拒绝或退出：'+event, stop=True)

    def advance_execution(self):
        if not self.executing:
            return
        if not self.panel.fresh() or not self.panel.authorized or self.execution_identity() != self.execution.identity:
            self.cancel_execution('反馈、控制权限或设备身份变化，已停止路径。', stop=True)
            return
        if self.completed_marker:
            motion, seq = self.completed_marker
            state = self.panel.last_state
            if state['motion_epoch'] < motion or state['state_seq'] < seq:
                return
            if not self.ready():
                return
            try:
                self.execution.confirm(self.panel.actual_q, self.execution_identity())
            except ValueError as error:
                self.cancel_execution(str(error), stop=True)
                return
            self.record('SEGMENT_COMPLETED', index=self.execution.index-1, motion=motion,
                        actual=self.panel.actual_q)
            self.completed_marker = self.started_motion = None
            if self.execution.done:
                self.panel.busy = False
                self.record('PATH_COMPLETED', identity=self.execution.identity)
                self.status.setText('完整规划路径已由设备逐段确认完成，保持使能。')
                self.execution = None
                self.path = None
                return
        if not self.execution.waiting:
            if not self.ready():
                return
            try:
                goal = self.execution.next_goal(self.panel.actual_q, self.execution_identity())
                command = (f'MOVE_SPEED {self.execution_speed:.1f} ACCEL {self.execution_acceleration:.2f} '+
                           ' '.join(f'{math.degrees(q):.8f}' for q in goal))
                if not self.panel.command(command):
                    raise ValueError('通信进程未接受运动命令')
                self.panel.busy = True
                self.segment_deadline = time.monotonic()+100.
                self.status.setText(f'执行完整路径：第 {self.execution.index} / {len(self.execution.points)-1} 段')
                self.record('SEGMENT_SUBMITTED', index=self.execution.index, goal=goal)
            except ValueError as error:
                self.cancel_execution(str(error), stop=True)

    def tick(self):
        available = self.panel.fresh() and hasattr(self.panel, 'actual_q')
        self.capture.setEnabled(available and not self.pending and not self.executing and self.planner.fk.service_is_ready())
        self.play.setEnabled(bool(self.path) and not self.executing and not self.samples and not self.checks)
        self.execute.setEnabled(bool(self.path) and self.play_index is None and not self.panel.busy and not self.executing and not self.samples and not self.checks and self.ready())
        for widget in self.inputs + self.jog_sliders + [self.step]:
            widget.setEnabled(not self.executing)
        for widget in self.preset_buttons:
            widget.setEnabled(available and not self.executing and not self.pending and self.planner.plan.service_is_ready())
        if self.preset_fk and self.preset_fk[0].done():
            future, revision = self.preset_fk
            self.preset_fk = None
            response = future.result()
            if revision == self.revision and response and response.error_code.val == 1 and response.pose_stamped:
                self.fill_pose(response.pose_stamped[0].pose)
        if self.path and self.current_identity() != self.identity:
            self.invalidate()
            self.status.setText('反馈失鲜或设备参考改变，路径已作废。')
        if self.path and not self.executing and available and any(abs(a-b) > .003 for a, b in zip(self.panel.actual_q, self.start)):
            self.invalidate()
            self.status.setText('实测起点已改变，请重新规划；未发送运动。')
        if self.pending:
            phase, future, deadline, revision, identity, start = self.pending
            if time.monotonic() >= deadline or revision != self.revision or identity != self.current_identity():
                future.cancel()
                self.pending = None
                self.status.setText('请求超时、目标或设备参考已改变；未发送运动。')
            elif future.done():
                self.pending = None
                try:
                    response = future.result()
                    if response is None:
                        raise ValueError('规划服务未返回结果')
                    if phase == 'fk':
                        if response.error_code.val != 1 or not response.pose_stamped:
                            raise ValueError(f'正解失败：{response.error_code.val}')
                        pose = response.pose_stamped[0].pose
                        self.fill_pose(pose)
                        self.status.setText('实际末端位姿已填入；可用 ±XYZ 微调后规划，不会立即运动。')
                    elif phase == 'ik':
                        if response.error_code.val != 1:
                            raise ValueError(f'此完整姿态无可用逆解 / 自碰撞：{response.error_code.val}。请减小步长、换方向或规划至 Home / Rest。')
                        js = response.solution.joint_state
                        by_name = dict(zip(js.name, js.position))
                        goal = [by_name[name] for name in self.planner.names]
                        self.panel.set_preview_pose([math.degrees(q) for q in goal])
                        self.begin('plan', self.planner.request_plan(start, goal), start)
                    else:
                        result = response.motion_plan_response
                        if result.error_code.val != 1:
                            raise ValueError(f'路径规划失败：{result.error_code.val}')
                        self.path = validate_trajectory(result.trajectory.joint_trajectory,
                                                        self.planner.names, self.panel.limits)
                        self.identity, self.start = identity, start
                        self.panel.set_preview_pose([math.degrees(q) for q in self.path[-1]])
                        self.times = tuple(p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in result.trajectory.joint_trajectory.points)
                        self.samples = list(collision_samples(self.path))
                        self.checked_count = 0
                        self.check_deadline = time.monotonic()+60.
                        self.record('PLAN_RECEIVED', identity=identity, points=self.path, original_times_s=self.times,
                                    sha256=hashlib.sha256(json.dumps(self.path).encode()).hexdigest())
                        self.status.setText(f'规划成功：{len(self.path)} 个路点；正在检查实际折线插值的自碰撞…')
                except (ValueError, KeyError, RuntimeError) as error:
                    self.path = None
                    self.set_target_failed(True)
                    text = str(error)
                    if phase == 'ik':
                        text += '；无逆解时红色虚影保留上次有效姿态，目标点显示输入位置。'
                    self.status.setText(text)
        if self.path and (self.samples or self.checks):
            try:
                if time.monotonic() >= self.check_deadline:
                    raise ValueError('路径自碰撞检查超时，未授权执行')
                remaining = []
                for future in self.checks:
                    if future.done():
                        result = future.result()
                        if not result or not result.valid:
                            raise ValueError('规划折线插值自碰撞，拒绝执行')
                        self.checked_count += 1
                    else:
                        remaining.append(future)
                self.checks = remaining
                while self.samples and len(self.checks) < 8:
                    self.checks.append(self.planner.request_validity(self.samples.pop(0)))
                if not self.samples and not self.checks:
                    endpoint = ' / '.join(f'J{i+1} {math.degrees(q):.2f}°' for i,q in enumerate(self.path[-1]))
                    self.status.setText(f'规划与检查完成：{len(self.path)} 路点 / {self.checked_count} 个样本。可直接点击“执行规划路径”；预览可选。\n终点：{endpoint}')
                    self.record('PATH_VALIDATED', samples=self.checked_count)
            except (ValueError, RuntimeError) as error:
                self.invalidate()
                self.set_target_failed(True)
                self.status.setText(str(error))
        if self.path and self.play_index is not None:
            now = time.monotonic()
            self.play_elapsed += max(0., now-self.play_clock)*self.play_speed.value()
            self.play_clock = now
            elapsed = self.play_elapsed+self.times[0]
            while self.play_index+1 < len(self.times) and elapsed >= self.times[self.play_index+1]:
                self.play_index += 1
            q = self.path[self.play_index]
            if self.play_index+1 < len(self.path):
                ratio = max(0., min(1., (elapsed-self.times[self.play_index])/(self.times[self.play_index+1]-self.times[self.play_index])))
                q = tuple(a+(b-a)*ratio for a,b in zip(q, self.path[self.play_index+1]))
            self.panel.set_preview_pose([math.degrees(v) for v in q])
            if self.play_index == len(self.path)-1:
                self.play_index = None
                self.previewed = True
                self.record('PREVIEW_COMPLETED')
                self.status.setText('预览完成；实物未运动。需要运动时单独点击“执行规划路径”并确认。')
        if self.executing:
            if self.execution.waiting and time.monotonic() > self.segment_deadline:
                self.cancel_execution('运动段等待超时，停止路径。', stop=True)
            elif not self.panel.fresh() or not self.panel.authorized or not self.panel.control_state_allowed((2, 3, 4, 5, 6)):
                self.cancel_execution('设备反馈、授权或生命周期不再允许控制。', stop=True)
            elif self.completed_marker or not self.execution.waiting:
                self.advance_execution()
            if self.executing:
                self.panel.busy = True
