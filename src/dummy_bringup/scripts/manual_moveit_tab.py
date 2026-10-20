"""末端位姿、规划与独立虚影播放；运动接入由原面板管理。"""
import math
import time
import hashlib
import json
import random
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
from manual_moveit_planner import (MoveItPlanner, validate_trajectory, nearest_ik_goal,
                                   IK_PREFERENCES, sample_reasonable_joints, ik_goal_cost)
from manual_panel_messages import moveit_error
from manual_moveit_execution import SegmentedExecution, collision_samples, execution_polyline
from manual_moveit_continuous import prepare_path, write_path, trajectory_velocities
from manual_moveit_target import target_markers
from manual_moveit_autotest import AutoTestSession


class MoveItTab(QtWidgets.QWidget):
    def __init__(self, panel):
        super().__init__(panel)
        self.panel = panel
        timeout = panel.cfg.get('moveit_segment_timeout_s', 660.)
        if (isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or
                not math.isfinite(timeout) or not 0. < timeout <= 660.):
            raise ValueError('moveit_segment_timeout_s必须为(0,660]内的有限秒数')
        self.segment_timeout_s = float(timeout)
        self.planner = MoveItPlanner(panel.node, [j['name'] for j in panel.cfg['joints']], panel.limits)
        self.planner.motion_parameters=lambda:(panel.speed_input.value(),panel.acceleration_input.value(),
            panel.cfg.get('motion_speed_max_deg_s',63.),panel.cfg.get('motion_acceleration_max_rad_s2',4.))
        self.pending = None
        self.planning_deadline = None
        self.planning_started_at = None
        self.planning_slow_noted = False
        self.random_goal = None
        self.random_attempts = 0
        self.path = None
        self.continuous_path = None
        self.continuous_enabled = panel.cfg.get('moveit_continuous_execution', True)
        self.path_file = None
        self.noop_goal = None
        self.identity = None
        self.revision = 0
        self.play_index = None
        self.execution = None
        self.completed_marker = None
        self.started_motion = None
        self.auto_test = AutoTestSession(self)
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
        hint = QtWidgets.QLabel('拖动 XYZ 滑块微调，松开即停；输入停更后自动规划。预览只播放虚影，执行独立提交。')
        hint.setWordWrap(True)
        layout.addWidget(hint)
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
            grid_row, grid_col = (i, 0) if i < 3 else (i-3, 4)
            name_label = QtWidgets.QLabel(name)
            name_label.setBuddy(box)
            box.setAccessibleName(name + ('位置（毫米）' if i < 3 else '姿态（度）'))
            grid.addWidget(name_label, grid_row, grid_col)
            grid.addWidget(box, grid_row, grid_col+1)
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
        grid.setColumnStretch(2, 1)
        grid.setColumnMinimumWidth(3, 24)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(10)
        grid.addWidget(QtWidgets.QLabel('微调速度'), 3, 0)
        grid.addWidget(self.step, 3, 1)
        layout.addLayout(grid)
        preference_row = QtWidgets.QHBoxLayout()
        preference_label = QtWidgets.QLabel('逆解偏好')
        self.ik_preference = QtWidgets.QComboBox()
        self.ik_preference.setAccessibleName('逆解偏好')
        preference_label.setBuddy(self.ik_preference)
        for key,label,description in IK_PREFERENCES:
            self.ik_preference.addItem(label,key)
            self.ik_preference.setItemData(self.ik_preference.count()-1,description,QtCore.Qt.ToolTipRole)
        self.ik_preference.setToolTip(IK_PREFERENCES[0][2])
        self.ik_preference.currentIndexChanged.connect(self.preference_changed)
        preference_row.addWidget(preference_label)
        preference_row.addWidget(self.ik_preference)
        self.retry_plan = QtWidgets.QPushButton('重新规划')
        self.retry_plan.setToolTip('保留当前XYZ与姿态重新求解和检查，不会发送运动；较难目标会自动增加一次规划搜索时间。')
        self.retry_plan.clicked.connect(lambda _:self.start_plan())
        preference_row.addWidget(self.retry_plan)
        preference_row.addStretch()
        from manual_moveit_planner import MOTION_AMPLITUDES
        preference_row.addWidget(QtWidgets.QLabel('变化幅度'))
        self.amplitude = QtWidgets.QComboBox()
        for key,label,_ in MOTION_AMPLITUDES:self.amplitude.addItem(label,key)
        self.amplitude.setCurrentIndex(1)
        self.amplitude.setToolTip('随机位姿与自动测试共用：每轴采样变化上限；逆解选择其他构型时关节行程可能增大。')
        preference_row.addWidget(self.amplitude)
        self.random_pose = QtWidgets.QPushButton('生成随机合理位姿')
        self.random_pose.setToolTip('在当前关节姿态附近生成范围内的可达目标，检查目标碰撞后填入XYZ与姿态并规划；不会自动执行。')
        self.random_pose.clicked.connect(self.start_random_pose)
        preference_row.addWidget(self.random_pose)
        layout.addLayout(preference_row)
        row = QtWidgets.QHBoxLayout()
        self.capture = QtWidgets.QPushButton('采用实际末端位姿')
        self.play = QtWidgets.QPushButton('预览 / 停止（播放虚影）')
        self.execute = QtWidgets.QPushButton('执行规划路径')
        self.execute.setProperty('role', 'primary')
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
        grid.addWidget(QtWidgets.QLabel('预览倍速'), 3, 4)
        grid.addWidget(self.play_speed, 3, 5)
        presets = QtWidgets.QHBoxLayout()
        self.preset_buttons = []
        for label, pose in (('规划至 Home', panel.cfg['home_deg']), ('规划至工作 Rest', panel.cfg['rest_deg'])):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _, q=pose: self.start_joint_plan(q))
            self.preset_buttons.append(button)
            presets.addWidget(button)
        layout.addLayout(presets)
        test_row = QtWidgets.QHBoxLayout()
        test_row.addWidget(QtWidgets.QLabel('自动测试时长'))
        self.auto_duration = QtWidgets.QDoubleSpinBox()
        self.auto_duration.setRange(.01,1440.)
        self.auto_duration.setDecimals(2)
        self.auto_duration.setValue(10.)
        self.auto_duration.setSuffix(' 分钟')
        self.auto_duration.setToolTip('到期后不再生成随机目标，完成当前段后规划回Home；回程不计入测试时长。发生错误立即退出，不强行回程。')
        test_row.addWidget(self.auto_duration)
        self.auto_continuous = QtWidgets.QCheckBox('连续轨迹')
        self.auto_continuous.setToolTip('提前规划并检查连接曲线，在同一RUN内滚动补充；目标附近圆滑过渡，不要求逐目标停稳。补充不及时则沿已检查尾段停车。')
        self.auto_continuous.toggled.connect(lambda checked:self.auto_duration.setMaximum(57. if checked else 1440.))
        test_row.addWidget(self.auto_continuous)
        self.auto_start = QtWidgets.QPushButton('开始自动测试')
        self.auto_start.setToolTip('需已使能；一次确认后循环执行已检查的随机目标。勾选连续轨迹后圆滑衔接后续路径，不逐目标停车；未勾选时逐目标确认到位。')
        self.auto_start.clicked.connect(lambda _:self.auto_test.start(self.auto_duration.value()*60.))
        self.auto_finish = QtWidgets.QPushButton('结束并回 Home')
        self.auto_finish.setToolTip('停止补充新目标，执行完已检查的缓冲路径后回Home并退出。需要立即受控停车且保持使能时，使用“立即中断测试”。')
        self.auto_finish.clicked.connect(lambda _:self.auto_test.request_finish())
        self.auto_interrupt = QtWidgets.QPushButton('立即中断测试')
        self.auto_interrupt.setToolTip('立即请求受控停车并保持使能，取消后续规划，不自动回Home；未确认停车时停止释放。手动中断显示橙色。')
        self.auto_interrupt.clicked.connect(lambda _:self.auto_test.interrupt())
        test_row.addWidget(self.auto_start);test_row.addWidget(self.auto_finish);test_row.addWidget(self.auto_interrupt);test_row.addStretch()
        layout.addLayout(test_row)
        self.auto_status = QtWidgets.QLabel('自动测试未开始；随机目标范围以本次起点为中心。')
        self.auto_status.setWordWrap(True)
        layout.addWidget(self.auto_status)
        from manual_target_presets import TargetPresets
        self.target_presets = TargetPresets('cartesian_targets',
            ['X（mm）', 'Y（mm）', 'Z（mm）', 'Roll（°）', 'Pitch（°）', 'Yaw（°）'],
            self.inputs, self.apply_target_preset, self)
        from manual_panel_ui import fold
        fold('快捷末端目标 · 3 组位置与姿态', self.target_presets, layout)
        layout.addWidget(QtWidgets.QLabel('实机沿规划几何路径重新限速；同一直线采样点等价合并，弯曲路径保留路点。不保证动画原耗时。'))
        layout.addStretch()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(60)
        # 碰撞服务响应独立于60ms界面刷新；仍保持8个在途请求及原完整采样精度。
        self.collision_timer = QtCore.QTimer(self)
        self.collision_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.collision_timer.setInterval(2)
        self.collision_timer.timeout.connect(self.poll_collision_checks)
        panel.speed_input.valueChanged.connect(self.motion_parameters_changed)
        panel.acceleration_input.valueChanged.connect(self.motion_parameters_changed)

    def scene_ready(self):
        scene=getattr(self.panel,'obstacle_tab',None)
        return scene is None or scene.scene_ready and not scene.blocked

    def scene_revision(self):
        scene=getattr(self.panel,'obstacle_tab',None)
        return scene.scene_revision if scene is not None else None

    def scene_change_requested(self):
        # 障碍控制器随后请求Cancel并等待真实Hold；此处先清空所有旧场景工作。
        if self.auto_test.active:self.auto_test.abort('障碍场景已改变，停止旧场景测试',stop=False,result='interrupted')
        self.cancel_execution('障碍场景已改变，旧路径已作废',stop=False)
        if self.pending is not None:
            self.pending[1].cancel();self.pending=None
        self.invalidate()
        self.preview_timer.stop();self.jog_timer.stop()
        self.status.setText('等待停车与新碰撞场景确认；确认后请重新规划。')

    def motion_parameters_changed(self, *_):
        if not self.executing and not self.auto_test.active and (self.path or self.pending):
            self.invalidate()
            self.status.setText('速度或加速度已改变，请重新规划；未发送运动。')

    def poll_collision_checks(self):
        if getattr(self.panel, 'closing', False):
            self.invalidate()
            return
        if not (self.auto_test.active or self.pending or self.preset_fk or (self.path and (self.samples or self.checks))):
            self.collision_timer.stop()
            return
        if self.panel.node is not None:
            import rclpy
            if not rclpy.ok():
                self.invalidate()
                return
            deadline = time.perf_counter()+.002
            for _ in range(16):
                rclpy.spin_once(self.panel.node, timeout_sec=0)
                if time.perf_counter() >= deadline:
                    break
        self.tick()

    def preference_changed(self, *_):
        self.ik_preference.setToolTip(self.ik_preference.currentData(QtCore.Qt.ToolTipRole))
        self.invalidate()
        self.target_changed()

    def start_random_pose(self):
        if not self.scene_ready():return
        if self.executing or self.panel.busy or self.pending or not self.panel.fresh() or not hasattr(self.panel,'actual_q'):
            return
        if not all(client.service_is_ready() for client in (self.planner.fk,self.planner.plan,self.planner.validity)):
            self.status.setText('随机目标需要FK、规划及碰撞服务就绪，请稍后重试。')
            return
        self.invalidate()
        self.preview_timer.stop()
        self.random_attempts = 0
        try:
            self.request_random_candidate(tuple(self.panel.actual_q))
        except ValueError as error:
            self.set_target_failed(True)
            self.status.setText(str(error)+'；未发送运动，请检查当前位置后重试。')
            if hasattr(self.panel,'message'):self.panel.message(self.status.text())

    def request_random_candidate(self, start):
        self.random_attempts += 1
        if self.random_attempts > 8:
            raise ValueError('附近暂未找到无碰撞随机姿态，请换起点或再次生成')
        self.random_goal = sample_reasonable_joints(start,self.panel.limits,random,
                                                   self.ik_preference.currentData(),self.amplitude.currentData())
        self.begin('random-validity',self.planner.request_validity(self.random_goal),start)

    def apply_target_preset(self, values):
        if self.executing or self.panel.busy:
            self.status.setText('正在执行，请停止后设置快捷目标。')
            return
        self.invalidate()
        blockers = [QtCore.QSignalBlocker(box) for box in self.inputs]
        for box, value in zip(self.inputs, values):
            box.setValue(value)
        del blockers
        self.target_changed()

    def invalidate(self, *_):
        if self.auto_test.active:self.auto_test.abort('规划内容已改变，停止自动测试')
        self.collision_timer.stop()
        self.planning_deadline = None
        self.planning_started_at = None
        self.planning_slow_noted = False
        if self.executing:
            self.cancel_execution('规划内容变化，已停止后续路径。', stop=True)
        self.revision += 1
        self.path = None
        self.continuous_path = None
        self.noop_goal = None
        self.play_index = None
        self.play.setEnabled(False)
        self.execute.setEnabled(False)
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
        if not self.executing and not self.auto_test.active:
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

    def request_ready(self):
        p = self.panel
        return (self.scene_ready() and p.fresh() and p.control_state_allowed((2,4)) and p.authorized and
                bool(p.calibration) and p.resume_position_issue() is None and
                p.last_state['velocity_valid_mask'] == 63 and
                all(math.isfinite(v) for v in p.last_state['estimated_velocity_rad_s']))

    def ready(self):
        if not self.request_ready():return False
        state = self.panel.last_state
        if state.get('core_state') == 4 and state.get('hold_ready'):
            held = state.get('hold_position_rad',())
            return (state.get('hold_valid_mask') == 63 and len(held) == 6 and
                    all(math.isfinite(q) and abs(q-a)<=.003 for q,a in zip(held,self.panel.actual_q)))
        limit = .02 if self.auto_test.active else .01
        return all(abs(v)<=limit for v in state['estimated_velocity_rad_s'])

    def record(self, event, **fields):
        descriptions = {
            'IK_SELECTED': '已按所选偏好选择范围内的关节解和等效圈数',
            'PLAN_RECEIVED': '已收到规划路径，正在检查完整运动折线的碰撞',
            'PATH_VALIDATED': '规划和碰撞检查完成，可单独确认执行',
            'RANDOM_POSE_CREATED': '已生成可达且目标无碰撞的随机位姿，未发送运动',
            'SEGMENT_SUBMITTED': '已提交一个运动段，等待设备到位确认',
            'PATH_COMPLETED': '整条规划路径已由设备确认完成',
            'EXECUTION_ABORTED': '执行已取消，请查看具体原因',
            'AUTO_TEST_STARTED': '自动测试已开始，将在设定时长后回Home',
            'AUTO_TEST_RETURNING': '自动测试不再生成新目标，当前段完成后回Home',
            'AUTO_TEST_CYCLE_COMPLETED': '自动测试已确认一个随机目标到位',
            'AUTO_TEST_COMPLETED': '自动测试已确认回Home并结束',
            'AUTO_TEST_ADMISSION_WAIT': '尚未起步，等待原起步条件后重试',
            'AUTO_TEST_ABORTED': '自动测试因停止或错误退出，不强行回Home',
        }
        description = descriptions.get(event,event)
        with (self.panel.report / 'moveit-events.jsonl').open('a') as f:
            f.write(json.dumps(dict(event=event,description=description,host_ns=time.monotonic_ns(),**fields),ensure_ascii=False)+'\n')
        if event in descriptions and hasattr(self.panel,'message'):
            detail = f'；耗时 {fields["total_elapsed_s"]:.3f}秒' if 'total_elapsed_s' in fields else ''
            self.panel.message(description+detail)

    def current_identity(self):
        state = self.panel.last_state
        return (state['boot_epoch'], state['reference_epoch']) if state and self.panel.fresh() else None

    def begin(self, phase, future, start):
        if not self.scene_ready():
            future.cancel();self.status.setText('碰撞场景尚未确认，等待场景同步后重新规划。');return
        self.path_scene_revision=self.scene_revision()
        if self.planning_deadline is None:
            self.planning_started_at = time.monotonic()
            self.planning_deadline = self.planning_started_at+12.
        self.pending = (phase, future, self.planning_deadline, self.revision,
                        self.current_identity(), tuple(start))
        self.collision_timer.start()
        self.status.setText({'fk': '读取实际末端位姿…', 'ik': '按偏好求解目标关节角…',
                             'plan': '规划自碰撞有效路径…', 'random-validity': '检查随机目标是否碰撞…',
                             'random-fk': '生成随机目标的末端位姿…'}[phase])

    def capture_actual(self):
        if self.auto_test.active or not self.panel.fresh() or self.pending or not hasattr(self.panel, 'actual_q'):
            return
        self.invalidate()
        self.preview_timer.stop()
        start = tuple(self.panel.actual_q)
        self.begin('fk', self.planner.request_fk(start), start)

    def start_plan(self):
        if self.auto_test.active:return
        self.preview_timer.stop()
        if self.pending and self.pending[3] != self.revision:
            self.pending[1].cancel()
            self.pending = None
        if self.executing or self.pending:
            return
        if not self.panel.fresh() or not hasattr(self.panel, 'actual_q'):
            self.status.setText('等待新鲜实测起点后自动规划，尚未发送运动。')
            self.preview_timer.start(100)
            return
        if not self.planner.ik.service_is_ready() or not self.planner.plan.service_is_ready():
            self.status.setText('等待规划服务就绪，尚未发送运动。')
            self.preview_timer.start(100)
            return
        self.invalidate()
        start = tuple(self.panel.actual_q)
        values = [box.value() for box in self.inputs]
        self.begin('ik', self.planner.request_ik(start,values[:3],values[3:],self.ik_preference.currentData(),
                                               self.panel.cfg['home_deg'],self.panel.cfg['rest_deg']),start)

    def start_joint_plan(self, degrees):
        if self.auto_test.active or not self.panel.fresh() or self.pending or self.executing or not hasattr(self.panel, 'actual_q'):
            return
        self.invalidate()
        self.preview_timer.stop()
        start = tuple(self.panel.actual_q)
        self.panel.set_preview_pose(degrees)
        self.preset_fk = (self.planner.request_fk([math.radians(v) for v in degrees]), self.revision)
        self.begin('plan', self.planner.request_plan(start, [math.radians(v) for v in degrees]), start)

    def toggle_preview(self):
        if self.noop_goal is not None:
            if self.current_identity() == self.identity and hasattr(self.panel, 'actual_q') and all(
                    abs(a-b) <= .003 for a, b in zip(self.panel.actual_q, self.start)):
                self.panel.set_preview_pose([math.degrees(q) for q in self.noop_goal])
                self.status.setText('已在目标位置，无需运动；仅显示当前目标。')
            return
        self.play_index = None if self.play_index is not None else (0 if self.path else None)
        if self.play_index == 0:
            self.preview_started = time.monotonic()
            self.play_clock = self.preview_started
            self.play_elapsed = 0.
            self.status.setText('仅播放虚影；预览结束后需单独点击“执行规划路径”，不会自动运动。')

    def start_execution(self, confirmed=False):
        if not self.scene_ready() or getattr(self,'path_scene_revision',None)!=self.scene_revision():
            self.status.setText('碰撞场景尚未确认或路径场景版本已改变，请重新规划。');return
        if not self.path or self.checks or self.samples or self.panel.busy or not self.request_ready() or self.pending or self.executing:
            return
        revision, path, identity = self.revision, self.path, self.execution_identity()
        speed = self.panel.speed_input.value()
        acceleration = self.panel.acceleration_input.value()
        if self.continuous_enabled and (self.continuous_path is None or
                self.continuous_path.speed != speed or self.continuous_path.acceleration != acceleration):
            self.status.setText('连续轨迹尚未检查或速度/加速度已改变，请重新规划。')
            return
        execution_path = (path[0],path[-1]) if self.continuous_enabled else execution_polyline(path)
        detail = (f'连续执行 {self.continuous_path.duration:.2f}秒，中间路点不逐个停稳。' if self.continuous_enabled else
                  f'几何等价执行 {len(execution_path)-1} 段，各段会停车。')
        answer = QtWidgets.QMessageBox.Yes if confirmed and self.auto_test.active else self.panel.confirm_execution('执行 MoveIt 完整路径', f'规划含 {len(path)} 个采样点；{detail}\n速度上限 {speed:g}°/s、加速度 {acceleration:g}rad/s²。确认现场路径畅通后执行。')
        if answer != QtWidgets.QMessageBox.Yes:
            return
        if revision != self.revision or path != self.path or identity != self.execution_identity() or self.panel.busy or not self.request_ready():
            self.status.setText('确认期间状态变化，未执行。请重新规划。')
            return
        try:
            self.execution = SegmentedExecution(execution_path, identity)
            self.execution.check(self.panel.actual_q, identity, path[0])
            self.execution_speed, self.execution_acceleration = speed, acceleration
            self.admission_retry_deadline = None;self.retry_not_before = 0.
            self.started_motion = self.completed_marker = None
            self.panel.busy = True
            self.segment_deadline = time.monotonic()+2.
            self.record('EXECUTION_CONFIRMED', identity=identity, points=path,
                        execution_points=execution_path, original_times_s=self.times,
                        speed_deg_s=speed, acceleration_rad_s2=acceleration,
                        continuous=self.continuous_enabled,
                        segment_timeout_s=self.segment_timeout_s)
            self.advance_execution()
        except ValueError as error:
            self.cancel_execution(str(error), stop=False)

    def cancel_execution(self, reason, stop=False):
        if self.auto_test.active:
            self.auto_test.abort(reason,stop=stop)
            return
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
        if self.auto_test.native_event(event,parts):return
        if not self.executing:
            return
        identity = (int(parts['boot'], 16), int(parts['reference']), int(parts['session']))
        if identity != self.execution.identity:
            self.cancel_execution('设备事件身份改变，停止整条路径。', stop=True)
            return
        if event == 'MOVE_STARTED':
            self.started_motion = int(parts['motion'])
            if self.path_file is not None:
                self.path_file.unlink(missing_ok=True)
                self.path_file = None
        elif event == 'MOVE_COMPLETED' and self.started_motion is not None and int(parts['motion']) > self.started_motion:
            self.completed_marker = (int(parts['motion']), int(parts['state_seq']))
        elif event == 'MOVE_NOOP':
            self.completed_marker = (int(parts['motion']), int(parts['state_seq']))
        elif event == 'MOVE_NOT_READY' and self.started_motion is None and self.auto_test.active:
            # 仅Begin前的同身份暂未就绪可重试；实际运动拒绝与策略拒绝仍中断。
            now = time.monotonic()
            if self.admission_retry_deadline is None:self.admission_retry_deadline = now+2.
            if now >= self.admission_retry_deadline:
                self.cancel_execution('设备起步条件持续2秒未满足',stop=True);return
            self.execution.waiting = False
            self.retry_not_before = now+.1
            self.segment_deadline = self.admission_retry_deadline
            self.record('AUTO_TEST_ADMISSION_WAIT',reason='MOVE_NOT_READY',retry_after_s=.1)
        elif (event.startswith('MOVE_REJECTED') or event=='MOVE_NOT_READY') and self.started_motion is None:

            # 同身份的显式拒绝发生在Begin之前；取消队列，保留既有Hold。
            self.cancel_execution('Native 未提交本段：'+event+'；路径已取消，保持当前使能。', stop=False)
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
            self.segment_deadline = time.monotonic()+2.
            self.admission_retry_deadline = None;self.retry_not_before = 0.
            if self.execution.done:
                self.panel.busy = False
                self.record('PATH_COMPLETED', identity=self.execution.identity)
                self.status.setText('完整规划路径已由设备逐段确认完成，保持使能。')
                self.execution = None
                self.path = None
                self.auto_test.on_path_completed()
                return
        if not self.execution.waiting:
            if time.monotonic() < self.retry_not_before:return
            if self.admission_retry_deadline is not None and time.monotonic() >= self.admission_retry_deadline:
                self.cancel_execution('设备起步条件持续2秒未满足',stop=True);return
            if not self.ready():
                self.status.setText('执行请求已确认，等待原静止条件（最多2秒）；未提交本段运动。')
                return
            try:
                goal = self.execution.next_goal(self.panel.actual_q, self.execution_identity())
                command = (f'MOVE_SPEED {self.execution_speed:.1f} ACCEL {self.execution_acceleration:.2f} '+
                           ' '.join(f'{math.degrees(q):.8f}' for q in goal))
                if self.continuous_enabled:
                    self.path_file,digest = write_path(self.continuous_path,Path('/tmp'),self.execution.identity)
                    (self.panel.report/self.path_file.name).write_bytes(self.path_file.read_bytes())
                    command = (f'MOVE_PATH {self.execution_speed:.8g} ACCEL {self.execution_acceleration:.8g} '
                               f'{digest} {self.path_file}')
                    if self.auto_test.active and self.auto_test.chain_enabled and not self.auto_test.executing_home:
                        command = (f'MOVE_CHAIN {self.execution_speed:.8g} ACCEL {self.execution_acceleration:.8g} '
                                   f'BUDGET {self.auto_test.chain_budget:.8g} {digest} {self.path_file}')
                    self.record('CONTINUOUS_PATH_SUBMITTED',sha256=digest,
                                samples=len(self.continuous_path.points),duration_s=self.continuous_path.duration)
                if not self.panel.command(command):
                    raise ValueError('通信进程未接受运动命令')
                self.panel.busy = True
                self.segment_deadline = time.monotonic()+self.segment_timeout_s
                self.status.setText(f'执行完整路径：第 {self.execution.index} / {len(self.execution.points)-1} 段')
                self.record('SEGMENT_SUBMITTED', index=self.execution.index, goal=goal)
            except ValueError as error:
                self.cancel_execution(str(error), stop=True)

    def tick(self):
        if not self.scene_ready() and (self.path or self.pending):
            self.scene_change_requested()
        available = self.panel.fresh() and hasattr(self.panel, 'actual_q')
        state = self.panel.last_state
        observed_identity = (state['boot_epoch'], state['reference_epoch']) if state else None
        feedback_identity = self.current_identity()
        self.capture.setEnabled(available and not self.pending and not self.executing and self.planner.fk.service_is_ready())
        self.ik_preference.setEnabled(not self.executing)
        self.random_pose.setEnabled(available and not self.panel.busy and not self.pending and not self.executing and
                                   all(client.service_is_ready() for client in (self.planner.fk,self.planner.plan,self.planner.validity)))
        self.play.setEnabled(available and (bool(self.path) or self.noop_goal is not None) and not self.executing and not self.samples and not self.checks)
        self.execute.setEnabled(bool(self.path) and self.play_index is None and not self.panel.busy and not self.executing and not self.samples and not self.checks and self.request_ready())
        self.retry_plan.setEnabled(available and not self.pending and not self.executing and not self.auto_test.active)
        active = self.auto_test.active
        self.auto_start.setEnabled(not active and not self.pending and not self.panel.busy and self.request_ready())
        self.auto_finish.setEnabled(active and not self.auto_test.returning)
        self.auto_duration.setEnabled(not active)
        self.auto_continuous.setEnabled(not active and self.continuous_enabled)
        self.auto_interrupt.setEnabled(active)
        self.amplitude.setEnabled(not active)
        self.panel.speed_input.setEnabled(not active)
        self.panel.acceleration_input.setEnabled(not active)
        if active:
            for widget in self.inputs+self.jog_sliders+self.preset_buttons+[self.capture,self.play,self.execute,self.random_pose,self.ik_preference,self.retry_plan,self.step]:widget.setEnabled(False)
        for widget in self.inputs + self.jog_sliders + [self.step]:
            widget.setEnabled(not self.executing and not active)
        for widget in self.preset_buttons:
            widget.setEnabled(available and not active and not self.executing and not self.pending and self.planner.plan.service_is_ready())
        if self.preset_fk and self.preset_fk[0].done():
            future, revision = self.preset_fk
            self.preset_fk = None
            response = future.result()
            if revision == self.revision and response and response.error_code.val == 1 and response.pose_stamped:
                self.fill_pose(response.pose_stamped[0].pose)
        if (self.path or self.noop_goal is not None) and feedback_identity != self.identity:
            retry = feedback_identity is None and observed_identity == self.identity and not self.executing
            self.invalidate()
            if retry:
                self.preview_timer.start(100)
            self.status.setText('反馈失鲜或设备参考改变，路径已作废。')
        if (self.path or self.noop_goal is not None) and not self.executing and available and any(abs(a-b) > .003 for a, b in zip(self.panel.actual_q, self.start)):
            self.invalidate()
            self.status.setText('实测起点已改变，请重新规划；未发送运动。')
        if self.pending:
            phase, future, deadline, revision, identity, start = self.pending
            if time.monotonic() >= deadline or revision != self.revision or identity != feedback_identity:
                future.cancel()
                self.pending = None
                if (phase in ('ik', 'plan') and time.monotonic() < deadline and revision == self.revision
                        and feedback_identity is None and observed_identity == identity):
                    self.preview_timer.start(100)
                self.status.setText('规划服务响应超时、目标或设备参考已改变；目标输入已保留，可点击“重新规划”重试。未发送运动。')
                if hasattr(self.panel,'message'):self.panel.message(self.status.text())
            elif future.done():
                self.pending = None
                try:
                    response = future.result()
                    if response is None:
                        raise ValueError('规划服务未返回结果')
                    if phase == 'random-validity':
                        if not response.valid:
                            self.request_random_candidate(start)
                        else:
                            self.begin('random-fk',self.planner.request_fk(self.random_goal),start)
                    elif phase == 'random-fk':
                        if response.error_code.val != 1 or not response.pose_stamped:
                            raise ValueError(moveit_error(response.error_code.val))
                        self.fill_pose(response.pose_stamped[0].pose)
                        self.panel.set_preview_pose([math.degrees(q) for q in self.random_goal])
                        self.record('RANDOM_POSE_CREATED',goal=self.random_goal,attempts=self.random_attempts)
                        self.begin('plan',self.planner.request_plan(start,self.random_goal),start)
                    elif phase == 'fk':
                        if response.error_code.val != 1 or not response.pose_stamped:
                            raise ValueError(moveit_error(response.error_code.val))
                        pose = response.pose_stamped[0].pose
                        self.fill_pose(pose)
                        self.target_changed()
                        self.status.setText('实际末端位姿已填入，停更100ms自动规划；不会立即运动。')
                    elif phase == 'ik':
                        if response.error_code.val != 1:
                            raise ValueError(moveit_error(response.error_code.val))
                        js = response.solution.joint_state
                        by_name = dict(zip(js.name, js.position))
                        raw_goal = [by_name[name] for name in self.planner.names]
                        goal = getattr(future,'selected_goal',None)
                        if goal is None:goal = nearest_ik_goal(start,raw_goal,self.panel.limits)
                        preference = self.ik_preference.currentData()
                        matched = preference not in ('wrist_same','wrist_positive','wrist_negative') or ik_goal_cost(start,goal,preference)[0] == 0
                        self.record('IK_SELECTED',start=start,raw_goal=raw_goal,goal=goal,
                                    preference=preference,preference_matched=matched)
                        if not matched and hasattr(self.panel,'message'):
                            self.panel.message('所选腕部偏好暂无可用候选，已回退到可行逆解；请先检查预览再执行。')
                        self.panel.set_preview_pose([math.degrees(q) for q in goal])
                        self.begin('plan', self.planner.request_plan(start, goal), start)
                    else:
                        result = response.motion_plan_response
                        if result.error_code.val != 1:
                            raise ValueError(moveit_error(result.error_code.val))
                        path = validate_trajectory(result.trajectory.joint_trajectory,
                                                   self.planner.names, self.panel.limits)
                        self.identity, self.start = identity, start
                        if len(path) == 1:
                            if any(abs(a-b) > 1e-12 for a, b in zip(path[0], start)):
                                raise ValueError('单点规划不等于实测起点，拒绝执行')
                            self.noop_goal = path[0]
                            self.panel.set_preview_pose([math.degrees(q) for q in self.noop_goal])
                            self.set_target_failed(False)
                            self.status.setText('已在目标位置，无需运动。')
                            self.record('PLAN_NOOP', identity=identity, goal=self.noop_goal)
                        else:
                            self.path = path
                            self.panel.set_preview_pose([math.degrees(q) for q in self.path[-1]])
                            self.times = tuple(p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in result.trajectory.joint_trajectory.points)
                            self.continuous_path = (prepare_path(self.path,self.times,self.panel.speed_input.value(),
                                self.panel.acceleration_input.value(),velocities=trajectory_velocities(
                                    result.trajectory.joint_trajectory,self.planner.names,
                                    required=self.panel.cfg.get('backend')=='real')) if self.continuous_enabled else None)
                            actual_path = self.continuous_path.points if self.continuous_path else execution_polyline(self.path)
                            self.samples = list(collision_samples(actual_path))
                            self.checked_count = 0
                            self.check_started_at = time.monotonic()
                            self.check_deadline = self.planning_deadline
                            self.collision_timer.start()
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
                    if hasattr(self.panel,'message'):self.panel.message(text)
        if self.path and (self.samples or self.checks):
            try:
                if time.monotonic() >= self.check_deadline:
                    raise ValueError('碰撞服务长时间未响应，未授权执行；目标已保留，可点击重新规划')
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
                    self.collision_timer.stop()
                    endpoint = ' / '.join(f'J{i+1} {math.degrees(q):.2f}°' for i,q in enumerate(self.path[-1]))
                    self.status.setText(f'规划与检查完成：{len(self.path)} 路点 / {self.checked_count} 个样本。可直接点击“执行规划路径”；预览可选。\n终点：{endpoint}')
                    self.record('PATH_VALIDATED', samples=self.checked_count,
                                elapsed_s=time.monotonic()-self.check_started_at,
                                total_elapsed_s=time.monotonic()-self.planning_started_at)
            except (ValueError, RuntimeError) as error:
                self.invalidate()
                self.set_target_failed(True)
                self.status.setText(str(error))
                if hasattr(self.panel,'message'):self.panel.message(str(error))
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
            if (not self.panel.fresh() or not self.panel.authorized or
                    not self.panel.control_state_allowed((2, 3, 4, 5, 6)) or
                    self.execution_identity() != self.execution.identity):
                self.cancel_execution('设备反馈、授权或生命周期不再允许控制。', stop=True)
            elif time.monotonic() > self.segment_deadline:
                submitted = self.execution.waiting
                self.cancel_execution('运动段等待超时，停止路径。' if submitted else
                                      '原静止条件未在2秒内满足，已取消待执行请求；保持当前使能。', stop=submitted)
            elif self.completed_marker or not self.execution.waiting:
                self.advance_execution()
            if self.executing:
                self.panel.busy = True
        self.auto_test.tick()
        if (not self.auto_test.active and (self.pending or self.samples or self.checks) and
                self.planning_started_at is not None and time.monotonic()-self.planning_started_at > 1. and
                not self.planning_slow_noted):
            self.planning_slow_noted = True
            text = '该目标需要更多计算，继续规划与碰撞检查；目标已保留，检查完成后才能执行。'
            self.status.setText(text)
            if hasattr(self.panel,'message'):self.panel.message(text)
