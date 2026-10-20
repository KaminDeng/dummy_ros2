"""ROS 桌面面板的分区布局和统一样式；不持有设备控制逻辑。"""
import math
from PyQt5 import QtCore, QtWidgets

# 浅色工业控制台：主操作青绿，停止深红，输入与反馈使用独立表面。
THEME = '''
QWidget { color:#203441; font-family:"Noto Sans CJK SC","DejaVu Sans"; font-size:13px; }
QWidget#jointPanel { background:#eef2f5; }
QFrame#card, QGroupBox { background:#ffffff; border:1px solid #d5dfe6; border-radius:8px; }
QGroupBox { margin-top:12px; padding:14px 10px 8px; font-weight:600; }
QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 5px; }
QLabel#heading { font-size:23px; font-weight:700; color:#142e3c; }
QLabel#sectionHeading { font-weight:600; color:#203441; }
QLabel#muted { color:#526877; }
QLabel#feedback { color:#0c615b; background:#eaf5f2; border-radius:5px; padding:6px; font-family:"DejaVu Sans Mono"; }
QLabel#mode { padding:5px 10px; background:#eaf0f5; border-radius:5px; font-weight:600; }
QPushButton, QToolButton { background:#ffffff; border:1px solid #c6d2dd; border-radius:5px; padding:7px 12px; min-height:22px; }
QPushButton:hover, QToolButton:hover { background:#eaf3f4; border-color:#6c999c; }
QPushButton:pressed, QToolButton:pressed { background:#d4e9e6; }
QPushButton:focus, QToolButton:focus, QComboBox:focus, QAbstractSpinBox:focus { border:2px solid #167a91; }
QPushButton[role="primary"] { background:#0d7069; color:white; border-color:#0d7069; font-weight:600; }
QPushButton[role="primary"]:hover { background:#095c56; }
QPushButton[role="danger"] { background:#b42332; color:white; border-color:#b42332; font-weight:600; min-height:28px; }
QPushButton[role="danger"]:hover { background:#941c29; }
QPushButton:disabled, QToolButton:disabled { background:#f0f3f6; color:#7b8d99; border-color:#dce3e8; }
QAbstractSpinBox, QComboBox { background:white; border:1px solid #bdcbd6; border-radius:5px; padding:5px 7px; min-height:22px; selection-background-color:#0d7069; }
QAbstractSpinBox:disabled, QComboBox:disabled { background:#edf1f4; color:#7b8d99; }
QTabWidget::pane { background:white; border:1px solid #d5dfe6; border-radius:6px; }
QTabBar::tab { padding:10px 22px; background:#e4ebf1; border:0; margin-right:4px; color:#526877; }
QTabBar::tab:selected { background:white; color:#0c615b; border-top:3px solid #0d7069; font-weight:600; }
QSlider::groove:horizontal { height:6px; background:#dce5eb; border-radius:3px; }
QSlider::sub-page:horizontal { background:#6baca5; border-radius:3px; }
QSlider::handle:horizontal { width:16px; margin:-6px 0; background:#0d7069; border-radius:8px; }
QCheckBox { spacing:8px; padding:4px; }
QCheckBox::indicator { width:18px; height:18px; }
QCheckBox:checked { color:#9b4c0c; font-weight:600; }
QPlainTextEdit { background:#f5f8fa; border:1px solid #d5dfe6; border-radius:5px; padding:6px; font-family:"DejaVu Sans Mono"; font-size:12px; }
QTableWidget { background:white; alternate-background-color:#f4f7fa; gridline-color:#e0e7ed; border:1px solid #d5dfe6; }
QHeaderView::section { background:#edf3f6; padding:8px; border:0; border-bottom:1px solid #d5dfe6; font-weight:600; }
QWidget#tabSurface { background:white; }
QScrollArea { border:0; background:white; }
QToolTip { background:#203441; color:white; border:0; padding:6px; }
'''


def card():
    frame = QtWidgets.QFrame()
    frame.setObjectName('card')
    layout = QtWidgets.QVBoxLayout(frame)
    layout.setContentsMargins(14, 10, 14, 10)
    layout.setSpacing(8)
    return frame, layout


def fold(title, content, parent_layout):
    button = QtWidgets.QToolButton()
    button.setText(title)
    button.setCheckable(True)
    button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
    button.setArrowType(QtCore.Qt.RightArrow)
    button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
    content.setVisible(False)
    def toggle(checked):
        content.setVisible(checked)
        button.setArrowType(QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow)
    button.toggled.connect(toggle)
    parent_layout.addWidget(button)
    parent_layout.addWidget(content)
    return button


def build_panel_ui(p, config):
    p.setObjectName('jointPanel')
    p.setStyleSheet(THEME)
    p.setWindowTitle('六轴机械臂 · 控制台')
    p.resize(1160, 900)
    root = QtWidgets.QVBoxLayout(p)
    root.setContentsMargins(20, 16, 20, 16)
    root.setSpacing(12)
    header = QtWidgets.QHBoxLayout()
    title = QtWidgets.QLabel('六轴机械臂控制台'); title.setObjectName('heading')
    header.addWidget(title); header.addStretch()
    p.backend_notice = QtWidgets.QLabel(); p.backend_notice.setObjectName('mode')
    header.addWidget(p.backend_notice); root.addLayout(header)

    connection, body = card()
    row = QtWidgets.QHBoxLayout(); row.setSpacing(8)
    row.addWidget(QtWidgets.QLabel('连接目标'))
    p.backend_choice = QtWidgets.QComboBox()
    p.backend_choice.addItem('真机 · USB / Native', 'real')
    p.backend_choice.addItem('纯软件模拟器 · 运动学', 'simulator')
    p.backend_choice.setCurrentIndex(1 if p.backend == 'simulator' else 0)
    p.backend_choice.setMinimumWidth(240)
    row.addWidget(p.backend_choice)
    p.reconnect = QtWidgets.QPushButton('重新连接')
    p.reconnect.setToolTip('只读重连；不会自动使能或运动')
    p.clear_fault = QtWidgets.QPushButton('清除已处理故障')
    row.addWidget(p.reconnect); row.addWidget(p.clear_fault); row.addStretch()
    body.addLayout(row)
    p.status = QtWidgets.QLabel('正在连接；启动不会采零或运动')
    p.status.setWordWrap(True); body.addWidget(p.status); root.addWidget(connection)

    settings, body = card()
    heading = QtWidgets.QLabel('运动参数 · 各关节上限'); heading.setObjectName('sectionHeading')
    body.addWidget(heading)
    row = QtWidgets.QHBoxLayout(); row.setSpacing(10)
    row.addWidget(QtWidgets.QLabel('速度上限'))
    p.speed_input = QtWidgets.QDoubleSpinBox()
    p.speed_input.setRange(.1, config.get('motion_speed_max_deg_s', 17.2))
    p.speed_input.setDecimals(1); p.speed_input.setSuffix(' °/s')
    p.speed_input.setValue(config.get('motion_speed_default_deg_s', 17.2))
    p.speed_input.setAccessibleName('各关节速度上限（度每秒）')
    p.speed_input.setToolTip('所有运动页共用的关节速度上限；实际执行速度由实时反馈测量，不是末端线速度。')
    row.addWidget(p.speed_input)
    row.addWidget(QtWidgets.QLabel('加速度上限'))
    p.acceleration_input = QtWidgets.QDoubleSpinBox()
    p.acceleration_input.setRange(.05, config.get('motion_acceleration_max_rad_s2', 1.))
    p.acceleration_input.setDecimals(2); p.acceleration_input.setSingleStep(.05)
    p.acceleration_input.setSuffix(' rad/s²')
    p.acceleration_input.setValue(config.get('motion_acceleration_default_rad_s2', .4))
    p.acceleration_input.setAccessibleName('各关节加速度上限（弧度每二次方秒）')
    row.addWidget(p.acceleration_input); row.addStretch()
    p.skip_confirmations = QtWidgets.QCheckBox('跳过所有执行前确认')
    p.skip_confirmations.setToolTip('仅本次窗口有效；勾选后，点击标定、恢复、执行或清故障即直接提交。反馈、权限与限位检查仍然有效。')
    row.addWidget(p.skip_confirmations); body.addLayout(row); root.addWidget(settings)

    p.tabs = QtWidgets.QTabWidget(); root.addWidget(p.tabs, 1)
    joint = QtWidgets.QWidget(); joint.setObjectName('tabSurface'); joint_layout = QtWidgets.QVBoxLayout(joint)
    joint_layout.setContentsMargins(16, 14, 16, 14); joint_layout.setSpacing(12)
    grid = QtWidgets.QGridLayout(); grid.setHorizontalSpacing(14); grid.setVerticalSpacing(8)
    for col, text in enumerate(('关节', '实际角度', '目标调节', '目标角度', '工作范围（°）')):
        label = QtWidgets.QLabel(text); label.setObjectName('muted'); grid.addWidget(label, 0, col)
    p.feedback_labels, p.inputs, p.sliders = [], [], []
    pose = config.get('calibration_pose_deg') if p.manual_limits_mode() else None
    if not isinstance(pose, (list, tuple)) or len(pose) != 6 or any(type(q) not in (int, float) or not math.isfinite(q) for q in pose):
        pose = config.get('home_deg', [0, 0, 90, 0, 0, 0])
    for i, (joint_cfg, (lo, hi)) in enumerate(zip(config['joints'], p.limits)):
        label = QtWidgets.QLabel('—'); label.setObjectName('feedback'); label.setMinimumWidth(108)
        label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(round(min(lo, pose[i])*100), round(max(hi, pose[i])*100))
        spin = QtWidgets.QDoubleSpinBox(); spin.setDecimals(2)
        spin.setRange(min(lo, pose[i]), max(hi, pose[i])); spin.setSingleStep(.1)
        spin.setSuffix(' °'); spin.setKeyboardTracking(False); spin.setValue(pose[i])
        spin.setMinimumWidth(120); spin.setAccessibleName(f'{joint_cfg["name"]}目标角度')
        slider.setAccessibleName(f'{joint_cfg["name"]}目标调节')
        slider.setValue(round(spin.value()*100))
        slider.valueChanged.connect(lambda v, box=spin: box.setValue(v/100))
        spin.valueChanged.connect(lambda v, bar=slider: p.sync_slider(bar, v))
        for col, widget in enumerate((QtWidgets.QLabel(joint_cfg['name']), label, slider, spin, QtWidgets.QLabel(f'{lo:g} … {hi:g}'))):
            grid.addWidget(widget, i+1, col)
        p.feedback_labels.append(label); p.inputs.append(spin); p.sliders.append(slider)
    grid.setColumnStretch(2, 1); joint_layout.addLayout(grid)
    row = QtWidgets.QHBoxLayout()
    p.copy = QtWidgets.QPushButton('实际角度 → 目标')
    row.addWidget(p.copy)
    for text, q in (('预览 Home', config['home_deg']), ('预览工作 Rest', config['rest_deg'])):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(lambda _, values=q: p.set_preview_pose(values)); row.addWidget(button)
    row.addStretch()
    p.move = QtWidgets.QPushButton('执行六轴目标'); p.move.setProperty('role', 'primary')
    row.addWidget(p.move); joint_layout.addLayout(row)
    p.target_hint = QtWidgets.QLabel('修改输入只更新目标；点击“执行六轴目标”才运动。')
    p.target_hint.setWordWrap(True); p.target_hint.setObjectName('muted'); joint_layout.addWidget(p.target_hint)
    for box in p.inputs:
        box.valueChanged.connect(lambda _: p.target_hint.setText('目标已修改，尚未发送；点击“执行六轴目标”执行。'))
    from manual_target_presets import TargetPresets
    p.target_presets = TargetPresets('joint_targets', [f'J{i+1}（°）' for i in range(6)], p.inputs, p.set_preview_pose, p)
    fold('快捷目标 · 3 组可保存的六轴角度', p.target_presets, joint_layout)
    joint_layout.addStretch()
    def tab(widget, text):
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        scroll.setWidget(widget); p.tabs.addTab(scroll, text)
    tab(joint, '关节控制')
    if config.get('moveit_enabled', False):
        from manual_moveit_tab import MoveItTab
        p.moveit_tab = MoveItTab(p); tab(p.moveit_tab, '末端规划 · MoveIt')
    from manual_motor_parameters import MotorParameterTab
    p.motor_parameter_tab = MotorParameterTab(p); tab(p.motor_parameter_tab, '电机参数')
    p.tabs.currentChanged.connect(p.planning_tab_changed)

    p.log = QtWidgets.QPlainTextEdit(); p.log.setReadOnly(True)
    p.log.setMaximumBlockCount(200); p.log.setFixedHeight(110)
    p.log_toggle = fold('运行日志', p.log, root)
    actions, body = card()
    heading = QtWidgets.QLabel('设备控制 · 所有页面共用'); heading.setObjectName('sectionHeading')
    body.addWidget(heading)
    row = QtWidgets.QHBoxLayout(); row.setSpacing(8)
    p.resume = QtWidgets.QPushButton('使能 / 恢复零点')
    p.zero = QtWidgets.QPushButton('双限位标定 → Rest' if p.manual_limits_mode() else 'Home → Rest 标定')
    p.zero.setToolTip('确认人工姿态后重新建立参考，并低速退回工作 Rest')
    p.recover = QtWidgets.QPushButton('低速回工作区')
    for button in (p.resume, p.zero, p.recover): row.addWidget(button)
    row.addStretch()
    p.stop = QtWidgets.QPushButton('停止并失能'); p.stop.setProperty('role', 'danger')
    p.stop.setMinimumWidth(160); p.stop.setToolTip('停止运动并释放控制；同 boot 已完成零点保留')
    row.addWidget(p.stop); body.addLayout(row)
    note = QtWidgets.QLabel('标定与恢复为各页公共操作；模拟模式不连接硬件。真机重新标定前先确认人工双限位姿态。')
    note.setObjectName('muted'); note.setWordWrap(True); body.addWidget(note); root.addWidget(actions)
    p.backend_choice.currentIndexChanged.connect(p.request_backend_change)
    for button, callback in ((p.reconnect,p.request_reconnect),(p.clear_fault,p.request_clear_fault),
                             (p.zero,p.start_zero),(p.resume,p.start_resume),(p.recover,p.start_recovery),
                             (p.move,p.start_move),(p.copy,p.copy_actual)):
        button.clicked.connect(callback)
    p.stop.clicked.connect(lambda: p.release('STOP'))
    p.skip_confirmations.toggled.connect(lambda checked: p.message('执行确认已跳过；点击操作将直接提交。' if checked else '已恢复执行前确认。'))
    controls = [p.backend_choice, p.reconnect, p.clear_fault, p.speed_input, p.acceleration_input,
                p.skip_confirmations, *p.inputs, p.copy, p.move, p.resume, p.zero, p.recover, p.stop]
    for first, second in zip(controls, controls[1:]):
        QtWidgets.QWidget.setTabOrder(first, second)
