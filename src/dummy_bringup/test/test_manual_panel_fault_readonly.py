"""直接执行产品面板方法；不构造QProcess、不打开ROS/USB，不修改控制协议。"""
import ast
import math
import json
from pathlib import Path
from types import SimpleNamespace
import time

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/manual_joint_panel.py'


class Button:
    def __init__(self): self.enabled = True; self.text = ''; self.number = 0
    def setEnabled(self, value): self.enabled = bool(value)
    def setText(self, value): self.text = value
    def value(self): return self.number
    def setValue(self, value): self.number = value


class JointState:
    def __init__(self): self.header = SimpleNamespace(stamp=None)


def logic():
    tree = ast.parse(SOURCE.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'JointPanel')
    cls.bases = []; cls.keywords = []
    qt = SimpleNamespace(QProcess=SimpleNamespace(Running=2, NotRunning=0))
    widgets = SimpleNamespace(QMessageBox=SimpleNamespace(Yes=1, No=0, question=lambda *args: 1))
    scope = dict(math=math, time=time, Path=Path, json=json, QtCore=qt, QtWidgets=widgets, JointState=JointState,
                 rclpy=SimpleNamespace(ok=lambda: True, spin_once=lambda *a, **k: None),
                 valid_goal=lambda q, limits: True)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), 'exec'), scope)
    return scope['JointPanel'], scope


def panel(tmp_path, core=9):
    cls, scope = logic(); p = object.__new__(cls)
    p.last_state = dict(boot_epoch=123, state_seq=10, reference_epoch=1, session_id=0,
                       core_state=core, position_valid_mask=63, velocity_valid_mask=63,
                       raw_position_valid_mask=63, can_rx_age_us=[0]*6,
                       estimated_velocity_rad_s=[0.0]*6,
                       position_rad=[0.0, math.radians(-75.12), math.pi/2, 0.0, 0.0, 0.0],
                       raw_motor_position_rev=[0.0]*6, schema_major=2, schema_minor=5, profile_id=201,
                       motion_epoch=5, management_epoch=11)
    p.last_state_ns = time.monotonic_ns(); p.state_boot = 123; p.state_sequence = 10
    p.retired_state_boots = set(); p.calibration = (123, 1); p.control_session = 77
    p.authorized = True; p.wait_home = True; p.pending_authorization = dict(
        boot_epoch=123, state_seq=10, reference_epoch=1, session_id=0, motion_epoch=5, management_epoch=11)
    p.readonly_fault = False; p.readonly_fault_identity = None
    p.busy = p.zero_busy = False; p.pending_home = True; p.pending_home_generation = 1
    p.worker_generation = 1; p.closing = False; p.restart_after_release = True
    p.stdout_buffer = ''; p.report = tmp_path; p.previous_raw = None; p.anchor = None
    p.display_initialized = True; p.calibration_event_ns = 0
    p.inputs = [Button() for _ in range(6)]; p.feedback_labels = [Button() for _ in range(6)]
    p.speed_input = Button(); p.speed_input.number = 17.2
    p.acceleration_input = Button(); p.acceleration_input.number = .4
    for name in ('zero', 'resume', 'move', 'recover', 'copy', 'stop', 'status'): setattr(p, name, Button())
    p.limits = [(-180, 180)]*6
    p.cfg = dict(joints=[dict(name=f'Joint{i+1}', direction=1, gear_ratio=50,
                             nominal_limits_deg=(-180, 180)) for i in range(6)],
                 rest_deg=[0, -60, 163.97, 0, 0, 0], inward_recovery_enabled=True)
    p.commands = []; p.messages = []; p.command = lambda text: p.commands.append(text) or True
    p.message = p.messages.append; p.read_state = lambda: None
    # AST逻辑fixture补齐构造函数的预览状态；渲染由独立MoveIt测试覆盖。
    p.preview_failed = None
    p.set_preview_failed = lambda value: setattr(p, 'preview_failed', value)
    p.show_actual_description = lambda value: setattr(p, 'description_visible', value)
    p.published = []; p.publisher = SimpleNamespace(publish=p.published.append)
    p.target_publisher = SimpleNamespace(publish=lambda msg: None)
    p.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)))
    p.worker = SimpleNamespace(state=lambda: 2, readAllStandardOutput=lambda: b'')
    return p, scope


def test_fault_state_disables_all_motion_but_publishes_actual_rviz(tmp_path):
    p, _ = panel(tmp_path); p.tick()
    assert not any(getattr(p, name).enabled for name in ('zero', 'resume', 'move', 'recover', 'copy'))
    assert p.stop.enabled and not p.authorized and not p.wait_home and p.pending_authorization is None
    assert p.calibration == (123, 1) and p.description_visible
    assert p.published[-1].position == p.last_state['position_rad']
    assert '故障禁止控制' in p.status.text and '参考仍存在' in p.status.text
    assert '已使能控制' not in p.status.text and '标定进行中' not in p.status.text
    assert p.commands == []


def test_fault_even_after_position_returns_in_range_does_not_enable_resume(tmp_path):
    p, _ = panel(tmp_path); p.last_state['position_rad'][1] = 0.0; p.tick()
    assert p.resume_position_issue() is None
    assert not p.resume.enabled and not p.recover.enabled
    p.start_zero(); p.start_resume(); p.start_move(); p.start_recovery()
    assert p.commands == []


def test_native_readonly_fault_event_clears_pending_intents_and_ignores_old_authorization(tmp_path):
    p, _ = panel(tmp_path, core=2)
    lines = ('home_rest_event=READONLY_FAULT boot=7b reference=1 state_seq=11 session=0 motion=6 management=12\n'
             'home_rest_event=WAIT_MANUAL_HOME boot=7b reference=1 state_seq=10 session=0\n'
             'home_rest_event=RESUME_NOT_READY boot=7b reference=1 state_seq=10 session=0\n'
             'home_rest_event=CALIBRATION_RESUMED boot=7b reference=1 state_seq=10 session=77 motion=5 management=11\n')
    p.worker.readAllStandardOutput = lambda: lines.encode(); p.stdout_ready()
    assert p.readonly_fault and not p.wait_home and not p.authorized and not p.busy and not p.zero_busy
    assert p.pending_authorization is None and not p.pending_home and not p.restart_after_release
    assert p.commands == []


def test_explicit_fault_clear_or_new_boot_does_not_reanimate_old_control_event(tmp_path):
    p, _ = panel(tmp_path); p.tick()
    state = dict(p.last_state, core_state=2, state_seq=11, session_id=0)
    assert p.accept_state(state, time.monotonic_ns()); p.reconcile_control_state()
    assert not p.readonly_fault and not p.authorized and not p.wait_home and p.pending_authorization is None
    state = dict(state, boot_epoch=124, reference_epoch=0, state_seq=1, position_valid_mask=0)
    assert p.accept_state(state, time.monotonic_ns()); p.reconcile_control_state()
    assert p.calibration is None and not p.authorized


def test_fault_arriving_during_resume_confirmation_prevents_send(tmp_path):
    p, scope = panel(tmp_path, core=2); p.pending_home = False; p.pending_authorization = None
    p.last_state['position_rad'][1] = 0.0
    def confirm(*_): p.last_state['core_state'] = 9; return 1
    scope['QtWidgets'].QMessageBox.question = confirm
    p.start_resume()
    assert p.commands == []


def test_first_tick_without_legacy_display_anchor_uses_device_reference(tmp_path):
    p, _ = panel(tmp_path)
    p.display_initialized = False
    p.tick()
    assert p.display_initialized and p.anchor is None
    assert p.published[-1].position == p.last_state['position_rad']
    assert not (tmp_path / 'display-only-Rest-anchor.json').exists()
    assert p.commands == []


def test_optional_display_anchor_rejects_missing_malformed_nonfinite_and_wrong_boot(tmp_path):
    p, _ = panel(tmp_path)
    valid = dict(boot='7b', motor_rev=[0.0]*6, home_rad=[0.0]*6)
    original_control = (p.authorized, p.control_session, p.calibration, p.pending_authorization)
    for candidate in (None, {}, dict(valid, motor_rev=[0.0]*5), dict(valid, home_rad=[float('nan')]*6), dict(valid, boot='7c')):
        p.cfg.update(initial_display_boot='7b', initial_display_anchor=candidate)
        assert p.optional_display_anchor(p.last_state) is None
    p.cfg.update(initial_display_boot='7b', initial_display_anchor=valid)
    assert p.optional_display_anchor(p.last_state) == valid
    assert (p.authorized, p.control_session, p.calibration, p.pending_authorization) == original_control
