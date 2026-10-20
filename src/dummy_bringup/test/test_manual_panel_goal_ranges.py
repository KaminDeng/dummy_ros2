"""真实Qt范围控件的离线回归；只执行产品类，不启动ROS、Native或USB。"""
import ast
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtWidgets
import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'scripts/manual_joint_panel.py'


@pytest.fixture(scope='module')
def qt_application():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


@pytest.fixture
def goal_panel(tmp_path, monkeypatch, qt_application):
    # 使用真实产品构造与方法；替换的仅是ROS发布端和不应运行的连接/轮询边界。
    tree = ast.parse(SOURCE.read_text())
    definitions = [item for item in tree.body
                   if isinstance(item, (ast.ClassDef, ast.FunctionDef))
                   and item.name in ('JointPanel', 'valid_goal')]
    publisher = SimpleNamespace(publish=lambda *_: None)
    scope = dict(QtCore=QtCore, QtWidgets=QtWidgets, Path=Path, math=math,
                 json=json, time=time, String=object, JointState=object,
                 QoSProfile=lambda **_: None,
                 DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
                 xacro=SimpleNamespace(process_file=lambda _: SimpleNamespace(toxml=lambda: '<robot/>')))
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(SOURCE), 'exec'), scope)
    cls = scope['JointPanel']
    monkeypatch.setattr(cls, 'start_worker', lambda self: None)
    monkeypatch.setattr(cls, 'tick', lambda self: None)
    config = json.loads((ROOT / 'config/manual_joint_panel.json').read_text())
    panel = cls(config, tmp_path / 'capture', lambda *_: None,
                SimpleNamespace(create_publisher=lambda *_: publisher))
    panel.timer.stop()
    yield panel, scope
    panel.worker = QtCore.QProcess(panel)
    panel.close()


def assert_nominal_request_endpoints(panel, scope):
    for axis, (box, slider, limits) in enumerate(zip(panel.inputs, panel.sliders, [j['nominal_limits_deg'] for j in panel.cfg['joints']])):
        lower, upper = limits
        assert (box.minimum(), box.maximum()) == (lower, upper)
        assert (slider.minimum(), slider.maximum()) == (round(lower * 100), round(upper * 100))
        for value in (slider.minimum(), slider.maximum()):
            slider.setValue(value)
            target = list(panel.cfg['rest_deg'])
            target[axis] = box.value()
            assert box.value() == value / 100
            assert scope['valid_goal'](target, [j['nominal_limits_deg'] for j in panel.cfg['joints']])


@pytest.mark.parametrize('event', ['CALIBRATION_READY', 'CALIBRATION_RESUMED'])
def test_calibration_preview_resets_all_six_ordinary_endpoints(goal_panel, event):
    panel, scope = goal_panel
    # 请求可填名义止挡，实际发送仍须由start_move内收至原working范围。
    assert panel.inputs[1].minimum() == -73
    assert panel.inputs[2].maximum() == 180
    panel.worker = SimpleNamespace(readAllStandardOutput=lambda:
        f'home_rest_event={event} boot=7b reference=1 session=77 state_seq=10 motion=5 management=11\n'.encode())
    panel.stdout_ready()
    assert_nominal_request_endpoints(panel, scope)
    assert not panel.authorized and panel.pending_authorization is not None


def test_home_rest_previews_do_not_expand_ordinary_ranges_or_send_commands(goal_panel):
    panel, scope = goal_panel
    panel.restore_ordinary_goal_ranges()
    before = [(box.minimum(), box.maximum()) for box in panel.inputs]
    calls = []
    panel.command = lambda command: calls.append(command)
    for preview in (panel.cfg['home_deg'], panel.cfg['rest_deg'], panel.cfg['calibration_pose_deg']):
        panel.set_preview_pose(preview)
        assert [(box.minimum(), box.maximum()) for box in panel.inputs] == before
        assert scope['valid_goal']([box.value() for box in panel.inputs], [j['nominal_limits_deg'] for j in panel.cfg['joints']])
    assert calls == []
    assert_nominal_request_endpoints(panel, scope)


def test_trajectory_preview_updates_without_commands_or_per_frame_log(goal_panel):
    panel,_=goal_panel
    from unittest.mock import Mock
    panel.command=Mock();panel.message=Mock()
    panel.set_preview_pose([1.,2.,90.,4.,5.,6.],announce=False)
    assert [box.value() for box in panel.inputs]==[1.,2.,90.,4.,5.,6.]
    panel.command.assert_not_called();panel.message.assert_not_called()


def prepare_move(panel, monkeypatch, answer=QtWidgets.QMessageBox.Yes):
    panel.restore_ordinary_goal_ranges()
    panel.authorized, panel.calibration, panel.busy = True, (123, 1), False
    panel.control_state_allowed = lambda *_: True
    panel.fresh = lambda: True
    commands, prompts = [], []
    panel.command = lambda command: commands.append(command) or True
    monkeypatch.setattr(QtWidgets.QMessageBox, 'question',
                        lambda _parent, _title, text, *_: prompts.append(text) or answer)
    return commands, prompts


@pytest.mark.parametrize('answer', [QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No])
def test_nominal_stop_request_is_explicitly_projected_to_existing_working_goal(goal_panel, monkeypatch, answer):
    panel, scope = goal_panel
    commands, prompts = prepare_move(panel, monkeypatch, answer)
    requested = [0, -73, 180, 0, 0, 0]
    for box, value in zip(panel.inputs, requested):
        box.setValue(value)
    assert [box.value() for box in panel.inputs] == requested
    panel.start_move()
    effective = [0, -72.45, 179.45, 0, 0, 0]
    assert [box.value() for box in panel.inputs] == effective
    assert [bar.value() / 100 for bar in panel.sliders] == effective
    assert len(prompts) == 1
    for text in (prompts[0], panel.log.toPlainText()):
        assert 'J2 -73.00° → -72.45°' in text
        assert 'J3 180.00° → 179.45°' in text
    assert '未发送' in panel.log.toPlainText()
    if answer == QtWidgets.QMessageBox.Yes:
        assert len(commands) == 1 and panel.busy
        assert [float(x) for x in commands[0].split()[4:]] == effective
        assert scope['valid_goal'](effective, panel.limits)
    else:
        assert commands == [] and not panel.busy


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('side', [0, 1])
def test_every_nominal_endpoint_is_projected_but_outside_hard_range_is_rejected(goal_panel, monkeypatch, axis, side):
    panel, scope = goal_panel
    commands, prompts = prepare_move(panel, monkeypatch)
    requested = list(panel.cfg['rest_deg'])
    requested[axis] = panel.cfg['joints'][axis]['nominal_limits_deg'][side]
    for box, value in zip(panel.inputs, requested):box.setValue(value)
    panel.start_move()
    assert len(commands) == 1
    effective = [float(x) for x in commands[0].split()[4:]]
    assert effective[axis] == panel.limits[axis][side]
    assert scope['valid_goal'](effective, panel.limits)
    # 模拟损坏/外部注入的输入值，检查函数仍保名义硬界拒绝，不能夹进来当合法请求。
    panel.busy = False
    commands.clear();prompts.clear()
    outside = requested[axis] + (-.01 if side == 0 else .01)
    panel.inputs[axis].setRange(outside - 1, outside + 1)
    panel.inputs[axis].setValue(outside)
    panel.start_move()
    assert commands == [] and prompts == [] and not panel.busy


def test_projection_confirmation_does_not_bypass_changed_control_authority(goal_panel, monkeypatch):
    panel, _ = goal_panel
    commands, prompts = prepare_move(panel, monkeypatch)
    panel.inputs[1].setValue(-73)
    def decline_authority(_parent, _title, text, *_):
        prompts.append(text);panel.authorized = False
        return QtWidgets.QMessageBox.Yes
    monkeypatch.setattr(QtWidgets.QMessageBox, 'question', decline_authority)
    panel.start_move()
    assert len(prompts) == 1 and commands == [] and not panel.busy


def test_ordinary_move_button_accepts_request_but_preserves_invalid_and_stale_guards(goal_panel):
    panel, scope = goal_panel
    panel.authorized, panel.calibration, panel.busy = True, (123, 1), False
    panel.fresh = lambda: True
    state = dict(core_state=4, position_valid_mask=63, velocity_valid_mask=63,
                 position_rad=[0., 0., math.pi/2, 0., 0., 0.],
                 estimated_velocity_rad_s=[.011, 0., 0., 0., 0., 0.])
    panel.last_state = state
    tree = ast.parse(SOURCE.read_text())
    tick = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'tick')
    call = next(n for n in ast.walk(tick) if isinstance(n, ast.Call) and
                isinstance(n.func, ast.Attribute) and n.func.attr == 'setEnabled' and
                isinstance(n.func.value, ast.Attribute) and n.func.value.attr == 'move')
    condition = compile(ast.Expression(call.args[0]), str(SOURCE), 'eval')
    assert eval(condition, scope, dict(self=panel, state=state, fresh=True))
    state['velocity_valid_mask'] = 0
    assert not eval(condition, scope, dict(self=panel, state=state, fresh=True))
    state['velocity_valid_mask'] = 63
    assert not eval(condition, scope, dict(self=panel, state=state, fresh=False))


def test_hold_status_text_does_not_reformat_instantaneous_velocity(goal_panel):
    panel, scope = goal_panel
    panel.authorized, panel.calibration, panel.busy = True, (123, 1), False
    tree = ast.parse(SOURCE.read_text())
    tick = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'tick')
    start = next(i for i,n in enumerate(tick.body) if isinstance(n, ast.Assign) and
                 any(isinstance(t, ast.Name) and t.id == 'speed_note' for t in n.targets))
    fragment = compile(ast.Module(body=tick.body[start:start+2], type_ignores=[]), str(SOURCE), 'exec')
    messages = []
    for velocity in (.009, .011, -.012, .008):
        local = dict(self=panel, state=dict(core_state=4, estimated_velocity_rad_s=[velocity]*6))
        exec(fragment, scope, local)
        messages.append(local['speed_note'])
    assert len(set(messages)) == 1
    assert messages[0]
