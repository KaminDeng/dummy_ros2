"""真实Qt范围控件的离线回归；只执行产品类，不启动ROS、Native或USB。"""
##
# @file test_manual_panel_goal_ranges.py
# @brief 真实Qt范围控件搭配AST产品方法的名义请求/工作范围回归。
#
# 真实QApplication/JointPanel控件；仅编译所选生产定义，start_worker/tick、ROS发布及xacro由替身隔离。输入目标单位deg，slider为百分之一度整数；实际State位置rad、速度rad/s。能证明UI与控制前重核，不证明Native/ROS/实机执行。
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


##
# @brief 模块级复用或创建offscreen QApplication，供控件fixture使用。
# @return yield QApplication；不启动ROS。
@pytest.fixture(scope='module')
def qt_application():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


##
# @brief 编译JointPanel和valid_goal并实际构造Qt面板，停timer且替换连接/轮询；yield后补QProcess对象再close。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @param qt_application 供真实Qt控件使用的QApplication fixture。
# @return yield (真实Qt面板, 编译scope)；退出关闭面板。
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


##
# @brief 逐轴核spinbox名义硬界、slider×100边界及端点目标valid_goal，未向设备发送。
# @param panel 面板对象，具体是Qt还是逻辑替身见本文件职责。
# @param scope 由产品AST编译得到的全局作用域，含函数及可变依赖替身。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 两类CALIBRATION事件仅恢复六轴名义请求控件范围并置pending_authorization，不能仅凭文本授权。
# @param goal_panel goal_panel fixture的真实Qt面板及替身scope。
# @param event CALIBRATION_READY或CALIBRATION_RESUMED人工stdout事件。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 预览Home/Rest/双限位姿势不改变普通输入硬界，不发命令。
# @param goal_panel goal_panel fixture的真实Qt面板及替身scope。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief announce=False更新六轴目标虚影输入，但不发MOVE也不逐帧记message。
# @param goal_panel goal_panel fixture的真实Qt面板及替身scope。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_trajectory_preview_updates_without_commands_or_per_frame_log(goal_panel):
    panel,_=goal_panel
    from unittest.mock import Mock
    panel.command=Mock();panel.message=Mock()
    panel.set_preview_pose([1.,2.,90.,4.,5.,6.],announce=False)
    assert [box.value() for box in panel.inputs]==[1.,2.,90.,4.,5.,6.]
    panel.command.assert_not_called();panel.message.assert_not_called()


##
# @brief 人工置可操作状态，记录command和确认框文本，提供Yes/No答案，隔离实际设备。
# @param panel 面板对象，具体是Qt还是逻辑替身见本文件职责。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @param answer 确认框替身返回Yes或No，默认Yes。
# @return (commands, prompts)记录列表，不代表真实发送队列。
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


##
# @brief 名义J2−73°/J3 180°投影到−72.45°/179.45°并说明未发送；只有Yes才发一次工作范围目标。
# @param goal_panel goal_panel fixture的真实Qt面板及替身scope。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @param answer Yes发送投影目标，No只更新预览不发送。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 六轴两端各投影至既有working限位；外部强塞超名义0.01°值仍直接拒绝，不夹紧成合法请求。
# @param goal_panel goal_panel fixture的真实Qt面板及替身scope。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @param axis 0..5对应J1..J6。
# @param side 0下端，1上端。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 确认框期间撤authorized后即便Yes也不发MOVE，防止确认前旧状态被沿用。
# @param goal_panel goal_panel fixture的真实Qt面板及替身scope。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_projection_confirmation_does_not_bypass_changed_control_authority(goal_panel, monkeypatch):
    panel, _ = goal_panel
    commands, prompts = prepare_move(panel, monkeypatch)
    panel.inputs[1].setValue(-73)
    ##
    # @par 内部回调 decline_authority
    # 记录弹框并撤panel.authorized，再返回Yes制造确认期间失权。
    # 参数 _parent：对话框父对象，替身忽略。
    # 参数 _title：确认标题，替身忽略。
    # 参数 text：确认正文，记录供断言。
    # 参数 _：位置参数，替身忽略。
    # 返回：QMessageBox.Yes，但此时授权已撤。
    def decline_authority(_parent, _title, text, *_):
        prompts.append(text);panel.authorized = False
        return QtWidgets.QMessageBox.Yes
    monkeypatch.setattr(QtWidgets.QMessageBox, 'question', decline_authority)
    panel.start_move()
    assert len(prompts) == 1 and commands == [] and not panel.busy


##
# @brief 只提取tick中move.setEnabled表达式求值：小速度不抖动禁用，但速度mask缺失或fresh=false仍阻断。
# @param goal_panel goal_panel fixture的真实Qt面板及替身scope。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 只编译speed_note两条语句，核±约0.01rad/s变化不反复重排Hold文本；不是完整tick或动态控制测试。
# @param goal_panel goal_panel fixture的真实Qt面板及替身scope。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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
