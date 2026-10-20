"""软件后端的六轴运动、限幅、停止与只读重连。"""
##
# @file test_manual_panel_simulator.py
# @brief 六轴软件模型的明确恢复、限速/加速度与STOP回归。
#
# 只使用SixAxisSimulator内存时钟advance；命令速度deg/s、目标deg、accel rad/s²，snapshot位置rad。软件State/RELEASE_CONFIRMED不能作为实体设备反馈或停止证据。
import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/manual_panel_simulator.py'


##
# @brief 动态加载软件模型，初始Rest(0,−60,164,0,0,0)°、六轴±170°范围。
# @return 尚未授权的纯软件SixAxisSimulator。
def simulator():
    assert SCRIPT.exists(), '缺少面板纯软件模拟后端'
    spec = importlib.util.spec_from_file_location('manual_panel_simulator', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.SixAxisSimulator(dict(
        rest_deg=[0, -60, 164, 0, 0, 0], home_deg=[0, 0, 90, 0, 0, 0],
        joints=[dict(limits_deg=[-170, 170]) for _ in range(6)]))


##
# @brief session0启动拒MOVE，只有显式RESUME后建立非零软件session。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_readonly_start_and_resume_are_explicit():
    sim = simulator()
    assert sim.snapshot()['session_id'] == 0
    assert sim.command('MOVE_SPEED 10 ACCEL 1 1 -60 164 0 0 0') == 'MOVE_NOT_READY'
    assert sim.command('RESUME') == 'CALIBRATION_RESUMED'
    assert sim.snapshot()['session_id'] != 0


##
# @brief 按10ms推进1500步核六轴差分速度≤10°/s、加速度≤0.5rad/s²，最终目标rad及Hold4。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_six_axes_move_with_bounded_speed_and_acceleration():
    sim = simulator(); sim.command('RESUME')
    assert sim.command('MOVE_SPEED 10 ACCEL 0.5 10 -50 154 20 -10 30') == 'MOVE_STARTED'
    previous = sim.snapshot()['position_rad']; velocity = [0.]*6
    for _ in range(1500):
        sim.advance(.01)
        state = sim.snapshot(); q = state['position_rad']
        current = [(b-a)/.01 for a,b in zip(previous,q)]
        assert max(map(abs,current)) <= math.radians(10) + 1e-6
        assert max(abs(a-b)/.01 for a,b in zip(current,velocity)) <= .5 + 1e-5
        previous, velocity = q, current
    assert sim.snapshot()['position_rad'] == pytest.approx(list(map(math.radians,[10,-50,154,20,-10,30])))
    assert sim.snapshot()['core_state'] == 4


##
# @brief NaN/171°越界/负速度均MOVE_REJECTED，推进后位置不变。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_invalid_goal_rejected_without_changing_motion():
    sim = simulator(); sim.command('RESUME'); before = sim.snapshot()['position_rad']
    for cmd in ('MOVE_SPEED 10 ACCEL 1 nan 0 0 0 0 0',
                'MOVE_SPEED 10 ACCEL 1 171 0 0 0 0 0',
                'MOVE_SPEED -1 ACCEL 1 0 0 0 0 0 0'):
        assert sim.command(cmd) == 'MOVE_REJECTED'
    sim.advance(1)
    assert sim.snapshot()['position_rad'] == before


##
# @brief STOP清session但保留reference，后续advance10s位置冻结；必须再次显式RESUME。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_stop_and_reconnect_preserve_reference_but_never_resume_motion():
    sim = simulator(); sim.command('RESUME')
    sim.command('MOVE_SPEED 10 ACCEL 1 10 -50 154 20 -10 30'); sim.advance(.1)
    assert sim.command('STOP') == 'RELEASE_CONFIRMED'
    stopped = sim.snapshot(); sim.advance(10)
    assert sim.snapshot()['position_rad'] == stopped['position_rad']
    assert stopped['session_id'] == 0 and stopped['reference_epoch'] != 0
    assert sim.command('RESUME') == 'CALIBRATION_RESUMED'
