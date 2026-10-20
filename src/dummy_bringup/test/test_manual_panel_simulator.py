"""软件后端的六轴运动、限幅、停止与只读重连。"""
import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/manual_panel_simulator.py'


def simulator():
    assert SCRIPT.exists(), '缺少面板纯软件模拟后端'
    spec = importlib.util.spec_from_file_location('manual_panel_simulator', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.SixAxisSimulator(dict(
        rest_deg=[0, -60, 164, 0, 0, 0], home_deg=[0, 0, 90, 0, 0, 0],
        joints=[dict(limits_deg=[-170, 170]) for _ in range(6)]))


def test_readonly_start_and_resume_are_explicit():
    sim = simulator()
    assert sim.snapshot()['session_id'] == 0
    assert sim.command('MOVE_SPEED 10 ACCEL 1 1 -60 164 0 0 0') == 'MOVE_NOT_READY'
    assert sim.command('RESUME') == 'CALIBRATION_RESUMED'
    assert sim.snapshot()['session_id'] != 0


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


def test_invalid_goal_rejected_without_changing_motion():
    sim = simulator(); sim.command('RESUME'); before = sim.snapshot()['position_rad']
    for cmd in ('MOVE_SPEED 10 ACCEL 1 nan 0 0 0 0 0',
                'MOVE_SPEED 10 ACCEL 1 171 0 0 0 0 0',
                'MOVE_SPEED -1 ACCEL 1 0 0 0 0 0 0'):
        assert sim.command(cmd) == 'MOVE_REJECTED'
    sim.advance(1)
    assert sim.snapshot()['position_rad'] == before


def test_stop_and_reconnect_preserve_reference_but_never_resume_motion():
    sim = simulator(); sim.command('RESUME')
    sim.command('MOVE_SPEED 10 ACCEL 1 10 -50 154 20 -10 30'); sim.advance(.1)
    assert sim.command('STOP') == 'RELEASE_CONFIRMED'
    stopped = sim.snapshot(); sim.advance(10)
    assert sim.snapshot()['position_rad'] == stopped['position_rad']
    assert stopped['session_id'] == 0 and stopped['reference_epoch'] != 0
    assert sim.command('RESUME') == 'CALIBRATION_RESUMED'
