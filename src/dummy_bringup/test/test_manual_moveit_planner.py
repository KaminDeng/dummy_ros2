"""规划路径验证：不能把末点当作完整避碰路径执行。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/manual_moveit_planner.py'
spec = importlib.util.spec_from_file_location('manual_moveit_planner', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

NAMES = [f'Joint{i}' for i in range(1, 7)]
LIMITS = [(-180, 180)] * 6


def trajectory(names=None, positions=None, times=None):
    positions = positions or [[0.] * 6, [.1] * 6, [.2] * 6]
    times = times or list(range(len(positions)))
    return NS(joint_names=names or NAMES, points=[
        NS(positions=q, time_from_start=NS(sec=t, nanosec=0))
        for q, t in zip(positions, times)])


def test_preserves_entire_path_and_reorders():
    path = module.validate_trajectory(trajectory(names=NAMES[::-1], positions=[
        [0.] * 6, [.1, .2, .3, .4, .5, .6], [.2] * 6]), NAMES, LIMITS)
    assert len(path) == 3
    assert path[1] == (.6, .5, .4, .3, .2, .1)


@pytest.mark.parametrize('bad', [
    trajectory(names=NAMES[:-1]),
    trajectory(names=[NAMES[0]] * 6),
    trajectory(positions=[[0.] * 6, [float('nan')] * 6]),
    trajectory(positions=[[0.] * 6, [4.] * 6]),
    trajectory(positions=[[0.] * 6, [0.] * 5]),
    trajectory(times=[0, 2, 1]),
    NS(joint_names=NAMES, points=[]),
])
def test_rejects_invalid_path(bad):
    with pytest.raises(ValueError):
        module.validate_trajectory(bad, NAMES, LIMITS)


def test_rpy_quaternion_is_normalized_and_concrete():
    assert module.rpy_quaternion(0., 0., 0.) == (0., 0., 0., 1.)
    q = module.rpy_quaternion(30., -20., 90.)
    assert sum(x*x for x in q) == pytest.approx(1.)
    with pytest.raises(ValueError):
        module.rpy_quaternion(float('inf'), 0., 0.)
