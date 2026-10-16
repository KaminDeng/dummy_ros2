"""完整规划折线执行顺序与身份；不运行 ROS/USB。"""
import importlib.util
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/manual_moveit_execution.py'
spec = importlib.util.spec_from_file_location('manual_moveit_execution', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def execution():
    return module.SegmentedExecution(((0.,)*6, (.1,)*6, (.2,)*6), (7, 2, 81))


def test_does_not_jump_to_endpoint_or_advance_without_receipt():
    e = execution()
    assert e.next_goal((0.,)*6, (7, 2, 81)) == (.1,)*6
    with pytest.raises(ValueError):
        e.next_goal((0.,)*6, (7, 2, 81))
    e.confirm((.1,)*6, (7, 2, 81))
    assert e.next_goal((.1,)*6, (7, 2, 81)) == (.2,)*6
    e.confirm((.2,)*6, (7, 2, 81))
    assert e.done


@pytest.mark.parametrize('identity', [(6, 2, 81), (7, 1, 81), (7, 2, 82)])
def test_rejects_wrong_boot_reference_or_session(identity):
    with pytest.raises(ValueError):
        execution().next_goal((0.,)*6, identity)


def test_cannot_continue_from_drift_or_false_completion():
    with pytest.raises(ValueError):
        execution().next_goal((.01,)*6, (7, 2, 81))
    e = execution()
    e.next_goal((0.,)*6, (7, 2, 81))
    with pytest.raises(ValueError):
        e.confirm((0.,)*6, (7, 2, 81))


def test_abort_permanently_disables_queue():
    e = execution()
    e.abort()
    with pytest.raises(ValueError):
        e.next_goal((0.,)*6, (7, 2, 81))


def test_collinear_sampling_can_execute_same_geometric_line_without_stopping_at_every_sample():
    points = ((0.,)*6, (.1,)*6, (.2,)*6, (.3,)*6)
    assert module.execution_polyline(points) == (points[0], points[-1])


@pytest.mark.parametrize('points', [
    ((0.,)*6, (.2,)*6, (.1,)*6, (.3,)*6),
    ((0.,)*6, (.1, .11, .1, .1, .1, .1), (.3,)*6),
])
def test_preserves_backtracking_and_noncollinear_path(points):
    assert module.execution_polyline(points) == points


def test_collision_samples_cover_every_edge_without_chord_shortcut():
    samples = module.collision_samples(((0.,)*6, (.03,)*6, (0.,)*6), .01)
    assert samples[0] == (0.,)*6 and samples[-1] == (0.,)*6
    assert (.03,)*6 in samples
    assert all(max(abs(a-b) for a,b in zip(p,q)) <= .010000001 for p,q in zip(samples, samples[1:]))


@pytest.mark.parametrize('points', [(), ((0.,)*6,), ((float('nan'),)*6, (0.,)*6)])
def test_rejects_empty_short_or_nonfinite_execution(points):
    with pytest.raises(ValueError):
        module.SegmentedExecution(points, (7, 2, 81))
