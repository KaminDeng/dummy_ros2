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


def test_dense_bent_path_only_stops_at_real_corners():
    # 两条直线相接：全路径不共线，也不能逐采样点停车。
    first = tuple((i/100., 0., 0., 0., 0., 0.) for i in range(101))
    second = tuple((1., i/100., 0., 0., 0., 0.) for i in range(1, 101))
    points = first+second
    assert module.execution_polyline(points) == (points[0], first[-1], points[-1])


def test_dense_backtracking_retains_turnaround_but_removes_internal_samples():
    points = tuple((v,)*6 for v in (0., .1, .2, .15, .1, .05, 0., 0., -.1))
    assert module.execution_polyline(points) == (points[0], points[2], points[-1])


def test_closed_path_preserves_corners_and_end_identity():
    points = tuple((a,b,0.,0.,0.,0.) for a,b in
                   ((0.,0.),(.1,0.),(.2,0.),(.2,.1),(.2,.2),(.1,.1),(0.,0.)))
    assert module.execution_polyline(points) == (points[0],points[2],points[4],points[-1])


def test_stationary_samples_do_not_create_stop_go_commands():
    points = ((0.,)*6,)*4+((.1,)*6,)*3+((.2,)*6,)*2
    assert module.execution_polyline(points) == (points[0],points[-1])


def test_small_real_detour_is_not_smoothed_out():
    points = ((0.,)*6, (.1,1e-7,0.,0.,0.,0.), (.2,0.,0.,0.,0.,0.))
    assert module.execution_polyline(points) == points


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
