"""完整规划折线执行顺序与身份；不运行 ROS/USB。"""
##
# @file test_manual_moveit_execution.py
# @brief 分段执行顺序、身份与折线采样几何的纯函数测试。
#
# 直接导入生产execution模块；没有ROS/USB。位置rad，identity顺序boot/reference/session，回执由confirm调用人工提供。collision_samples只保证折线采样覆盖，不实际调用碰撞检查服务。
import importlib.util
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/manual_moveit_execution.py'
spec = importlib.util.spec_from_file_location('manual_moveit_execution', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


##
# @brief 构造0→0.1→0.2rad六轴折线、绑定(7,2,81)的SegmentedExecution。
# @return 尚未派发任何段的SegmentedExecution。
def execution():
    return module.SegmentedExecution(((0.,)*6, (.1,)*6, (.2,)*6), (7, 2, 81))


##
# @brief 首段只取0.1rad，未confirm不得再next；逐段位置确认后才done。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_does_not_jump_to_endpoint_or_advance_without_receipt():
    e = execution()
    assert e.next_goal((0.,)*6, (7, 2, 81)) == (.1,)*6
    with pytest.raises(ValueError):
        e.next_goal((0.,)*6, (7, 2, 81))
    e.confirm((.1,)*6, (7, 2, 81))
    assert e.next_goal((.1,)*6, (7, 2, 81)) == (.2,)*6
    e.confirm((.2,)*6, (7, 2, 81))
    assert e.done


##
# @brief 分别替换boot/reference/session，next_goal必须拒绝，不能沿旧路径续走。
# @param identity 与基准(7,2,81)恰一字段不同的身份。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
@pytest.mark.parametrize('identity', [(6, 2, 81), (7, 1, 81), (7, 2, 82)])
def test_rejects_wrong_boot_reference_or_session(identity):
    with pytest.raises(ValueError):
        execution().next_goal((0.,)*6, identity)


##
# @brief 起点漂移0.01rad或未实际到0.1rad的confirm均拒绝。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_cannot_continue_from_drift_or_false_completion():
    with pytest.raises(ValueError):
        execution().next_goal((.01,)*6, (7, 2, 81))
    e = execution()
    e.next_goal((0.,)*6, (7, 2, 81))
    with pytest.raises(ValueError):
        e.confirm((0.,)*6, (7, 2, 81))


##
# @brief abort后next_goal始终ValueError，不能重用同对象恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_abort_permanently_disables_queue():
    e = execution()
    e.abort()
    with pytest.raises(ValueError):
        e.next_goal((0.,)*6, (7, 2, 81))


##
# @brief 四个共线样本压成起终点，不改变几何直线。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_collinear_sampling_can_execute_same_geometric_line_without_stopping_at_every_sample():
    points = ((0.,)*6, (.1,)*6, (.2,)*6, (.3,)*6)
    assert module.execution_polyline(points) == (points[0], points[-1])


##
# @brief 两段各百点直线压成起点/转角/终点，不以起终弦跨角。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_dense_bent_path_only_stops_at_real_corners():
    # 两条直线相接：全路径不共线，也不能逐采样点停车。
    first = tuple((i/100., 0., 0., 0., 0., 0.) for i in range(101))
    second = tuple((1., i/100., 0., 0., 0., 0.) for i in range(1, 101))
    points = first+second
    assert module.execution_polyline(points) == (points[0], first[-1], points[-1])


##
# @brief 保留0.2rad折返极值，去除同方向内部和重复静止样本。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_dense_backtracking_retains_turnaround_but_removes_internal_samples():
    points = tuple((v,)*6 for v in (0., .1, .2, .15, .1, .05, 0., 0., -.1))
    assert module.execution_polyline(points) == (points[0], points[2], points[-1])


##
# @brief 闭环仍保留两个转角及最后回起点，不因起终相等删掉全路径。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_closed_path_preserves_corners_and_end_identity():
    points = tuple((a,b,0.,0.,0.,0.) for a,b in
                   ((0.,0.),(.1,0.),(.2,0.),(.2,.1),(.2,.2),(.1,.1),(0.,0.)))
    assert module.execution_polyline(points) == (points[0],points[2],points[4],points[-1])


##
# @brief 重复停留位置不产生额外段，只保留共线起终点。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_stationary_samples_do_not_create_stop_go_commands():
    points = ((0.,)*6,)*4+((.1,)*6,)*3+((.2,)*6,)*2
    assert module.execution_polyline(points) == (points[0],points[-1])


##
# @brief 1e-7rad非共线偏移仍保留，执行简化不应变成未经检查平滑。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_small_real_detour_is_not_smoothed_out():
    points = ((0.,)*6, (.1,1e-7,0.,0.,0.,0.), (.2,0.,0.,0.,0.,0.))
    assert module.execution_polyline(points) == points


##
# @brief 参数化折返/非共线折线原样保留，防止删除真实绕行点。
# @param points 参数化六轴rad折返/非共线路径。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
@pytest.mark.parametrize('points', [
    ((0.,)*6, (.2,)*6, (.1,)*6, (.3,)*6),
    ((0.,)*6, (.1, .11, .1, .1, .1, .1), (.3,)*6),
])
def test_preserves_backtracking_and_noncollinear_path(points):
    assert module.execution_polyline(points) == points


##
# @brief 往返每条边按≤0.01rad最大轴步长采样且含0.03rad极点；只核采样，不判碰撞。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_collision_samples_cover_every_edge_without_chord_shortcut():
    samples = module.collision_samples(((0.,)*6, (.03,)*6, (0.,)*6), .01)
    assert samples[0] == (0.,)*6 and samples[-1] == (0.,)*6
    assert (.03,)*6 in samples
    assert all(max(abs(a-b) for a,b in zip(p,q)) <= .010000001 for p,q in zip(samples, samples[1:]))


##
# @brief 空、单点或NaN轨迹在SegmentedExecution构造时拒绝。
# @param points 空、单点或包含NaN的六轴rad轨迹。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
@pytest.mark.parametrize('points', [(), ((0.,)*6,), ((float('nan'),)*6, (0.,)*6)])
def test_rejects_empty_short_or_nonfinite_execution(points):
    with pytest.raises(ValueError):
        module.SegmentedExecution(points, (7, 2, 81))
