"""规划路径验证：不能把末点当作完整避碰路径执行。"""
##
# @file test_manual_moveit_planner.py
# @brief 规划请求形状、完整轨迹验证、腕部等价角及异步重试回归。
#
# MoveItPlanner多以__new__跳过ROS构造；自动fixture注入Graph发现图，call_async或底层请求方法用列表/Future替代，没有实际服务。关节位置rad、LIMITS deg，xyz目标mm/rpy deg；通过形状/数值不表示碰撞自由或设备许可。
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


##
# @brief 为跳过__init__的请求构造器补类属性node=Graph，满足生产拓扑门；不真实查询DDS。
# @param monkeypatch pytest替换fixture，作用域结束恢复属性/环境。
# @return None；执行上述夹具装配、手动状态推进或清理。
@pytest.fixture(autouse=True)
def unique_service_graph_for_request_builders(monkeypatch):
    from test_manual_moveit_service_topology import Graph
    # Existing request-shape tests bypass __init__; supply a real-shaped discovery fixture.
    monkeypatch.setattr(module.MoveItPlanner, 'node', Graph(), raising=False)


##
# @brief 构造仅含joint_names/positions/time_from_start的NS轨迹，默认六轴0/0.1/0.2rad和0/1/2s。
# @param names 关节名列表，空/None回退NAMES。
# @param positions 各点六轴rad列表，空/None回退默认三点。
# @param times 整数秒列表，空/None回退range；zip按较短列表构造。
# @return NS轨迹替身，无速度/加速度字段或真实ROS连接。
def trajectory(names=None, positions=None, times=None):
    positions = positions or [[0.] * 6, [.1] * 6, [.2] * 6]
    times = times or list(range(len(positions)))
    return NS(joint_names=names or NAMES, points=[
        NS(positions=q, time_from_start=NS(sec=t, nanosec=0))
        for q, t in zip(positions, times)])


##
# @brief 反序关节名响应须重排全部三点，保留中间0.6..0.1rad，不能只取终点。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_preserves_entire_path_and_reorders():
    path = module.validate_trajectory(trajectory(names=NAMES[::-1], positions=[
        [0.] * 6, [.1, .2, .3, .4, .5, .6], [.2] * 6]), NAMES, LIMITS)
    assert len(path) == 3
    assert path[1] == (.6, .5, .4, .3, .2, .1)


##
# @brief 面板60°/s、4rad/s²相对上限63°/s、4rad/s²，计划scaling为60/63和1而非固定10%。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_planning_uses_panel_dynamics_instead_of_fixed_ten_percent():
    from moveit_msgs.msg import RobotState
    captured=[]
    planner=object.__new__(module.MoveItPlanner)
    planner.names=NAMES;planner.limits=LIMITS
    planner.motion_parameters=lambda:(60.,4.,63.,4.)
    planner.robot_state=lambda _:RobotState()
    planner.plan=NS(call_async=lambda request:captured.append(request))
    planner._request_plan_single([0.]*6,[.1]*6)
    request=captured[0].motion_plan_request
    assert request.max_velocity_scaling_factor==pytest.approx(60./63.)
    assert request.max_acceleration_scaling_factor==1.


##
# @brief 快速J6局部窗含起终点外扩30°，例−146..−20°；显式extended且wrist_window=false才撤该约束。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_fast_plan_bounds_wrist_detours_but_extended_search_keeps_full_workspace():
    import math
    from moveit_msgs.msg import RobotState
    captured=[]
    planner=object.__new__(module.MoveItPlanner)
    planner.names=NAMES;planner.limits=LIMITS[:-1]+[(-719.,719.)]
    planner.robot_state=lambda _:RobotState()
    planner.plan=NS(call_async=lambda request:captured.append(request))
    start=[0.,0.,1.5,0.,0.,math.radians(-50.)]
    goal=[.1,0.,1.5,0.,0.,math.radians(-116.)]
    planner._request_plan_single(start,goal)
    constraints=captured[-1].motion_plan_request.path_constraints.joint_constraints
    assert len(constraints)==1 and constraints[0].joint_name=='Joint6'
    c=constraints[0]
    assert math.degrees(c.position-c.tolerance_below)==pytest.approx(-146.)
    assert math.degrees(c.position+c.tolerance_above)==pytest.approx(-20.)
    planner._request_plan_single(start,goal,extended=True,wrist_window=False)
    assert not captured[-1].motion_plan_request.path_constraints.joint_constraints


##
# @brief 710→700°窗口上界夹在719°硬界，仍覆盖700°终点。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_wrist_search_window_is_clipped_to_limits_and_keeps_both_endpoints():
    import math
    from moveit_msgs.msg import RobotState
    captured=[]
    planner=object.__new__(module.MoveItPlanner)
    planner.names=NAMES;planner.limits=LIMITS[:-1]+[(-719.,719.)]
    planner.robot_state=lambda _:RobotState()
    planner.plan=NS(call_async=lambda request:captured.append(request))
    planner._request_plan_single([0.]*5+[math.radians(710.)],[0.]*5+[math.radians(700.)])
    c=captured[0].motion_plan_request.path_constraints.joint_constraints[0]
    assert c.position+c.tolerance_above==pytest.approx(math.radians(719.))
    assert c.position-c.tolerance_below<=math.radians(700.)


##
# @brief 缺/重名关节、NaN/越界/维数错、时间倒序及空轨迹逐项ValueError。
# @param bad 参数化非法轨迹NS。
# @return None；以assert/unittest断言报告结果，失败抛异常。
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


##
# @brief 零姿态四元数明确为(0,0,0,1)，一般RPY归一化且inf输入拒绝。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_rpy_quaternion_is_normalized_and_concrete():
    assert module.rpy_quaternion(0., 0., 0.) == (0., 0., 0., 1.)
    q = module.rpy_quaternion(30., -20., 90.)
    assert sum(x*x for x in q) == pytest.approx(1.)
    with pytest.raises(ValueError):
        module.rpy_quaternion(float('inf'), 0., 0.)


##
# @brief 实际J6−251.95°、IK607.57°时选等价−112.43°；实际607°时保留原圈，不折叠实际反馈。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_ik_uses_nearest_equivalent_turn_from_actual_multiturn_feedback():
    import math
    limits = [(-179.45, 179.45)] * 5 + [(-719.45, 719.45)]
    start = [0.] * 5 + [math.radians(-251.94790444)]
    goal = [0.] * 5 + [math.radians(607.57155163)]
    selected = module.nearest_ik_goal(start, goal, limits)
    assert math.degrees(selected[5]) == pytest.approx(-112.42844837)
    assert math.remainder(selected[5]-goal[5], 2*math.pi) == pytest.approx(0.)
    start[5] = math.radians(607.)
    assert module.nearest_ik_goal(start, goal, limits)[5] == pytest.approx(goal[5])


##
# @brief 近719°起点与1°等价角选361°而非越719.45°；其它轴180°越工作界和NaN起点拒绝。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_ik_equivalent_angle_stays_in_actual_working_limits():
    import math
    limits = [(-179.45, 179.45)] * 5 + [(-719.45, 719.45)]
    start = [0.] * 5 + [math.radians(719.)]
    goal = [0.] * 5 + [math.radians(1.)]
    result = module.nearest_ik_goal(start, goal, limits)
    assert math.degrees(result[5]) == pytest.approx(361.)
    with pytest.raises(ValueError):
        module.nearest_ik_goal(start, [math.radians(180.)]+[0.]*5, limits)
    with pytest.raises(ValueError):
        module.nearest_ik_goal([float('nan')]+[0.]*5, goal, limits)


##
# @brief 对给定near/flipped两个合法解核nearest/同腕/正负腕排序，不声称枚举完整IK解空间。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_ik_preferences_rank_valid_solutions_and_keep_minimax_default():
    start = [0., 0., 1.5, 0., .3, 0.]
    near = [0., 0., 1.5, .1, .4, .1]
    flipped = [0., 0., 1.5, .2, -.4, .2]
    assert module.ik_goal_cost(start, near, 'nearest') < module.ik_goal_cost(start, flipped, 'nearest')
    assert module.ik_goal_cost(start, near, 'wrist_same') < module.ik_goal_cost(start, flipped, 'wrist_same')
    assert module.ik_goal_cost(start, flipped, 'wrist_negative') < module.ik_goal_cost(start, near, 'wrist_negative')
    assert module.ik_goal_cost(start, near, 'wrist_positive') < module.ik_goal_cost(start, flipped, 'wrist_positive')


##
# @brief 固定种子100个不重复目标均有限、在working范围且每轴距实际起点≤30°。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_random_goal_is_finite_local_and_inside_working_limits():
    import random
    import math
    limits = [(-169.45, 169.45), (-72.45, 89.45), (35.55, 179.45),
              (-179.45, 179.45), (-99.45, 99.45), (-719.45, 719.45)]
    start = [0., 0., math.pi/2, 0., 0., math.radians(607.)]
    rng = random.Random(27)
    goals = [module.sample_reasonable_joints(start, limits, rng) for _ in range(100)]
    assert len(set(goals)) == 100
    for goal in goals:
        assert all(math.isfinite(q) and lo <= math.degrees(q) <= hi for q,(lo,hi) in zip(goal,limits))
        assert all(abs(math.degrees(q-a)) <= 30.000001 for q,a in zip(goal,start))


##
# @brief 真实聚合Future消费合成子响应，负腕偏好选J5<0；取消聚合会取消全部子Future，非远端服务取消证明。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_preferred_ik_combines_real_service_responses_and_cancels_children():
    from concurrent.futures import Future
    planner = module.MoveItPlanner.__new__(module.MoveItPlanner)
    planner.names, planner.limits = NAMES, [(-179.45, 179.45)]*5+[(-719.45,719.45)]
    children = []
    ##
    # @par 内部回调 request
    # 构造并记录合成IK Future；由对应外层用例手动完成或立即置固定成功解，不发送IK服务。
    # 参数 args：调用位置参数，原样记录或由本替身忽略。
    # 返回：合成IK Future。
    def request(*args):
        f = Future();children.append(f);return f
    planner._request_ik_single = request
    combined = planner.request_ik([0.,0.,1.5,0.,.3,0.], [100.,0.,200.], [0.,0.,0.], 'wrist_negative')
    assert len(children) >= 2
    for index, child in enumerate(children):
        child.set_result(NS(error_code=NS(val=1),solution=NS(joint_state=NS(
            name=NAMES, position=[0.,0.,1.5,0.,.4 if index==0 else -.4,0.]))))
    assert combined.done() and combined.result().solution.joint_state.position[4] < 0.
    children.clear()
    combined = planner.request_ik([0.,0.,1.5,0.,.3,0.], [100.,0.,200.], [0.,0.,0.])
    combined.cancel()
    assert all(child.cancelled() for child in children)


##
# @brief 起点J5约零时同腕偏好罚项为0，避免奇异附近强迫符号。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_keep_wrist_preference_is_neutral_at_singular_zero():
    start = [0.,0.,1.5,0.,1e-6,0.]
    goal = [0.,0.,1.5,0.,-.1,0.]
    assert module.ik_goal_cost(start,goal,'wrist_same')[0] == 0


##
# @brief near_home偏好可从当前607°选Home J6零圈，依靠合成IK回执，不改变实际反馈。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_home_preference_can_choose_home_turn_instead_of_nearest_current_turn():
    import math
    from concurrent.futures import Future
    planner = module.MoveItPlanner.__new__(module.MoveItPlanner)
    planner.names,planner.limits = NAMES,[(-179.45,179.45)]*5+[(-719.45,719.45)]
    ##
    # @par 内部回调 request
    # 构造并记录合成IK Future；由对应外层用例手动完成或立即置固定成功解，不发送IK服务。
    # 参数 args：调用位置参数，原样记录或由本替身忽略。
    # 返回：合成IK Future。
    def request(*args):
        f=Future();f.set_result(NS(error_code=NS(val=1),solution=NS(joint_state=NS(
            name=NAMES,position=[0.,0.,math.pi/2,0.,0.,0.]))));return f
    planner._request_ik_single=request
    f=planner.request_ik([0.,0.,math.pi/2,0.,0.,math.radians(607.)], [100.,0.,200.], [0.,0.,0.],
                         'near_home',[0.,0.,90.,0.,0.,0.])
    assert f.result().error_code.val==1
    assert f.selected_goal[5]==pytest.approx(0.)


##
# @brief 四类失败码触发extended重试且目标不变，−2/99999启precise；第二响应成功才完成聚合。
# @param code −1/−6/−2/99999合成MoveIt错误码。
# @return None；以assert/unittest断言报告结果，失败抛异常。
@pytest.mark.parametrize('code',[-1,-6,-2,99999])
def test_planning_fast_attempt_retries_with_more_time_and_retains_goal(code):
    from concurrent.futures import Future
    planner=module.MoveItPlanner.__new__(module.MoveItPlanner)
    requests=[]
    ##
    # @par 内部回调 call
    # 记录start/goal与extended/wrist_window/precise选项并返回未完成Future，供外层按顺序注入失败/成功。
    # 参数 start：计划起点六轴rad。
    # 参数 goal：计划目标六轴rad。
    # 参数 extended：是否扩展搜索时间。
    # 参数 wrist_window：是否保留局部腕部路径窗。
    # 参数 precise：是否启精细规划配置。
    # 返回：合成服务Future/完成状态替身，见外层用例。
    def call(start,goal,extended=False,wrist_window=True,precise=False):
        f=Future();requests.append((start,goal,extended,f,wrist_window,precise));return f
    planner._request_plan_single=call
    start=[0.,0.,1.5,0.,0.,0.];goal=[.1,0.,1.5,0.,0.,0.]
    combined=planner.request_plan(start,goal)
    requests[0][3].set_result(NS(motion_plan_response=NS(error_code=NS(val=code))))
    assert len(requests)==2 and requests[1][2] is True and requests[1][1]==goal
    assert requests[1][5] == (code in (-2,99999))
    requests[1][3].set_result(NS(motion_plan_response=NS(error_code=NS(val=1))))
    assert combined.result().motion_plan_response.error_code.val==1


##
# @brief 先快速失败再长局部失败，第三次才wrist_window=false；取消聚合要取消第三Future。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_full_wrist_workspace_is_only_used_after_longer_local_search_failed():
    from concurrent.futures import Future
    planner=module.MoveItPlanner.__new__(module.MoveItPlanner)
    requests=[]
    ##
    # @par 内部回调 call
    # 记录start/goal与extended/wrist_window/precise选项并返回未完成Future，供外层按顺序注入失败/成功。
    # 参数 start：计划起点六轴rad。
    # 参数 goal：计划目标六轴rad。
    # 参数 extended：是否扩展搜索时间。
    # 参数 wrist_window：是否保留局部腕部路径窗。
    # 参数 precise：是否启精细规划配置。
    # 返回：合成服务Future/完成状态替身，见外层用例。
    def call(start,goal,extended=False,wrist_window=True,precise=False):
        f=Future();requests.append((extended,wrist_window,f));return f
    planner._request_plan_single=call
    combined=planner.request_plan([0.]*6,[.1]*6)
    requests[0][2].set_result(NS(motion_plan_response=NS(error_code=NS(val=-6))))
    assert requests[1][:2]==(True,True)
    requests[1][2].set_result(NS(motion_plan_response=NS(error_code=NS(val=99999))))
    assert requests[2][:2]==(True,False)
    combined.cancel()
    assert requests[2][2].cancelled()


##
# @brief 检查共享RobotState构造带base_link固定floor，顶面0.055m，touch_links仅base/clearance_base；未逐个真实发服务。
# @return None；以assert/unittest断言报告结果，失败抛异常。
def test_robot_state_carries_fixed_floor_for_every_ik_plan_and_validity_request():
    planner=module.MoveItPlanner.__new__(module.MoveItPlanner);planner.names=NAMES
    state=planner.robot_state([0.,0.,1.57079632679,0.,0.,0.])
    assert len(state.attached_collision_objects)==1
    attached=state.attached_collision_objects[0]
    assert attached.link_name=='base_link' and attached.touch_links==['base_link','clearance_base_link']
    floor=attached.object
    assert floor.id=='manual_base_floor' and floor.header.frame_id=='base_link'
    assert floor.primitive_poses[0].position.z+floor.primitives[0].dimensions[2]/2==pytest.approx(.055)
