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


def test_ik_preferences_rank_valid_solutions_and_keep_minimax_default():
    start = [0., 0., 1.5, 0., .3, 0.]
    near = [0., 0., 1.5, .1, .4, .1]
    flipped = [0., 0., 1.5, .2, -.4, .2]
    assert module.ik_goal_cost(start, near, 'nearest') < module.ik_goal_cost(start, flipped, 'nearest')
    assert module.ik_goal_cost(start, near, 'wrist_same') < module.ik_goal_cost(start, flipped, 'wrist_same')
    assert module.ik_goal_cost(start, flipped, 'wrist_negative') < module.ik_goal_cost(start, near, 'wrist_negative')
    assert module.ik_goal_cost(start, near, 'wrist_positive') < module.ik_goal_cost(start, flipped, 'wrist_positive')


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


def test_preferred_ik_combines_real_service_responses_and_cancels_children():
    from concurrent.futures import Future
    planner = module.MoveItPlanner.__new__(module.MoveItPlanner)
    planner.names, planner.limits = NAMES, [(-179.45, 179.45)]*5+[(-719.45,719.45)]
    children = []
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


def test_keep_wrist_preference_is_neutral_at_singular_zero():
    start = [0.,0.,1.5,0.,1e-6,0.]
    goal = [0.,0.,1.5,0.,-.1,0.]
    assert module.ik_goal_cost(start,goal,'wrist_same')[0] == 0


def test_home_preference_can_choose_home_turn_instead_of_nearest_current_turn():
    import math
    from concurrent.futures import Future
    planner = module.MoveItPlanner.__new__(module.MoveItPlanner)
    planner.names,planner.limits = NAMES,[(-179.45,179.45)]*5+[(-719.45,719.45)]
    def request(*args):
        f=Future();f.set_result(NS(error_code=NS(val=1),solution=NS(joint_state=NS(
            name=NAMES,position=[0.,0.,math.pi/2,0.,0.,0.]))));return f
    planner._request_ik_single=request
    f=planner.request_ik([0.,0.,math.pi/2,0.,0.,math.radians(607.)], [100.,0.,200.], [0.,0.,0.],
                         'near_home',[0.,0.,90.,0.,0.,0.])
    assert f.result().error_code.val==1
    assert f.selected_goal[5]==pytest.approx(0.)


@pytest.mark.parametrize('code',[-1,-6,-2,99999])
def test_planning_fast_attempt_retries_with_more_time_and_retains_goal(code):
    from concurrent.futures import Future
    planner=module.MoveItPlanner.__new__(module.MoveItPlanner)
    requests=[]
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


def test_full_wrist_workspace_is_only_used_after_longer_local_search_failed():
    from concurrent.futures import Future
    planner=module.MoveItPlanner.__new__(module.MoveItPlanner)
    requests=[]
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


def test_robot_state_carries_fixed_floor_for_every_ik_plan_and_validity_request():
    planner=module.MoveItPlanner.__new__(module.MoveItPlanner);planner.names=NAMES
    state=planner.robot_state([0.,0.,1.57079632679,0.,0.,0.])
    assert len(state.attached_collision_objects)==1
    attached=state.attached_collision_objects[0]
    assert attached.link_name=='base_link' and attached.touch_links==['base_link','clearance_base_link']
    floor=attached.object
    assert floor.id=='manual_base_floor' and floor.header.frame_id=='base_link'
    assert floor.primitive_poses[0].position.z+floor.primitives[0].dimensions[2]/2==pytest.approx(.055)
