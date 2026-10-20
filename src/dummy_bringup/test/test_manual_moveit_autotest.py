"""限时自动测试的目标连续性与终止协议；不连接硬件。"""
import sys,math,random
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from manual_moveit_autotest import AutoTestSession, next_test_goal


def test_random_test_targets_are_bounded_different_and_prefer_forward_direction():
    start=(0.,0.,math.pi/2,0.,.3,0.)
    limits=[(-169.,169.),(-72.,89.),(36.,179.),(-179.,179.),(-99.,99.),(-719.,719.)]
    rng=random.Random(91)
    for _ in range(100):
        q=next_test_goal(start,limits,start,(.1,0.,0.,0.,0.,0.),rng)
        assert math.radians(3.) <= max(abs(a-b) for a,b in zip(start,q)) <= math.radians(15.)+1e-12
        assert all(lo<=math.degrees(v)<=hi for v,(lo,hi) in zip(q,limits))
        assert q[0]>start[0]
        assert q[4]>0.


def session():
    panel=NS(actual_q=[0.,0.,math.pi/2,0.,0.,0.],cfg={'home_deg':[0,0,90,0,0,0]},busy=True,
             speed_input=Mock(),acceleration_input=Mock(),release=Mock(),fresh=lambda:True,authorized=True,
             control_state_allowed=lambda states:True)
    tab=NS(panel=panel,executing=False,cancel_execution=Mock(),record=Mock(),
           execution_identity=lambda:(7,1,9),status=Mock(),collision_timer=Mock())
    s=AutoTestSession(tab);s.active=True;s.identity=(7,1,9);s.deadline=10.;s.started_at=0.;s.anchor=tuple(panel.actual_q)
    return s,tab


def test_expiry_does_not_interrupt_inflight_motion_and_requires_home_receipt():
    s,tab=session();tab.executing=True;s.request_finish('测试时长到期')
    assert s.active and s.returning
    tab.cancel_execution.assert_not_called()
    s.executing_home=True;tab.executing=False;s.on_path_completed()
    assert not s.active and s.completed==0
    assert s.result=='completed'


def test_error_aborts_and_never_attempts_automatic_home():
    s,tab=session();tab.executing=True;s.abort('碰撞检查失败')
    assert not s.active and s.result=='error'
    tab.panel.release.assert_called_once_with('STOP')
    assert not s.returning


def test_cancel_cancels_prefetch_and_has_no_future_home_motion():
    from concurrent.futures import Future
    s,tab=session();f=Future();s.future=f;s.abort('用户停止',stop=False)
    assert f.cancelled() and not s.active
    tab.panel.release.assert_not_called()


def test_home_completion_requires_actual_home_position():
    s,tab=session();s.executing_home=True;tab.panel.actual_q[0]=.1
    s.on_path_completed()
    assert not s.active and s.result=='error'
    tab.panel.release.assert_called_once_with('STOP')

@pytest.mark.parametrize('amplitude,maximum,minimum', [('small',15,3),('medium',40,8),('large',80,16)])
def test_amplitude_tiers_change_motion_range(amplitude, maximum, minimum):
    start=(0.,0.,math.pi/2,0.,.3,0.)
    limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    goals=[next_test_goal(start,limits,start,(0.,)*6,random.Random(i),amplitude=amplitude,
                          preference='wrist_positive') for i in range(30)]
    peaks=[math.degrees(max(abs(a-b) for a,b in zip(q,start))) for q in goals]
    assert min(peaks)>=minimum-1e-8 and max(peaks)<=maximum+1e-8
    assert max(peaks)>maximum*.9
    assert all(q[4]>=0 for q in goals)


def test_manual_interrupt_cancels_motion_and_is_distinct_from_failure():
    s,tab=session();tab.auto_status=Mock();tab.executing=True
    s.interrupt()
    assert not s.active and s.result=='interrupted'
    tab.panel.release.assert_called_once_with('STOP')
    assert '中断' in tab.auto_status.setText.call_args[0][0]
    assert '#b45309' in tab.auto_status.setStyleSheet.call_args[0][0]


def test_failure_and_pass_have_visible_distinct_result_colors():
    s,tab=session();tab.auto_status=Mock();s.abort('故障')
    assert '#b91c1c' in tab.auto_status.setStyleSheet.call_args[0][0]
    s,tab=session();tab.auto_status=Mock();s._complete()
    assert '#15803d' in tab.auto_status.setStyleSheet.call_args[0][0]


def test_sampling_can_return_from_an_ik_branch_outside_original_anchor_box():
    anchor=(0.,0.,math.pi/2,0.,.3,0.)
    start=(0.,0.,math.pi/2,math.radians(150),.3,math.radians(200))
    limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    q=next_test_goal(start,limits,anchor,(0.,)*6,random.Random(1),amplitude='small')
    assert all(lo<=math.degrees(v)<=hi for v,(lo,hi) in zip(q,limits))
    assert max(abs(a-b) for a,b in zip(q,start))<=math.radians(15)+1e-9


def test_manual_interrupt_uses_controlled_hold_when_transport_supports_it():
    s,tab=session();tab.panel.cancel_motion_hold=Mock();s.interrupt()
    tab.panel.cancel_motion_hold.assert_called_once_with()
    tab.panel.release.assert_not_called()
    assert s.result=='interrupted'


def test_long_random_walk_does_not_accumulate_wrist_turns_outside_session_envelope():
    anchor=(0.,0.,math.pi/2,0.,.3,0.);q=anchor;direction=(0.,)*6
    limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    rng=random.Random(3)
    for _ in range(300):
        goal=next_test_goal(q,limits,anchor,direction,rng,amplitude='medium')
        assert all(abs(math.degrees(v-a))<= (80. if i<3 else 120.)+1e-8
                   for i,(v,a) in enumerate(zip(goal,anchor)))
        direction=tuple(b-a for a,b in zip(q,goal));q=goal


def test_opposite_wrist_preference_can_approach_sign_without_exceeding_step():
    q=(0.,0.,math.pi/2,0.,math.radians(80.),0.)
    limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    goal=next_test_goal(q,limits,q,(0.,)*6,random.Random(6),amplitude='medium',preference='wrist_negative')
    assert 0.<goal[4]<q[4]
    assert max(abs(a-b) for a,b in zip(q,goal))<=math.radians(40)+1e-9


def test_candidate_collision_rejections_do_not_end_unattended_session():
    s,tab=session()
    tab.panel.limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    tab.planner=Mock()
    s.plan_start=s.anchor=tuple(tab.panel.actual_q)
    s.amplitude='medium';s.preference='nearest';s.attempts=0
    # 每次调用代表上一个候选被碰撞检查或IK筛掉；超过原来的8次仍必须继续。
    for _ in range(100):s._candidate()
    assert s.active and s.result is None and s.attempts==100
    assert tab.planner.request_validity.call_count==100
    assert s.stage=='validity'
    tab.panel.release.assert_not_called()
    tab.cancel_execution.assert_not_called()


def test_new_candidate_has_own_service_deadline_without_extending_test(monkeypatch):
    s,tab=session()
    tab.panel.limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    tab.planner=Mock();s.plan_start=s.anchor=tuple(tab.panel.actual_q)
    s.amplitude='medium';s.preference='nearest';s.attempts=0
    s.deadline=1000.
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:42.)
    s._candidate()
    assert s.plan_deadline==54. and s.deadline==1000.
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:53.)
    s._candidate()
    assert s.plan_deadline==65. and s.deadline==1000.


def test_chain_update_does_not_start_plan_from_old_tail_before_ack(tmp_path):
    from manual_moveit_continuous import PreparedPath
    s,t=session();s.chain_enabled=True;s.chain_live=True;s.chain_phase=400000
    s.chain_origin=0;s.chain_path=PreparedPath(tuple((i*.001,0.,0.,0.,0.,0.) for i in range(151)),60.,4.)
    joined=PreparedPath(s.chain_path.points+tuple((.15,i*.001,0.,0.,0.,0.) for i in range(1,100)),60.,4.)
    s.queued=dict(home=False,joined_path=joined,continuous_path=joined,join_cut=100)
    s.returning=False;t.started_motion=10;t.panel.report=tmp_path;t.panel.command=Mock(return_value=True)
    s._begin=Mock()
    s._chain_tick()
    assert s.chain_pending is not None
    s._begin.assert_not_called()


def test_chain_waits_for_native_begin_receipt_before_submitting_update():
    s,t=session();s.chain_live=True;t.started_motion=None
    s.queued={'home':False}
    s._chain_tick()
    assert s.queued=={'home':False} and s.chain_pending is None


def test_chain_preview_tracks_accepted_buffer_and_freezes_when_progress_is_stale(monkeypatch):
    from manual_moveit_continuous import PreparedPath
    s,t=session();s.chain_live=True;t.started_motion=10
    t.panel.set_preview_pose=Mock()
    s.chain_origin=100;s.chain_phase=2200000;s.chain_received_at=20.
    s.chain_path=PreparedPath(tuple((i*.001,0.,0.,0.,0.,0.) for i in range(100)),60.,4.)
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:20.)
    s.update_preview()
    first=t.panel.set_preview_pose.call_args
    assert first.args[0][0]==pytest.approx(math.degrees(.0175))
    assert first.kwargs=={'announce':False}
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:20.1)
    s.update_preview()
    assert t.panel.set_preview_pose.call_args.args[0][0]>first.args[0][0]
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:21.)
    s.update_preview();last=t.panel.set_preview_pose.call_args
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:25.)
    s.update_preview()
    assert t.panel.set_preview_pose.call_args==last
    s.active=False;t.panel.set_preview_pose.reset_mock();s.update_preview()
    t.panel.set_preview_pose.assert_not_called()


@pytest.mark.parametrize('home,target,reject',[(False,.2,True),(False,7.,False),(True,.2,False)])
def test_autotest_discards_gratuitous_multi_turn_wrist_route_before_execution(monkeypatch,home,target,reject):
    from concurrent.futures import Future
    s,t=session();s.stage='plan';s.plan_deadline=100.;s.plan_home=home;s.attempts=1
    t.panel.limits=[(-179.,179.)]*5+[(-719.,719.)]
    t.planner=NS(names=[f'Joint{i}' for i in range(1,7)],request_validity=Mock(return_value=Future()))
    t.continuous_enabled=False;s.speed=60.;s.acceleration=4.
    s.plan_start=(0.,0.,math.pi/2,0.,0.,0.)
    s.goal=(.1,0.,math.pi/2,0.,0.,target)
    points=(s.plan_start,(0.,0.,math.pi/2,0.,0.,7.),s.goal)
    trajectory=NS(joint_names=t.planner.names,points=[NS(positions=q,time_from_start=NS(sec=i,nanosec=0)) for i,q in enumerate(points)])
    s.future=Future();s.future.set_result(NS(motion_plan_response=NS(error_code=NS(val=1),trajectory=NS(joint_trajectory=trajectory))))
    s._candidate=Mock();monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:0.)
    s._poll_plan()
    if reject:
        s._candidate.assert_called_once_with()
        assert t.record.call_args.kwargs['reason']=='excess_wrist_detour'
    else:
        s._candidate.assert_not_called()
        assert s.stage=='checking'
    assert s.queued is None and s.active
