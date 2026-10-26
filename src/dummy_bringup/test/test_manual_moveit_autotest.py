"""限时自动测试的目标连续性与终止协议；不连接硬件。"""
##
# @file test_manual_moveit_autotest.py
# @brief 自动测试会话的采样包络、期限与连续路径回执的离线回归。
#
# AutoTestSession为真实产品对象，panel/tab/planner由NS、Mock和Future替代。q/路径为rad，limits与速度输入为deg/deg/s，Host monotonic期限为秒，chain_phase沿产品协议使用µs；没有发送设备命令。
import sys,math,random
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from manual_moveit_autotest import AutoTestSession, next_test_goal


##
# @brief 固定随机种子采100目标，核单步3..15°、硬范围及正向偏好，不当作碰撞可行性证明。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 构造active会话与Mock面板，人工身份(7,1,9)、deadline10s及Home=J3 90°，供状态机测试复用。
# @return (AutoTestSession, tab替身)；对象已人工置active，不代表已获设备许可。
def session():
    panel=NS(actual_q=[0.,0.,math.pi/2,0.,0.,0.],cfg={'home_deg':[0,0,90,0,0,0]},busy=True,
             speed_input=Mock(),acceleration_input=Mock(),release=Mock(),fresh=lambda:True,authorized=True,
             control_state_allowed=lambda states:True)
    tab=NS(panel=panel,executing=False,cancel_execution=Mock(),record=Mock(),
           execution_identity=lambda:(7,1,9),status=Mock(),collision_timer=Mock())
    s=AutoTestSession(tab);s.active=True;s.identity=(7,1,9);s.deadline=10.;s.started_at=0.;s.anchor=tuple(panel.actual_q)
    return s,tab


##
# @brief 时长到期只进入returning，不取消正在执行的段；收到标记为Home的完成且实际位于Home后才completed。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_expiry_does_not_interrupt_inflight_motion_and_requires_home_receipt():
    s,tab=session();tab.executing=True;s.request_finish('测试时长到期')
    assert s.active and s.returning
    tab.cancel_execution.assert_not_called()
    s.executing_home=True;tab.executing=False;s.on_path_completed()
    assert not s.active and s.completed==0
    assert s.result=='completed'


##
# @brief 碰撞类错误终止会话、请求STOP且不自动规划回Home。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_error_aborts_and_never_attempts_automatic_home():
    s,tab=session();tab.executing=True;s.abort('碰撞检查失败')
    assert not s.active and s.result=='error'
    tab.panel.release.assert_called_once_with('STOP')
    assert not s.returning


##
# @brief abort(stop=False)取消未完成Future并结束会话，不向panel.release发STOP或安排返程。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_cancel_cancels_prefetch_and_has_no_future_home_motion():
    from concurrent.futures import Future
    s,tab=session();f=Future();s.future=f;s.abort('用户停止',stop=False)
    assert f.cancelled() and not s.active
    tab.panel.release.assert_not_called()


##
# @brief J1偏离Home0.1rad时，即使回调声称Home完成也必须error并请求STOP。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_home_completion_requires_actual_home_position():
    s,tab=session();s.executing_home=True;tab.panel.actual_q[0]=.1
    s.on_path_completed()
    assert not s.active and s.result=='error'
    tab.panel.release.assert_called_once_with('STOP')

##
# @brief 按small/medium/large核单步峰值分别3..15°、8..40°、16..80°，并保留正腕偏好。
# @param amplitude 采样档位small/medium/large。
# @param maximum 该档单步最大角度deg。
# @param minimum 该档单步最小峰值deg。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 手动中断走interrupted并发STOP，文本及琥珀色区别于失败。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_manual_interrupt_cancels_motion_and_is_distinct_from_failure():
    s,tab=session();tab.auto_status=Mock();tab.executing=True
    s.interrupt()
    assert not s.active and s.result=='interrupted'
    tab.panel.release.assert_called_once_with('STOP')
    assert '中断' in tab.auto_status.setText.call_args[0][0]
    assert '#b45309' in tab.auto_status.setStyleSheet.call_args[0][0]


##
# @brief Mock状态标签核error红色、complete绿色；只检UI调用值。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_failure_and_pass_have_visible_distinct_result_colors():
    s,tab=session();tab.auto_status=Mock();s.abort('故障')
    assert '#b91c1c' in tab.auto_status.setStyleSheet.call_args[0][0]
    s,tab=session();tab.auto_status=Mock();s._complete()
    assert '#15803d' in tab.auto_status.setStyleSheet.call_args[0][0]


##
# @brief 起点J4/J6远离anchor时仍能生成合法且单步≤15°目标，不强迫瞬间回anchor包络。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_sampling_can_return_from_an_ik_branch_outside_original_anchor_box():
    anchor=(0.,0.,math.pi/2,0.,.3,0.)
    start=(0.,0.,math.pi/2,math.radians(150),.3,math.radians(200))
    limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    q=next_test_goal(start,limits,anchor,(0.,)*6,random.Random(1),amplitude='small')
    assert all(lo<=math.degrees(v)<=hi for v,(lo,hi) in zip(q,limits))
    assert max(abs(a-b) for a,b in zip(q,start))<=math.radians(15)+1e-9


##
# @brief 有cancel_motion_hold时中断优先调用受控Hold接口，不再调用release；只核方法选择。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_manual_interrupt_uses_controlled_hold_when_transport_supports_it():
    s,tab=session();tab.panel.cancel_motion_hold=Mock();s.interrupt()
    tab.panel.cancel_motion_hold.assert_called_once_with()
    tab.panel.release.assert_not_called()
    assert s.result=='interrupted'


##
# @brief 固定种子走300步，核相对anchor前三轴≤80°、后三轴≤120°，防止腕部累积绕圈。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_long_random_walk_does_not_accumulate_wrist_turns_outside_session_envelope():
    anchor=(0.,0.,math.pi/2,0.,.3,0.);q=anchor;direction=(0.,)*6
    limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    rng=random.Random(3)
    for _ in range(300):
        goal=next_test_goal(q,limits,anchor,direction,rng,amplitude='medium')
        assert all(abs(math.degrees(v-a))<= (80. if i<3 else 120.)+1e-8
                   for i,(v,a) in enumerate(zip(goal,anchor)))
        direction=tuple(b-a for a,b in zip(q,goal));q=goal


##
# @brief J5从正80°向负偏好靠近但不强跨零，单步仍≤40°。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_opposite_wrist_preference_can_approach_sign_without_exceeding_step():
    q=(0.,0.,math.pi/2,0.,math.radians(80.),0.)
    limits=[(-169,169),(-72,89),(36,179),(-179,179),(-99,99),(-719,719)]
    goal=next_test_goal(q,limits,q,(0.,)*6,random.Random(6),amplitude='medium',preference='wrist_negative')
    assert 0.<goal[4]<q[4]
    assert max(abs(a-b) for a,b in zip(q,goal))<=math.radians(40)+1e-9


##
# @brief 100次候选继续请求validity，不能因旧8次上限结束会话；Mock没有真实碰撞结果。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 注入42s/53s时钟使每个候选各获12s服务期限，但会话总deadline1000s不变。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 构造待拼接路径并调用chain tick，更新未确认时只保留chain_pending，不从旧尾部提前_begin。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief started_motion缺失时保留queued，不发送链更新，不建立chain_pending。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_chain_waits_for_native_begin_receipt_before_submitting_update():
    s,t=session();s.chain_live=True;t.started_motion=None
    s.queued={'home':False}
    s._chain_tick()
    assert s.queued=={'home':False} and s.chain_pending is None


##
# @brief 按接受缓冲和新鲜进度外推虚影，进度陈旧后冻结，inactive不改预览；虚影不是实际反馈。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 普通近腕目标若路径绕到7rad则重新采样，远目标/返Home路径保留；所有分支仍未入执行队列。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @param home 是否正在规划Home，返程不套普通近腕绕行拒绝。
# @param target J6目标rad，0.2或7。
# @param reject 期望是否丢弃路径并重采样。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


def test_ros_auto_start_uses_selected_session_not_calibration_baseline():
    s, t = session()
    s.active = False
    t.panel.backend = 'ros';t.panel.busy = False
    t.panel.ros_backend = NS(request_calibration_and_test=Mock())
    t.request_ready=lambda:True;t.pending=None;t.invalidate=Mock();t.preview_timer=Mock()
    t.panel.confirm_execution=lambda *args: __import__('PyQt5.QtWidgets',fromlist=['QMessageBox']).QMessageBox.Yes
    t.amplitude=NS(currentData=lambda:'small',currentText=lambda:'小')
    t.ik_preference=NS(currentData=lambda:'nearest',currentText=lambda:'最近')
    t.scene_revision=lambda:1;t.continuous_enabled=True
    t.auto_continuous=NS(isChecked=lambda:False)
    t.panel.speed_input.value=lambda:60.;t.panel.acceleration_input.value=lambda:4.
    s._begin=Mock()
    assert s.start(6.)
    t.panel.ros_backend.request_calibration_and_test.assert_not_called()
    assert s.active and s.requested_seconds == 6.
    s._begin.assert_called_once()


def test_ros_stream_start_selects_rolling_and_keeps_original_test_deadline(monkeypatch):
    s,t=session();s.active=False;t.panel.backend='ros';t.panel.busy=False
    t.request_ready=lambda:True;t.pending=None;t.invalidate=Mock();t.preview_timer=Mock()
    t.panel.confirm_execution=Mock(return_value=__import__('PyQt5.QtWidgets',fromlist=['QMessageBox']).QMessageBox.Yes)
    t.amplitude=NS(currentData=lambda:'large',currentText=lambda:'大')
    t.ik_preference=NS(currentData=lambda:'nearest',currentText=lambda:'最近')
    t.scene_revision=lambda:1;t.continuous_enabled=True;t.auto_continuous=NS(isChecked=lambda:True)
    t.panel.speed_input.value=lambda:63.;t.panel.acceleration_input.value=lambda:4.
    s._begin=Mock();monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:10.)
    assert s.start(180.) and s.ros_stream and s.chain_version==0
    assert s.deadline==190. and not s.chain_live
    assert '整轮预规划' not in t.panel.confirm_execution.call_args.args[1]


def test_ros_continuous_capacity_rejection_never_submits_or_uses_native():
    from manual_moveit_continuous import PreparedPath
    s,t=stream_session();q=s.anchor
    path=PreparedPath((q,)*31502,60.,4.)
    s.queued=dict(continuous_path=path,joined_path=path,home=False,join_cut=200)
    with pytest.raises(ValueError,match='630秒'):
        s._chain_tick()
    t.panel.ros_backend.extend_trajectory.assert_not_called();t.panel.command.assert_not_called()

@pytest.mark.parametrize('ros_stream',[True,False])
def test_ros_stream_valid_joint_candidate_goes_direct_to_plan_native_keeps_fk(ros_stream):
    from concurrent.futures import Future
    s,t=session();s.ros_stream=ros_stream;s.stage='validity';s.plan_home=False;s.plan_deadline=float('inf')
    s.plan_start=tuple(t.panel.actual_q);s.goal=tuple(q+.01 for q in s.plan_start)
    t.planner=NS(request_fk=Mock(return_value=Future()),request_plan=Mock(return_value=Future()))
    s.future=Future();s.future.set_result(NS(valid=True));s._poll_plan()
    if ros_stream:
        t.planner.request_plan.assert_called_once_with(s.plan_start,s.goal)
        t.planner.request_fk.assert_not_called();assert s.stage=='plan'
    else:
        t.planner.request_fk.assert_called_once_with(s.goal)
        t.planner.request_plan.assert_not_called();assert s.stage=='fk'


def test_ros_home_collision_never_replaces_home_with_random_candidate():
    from concurrent.futures import Future
    s,t=session();s.ros_stream=True;s.plan_home=True;s.stage='validity';s.plan_deadline=float('inf')
    s.future=Future();s.future.set_result(NS(valid=False));s._candidate=Mock()
    with pytest.raises(ValueError,match='Home.*碰撞'):
        s._poll_plan()
    s._candidate.assert_not_called()


def test_ros_preplan_progress_reports_prepared_targets_duration_and_deadline(monkeypatch):
    from manual_moveit_continuous import PreparedPath
    s,t=session();t.panel.backend='ros';s.ros_stream=True;s.chain_enabled=True;s.chain_live=False
    s.scene_version=1;t.scene_ready=lambda:True;t.scene_revision=lambda:1
    s.deadline=1000.;s.ros_preplan_deadline=100.;s.requested_seconds=180.;s.chain_markers=[100,200]
    s.chain_path=PreparedPath((tuple(t.panel.actual_q),)*501,60.,4.)
    s.stage='validity';s.attempts=2;t.auto_status=Mock()
    s._poll_plan=Mock();s._chain_tick=Mock();s.update_preview=Mock()
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:10.)
    s.tick();text=t.auto_status.setText.call_args.args[0]
    assert '准备起步前瞻' in text and '10.0/6' in text
    assert '90' in text and '寻找目标' not in text


def test_ros_stream_180_second_representative_planning_saves_queries_keeps_collision_samples(monkeypatch):
    from concurrent.futures import Future
    from collections import Counter
    from manual_moveit_continuous import PreparedPath
    monkeypatch.setattr('manual_moveit_autotest.pose_values',lambda pose:((0.,)*3,(0.,)*3))
    totals=[]
    for ros_stream in (False,True):
        count=Counter()
        for _ in range(90):  # 180秒、每段2秒的服务替身工作量，不冒充现场耗时。
            s,t=session();s.ros_stream=ros_stream;s.chain_enabled=False;s.plan_home=False
            s.plan_deadline=float('inf');s.attempts=1;s.speed=60.;s.acceleration=4.
            s.preference='nearest';s.amplitude='medium';t.panel.cfg['rest_deg']=[0,0,90,0,0,0]
            s.plan_start=tuple(t.panel.actual_q);s.goal=tuple(q+.01 for q in s.plan_start)
            t.panel.limits=[(-179.,179.)]*6;t.continuous_enabled=True
            prepared=PreparedPath((s.plan_start,)*100+(s.goal,),60.,4.)
            monkeypatch.setattr('manual_moveit_autotest.prepare_path',lambda *a,**k:prepared)
            monkeypatch.setattr('manual_moveit_autotest.collision_samples',lambda points:(points[0],points[50],points[-1]))
            def ready(kind,response):
                count[kind]+=1;future=Future();future.set_result(response);return future
            def ik(*a):
                future=ready('ik',NS(error_code=NS(val=1)));future.selected_goal=s.goal;return future
            trajectory=NS(joint_names=[f'Joint{i}' for i in range(6)],points=[NS(positions=q,time_from_start=NS(sec=i,nanosec=0)) for i,q in enumerate((s.plan_start,s.goal))])
            t.planner=NS(names=trajectory.joint_names,
                request_validity=lambda goal:ready('validity',NS(valid=True)),
                request_fk=lambda goal:ready('fk',NS(error_code=NS(val=1),pose_stamped=[NS(pose=NS())])),
                request_ik=ik,
                request_plan=lambda *a:ready('plan',NS(motion_plan_response=NS(error_code=NS(val=1),trajectory=NS(joint_trajectory=trajectory)))))
            s.stage='validity';s.future=t.planner.request_validity(s.goal)
            for _ in range(10):
                s._poll_plan()
                if s.queued is not None:break
            assert s.queued is not None and not s.samples and not s.checks
            assert s.queued['continuous_path'] is prepared
        totals.append(count)
    assert totals[0]==dict(validity=450,fk=90,ik=90,plan=90)
    assert totals[1]==dict(validity=360,plan=90)


def test_ros_preplan_deadline_aborts_without_submitting_or_extending_budget(monkeypatch):
    s,t=session();t.panel.backend='ros';s.ros_stream=True;s.chain_enabled=True;s.chain_live=False
    s.scene_version=1;t.scene_ready=lambda:True;t.scene_revision=lambda:1
    s.deadline=1000.;s.ros_preplan_deadline=20.;s._poll_plan=Mock();s._submit=Mock()
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:20.)
    s.tick()
    assert not s.active and s.result=='error' and s.ros_preplan_deadline==20.
    s._poll_plan.assert_not_called();s._submit.assert_not_called()
    t.panel.release.assert_called_once_with('STOP')


def stream_session():
    from manual_moveit_continuous import PreparedPath
    s,t=session();s.ros_stream=True;s.chain_enabled=True;s.chain_live=True;s.chain_version=0
    t.panel.backend='ros';t.started_motion=None;t.planner=NS(names=[f'Joint{i}' for i in range(6)])
    t.panel.ros_backend=NS(extend_trajectory=Mock(return_value=True))
    t.panel.command=Mock(side_effect=AssertionError('ROS不得使用Native续接'))
    t.panel.set_preview_pose=Mock();t.execution=NS(points=(s.anchor,s.anchor))
    s.ros_first_slot_ns=40_000_000;s.chain_phase=400_000;s.chain_received_at=10.
    s.chain_path=PreparedPath(tuple((max(0,i-1)*.001,)+s.anchor[1:] for i in range(301)),60.,4.)
    s.chain_last=dict(path=(s.anchor,s.chain_path.points[-1]),times=(0.,6.),pose=None,home=False,
                      start=s.anchor,goal=s.chain_path.points[-1])
    t.ros_execution_goal=s.chain_path.points[-1];s._begin=Mock()
    return s,t


def test_ros_stream_submits_tail_without_native_begin_and_waits_for_commit():
    from manual_moveit_continuous import PreparedPath
    s,t=stream_session();old=s.chain_path
    joined=PreparedPath(old.points+tuple((old.points[-1][0]+i*.001,)+s.anchor[1:] for i in range(1,101)),60.,4.)
    s.queued=dict(home=False,joined_path=joined,continuous_path=joined,join_cut=250,
                  start=old.points[-1],goal=joined.points[-1])
    s._chain_tick()
    assert s.chain_pending is not None and s.chain_path is old
    trajectory,version,through=t.panel.ros_backend.extend_trajectory.call_args.args
    assert version==0 and through==5_040_000_000
    assert trajectory.points[0].time_from_start.nanosec==40_000_000
    assert len(trajectory.points)==len(joined.points)
    s._begin.assert_not_called();t.panel.command.assert_not_called()
    s.ros_event('trajectory_extended',dict(version=1,phase_ns=540_000_000,trajectory_digest=[1]*32))
    assert s.chain_path is joined and s.chain_version==1 and s.chain_pending is None
    assert t.ros_execution_goal==joined.points[-1] and s.chain_phase==500_000
    s._begin.assert_called_once_with(joined.points[-1])


def test_ros_stream_feedback_moves_preview_and_never_changes_actual(monkeypatch):
    s,t=stream_session();actual=tuple(t.panel.actual_q)
    monkeypatch.setattr('manual_moveit_autotest.time.monotonic',lambda:10.)
    assert s.ros_event('progress',NS(desired=NS(time_from_start=NS(sec=1,nanosec=40_000_000))))
    s.update_preview();q=t.panel.set_preview_pose.call_args.args[0]
    assert q[0]==pytest.approx(math.degrees(.0565))
    assert s.chain_phase==1_000_000 and tuple(t.panel.actual_q)==actual
    s.ros_event('progress',NS(desired=NS(time_from_start=NS(sec=0,nanosec=540_000_000))))
    assert s.chain_phase==1_000_000


def test_ros_stream_rejected_tail_keeps_original_safe_buffer():
    s,t=stream_session();old=s.chain_path
    s.chain_pending=dict(item={},path=old,origin=0,marker=100,file=None,deadline=float('inf'))
    assert s.ros_event('trajectory_extension_rejected',dict(version=0,phase_ns=440_000_000,reason='LATE'))
    assert s.active and s.chain_path is old and s.chain_pending is None and s.chain_draining
    t.panel.release.assert_not_called();t.cancel_execution.assert_not_called()


def test_ros_stream_starts_after_short_lookahead_instead_of_whole_test():
    s,t=stream_session();s.chain_live=False;s.requested_seconds=180.;s.chain_markers=[100,200]
    s._submit=Mock();s._chain_tick()
    assert s.chain_live and s.chain_path.duration==6.
    s._submit.assert_called_once();s._begin.assert_called_once_with(s.chain_path.points[-1])


def test_ros_stream_preserves_queued_tail_until_feedback_passes_first_slot():
    from manual_moveit_continuous import PreparedPath
    s,t=stream_session();old=s.chain_path;s.chain_phase=0;s.chain_received_at=None
    joined=PreparedPath(old.points+tuple((old.points[-1][0]+i*.001,)+s.anchor[1:] for i in range(1,101)),60.,4.)
    queued=dict(home=False,joined_path=joined,continuous_path=joined,join_cut=250,
                start=old.points[-1],goal=joined.points[-1])
    s.queued=queued
    for phase_ns in (0,20_000_000,40_000_000):
        s.ros_event('progress',NS(desired=NS(time_from_start=NS(sec=0,nanosec=phase_ns))))
        s._chain_tick()
        assert s.queued is queued and s.chain_pending is None and s.chain_path is old
        t.panel.ros_backend.extend_trajectory.assert_not_called()
    s.ros_event('progress',NS(desired=NS(time_from_start=NS(sec=0,nanosec=60_000_000))))
    s._chain_tick();s._chain_tick()
    t.panel.ros_backend.extend_trajectory.assert_called_once()
    assert s.queued is None and s.chain_pending['path'] is joined and s.chain_path is old
