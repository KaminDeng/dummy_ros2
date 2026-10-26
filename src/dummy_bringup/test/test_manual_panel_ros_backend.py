"""ROS 薄客户端仅转发请求与权威终态，不加载 Native。"""
import sys
import pytest
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

def test_release_queued_is_progress_and_only_cpp_confirms_release():
    from manual_panel_ros_backend import RosBackend
    class Transport:
        status_max_age_ms = 731  # 假C++响应，不是产品预算。
        def request(self, service, callback):
            self.callback = callback
        def close(self): pass
    transport = Transport()
    events = []
    backend = RosBackend(transport, lambda kind, value: events.append((kind, value)))
    backend.connect()
    backend.release()
    transport.callback(NS(success=False, message='RELEASE_QUEUED'))
    assert events[-1] == ('progress', 'RELEASE_QUEUED')
    assert backend.released is False
    backend.poll()
    transport.callback(NS(success=True, message='RELEASED_IDLE'))
    assert backend.released is True
    assert events[-1][0] == 'released'

def test_trigger_acceptance_does_not_complete_auto_task():
    from manual_panel_ros_backend import RosBackend
    class Transport:
        status_max_age_ms = 731  # 假C++响应，不是产品预算。
        def request(self, service, callback): self.callback = callback
    transport = Transport(); events = []
    backend = RosBackend(transport, lambda *event: events.append(event))
    backend.connect()
    assert events == [('connected', 'ReadOnly')]
    backend.request_calibration_and_test()
    transport.callback(NS(success=True, message='queued'))
    assert events[-1] == ('accepted', 'queued')
    assert not any(event[0] == 'completed' for event in events)

def test_release_waits_for_inflight_poll_then_submits_release():
    from manual_panel_ros_backend import RosBackend
    class Transport:
        status_max_age_ms = 731  # 假C++响应，不是产品预算。
        def __init__(self): self.calls = []
        def request(self, service, callback): self.calls.append((service, callback))
    transport = Transport(); events = []
    backend = RosBackend(transport, lambda *event: events.append(event))
    backend.connect(); backend.poll(); backend.release()
    transport.calls[0][1](NS(success=False, message='BOOTSTRAP_PENDING'))
    backend.poll()
    assert transport.calls[-1][0] == 'disable_and_release'

def test_ros_config_requires_ros_deployment_but_not_native(tmp_path):
    import json
    from manual_panel_config import load_panel_config
    description = tmp_path / 'robot.xacro'; description.write_text('<robot/>')
    controller = tmp_path / 'controller.yaml'; controller.write_text('controller: {}')
    path = tmp_path / 'panel.json'
    path.write_text(json.dumps(dict(sdk_prefix=str(tmp_path), ros_controller_config=str(controller), description_xacro=str(description))))
    assert load_panel_config(path, 'ros')['sdk_prefix'] == str(tmp_path)


def test_auto_polls_cpp_baseline_terminal_instead_of_bootstrap_operation():
    from manual_panel_ros_backend import RosBackend
    class Transport:
        status_max_age_ms = 731  # 假C++响应，不是产品预算。
        def __init__(self): self.calls = []
        def request(self, service, callback): self.calls.append((service, callback))
    transport = Transport(); events = []
    backend = RosBackend(transport, lambda *event: events.append(event))
    backend.connect(); backend.request_calibration_and_test()
    transport.calls[-1][1](NS(success=True, message='BASELINE_QUEUED'))
    backend.poll()
    assert transport.calls[-1][0] == 'baseline_status'
    terminal = 'BASELINE_PASSED boot=123 report=/tmp/baseline.log'
    transport.calls[-1][1](NS(success=True, message=terminal))
    assert events[-1] == ('completed', terminal)


def test_failed_cpp_baseline_is_a_failure_even_after_trigger_acceptance():
    from manual_panel_ros_backend import RosBackend
    class Transport:
        status_max_age_ms = 731  # 假C++响应，不是产品预算。
        def request(self, service, callback): self.callback = callback
    transport = Transport(); events = []
    backend = RosBackend(transport, lambda *event: events.append(event))
    backend.connect(); backend.request_calibration_and_test()
    transport.callback(NS(success=True, message='BASELINE_QUEUED'))
    backend.poll()
    transport.callback(NS(success=False, message='BASELINE_FAILED:CALIBRATION_REJECTED'))
    assert events[-1][0] == 'failed'
    assert backend.operation is None


def test_action_cancel_terminal_preserves_actual_goal_uuid():
    from manual_panel_ros_backend import RosTransport
    class Future:
        def __init__(self, result): self.value = result
        def result(self): return self.value
        def add_done_callback(self, callback): callback(self)
    goal = NS(accepted=True, goal_id=NS(uuid=[1]*16),
              get_result_async=lambda: Future(NS(status=5, result=NS(error_code=0, error_string=''))))
    action = NS(server_is_ready=lambda: True,
                send_goal_async=lambda *_args, **_kwargs: Future(goal))
    transport = object.__new__(RosTransport)
    transport.node = NS(count_services=lambda _: 1, count_publishers=lambda _: 1)
    transport.action = action; transport.action_type = NS(Goal=lambda: NS())
    transport.uuid_type=lambda **kw:NS(**kw)
    transport.closed=False;transport._submission=None
    events = []
    assert transport.submit_trajectory(NS(), lambda *event: events.append(event))
    assert events[0] == ('accepted', {'goal_uuid': '01'*16})
    assert events[-1] == ('canceled', {'goal_uuid': '01'*16, 'status': 5})


def test_manual_confirmation_is_explicit_and_does_not_start_auto_task():
    from manual_panel_ros_backend import confirmed_ros_launch_argv
    original = ['ros2', 'launch', 'dummy_bringup', 'manual_ros_control.launch.py',
                'operator_confirmed_double_limits:=false', 'auto_calibrate_and_test:=false']
    result = confirmed_ros_launch_argv(original, True)
    assert original[-2] == 'operator_confirmed_double_limits:=false'
    assert result.count('operator_confirmed_double_limits:=true') == 1
    assert 'operator_confirmed_double_limits:=false' not in result
    assert 'auto_calibrate_and_test:=false' in result
    assert confirmed_ros_launch_argv(original, False) == original


def test_manual_prepare_is_thin_and_ready_is_not_baseline_success():
    from manual_panel_ros_backend import RosBackend
    calls, events = [], []
    class Transport:
        status_max_age_ms = 731  # 假C++响应，不是产品预算。
        def request(self, name, callback):
            calls.append(name)
            callback(NS(success=True, message='MANUAL_CONTROL_READY boot=7 session=8' if name == 'manual_control_status' else 'MANUAL_CONTROL_ACCEPTED'))
    backend = RosBackend(Transport(), lambda *event: events.append(event))
    backend.connect()
    assert backend.request_prepare_manual_control()
    assert calls == ['prepare_manual_control']
    backend.poll()
    assert calls[-1] == 'manual_control_status'
    assert events[-1][0] == 'manual_ready'
    assert not backend.released


def test_baseline_terminal_releases_admission_without_losing_details():
    from manual_panel_ros_backend import RosBackend
    class Transport:
        status_max_age_ms = 731  # 假C++响应，不是产品预算。
        def request(self, name, callback):
            callback(NS(success=True, message='BASELINE_PASSED boot=7 report=/evidence/log'))
    backend = RosBackend(Transport(), lambda *event: None)
    backend.connect(); backend.operation = 'auto'
    backend.poll()
    assert backend.released and not backend.admitting


def test_manual_ready_poll_does_not_reset_busy_for_each_following_goal():
    from manual_panel_ros_backend import RosBackend
    events=[]
    class Transport:
        status_max_age_ms = 731  # 假C++响应，不是产品预算。
        def request(self, name, callback):
            callback(NS(success=True, message='MANUAL_CONTROL_READY boot=7 session=8'))
    backend=RosBackend(Transport(),lambda *event:events.append(event))
    backend.connect();backend.operation='manual'
    backend.poll();backend.poll()
    assert sum(kind=='manual_ready' for kind,value in events)==1


def test_parameter_budget_is_nonblocking_single_request_and_authoritative():
    from manual_panel_ros_backend import RosTransport
    class Future:
        def add_done_callback(self, callback): self.callback = callback
        def result(self): return NS(values=[NS(type=2, integer_value=731), NS(type=2, integer_value=251)])
    future = Future()
    calls=[]
    transport=object.__new__(RosTransport)
    transport.node = NS(count_services=lambda _: 1, count_publishers=lambda _: 1)
    transport.parameter_client=NS(service_is_ready=lambda:True,call_async=lambda request:calls.append(request.names) or future)
    transport.get_parameters_type=NS(Request=lambda:NS())
    transport.parameter_integer_type=2
    transport.status_max_age_ms=None
    transport.parameter_pending=False
    transport.parameter_retry_at=0.
    transport.closed=False
    transport.refresh_status_budget()
    transport.refresh_status_budget()
    assert calls==[['status_max_age_ms','feedback_max_age_ms']]
    assert transport.status_max_age_ms is None
    future.callback(future)
    assert transport.status_max_age_ms==731
    transport.refresh_status_budget()
    assert len(calls)==1


def test_unavailable_cpp_budget_keeps_motion_readonly_but_release_available():
    from manual_panel_ros_backend import RosBackend
    calls=[]
    transport=NS(status_max_age_ms=None,request=lambda name,callback:calls.append(name))
    backend=RosBackend(transport,lambda *event:None)
    backend.connect()
    assert not backend.request_calibration_and_test()
    assert not backend.request_prepare_manual_control()
    assert not backend.submit_trajectory(NS())
    assert calls==[]
    backend.release()
    assert calls==['disable_and_release']


def test_invalid_parameter_response_does_not_invent_a_budget():
    from manual_panel_ros_backend import RosTransport
    transport=object.__new__(RosTransport)
    transport.node = NS(count_services=lambda _: 1, count_publishers=lambda _: 1)
    class Future:
        def add_done_callback(self, callback): callback(self)
        def result(self): return NS(values=[NS(type=0,integer_value=0)])
    transport.parameter_client=NS(service_is_ready=lambda:True,call_async=lambda request:Future())
    transport.get_parameters_type=NS(Request=lambda:NS())
    transport.parameter_integer_type=2
    transport.status_max_age_ms=None;transport.parameter_pending=False;transport.parameter_retry_at=0.;transport.closed=False
    transport.refresh_status_budget()
    assert transport.status_max_age_ms is None
    assert not transport.parameter_pending


# 独立假时钟/服务/Action刺激续租占槽排序；不创建SDK设备连接。
def renewal_transport(monkeypatch, receipts=None):
    import manual_panel_ros_backend as module
    class Future:
        def __init__(self,value): self.value=value
        def result(self): return self.value
        def add_done_callback(self,callback): callback(self)
    clock=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    transport=object.__new__(module.RosTransport)
    transport.node = NS(count_services=lambda _: 1, count_publishers=lambda _: 1)
    transport.closed=False;transport.goal=None;transport._submission=None
    transport.uuid_type=lambda **kwargs:NS(**kwargs)
    transport.action_type=NS(Goal=lambda:NS())
    transport.status_max_age_ms=731;transport.feedback_max_age_ms=251
    state=NS(device_id=[1]*16,boot_epoch=7,profile_id=201,session_id=8,reference_epoch=9,
             motion_epoch=10,management_epoch=11,state_seq=1,core_state=4,hold_ready=True,
             position_valid_mask=63,velocity_valid_mask=63,can_rx_age_us=[0]*6)
    status=NS(**vars(state),real_motion_permitted=True,permit_remaining_us=10000000,evidence_lost=False)
    transport.latest_state=state;transport.latest_status=status
    transport.state_at=transport.status_at=10.
    calls=[];pending=[];events=[]
    result=NS(accepted=True,goal_id=NS(uuid=[2]*16),get_result_async=lambda:NS(add_done_callback=lambda cb:None),
              cancel_goal_async=lambda:calls.append('cancel_accepted'))
    def send(goal,**kwargs):
        calls.append((goal,kwargs))
        return Future(NS(accepted=False) if len([c for c in calls if isinstance(c,tuple)])==1 else result)
    transport.action=NS(server_is_ready=lambda:True,send_goal_async=send)
    transport.request=lambda name,cb:pending.append((name,cb))
    trajectory=NS(points=[NS(time_from_start=NS(sec=1,nanosec=0))],joint_names=['Joint1'])
    return transport,trajectory,clock,calls,pending,events


def test_renewal_retries_only_uuid_receipt_and_cpp_ready_same_binding(monkeypatch):
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    assert transport.submit_trajectory(trajectory,lambda *e:events.append(e))
    transport._poll_trajectory_retry()
    assert pending[-1][0]=='last_goal_rejection'
    first_uuid=bytes(calls[0][1]['goal_uuid'].uuid).hex()
    assert not any(e[0]=='accepted' for e in events)
    pending.pop()[1](NS(success=True,message='RENEWAL_PENDING:'+first_uuid))
    transport._poll_trajectory_retry()
    assert pending[-1][0]=='bootstrap_status'
    pending.pop()[1](NS(success=False,message='LEASE_RENEWAL_PENDING'))
    assert len(calls)==1
    transport.latest_state.management_epoch=12;transport.latest_status.management_epoch=12
    transport.latest_state.state_seq=2;clock[0]+=.01;transport.status_at=clock[0]
    transport._poll_trajectory_retry()
    pending.pop()[1](NS(success=True,message='ENABLED_HOLD'))  # 自动Renew无需LEASE_RENEWED提示。
    assert len(calls)==2
    assert vars(calls[0][0].trajectory)==vars(calls[1][0].trajectory)
    assert bytes(calls[0][1]['goal_uuid'].uuid)!=bytes(calls[1][1]['goal_uuid'].uuid)
    assert sum(e[0]=='accepted' for e in events)==1


@pytest.mark.parametrize('message', ['NOT_READY','INVALID_GOAL','RENEWAL_PENDING:'+'00'*16])
def test_other_rejection_never_retries(monkeypatch,message):
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    transport.submit_trajectory(trajectory,lambda *e:events.append(e))
    transport._poll_trajectory_retry();pending.pop()[1](NS(success=True,message=message))
    transport._poll_trajectory_retry()
    assert len(calls)==1 and events[-1][0]=='failed'


@pytest.mark.parametrize('changed', ['stale','session','reference','motion','management','cancel','deadline'])
def test_pending_retry_cannot_survive_changed_binding_cancel_or_deadline(monkeypatch,changed):
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    transport.submit_trajectory(trajectory,lambda *e:events.append(e))
    transport._poll_trajectory_retry()
    first_uuid=bytes(calls[0][1]['goal_uuid'].uuid).hex()
    pending.pop()[1](NS(success=True,message='RENEWAL_PENDING:'+first_uuid))
    transport._poll_trajectory_retry()
    cb=pending.pop()[1]
    if changed=='stale':clock[0]+=.5
    elif changed=='deadline':clock[0]+=2.01
    elif changed=='cancel':transport.cancel()
    else:
        field={'session':'session_id','reference':'reference_epoch','motion':'motion_epoch','management':'management_epoch'}[changed]
        setattr(transport.latest_state,field,getattr(transport.latest_state,field)+2)
        setattr(transport.latest_status,field,getattr(transport.latest_status,field)+2)
    cb(NS(success=True,message='ENABLED_HOLD'))
    transport._poll_trajectory_retry()
    assert len(calls)==1 and events[-1][0]=='failed'
    assert not any(e[0] in ('accepted','canceled') for e in events)


def test_ready_waits_for_management_streams_without_extending_original_retry_deadline(monkeypatch):
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    transport.submit_trajectory(trajectory,lambda *e:events.append(e))
    deadline=transport._submission['retry_deadline']
    transport._poll_trajectory_retry()
    first_uuid=bytes(calls[0][1]['goal_uuid'].uuid).hex()
    pending.pop()[1](NS(success=True,message='RENEWAL_PENDING:'+first_uuid))
    transport.latest_state.management_epoch=12
    transport._poll_trajectory_retry();pending.pop()[1](NS(success=True,message='ENABLED_HOLD'))
    assert len(calls)==1 and transport._submission['retry_deadline']==deadline
    transport.latest_status.management_epoch=12
    transport.latest_state.state_seq=2;clock[0]+=.01;transport.status_at=clock[0]
    transport._poll_trajectory_retry();pending.pop()[1](NS(success=True,message='ENABLED_HOLD'))
    assert len(calls)==2 and transport._submission['retry_deadline']==deadline


def test_late_rejection_callback_cannot_recreate_closed_submission(monkeypatch):
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    transport.submit_trajectory(trajectory,lambda *e:events.append(e))
    transport._poll_trajectory_retry();callback=pending.pop()[1]
    transport.cancel_pending_submission();transport.closed=True
    callback(NS(success=True,message='RENEWAL_PENDING:'+bytes(calls[0][1]['goal_uuid'].uuid).hex()))
    transport._poll_trajectory_retry()
    assert len(calls)==1 and events[-1][0]=='failed'


def test_real_rclpy_uuid_rejection_receipt_ready_and_same_trajectory():
    import time
    rclpy=pytest.importorskip('rclpy')
    msg=pytest.importorskip('dpsdk_interfaces.msg')
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    from std_srvs.srv import Trigger
    from rclpy.action import ActionServer,GoalResponse
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.context import Context
    from manual_panel_ros_backend import RosTransport
    context=Context();rclpy.init(context=context,domain_id=214)
    server=rclpy.create_node('synthetic_manual_retry_peer',context=context)
    client=rclpy.create_node('synthetic_manual_retry_client',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(server);executor.add_node(client)
    received=[];attempt_uuids=[];events=[];renew_polls=[]
    transport=RosTransport(client,lambda value:None,lambda value:None)
    transport.status_max_age_ms=731;transport.feedback_max_age_ms=251
    from unique_identifier_msgs.msg import UUID
    transport.uuid_type=lambda **kwargs:attempt_uuids.append(bytes(kwargs['uuid']).hex()) or UUID(**kwargs)
    def goal(request):
        received.append(request.trajectory)
        return GoalResponse.REJECT if len(received)==1 else GoalResponse.ACCEPT
    def execute(handle):
        handle.succeed();return FollowJointTrajectory.Result()
    action=ActionServer(server,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory',
                        goal_callback=goal,execute_callback=execute)
    def receipt(request,response):
        response.success=True;response.message='RENEWAL_PENDING:'+attempt_uuids[0];return response
    def ready(request,response):
        renew_polls.append(True);response.success=True;response.message='ENABLED_HOLD';return response
    services=[server.create_service(Trigger,'/device_action_gate/last_goal_rejection',receipt),
              server.create_service(Trigger,'/device_action_gate/bootstrap_status',ready)]
    status_pub=server.create_publisher(msg.DummyDeviceStatusV2,'/device_action_gate/device_status',10)
    state_pub=server.create_publisher(msg.DummyJointStateV2,'/device_action_gate/device_state',10)
    status=msg.DummyDeviceStatusV2();state=msg.DummyJointStateV2()
    for value in (state,status):
        value.device_id=[1]*16;value.boot_epoch=7;value.profile_id=201;value.session_id=8
        value.reference_epoch=9;value.motion_epoch=10;value.management_epoch=11;value.core_state=4
    state.hold_ready=True;state.position_valid_mask=63;state.velocity_valid_mask=63
    status.real_motion_permitted=True;status.permit_remaining_us=10000000
    def publish():
        state.state_seq+=1;state_pub.publish(state);status_pub.publish(status)
    timer=server.create_timer(.02,publish)
    try:
        until=time.monotonic()+5
        while time.monotonic()<until and (not transport.action.server_is_ready() or transport._fresh_retry_binding() is None):
            executor.spin_once(timeout_sec=.02)
        assert transport._fresh_retry_binding() is not None and transport.action.server_is_ready()
        trajectory=JointTrajectory();trajectory.joint_names=['Joint1']
        point=JointTrajectoryPoint();point.positions=[.001];point.time_from_start.sec=1;trajectory.points=[point]
        assert transport.submit_trajectory(trajectory,lambda *event:events.append(event))
        until=time.monotonic()+4
        while time.monotonic()<until and not any(e[0] in ('completed','failed') for e in events):
            executor.spin_once(timeout_sec=.02)
        assert received==[trajectory,trajectory]
        assert len(set(attempt_uuids))==2 and renew_polls
        assert [e[0] for e in events]==['accepted','completed']
    finally:
        transport.close();action.destroy();server.destroy_timer(timer)
        for service in services:server.destroy_service(service)
        executor.shutdown();client.destroy_node();server.destroy_node();rclpy.shutdown(context=context)


def test_second_renewal_rejection_does_not_create_an_unbounded_retry(monkeypatch):
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    def reject(goal,**kwargs):
        calls.append((goal,kwargs))
        future=NS(result=lambda:NS(accepted=False))
        future.add_done_callback=lambda cb:cb(future)
        return future
    transport.action.send_goal_async=reject
    transport.submit_trajectory(trajectory,lambda *e:events.append(e))
    transport._poll_trajectory_retry()
    pending.pop()[1](NS(success=True,message='RENEWAL_PENDING:'+bytes(calls[0][1]['goal_uuid'].uuid).hex()))
    transport.latest_state.state_seq=2;clock[0]+=.01;transport.status_at=clock[0]
    transport._poll_trajectory_retry();pending.pop()[1](NS(success=True,message='ENABLED_HOLD'))
    assert len(calls)==2
    transport._poll_trajectory_retry()
    pending.pop()[1](NS(success=True,message='RENEWAL_PENDING:'+bytes(calls[1][1]['goal_uuid'].uuid).hex()))
    transport._poll_trajectory_retry()
    assert len(calls)==2 and events[-1][0]=='failed'


def test_close_cancels_pending_retry_and_destroys_timer(monkeypatch):
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    destroyed=[]
    transport.node=NS(count_services=lambda _:1,count_publishers=lambda _:1,destroy_timer=lambda timer:destroyed.append('timer'),destroy_client=lambda client:None,
                      destroy_subscription=lambda sub:None)
    transport.retry_timer=object();transport.parameter_client=object();transport.clients={};transport.subscriptions=[]
    transport.action.destroy=lambda:destroyed.append('action')
    transport.submit_trajectory(trajectory,lambda *e:events.append(e))
    transport._poll_trajectory_retry();cb=pending.pop()[1]
    transport.close()
    cb(NS(success=True,message='RENEWAL_PENDING:'+bytes(calls[0][1]['goal_uuid'].uuid).hex()))
    assert destroyed==['timer','action'] and transport._submission is None
    assert len(calls)==1 and events[-1][0]=='failed'


def test_release_revokes_pending_retry_before_forwarding_disable(monkeypatch):
    from manual_panel_ros_backend import RosBackend
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    backend=RosBackend(transport,lambda *event:events.append(event));backend.connect()
    backend.submit_trajectory(trajectory)
    backend.release()
    assert transport._submission is None and not backend.admitting
    assert pending[-1][0]=='disable_and_release'
    assert not any(e[0] in ('accepted','canceled') for e in events)


@pytest.mark.parametrize('advanced', ['neither','state_only','status_only'])
def test_cpp_ready_alone_cannot_reuse_pre_rejection_snapshots(monkeypatch,advanced):
    transport,trajectory,clock,calls,pending,events=renewal_transport(monkeypatch)
    transport.submit_trajectory(trajectory,lambda *e:events.append(e))
    transport._poll_trajectory_retry()
    pending.pop()[1](NS(success=True,message='RENEWAL_PENDING:'+bytes(calls[0][1]['goal_uuid'].uuid).hex()))
    clock[0]+=.01
    if advanced=='state_only':transport.latest_state.state_seq=2
    if advanced=='status_only':transport.status_at=clock[0]
    transport._poll_trajectory_retry();pending.pop()[1](NS(success=True,message='ENABLED_HOLD'))
    assert len(calls)==1 and not any(e[0]=='accepted' for e in events)
    clock[0]+=2.
    transport._poll_trajectory_retry()
    assert events[-1][0]=='failed'


def test_release_unavailable_retries_with_fixed_deadline_and_queued_is_not_released(monkeypatch):
    import manual_panel_ros_backend as module
    from types import SimpleNamespace
    now = [10.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    class Transport:
        def __init__(self): self.calls = []; self.closed = False
        def request(self, service, callback): self.calls.append((service, callback))
        def close(self): self.closed = True
    transport = Transport(); events = []
    backend = module.RosBackend(transport, lambda *event: events.append(event))
    backend.connect(); backend.release()
    transport.calls[-1][1](SimpleNamespace(success=False, message='RELEASE_UNAVAILABLE'))
    assert not backend.released and not backend.close()
    now[0] = 10.199; backend.poll()
    assert len(transport.calls) == 1
    now[0] = 10.2; backend.poll()
    assert [x[0] for x in transport.calls] == ['disable_and_release'] * 2
    transport.calls[-1][1](SimpleNamespace(success=False, message='RELEASE_QUEUED'))
    backend.release(); backend.poll()
    assert [x[0] for x in transport.calls].count('disable_and_release') == 2
    assert transport.calls[-1][0] == 'bootstrap_status' and not backend.released
    transport.calls[-1][1](SimpleNamespace(success=False, message='RELEASE_PENDING'))
    backend.poll()
    transport.calls[-1][1](SimpleNamespace(success=True, message='RELEASED_IDLE'))
    assert backend.released and events[-1] == ('released', 'RELEASED_IDLE')
    assert backend.close() and transport.closed


def test_release_unavailable_expires_original_deadline_and_repeat_cannot_restart(monkeypatch):
    import manual_panel_ros_backend as module
    from types import SimpleNamespace
    now = [40.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    class Transport:
        def __init__(self): self.calls = []; self.closed = False
        def request(self, service, callback): self.calls.append((service, callback))
        def close(self): self.closed = True
    transport = Transport(); events = []
    backend = module.RosBackend(transport, lambda *event: events.append(event))
    backend.connect(); backend.release()
    transport.calls[-1][1](SimpleNamespace(success=False, message='RELEASE_UNAVAILABLE'))
    now[0] = 59.9; backend.release()
    assert len(transport.calls) == 2
    transport.calls[-1][1](SimpleNamespace(success=False, message='RELEASE_UNAVAILABLE'))
    now[0] = 60.0; backend.poll()
    assert not backend.released and not backend.close() and not transport.closed
    assert events[-1][0] == 'failed' and 'RELEASE_UNCONFIRMED' in events[-1][1]
    backend.release(); now[0] = 80.0; backend.poll()
    assert len(transport.calls) == 2
    assert sum(event[0] == 'failed' for event in events) == 1


def test_control_graph_counts_real_servers_and_refuses_existing_owner(monkeypatch):
    import manual_panel_ros_backend as module
    services = {}; publishers = {}
    node = NS(count_services=lambda name: services.get(name, 0),
              count_publishers=lambda name: publishers.get(name, 0))
    transport = object.__new__(module.RosTransport); transport.node = node
    for name in module.CONTROL_SERVICES: services[name] = 1
    for name in module.CONTROL_TOPICS: publishers[name] = 1
    assert transport.control_graph_error() == ''
    services[module.CONTROL_SERVICES[0]] = 2
    assert '控制链' in transport.control_graph_error()
    services[module.CONTROL_SERVICES[0]] = 1
    publishers[module.CONTROL_TOPICS[0]] = 2
    assert transport.control_graph_error()
    with pytest.raises(RuntimeError, match='已有ROS控制链'):
        transport.assert_no_existing_owner(timeout=0)
    services.clear(); publishers.clear()
    assert transport.control_graph_error()  # Conflict is latched, no later takeover.
    del transport._startup_owner_conflict
    transport.assert_no_existing_owner(timeout=0)
    assert transport.control_graph_error()  # Missing graph cannot authorize attach.


def test_duplicate_control_graph_blocks_management_and_fjt_without_wire_requests():
    from manual_panel_ros_backend import RosTransport
    transport = object.__new__(RosTransport)
    transport.node = NS(count_services=lambda _:2, count_publishers=lambda _:1)
    events = []
    transport.request('disable_and_release', lambda result:events.append(result))
    assert isinstance(events[-1], RuntimeError)
    assert not transport.submit_trajectory(NS(), lambda *event:events.append(event))
    assert events[-1][0] == 'failed' and '重复实例' in events[-1][1]
