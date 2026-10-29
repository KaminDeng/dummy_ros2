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

@pytest.mark.parametrize('from_poll',[False,True])
@pytest.mark.parametrize('success',[False,True])
def test_cpp_readonly_retirement_closes_only_local_connection_without_claiming_disable(from_poll,success):
    from manual_panel_ros_backend import RosBackend
    class Transport:
        def request(self,service,callback):self.callback=callback
        def close(self):self.closed=True
    transport=Transport();events=[];backend=RosBackend(transport,lambda *event:events.append(event))
    backend.connect();backend.release()
    if from_poll:
        transport.callback(NS(success=False,message='RELEASE_QUEUED'));backend.poll()
    transport.callback(NS(success=success,message='LOCAL_READONLY_RETIRED'))
    assert backend.local_retired and backend.can_close
    assert not backend.released and not backend.admitting and not backend.manual_ready
    assert events[-1]==('retired','LOCAL_READONLY_RETIRED')
    assert not any(k in ('released','completed') for k,v in events)
    assert backend.close() and transport.closed

def test_unsolicited_readonly_retirement_does_not_close_active_connection():
    from manual_panel_ros_backend import RosBackend
    class Transport:
        def request(self,service,callback):self.callback=callback
    t=Transport();events=[];b=RosBackend(t,lambda *event:events.append(event));b.connect();b.poll()
    t.callback(NS(success=True,message='LOCAL_READONLY_RETIRED'))
    assert not b.can_close and not b.local_retired and not b.released
    assert events[-1][0]=='progress'

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
    import time
    now=time.monotonic()
    transport.latest_state=NS(device_id=[1]*16,profile_id=201,boot_epoch=7,session_id=8,
        reference_epoch=9,motion_epoch=10,management_epoch=1,state_seq=1,core_state=4,
        hold_ready=True,position_valid_mask=63,velocity_valid_mask=63,can_rx_age_us=[0]*6)
    transport.latest_status=NS(**vars(transport.latest_state),real_motion_permitted=True,
        permit_remaining_us=20000000,evidence_lost=False)
    transport.state_at=transport.status_at=now
    transport.feedback_max_age_ms=250;transport.status_max_age_ms=1000
    def request(name,cb):
        if name=='set_bench_permit':
            transport.latest_state.management_epoch+=1;transport.latest_status.management_epoch+=1
            transport.latest_state.state_seq+=1
            transport.state_at=transport.status_at=time.monotonic()
            cb(NS(success=True,message='QUEUED'))
        else:cb(NS(success=True,message='ENABLED_HOLD'))
    transport.request=request
    events = []
    assert transport.submit_trajectory(NS(points=[NS(time_from_start=NS(sec=1,nanosec=0))]), lambda *event: events.append(event))
    transport._readonly_requests={}
    transport._poll_trajectory_retry()
    transport._poll_trajectory_retry()
    transport._poll_trajectory_retry()
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
def initial_permit_readiness(t,pending):
    t._poll_trajectory_retry()
    name,ready=pending.pop(0)
    assert name=='bootstrap_status'
    ready(NS(success=True,message='ENABLED_HOLD'))


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
    transport.closed=False;transport.goal=None;transport._submission=None;transport._readonly_requests={}
    transport.uuid_type=lambda **kwargs:NS(**kwargs)
    transport.action_type=NS(Goal=lambda:NS())
    transport.status_max_age_ms=731;transport.feedback_max_age_ms=251
    state=NS(device_id=[1]*16,boot_epoch=7,profile_id=201,session_id=8,reference_epoch=9,
             motion_epoch=10,management_epoch=10,state_seq=1,core_state=4,hold_ready=True,
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
    original_submit = transport.submit_trajectory
    def submit_with_initial_cpp_ready(trajectory, callback):
        accepted = original_submit(trajectory, callback)
        if accepted:
            initial_permit_readiness(transport,pending)
            transport._poll_trajectory_retry()
            name, response = pending.pop(0)
            assert name == 'set_bench_permit'
            response(NS(success=True, message='QUEUED'))
            transport.latest_state.management_epoch+=1;transport.latest_status.management_epoch+=1
            transport.latest_state.state_seq+=1;clock[0]+=.001
            transport.state_at=transport.status_at=clock[0]
            transport._poll_trajectory_retry()
            name,response=pending.pop(0)
            assert name=='bootstrap_status'
            response(NS(success=True, message='ENABLED_HOLD'))
        return accepted
    transport.submit_trajectory = submit_with_initial_cpp_ready
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
    transport.latest_state.state_seq=3;clock[0]+=.01;transport.status_at=clock[0]
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
    transport.latest_state.state_seq=3;clock[0]+=.01;transport.status_at=clock[0]
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
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterValue,ParameterType
    from controller_manager_msgs.srv import ListControllers
    from rclpy.action import ActionServer,GoalResponse
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.context import Context
    from manual_panel_ros_backend import RosTransport,CONTROL_SERVICES,CONTROL_TOPICS
    context=Context();rclpy.init(context=context,domain_id=214)
    server=rclpy.create_node('synthetic_manual_retry_peer',context=context)
    client=rclpy.create_node('synthetic_manual_retry_client',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(server);executor.add_node(client)
    received=[];attempt_uuids=[];events=[];renew_polls=[]
    transport=RosTransport(client,lambda value:None,lambda value:None)
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
    def install_permit(request,response):
        state.management_epoch+=1;status.management_epoch+=1
        response.success=True;response.message='QUEUED';return response
    def unused_control(request,response):
        raise AssertionError('续租窄重试场景不应调用标定、释放或控制器管理')
    def parameters(request,response):
        budgets={'status_max_age_ms':731,'feedback_max_age_ms':251}
        response.values=[ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                        integer_value=budgets[name]) for name in request.names]
        return response
    services=[server.create_service(Trigger,'/device_action_gate/last_goal_rejection',receipt),
              server.create_service(Trigger,'/device_action_gate/bootstrap_status',ready),
              server.create_service(Trigger,'/device_action_gate/set_bench_permit',install_permit),
              server.create_service(GetParameters,'/manual_ros_baseline/get_parameters',parameters),
              server.create_service(ListControllers,'/controller_manager/list_controllers',unused_control)]
    # 注册真实DDS服务与下方typed发布者，满足当前唯一控制owner守卫；不伪造计数。
    services.extend(server.create_service(Trigger,name,unused_control) for name in CONTROL_SERVICES
                    if name not in ('/device_action_gate/bootstrap_status','/device_action_gate/set_bench_permit',
                                    '/manual_ros_baseline/get_parameters',
                                    '/controller_manager/list_controllers'))
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
            transport.refresh_status_budget()
            executor.spin_once(timeout_sec=.02)
        assert transport._fresh_retry_binding() is not None and transport.action.server_is_ready()
        assert all(client.count_services(name)==1 for name in CONTROL_SERVICES)
        assert all(client.count_publishers(name)==1 for name in CONTROL_TOPICS)
        assert transport.status_max_age_ms==731 and transport.feedback_max_age_ms==251
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
    transport.latest_state.state_seq=3;clock[0]+=.01;transport.status_at=clock[0]
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
    if advanced=='state_only':transport.latest_state.state_seq=3
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
    assert sum(name == 'disable_and_release' for name, _ in transport.calls) == 2
    assert transport.calls[-1][0] == 'bootstrap_status'
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
    transport = object.__new__(RosTransport);transport.closed=False
    transport.node = NS(count_services=lambda _:2, count_publishers=lambda _:1)
    events = []
    transport.request('disable_and_release', lambda result:events.append(result))
    assert isinstance(events[-1], RuntimeError)
    assert not transport.submit_trajectory(NS(), lambda *event:events.append(event))
    assert events[-1][0] == 'failed' and '重复实例' in events[-1][1]


def test_owner_preflight_waits_for_departing_graph_without_reentrant_callbacks(monkeypatch):
    import manual_panel_ros_backend as module
    clock = [0.0]; counts = [1]
    def wait(seconds): clock[0] += seconds; counts[0] = 0
    monkeypatch.setattr(module.time, 'monotonic', lambda:clock[0])
    monkeypatch.setattr(module.time, 'sleep', wait)
    transport = object.__new__(module.RosTransport)
    transport.node = NS(count_services=lambda _:counts[0],count_publishers=lambda _:counts[0])
    transport._startup_owner_conflict = 'previous explicit attempt failed'
    transport.assert_no_existing_owner(timeout=.1)
    assert clock[0] >= .1 and not transport._startup_owner_conflict
    counts[0] = 1
    monkeypatch.setattr(module.time, 'sleep', lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    start = clock[0]
    with pytest.raises(RuntimeError,match='已有ROS控制链'):
        transport.assert_no_existing_owner(timeout=.1)
    assert .1 <= clock[0]-start < .13
    assert transport.control_graph_error()
    counts[0] = 0
    transport.assert_no_existing_owner(timeout=.1)
    assert not transport._startup_owner_conflict


def test_lost_parameter_reply_retries_and_ignores_old_generation(monkeypatch):
    import manual_panel_ros_backend as module
    clock=[10.0];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    class Future:
        cancelled=False
        def add_done_callback(self,callback):self.callback=callback
        def cancel(self):self.cancelled=True
        def result(self):return NS(values=[NS(type=2,integer_value=1000),NS(type=2,integer_value=250)])
    futures=[]
    def send(request):
        future=Future();futures.append(future);return future
    transport=object.__new__(module.RosTransport)
    transport.parameter_client=NS(service_is_ready=lambda:True,call_async=send)
    transport.get_parameters_type=NS(Request=lambda:NS());transport.parameter_integer_type=2
    transport.closed=False;transport.status_max_age_ms=None;transport.feedback_max_age_ms=None
    transport.parameter_pending=False;transport.parameter_retry_at=0.
    transport.refresh_status_budget()
    clock[0]=10.999;transport.refresh_status_budget();assert len(futures)==1
    clock[0]=11.;transport.refresh_status_budget()
    assert len(futures)==2 and futures[0].cancelled
    futures[0].callback(futures[0])
    assert transport.parameter_pending and transport.status_max_age_ms is None
    futures[1].callback(futures[1])
    assert not transport.parameter_pending
    assert (transport.status_max_age_ms,transport.feedback_max_age_ms)==(1000,250)
    transport.closed=True;futures[0].callback(futures[0])
    assert transport.status_max_age_ms==1000


def test_owner_preflight_default_allows_700ms_dds_departure_but_never_takes_live_owner(monkeypatch):
    import manual_panel_ros_backend as module
    clock=[0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.time,'sleep',lambda delay:clock.__setitem__(0,clock[0]+delay))
    transport=module.RosTransport.__new__(module.RosTransport)
    transport._startup_owner_conflict='previous startup refused'
    transport.node=NS(count_services=lambda _:int(clock[0]<.7),count_publishers=lambda _:int(clock[0]<.7))
    transport.assert_no_existing_owner()
    assert .7<=clock[0]<2. and not transport._startup_owner_conflict
    # 始终为空仍只消耗原半秒发现期，不能无条件延长所有连接。
    clock[0]=0.
    transport.node=NS(count_services=lambda _:0,count_publishers=lambda _:0)
    transport.assert_no_existing_owner()
    assert .5<=clock[0]<.53
    # 真拥有者始终存在；原固定2秒截止拒绝，不接管或清理外部链。
    clock[0]=0.
    transport.node=NS(count_services=lambda _:1,count_publishers=lambda _:1)
    with pytest.raises(RuntimeError,match='已有ROS控制链'):
        transport.assert_no_existing_owner()
    assert 2.<=clock[0]<2.03 and transport._startup_owner_conflict
    # 发现期中较晚出现的新拥有者也必须拒绝，不能只采第一个空图。
    clock[0]=0.
    transport.node=NS(count_services=lambda _:int(clock[0]>=.3),count_publishers=lambda _:0)
    with pytest.raises(RuntimeError,match='已有ROS控制链'):
        transport.assert_no_existing_owner()
    assert 2.<=clock[0]<2.03 and transport._startup_owner_conflict


def readonly_rpc_transport(monkeypatch):
    import manual_panel_ros_backend as module
    clock=[10.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    class Future:
        def __init__(self):self.callback=None;self.value=None;self.cancelled=False
        def add_done_callback(self,callback):self.callback=callback
        def result(self):return self.value
        def cancel(self):
            self.cancelled=True
            if self.callback:self.callback(self)  # 取消也可同步触发旧回调。
        def resolve(self,success,message):
            self.value=NS(success=success,message=message);self.callback(self)
    transport=module.RosTransport.__new__(module.RosTransport)
    transport.closed=False;transport._submission=None;transport._readonly_requests={}
    transport.goal=None;transport.trigger=NS(Request=lambda:NS(),Response=lambda **kw:NS(**kw))
    transport.node=NS(count_services=lambda _:1,count_publishers=lambda _:1,
        destroy_timer=lambda _:None,destroy_client=lambda _:None,destroy_subscription=lambda _:None)
    transport.retry_timer=object();transport.parameter_client=object();transport.subscriptions=[]
    transport.action=NS(destroy=lambda:None)
    calls=[]
    def call(service):
        future=Future();calls.append((service,future));return future
    transport.clients={name:NS(service_is_ready=lambda:True,call_async=lambda _,name=name:call(name))
        for name in ['bootstrap_status','manual_control_status','baseline_status','manual_reference_status','last_goal_rejection','disable_and_release','prepare_manual_control']}
    return module,transport,clock,calls


@pytest.mark.parametrize('late_after_acceptance', [False, True])
def test_manual_prepare_is_sent_once_while_bootstrap_query_is_inflight(monkeypatch, late_after_acceptance):
    module,transport,clock,calls=readonly_rpc_transport(monkeypatch)
    transport.status_max_age_ms=731
    events=[];backend=module.RosBackend(transport,lambda *event:events.append(event));backend.connect()
    assert backend.poll();old=calls[-1][1]
    assert backend.request_prepare_manual_control()
    assert [name for name,_ in calls]==['bootstrap_status','prepare_manual_control']
    prepare=calls[-1][1]
    assert not backend.request_prepare_manual_control()
    assert not backend.request_calibration_and_test() and backend.operation=='manual'
    assert not backend.poll() and len(calls)==2
    if late_after_acceptance:
        prepare.resolve(True,'MANUAL_CONTROL_ACCEPTED')
    # 旧只读结果既不能清管理单飞，也不能在新操作中伪造终态。
    old.resolve(True,'BASELINE_PASSED boot=7')
    assert backend.pending is (not late_after_acceptance)
    assert backend.operation=='manual' and not backend.released
    assert not any(kind=='completed' for kind,_ in events)
    if not late_after_acceptance:
        prepare.resolve(True,'MANUAL_CONTROL_ACCEPTED')
    assert events[-1]==('accepted','MANUAL_CONTROL_ACCEPTED')
    assert backend.poll() and calls[-1][0]=='manual_control_status'
    calls[-1][1].resolve(True,'MANUAL_CONTROL_READY boot=7 session=8')
    assert events[-1][0]=='manual_ready'
    assert [name for name,_ in calls].count('prepare_manual_control')==1


def test_readonly_query_is_single_flight_and_never_overtakes_management(monkeypatch):
    module,transport,clock,calls=readonly_rpc_transport(monkeypatch)
    transport.status_max_age_ms=731
    backend=module.RosBackend(transport,lambda *_:None);backend.connect()
    assert backend.poll() and not backend.poll() and len(calls)==1
    assert backend.request_prepare_manual_control()
    assert not backend.poll() and len(calls)==2
    calls[0][1].resolve(False,'BOOTSTRAP_PENDING')
    assert backend.pending and not backend.poll() and len(calls)==2
    calls[1][1].resolve(True,'MANUAL_CONTROL_ACCEPTED')
    assert backend.poll() and not backend.poll() and len(calls)==3


def test_release_has_priority_over_inflight_readonly_and_later_prepare(monkeypatch):
    module,transport,clock,calls=readonly_rpc_transport(monkeypatch)
    transport.status_max_age_ms=731
    backend=module.RosBackend(transport,lambda *_:None);backend.connect();backend.poll()
    assert backend.release()
    assert [name for name,_ in calls]==['bootstrap_status','disable_and_release']
    assert backend.pending and not backend.request_prepare_manual_control()
    assert not backend.poll() and len(calls)==2
    calls[0][1].resolve(True,'RELEASED_IDLE')
    assert backend.pending and not backend.released
    calls[1][1].resolve(False,'RELEASE_QUEUED')
    assert backend.poll() and calls[-1][0]=='bootstrap_status'
    calls[-1][1].resolve(True,'RELEASED_IDLE')
    assert backend.released
    assert [name for name,_ in calls].count('disable_and_release')==1


def test_pending_management_rejects_second_write_without_changing_operation(monkeypatch):
    module,transport,clock,calls=readonly_rpc_transport(monkeypatch)
    transport.status_max_age_ms=731
    backend=module.RosBackend(transport,lambda *_:None);backend.connect()
    assert backend.request_prepare_manual_control()
    assert not backend.request_calibration_and_test()
    assert backend.operation=='manual' and backend.pending
    assert [name for name,_ in calls]==['prepare_manual_control']


def test_lost_readonly_rpc_does_not_block_release_and_late_reply_cannot_confirm_it(monkeypatch):
    module,transport,clock,calls=readonly_rpc_transport(monkeypatch);events=[]
    backend=module.RosBackend(transport,lambda *event:events.append(event));backend.connect()
    backend.poll();old=calls[-1][1]
    assert backend.readonly_query_pending and not backend.pending and calls[-1][0]=='bootstrap_status'
    backend.release();deadline=backend._release_deadline
    assert len(calls)==2 and calls[-1][0]=='disable_and_release' and backend.pending
    clock[0]+=1.01;transport._poll_trajectory_retry()
    assert backend.pending and not backend.readonly_query_pending and old.cancelled and not backend.released
    assert not backend.poll() and calls[-1][0]=='disable_and_release' and backend.pending
    calls[-1][1].resolve(False,'RELEASE_QUEUED')
    old.resolve(True,'RELEASED_IDLE')
    assert not backend.released and backend._release_deadline==deadline
    backend.poll();assert calls[-1][0]=='bootstrap_status'
    calls[-1][1].resolve(True,'RELEASED_IDLE')
    assert backend.released and events[-1][0]=='released'
    assert [x[0] for x in calls].count('disable_and_release')==1


def test_trigger_mutation_is_never_expired_or_reissued_and_close_retires_readonly(monkeypatch):
    module,transport,clock,calls=readonly_rpc_transport(monkeypatch);events=[]
    transport.request('prepare_manual_control',events.append)
    mutation=calls[-1][1];clock[0]+=40.;transport._poll_trajectory_retry()
    assert len(calls)==1 and not mutation.cancelled and not events
    transport.request('manual_control_status',events.append);readonly=calls[-1][1]
    transport.close();readonly.resolve(True,'MANUAL_CONTROL_READY');mutation.resolve(True,'QUEUED')
    assert readonly.cancelled and not transport._readonly_requests and not events
    transport.node.count_services=lambda _:(_ for _ in ()).throw(AssertionError('关闭后不得再访问图'))
    transport.request('bootstrap_status',events.append)
    assert not events


def test_repeated_readonly_timeout_preserves_original_release_deadline(monkeypatch):
    module,transport,clock,calls=readonly_rpc_transport(monkeypatch);events=[]
    backend=module.RosBackend(transport,lambda *event:events.append(event));backend.connect();backend.release()
    calls[-1][1].resolve(False,'RELEASE_QUEUED');deadline=backend._release_deadline
    for _ in range(4):
        backend.poll();clock[0]+=1.01;transport._poll_trajectory_retry()
        assert backend._release_deadline==deadline and not backend.released
    clock[0]=deadline;backend.poll()
    assert not backend.released and backend._release_timeout_reported
    assert [x[0] for x in calls].count('disable_and_release')==1
    assert 'RELEASE_UNCONFIRMED' in events[-1][1]


@pytest.mark.parametrize('cleanup,released',[
    (' cleanup=RELEASED_IDLE',True),(' cleanup=RELEASED_FAULT',True),('',False),
    (' cleanup=UNCONFIRMED',False),(' cleanup=NOT_ACQUIRED',False)])
@pytest.mark.parametrize('operation,prefix,service',[
    ('auto','BASELINE_FAILED:','auto_calibrate_and_test'),
    ('manual','MANUAL_CONTROL_FAILED:','prepare_manual_control')])
def test_auto_failed_preserves_error_and_only_confirmed_release_allows_close(cleanup,released,operation,prefix,service):
    from manual_panel_ros_backend import RosBackend
    failure=prefix+' 原任务故障'+cleanup
    class Transport:
        status_max_age_ms=1000
        def __init__(self):self.calls=[];self.closed=False
        def request(self,service,callback):
            self.calls.append(service)
            callback(NS(success=service in ('auto_calibrate_and_test','prepare_manual_control'),
                message='BASELINE_ACCEPTED' if service in ('auto_calibrate_and_test','prepare_manual_control') else failure))
        def close(self):self.closed=True
    transport=Transport();events=[];backend=RosBackend(transport,lambda *event:events.append(event))
    backend.connect()
    request=backend.request_calibration_and_test if operation=='auto' else backend.request_prepare_manual_control
    assert request();backend.poll()
    assert events[-1]==('failed',failure)
    assert backend.operation is None and not backend.admitting
    assert backend.released is released
    assert backend.close() is released and transport.closed is released
    assert transport.calls==[service,'baseline_status' if operation=='auto' else 'manual_control_status']


def test_first_goal_waits_for_cpp_ready_without_sending_or_resetting_deadline(monkeypatch):
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    t.request=lambda name,cb:pending.append((name,cb))
    from manual_panel_ros_backend import RosTransport
    assert RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    deadline=t._submission['retry_deadline']
    t._poll_trajectory_retry();name,installed=pending.pop(0)
    assert name=='set_bench_permit'
    installed(NS(success=True,message='QUEUED'))
    t.latest_state.management_epoch+=1;t.latest_status.management_epoch+=1
    t.latest_state.state_seq+=1
    assert calls==[]
    t._poll_trajectory_retry()
    name,cb=pending.pop(0)
    assert name=='bootstrap_status'
    cb(NS(success=False,message='ENABLED_HOLD_PENDING'))
    assert calls==[]
    clock[0]+=.1;t.state_at=t.status_at=clock[0]
    t._poll_trajectory_retry();pending.pop(0)[1](NS(success=True,message='ENABLED_HOLD'))
    assert len(calls)==1 and t._submission['retry_deadline']==deadline


@pytest.mark.parametrize('failure',['owner','stale','pending'])
def test_preflight_never_sends_on_owner_drift_stale_or_unready(failure,monkeypatch):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    t.request=lambda name,cb:pending.append((name,cb))
    assert RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    deadline=t._submission['retry_deadline']
    t._poll_trajectory_retry();name,installed=pending.pop(0)
    assert name=='set_bench_permit'
    installed(NS(success=True,message='QUEUED'))
    t.latest_state.management_epoch+=1;t.latest_status.management_epoch+=1
    t.latest_state.state_seq+=1
    if failure=='owner':t.latest_state.session_id+=1
    if failure=='stale':clock[0]+=.5
    t._poll_trajectory_retry()
    pending.pop(0)[1](NS(success=failure!='pending',message='ENABLED_HOLD'))
    assert not calls
    clock[0]=deadline+.001
    t._poll_trajectory_retry()
    assert t._submission is None and events[-1][0]=='failed' and not calls


@pytest.mark.parametrize('tag,confirmed',[
 ('EVIDENCE_INCOMPLETE;RELEASED_IDLE',True),
 ('EVIDENCE_INCOMPLETE;RELEASED_FAULT',True),
 ('EVIDENCE_INCOMPLETE;RELEASE_UNCONFIRMED',False),
 ('EVIDENCE_INCOMPLETE;RELEASE_PENDING',False),
 ('prefix EVIDENCE_INCOMPLETE;RELEASED_IDLE',False)])
def test_release_compound_tag_requires_explicit_intent_and_exact_authority(tag,confirmed):
    from manual_panel_ros_backend import RosBackend
    calls=[];events=[]
    t=NS(request=lambda name,cb:(calls.append(name),cb(NS(success=False,message='RELEASE_QUEUED' if name=='disable_and_release' else tag))),close=lambda:None)
    b=RosBackend(t,lambda *x:events.append(x));b.connect();b.release();b.poll()
    assert b.released is confirmed
    if confirmed:assert events[-1]==('released',tag) and b.close()
    other=RosBackend(t,lambda *x:events.append(x));other.operation='auto';other.poll()
    assert not other.released and other.operation=='auto'


@pytest.mark.parametrize('confirmed_tag', ['EVIDENCE_INCOMPLETE;RELEASED_IDLE',
                                          'EVIDENCE_INCOMPLETE;RELEASED_FAULT'])
def test_late_readonly_release_confirmation_unlocks_close_without_resending_d10(monkeypatch, confirmed_tag):
    import manual_panel_ros_backend as m
    clock=[1.];monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    calls=[];events=[];reply=['RELEASE_PENDING']
    t=NS(request=lambda name,cb:(calls.append(name),cb(NS(success=False,message='RELEASE_QUEUED' if name=='disable_and_release' else reply[0]))),close=lambda:None)
    b=m.RosBackend(t,lambda *x:events.append(x));b.release();clock[0]=22.;b.poll()
    assert not b.released and any(x[0]=='failed'for x in events)
    reply[0]=confirmed_tag
    b.poll()
    assert b.released and b.close()
    assert calls.count('disable_and_release')==1 and calls[-1]=='bootstrap_status'
    assert any('deadline expired'in str(x)for x in events)


def test_fault_release_retires_owner_without_restoring_motion_authority():
    from manual_panel_ros_backend import RosBackend
    events=[]
    t=NS(request=lambda name,cb:cb(NS(success=True,message='RELEASE_QUEUED' if name=='disable_and_release' else 'RELEASED_FAULT')))
    b=RosBackend(t,lambda *x:events.append(x));b.manual_ready=True;b.admitting=True;b.release();b.poll()
    assert b.released and not b.manual_ready and not b.admitting
    assert events[-1]==('released','RELEASED_FAULT')


def test_pre_management_readonly_release_cannot_complete_inflight_release_and_emits_once():
    from manual_panel_ros_backend import RosBackend
    calls=[];events=[];callbacks=[]
    t=NS(request=lambda name,cb:(calls.append(name),callbacks.append(cb)))
    b=RosBackend(t,lambda *x:events.append(x));b.poll();b.release()
    callbacks.pop(0)(NS(success=True,message='RELEASED_IDLE'))
    assert not b.released and b.pending
    callbacks.pop(0)(NS(success=False,message='RELEASE_QUEUED'))
    b.poll()
    assert calls==['bootstrap_status','disable_and_release','bootstrap_status']
    callbacks.pop(0)(NS(success=True,message='RELEASED_IDLE'))
    assert b.released
    b.poll();callbacks.pop(0)(NS(success=True,message='RELEASED_IDLE'))
    assert sum(x[0]=='released' for x in events)==1


@pytest.mark.parametrize('operation,prefix', [('manual', 'MANUAL_CONTROL_FAILED:'),
                                           ('auto', 'BASELINE_FAILED:')])
def test_cpp_failed_cleanup_late_readonly_release_retires_owner_once(operation, prefix):
    from manual_panel_ros_backend import RosBackend
    replies = [NS(success=False, message=prefix + ' 原故障 cleanup=UNCONFIRMED'),
               NS(success=False, message='EVIDENCE_INCOMPLETE;RELEASE_PENDING'),
               NS(success=False, message='EVIDENCE_INCOMPLETE;RELEASED_IDLE'),
               NS(success=False, message='EVIDENCE_INCOMPLETE;RELEASED_IDLE')]
    calls, events = [], []
    transport = NS(request=lambda name, cb: (calls.append(name), cb(replies.pop(0))),
                   close=lambda: calls.append('close'))
    backend = RosBackend(transport, lambda *event: events.append(event))
    backend.operation, backend.admitting, backend.manual_ready = operation, True, True
    backend.poll()
    assert backend.operation is None and not backend.admitting and not backend.manual_ready
    assert not backend.released and events[-1][0] == 'failed'
    backend.poll()
    assert not backend.released
    backend.poll()
    backend.poll()
    assert backend.released and backend.close()
    assert sum(event[0] == 'released' for event in events) == 1
    assert events[0] == ('failed', prefix + ' 原故障 cleanup=UNCONFIRMED')
    assert 'disable_and_release' not in calls


@pytest.mark.parametrize('cleanup', ['cleanup=NOT_ACQUIRED', '', 'cleanup=UNCONFIRMED suffix'])
def test_unconfirmed_cleanup_intent_requires_exact_cpp_terminal_suffix(cleanup):
    from manual_panel_ros_backend import RosBackend
    replies = [NS(success=False, message='MANUAL_CONTROL_FAILED: 拒绝 ' + cleanup),
               NS(success=False, message='EVIDENCE_INCOMPLETE;RELEASED_IDLE')]
    events = []
    backend = RosBackend(NS(request=lambda name, cb: cb(replies.pop(0))),
                         lambda *event: events.append(event))
    backend.operation = 'manual'
    backend.poll()
    backend.poll()
    assert not backend.released and not any(event[0] == 'released' for event in events)


def extension_transport(monkeypatch):
    transport, trajectory, clock, calls, pending, events = renewal_transport(monkeypatch)
    class Future:
        def add_done_callback(self, callback): self.callback = callback
        def result(self): return self.value
        def cancel(self): self.cancelled = True
    future = Future()
    requests = []
    transport.extension_type = NS(Request=lambda: NS())
    transport.extension_client = NS(service_is_ready=lambda: True,
        call_async=lambda request: (requests.append(request) or future))
    transport.goal = NS(goal_id=NS(uuid=[2]*16), cancel_goal_async=lambda: calls.append('cancel'))
    transport._submission = dict(stage='active', binding=((1,)*16, 201, 7, 8, 9, 10))
    transport.latest_state.core_state = transport.latest_status.core_state = 5
    return transport, trajectory, clock, calls, events, requests, future


def test_extension_receipt_only_after_installed_and_input_is_copied(monkeypatch):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    assert transport.extend_trajectory(path, 1, 20000000, lambda *event: events.append(event))
    assert not transport.extend_trajectory(path, 1, 20000000, lambda *event: events.append(event))
    path.joint_names[0] = 'mutated'
    assert requests[0].trajectory.joint_names == ['Joint1']
    assert requests[0].goal_id.uuid == [2]*16 and requests[0].motion_epoch == 10
    assert events == []
    future.value = NS(accepted=True, version=2, phase_ns=40, trajectory_digest=[3]*32, reason='')
    future.callback(future)
    assert events == [('trajectory_extended', dict(version=2, phase_ns=40, trajectory_digest=[3]*32))]


def test_real_ros_extension_receipt_can_be_recorded_as_json_without_cancel(monkeypatch):
    import json
    from rclpy.serialization import serialize_message, deserialize_message
    from dummy_motion_interfaces.srv import ExtendJointTrajectory
    transport,path,clock,calls,events,requests,future=extension_transport(monkeypatch)
    def record(kind,value):
        events.append((kind,json.loads(json.dumps(value))))
    assert transport.extend_trajectory(path,0,20,record)
    response=ExtendJointTrajectory.Response(accepted=True,version=1,phase_ns=60_000_000,
                                           trajectory_digest=list(range(32)),reason='TAIL_INSTALLED')
    future.value=deserialize_message(serialize_message(response),ExtendJointTrajectory.Response)
    future.callback(future)
    assert events==[('trajectory_extended',dict(version=1,phase_ns=60_000_000,
                                              trajectory_digest=list(range(32))))]
    assert calls==[]


@pytest.mark.parametrize('retire', ['timeout', 'cancel', 'terminal', 'identity'])
def test_extension_late_reply_cannot_confirm_retired_or_foreign_goal(monkeypatch, retire):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    assert transport.extend_trajectory(path, 1, 20, lambda *event: events.append(event))
    if retire == 'timeout':
        clock[0] += 2.
        transport._poll_trajectory_retry()
        assert calls == ['cancel'] and events[0][0] == 'failed'
    elif retire == 'cancel': transport.cancel()
    elif retire == 'terminal': transport._submission = None
    else: transport.latest_state.session_id += 1
    future.value = NS(accepted=True, version=2, phase_ns=40, trajectory_digest=[3]*32, reason='')
    future.callback(future)
    assert not any(event[0] == 'trajectory_extended' for event in events)


def test_extension_rejection_preserves_response_without_accepted_event(monkeypatch):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    assert transport.extend_trajectory(path, 4, 20, lambda *event: events.append(event))
    future.value = NS(accepted=False, version=5, phase_ns=40, trajectory_digest=[0]*32, reason='PREFIX_CHANGED')
    future.callback(future)
    assert events == [('trajectory_extension_rejected', dict(reason='PREFIX_CHANGED', version=5, phase_ns=40))]


@pytest.mark.parametrize('invalid', ['stale', 'foreign', 'hold', 'no_permit'])
def test_extension_requires_fresh_same_submission_running_identity(monkeypatch, invalid):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    if invalid == 'stale': clock[0] += .3
    elif invalid == 'foreign':
        transport.latest_state.session_id += 1
        transport.latest_status.session_id += 1
    elif invalid == 'hold': transport.latest_state.core_state = 4
    else: transport.latest_status.permit_remaining_us = 0
    assert not transport.extend_trajectory(path, 0, 20, lambda *event: events.append(event))
    assert requests == []


def test_extension_close_retires_future_and_late_reply_is_silent(monkeypatch):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    transport.node.destroy_timer = lambda _: None
    transport.node.destroy_client = lambda _: None
    transport.node.destroy_subscription = lambda _: None
    transport.action.destroy = lambda: None
    transport.retry_timer = transport.parameter_client = None
    transport.clients = {}; transport.subscriptions = []
    assert transport.extend_trajectory(path, 0, 20, lambda *event: events.append(event))
    transport.close()
    future.value = NS(accepted=True, version=1, phase_ns=40, trajectory_digest=[3]*32, reason='')
    future.callback(future)
    assert future.cancelled and events == []


def test_backend_extension_is_only_thin_transport_forwarding():
    from manual_panel_ros_backend import RosBackend
    calls = []
    transport = NS(status_max_age_ms=1000,
        extend_trajectory=lambda *args: (calls.append(args) or True))
    callback = lambda *event: None
    backend = RosBackend(transport, callback)
    path = NS(points=[])
    assert not backend.extend_trajectory(path, 0, 20)
    backend.connect()
    assert backend.extend_trajectory(path, 0, 20)
    assert calls == [(path, 0, 20, callback)]
    backend.released = True
    assert not backend.extend_trajectory(path, 1, 40)


def test_extension_cached_reply_waits_for_fresh_dds_without_resetting_deadline(monkeypatch):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    assert transport.extend_trajectory(path, 0, 20, lambda *event: events.append(event))
    deadline = transport._extension['deadline']
    clock[0] += .3
    future.value = NS(accepted=True, version=1, phase_ns=40, trajectory_digest=[3]*32, reason='')
    future.callback(future)
    assert events == [] and calls == [] and transport._extension['deadline'] == deadline
    transport._poll_trajectory_retry()
    assert events == [] and len(requests) == 1
    transport.state_at = transport.status_at = clock[0]
    transport._poll_trajectory_retry(); transport._poll_trajectory_retry(); future.callback(future)
    assert len(events) == 1 and events[0][0] == 'trajectory_extended'
    assert calls == [] and len(requests) == 1


def test_extension_stale_cache_without_reply_waits_then_fresh_reply_confirms(monkeypatch):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    assert transport.extend_trajectory(path, 0, 20, lambda *event: events.append(event))
    clock[0] += .3; transport._poll_trajectory_retry()
    assert events == [] and calls == []
    transport.state_at = transport.status_at = clock[0]
    future.value = NS(accepted=True, version=1, phase_ns=40, trajectory_digest=[3]*32, reason='')
    future.callback(future)
    assert len(events) == 1 and events[0][0] == 'trajectory_extended'


def test_extension_persistent_stale_uses_original_deadline_and_cancels_once(monkeypatch):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    assert transport.extend_trajectory(path, 0, 20, lambda *event: events.append(event))
    clock[0] += .3; transport._poll_trajectory_retry()
    assert events == []
    future.value = NS(accepted=True, version=1, phase_ns=40, trajectory_digest=[3]*32, reason='')
    future.callback(future)
    clock[0] = 12.; transport._poll_trajectory_retry(); transport._poll_trajectory_retry()
    transport.state_at = transport.status_at = clock[0]; future.callback(future)
    assert calls == ['cancel'] and len(events) == 1 and events[0][0] == 'failed'
    assert 'DEADLINE_EXPIRED' in events[0][1] and 'state_age_ms=' in events[0][1]
    assert len(requests) == 1


@pytest.mark.parametrize('change', ['identity', 'permission', 'uuid'])
def test_extension_fresh_identity_or_permission_change_fails_immediately(monkeypatch, change):
    transport, path, clock, calls, events, requests, future = extension_transport(monkeypatch)
    assert transport.extend_trajectory(path, 0, 20, lambda *event: events.append(event))
    if change == 'identity': transport.latest_state.session_id += 1
    elif change == 'permission': transport.latest_status.real_motion_permitted = False
    else: transport.goal.goal_id.uuid[0] += 1
    transport._poll_trajectory_retry()
    assert calls == ['cancel'] and len(events) == 1 and events[0][0] == 'failed'
    assert ('UUID_CHANGED' if change == 'uuid' else 'IDENTITY_OR_PERMISSION_CHANGED') in events[0][1]


def test_explicit_goal_installs_once_then_waits_for_fresh_committed_permit(monkeypatch):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    assert RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    deadline=t._submission['retry_deadline']
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    assert name=='set_bench_permit' and not calls
    cb(NS(success=True,message='QUEUED'))
    for _ in range(2):
        t._poll_trajectory_retry();name,cb=pending.pop(0)
        assert name=='bootstrap_status'
        cb(NS(success=True,message='ENABLED_HOLD'))
        assert not calls  # 旧ready及排队回执均不能确认新许可。
    t.latest_state.management_epoch+=1;t.latest_status.management_epoch+=1
    t.latest_state.state_seq+=1;clock[0]+=.02;t.state_at=t.status_at=clock[0]
    t._poll_trajectory_retry();name,cb=pending.pop(0);cb(NS(success=True,message='ENABLED_HOLD'))
    assert len(calls)==1 and t._submission['retry_deadline']==deadline
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    assert name=='last_goal_rejection'
    cb(NS(success=True,message='RENEWAL_PENDING:'+bytes(calls[0][1]['goal_uuid'].uuid).hex()))
    t.latest_state.state_seq+=1;clock[0]+=.02;t.status_at=clock[0]
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    assert name=='bootstrap_status'
    cb(NS(success=True,message='ENABLED_HOLD'))
    assert len(calls)==2 and t._submission['retry_deadline']==deadline


@pytest.mark.parametrize('change',['cancel','owner','deadline','denied'])
def test_permit_install_late_callback_never_sends_or_reinstalls(monkeypatch,change):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    assert name=='set_bench_permit'
    if change=='cancel':t.cancel_pending_submission()
    elif change=='owner':t.latest_state.session_id+=1;t.latest_status.session_id+=1
    elif change=='deadline':clock[0]+=2.01
    cb(NS(success=change!='denied',message='QUEUED' if change!='denied' else 'POLICY_DENIED'))
    t._poll_trajectory_retry()
    assert not calls and not pending and t._submission is None
    assert events[-1][0]=='failed'


@pytest.mark.parametrize('missing',['management','state_seq','status_at','state_age','status_age'])
def test_new_permit_needs_independent_fresh_dual_stream_watermarks(monkeypatch,missing):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    t.latest_status.real_motion_permitted=False;t.latest_status.permit_remaining_us=0
    RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    t._poll_trajectory_retry();name,install=pending.pop(0)
    assert name=='set_bench_permit'  # 旧许可已过期仍可显式申请本次新窗口。
    install(NS(success=True,message='QUEUED'))
    old=t._submission.copy();clock[0]+=.01
    t.latest_state.management_epoch+=1;t.latest_status.management_epoch+=1
    t.latest_state.state_seq+=1;t.state_at=t.status_at=clock[0]
    t.latest_status.real_motion_permitted=True;t.latest_status.permit_remaining_us=900000000
    if missing=='management':t.latest_state.management_epoch=t.latest_status.management_epoch=old['management']
    elif missing=='state_seq':t.latest_state.state_seq=old['state_seq']
    elif missing=='status_at':t.status_at=old['status_at']
    elif missing=='state_age':t.state_at=clock[0]-.3
    elif missing=='status_age':t.status_at=clock[0]-1.
    t._poll_trajectory_retry();pending.pop(0)[1](NS(success=True,message='ENABLED_HOLD'))
    assert not calls
    clock[0]=old['retry_deadline']+.01;t._poll_trajectory_retry()
    install(NS(success=True,message='QUEUED'))
    assert t._submission is None and not pending and not calls


def test_new_permit_waits_for_readiness_before_single_write_under_original_deadline(monkeypatch):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    deadline=t._submission['retry_deadline']
    for _ in range(2):
        t._poll_trajectory_retry();name,cb=pending.pop(0)
        assert name=='bootstrap_status'
        cb(NS(success=False,message='LEASE_RENEWAL_PENDING'))
        assert not calls and t._submission['retry_deadline']==deadline
        clock[0]+=.01;t.state_at=t.status_at=clock[0]
    t._poll_trajectory_retry();name,old_ready=pending.pop(0)
    old_ready(NS(success=True,message='ENABLED_HOLD'))
    t._poll_trajectory_retry();name,queued=pending.pop(0)
    assert name=='set_bench_permit'
    queued(NS(success=True,message='QUEUED'))
    old_ready(NS(success=True,message='ENABLED_HOLD'))
    assert t._submission['stage']=='preflight'
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    assert name=='bootstrap_status'
    cb(NS(success=True,message='ENABLED_HOLD'))
    assert not calls and t._submission['retry_deadline']==deadline


def test_unconsumed_permit_reuse_sends_without_new_management_commit(monkeypatch):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    management=t.latest_state.management_epoch
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    assert name=='set_bench_permit'
    cb(NS(success=True,message='PERMIT_REUSED'))
    assert len(calls)==1 and t.latest_state.management_epoch==management
    assert not pending


def test_not_queued_permit_rechecks_readiness_without_replaying_accepted_write(monkeypatch):
    """两次RPC之间准入被占用：只重试明确未入队的请求，仍等真实许可提交。"""
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    assert RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    deadline=t._submission['retry_deadline']
    t._poll_trajectory_retry();name,rejected=pending.pop(0)
    assert name=='set_bench_permit'
    rejected(NS(success=False,message='PERMIT_NOT_QUEUED'))
    assert t._submission is not None and not events and not calls
    assert t._submission['retry_deadline']==deadline
    t._poll_trajectory_retry();name,ready=pending.pop(0)
    assert name=='bootstrap_status'
    ready(NS(success=False,message='LEASE_RENEWAL_PENDING'))
    clock[0]+=.02;t.state_at=t.status_at=clock[0]
    t._poll_trajectory_retry();name,ready=pending.pop(0)
    assert name=='bootstrap_status'
    ready(NS(success=True,message='ENABLED_HOLD_PERMIT_REQUIRED'))
    t._poll_trajectory_retry();name,accepted=pending.pop(0)
    assert name=='set_bench_permit'
    accepted(NS(success=True,message='QUEUED'))
    for _ in range(2):
        t._poll_trajectory_retry();name,confirmed=pending.pop(0)
        assert name=='bootstrap_status'  # 已入队后只查询，绝不重发写请求。
        confirmed(NS(success=True,message='ENABLED_HOLD'))
        assert not calls
    t.latest_state.management_epoch+=1;t.latest_status.management_epoch+=1
    t.latest_state.state_seq+=1;clock[0]+=.02;t.state_at=t.status_at=clock[0]
    t._poll_trajectory_retry();name,confirmed=pending.pop(0)
    assert name=='bootstrap_status'
    confirmed(NS(success=True,message='ENABLED_HOLD'))
    assert len(calls)==1 and t._submission['retry_deadline']==deadline


@pytest.mark.parametrize('change',['cancel','owner','deadline','stale'])
def test_not_queued_permit_cannot_outlive_original_binding_or_deadline(monkeypatch,change):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    deadline=t._submission['retry_deadline']
    t._poll_trajectory_retry();name,rejected=pending.pop(0)
    assert name=='set_bench_permit'
    rejected(NS(success=False,message='PERMIT_NOT_QUEUED'))
    assert t._submission is not None
    if change=='cancel':t.cancel_pending_submission()
    elif change=='owner':
        t.latest_state.session_id+=1;t.latest_status.session_id+=1
    elif change=='deadline':clock[0]=deadline
    else:clock[0]+=.3
    t._poll_trajectory_retry()
    while pending:
        name,ready=pending.pop(0)
        assert name=='bootstrap_status'
        ready(NS(success=True,message='ENABLED_HOLD'))
    assert not calls
    if t._submission is not None:
        assert t._submission['retry_deadline']==deadline
        clock[0]=deadline;t._poll_trajectory_retry()
    assert t._submission is None and events[-1][0]=='failed'


@pytest.mark.parametrize('bad',['short','aged_budget','no_permit','stale','owner'])
def test_reused_permit_requires_fresh_identity_and_whole_trajectory_budget(monkeypatch,bad):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    initial_permit_readiness(t,pending)
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    if bad=='short':t.latest_status.permit_remaining_us=999999
    elif bad=='aged_budget':
        t.latest_status.permit_remaining_us=1010000;clock[0]+=.02
    elif bad=='no_permit':t.latest_status.real_motion_permitted=False
    elif bad=='stale':clock[0]+=.3
    elif bad=='owner':t.latest_status.session_id+=1
    cb(NS(success=True,message='PERMIT_REUSED'))
    assert not calls and t._submission is None and events[-1][0]=='failed'


def test_permit_required_marker_only_allows_install_not_direct_send(monkeypatch):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    t.latest_status.real_motion_permitted=False;t.latest_status.permit_remaining_us=0
    RosTransport.submit_trajectory(t,path,lambda *e:events.append(e))
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    assert name=='bootstrap_status'
    cb(NS(success=True,message='ENABLED_HOLD_PERMIT_REQUIRED'))
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    assert name=='set_bench_permit' and not calls
    cb(NS(success=True,message='QUEUED'))
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    cb(NS(success=True,message='ENABLED_HOLD_PERMIT_REQUIRED'))
    assert not calls and t._submission['stage']=='preflight'


@pytest.mark.parametrize('remaining,accepted',[(40000000,False),(70000000,True)])
def test_short_mother_goal_requires_frozen_whole_task_permit(monkeypatch,remaining,accepted):
    from manual_panel_ros_backend import RosTransport
    t,path,clock,calls,pending,events=renewal_transport(monkeypatch)
    t.latest_status.permit_remaining_us=remaining
    RosTransport.submit_trajectory(t,path,lambda *e:events.append(e),task_budget_s=60.)
    initial_permit_readiness(t,pending)
    t._poll_trajectory_retry();name,cb=pending.pop(0)
    cb(NS(success=True,message='PERMIT_REUSED'))
    assert bool(calls) is accepted


def owned_node_fakes(monkeypatch, fail_entities=False):
    """只安装叶层Fake；绝不创建ROS节点或DDS participant。"""
    import manual_panel_ros_backend as module
    calls=[];nodes=[];contexts=[]
    class Context:
        def __init__(self):contexts.append(self)
        def init(self, **kw):calls.append(('context_init',kw))
        def try_shutdown(self):calls.append(('context_shutdown',self))
        def destroy(self):calls.append(('context_destroy',self))
    class Node:
        def __init__(self, **kw):self.kw=kw;self.callbacks=[];self.count=0;nodes.append(self)
        def create_client(self,*args):
            if fail_entities:raise RuntimeError('entity construction failure')
            return object()
        def create_timer(self,*args):return object()
        def create_subscription(self,typ,name,cb,qos):self.callbacks.append(cb);return object()
        def count_services(self,name):return self.count
        def count_publishers(self,name):return self.count
        def destroy_node(self):calls.append(('node_destroy',self))
        def destroy_timer(self,*a):pass
        def destroy_client(self,*a):pass
        def destroy_subscription(self,*a):pass
    class Executor:
        def __init__(self,**kw):self.kw=kw
        def add_node(self,node):calls.append(('executor_add',node))
        def remove_node(self,node):calls.append(('executor_remove',node))
        def spin_once(self,**kw):calls.append(('spin',kw))
        def shutdown(self,**kw):calls.append(('executor_shutdown',kw))
    modules={
        'std_srvs.srv':NS(Trigger=object), 'rcl_interfaces.srv':NS(GetParameters=object),
        'rcl_interfaces.msg':NS(ParameterType=NS(PARAMETER_INTEGER=2)),
        'dpsdk_interfaces.msg':NS(DummyDeviceStatusV2=object,DummyJointStateV2=object),
        'control_msgs.action':NS(FollowJointTrajectory=object),
        'rclpy.action':NS(ActionClient=lambda *a:NS(destroy=lambda:None)),
        'unique_identifier_msgs.msg':NS(UUID=object),
        'dummy_motion_interfaces.srv':NS(ExtendJointTrajectory=object),
        'rclpy.context':NS(Context=Context), 'rclpy.executors':NS(SingleThreadedExecutor=Executor),
        'rclpy':NS(create_node=lambda name,**kw:Node(**kw)),
    }
    for name,value in modules.items():monkeypatch.setitem(sys.modules,name,value)
    shared=NS(context=NS(get_domain_id=lambda:77),get_namespace=lambda:'/manual_control',
        count_services=lambda _:1,count_publishers=lambda _:1)
    return module,shared,calls,nodes,contexts


def test_owned_control_context_same_node_preflight_and_idempotent_close(monkeypatch):
    module,shared,calls,nodes,contexts=owned_node_fakes(monkeypatch)
    seen=[];t=module.RosTransport(shared,lambda v:seen.append(v),lambda v:seen.append(v),own_control_node=True)
    assert t.node is nodes[0] and t.node is not shared and t.owns_control_node
    assert nodes[0].kw['context'] is contexts[0]
    assert nodes[0].kw['namespace']=='/manual_control'
    assert calls[0][0]=='context_init' and calls[0][1]['domain_id']==77
    t.assert_no_existing_owner(timeout=0)
    t.pump();assert ('spin',dict(timeout_sec=0)) in calls
    nodes[0].count=1;t.close();t.close()
    for cb in nodes[0].callbacks:cb(NS(boot_epoch=1,state_seq=99))
    assert not seen
    assert sum(name=='node_destroy' for name,*rest in calls)==1
    assert sum(name=='context_shutdown' for name,*rest in calls)==1
    assert ('node_destroy',shared) not in calls


def test_owned_node_partial_entity_failure_cleans_only_owned_context(monkeypatch):
    module,shared,calls,nodes,contexts=owned_node_fakes(monkeypatch,True)
    with pytest.raises(RuntimeError,match='entity construction failure'):
        module.RosTransport(shared,lambda v:None,lambda v:None,own_control_node=True)
    assert ('node_destroy',nodes[0]) in calls
    assert ('context_shutdown',contexts[0]) in calls
    assert ('node_destroy',shared) not in calls


@pytest.mark.parametrize('bad',['none','stale','same_seq','old_state','old_status','identity','permit','uncommitted','core','fault','ok'])
def test_owned_gui_can_close_requires_authority_and_fresh_advancing_double_disable(monkeypatch,bad):
    import manual_panel_ros_backend as module
    monkeypatch.setattr(module.time,'monotonic',lambda:10.)
    t=object.__new__(module.RosTransport);t.owns_control_node=True
    fields=dict(device_id=[1,2],profile_id=201,boot_epoch=3,reference_epoch=4,session_id=7)
    t.latest_state=NS(**fields,state_seq=10);t.latest_status=NS(**fields)
    t.state_at=t.status_at=9.;t.feedback_max_age_ms=t.status_max_age_ms=500
    checkpoint=t.release_checkpoint()
    t.latest_state=NS(**dict(fields,session_id=0),state_seq=11,core_state=2,first_failure_reason=0)
    t.latest_status=NS(**dict(fields,session_id=0),core_state=2,first_failure_reason=0,evidence_lost=False,
        release_committed=True,real_motion_permitted=False,permit_remaining_us=0)
    t.state_at=t.status_at=10.1
    monkeypatch.setattr(module.time,'monotonic',lambda:10.2)
    if bad=='none':t.latest_status=None
    elif bad=='stale':t.state_at=9.
    elif bad=='same_seq':t.latest_state.state_seq=10
    elif bad=='old_state':t.state_at=9.9
    elif bad=='old_status':t.status_at=9.9
    elif bad=='identity':t.latest_status.boot_epoch=99
    elif bad=='permit':t.latest_status.permit_remaining_us=1
    elif bad=='uncommitted':t.latest_status.release_committed=False
    elif bad=='core':t.latest_state.core_state=4
    elif bad=='fault':t.latest_status.evidence_lost=True
    b=module.RosBackend(t,lambda *v:None);b._release_observation=checkpoint
    assert not b.can_close
    b.released=True
    assert b.can_close is (bad=='ok')


def test_default_transport_never_owns_or_destroys_callers_node(monkeypatch):
    module,shared,calls,nodes,contexts=owned_node_fakes(monkeypatch)
    # 外层拥有且自行spin；使用叶层Fake Node，不导入真实ROS。
    node=type(nodes[0])() if nodes else None
    if node is None:
        import rclpy
        node=rclpy.create_node('external')
    t=module.RosTransport(node,lambda v:None,lambda v:None)
    t.pump();t.close()
    assert not t.owns_control_node and not contexts
    assert not any(name in ('node_destroy','context_shutdown','executor_shutdown','spin') for name,*rest in calls)


@pytest.mark.parametrize('count',[1,2])
def test_owned_fresh_node_existing_owner_stays_refused_without_any_d10(monkeypatch,count):
    module,shared,calls,nodes,contexts=owned_node_fakes(monkeypatch)
    t=module.RosTransport(shared,lambda v:None,lambda v:None,own_control_node=True)
    nodes[0].count=count
    with pytest.raises(RuntimeError,match='已有ROS控制链'):t.assert_no_existing_owner(timeout=0)
    replies=[];t.request('disable_and_release',replies.append)
    assert len(replies)==1 and isinstance(replies[0],RuntimeError)
    t.close()


def test_owned_release_proof_survives_own_process_shutdown_delay(monkeypatch):
    import manual_panel_ros_backend as module
    observed=[True]
    t=NS(owns_control_node=True,release_observed=lambda checkpoint:observed[0],close=lambda:None)
    b=module.RosBackend(t,lambda *v:None);b.released=True
    assert b.can_close
    # PGID确认退出可能花6秒；旧子链已停后不能要求再收到新消息。
    observed[0]=False
    assert b.can_close and b.close()


def test_owned_authoritative_release_without_fresh_double_times_out_original_deadline(monkeypatch):
    import manual_panel_ros_backend as module
    clock=[10.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    requests=[];events=[]
    t=NS(owns_control_node=True,release_checkpoint=lambda:{'at':clock[0]},release_observed=lambda checkpoint:False,
        request=lambda name,cb:requests.append((name,cb)))
    b=module.RosBackend(t,lambda *v:events.append(v));b.release()
    requests[-1][1](NS(success=False,message='RELEASE_QUEUED'))
    b.poll();requests[-1][1](NS(success=True,message='RELEASED_IDLE'))
    assert b.released and not b.can_close
    deadline=b._release_deadline;clock[0]=deadline+.1
    b.poll();requests[-1][1](NS(success=True,message='RELEASED_IDLE'))
    assert any(kind=='failed' and 'RELEASE_UNCONFIRMED' in str(value) for kind,value in events)
    assert not b.can_close and not b.release()
    assert b._release_deadline==deadline
    assert [name for name,cb in requests].count('disable_and_release')==1
    assert [kind for kind,value in events].count('released')==1


def test_owned_manual_terminal_bounds_first_double_feedback_wait_without_d10(monkeypatch):
    import manual_panel_ros_backend as module
    clock=[10.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    requests=[];events=[]
    t=NS(owns_control_node=True,release_checkpoint=lambda:{'at':clock[0]},release_observed=lambda checkpoint:False,
        request=lambda name,cb:requests.append((name,cb)))
    b=module.RosBackend(t,lambda *v:events.append(v));b.operation='manual'
    b.poll();requests[-1][1](NS(success=True,message='MANUAL_CONTROL_RELEASED'))
    assert b._release_deadline==30. and not b.can_close
    clock[0]=31.;b.poll()
    assert any(k=='failed' and 'RELEASE_UNCONFIRMED' in str(v) for k,v in events)
    assert all(name!='disable_and_release' for name,cb in requests)
