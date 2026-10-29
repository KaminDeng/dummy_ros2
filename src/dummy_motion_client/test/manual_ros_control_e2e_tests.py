"""真实C++节点+合成ROS端点：手动Ready不提交基线轨迹，终止仍核释放；无设备。"""
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time
import yaml
os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%100))
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController, ListControllers
from control_msgs.action import FollowJointTrajectory
from dpsdk_interfaces.msg import DummyDeviceStatusV2, DummyJointStateV2
mode = sys.argv[2]
assert mode in ('external', 'stale', 'shutdown', 'timeout', 'controllers_late', 'controllers_arm_active', 'controllers_timeout', 'controllers_partial_active', 'ready_past_deadline', 'spent_permit', 'dds_gap')
controller_mode=mode.startswith('controllers_')
calibration_at=None
controller_states={'joint_state_broadcaster':'unconfigured' if controller_mode else 'inactive', 'arm_controller':'active' if mode=='controllers_arm_active' else 'inactive'}
rclpy.init(); node=Node('manual_control_software_peer')
root=pathlib.Path(tempfile.mkdtemp(prefix='manual-control-software-'))
acquired=holding=released=freeze=permit_consumed=False
calls={'calibrate':0,'release':0,'goal':0,'switch':0,'enable':0,'premature':0};seq=0
sp=node.create_publisher(DummyDeviceStatusV2,'/device_action_gate/device_status',10)
qp=node.create_publisher(DummyJointStateV2,'/device_action_gate/device_state',10)
def publish():
    global seq
    if freeze:return
    seq+=1;s=DummyDeviceStatusV2();q=DummyJointStateV2()
    s.boot_epoch=q.boot_epoch=23;s.device_id=q.device_id=[1]*16
    s.reference_epoch=q.reference_epoch=7;s.reference_kind=1;s.profile_id=q.profile_id=201
    s.configured_mask=q.configured_mask=63
    s.session_id=q.session_id=42 if acquired and not released else 0
    s.core_state=q.core_state=4 if holding and not released else 2
    s.real_motion_permitted=bool(acquired and not released and not permit_consumed);s.permit_remaining_us=5000000 if s.real_motion_permitted else 0
    s.release_committed=released;s.position_min_rad=[-1.0]*6;s.position_max_rad=[1.0]*6
    s.velocity_max_rad_s=[0.03]*6;s.acceleration_max_rad_s=[0.1]*6
    q.state_seq=seq;q.feedback_seq=[seq]*6;q.can_rx_age_us=[11000]*6
    q.position_valid_mask=q.velocity_valid_mask=q.raw_position_valid_mask=63
    q.hold_ready=bool(holding and not released)
    sp.publish(s);qp.publish(q)
def calibration(req,res):
    global acquired,calibration_at
    calibration_at=time.monotonic()
    calls['calibrate']+=1;acquired=True;res.success=True;res.message='CALIBRATION_ACCEPTED';return res
def reference(req,res):res.success=acquired and mode != 'timeout';res.message='REFERENCE_READY' if res.success else 'PENDING';return res
def enable(req,res):
    global holding
    calls['enable']+=1
    if controller_mode and any(v!='active' for v in controller_states.values()):calls['premature']+=1
    holding=True;res.success=True;res.message='HOLD_QUEUED';return res
def release(req,res):
    global released,freeze
    calls['release']+=1;released=True;freeze=False;res.success=False;res.message='RELEASE_QUEUED';return res
def bootstrap(req,res):res.success=holding or released;res.message='RELEASED_IDLE' if released else 'HOLD' if holding else 'READ_ONLY';return res
def hardware(req,res):res.ok=True;res.state.id=3;return res
def switch(req,res):
    calls['switch']+=1
    if controller_mode:
        if req.strictness!=SwitchController.Request.STRICT:calls['premature']+=1
        if any(v=='unconfigured' for v in controller_states.values()):calls['premature']+=1
    for name in req.activate_controllers:
        if controller_states[name]!='unconfigured' and not (mode=='controllers_partial_active' and name=='joint_state_broadcaster'):
            controller_states[name]='active'
    res.ok=True;return res
def list_controllers(req,res):
    from controller_manager_msgs.msg import ControllerState
    if controller_mode and mode!='controllers_timeout' and calibration_at is not None and time.monotonic()-calibration_at>=1.0:
        if controller_states['joint_state_broadcaster']=='unconfigured':controller_states['joint_state_broadcaster']='inactive'
    res.controller=[ControllerState(name=k,state=v) for k,v in controller_states.items()]
    return res
def goal(req):calls['goal']+=1;return GoalResponse.REJECT
services=[node.create_service(Trigger,'/device_action_gate/'+name,cb) for name,cb in (
 ('calibrate_manual_reference',calibration),('manual_reference_status',reference),('enable_hold',enable),
 ('disable_and_release',release),('bootstrap_status',bootstrap))]
services += [node.create_service(SetHardwareComponentState,'/controller_manager/set_hardware_component_state',hardware),
 node.create_service(SwitchController,'/controller_manager/switch_controller',switch),
 node.create_service(ListControllers,'/controller_manager/list_controllers',list_controllers)]
action=ActionServer(node,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory',execute_callback=lambda h:None,goal_callback=goal)
timer=node.create_timer(.02,publish)
(root/'raw').write_bytes(b'synthetic raw');(root/'timeline').write_text('synthetic timeline\n')
params={'joint_names':['Joint'+str(i) for i in range(1,7)],'lower_rad':[-1.0]*6,'upper_rad':[1.0]*6,
 'rest_rad':[0.0]*6,'velocity_limits_rad_s':[0.03]*6,'acceleration_limits_rad_s2':[0.1]*6,
 'report_path':str(root/'baseline.log'),'raw_evidence_path':str(root/'raw'),'goal_timeline_path':str(root/'timeline'),
 'total_timeout_s':3 if mode in ('timeout','controllers_timeout','controllers_partial_active','ready_past_deadline') else 20}
(root/'params.yaml').write_text(yaml.safe_dump({'manual_ros_baseline':{'ros__parameters':params}}))
log=open(root/'node.log','w');process=subprocess.Popen([sys.argv[1],'--ros-args','--params-file',str(root/'params.yaml')],stdout=log,stderr=log)
def call(name):
    client=node.create_client(Trigger,'/device_action_gate/'+name);deadline=time.monotonic()+5
    while not client.service_is_ready() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    assert client.service_is_ready(),(name,root)
    f=client.call_async(Trigger.Request())
    while not f.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    assert f.done(),(name,root)
    result=f.result();node.destroy_client(client);return result
def until(predicate,seconds=7):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        response=call('manual_control_status')
        if predicate(response):return response
    raise AssertionError((root,response.message))
try:
    assert not call('manual_control_status').success
    assert call('prepare_manual_control').success
    assert not call('prepare_manual_control').success
    assert not call('auto_calibrate_and_test').success
    if controller_mode:
        until(lambda r:calls['calibrate']>0)
        end=calibration_at+.75
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            status=call('manual_control_status')
            assert not status.message.startswith('MANUAL_CONTROL_READY'),(root,'Ready before spawner configured',calls)
            assert calls['enable']==calls['premature']==0,(root,'early controller activation/EnableHold',calls)
        if mode in ('controllers_timeout','controllers_partial_active'):
            terminal=until(lambda r:r.message.startswith('MANUAL_CONTROL_FAILED:') and 'cleanup=RELEASED_IDLE' in r.message)
            assert calls['enable']==calls['premature']==0,(root,calls)
            assert calls['release']>=1 and calls['goal']==0,(root,calls)
            print('PASS bounded controller readiness failure',mode,root)
            raise SystemExit(0)
    if mode == 'timeout':
        terminal=until(lambda r:r.message.startswith('MANUAL_CONTROL_FAILED:') and 'cleanup=RELEASED_IDLE' in r.message)
        assert '自动任务总期限超时' in terminal.message and calls['enable'] == calls['goal'] == 0, (root, terminal.message, calls)
        assert calls['release'] >= 1, (root, calls)
        print('PASS preparation remains bounded', root)
        raise SystemExit(0)
    ready=until(lambda r:r.success and r.message.startswith('MANUAL_CONTROL_READY'))
    if controller_mode:
        assert all(v=='active' for v in controller_states.values()) and calls['premature']==0,(root,calls,controller_states)
        assert calls['switch']==calls['enable']==1,(root,calls)
    assert calls['calibrate']==1 and calls['goal']==calls['release']==0,(calls,root)
    assert not call('baseline_status').success
    end=time.monotonic()+.3
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    assert calls['goal']==0
    if mode == 'dds_gap':
        freeze=True
        end=time.monotonic()+.45
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        freeze=False
        owner=until(lambda r:r.success and r.message.startswith('MANUAL_CONTROL_READY'))
        assert calls['release']==calls['goal']==0,(root,owner.message,calls)
    if mode == 'spent_permit':
        permit_consumed=True
        end=time.monotonic()+2.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        owner=call('manual_control_status')
        assert owner.success and owner.message.startswith('MANUAL_CONTROL_READY') and calls['release']==0, (root,owner.message,calls)
    if mode == 'ready_past_deadline':
        end = time.monotonic() + 3.5
        while time.monotonic() < end:rclpy.spin_once(node, timeout_sec=.02)
        still_ready = call('manual_control_status')
        assert still_ready.success and still_ready.message.startswith('MANUAL_CONTROL_READY'), (root, still_ready.message, calls)
        assert calls['release'] == calls['goal'] == 0, (root, calls)
    if mode in ('external','controllers_late','controllers_arm_active','ready_past_deadline','spent_permit','dds_gap'):
        call('disable_and_release')
        terminal=until(lambda r:r.success and r.message.startswith('MANUAL_CONTROL_RELEASED'))
    elif mode=='shutdown':
        process.send_signal(signal.SIGTERM)
        end=time.monotonic()+7
        while process.poll() is None and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        assert process.poll()==0,(root,calls)
    else:
        if mode=='stale':freeze=True
        terminal=until(lambda r:r.message.startswith('MANUAL_CONTROL_FAILED:') and 'cleanup=RELEASED_IDLE' in r.message)
    assert calls['release']>=1 and calls['goal']==0,(calls,root)
    if mode!='shutdown':assert not call('prepare_manual_control').success
    report=(root/'baseline.log').read_text()
    assert 'BASELINE_PASSED' not in report and 'GOAL_SENT' not in report,report
    print('PASS manual',mode,'no baseline trajectory; confirmed cleanup',root)
finally:
    if process.poll() is None:process.send_signal(signal.SIGTERM)
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:process.kill();process.wait()
    action.destroy();node.destroy_node();rclpy.shutdown();log.close()
