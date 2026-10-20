"""报告采集模式离线 SOURCE 测试；不启动 ROS、设备或授权运动。"""
##
# @file test_ros_hardware_deactivation_capture.py
# @brief 运动中hardware deactivation采集actor的可信见证/共享截止离线测试。
#
# 真实actor方法使用NS adapter、假goal handle/Future、模拟时钟与rclpy spin；不调用真实controller/hardware服务。witness/运动反馈人工构造，仅证明触发/拒绝与原错误保留。Host秒/ns/µs按接口分别使用，五秒预留不能扩大原goal deadline；设备Stop证明仍另需raw。
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest

spec = importlib.util.spec_from_file_location('stop_capture_actor', Path(__file__).resolve().parents[1] / 'ros_six_axis_movej.py')
actor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(actor)
UUID = 'ab' * 16

##
# @brief 构造boot7/session17/ref2/motion3/management4/profile201/mask63/run9，两条前进Source生产State及新鲜Run Status。
# @return 人工trusted witness字典，Source生产1000/2000µs不与Host950000/990000µs直接作同步比较。
def witness():
    root = dict(boot_epoch=7, session_id=17, reference_epoch=2, motion_epoch=3,
                management_epoch=4, profile_id=201, configured_mask=63, run_id=9)
    fields = dict(root, core_state=5, first_failure_reason=0)
    return dict(goal_id=UUID, after_begin_us=900000, root=root,
                states=[dict(when=950000, fields=dict({k:v for k,v in fields.items() if k != 'run_id'}, produced_at_us=1000)),
                        dict(when=990000, fields=dict({k:v for k,v in fields.items() if k != 'run_id'}, produced_at_us=2000))],
                status=dict(when=990000, fields=dict(fields, real_motion_permitted=1,
                                                     permit_remaining_us=1000000, attempt_id=9, attempt_kind=1, parent_run_id=0, final_state=0, evidence_lost=0)))

##
# @brief 断言actor有严格witness接口，传固定UUID和Host当前1000000µs验证，不请求服务。
# @param data 待验证的合成trusted witness字典，可能为None/被变异值。
# @return actor验证后的root身份字典；失败RuntimeError。
def validate(data):
    assert hasattr(actor, 'validate_hardware_deactivation_witness'), 'missing strict STOP capture witness gate'
    return actor.validate_hardware_deactivation_witness(data, UUID, 1000000)

##
# @brief 合成人工Run见证通过，返回run9；不证明真实设备正在运动。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_fresh_strong_running_witness():
    assert validate(witness())['run_id'] == 9

##
# @brief 缺/错goal、根身份旧、Host失鲜/未来、非Run、before_begin或Source不前进均RuntimeError；本例只直接验证见证门，不实际接服务。
# @param bad 八类witness故障分支名。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
@pytest.mark.parametrize('bad', ['missing', 'wronggoal', 'oldroot', 'stale', 'future', 'donecore', 'beforebegin', 'noadvance'])
def test_no_service_when_witness_invalid(bad):
    data = witness()
    if bad == 'missing': data = None
    elif bad == 'wronggoal': data['goal_id'] = 'cd' * 16
    elif bad == 'oldroot': data['states'][1]['fields']['session_id'] = 18
    elif bad == 'stale': data['states'][0]['when'] = 749999
    elif bad == 'future': data['status']['when'] = 1000001
    elif bad == 'donecore': data['status']['fields']['core_state'] = 4
    elif bad == 'beforebegin': data['states'][0]['when'] = 899999
    elif bad == 'noadvance': data['states'][1]['fields']['produced_at_us'] = 1000
    with pytest.raises(RuntimeError): validate(data)

##
# @brief 模拟先停arm_controller再hardware inactive，两调用同deadline6s，返回原ABORTED对象/错误不重写，Stop raw仍另需。
# @param monkeypatch pytest替换fixture，作用域结束恢复属性/环境。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_two_services_share_deadline_and_keep_aborted_original(monkeypatch):
    assert hasattr(actor, 'await_mid_motion_hardware_deactivation'), 'missing STOP actor'
    calls = []
    result = SimpleNamespace(status=6, result=SimpleNamespace(error_code=-4, error_string='ORIGINAL_ABORT'))
    future = SimpleNamespace(done=lambda: bool(calls), result=lambda: result)
    handle = SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes.fromhex(UUID)), get_result_async=lambda: future)
    adapter = SimpleNamespace(hardware_deactivation_witness=lambda goal: witness(),
        switch_controller=lambda name, active, deadline: calls.append(('controller', name, active, deadline)),
        set_hardware_state=lambda name, state, deadline: calls.append(('hardware', name, state, deadline)))
    monkeypatch.setattr(actor.time, 'monotonic', lambda: 1.0)
    monkeypatch.setattr(actor.time, 'monotonic_ns', lambda: 1000000000)
    monkeypatch.setattr(actor, 'wait', lambda node, f, timeout: f.result())
    feedback = [dict(goal_id=UUID, actual_rad=[0.0]*6), dict(goal_id=UUID, actual_rad=[.002]*6)]
    got, report = actor.await_mid_motion_hardware_deactivation(None, handle, feedback, 45.0, 0, adapter)
    assert got is result and got.result.error_string == 'ORIGINAL_ABORT'
    assert [c[:3] for c in calls] == [('controller', 'arm_controller', False), ('hardware', actor.HARDWARE_NAME, 'inactive')]
    assert calls[0][3] == calls[1][3] == 6.0
    assert report['device_stop_confirmation'] == 'REQUIRED_SEPARATELY'

##
# @brief goal result已done时actor拒绝，不为已完目标再作中途停机触发。
# @param monkeypatch pytest替换fixture，作用域结束恢复属性/环境。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_completed_goal_never_deactivates(monkeypatch):
    assert hasattr(actor, 'await_mid_motion_hardware_deactivation'), 'missing STOP actor'
    handle = SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes.fromhex(UUID)), get_result_async=lambda: SimpleNamespace(done=lambda: True))
    adapter = SimpleNamespace(hardware_deactivation_witness=lambda goal: witness())
    with pytest.raises(RuntimeError):
        actor.await_mid_motion_hardware_deactivation(None, handle, [], 45.0, 0, adapter)

##
# @brief adapter无可信live typed witness hook时立即RuntimeError且两个服务记录都空。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_missing_trusted_hook_never_calls_service():
    calls=[]
    adapter=SimpleNamespace(switch_controller=lambda *args:calls.append(args),
                            set_hardware_state=lambda *args:calls.append(args))
    with pytest.raises(RuntimeError, match='trusted live typed witness'):
        actor.await_mid_motion_hardware_deactivation(None, None, [], 45.0, 0, adapter)
    assert calls==[]

##
# @brief 首controller替身抛原错误时包装异常保留trigger run9和原文本，只调用一次，不继续hardware服务。
# @param monkeypatch pytest替换fixture，作用域结束恢复属性/环境。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_hardware_service_failure_retains_first_trigger(monkeypatch):
    calls=[];future=SimpleNamespace(done=lambda:False)
    handle=SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes.fromhex(UUID)),get_result_async=lambda:future)
    ##
    # @par 内部回调 fail
    # 记录服务参数后抛ORIGINAL_SERVICE_REJECT，模拟首服务失败，不联系ROS。
    # 参数 args：服务替身收到的位置参数，只记录后抛错。
    # 返回：无正常返回；抛RuntimeError并保留一次服务调用记录。
    def fail(*args):calls.append(args);raise RuntimeError('ORIGINAL_SERVICE_REJECT')
    adapter=SimpleNamespace(hardware_deactivation_witness=lambda goal:witness(),
                            switch_controller=fail,set_hardware_state=lambda *args:calls.append(args))
    monkeypatch.setattr(actor.time,'monotonic',lambda:1.0)
    monkeypatch.setattr(actor.time,'monotonic_ns',lambda:1000000000)
    feedback=[dict(goal_id=UUID,actual_rad=[0.]*6),dict(goal_id=UUID,actual_rad=[.002]*6)]
    with pytest.raises(actor.HardwareDeactivationAttemptError) as error:
        actor.await_mid_motion_hardware_deactivation(None,handle,feedback,45.,0,adapter)
    assert str(error.value)=='ORIGINAL_SERVICE_REJECT'
    assert error.value.trigger['root']['run_id']==9 and len(calls)==1

##
# @brief 见证先None后完整时原循环spin一次再调用两服务，不为未就绪见证先发停机。
# @param monkeypatch pytest替换fixture，作用域结束恢复属性/环境。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_pending_witness_waits_original_loop_without_services(monkeypatch):
    import sys
    calls=[];observations=[None,witness()];spins=[]
    result=SimpleNamespace(status=6,result=SimpleNamespace(error_code=-4,error_string='ORIGINAL'))
    future=SimpleNamespace(done=lambda:bool(calls),result=lambda:result)
    handle=SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes.fromhex(UUID)),get_result_async=lambda:future)
    adapter=SimpleNamespace(hardware_deactivation_witness=lambda goal:observations.pop(0),
        switch_controller=lambda *args:calls.append(args),set_hardware_state=lambda *args:calls.append(args))
    monkeypatch.setitem(sys.modules,'rclpy',SimpleNamespace(spin_once=lambda *args,**kwargs:spins.append(kwargs)))
    monkeypatch.setattr(actor.time,'monotonic',lambda:1.)
    monkeypatch.setattr(actor.time,'monotonic_ns',lambda:1000000000)
    monkeypatch.setattr(actor,'wait',lambda node,f,timeout:f.result())
    actor.await_mid_motion_hardware_deactivation(None,handle,[dict(goal_id=UUID,actual_rad=[0.]*6),
         dict(goal_id=UUID,actual_rad=[.002]*6)],45.,0,adapter)
    assert len(spins)==1 and len(calls)==2

##
# @brief 当前40s、原deadline45s没有完整五秒余量时拒绝，服务列表空。
# @param monkeypatch pytest替换fixture，作用域结束恢复属性/环境。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_five_second_reserve_exhaustion_never_services(monkeypatch):
    calls=[];handle=SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes.fromhex(UUID)),
        get_result_async=lambda:SimpleNamespace(done=lambda:False))
    adapter=SimpleNamespace(hardware_deactivation_witness=lambda goal:witness(),switch_controller=lambda *args:calls.append(args))
    monkeypatch.setattr(actor.time,'monotonic',lambda:40.)
    with pytest.raises(RuntimeError):actor.await_mid_motion_hardware_deactivation(None,handle,[],45.,0,adapter)
    assert calls==[]


##
# @brief ns单锚保证outer−requested恰5000000µs、两个deadline同6s；原goal5.9s不足预留时完全不发服务。
# @param monkeypatch pytest替换fixture，作用域结束恢复属性/环境。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_single_host_anchor_keeps_five_second_microsecond_budget_and_rejects_short_goal(monkeypatch):
    calls=[]
    result=SimpleNamespace(status=6,result=SimpleNamespace(error_code=-4,error_string='ORIGINAL_ABORT'))
    future=SimpleNamespace(done=lambda:bool(calls),result=lambda:result)
    handle=SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes.fromhex(UUID)),get_result_async=lambda:future)
    adapter=SimpleNamespace(hardware_deactivation_witness=lambda goal:witness(),
        switch_controller=lambda name,active,deadline:calls.append(('controller',deadline)),
        set_hardware_state=lambda name,state,deadline:calls.append(('hardware',deadline)))
    monkeypatch.setattr(actor.time,'monotonic_ns',lambda:1000000000)
    monkeypatch.setattr(actor.time,'monotonic',lambda:1.000015)
    monkeypatch.setattr(actor,'wait',lambda node,f,timeout:f.result())
    feedback=[dict(goal_id=UUID,actual_rad=[0.]*6),dict(goal_id=UUID,actual_rad=[.002]*6)]
    got,trigger=actor.await_mid_motion_hardware_deactivation(None,handle,feedback,45.,0,adapter)
    assert got is result and trigger['outer_deadline_steady_us']-trigger['requested_steady_us']==5000000
    assert calls==[('controller',6.0),('hardware',6.0)]
    # 原Goal不足5s预留必须拒；单锚修复不扩大原目标截止或发任一服务。
    calls.clear()
    with pytest.raises(RuntimeError):
        actor.await_mid_motion_hardware_deactivation(None,handle,feedback,5.9,0,adapter)
    assert calls==[]
