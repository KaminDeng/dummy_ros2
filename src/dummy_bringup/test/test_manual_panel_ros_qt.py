"""Qt ROS 面板不使用 Native 解码器或实测关节发布器。"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from PyQt5 import QtCore, QtWidgets
import rclpy
SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


def root_feedback(*, session=0, core=2):
    """用实际生成Root建立UI反馈，Source不借用已退役的boot平行字段。"""
    from copy import deepcopy
    from test_manual_panel_root import state as make_state
    from dpsdk_interfaces.msg import DummyDeviceStatus
    state=make_state();state.reference_epoch=7;state.state_seq=1
    state.session_id=session;state.core_state=core
    for row in state.joints:
        row.position_rad=0.;row.hold_position_rad=0.
    status=DummyDeviceStatus()
    for name in status.get_fields_and_field_types():
        if name.startswith('has_'):setattr(status,name,True)
    status.source=deepcopy(state.source);status.profile_id=state.profile_id
    status.reference_epoch=state.reference_epoch;status.session_id=session;status.core_state=core
    return state,status


def test_ros_panel_readonly_without_native(tmp_path, monkeypatch):
    import manual_panel_ros_backend
    from manual_joint_panel import JointPanel
    class Transport:
        def control_graph_error(self): return ''
        def __init__(self, node, on_status, on_state, *, own_control_node=False):
            assert own_control_node
            self.requests = []
        def request(self, service, callback):
            self.requests.append(service)
            callback(NS(success=True, message='RELEASED_IDLE' if service == 'bootstrap_status' else 'RELEASE_QUEUED'))
        def close(self): pass
    monkeypatch.setattr(manual_panel_ros_backend, 'RosTransport', Transport)
    monkeypatch.setattr(QtCore.QProcess, 'start', lambda *args: (_ for _ in ()).throw(AssertionError('禁止启动 Native')))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config = json.loads((SCRIPTS.parent / 'config/manual_joint_panel.json').read_text())
    config.update(backend='ros', moveit_enabled=False, ros_launch_command=[])
    config['description_xacro'] = str(SCRIPTS.parents[1] / 'dummy-ros2_description/urdf/dummy-ros2.xacro')
    config.pop('native_binary', None); config.pop('decoder_directory', None)
    rclpy.init(args=[])
    node = rclpy.create_node('ros_panel_fake_test')
    panel = JointPanel(config, tmp_path / 'report', None, node)
    try:
        assert panel.publisher is None
        assert panel.decode is None
        assert panel.backend_choice.currentData() == 'ros'
        assert panel.cfg['motion_speed_max_deg_s'] == panel.speed_input.maximum() == 63.
        assert panel.cfg['motion_acceleration_max_rad_s2'] == panel.acceleration_input.maximum() == 4.
        assert panel.speed_input.value() == 60. and panel.acceleration_input.value() == 4.
        assert 'auto_calibrate_and_test' not in panel.ros_backend.transport.requests
        assert not panel.motor_parameter_tab.allowed()
        assert panel.ros_runtime.state == 'Starting'
        panel.ros_status_received(root_feedback()[1])
        assert panel.ros_runtime.state == 'ReadOnly'
        # Native忙期留下的按钮状态不能锁住已返回只读的ROS连接。
        panel.reconnect.setEnabled(False)
        panel.backend_choice.setEnabled(False)
        panel.ros_tick()
        assert panel.reconnect.isEnabled() and panel.backend_choice.isEnabled()
        panel.busy = True
        panel.ros_tick()
        assert not panel.reconnect.isEnabled() and not panel.backend_choice.isEnabled()
    finally:
        panel.ros_backend.released = True
        panel.close(); panel.timer.stop(); node.destroy_node(); rclpy.shutdown()


def test_ros_resume_only_sends_cpp_prepare_after_confirmation():
    from manual_joint_panel import JointPanel
    calls=[]
    panel=NS(backend='ros', closing=False, busy=False, calibration=(1,7),
        last_state={'session_id':0}, fresh=lambda:True, control_state_allowed=lambda _:True,
        confirm_execution=lambda *args:QtWidgets.QMessageBox.Yes,
        command=lambda *args:(_ for _ in ()).throw(AssertionError('不得发送Native RESUME')),
        ros_backend=NS(released=False, admitting=True, request_prepare_manual_control=lambda:calls.append('prepare')))
    JointPanel.start_resume(panel)
    assert calls==['prepare']
    panel.ros_backend.released=True
    JointPanel.start_resume(panel)
    assert calls==['prepare']


def test_ros_reconnect_does_not_replace_owner_before_release():
    from manual_joint_panel import JointPanel
    calls=[]
    panel=NS(backend='ros',ros_runtime=NS(state='Active',process=object()),ros_backend=NS(released=False),
        release=lambda *args:calls.append('release'),message=lambda *args:None)
    JointPanel.request_reconnect(panel)
    assert calls==['release']

def test_pending_reconnect_rechecks_usb_enumeration_and_retires_old_connection_once(monkeypatch):
    from manual_joint_panel import JointPanel
    calls=[];clock=[1.];detached=[False]
    monkeypatch.setattr('manual_joint_panel.time.monotonic',lambda:clock[0])
    panel=NS(closing=False,ros_reconnect_pending=True,authorized=True,
        refresh_ros_connection_controls=lambda:None,ros_started_at=0.,cfg={},
        ros_runtime=NS(check_startup_timeout=lambda *_:None,
            retire_disconnected_owner=lambda:calls.append('check') or detached[0]),
        ros_backend=NS(released=False,transport=NS(control_graph_error=lambda:'等待设备',close=lambda:calls.append('close'))),
        show_actual_description=lambda *_:None,status=NS(setText=lambda *_:None),
        message=lambda *_:None,apply_backend_change=lambda:calls.append('rebuild'))
    JointPanel.ros_tick(panel)
    assert calls==['check'] and panel.ros_reconnect_pending
    clock[0]+=1.;detached[0]=True
    JointPanel.ros_tick(panel)
    assert calls==['check','check','close','rebuild']
    assert not panel.ros_reconnect_pending and not panel.authorized
    assert not panel.ros_backend.released and panel.pending_backend=='ros'
    JointPanel.ros_tick(panel)
    assert calls==['check','check','close','rebuild']


def test_ros_confirmed_reconnect_stops_only_owned_runtime_and_rebuilds_readonly():
    from manual_joint_panel import JointPanel
    calls=[]
    panel=NS(backend='ros', ros_backend=NS(released=True,close=lambda:calls.append('close') or True),
        ros_runtime=NS(process=object(),stop=lambda confirmed:calls.append(('stop',confirmed)) or True),
        apply_backend_change=lambda:calls.append('rebuild'))
    JointPanel.request_reconnect(panel)
    assert calls==[('stop',True),'close','rebuild']
    assert panel.pending_backend=='ros'

def test_cpp_local_retirement_resumes_pending_reconnect_without_another_release():
    from manual_joint_panel import JointPanel
    from manual_panel_ros_backend import RosBackend
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls=[]
    class Transport:
        def request(self,name,callback):calls.append(name);self.callback=callback
        def close(self):calls.append('close')
    t=Transport()
    panel=NS(backend='ros',busy=True,ros_reconnect_pending=True,message=lambda *_:None,
        ros_runtime=NS(process=object(),stop=lambda confirmed:calls.append(('stop',confirmed)) or True),
        apply_backend_change=lambda:calls.append('rebuild'))
    panel.request_reconnect=lambda:JointPanel.request_reconnect(panel)
    b=RosBackend(t,lambda kind,value:JointPanel.ros_event(panel,kind,value));panel.ros_backend=b
    b.connect();b.release();t.callback(NS(success=False,message='LOCAL_READONLY_RETIRED'))
    app.processEvents()
    assert calls==['disable_and_release',('stop',True),'close','rebuild']
    assert not b.released and b.local_retired and not panel.busy


def test_ros_detached_reconnect_retires_only_old_connection_without_claiming_release():
    from manual_joint_panel import JointPanel
    calls=[]
    panel=NS(backend='ros', authorized=True,
        ros_runtime=NS(process=object(),retire_disconnected_owner=lambda:calls.append('retire') or True),
        ros_backend=NS(released=False,transport=NS(close=lambda:calls.append('close'))),
        release=lambda *args:calls.append('release'),message=lambda *args:None,
        apply_backend_change=lambda:calls.append('rebuild'))
    JointPanel.request_reconnect(panel)
    assert calls==['retire','close','rebuild']
    assert not panel.authorized and not panel.ros_backend.released
    assert panel.pending_backend=='ros'


def test_status_buttons_wait_for_cpp_parameter_and_use_returned_budget(tmp_path, monkeypatch):
    import time
    import manual_panel_ros_backend
    from manual_joint_panel import JointPanel
    class Transport:
        def control_graph_error(self): return ''
        def __init__(self, *args, **kwargs): self.status_max_age_ms = None
        def ready(self, *args): return False
        def close(self): pass
    monkeypatch.setattr(manual_panel_ros_backend, 'RosTransport', Transport)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config = json.loads((SCRIPTS.parent / 'config/manual_joint_panel.json').read_text())
    config.update(backend='ros',moveit_enabled=False,ros_launch_command=[])
    config['description_xacro'] = str(SCRIPTS.parents[1] / 'dummy-ros2_description/urdf/dummy-ros2.xacro')
    rclpy.init(args=[])
    node = rclpy.create_node('ros_parameter_ui_test')
    panel = JointPanel(config,tmp_path/'report',None,node)
    try:
        state,status=root_feedback()
        # 现场参考已建立的近双限位姿态：恢复入口不能被MoveIt内侧规划余量误封。
        state.joints[1].position_rad=__import__('math').radians(-72.913)
        state.joints[2].position_rad=__import__('math').radians(179.878)
        panel.ros_state_received(state)
        panel.fresh = lambda: True  # 本例只刺激Status参数源，State另有独立测试。
        panel.ros_status_received(status)
        panel.ros_status_ns = time.monotonic_ns() - 600_000_000
        panel.ros_tick()
        assert not panel.resume.isEnabled() and not panel.zero.isEnabled()
        assert not panel.authorized
        panel.ros_backend.transport.status_max_age_ms = 731  # 假C++有效参数，非产品默认值。
        panel.ros_tick()
        assert panel.resume.isEnabled() and panel.zero.isEnabled()
        # 同boot但不同上下文/设备不得让旧反馈打开恢复按钮。
        for field in ('context_id','boot_hi'):
            previous=getattr(status.source,field);setattr(status.source,field,previous+1)
            panel.ros_tick();assert not panel.resume.isEnabled()
            setattr(status.source,field,previous)
        previous=list(status.source.device);status.source.device=list(reversed(previous))
        panel.ros_tick();assert not panel.resume.isEnabled();status.source.device=previous
        status.profile_id+=1;panel.ros_tick();assert not panel.resume.isEnabled();status.profile_id-=1
        status.has_reference_epoch=False;panel.ros_tick();assert not panel.resume.isEnabled()
        status.has_reference_epoch=True;panel.ros_tick();assert panel.resume.isEnabled()
        assert '731ms' in panel.status.text()
        panel.ros_status_ns = time.monotonic_ns() - 800_000_000
        panel.ros_tick()
        assert not panel.resume.isEnabled() and not panel.zero.isEnabled()
    finally:
        panel.ros_backend.released=True
        panel.close();panel.timer.stop();node.destroy_node();rclpy.shutdown()


def test_explicit_reconnect_after_refused_start_never_releases_external_owner():
    from manual_joint_panel import JointPanel
    calls=[]
    panel=NS(backend='ros',ros_runtime=NS(process=None),
        ros_backend=NS(released=False,transport=NS(_startup_owner_conflict='existing owner',close=lambda:calls.append('close-local-clients'))),
        release=lambda *_:calls.append('FORBIDDEN_D10'),apply_backend_change=lambda:calls.append('recheck-new-attempt'))
    JointPanel.request_reconnect(panel)
    assert calls==['close-local-clients','recheck-new-attempt']
    assert panel.pending_backend=='ros'


def test_release_after_refused_start_does_not_latch_busy_or_touch_external_owner():
    from manual_joint_panel import JointPanel
    calls=[]
    panel=NS(backend='ros',busy=False,authorized=False,
        ros_runtime=NS(process=None,begin_release=lambda:calls.append('FORBIDDEN_begin_release')),
        ros_backend=NS(released=False,transport=NS(_startup_owner_conflict='existing owner'),
                       release=lambda:calls.append('FORBIDDEN_D10')),
        message=lambda message:calls.append(message))
    JointPanel.release(panel,'RELEASE')
    assert not panel.busy
    assert not any(item.startswith('FORBIDDEN') for item in calls)


def test_repeated_expired_release_does_not_leave_connection_controls_busy():
    from manual_joint_panel import JointPanel
    calls=[]
    panel=NS(backend='ros',busy=False,authorized=True,
        ros_runtime=NS(process=object(),begin_release=lambda:calls.append('release')),
        ros_backend=NS(released=False,pending=False,transport=NS(),release=lambda:False))
    JointPanel.release(panel,'RELEASE')
    assert not panel.busy and not panel.authorized


def test_ros_actual_model_requires_cpp_ready_and_fresh_matched_feedback(tmp_path, monkeypatch):
    import time
    import manual_panel_ros_backend
    from manual_joint_panel import JointPanel
    class Transport:
        def __init__(self, *args, **kwargs): self.status_max_age_ms=1000
        def control_graph_error(self): return ''
        def ready(self, *args): return False
        def close(self): pass
    monkeypatch.setattr(manual_panel_ros_backend,'RosTransport',Transport)
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config=json.loads((SCRIPTS.parent/'config/manual_joint_panel.json').read_text())
    config.update(backend='ros',moveit_enabled=False,ros_launch_command=[])
    config['description_xacro']=str(SCRIPTS.parents[1]/'dummy-ros2_description/urdf/dummy-ros2.xacro')
    rclpy.init(args=[]);node=rclpy.create_node('readonly_model_visibility_test')
    panel=JointPanel(config,tmp_path/'report',None,node)
    try:
        state,status=root_feedback()
        state.joints[2].position_rad=1.57;state.joints[2].hold_position_rad=1.57
        panel.ros_status_received(status)
        panel.ros_state_received(state);panel.ros_tick()
        assert panel.actual_q==[0.,0.,1.57,0.,0.,0.] and not panel.description_visible
        state.state_seq=2;state.session_id=9;state.core_state=4
        status.session_id=9;status.core_state=4
        status.real_motion_permitted=True;status.permit_remaining_us=1000000
        panel.ros_status_received(status)
        panel.ros_state_received(state);panel.ros_backend.manual_ready=True
        panel.ros_event('manual_ready','MANUAL_CONTROL_READY');panel.ros_tick()
        assert panel.description_visible and panel.publisher is None
        # C++常驻监视仍持有本owner；下一次明确执行会先申请许可，不直接授予设备运动。
        state.hold_ready=True;state.state_seq=3;panel.ros_state_received(state)
        panel.ros_status.real_motion_permitted=False;panel.ros_status.permit_remaining_us=0
        panel.ros_tick()
        assert panel.authorized and panel.move.isEnabled()
        # 真Qt面板回调→Root UI投影→真实MoveItTab准入；场景服务在本例独立隔离。
        from manual_moveit_tab import MoveItTab
        tab=MoveItTab(panel);tab.scene_ready=lambda:True
        assert tab.ready()
        state.state_seq+=1;state.joints[2].hold_valid_quality=False
        state.joints[2].hold_position_rad=float('nan')
        panel.ros_state_received(state);panel.ros_tick();assert not tab.ready()
        state.state_seq+=1;state.joints[2].hold_valid_quality=True
        state.joints[2].hold_position_rad=state.joints[2].position_rad
        panel.ros_state_received(state);panel.ros_tick();assert tab.ready()
        state.state_seq+=1;state.joints[2].velocity_valid_quality=False
        panel.ros_state_received(state);panel.ros_tick();assert not tab.ready()
        state.state_seq+=1;state.joints[2].velocity_valid_quality=True
        panel.ros_state_received(state);panel.ros_tick();assert tab.ready()
        panel.ros_backend.manual_ready=False;panel.ros_tick()
        assert not panel.authorized
        panel.ros_backend.manual_ready=True
        panel.last_state_ns=time.monotonic_ns()-300_000_000;panel.ros_tick()
        assert not panel.description_visible
        # 自动基线没有manual_ready事件，但真实双Hold表明已通过Node控制器激活门。
        panel.last_state_ns=time.monotonic_ns();panel.ros_backend.manual_ready=False
        panel.ros_backend.operation='auto';panel.ros_tick()
        assert panel.description_visible
        panel.ros_status.reference_epoch=8;panel.ros_tick()
        assert not panel.description_visible
    finally:
        panel.ros_backend.released=True
        panel.close();panel.timer.stop();node.destroy_node();rclpy.shutdown()


def test_ros_reference_entry_prepares_manual_control_without_automatic_baseline():
    from manual_joint_panel import JointPanel
    for has_reference in (False, True):
        calls = []
        panel = NS(backend='ros', cfg={'operator_confirmed_double_limits': not has_reference},
            calibration=(1, 2) if has_reference else None, fresh=lambda:True,
            ros_status=NS(session_id=0), closing=False, busy=False,
            confirm_execution=lambda *args:QtWidgets.QMessageBox.Yes,
            message=lambda *args:None,
            ros_backend=NS(released=False, admitting=True,
                request_prepare_manual_control=lambda:calls.append('manual') or True,
                request_calibration_and_test=lambda:calls.append('automatic') or True))
        JointPanel.start_zero(panel)
        assert calls == ['manual'], '标定入口不得自动执行验收运动或依赖圆柱场景'


def test_ros_reference_entry_still_requires_confirmed_pose_without_reference():
    from manual_joint_panel import JointPanel
    calls = []
    panel = NS(backend='ros', cfg={'operator_confirmed_double_limits':False},
        calibration=None, fresh=lambda:True, ros_status=NS(session_id=0),
        closing=False, busy=False, ros_backend=NS(released=False, admitting=True),
        message=lambda text:calls.append(text),
        confirm_execution=lambda *args:(_ for _ in ()).throw(AssertionError('不得提交')))
    JointPanel.start_zero(panel)
    assert len(calls) == 1 and '未确认双限位' in calls[0]


def test_ros_target_switch_waits_for_retirement_and_keeps_requested_backend(monkeypatch):
    """等待C++只读退休不丢失目标；证明到达后退出旧owner再切换，不重连ROS。"""
    from manual_joint_panel import JointPanel
    from manual_panel_ros_backend import RosBackend
    import manual_panel_runtime
    import manual_panel_config
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(manual_panel_runtime, 'load_runtime_panel_config', lambda *_: {'target': 'native'})
    monkeypatch.setattr(manual_panel_config, 'load_decoder', lambda *_: 'decoder')
    for target in ('real', 'simulator'):
        calls = []
        choice = QtWidgets.QComboBox()
        for label, value in [('ROS', 'ros'), ('Native', 'real'), ('Simulator', 'simulator')]:
            choice.addItem(label, value)
        choice.setCurrentIndex(choice.findData(target))
        class Transport:
            def request(self, name, callback):
                calls.append(name)
                self.callback = callback
            def close(self): calls.append('close')
        t = Transport()
        panel = NS(backend='ros', backend_choice=choice, cfg={'_config_path': '/selected/config'},
            pending_backend=None, closing=False, busy=False, authorized=False,
            ros_reconnect_pending=False, message=lambda *_:None,
            ros_runtime=NS(process=object(), state='ReadOnly', begin_release=lambda:None,
                stop=lambda proof:calls.append(('stop', proof)) or True),
            apply_backend_change=lambda:calls.append(('switch', panel.pending_backend)),
            refresh_ros_connection_controls=lambda:None,
            request_reconnect=lambda:calls.append('FORBIDDEN_ROS_RECONNECT'))
        panel.request_backend_change=lambda *args, **kwargs:JointPanel.request_backend_change(panel, *args, **kwargs)
        panel.release=lambda command:JointPanel.release(panel, command)
        b=RosBackend(t, lambda kind,value:JointPanel.ros_event(panel,kind,value));panel.ros_backend=b
        b.connect()
        JointPanel.request_backend_change(panel)
        assert calls == ['disable_and_release']
        assert panel.pending_backend == target
        assert not panel.ros_reconnect_pending
        assert choice.currentData() == 'ros'
        t.callback(NS(success=False, message='LOCAL_READONLY_RETIRED'))
        app.processEvents()
        JointPanel.ros_tick(panel)
        assert calls == ['disable_and_release', ('stop', True), 'close', ('switch', target)]
        assert not b.released and b.local_retired


def test_ros_target_selection_cancels_already_queued_reconnect():
    """较早释放回执排队的重连回调不能覆盖随后选择的连接目标。"""
    from manual_joint_panel import JointPanel
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls=[]
    panel=NS(ros_backend=NS(can_close=True),ros_runtime=NS(process=object()),
        pending_backend=None,ros_reconnect_pending=True,closing=False,busy=True,
        message=lambda *_:None,request_reconnect=lambda:calls.append('unexpected_reconnect'))
    JointPanel.ros_event(panel,'retired','LOCAL_READONLY_RETIRED')
    panel.pending_backend='real'
    panel.ros_reconnect_pending=False
    app.processEvents()
    assert calls==[]


def test_failed_owned_fault_cleanup_retires_runtime_only_after_actual_fresh_root_pair(monkeypatch):
    """Qt退出通知保留业务失败，只在权威故障释放双消息成立后退出本测试拥有的进程。"""
    import manual_panel_ros_backend as module
    from manual_panel_runtime import PanelRuntime
    from manual_joint_panel import JointPanel
    from test_manual_panel_ros_backend import actual_release_feedback
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    clock=[10.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    calls=[]
    class Process:
        def __init__(self):self.returncode=None
        def poll(self):return self.returncode
        def terminate(self):calls.append('terminate-owned');self.returncode=0
        def wait(self,timeout):return self.returncode
    runtime=PanelRuntime(spawn=lambda *args,**kwargs:None)
    runtime.process=Process();runtime.state='Active'
    t=object.__new__(module.RosTransport);t.owns_control_node=True
    t.latest_state,t.latest_status=actual_release_feedback(core=9,reason=0x1008)
    t.state_at=t.status_at=9.99;t.feedback_max_age_ms=250;t.status_max_age_ms=1000
    failure='MANUAL_CONTROL_FAILED: 原参考失效 cleanup=RELEASED_FAULT'
    replies=[NS(success=False,message=failure),NS(success=False,message='EVIDENCE_INCOMPLETE;RELEASED_FAULT')]
    t.request=lambda name,cb:cb(replies.pop(0));t.close=lambda:calls.append('close-observer')
    panel=NS(backend='ros',closing=False,busy=True,ros_runtime=runtime,
        message=lambda value:calls.append(('log',value)),apply_backend_change=lambda:calls.append('rebuild-readonly'))
    panel.request_reconnect=lambda:JointPanel.request_reconnect(panel)
    b=module.RosBackend(t,lambda *event:JointPanel.ros_event(panel,*event));panel.ros_backend=b
    b.operation='manual';b.manual_ready=b.admitting=True
    b.poll();app.processEvents()
    assert runtime.state=='Error' and runtime.error==failure and 'terminate-owned'not in calls
    t.latest_state.state_seq+=1;t.state_at=t.status_at=10.1;clock[0]=10.2
    b.poll();app.processEvents()
    assert calls[-3:]==['terminate-owned','close-observer','rebuild-readonly']
    assert runtime.state=='Disconnected' and runtime.error==failure
    assert b.can_close and not b.manual_ready and not b.admitting
    assert not any('ROS completed:' in str(value)for value in calls)
