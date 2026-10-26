"""Qt ROS 面板不使用 Native 解码器或实测关节发布器。"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from PyQt5 import QtCore, QtWidgets
import rclpy
SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


def test_ros_panel_readonly_without_native(tmp_path, monkeypatch):
    import manual_panel_ros_backend
    from manual_joint_panel import JointPanel
    class Transport:
        def control_graph_error(self): return ''
        def __init__(self, node, on_status, on_state): self.requests = []
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
        panel.ros_status_received(NS(boot_epoch=1, reference_epoch=1))
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
    panel=NS(backend='ros',ros_runtime=NS(process=object()),ros_backend=NS(released=False),
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
        def __init__(self, *args): self.status_max_age_ms = None
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
        panel.last_state = dict(boot_epoch=1,reference_epoch=7,session_id=0,position_valid_mask=63,core_state=2)
        panel.fresh = lambda: True  # 本例只刺激Status参数源，State另有独立测试。
        panel.ros_status_received(NS(boot_epoch=1,reference_epoch=7,session_id=0,core_state=2,
                                     real_motion_permitted=False,permit_remaining_us=0))
        panel.ros_status_ns = time.monotonic_ns() - 600_000_000
        panel.ros_tick()
        assert not panel.resume.isEnabled() and not panel.zero.isEnabled()
        assert not panel.authorized
        panel.ros_backend.transport.status_max_age_ms = 731  # 假C++有效参数，非产品默认值。
        panel.ros_tick()
        assert panel.resume.isEnabled() and panel.zero.isEnabled()
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


def test_ros_actual_model_requires_cpp_ready_and_fresh_matched_feedback(tmp_path, monkeypatch):
    import time
    import manual_panel_ros_backend
    from manual_joint_panel import JointPanel
    from dpsdk_interfaces.msg import DummyJointStateV2
    class Transport:
        def __init__(self, *args): self.status_max_age_ms=1000
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
        state=DummyJointStateV2();state.boot_epoch=1;state.reference_epoch=7;state.state_seq=1
        state.position_valid_mask=63;state.position_rad=[0.]*6;state.can_rx_age_us=[0]*6
        state.core_state=2
        panel.ros_status_received(NS(boot_epoch=1,reference_epoch=7,session_id=0,core_state=2,
            real_motion_permitted=False,permit_remaining_us=0))
        panel.ros_state_received(state);panel.ros_tick()
        assert panel.actual_q==[0.]*6 and not panel.description_visible
        state.state_seq=2;state.session_id=9;state.core_state=4
        panel.ros_status_received(NS(boot_epoch=1,reference_epoch=7,session_id=9,core_state=4,
            real_motion_permitted=True,permit_remaining_us=1000000))
        panel.ros_state_received(state);panel.ros_backend.manual_ready=True
        panel.ros_event('manual_ready','MANUAL_CONTROL_READY');panel.ros_tick()
        assert panel.description_visible and panel.publisher is None
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
