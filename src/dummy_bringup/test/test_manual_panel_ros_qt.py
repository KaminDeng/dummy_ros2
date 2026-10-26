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
        assert 'auto_calibrate_and_test' not in panel.ros_backend.transport.requests
        assert not panel.motor_parameter_tab.allowed()
        assert panel.ros_runtime.state == 'Starting'
        panel.ros_status_received(NS(boot_epoch=1, reference_epoch=1))
        assert panel.ros_runtime.state == 'ReadOnly'
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
    panel=NS(backend='ros',ros_backend=NS(released=False),
        release=lambda *args:calls.append('release'),message=lambda *args:None)
    JointPanel.request_reconnect(panel)
    assert calls==['release']


def test_ros_confirmed_reconnect_stops_only_owned_runtime_and_rebuilds_readonly():
    from manual_joint_panel import JointPanel
    calls=[]
    panel=NS(backend='ros', ros_backend=NS(released=True,close=lambda:calls.append('close') or True),
        ros_runtime=NS(stop=lambda confirmed:calls.append(('stop',confirmed)) or True),
        apply_backend_change=lambda:calls.append('rebuild'))
    JointPanel.request_reconnect(panel)
    assert calls==[('stop',True),'close','rebuild']
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
