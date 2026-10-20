"""真实Qt面板验证：模拟模式不启动Native，切换目标后清除原授权。"""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

from PyQt5 import QtCore, QtWidgets, QtTest
import rclpy

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from manual_joint_panel import JointPanel


def pump(panel):
    for _ in range(5):
        QtWidgets.QApplication.processEvents()
        QtTest.QTest.qWait(20)
    panel.tick()


def test_simulator_panel_move_stop_and_switch_without_hardware(tmp_path, monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config = json.loads((SCRIPTS.parent/'config/manual_joint_panel.json').read_text())
    config['description_xacro'] = str(SCRIPTS.parents[1]/'dummy-ros2_description/urdf/dummy-ros2.xacro')
    config['backend'] = 'simulator'
    config['moveit_enabled'] = True
    config['obstacles_file'] = str(tmp_path/'obstacles.json')
    # 即使Native不可执行，模拟面板也必须完整可用。
    binary = tmp_path/'not-an-executable'; binary.write_text('no hardware')
    config['native_binary'] = str(binary)
    starts = []
    monkeypatch.setattr(QtCore.QProcess, 'start', lambda *args: starts.append(args))
    monkeypatch.setattr(QtWidgets.QMessageBox, 'question', lambda *args: QtWidgets.QMessageBox.Yes)
    rclpy.init(args=[])
    node = rclpy.create_node('panel_backend_test')
    panel = JointPanel(config, tmp_path/'report', lambda *_: None, node)
    # 本测试隔离MoveIt进程，显式模拟ApplyPlanningScene的成功回执。
    panel.obstacle_tab.phase = 'apply'
    panel.obstacle_tab.future = NS(done=lambda: True, result=lambda: NS(success=True))
    try:
        pump(panel)
        assert panel.backend_choice.currentData() == 'simulator'
        assert panel.fresh() and not panel.authorized and panel.wait_home
        assert not panel.skip_confirmations.isChecked()
        dialogs = []
        monkeypatch.setattr(QtWidgets.QMessageBox, 'question', lambda *args: dialogs.append(args[1]) or QtWidgets.QMessageBox.No)
        panel.resume.click(); pump(panel)
        assert dialogs == ['恢复控制'] and not panel.authorized
        panel.skip_confirmations.setChecked(True)
        def unexpected_dialog(*args):
            raise AssertionError('勾选跳过确认后仍弹出执行确认')
        monkeypatch.setattr(QtWidgets.QMessageBox, 'question', unexpected_dialog)
        assert panel.resume.isEnabled()
        panel.resume.click(); pump(panel)
        assert panel.authorized and panel.calibration
        goal = list(config['rest_deg']); goal[0] = 2.
        for box, value in zip(panel.inputs, goal): box.setValue(value)
        assert panel.move.isEnabled()
        panel.move.click(); pump(panel)
        assert panel.busy
        event = panel.worker.model.advance(100)
        panel.worker.emit_event(event); pump(panel)
        assert not panel.busy and math.degrees(panel.actual_q[0]) == 2.
        assert '纯软件模拟反馈' in panel.status.text()
        panel.stop.click(); pump(panel)
        assert not panel.authorized and panel.wait_home and panel.calibration
        assert starts == [] and not list((tmp_path/'report').glob('*.raw'))
        panel.backend_choice.setCurrentIndex(0); pump(panel)
        assert panel.backend == 'real' and len(starts) == 1
        assert panel.last_state is None and panel.calibration is None
        assert all(label.text() == '—' for label in panel.feedback_labels)
        assert not panel.authorized and not hasattr(panel, 'actual_q')
        panel.backend_choice.setCurrentIndex(1); pump(panel)
        assert panel.backend == 'simulator' and panel.fresh() and not panel.authorized
        assert len(starts) == 1
    finally:
        panel.close(); app.processEvents()
        panel.timer.stop(); node.destroy_node(); rclpy.shutdown()
