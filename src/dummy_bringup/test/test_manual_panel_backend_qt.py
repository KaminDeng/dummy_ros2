"""真实Qt面板验证：模拟模式不启动Native，切换目标后清除原授权。"""
##
# @file test_manual_panel_backend_qt.py
# @brief 真实Qt面板搭配软件后端的交互与后端切换回归。
#
# 构造真实JointPanel和rclpy节点，但QProcess.start被替身拦截，三项发现查询注入内存Graph，ApplyPlanningScene成功由Future样式NS注入，不启动Native/MoveIt。Qt事件泵和软件advance有实际运行，所得反馈是纯软件，不是实机。
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


##
# @brief 处理五轮Qt事件、每轮等待20ms后手动tick，用于驱动按钮和软件worker信号。
# @param panel 真实Qt JointPanel，worker为软件后端。
# @return 无；事件泵不等待真实运动完成。
def pump(panel):
    for _ in range(5):
        QtWidgets.QApplication.processEvents()
        QtTest.QTest.qWait(20)
    panel.tick()


##
# @brief 核确认拒绝/跳过确认、软件2°运动和STOP；切real仅记录一次start调用并清旧反馈/授权，切回sim仍不自动恢复授权。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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
    # 与Apply回执替身配套，提供唯一发现图；原生产门禁与UI断言均保留。
    from test_manual_moveit_service_topology import Graph
    graph = Graph()
    for method in ('count_services', 'get_node_names_and_namespaces',
                   'get_service_names_and_types_by_node'):
        monkeypatch.setattr(node, method, getattr(graph, method))
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
        ##
        # @par 内部回调 unexpected_dialog
        # 跳过确认已选中后若仍弹框立即抛AssertionError，防止测试默许多余确认。
        # 参数 args：额外位置参数，替身忽略。
        # 返回：无正常返回；总抛AssertionError。
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
        assert panel.status.text().startswith('仿真-本地 · 运动学反馈')
        panel.stop.click(); pump(panel)
        assert not panel.authorized and panel.wait_home and panel.calibration
        assert starts == [] and not list((tmp_path/'report').glob('*.raw'))
        panel.backend_choice.setCurrentIndex(panel.backend_choice.findData('real')); pump(panel)
        assert panel.backend == 'real' and len(starts) == 1
        assert panel.last_state is None and panel.calibration is None
        assert all(label.text() == '—' for label in panel.feedback_labels)
        assert not panel.authorized and not hasattr(panel, 'actual_q')
        panel.backend_choice.setCurrentIndex(panel.backend_choice.findData('simulator')); pump(panel)
        assert panel.backend == 'simulator' and panel.fresh() and not panel.authorized
        assert len(starts) == 1
    finally:
        panel.close(); app.processEvents()
        panel.timer.stop(); node.destroy_node(); rclpy.shutdown()
