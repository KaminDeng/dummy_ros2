"""实际 Qt 参数表的异步串行请求、写后核对与失效边界。"""
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from PyQt5 import QtCore, QtWidgets, QtTest
from manual_motor_parameters import MotorParameterTab, PARAMETERS


class Panel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.last_state = dict(boot_epoch=7, core_state=2, session_id=0)
        self.worker_generation = 1
        self.busy = self.closing = False
        self.worker = SimpleNamespace(state=lambda: QtCore.QProcess.Running)
        self.is_fresh = True
        self.commands = []

    def fresh(self):
        return self.is_fresh

    def command(self, command):
        self.commands.append(command)
        return True


class MotorParametersTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.panel = Panel()
        self.tab = MotorParameterTab(self.panel)
        self.tab.tick()

    def tearDown(self):
        self.tab.close()
        self.panel.close()

    def answer(self, raw=123, status=0):
        p = self.tab.pending
        return self.tab.handle_line(f'MOTOR_PARAM_RESULT {p["sequence"]} {p["node"]} {p["parameter"]} {status} {raw}')

    def test_read_all_serializes_runtime_and_eeprom_for_selected_motor(self):
        self.tab.motor.setCurrentIndex(3)
        self.tab.read.click()
        self.assertEqual(len(self.panel.commands), 1)
        self.assertIn(' 4 1 0 0', self.panel.commands[-1])
        for _ in range(24):
            self.answer()
        self.assertEqual(len(self.panel.commands), 24)
        self.assertIsNone(self.tab.pending)
        self.assertTrue(all(self.tab.table.item(row, col).text() == '123'
                            for row in range(12) for col in (2, 3)))

    def test_apply_and_readback_mismatch(self):
        self.tab.table.selectRow(1)
        self.tab.inputs[2].setValue(1500)
        self.tab.apply.click()
        self.assertTrue(self.panel.commands[-1].endswith(' 2 1 1500'))
        self.answer(1500)
        self.assertTrue(self.panel.commands[-1].endswith(' 2 0 0'))
        self.answer(1499)
        self.assertIn('不一致', self.tab.status.text())
        self.assertIsNone(self.tab.pending)

    def test_edit_ki_after_selecting_kd_saves_ki_and_commits_typed_value(self):
        self.tab.motor.setCurrentIndex(5)
        self.tab.inputs[9].setValue(800)
        self.tab.inputs[8].setValue(3)
        self.tab.table.selectRow(8)
        self.panel.resize(1200, 800)
        self.panel.show()
        self.tab.resize(1100, 700)
        self.tab.show()
        self.app.processEvents()
        editor = self.tab.inputs[8].lineEdit()
        QtTest.QTest.mouseClick(editor, QtCore.Qt.LeftButton)
        QtTest.QTest.keyClick(editor, QtCore.Qt.Key_A, QtCore.Qt.ControlModifier)
        QtTest.QTest.keyClicks(editor, '50')
        QtTest.QTest.mouseClick(self.tab.save, QtCore.Qt.LeftButton)
        self.assertEqual(self.panel.commands[-1].split()[2:], ['6', '8', '2', '50'])

    def test_save_reads_eeprom_and_confirms(self):
        self.tab.table.selectRow(5)
        self.tab.inputs[6].setValue(42)
        self.tab.save.click()
        self.assertTrue(self.panel.commands[-1].endswith(' 6 2 42'))
        self.answer(42)
        self.assertTrue(self.panel.commands[-1].endswith(' 6 0 0'))
        self.answer(42)
        self.assertTrue(self.panel.commands[-1].endswith(' 6 3 0'))
        self.answer(42)
        self.assertIn('核对成功', self.tab.status.text())
        self.assertEqual(self.tab.table.item(5, 3).text(), '42')

    def test_every_writable_parameter_targets_edited_row_for_all_motors_and_actions(self):
        self.panel.resize(1250, 950)
        self.panel.show()
        self.tab.resize(1200, 900)
        self.tab.show()
        self.app.processEvents()
        values = {2: 1500, 3: 204800, 4: 102400, 5: 1, 6: 510,
                  7: 81, 8: 50, 9: 800, 10: 1, 12: 1}
        for motor in range(6):
            for parameter, value in values.items():
                for action in (1, 2):
                    for method in ('typing', 'arrow'):
                        with self.subTest(motor=motor + 1, parameter=parameter,
                                          action=action, method=method):
                            self.tab.motor.setCurrentIndex(motor)
                            box = self.tab.inputs[parameter]
                            self.tab.motor.setFocus()
                            box.setValue(value - 1)
                            # 从只读行切换到编辑器，同样必须正确启用并定位写入。
                            self.tab.table.selectRow(0)
                            if method == 'typing':
                                editor = box.lineEdit()
                                QtTest.QTest.mouseClick(editor, QtCore.Qt.LeftButton)
                                QtTest.QTest.keyClick(editor, QtCore.Qt.Key_A, QtCore.Qt.ControlModifier)
                                QtTest.QTest.keyClicks(editor, str(value))
                            else:
                                option = QtWidgets.QStyleOptionSpinBox()
                                box.initStyleOption(option)
                                arrow = box.style().subControlRect(QtWidgets.QStyle.CC_SpinBox,
                                    option, QtWidgets.QStyle.SC_SpinBoxUp, box)
                                QtTest.QTest.mouseClick(box, QtCore.Qt.LeftButton, pos=arrow.center())
                            button = self.tab.apply if action == 1 else self.tab.save
                            QtTest.QTest.mouseClick(button, QtCore.Qt.LeftButton)
                            self.assertEqual(self.panel.commands[-1].split()[2:],
                                             [str(motor + 1), str(parameter), str(action), str(value)])
                            self.answer(value)
                            self.assertEqual(self.panel.commands[-1].split()[2:],
                                             [str(motor + 1), str(parameter), '0', '0'])
                            self.answer(value)
                            if action == 2:
                                self.assertEqual(self.panel.commands[-1].split()[2:],
                                                 [str(motor + 1), str(parameter), '3', '0'])
                                self.answer(value)
                            self.assertIsNone(self.tab.pending)

    def test_keyboard_tab_between_gain_editors_selects_focused_parameter(self):
        self.panel.resize(1200, 900)
        self.panel.show()
        self.tab.resize(1100, 850)
        self.tab.show()
        self.app.processEvents()
        QtTest.QTest.mouseClick(self.tab.inputs[6].lineEdit(), QtCore.Qt.LeftButton)
        QtTest.QTest.keyClick(self.tab.inputs[6], QtCore.Qt.Key_Tab)
        self.assertEqual(self.tab.table.currentRow(), 6)
        self.tab.inputs[7].setValue(88)
        QtTest.QTest.mouseClick(self.tab.save, QtCore.Qt.LeftButton)
        self.assertEqual(self.panel.commands[-1].split()[2:], ['1', '7', '2', '88'])

    def test_programmatic_readback_does_not_change_selected_row(self):
        self.tab.table.selectRow(7)
        self.tab.read_all()
        for parameter, *_ in PARAMETERS:
            self.answer(1)
            self.answer(1)
            self.assertEqual(self.tab.table.currentRow(), 7)

    def test_inflight_read_prevents_editing_values_that_will_be_refreshed(self):
        self.tab.read.click()
        self.assertTrue(all(not box.isEnabled() for box in self.tab.inputs.values()))
        self.assertFalse(self.tab.motor.isEnabled())
        for _ in range(24):
            self.answer(1)
        self.assertTrue(all(box.isEnabled() for box in self.tab.inputs.values()))
        self.assertTrue(self.tab.motor.isEnabled())

    def test_previous_motor_response_cannot_populate_new_selection(self):
        self.tab.read.click()
        self.tab.motor.setCurrentIndex(4)
        self.answer(99)
        self.assertIsNone(self.tab.pending)
        self.assertFalse(self.tab.queue)
        self.assertEqual(self.tab.table.item(0, 2).text(), '—')
        self.assertEqual(len(self.panel.commands), 1)

    def test_stale_sequence_ignored_and_disconnect_clears_values(self):
        self.tab.read.click()
        self.tab.handle_line('MOTOR_PARAM_RESULT 999 1 1 0 88')
        self.assertEqual(self.tab.table.item(0, 2).text(), '—')
        self.answer(1)
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertIsNone(self.tab.pending)
        self.assertFalse(self.tab.queue)
        self.assertEqual(self.tab.table.item(0, 2).text(), '—')
        self.assertFalse(self.tab.read.isEnabled())

    def test_timeout_and_firmware_error_stop_batch(self):
        self.tab.read.click()
        with patch('manual_motor_parameters.time.monotonic', return_value=self.tab.pending['deadline'] + 1):
            self.tab.tick()
        self.assertIn('超时', self.tab.status.text())
        self.tab.read.click()
        self.answer(status=3)
        self.assertIn('固件不支持', self.tab.status.text())
        self.assertFalse(self.tab.queue)

    def test_hold_and_readonly_parameters_cannot_write(self):
        self.tab.table.selectRow(0)
        self.assertFalse(self.tab.apply.isEnabled())
        self.tab.table.selectRow(10)
        self.assertFalse(self.tab.save.isEnabled())
        self.panel.last_state['core_state'] = 4
        self.tab.tick()
        self.assertFalse(self.tab.read.isEnabled())
        self.tab.read_all()
        self.assertEqual(self.panel.commands, [])

    def test_reconnect_boot_discards_old_result(self):
        self.tab.read.click()
        old = self.tab.pending.copy()
        self.panel.last_state['boot_epoch'] = 8
        self.tab.tick()
        self.tab.handle_line(f'MOTOR_PARAM_RESULT {old["sequence"]} 1 1 0 9')
        self.assertEqual(self.tab.table.item(0, 2).text(), '—')
        self.assertIsNone(self.tab.pending)
