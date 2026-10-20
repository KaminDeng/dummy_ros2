"""真实 Qt 控件与假异步服务；不创建 ROS 节点或硬件连接。"""
import os
import ast
import sys
import unittest
import tempfile
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from PyQt5 import QtWidgets
import manual_moveit_tab as module


class Client:
    ready = True
    def service_is_ready(self):
        return self.ready


class Planner:
    def __init__(self, node, names, limits):
        self.names = names
        self.fk, self.ik, self.plan, self.validity = Client(), Client(), Client(), Client()
        self.calls = []

    def request(self, phase, *args):
        future = Future()
        self.calls.append((phase, args, future))
        return future

    def request_fk(self, *args):
        return self.request('fk', *args)

    def request_ik(self, *args):
        return self.request('ik', *args)

    def request_plan(self, *args):
        return self.request('plan', *args)

    def request_validity(self, *args):
        return self.request('validity', *args)


class Panel(QtWidgets.QWidget):
    def confirm_execution(self, title, text):
        return QtWidgets.QMessageBox.question(self, title, text,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)

    def __init__(self):
        super().__init__()
        self.node = None
        self.cfg = {'joints': [{'name': f'Joint{i}'} for i in range(1, 7)],
                    'moveit_continuous_execution': False,
                    'home_deg': [0, 0, 90, 0, 0, 0], 'rest_deg': [0, -60, 164, 0, 0, 0]}
        self.limits = [(-180, 180)] * 6
        self.actual_q = [0, 0, 1.5707963267948966, 0, 0, 0]
        self.last_state = {'boot_epoch': 7, 'reference_epoch': 2, 'session_id': 9,
                           'velocity_valid_mask': 63, 'estimated_velocity_rad_s': [0]*6,
                           'motion_epoch': 10, 'state_seq': 100}
        self.authorized, self.calibration, self.busy = True, {'valid': True}, False
        self.commands, self.releases = [], []
        self.report_dir = tempfile.TemporaryDirectory()
        self.report = Path(self.report_dir.name)
        self.speed_input, self.acceleration_input = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
        self.speed_input.setValue(17.2)
        self.acceleration_input.setValue(.4)
        self.is_fresh = True
        self.previews = []

    def control_state_allowed(self, states):
        return 4 in states

    def resume_position_issue(self):
        return None

    def command(self, value):
        self.commands.append(value)
        return True

    def release(self, value):
        self.releases.append(value)

    def fresh(self):
        return self.is_fresh

    def set_preview_pose(self, values):
        self.previews.append(values)


class MoveItTabTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.patch = patch.object(module, 'MoveItPlanner', Planner)
        self.patch.start()
        self.panel = Panel()
        self.tab = module.MoveItTab(self.panel)
        self.tab.timer.stop()

    def tearDown(self):
        self.panel.close()
        self.panel.deleteLater()
        self.panel.report_dir.cleanup()
        self.patch.stop()

    def test_buttons_follow_freshness_and_service_readiness(self):
        self.tab.tick()
        self.assertTrue(self.tab.capture.isEnabled())
        self.assertFalse(hasattr(self.tab, 'plan_button'))
        self.assertFalse(self.tab.play.isEnabled())
        self.tab.planner.ik.ready = False
        self.tab.tick()
        self.assertFalse(hasattr(self.tab, 'plan_button'))
        self.assertTrue(self.tab.capture.isEnabled())
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertFalse(self.tab.capture.isEnabled())

    def test_target_edit_cancels_pending_request(self):
        self.tab.start_plan()
        future = self.tab.pending[1]
        self.tab.inputs[0].setValue(5)
        self.tab.tick()
        self.assertTrue(future.cancelled())
        self.assertIsNone(self.tab.pending)
        self.assertIsNone(self.tab.path)

    def test_reference_change_cancels_pending_request(self):
        self.tab.capture_actual()
        future = self.tab.pending[1]
        self.panel.last_state['reference_epoch'] += 1
        self.tab.tick()
        self.assertTrue(future.cancelled())

    def test_timeout_cancels_request(self):
        self.tab.capture_actual()
        future = self.tab.pending[1]
        with patch.object(module.time, 'monotonic', return_value=self.tab.pending[2] + 1):
            self.tab.tick()
        self.assertTrue(future.cancelled())
        self.assertIn('超时', self.tab.status.text())

    def test_ik_unwraps_j6_before_planning_not_after_collision_check(self):
        import math
        self.panel.limits[5] = (-719.45, 719.45)
        self.panel.actual_q[5] = math.radians(-251.94790444)
        self.tab.start_plan()
        goal = list(self.panel.actual_q)
        goal[5] = math.radians(607.57155163)
        self.tab.pending[1].set_result(NS(error_code=NS(val=1), solution=NS(joint_state=NS(
            name=self.tab.planner.names, position=goal))))
        self.tab.tick()
        phase, args, _ = self.tab.planner.calls[-1]
        self.assertEqual(phase, 'plan')
        self.assertAlmostEqual(math.degrees(args[1][5]), -112.42844837)
        self.assertEqual(self.panel.commands, [])

    def test_collision_check_pump_is_fast_and_stops_when_plan_is_invalidated(self):
        self.complete_plan()
        self.assertTrue(self.tab.collision_timer.isActive())
        self.assertLessEqual(self.tab.collision_timer.interval(), 5)
        self.tab.invalidate()
        self.assertFalse(self.tab.collision_timer.isActive())
        self.assertEqual(self.panel.commands, [])

    def test_closing_panel_cancels_fast_collision_pump_without_motion(self):
        self.complete_plan()
        self.panel.closing = True
        self.tab.poll_collision_checks()
        self.assertFalse(self.tab.collision_timer.isActive())
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.panel.commands, [])

    def test_ik_preference_has_tooltips_and_is_passed_to_solver(self):
        self.assertGreaterEqual(self.tab.ik_preference.count(), 5)
        for index in range(self.tab.ik_preference.count()):
            self.assertTrue(self.tab.ik_preference.itemData(index, module.QtCore.Qt.ToolTipRole))
        index = self.tab.ik_preference.findData('wrist_negative')
        self.tab.ik_preference.setCurrentIndex(index)
        self.tab.preview_timer.stop()
        self.tab.start_plan()
        self.assertEqual(self.tab.planner.calls[-1][1][3], 'wrist_negative')
        self.assertGreater(self.tab.pending[2]-module.time.monotonic(), 1.)
        self.assertEqual(self.panel.commands, [])

    def test_random_pose_checks_endpoint_then_fk_then_plan_without_motion(self):
        self.tab.random_pose.click()
        self.assertEqual(self.tab.pending[0], 'random-validity')
        self.tab.pending[1].set_result(NS(valid=True))
        self.tab.tick()
        self.assertEqual(self.tab.pending[0], 'random-fk')
        pose = NS(position=NS(x=.1,y=.2,z=.3),orientation=NS(x=0.,y=0.,z=0.,w=1.))
        self.tab.pending[1].set_result(NS(error_code=NS(val=1),pose_stamped=[NS(pose=pose)]))
        self.tab.tick()
        self.assertEqual(self.tab.pending[0], 'plan')
        self.assertEqual([box.value() for box in self.tab.inputs[:3]], [100.,200.,300.])
        self.assertEqual(self.panel.commands, [])

    def test_random_colliding_candidate_is_retried_without_motion(self):
        self.tab.start_random_pose()
        first = self.tab.random_goal
        self.tab.pending[1].set_result(NS(valid=False))
        self.tab.tick()
        self.assertEqual(self.tab.pending[0], 'random-validity')
        self.assertNotEqual(first, self.tab.random_goal)
        self.assertEqual(self.panel.commands, [])

    def test_slow_checks_continue_after_one_second_without_authorizing_execution(self):
        self.complete_plan()
        waiting=list(self.tab.checks)
        with patch.object(module.time,'monotonic',return_value=self.tab.planning_started_at+1.01):
            self.tab.tick()
        self.assertIsNotNone(self.tab.path)
        self.assertTrue(all(not f.cancelled() for f in waiting))
        self.assertFalse(self.tab.execute.isEnabled())
        self.assertIn('继续',self.tab.status.text())
        self.assertEqual(self.panel.commands,[])

    def test_random_outside_sampleable_range_reports_reason_without_motion(self):
        self.panel.limits[1]=(-72.45,89.45)
        self.panel.actual_q[1]=3.
        self.tab.start_random_pose()
        self.assertIsNone(self.tab.pending)
        self.assertTrue(self.tab.target_failed)
        self.assertIn('范围',self.tab.status.text())
        self.assertEqual(self.panel.commands,[])

    def test_auto_test_snapshots_panel_parameters_and_manual_interrupt_remains_enabled(self):
        self.tab.amplitude.setCurrentIndex(2)
        self.tab.ik_preference.setCurrentIndex(3)
        with patch.object(QtWidgets.QMessageBox,'question',return_value=QtWidgets.QMessageBox.Yes):
            self.assertTrue(self.tab.auto_test.start(60.))
        session=self.tab.auto_test
        self.assertEqual((session.amplitude,session.preference),('large','wrist_negative'))
        self.assertEqual((session.speed,session.acceleration),(17.2,.4))
        self.tab.tick()
        self.assertTrue(self.tab.auto_interrupt.isEnabled())
        self.assertFalse(self.tab.amplitude.isEnabled())
        self.assertFalse(self.tab.ik_preference.isEnabled())
        self.tab.auto_interrupt.click()
        self.assertEqual(session.result,'interrupted')
        self.assertEqual(self.panel.releases,['STOP'])
        self.assertIn('#b45309',self.tab.auto_status.styleSheet())

    def test_confirmed_device_hold_does_not_wait_for_noisy_raw_velocity(self):
        state=self.panel.last_state
        state.update(core_state=4,hold_ready=1,hold_valid_mask=63,hold_position_rad=list(self.panel.actual_q))
        state['estimated_velocity_rad_s'][1]=.0282
        self.assertTrue(self.tab.ready())
        self.panel.actual_q[1]+=.004
        self.assertFalse(self.tab.ready())

    def test_auto_test_has_duration_and_only_one_start_confirmation(self):
        self.assertGreater(self.tab.auto_duration.value(),0.)
        with patch.object(QtWidgets.QMessageBox,'question',return_value=QtWidgets.QMessageBox.Yes) as question:
            self.assertTrue(self.tab.auto_test.start(60.))
            self.assertFalse(self.tab.auto_test.start(60.))
        self.assertEqual(question.call_count,1)
        self.assertTrue(self.panel.busy)
        self.assertEqual(self.tab.auto_test.stage,'validity')
        self.assertEqual(self.panel.commands,[])
        self.tab.auto_test.abort('测试清理',stop=False)

    def test_auto_expiry_cancels_prefetch_and_finishes_if_already_home(self):
        with patch.object(QtWidgets.QMessageBox,'question',return_value=QtWidgets.QMessageBox.Yes):
            self.tab.auto_test.start(1.)
        future=self.tab.auto_test.future
        with patch.object(module.time,'monotonic',return_value=self.tab.auto_test.deadline+.01):
            self.tab.tick()
        self.assertFalse(self.tab.auto_test.active)
        self.assertEqual(self.tab.auto_test.result,'completed')
        self.assertTrue(future.cancelled())
        self.assertEqual(self.panel.commands,[])

    def test_auto_test_error_exits_without_unchecked_home(self):
        with patch.object(QtWidgets.QMessageBox,'question',return_value=QtWidgets.QMessageBox.Yes):
            self.tab.auto_test.start(60.)
        self.tab.auto_test.future.set_result(NS(valid=True))
        self.panel.is_fresh=False
        self.tab.tick()
        self.assertFalse(self.tab.auto_test.active)
        self.assertEqual(self.panel.releases,['STOP'])
        self.assertEqual(self.panel.commands,[])

    def test_after_rest_calibrated_idle_allows_checked_moveit_execution(self):
        self.panel.last_state['core_state']=2
        self.panel.control_state_allowed=lambda states:2 in states
        self.complete_plan();self.validate_plan();self.tab.tick()
        self.assertTrue(self.tab.execute.isEnabled())
        self.assertEqual(self.panel.commands,[])
        self.panel.calibration=None
        self.tab.tick()
        self.assertFalse(self.tab.execute.isEnabled())

    def test_ik_failure_does_not_submit_plan(self):
        self.tab.start_plan()
        self.tab.pending[1].set_result(NS(error_code=NS(val=-31)))
        self.tab.tick()
        self.assertEqual([call[0] for call in self.tab.planner.calls], ['ik'])
        self.assertIsNone(self.tab.path)
        self.assertIn('-31', self.tab.status.text())

    def test_fk_populates_inputs_without_motion(self):
        self.tab.capture_actual()
        pose = NS(position=NS(x=.1, y=-.2, z=.3), orientation=NS(x=0, y=0, z=0, w=1))
        self.tab.pending[1].set_result(NS(error_code=NS(val=1), pose_stamped=[NS(pose=pose)]))
        self.tab.tick()
        self.assertEqual([box.value() for box in self.tab.inputs], [100, -200, 300, 0, 0, 0])
        self.assertEqual(self.panel.previews, [])

    def test_captured_pose_schedules_automatic_plan(self):
        self.tab.capture_actual()
        pose = NS(position=NS(x=.1, y=-.2, z=.3), orientation=NS(x=0, y=0, z=0, w=1))
        self.tab.pending[1].set_result(NS(error_code=NS(val=1), pose_stamped=[NS(pose=pose)]))
        self.tab.tick()
        self.assertTrue(self.tab.preview_timer.isActive())
        self.tab.preview_timer.stop()
        self.tab.preview_timer.timeout.emit()
        self.assertEqual(self.tab.pending[0], 'ik')
        self.assertEqual(self.panel.commands, [])

    def test_automatic_plan_waits_for_fresh_feedback_without_losing_target(self):
        self.tab.inputs[0].setValue(123)
        self.panel.is_fresh = False
        self.tab.preview_timer.stop()
        self.tab.preview_timer.timeout.emit()
        self.assertIsNone(self.tab.pending)
        self.assertTrue(self.tab.preview_timer.isActive())
        self.panel.is_fresh = True
        self.tab.preview_timer.stop()
        self.tab.preview_timer.timeout.emit()
        self.assertEqual(self.tab.pending[0], 'ik')
        self.assertEqual(self.tab.planner.calls[-1][1][1][0], 123)
        self.assertEqual(self.panel.commands, [])

    def test_pose_inputs_preserve_precision_for_near_singular_ik(self):
        for box in self.tab.inputs:
            box.setValue(1.2345)
            self.assertAlmostEqual(box.value(), 1.2345, places=4)

    def test_adopting_actual_pose_invalidates_old_plan_even_when_inputs_do_not_change(self):
        self.tab.path = (tuple(self.panel.actual_q), tuple(self.panel.actual_q))
        self.tab.capture_actual()
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.tab.pending[0], 'fk')

    def complete_plan(self):
        self.tab.start_plan()
        start = list(self.panel.actual_q)
        self.tab.pending[1].set_result(NS(error_code=NS(val=1), solution=NS(joint_state=NS(
            name=self.tab.planner.names, position=start))))
        self.tab.tick()
        self.assertEqual(self.tab.pending[0], 'plan')
        points = [NS(positions=start, time_from_start=NS(sec=i, nanosec=0)) for i in range(2)]
        trajectory = NS(joint_names=self.tab.planner.names, points=points)
        self.tab.pending[1].set_result(NS(motion_plan_response=NS(error_code=NS(val=1),
            trajectory=NS(joint_trajectory=trajectory))))
        self.tab.tick()
        self.assertEqual(len(self.tab.path), 2)

    def validate_plan(self):
        while self.tab.checks or self.tab.samples:
            for future in self.tab.checks:
                if not future.done():
                    future.set_result(NS(valid=True))
            self.tab.tick()

    def complete_single_point(self, delta=0.):
        self.tab.start_joint_plan([0, 0, 90, 0, 0, 0])
        point = list(self.panel.actual_q)
        point[0] += delta
        trajectory = NS(joint_names=self.tab.planner.names, points=[NS(
            positions=point, time_from_start=NS(sec=0, nanosec=0))])
        self.tab.pending[1].set_result(NS(motion_plan_response=NS(error_code=NS(val=1),
            trajectory=NS(joint_trajectory=trajectory))))
        self.tab.tick()

    def test_single_point_at_start_is_readonly_noop_with_preview(self):
        self.complete_single_point(5e-13)
        self.assertFalse(self.tab.target_failed)
        self.assertEqual(self.tab.noop_goal, tuple([5e-13] + self.panel.actual_q[1:]))
        self.assertIsNone(self.tab.path)
        self.assertFalse(self.tab.target_failed)
        self.assertIn('已在目标位置，无需运动', self.tab.status.text())
        self.tab.tick()
        self.assertTrue(self.tab.play.isEnabled())
        self.assertFalse(self.tab.execute.isEnabled())
        with patch.object(QtWidgets.QMessageBox, 'question') as question:
            self.tab.play.click()
            self.tab.start_execution()
            question.assert_not_called()
        self.assertEqual(self.panel.commands, [])
        self.assertEqual(self.tab.play_index, None)
        self.assertAlmostEqual(self.panel.previews[-1][0], module.math.degrees(5e-13))

    def test_single_point_away_from_start_is_rejected(self):
        self.complete_single_point(2e-12)
        self.assertIsNone(getattr(self.tab, 'noop_goal', None))
        self.assertIsNone(self.tab.path)
        self.assertTrue(self.tab.target_failed)
        self.assertIn('单点', self.tab.status.text())
        self.assertEqual(self.panel.commands, [])

    def test_noop_cleared_by_target_edit(self):
        self.complete_single_point()
        self.assertFalse(self.tab.target_failed)
        self.tab.inputs[0].setValue(123)
        self.assertIsNone(self.tab.noop_goal)
        self.assertFalse(self.tab.play.isEnabled())

    def test_noop_cleared_by_boot_or_reference_change(self):
        for field in ('boot_epoch', 'reference_epoch'):
            with self.subTest(field=field):
                self.complete_single_point()
                self.assertFalse(self.tab.target_failed)
                self.panel.last_state[field] += 1
                self.tab.tick()
                self.assertIsNone(self.tab.noop_goal)
                self.assertFalse(self.tab.play.isEnabled())
                self.assertFalse(self.tab.execute.isEnabled())

    def test_stale_noop_disables_preview_and_execution(self):
        self.complete_single_point()
        self.tab.tick()
        self.assertTrue(self.tab.play.isEnabled())
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertIsNone(self.tab.noop_goal)
        self.assertFalse(self.tab.play.isEnabled())
        self.assertFalse(self.tab.execute.isEnabled())
        self.assertTrue(self.tab.preview_timer.isActive())
        self.tab.start_execution()
        self.assertEqual(self.panel.commands, [])

    def preview_plan(self):
        self.validate_plan()
        self.tab.toggle_preview()
        with patch.object(module.time, 'monotonic', return_value=self.tab.preview_started + 2):
            self.tab.tick()
        self.assertTrue(self.tab.previewed)

    def prepare_execution(self):
        self.complete_plan()
        start = tuple(self.panel.actual_q)
        middle = (start[0]+.01,) + start[1:]
        end = (start[0]+.02,) + start[1:]
        self.tab.path = (start, middle, end)
        self.tab.times = (0., 1., 2.)
        for future in self.tab.checks:
            future.cancel()
        self.tab.checks = []
        self.tab.samples = list(module.collision_samples(self.tab.path))
        self.preview_plan()

    def confirm_execution(self):
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            self.tab.start_execution()
        self.assertEqual(len(self.panel.commands), 1)

    def test_ik_then_plan_and_preview_only(self):
        self.complete_plan()
        self.validate_plan()
        self.tab.tick()
        self.assertTrue(self.tab.play.isEnabled())
        self.tab.toggle_preview()
        self.tab.tick()
        self.assertEqual(len(self.panel.previews), 3)
        self.assertEqual([call[0] for call in self.tab.planner.calls if call[0] != 'validity'], ['ik', 'plan'])

    def test_completed_path_invalidated_by_actual_start_drift(self):
        self.complete_plan()
        self.panel.actual_q[0] += .004
        self.tab.tick()
        self.assertIsNone(self.tab.path)
        self.assertIn('起点', self.tab.status.text())

    def test_completed_path_invalidated_by_stale_feedback(self):
        self.complete_plan()
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertIsNone(self.tab.path)
        self.assertEqual(len(self.panel.previews), 2)

    def test_stale_validated_path_disables_buttons_and_replans_from_new_start(self):
        self.complete_plan()
        self.validate_plan()
        self.tab.tick()
        self.assertTrue(self.tab.play.isEnabled())
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertIsNone(self.tab.path)
        self.assertFalse(self.tab.play.isEnabled())
        self.assertFalse(self.tab.execute.isEnabled())
        self.assertTrue(self.tab.preview_timer.isActive())
        self.tab.start_execution()
        self.assertEqual(self.panel.commands, [])
        self.panel.actual_q[0] += .004
        self.panel.is_fresh = True
        self.tab.preview_timer.stop()
        self.tab.preview_timer.timeout.emit()
        self.assertEqual(self.tab.pending[0], 'ik')
        self.assertEqual(self.tab.pending[5], tuple(self.panel.actual_q))
        self.assertIsNone(self.tab.path)

    def test_stale_ik_and_plan_requests_are_cancelled_then_restarted(self):
        for phase in ('ik', 'plan'):
            with self.subTest(phase=phase):
                self.panel.is_fresh = True
                self.tab.start_plan()
                if phase == 'plan':
                    self.tab.pending[1].set_result(NS(error_code=NS(val=1), solution=NS(joint_state=NS(
                        name=self.tab.planner.names, position=list(self.panel.actual_q)))))
                    self.tab.tick()
                future = self.tab.pending[1]
                self.panel.is_fresh = False
                self.tab.tick()
                self.assertTrue(future.cancelled())
                self.assertIsNone(self.tab.pending)
                self.assertIsNone(self.tab.path)
                self.assertTrue(self.tab.preview_timer.isActive())
                self.panel.is_fresh = True
                self.tab.preview_timer.stop()
                self.tab.preview_timer.timeout.emit()
                self.assertEqual(self.tab.pending[0], 'ik')
                self.assertIsNot(self.tab.pending[1], future)
                self.tab.pending[1].cancel()
                self.tab.pending = None
        self.assertEqual(self.panel.commands, [])

    def test_stale_capture_does_not_plan_from_old_inputs(self):
        self.tab.capture_actual()
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertIsNone(self.tab.pending)
        self.assertFalse(self.tab.preview_timer.isActive())
        self.assertEqual([call[0] for call in self.tab.planner.calls], ['fk'])

    def test_changed_reference_does_not_retry_old_plan_request(self):
        self.tab.start_plan()
        future = self.tab.pending[1]
        self.panel.last_state['reference_epoch'] += 1
        self.tab.tick()
        self.assertTrue(future.cancelled())
        self.assertIsNone(self.tab.pending)
        self.assertFalse(self.tab.preview_timer.isActive())

    def test_changed_reference_does_not_retry_old_path(self):
        self.complete_plan()
        self.panel.last_state['reference_epoch'] += 1
        self.tab.tick()
        self.assertIsNone(self.tab.path)
        self.assertFalse(self.tab.preview_timer.isActive())

    def test_skip_confirmation_executes_validated_moveit_path_without_dialog(self):
        from manual_joint_panel import JointPanel
        self.prepare_execution()
        self.validate_plan()
        self.panel.skip_confirmations = QtWidgets.QCheckBox()
        self.panel.skip_confirmations.setChecked(True)
        self.panel.message = lambda text: None
        self.panel.confirm_execution = lambda title, text: JointPanel.confirm_execution(self.panel, title, text)
        with patch.object(QtWidgets.QMessageBox, 'question') as question:
            self.tab.start_execution()
            question.assert_not_called()
        self.assertEqual(len(self.panel.commands), 1)

    def test_unvalidated_path_never_submits_but_preview_is_optional(self):
        self.complete_plan()
        self.tab.previewed = True
        with patch.object(QtWidgets.QMessageBox, 'question') as question:
            self.tab.start_execution()
            question.assert_not_called()
        self.assertEqual(self.panel.commands, [])
        self.validate_plan()
        self.tab.previewed = False
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes) as question:
            self.tab.start_execution()
            question.assert_called_once()
        self.assertEqual(len(self.panel.commands), 1)

    def test_ordinary_busy_blocks_before_confirmation(self):
        self.prepare_execution()
        self.panel.busy = True
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes) as question:
            self.tab.start_execution()
            question.assert_not_called()
        self.assertEqual(self.panel.commands, [])
        self.assertFalse(self.tab.executing)

    def test_busy_during_confirmation_blocks_submission(self):
        self.prepare_execution()
        def answer(*args):
            self.panel.busy = True
            return QtWidgets.QMessageBox.Yes
        with patch.object(QtWidgets.QMessageBox, 'question', side_effect=answer) as question:
            self.tab.start_execution()
            question.assert_called_once()
        self.assertEqual(self.panel.commands, [])
        self.assertFalse(self.tab.executing)

    def test_confirmation_no_never_submits(self):
        self.prepare_execution()
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.No):
            self.tab.start_execution()
        self.assertEqual(self.panel.commands, [])
        self.assertFalse(self.tab.executing)

    def test_request_button_stays_available_during_fresh_velocity_crossings(self):
        self.prepare_execution()
        self.validate_plan()
        for velocity in (.009, .011, -.012, .008):
            self.panel.last_state['estimated_velocity_rad_s'] = [velocity] + [0.] * 5
            self.tab.tick()
            self.assertTrue(self.tab.execute.isEnabled())
        self.panel.last_state['velocity_valid_mask'] = 0
        self.tab.tick()
        self.assertFalse(self.tab.execute.isEnabled())
        self.panel.last_state['velocity_valid_mask'] = 63
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertFalse(self.tab.execute.isEnabled())

    def test_20ms_samples_keep_request_stable_but_real_250ms_age_disables(self):
        self.prepare_execution()
        self.validate_plan()
        source = Path(__file__).resolve().parents[1] / 'scripts/manual_joint_panel.py'
        fresh = next(n for n in ast.walk(ast.parse(source.read_text()))
                     if isinstance(n, ast.FunctionDef) and n.name == 'fresh')
        scope = dict(time=module.time)
        exec(compile(ast.Module(body=[fresh], type_ignores=[]), str(source), 'exec'), scope)
        self.panel.last_state['can_rx_age_us'] = [22000] * 6
        self.panel.fresh = lambda: scope['fresh'](self.panel)
        clock = [1_000_000_000]
        with patch.object(module.time, 'monotonic_ns', side_effect=lambda: clock[0]):
            for milliseconds in range(0, 1001, 20):
                clock[0] = 1_000_000_000 + milliseconds * 1_000_000
                self.panel.last_state_ns = clock[0]
                velocity = .011 if milliseconds % 40 else .009
                self.panel.last_state['estimated_velocity_rad_s'] = [velocity] + [0.] * 5
                self.tab.tick()
                self.assertTrue(self.tab.execute.isEnabled())
            clock[0] += 251_000_000
            self.tab.tick()
            self.assertFalse(self.tab.execute.isEnabled())

    def test_confirmed_request_waits_for_original_velocity_guard_before_submission(self):
        self.prepare_execution()
        self.panel.last_state['estimated_velocity_rad_s'] = [.02] + [0.] * 5
        with patch.object(module.time, 'monotonic', return_value=10.), patch.object(
                QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            self.tab.start_execution()
        self.assertTrue(self.tab.executing)
        self.assertEqual(self.panel.commands, [])
        self.panel.last_state['estimated_velocity_rad_s'] = [.009] + [0.] * 5
        with patch.object(module.time, 'monotonic', return_value=11.):
            self.tab.tick()
        self.assertEqual(len(self.panel.commands), 1)
        self.assertEqual(self.tab.segment_deadline, 671.)

    def test_unsubmitted_request_timeout_cancels_without_stop_or_authority_loss(self):
        self.prepare_execution()
        self.panel.last_state['estimated_velocity_rad_s'] = [.02] + [0.] * 5
        with patch.object(module.time, 'monotonic', return_value=10.), patch.object(
                QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            self.tab.start_execution()
        self.assertTrue(self.tab.executing)
        self.assertEqual(self.panel.commands, [])
        with patch.object(module.time, 'monotonic', return_value=13.):
            self.tab.tick()
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.commands, [])
        self.assertEqual(self.panel.releases, [])
        self.assertTrue(self.panel.authorized)

    def test_stale_pending_request_still_stops_even_when_prepare_window_expired(self):
        self.prepare_execution()
        self.panel.last_state['estimated_velocity_rad_s'] = [.02] + [0.] * 5
        with patch.object(module.time, 'monotonic', return_value=10.), patch.object(
                QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            self.tab.start_execution()
        self.assertTrue(self.tab.executing)
        self.panel.is_fresh = False
        with patch.object(module.time, 'monotonic', return_value=13.):
            self.tab.tick()
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.commands, [])
        self.assertEqual(self.panel.releases, ['STOP'])

    def test_segment_watchdog_uses_public_660_second_budget(self):
        self.prepare_execution()
        with patch.object(module.time, 'monotonic', return_value=10.):
            self.confirm_execution()
        self.assertEqual(self.tab.segment_deadline, 670.)
        with patch.object(module.time, 'monotonic', return_value=111.):
            self.tab.tick()
        self.assertTrue(self.tab.executing)
        self.assertEqual(self.panel.releases, [])
        with patch.object(module.time, 'monotonic', return_value=671.):
            self.tab.tick()
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, ['STOP'])

    def test_invalid_segment_watchdog_is_rejected_before_execution(self):
        for value in (0, -1, float('inf'), float('nan'), 661, True, '660'):
            with self.subTest(value=value):
                self.panel.cfg['moveit_segment_timeout_s'] = value
                with self.assertRaises(ValueError):
                    module.MoveItTab(self.panel)
        self.assertEqual(self.panel.commands, [])

    def test_prestart_policy_rejection_discards_path_without_stop(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_REJECTED_POLICY', dict(boot='7', reference='2', session='9', motion='10', state_seq='100'))
        self.assertFalse(self.tab.executing)
        self.assertFalse(self.panel.busy)
        self.assertTrue(self.panel.authorized)
        self.assertEqual(self.panel.releases, [])
        self.tab.native_event('MOVE_COMPLETED', dict(boot='7', reference='2', session='9', motion='11', state_seq='101'))
        self.tab.advance_execution()
        self.assertEqual(len(self.panel.commands), 1)

    def test_autotest_retries_transient_prestart_not_ready_without_new_motion(self):
        self.prepare_execution();self.confirm_execution()
        session=self.tab.auto_test;session.active=True
        session._drop_planning= lambda:None
        self.tab.native_event('MOVE_NOT_READY', dict(boot='7',reference='2',session='9',motion='10',state_seq='100'))
        self.assertTrue(self.tab.executing)
        self.assertFalse(self.tab.execution.waiting)
        self.assertEqual(self.panel.releases,[])
        count=len(self.panel.commands)
        self.tab.advance_execution()
        self.assertEqual(len(self.panel.commands),count)
        with patch.object(module.time,'monotonic',return_value=self.tab.retry_not_before+.001):
            self.tab.advance_execution()
        self.assertEqual(len(self.panel.commands),count+1)
        session.active=False

    def test_prestart_not_ready_does_not_release_authority(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_NOT_READY', dict(boot='7', reference='2', session='9', motion='10', state_seq='100'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, [])
        self.assertTrue(self.panel.authorized)

    def test_policy_rejection_after_started_motion_still_stops(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_STARTED', dict(boot='7', reference='2', session='9', motion='11', state_seq='101'))
        self.tab.native_event('MOVE_REJECTED_POLICY', dict(boot='7', reference='2', session='9', motion='11', state_seq='102'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, ['STOP'])

    def test_not_ready_after_started_motion_still_stops(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_STARTED', dict(boot='7', reference='2', session='9', motion='11', state_seq='101'))
        self.tab.native_event('MOVE_NOT_READY', dict(boot='7', reference='2', session='9', motion='11', state_seq='102'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, ['STOP'])

    def test_rejection_with_changed_identity_still_stops(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_REJECTED_POLICY', dict(boot='7', reference='2', session='10', motion='10', state_seq='100'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, ['STOP'])

    def test_wrong_session_completion_aborts_without_next_segment(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_COMPLETED', dict(boot='7', reference='2', session='10', motion='11', state_seq='101'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(len(self.panel.commands), 1)
        self.assertEqual(self.panel.releases, ['STOP'])

    def test_stop_discards_queue_and_late_completion(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.cancel_execution('用户停止', stop=True)
        self.tab.native_event('MOVE_COMPLETED', dict(boot='7', reference='2', session='9', motion='11', state_seq='101'))
        self.tab.advance_execution()
        self.tab.tick()
        self.assertEqual(len(self.panel.commands), 1)
        self.assertEqual(self.panel.releases, ['STOP'])
        self.assertFalse(self.tab.executing)

    def test_collision_rejection_invalidates_path_and_never_submits(self):
        self.complete_plan()
        self.tab.checks[0].set_result(NS(valid=False))
        self.tab.tick()
        self.tab.start_execution()
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.panel.commands, [])

    def test_dense_bent_path_checks_and_executes_only_geometric_corners(self):
        self.complete_plan()
        start = tuple(self.panel.actual_q)
        first = tuple((start[0]+i*.001,)+start[1:] for i in range(101))
        second = tuple((start[0]+.1,start[1]+i*.001)+start[2:] for i in range(1,101))
        self.tab.path = first+second
        self.tab.times = tuple(i*.01 for i in range(201))
        for future in self.tab.checks:
            future.cancel()
        self.tab.checks = []
        expected = (start,first[-1],second[-1])
        self.tab.samples = list(module.collision_samples(module.execution_polyline(self.tab.path)))
        self.validate_plan()
        self.confirm_execution()
        self.assertEqual(self.tab.execution.points, expected)
        # 第一段只能提交到真实拐点，收到设备完成收据后才允许继续。
        self.assertEqual(self.tab.execution.index, 1)
        self.assertEqual(len(self.panel.commands), 1)
        self.assertTrue(self.tab.execution.waiting)
        self.assertEqual(self.panel.releases, [])

    def test_continuous_execution_submits_one_path_instead_of_dense_moves(self):
        self.tab.continuous_enabled = True
        self.complete_plan()
        self.validate_plan()
        self.confirm_execution()
        self.assertTrue(self.panel.commands[0].startswith('MOVE_PATH '))
        self.assertEqual(len(self.tab.execution.points), 2)
        self.assertTrue(self.tab.path_file.is_file())
        self.tab.native_event('MOVE_STARTED',dict(boot='7',reference='2',session='9',motion='11',state_seq='100'))
        self.assertIsNone(self.tab.path_file)
        self.assertEqual(len(self.panel.commands),1)

    def test_continuous_speed_change_requires_new_collision_validation(self):
        self.tab.continuous_enabled = True
        self.complete_plan()
        self.validate_plan()
        self.panel.speed_input.setValue(30.)
        self.tab.start_execution()
        self.assertEqual(self.panel.commands,[])
        self.assertIn('重新规划',self.tab.status.text())

    def test_autotest_rejects_interpolated_candidate_without_ending_session(self):
        from collections import deque
        with patch.object(QtWidgets.QMessageBox,'question',return_value=QtWidgets.QMessageBox.Yes):
            self.assertTrue(self.tab.auto_test.start(60.))
        session=self.tab.auto_test
        session.future.cancel();session.future=None;session.stage='checking'
        rejected=Future();rejected.set_result(NS(valid=False))
        outstanding=Future();session.checks=[rejected,outstanding]
        session.samples=deque([tuple(self.panel.actual_q)])
        session.tick()
        self.assertTrue(session.active)
        self.assertEqual(session.stage,'validity')
        self.assertEqual(session.attempts,2)
        self.assertTrue(outstanding.cancelled())
        self.assertEqual(self.panel.commands,[])
        self.assertEqual(self.panel.releases,[])
        session.active=False

    def test_joint_preset_skips_ik(self):
        self.tab.preset_buttons[0].click()
        self.assertEqual([call[0] for call in self.tab.planner.calls], ['fk', 'plan'])
        phase, args, _ = self.tab.planner.calls[1]
        self.assertEqual(phase, 'plan')
        self.assertAlmostEqual(args[1][2], 1.5707963267948966)

    def test_preview_never_opens_execution_confirmation(self):
        self.complete_plan()
        self.validate_plan()
        self.tab.tick()
        with patch.object(QtWidgets.QMessageBox, 'question') as question:
            self.tab.play.click()
            with patch.object(module.time, 'monotonic', return_value=self.tab.preview_started + 2):
                self.tab.tick()
            self.app.processEvents()
            self.assertTrue(self.tab.previewed)
            question.assert_not_called()
            self.assertEqual(self.panel.commands, [])

    def test_configurable_preview_speed(self):
        self.complete_plan()
        self.validate_plan()
        self.tab.play_speed.setValue(2.)
        self.tab.toggle_preview()
        with patch.object(module.time, 'monotonic', return_value=self.tab.preview_started + .6):
            self.tab.tick()
        self.assertTrue(self.tab.previewed)
        self.assertEqual(self.panel.commands, [])

    def test_increment_slider_release_keeps_position(self):
        self.tab.inputs[0].setValue(100.)
        slider = self.tab.jog_sliders[0]
        slider.setSliderDown(True)
        slider.setValue(50)
        self.tab.advance_jog()
        self.tab.advance_jog()
        self.assertAlmostEqual(self.tab.inputs[0].value(), 100.+.01*self.tab.step.value())
        slider.setSliderDown(False)
        self.assertEqual(slider.value(), 0)
        self.assertAlmostEqual(self.tab.inputs[0].value(), 100.+.01*self.tab.step.value())
        self.assertFalse(self.tab.jog_timer.isActive())
        self.assertEqual(self.panel.commands, [])

    def test_planning_failure_marks_preview_red(self):
        flags = []
        self.panel.set_preview_failed = flags.append
        self.tab.start_plan()
        self.tab.pending[1].set_result(NS(error_code=NS(val=-31)))
        self.tab.tick()
        self.assertTrue(self.tab.target_failed)
        self.assertTrue(flags[-1])
        self.assertEqual(self.panel.commands, [])

    def test_input_debounce_ik_updates_ghost_without_hardware_command(self):
        self.tab.inputs[0].setValue(123)
        self.assertTrue(self.tab.preview_timer.isActive())
        self.assertEqual(self.tab.planner.calls, [])
        self.tab.preview_timer.stop()
        self.tab.preview_timer.timeout.emit()
        self.assertEqual(self.tab.pending[0], 'ik')
        self.assertEqual(self.tab.preview_timer.interval(), 100)
        phase, args, future = self.tab.planner.calls[-1]
        self.assertEqual(phase, 'ik')
        self.assertEqual(args[1][0], 123)
        future.set_result(NS(error_code=NS(val=1), solution=NS(joint_state=NS(
            name=self.tab.planner.names, position=list(self.panel.actual_q)))))
        self.tab.tick()
        self.assertEqual(len(self.panel.previews), 1)
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.panel.commands, [])
        self.assertEqual([call[0] for call in self.tab.planner.calls], ['ik', 'plan'])

    def test_prebegin_path_rejection_keeps_existing_hold(self):
        self.prepare_execution();self.confirm_execution()
        self.tab.native_event('MOVE_REJECTED_PATH',dict(boot='7',reference='2',session='9',motion='10',state_seq='100'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases,[])
        self.assertTrue(self.panel.authorized)

    def test_scene_update_invalidates_plan_and_autotest_without_releasing(self):
        self.prepare_execution();self.confirm_execution()
        self.tab.scene_change_requested()
        self.assertFalse(self.tab.executing)
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.panel.releases,[])

    def test_unconfirmed_scene_blocks_new_execution_and_planning(self):
        self.prepare_execution()
        self.panel.obstacle_tab=NS(scene_ready=False,blocked=True,scene_revision=1)
        self.tab.start_execution(confirmed=True)
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.commands,[])
        f=Future();self.tab.begin('plan',f,self.panel.actual_q)
        self.assertTrue(f.cancelled())


if __name__ == '__main__':
    unittest.main()
