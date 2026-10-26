"""真实 Qt 控件与假异步服务；不创建 ROS 节点或硬件连接。"""
##
# @file test_manual_moveit_tab.py
# @brief 真实Qt MoveIt页面与手工Future的规划、虚影、执行及失效矩阵。
#
# 真实MoveItTab控件/定时器被直接tick，Planner/Panel是测试替身，没有ROS节点或Native设备。path/actual_q为rad，姿态输入xyz mm/rpy deg，speed deg/s、accel rad/s²；monotonic期限秒，State新鲜度monotonic_ns为ns。命令列表/事件均为模拟边界，不能把断言执行成功视为机械完成。
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


##
# @brief 只有ready布尔的服务可用性替身，不提供实际ROS client。
class Client:
    ready = True
    ##
    # @brief 返回人工ready字段，模拟单服务可用性而非发现图唯一性。
    # @return ready原布尔。
    def service_is_ready(self):
        return self.ready


##
# @brief 按phase记录参数和Future的计划服务替身，生命周期覆盖被测页面。
class Planner:
    ##
    # @brief 保存names，创建四个Client及calls；node/limits不参与真实ROS构造。
    # @param node 传入节点，本替身忽略。
    # @param names 保存的关节名顺序。
    # @param limits 角度范围，本替身忽略。
    # @return None；只初始化本测试对象及替身字段。
    def __init__(self, node, names, limits):
        self.names = names
        self.fk, self.ik, self.plan, self.validity = Client(), Client(), Client(), Client()
        self.calls = []

    ##
    # @brief 记录(phase,args,Future)供测试定位并手动完成，不调任何服务。
    # @param phase fk/ik/plan/validity阶段标签。
    # @param args 调用参数原值，记录后供用例断言。
    # @return 新的concurrent.futures.Future，未完成。
    def request(self, phase, *args):
        future = Future()
        self.calls.append((phase, args, future))
        return future

    ##
    # @brief 将参数原样记为fk请求，返回未完成Future。
    # @param args 调用位置参数，原样记录或由本替身忽略。
    # @return 新的未完成Future，等待用例人工set_result。
    def request_fk(self, *args):
        return self.request('fk', *args)

    ##
    # @brief 将参数原样记为ik请求，返回未完成Future。
    # @param args 调用位置参数，原样记录或由本替身忽略。
    # @return 新的未完成Future，等待用例人工set_result。
    def request_ik(self, *args):
        return self.request('ik', *args)

    ##
    # @brief 将参数原样记为plan请求，返回未完成Future。
    # @param args 调用位置参数，原样记录或由本替身忽略。
    # @return 新的未完成Future，等待用例人工set_result。
    def request_plan(self, *args):
        return self.request('plan', *args)

    ##
    # @brief 将参数原样记为validity请求，返回未完成Future。
    # @param args 调用位置参数，原样记录或由本替身忽略。
    # @return 新的未完成Future，等待用例人工set_result。
    def request_validity(self, *args):
        return self.request('validity', *args)


##
# @brief 真实QWidget宿主，六轴反馈/授权和命令输出均为人工内存数据。
class Panel(QtWidgets.QWidget):
    ##
    # @brief 调用真实QMessageBox.question，测试用patch决定答案，默认按钮No。
    # @param title 确认框标题。
    # @param text 确认正文。
    # @return QMessageBox Yes/No枚举。
    def confirm_execution(self, title, text):
        return QtWidgets.QMessageBox.question(self, title, text,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)

    ##
    # @brief 创建真实QWidget、六轴人工状态/授权、速度输入和临时report；commands/releases/previews仅记录调用。
    # @return None；只初始化本测试对象及替身字段。
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
        self.backend = 'real'  # This fixture exercises the existing Native path.
        self.authorized, self.calibration, self.busy = True, {'valid': True}, False
        self.commands, self.releases = [], []
        self.report_dir = tempfile.TemporaryDirectory()
        self.report = Path(self.report_dir.name)
        self.speed_input, self.acceleration_input = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
        self.speed_input.setValue(17.2)
        self.acceleration_input.setValue(.4)
        self.is_fresh = True
        self.previews = []

    ##
    # @brief 仅判断调用方允许集合含4，模拟Hold；不读取真实core或授权。
    # @param states 调用者允许的core集合，本替身只查是否含4。
    # @return 4是否属于states。
    def control_state_allowed(self, states):
        return 4 in states

    ##
    # @brief 固定无位置问题，隔离主面板范围恢复逻辑。
    # @return None，固定无问题。
    def resume_position_issue(self):
        return None

    ##
    # @brief 记录命令并固定True，不启动Native或等待设备接受。
    # @param value 模拟Native命令字符串。
    # @return True，固定提交声明。
    def command(self, value):
        self.commands.append(value)
        return True

    ##
    # @brief 只记录STOP等字符串，不提供设备停止证明。
    # @param value STOP等释放原因字符串，只记录。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    def release(self, value):
        self.releases.append(value)

    ##
    # @brief 返回人工is_fresh，不以时钟计算；个别用例显式替换为生产fresh方法。
    # @return is_fresh原布尔。
    def fresh(self):
        return self.is_fresh

    ##
    # @brief 保存预览值列表，不覆盖actual_q或发布实体命令。
    # @param values 关节预览deg列表，只追加到previews。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    def set_preview_pose(self, values):
        self.previews.append(values)


##
# @brief 真实Qt页面配手工Future的状态机/准入/失效回归；无ROS/Native运动。
class MoveItTabTests(unittest.TestCase):
    ##
    # @brief 创建/复用真实QApplication供所有控件用例。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    ##
    # @brief patch生产MoveItPlanner为本文件Planner，构造Panel/MoveItTab并停主timer以手动推进。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    def setUp(self):
        self.patch = patch.object(module, 'MoveItPlanner', Planner)
        self.patch.start()
        self.panel = Panel()
        self.tab = module.MoveItTab(self.panel)
        self.tab.timer.stop()

    ##
    # @brief 关父控件、安排deleteLater、删临时report并撤planner patch；没有设备释放协议。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    def tearDown(self):
        self.panel.close()
        self.panel.deleteLater()
        self.panel.report_dir.cleanup()
        self.patch.stop()

    ##
    # @brief fresh控制捕获按钮，IK不可用不妨碍只读捕获；旧独立plan_button不存在，未验证路径不能播放。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_duplicate_graph_requests_leave_window_alive_without_motion(self):
        for method, request, args in (
                ('capture_actual', 'request_fk', ()),
                ('start_plan', 'request_ik', ()),
                ('start_joint_plan', 'request_fk', ([0.0] * 6,)),
                ('start_joint_plan', 'request_plan', ([0.0] * 6,))):
            with self.subTest(method=method, request=request):
                with patch.object(self.tab.planner, request, side_effect=ValueError('MoveIt服务必须各有一个实例 duplicate graph')):
                    getattr(self.tab, method)(*args)
                self.assertIsNone(self.tab.pending)
                self.assertIsNone(self.tab.preset_fk)
                self.assertFalse(self.tab.executing)
                self.assertIn('duplicate graph', self.tab.status.text())
                self.assertFalse(self.tab.preview_timer.isActive())
                self.assertFalse(self.tab.collision_timer.isActive())
                self.assertFalse(self.panel.busy)

    def test_preset_fk_future_graph_error_stays_inside_timer(self):
        future = Future()
        future.set_exception(ValueError('duplicate graph async FK'))
        self.tab.preset_fk = (future, self.tab.revision)
        self.tab.tick()
        self.assertIsNone(self.tab.pending)
        self.assertIsNone(self.tab.preset_fk)
        self.assertIn('duplicate graph async FK', self.tab.status.text())
        self.assertFalse(self.tab.executing)
        self.assertFalse(self.panel.busy)

    def prepare_ros_inflight(self):
        self.panel.backend='ros'
        self.ros_cancels=[]
        self.panel.ros_backend=NS(cancel=lambda:self.ros_cancels.append('cancel'))
        self.tab.execution=NS(done=False,aborted=False,waiting=True,
            identity=self.tab.execution_identity(),abort=lambda:None)
        self.tab.segment_deadline=module.time.monotonic()+30.
        self.panel.busy=True

    def test_ros_inflight_complete_transition_waits_for_action_without_python_cancel(self):
        self.prepare_ros_inflight()
        self.panel.authorized=False
        for core in (3,7,4):
            self.panel.last_state.update(core_state=core,real_motion_permitted=False,permit_remaining_us=0)
            self.tab.tick()
            self.assertTrue(self.tab.executing)
            self.assertEqual(self.ros_cancels,[])
            self.assertTrue(self.panel.busy)
        self.tab.path=(tuple(self.panel.actual_q),tuple(self.panel.actual_q))
        self.tab.start=tuple(self.panel.actual_q)
        self.tab.ros_event('completed',{'goal_uuid':'bound-goal'})
        self.panel.actual_q[0]+=.02
        self.tab.tick()
        self.assertIsNone(self.tab.path)
        self.assertIn('已确认整轨迹完成',self.tab.status.text())
        self.assertFalse(self.tab.executing)
        self.assertFalse(self.panel.busy)
        self.assertEqual(self.ros_cancels,[])

    def test_ros_inflight_identity_change_and_real_deadline_still_cancel(self):
        for field in ('boot_epoch','reference_epoch','deadline'):
            with self.subTest(field=field):
                self.prepare_ros_inflight()
                if field=='deadline': self.tab.segment_deadline=module.time.monotonic()-1.
                else:self.panel.last_state[field]+=1
                self.tab.tick()
                self.assertFalse(self.tab.executing)
                self.assertEqual(self.ros_cancels,['cancel'])

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

    ##
    # @brief 目标编辑使在途Future取消并清pending/path，旧响应不得继续。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_target_edit_cancels_pending_request(self):
        self.tab.start_plan()
        future = self.tab.pending[1]
        self.tab.inputs[0].setValue(5)
        self.tab.tick()
        self.assertTrue(future.cancelled())
        self.assertIsNone(self.tab.pending)
        self.assertIsNone(self.tab.path)

    ##
    # @brief 参考epoch改变使捕获请求失效并取消Future。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_reference_change_cancels_pending_request(self):
        self.tab.capture_actual()
        future = self.tab.pending[1]
        self.panel.last_state['reference_epoch'] += 1
        self.tab.tick()
        self.assertTrue(future.cancelled())

    ##
    # @brief 注入超过pending期限的时刻，取消Future并显示超时。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_timeout_cancels_request(self):
        self.tab.capture_actual()
        future = self.tab.pending[1]
        with patch.object(module.time, 'monotonic', return_value=self.tab.pending[2] + 1):
            self.tab.tick()
        self.assertTrue(future.cancelled())
        self.assertIn('超时', self.tab.status.text())

    ##
    # @brief 实际−251.95°时IK607.57°先转等价−112.43°再发plan；不在碰撞检查后改圈。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 规划后检查timer周期≤5ms；invalidate停止检查泵且不发运动。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_collision_check_pump_is_fast_and_stops_when_plan_is_invalidated(self):
        self.complete_plan()
        self.assertTrue(self.tab.collision_timer.isActive())
        self.assertLessEqual(self.tab.collision_timer.interval(), 5)
        self.tab.invalidate()
        self.assertFalse(self.tab.collision_timer.isActive())
        self.assertEqual(self.panel.commands, [])

    ##
    # @brief closing时停检查timer、清path，不提交运动。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_closing_panel_cancels_fast_collision_pump_without_motion(self):
        self.complete_plan()
        self.panel.closing = True
        self.tab.poll_collision_checks()
        self.assertFalse(self.tab.collision_timer.isActive())
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.panel.commands, [])

    ##
    # @brief 至少五偏好均有tooltip，负腕选项传给IK且请求期限>1s；仅核调用参数。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 随机候选先validity，再FK把m转mm填姿态，再plan；整个阶段不发运动。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 候选valid=false后换目标继续validity，不沿碰撞候选执行。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_random_colliding_candidate_is_retried_without_motion(self):
        self.tab.start_random_pose()
        first = self.tab.random_goal
        self.tab.pending[1].set_result(NS(valid=False))
        self.tab.tick()
        self.assertEqual(self.tab.pending[0], 'random-validity')
        self.assertNotEqual(first, self.tab.random_goal)
        self.assertEqual(self.panel.commands, [])

    ##
    # @brief 超过1s仍保留检查Future继续等待，execute保持禁用而不是把慢当通过。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 实际J2=3rad超范围时不建请求，标目标失败并解释范围。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_random_outside_sampleable_range_reports_reason_without_motion(self):
        self.panel.limits[1]=(-72.45,89.45)
        self.panel.actual_q[1]=3.
        self.tab.start_random_pose()
        self.assertIsNone(self.tab.pending)
        self.assertTrue(self.tab.target_failed)
        self.assertIn('范围',self.tab.status.text())
        self.assertEqual(self.panel.commands,[])

    ##
    # @brief 自动会话冻结large/负腕及17.2deg/s、0.4rad/s²；运行时锁选项但中断可用并请求STOP。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 已确认Hold位置/全mask时速度噪声0.0282rad/s不单独阻塞ready，但位置漂移0.004rad仍拒绝。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_confirmed_device_hold_does_not_wait_for_noisy_raw_velocity(self):
        state=self.panel.last_state
        state.update(core_state=4,hold_ready=1,hold_valid_mask=63,hold_position_rad=list(self.panel.actual_q))
        state['estimated_velocity_rad_s'][1]=.0282
        self.assertTrue(self.tab.ready())
        self.panel.actual_q[1]+=.004
        self.assertFalse(self.tab.ready())

    ##
    # @brief 正时长会话只确认一次且第二start拒绝，置busy/validity但不立即发运动。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 已在Home的会话期限到取消预取Future后completed，不做额外回Home动作。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 自动会话反馈过期即结束并STOP，不能未经检查自动回Home。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_auto_test_error_exits_without_unchecked_home(self):
        with patch.object(QtWidgets.QMessageBox,'question',return_value=QtWidgets.QMessageBox.Yes):
            self.tab.auto_test.start(60.)
        self.tab.auto_test.future.set_result(NS(valid=True))
        self.panel.is_fresh=False
        self.tab.tick()
        self.assertFalse(self.tab.auto_test.active)
        self.assertEqual(self.panel.releases,['STOP'])
        self.assertEqual(self.panel.commands,[])

    ##
    # @brief 人工Idle2且已有参考/授权允许已检路径；清calibration立即禁execute。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_after_rest_calibrated_idle_allows_checked_moveit_execution(self):
        self.panel.last_state['core_state']=2
        self.panel.control_state_allowed=lambda states:2 in states
        self.complete_plan();self.validate_plan();self.tab.tick()
        self.assertTrue(self.tab.execute.isEnabled())
        self.assertEqual(self.panel.commands,[])
        self.panel.calibration=None
        self.tab.tick()
        self.assertFalse(self.tab.execute.isEnabled())

    ##
    # @brief IK−31终止并解释错误，只出现ik调用，不建path或发plan。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_ik_failure_does_not_submit_plan(self):
        self.tab.start_plan()
        self.tab.pending[1].set_result(NS(error_code=NS(val=-31)))
        self.tab.tick()
        self.assertEqual([call[0] for call in self.tab.planner.calls], ['ik'])
        self.assertIsNone(self.tab.path)
        self.assertIn('-31', self.tab.status.text())

    ##
    # @brief FK米坐标转成100/−200/300mm及零姿态输入，不直接改关节虚影列表。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_fk_populates_inputs_without_motion(self):
        self.tab.capture_actual()
        pose = NS(position=NS(x=.1, y=-.2, z=.3), orientation=NS(x=0, y=0, z=0, w=1))
        self.tab.pending[1].set_result(NS(error_code=NS(val=1), pose_stamped=[NS(pose=pose)]))
        self.tab.tick()
        self.assertEqual([box.value() for box in self.tab.inputs], [100, -200, 300, 0, 0, 0])
        self.assertEqual(self.panel.previews, [])

    ##
    # @brief 捕获FK后触发debounce timer，再手动timeout进入IK；不发设备命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 目标123mm在反馈旧时保留并重排timer，新鲜后用该目标请求IK。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 六输入框保存1.2345四位精度，避免量化过粗影响近奇异IK。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_pose_inputs_preserve_precision_for_near_singular_ik(self):
        for box in self.tab.inputs:
            box.setValue(1.2345)
            self.assertAlmostEqual(box.value(), 1.2345, places=4)

    ##
    # @brief capture_actual即使姿态值看似未变也清旧path并请求FK。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_adopting_actual_pose_invalidates_old_plan_even_when_inputs_do_not_change(self):
        self.tab.path = (tuple(self.panel.actual_q), tuple(self.panel.actual_q))
        self.tab.capture_actual()
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.tab.pending[0], 'fk')

    ##
    # @brief 手工完成IK和两点同位置计划，建立path但保留待碰撞检查；不是服务真的成功。
    # @return None；执行上述夹具装配、手动状态推进或清理。
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

    ##
    # @brief 循环将当前所有validity Future手工置valid=True并tick，直到samples/checks清空。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    def validate_plan(self):
        while self.tab.checks or self.tab.samples:
            for future in self.tab.checks:
                if not future.done():
                    future.set_result(NS(valid=True))
            self.tab.tick()

    ##
    # @brief 构造单点计划，首轴在actual上加delta，用于核只读noop容差而非运动执行。
    # @param delta 第一轴相对actual的偏差rad，默认0。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    def complete_single_point(self, delta=0.):
        self.tab.start_joint_plan([0, 0, 90, 0, 0, 0])
        point = list(self.panel.actual_q)
        point[0] += delta
        trajectory = NS(joint_names=self.tab.planner.names, points=[NS(
            positions=point, time_from_start=NS(sec=0, nanosec=0))])
        self.tab.pending[1].set_result(NS(motion_plan_response=NS(error_code=NS(val=1),
            trajectory=NS(joint_trajectory=trajectory))))
        self.tab.tick()

    ##
    # @brief 偏差5e−13rad单点只可预览，不能execute/弹确认/发命令，提示已在目标无需运动。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 偏差2e−12rad单点超过noop门，清path/noop并标单点失败。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_single_point_away_from_start_is_rejected(self):
        self.complete_single_point(2e-12)
        self.assertIsNone(getattr(self.tab, 'noop_goal', None))
        self.assertIsNone(self.tab.path)
        self.assertTrue(self.tab.target_failed)
        self.assertIn('单点', self.tab.status.text())
        self.assertEqual(self.panel.commands, [])

    ##
    # @brief 修改目标立即清noop并禁play，不沿旧只读结果显示。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_noop_cleared_by_target_edit(self):
        self.complete_single_point()
        self.assertFalse(self.tab.target_failed)
        self.tab.inputs[0].setValue(123)
        self.assertIsNone(self.tab.noop_goal)
        self.assertFalse(self.tab.play.isEnabled())

    ##
    # @brief 分别变boot/ref撤noop，play和execute均禁用。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 过期反馈撤noop并安排重规划，主动start_execution也不发命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 先人工全路径valid再播放，注入2s时间推进并断言previewed；只更新虚影。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    def preview_plan(self):
        self.validate_plan()
        self.tab.toggle_preview()
        with patch.object(module.time, 'monotonic', return_value=self.tab.preview_started + 2):
            self.tab.tick()
        self.assertTrue(self.tab.previewed)

    ##
    # @brief 建立三点J1 0/0.01/0.02rad路径与0/1/2s时间，重做碰撞采样并人工验证预览。
    # @return None；执行上述夹具装配、手动状态推进或清理。
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

    ##
    # @brief patch真实确认框Yes后调用start_execution，要求只记录一条模拟命令。
    # @return None；执行上述夹具装配、手动状态推进或清理。
    def confirm_execution(self):
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            self.tab.start_execution()
        self.assertEqual(len(self.panel.commands), 1)

    ##
    # @brief 手工IK/plan及validity后只播放虚影，核非validity调用顺序为IK→plan。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_ik_then_plan_and_preview_only(self):
        self.complete_plan()
        self.validate_plan()
        self.tab.tick()
        self.assertTrue(self.tab.play.isEnabled())
        self.tab.toggle_preview()
        self.tab.tick()
        self.assertEqual(len(self.panel.previews), 3)
        self.assertEqual([call[0] for call in self.tab.planner.calls if call[0] != 'validity'], ['ik', 'plan'])

    ##
    # @brief actual首轴漂移0.004rad撤path并提示起点，不沿旧起点执行。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_completed_path_invalidated_by_actual_start_drift(self):
        self.complete_plan()
        self.panel.actual_q[0] += .004
        self.tab.tick()
        self.assertIsNone(self.tab.path)
        self.assertIn('起点', self.tab.status.text())

    ##
    # @brief 反馈旧时撤path，保留既有预览计数语义，不再授权执行。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_completed_path_invalidated_by_stale_feedback(self):
        self.complete_plan()
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertIsNone(self.tab.path)
        self.assertEqual(len(self.panel.previews), 2)

    ##
    # @brief 已检路径遇旧反馈清除/禁播放执行，新鲜后IK使用新actual起点而非旧path。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief IK和plan各在过期时取消并清pending，恢复新鲜后建新Future从IK重来，不复用旧请求。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 捕获FK尚未完成就反馈旧，只取消捕获，不自动用旧输入规划。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_stale_capture_does_not_plan_from_old_inputs(self):
        self.tab.capture_actual()
        self.panel.is_fresh = False
        self.tab.tick()
        self.assertIsNone(self.tab.pending)
        self.assertFalse(self.tab.preview_timer.isActive())
        self.assertEqual([call[0] for call in self.tab.planner.calls], ['fk'])

    ##
    # @brief ref改变取消在途规划且不设重试timer，旧目标不跨参考续用。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_changed_reference_does_not_retry_old_plan_request(self):
        self.tab.start_plan()
        future = self.tab.pending[1]
        self.panel.last_state['reference_epoch'] += 1
        self.tab.tick()
        self.assertTrue(future.cancelled())
        self.assertIsNone(self.tab.pending)
        self.assertFalse(self.tab.preview_timer.isActive())

    ##
    # @brief ref改变清已完成path且不自动重排旧路径。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_changed_reference_does_not_retry_old_path(self):
        self.complete_plan()
        self.panel.last_state['reference_epoch'] += 1
        self.tab.tick()
        self.assertIsNone(self.tab.path)
        self.assertFalse(self.tab.preview_timer.isActive())

    ##
    # @brief 接入真实JointPanel确认方法，skip已选时已检路径不弹框但仍只记一条命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief previewed标志不能代替碰撞检查；完成检查后无需先播放也可确认提交。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 普通运动busy时在弹框前拒绝，既不executing也不发命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_ordinary_busy_blocks_before_confirmation(self):
        self.prepare_execution()
        self.panel.busy = True
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes) as question:
            self.tab.start_execution()
            question.assert_not_called()
        self.assertEqual(self.panel.commands, [])
        self.assertFalse(self.tab.executing)

    ##
    # @brief 弹框回调置busy后虽Yes仍二次拒绝，防止确认期间抢占。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_busy_during_confirmation_blocks_submission(self):
        self.prepare_execution()
        ##
        # @par 内部回调 answer
        # 模拟确认框期间普通运动开始：置busy=True再返回Yes。
        # 参数 args：调用位置参数，原样记录或由本替身忽略。
        # 返回：QMessageBox.Yes，当前busy已改变。
        def answer(*args):
            self.panel.busy = True
            return QtWidgets.QMessageBox.Yes
        with patch.object(QtWidgets.QMessageBox, 'question', side_effect=answer) as question:
            self.tab.start_execution()
            question.assert_called_once()
        self.assertEqual(self.panel.commands, [])
        self.assertFalse(self.tab.executing)

    ##
    # @brief 用户No保持未执行，不写命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_confirmation_no_never_submits(self):
        self.prepare_execution()
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.No):
            self.tab.start_execution()
        self.assertEqual(self.panel.commands, [])
        self.assertFalse(self.tab.executing)

    ##
    # @brief 约±0.01rad/s噪声不抖动禁按钮，但velocity mask缺失或反馈过期仍禁用。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 提取真实fresh方法，以20ms人工更新持续可请求；停止更新251ms后禁execute，不以GUI tick冒充新State。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 确认时速度0.02rad/s先排executing不发送；降0.009后提交，段deadline从真实提交11s加660s=671s。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 速度未安静、准备窗口超时且未发命令时只撤请求，不STOP或撤已有授权。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 准备超时同时反馈过期优先按失鲜STOP，不能借未提交例外忽略失权。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 人工10s提交deadline670s，111s仍执行，671s超时STOP；不真实等待660s。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 0/负/非有限/>660、bool/字符串watchdog在构造时ValueError，无运动提交。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_invalid_segment_watchdog_is_rejected_before_execution(self):
        for value in (0, -1, float('inf'), float('nan'), 661, True, '660'):
            with self.subTest(value=value):
                self.panel.cfg['moveit_segment_timeout_s'] = value
                with self.assertRaises(ValueError):
                    module.MoveItTab(self.panel)
        self.assertEqual(self.panel.commands, [])

    ##
    # @brief 未MOVE_STARTED时同身份policy拒绝只撤请求保Hold授权，晚到Complete不能推进下一段。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 自动会话prestart NOT_READY不STOP，retry_not_before前不重发，过退避点才重发一次。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 普通prestart NOT_READY撤executing但保授权，无STOP。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_prestart_not_ready_does_not_release_authority(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_NOT_READY', dict(boot='7', reference='2', session='9', motion='10', state_seq='100'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, [])
        self.assertTrue(self.panel.authorized)

    ##
    # @brief 已收到MOVE_STARTED后policy拒绝必须STOP，不能按尚未启动例外处理。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_policy_rejection_after_started_motion_still_stops(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_STARTED', dict(boot='7', reference='2', session='9', motion='11', state_seq='101'))
        self.tab.native_event('MOVE_REJECTED_POLICY', dict(boot='7', reference='2', session='9', motion='11', state_seq='102'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, ['STOP'])

    ##
    # @brief 已启动后NOT_READY同样STOP并撤执行。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_not_ready_after_started_motion_still_stops(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_STARTED', dict(boot='7', reference='2', session='9', motion='11', state_seq='101'))
        self.tab.native_event('MOVE_NOT_READY', dict(boot='7', reference='2', session='9', motion='11', state_seq='102'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, ['STOP'])

    ##
    # @brief 拒绝事件session变10时必须STOP，不借prestart拒绝保留错误身份控制。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_rejection_with_changed_identity_still_stops(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_REJECTED_POLICY', dict(boot='7', reference='2', session='10', motion='10', state_seq='100'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases, ['STOP'])

    ##
    # @brief 其它session的完成不能推进段，只保持首命令并STOP。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_wrong_session_completion_aborts_without_next_segment(self):
        self.prepare_execution()
        self.confirm_execution()
        self.tab.native_event('MOVE_COMPLETED', dict(boot='7', reference='2', session='10', motion='11', state_seq='101'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(len(self.panel.commands), 1)
        self.assertEqual(self.panel.releases, ['STOP'])

    ##
    # @brief 用户取消清执行队列，晚到完成加tick仍不能发第二命令，STOP只记一次。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 任一validity false撤path，主动执行入口也不发运动。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_collision_rejection_invalidates_path_and_never_submits(self):
        self.complete_plan()
        self.tab.checks[0].set_result(NS(valid=False))
        self.tab.tick()
        self.tab.start_execution()
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.panel.commands, [])

    ##
    # @brief 201点折线只保真实转角为执行段；首段停在拐点并waiting，未收完成不提前下一段。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 连续模式写临时path并只发MOVE_PATH；STARTED后释放本地path_file引用，未额外发逐点MOVE。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 连续路径检查后速度改30deg/s，执行要求重新规划，不沿旧采样提交。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_continuous_speed_change_requires_new_collision_validation(self):
        self.tab.continuous_enabled = True
        self.complete_plan()
        self.validate_plan()
        self.panel.speed_input.setValue(30.)
        self.tab.start_execution()
        self.assertEqual(self.panel.commands,[])
        self.assertIn('重新规划',self.tab.status.text())

    ##
    # @brief 插值检查false取消其它Future并重采候选，但保持会话active且不STOP/发运动。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 关节预设直接FK→plan，目标J3π/2，不多做IK。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_joint_preset_skips_ik(self):
        self.tab.preset_buttons[0].click()
        self.assertEqual([call[0] for call in self.tab.planner.calls], ['fk', 'plan'])
        phase, args, _ = self.tab.planner.calls[1]
        self.assertEqual(phase, 'plan')
        self.assertAlmostEqual(args[1][2], 1.5707963267948966)

    ##
    # @brief 播放已检路径仅虚影变化，过程不弹执行确认且无命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief play_speed2倍时0.6s可播完测试路径，只影响虚影不发运动。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_configurable_preview_speed(self):
        self.complete_plan()
        self.validate_plan()
        self.tab.play_speed.setValue(2.)
        self.tab.toggle_preview()
        with patch.object(module.time, 'monotonic', return_value=self.tab.preview_started + .6):
            self.tab.tick()
        self.assertTrue(self.tab.previewed)
        self.assertEqual(self.panel.commands, [])

    ##
    # @brief 增量滑块两次advance后回零并停jog timer，但目标输入保留已累积位移，不当速度归零就复位目标。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief IK失败−31触发target_failed及预览红色回调，无设备命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_planning_failure_marks_preview_red(self):
        flags = []
        self.panel.set_preview_failed = flags.append
        self.tab.start_plan()
        self.tab.pending[1].set_result(NS(error_code=NS(val=-31)))
        self.tab.tick()
        self.assertTrue(self.tab.target_failed)
        self.assertTrue(flags[-1])
        self.assertEqual(self.panel.commands, [])

    ##
    # @brief 编辑123mm先100ms debounce，IK完成改关节虚影并转plan，path未完成且不发命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 同身份未Begin时MOVE_REJECTED_PATH只撤请求，保授权且不STOP。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_prebegin_path_rejection_keeps_existing_hold(self):
        self.prepare_execution();self.confirm_execution()
        self.tab.native_event('MOVE_REJECTED_PATH',dict(boot='7',reference='2',session='9',motion='10',state_seq='100'))
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.releases,[])
        self.assertTrue(self.panel.authorized)

    ##
    # @brief scene_change_requested清执行/path但不直接release；场景控制器另负责Hold屏障，不能据此声称停机已确认。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_scene_update_invalidates_plan_and_autotest_without_releasing(self):
        self.prepare_execution();self.confirm_execution()
        self.tab.scene_change_requested()
        self.assertFalse(self.tab.executing)
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.panel.releases,[])

    ##
    # @brief 场景blocked/not ready阻执行，begin的新规划Future也立即取消。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_unconfirmed_scene_blocks_new_execution_and_planning(self):
        self.prepare_execution()
        self.panel.obstacle_tab=NS(scene_ready=False,blocked=True,scene_revision=1)
        self.tab.start_execution(confirmed=True)
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.commands,[])
        f=Future();self.tab.begin('plan',f,self.panel.actual_q)
        self.assertTrue(f.cancelled())

    ##
    # @brief 执行前validate_topology失败回调撤旧path，即便此前scene_ready也不发命令。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_duplicate_provider_revokes_checked_path_before_execution(self):
        self.prepare_execution()
        ##
        # @par 内部回调 topology_failed
        # 模拟拓扑重核失败，先通知scene_change_requested撤路径再返回False。
        # 返回：False，路径已撤。
        def topology_failed():
            self.tab.scene_change_requested()
            return False
        self.panel.obstacle_tab=NS(scene_ready=True,blocked=False,scene_revision=1,
                                  validate_topology=topology_failed)
        self.tab.path_scene_revision=1
        self.tab.start_execution(confirmed=True)
        self.assertFalse(self.tab.executing)
        self.assertIsNone(self.tab.path)
        self.assertEqual(self.panel.commands,[])

    ##
    # @brief 确认框期间将拓扑valid变False，Yes后必须再核并拒绝，堵住确认前旧发现图窗口。
    # @return None；以assert/unittest断言报告结果，失败抛异常。
    def test_provider_changes_while_confirmation_dialog_open_blocks_execution(self):
        self.prepare_execution()
        scene=NS(scene_ready=True,blocked=False,scene_revision=1,validate_topology=lambda:True)
        self.panel.obstacle_tab=scene
        self.tab.path_scene_revision=1
        ##
        # @par 内部回调 confirm
        # 模拟弹框期间服务提供者失效，返回Yes但下一次拓扑校验False。
        # 参数 args：调用位置参数，原样记录或由本替身忽略。
        # 返回：QMessageBox.Yes，后续拓扑验证将失败。
        def confirm(*args):
            scene.validate_topology=lambda:False
            return QtWidgets.QMessageBox.Yes
        self.panel.confirm_execution=confirm
        self.tab.start_execution()
        self.assertFalse(self.tab.executing)
        self.assertEqual(self.panel.commands,[])


    def test_ros_rolling_home_result_waits_for_fresh_actual_without_native(self):
        from manual_moveit_continuous import prepare_path
        self.panel.backend='ros';self.tab.continuous_enabled=True
        sent=[];self.panel.ros_backend=NS(submit_trajectory=lambda trajectory:sent.append(trajectory) or True)
        home=tuple(self.panel.actual_q);start=(home[0]+.1,)+home[1:]
        back=prepare_path((start,home),(0.,1.5),self.panel.speed_input.value(),self.panel.acceleration_input.value())
        self.panel.actual_q=list(start)
        auto=self.tab.auto_test
        auto.active=True;auto.ros_stream=True;auto.chain_enabled=True;auto.completed=2
        auto.identity=self.tab.execution_identity();auto.scene_version=self.tab.scene_revision()
        auto.started_at=module.time.monotonic()
        auto.queued=dict(path=(start,home),times=(0.,back.duration),continuous_path=back,
                         home=True,goal=home,start=start,pose=None,checked_at=module.time.monotonic())
        auto._submit()
        self.assertTrue(self.tab.executing);self.assertTrue(auto.executing_home)
        self.assertEqual(len(sent),1);self.assertEqual(len(sent[0].points),len(back.points))
        self.assertEqual(tuple(sent[0].points[-1].positions),home)
        self.assertEqual(self.panel.commands,[])
        self.tab.ros_event('completed',{})
        self.assertTrue(auto.active)
        self.panel.authorized=False;auto.tick();self.assertTrue(auto.active)
        self.panel.actual_q=list(home);auto.tick()
        self.assertEqual(auto.result,'completed');self.assertEqual(auto.completed,2)
        self.assertEqual(len(sent),1)

    def test_other_ros_action_result_does_not_complete_idle_moveit_tab(self):
        self.panel.backend = 'ros'
        self.tab.ros_event('accepted',{'goal_uuid':'direct-joint'})
        self.tab.ros_event('completed',{'goal_uuid':'direct-joint'})
        self.assertFalse(self.tab.executing)
        self.assertFalse((self.panel.report/'moveit-events.jsonl').exists())


    def test_ros_auto_options_follow_backend_without_changing_native_limits(self):
        self.tab.auto_continuous.setChecked(True)
        self.assertEqual(self.tab.auto_duration.maximum(),57.)
        self.panel.backend = 'ros'
        self.tab.refresh_auto_backend_options()
        self.assertEqual(self.tab.auto_duration.maximum(),9.5)
        self.assertIn('边运动边规划',self.tab.auto_continuous.toolTip())
        self.assertTrue(self.tab.continuous_enabled)
        self.panel.backend = 'real'
        self.tab.refresh_auto_backend_options()
        self.assertEqual(self.tab.auto_duration.maximum(),57.)
        self.assertNotIn('非在线续接',self.tab.auto_continuous.toolTip())


if __name__ == '__main__':
    unittest.main()


def _inflight_cache_wait_fixture(case, clock):
    from unittest.mock import Mock
    from manual_panel_ros_backend import RosTransport
    case.prepare_execution();case.prepare_ros_inflight()
    case.panel.last_state.update(core_state=5,position_valid_mask=63,can_rx_age_us=[0]*6)
    case.panel.last_state_ns=int(clock[0]*1e9)
    transport=object.__new__(RosTransport)
    transport.closed=False;transport.state_at=transport.status_at=clock[0]
    transport.node=NS(count_services=lambda _:1,count_publishers=lambda _:1)
    transport.feedback_max_age_ms=250;transport.status_max_age_ms=1000
    state=NS(**case.panel.last_state,device_id=[1]*16,profile_id=201)
    transport.latest_state=state
    transport.latest_status=NS(**vars(state),real_motion_permitted=True,permit_remaining_us=1000000,evidence_lost=False)
    transport.goal=NS(goal_id=NS(uuid=[2]*16))
    transport._submission=dict(stage='active',binding=((1,)*16,201,7,9,2,10))
    case.panel.ros_backend.transport=transport
    case.tab.ros_event('accepted',{'goal_uuid':bytes([2]*16).hex()})
    case.panel.is_fresh=False
    case.tab.auto_test.active=True
    case.tab.auto_test.scene_version=case.tab.scene_revision()
    case.tab.auto_test.identity=(7,2,9)
    case.tab.auto_test.deadline=clock[0]+180.
    case.tab.auto_test.started_at=clock[0]
    case.tab.auto_test.tick=Mock(wraps=case.tab.auto_test.tick)
    return transport


def test_ros_inflight_cache_wait_keeps_entire_tick_quiet_then_resumes():
    MoveItTabTests.setUpClass()
    case=MoveItTabTests();case.setUp()
    try:
        clock=[10.]
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
            transport=_inflight_cache_wait_fixture(case,clock)
            old_path=case.tab.path;clock[0]=10.3
            case.tab.tick()
            assert case.tab.auto_test.active and case.tab.path is old_path and case.tab.executing
            case.tab.auto_test.tick.assert_not_called()
            assert case.ros_cancels==[] and case.panel.releases==[]
            transport.state_at=transport.status_at=clock[0]
            case.panel.last_state_ns=int(clock[0]*1e9);case.panel.is_fresh=True
            case.tab.tick()
            assert case.tab.auto_test.active and case.tab.executing
            case.tab.auto_test.tick.assert_called_once()
    finally:case.tearDown()


def test_ros_inflight_cache_wait_deadline_is_original_receive_anchor():
    MoveItTabTests.setUpClass()
    case=MoveItTabTests();case.setUp()
    try:
        clock=[10.]
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
            _inflight_cache_wait_fixture(case,clock)
            for now in (10.3,10.9,11.99):
                clock[0]=now;case.tab.tick();assert case.tab.auto_test.active
            clock[0]=12.;case.tab.tick();case.tab.tick()
            assert not case.tab.auto_test.active and not case.tab.executing
            assert len(case.panel.releases)+len(case.ros_cancels)==1
    finally:case.tearDown()


def test_ros_inflight_cache_wait_does_not_hide_real_changes_edit_or_native():
    MoveItTabTests.setUpClass()
    for change in ('boot','reference','session','uuid','permission','core','can','edit','scene','native'):
        case=MoveItTabTests();case.setUp()
        try:
            clock=[10.]
            with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
                transport=_inflight_cache_wait_fixture(case,clock);clock[0]=10.3
                if change in ('boot','reference','session'):
                    case.panel.last_state[{'boot':'boot_epoch','reference':'reference_epoch','session':'session_id'}[change]]+=1
                elif change=='uuid': transport.goal.goal_id.uuid[0]+=1
                elif change=='permission': transport.latest_status.real_motion_permitted=False
                elif change=='core': transport.latest_state.core_state=9
                elif change=='can': case.panel.last_state['can_rx_age_us'][0]=300000
                elif change=='scene':
                    case.tab.scene_revision=lambda:1
                    auto=case.tab.auto_test
                    auto.tick=type(auto).tick.__get__(auto,type(auto))
                elif change=='native': case.panel.backend='real'
                elif change=='edit': case.tab.invalidate()
                case.tab.tick()
                assert not case.tab.auto_test.active and not case.tab.executing, change
                assert len(case.panel.releases)+len(case.ros_cancels)==1, change
        finally:case.tearDown()


def _completed_cache_fixture(case, clock):
    _inflight_cache_wait_fixture(case,clock)
    case.panel.is_fresh=True
    case.tab.ros_execution_goal=tuple(case.panel.actual_q)
    case.tab.auto_test.executing_home=True
    case.tab.ros_event('completed',{})
    return case.tab.auto_test


def test_ros_completed_waits_for_fresh_actual_before_confirming_home():
    MoveItTabTests.setUpClass();case=MoveItTabTests();case.setUp()
    try:
        clock=[10.]
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
            auto=_completed_cache_fixture(case,clock)
            case.panel.is_fresh=False;clock[0]=10.3;auto.tick()
            assert auto.active and auto.ros_completion_pending
            case.panel.is_fresh=True;clock[0]=10.4;auto.tick()
            assert not auto.active and auto.result=='completed'
    finally:case.tearDown()


def test_ros_completed_wait_rejects_real_identity_change_or_closing():
    MoveItTabTests.setUpClass()
    for change in ('boot_epoch','reference_epoch','session_id','closing'):
        case=MoveItTabTests();case.setUp()
        try:
            clock=[10.]
            with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
                auto=_completed_cache_fixture(case,clock);case.panel.is_fresh=False
                if change=='closing':case.panel.closing=True
                else:case.panel.last_state[change]+=1
                auto.tick()
                assert not auto.active and auto.result=='error',change
        finally:case.tearDown()


def test_ros_completed_wait_keeps_two_second_deadline_and_late_feedback_cannot_revive():
    MoveItTabTests.setUpClass();case=MoveItTabTests();case.setUp()
    try:
        clock=[10.]
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
            auto=_completed_cache_fixture(case,clock);case.panel.is_fresh=False
            assert auto.ros_completion_deadline==12.
            for now in (10.3,11.,11.99):
                clock[0]=now;auto.tick();assert auto.active and auto.ros_completion_deadline==12.
            clock[0]=12.;auto.tick()
            assert not auto.active and auto.result=='error'
            case.panel.is_fresh=True;case.tab.ros_event('completed',{});auto.tick()
            assert not auto.active and auto.result=='error'
    finally:case.tearDown()
