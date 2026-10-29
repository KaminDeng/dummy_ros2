"""真实AutoTestSession/采样/拼接流水；仅MoveIt服务与ROS传输为内存替身。"""
import hashlib
import json
import math
import random
import sys
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from manual_moveit_autotest import AutoTestSession
from manual_moveit_execution import collision_samples, ros_trajectory


def stamp(seconds):
    ns = round(seconds * 1e9)
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def positions(trajectory):
    return tuple(tuple(point.positions) for point in trajectory.points)


class Planner:
    names = tuple(f'Joint{i}' for i in range(1, 7))

    def __init__(self):
        self.validated = []
        self.plans = 0
        self.plan_validation_floor = 0

    @staticmethod
    def ready(value):
        future = Future()
        future.set_result(value)
        return future

    def request_validity(self, q):
        self.validated.append(tuple(q))
        return self.ready(NS(valid=True))

    def request_plan(self, start, goal):
        self.plans += 1
        self.plan_validation_floor = len(self.validated)
        # 一个具有真实转角的六轴绕行，不能只执行首末点；中点速度连续。
        middle = tuple((a + b) / 2 + (.025 if i == 1 else 0.)
                       for i, (a, b) in enumerate(zip(start, goal)))
        velocity = tuple((b - a) / 3 for a, b in zip(start, goal))
        points = [NS(positions=q, velocities=v, time_from_start=stamp(at))
                  for q, v, at in ((start, (0.,) * 6, 0.),
                                   (middle, velocity, 1.5),
                                   (goal, (0.,) * 6, 3.))]
        trajectory = NS(joint_names=self.names, points=points)
        return self.ready(NS(motion_plan_response=NS(error_code=NS(val=1),
                                                    trajectory=NS(joint_trajectory=trajectory))))

    def request_fk(self, *args):
        raise AssertionError('ROS随机关节候选不应进入Native FK/IK分支')


class Transport:
    def __init__(self, harness):
        self.h = harness
        self.mothers = []
        self.extensions = []
        self.current = None
        self.version = 0

    def submit_trajectory(self, trajectory, task_budget_s=None):
        self.mothers.append(trajectory)
        self.current = trajectory
        return True

    def extend_trajectory(self, trajectory, version, through):
        assert len(self.mothers) == 1
        assert version == self.version
        cut = (through - self.h.session.ros_first_slot_ns) // 20_000_000
        old, new = positions(self.current), positions(trajectory)
        assert new[:cut + 1] == old[:cut + 1]
        pending = self.h.session.chain_pending
        samples = collision_samples(pending['path'].points[pending['item']['join_cut']:])
        checked = self.h.planner.validated[self.h.planner.plan_validation_floor:]
        # 校验真实修改区逐样本均完成validity，而不是仅检查随机末点。
        assert all(sample in checked for sample in samples)
        assert len(samples) > 2
        self.extensions.append(dict(trajectory=trajectory, version=version, through=through,
                                    samples=len(samples), old=self.current))
        return True

    def acknowledge(self):
        item = self.extensions[-1]
        self.version += 1
        self.current = item['trajectory']
        digest = hashlib.sha256(json.dumps(positions(self.current)).encode()).hexdigest()
        self.h.session.ros_event('trajectory_extended',
                                dict(version=self.version, phase_ns=self.h.phase_ns,
                                     trajectory_digest=digest))


class Harness:
    def __init__(self, monkeypatch):
        self.now = 1.
        self.phase = 0.
        self.records = []
        self.preview = []
        self.planner = Planner()
        monkeypatch.setattr('manual_moveit_autotest.time.monotonic', lambda: self.now)
        monkeypatch.setattr('manual_moveit_autotest.random', random.Random(1729))
        panel = NS(backend='ros', actual_q=[0., 0., math.pi / 2, 0., 0., 0.], busy=False,
                   cfg=dict(home_deg=[0, 0, 90, 0, 0, 0], rest_deg=[0, -60, 164, 0, 0, 0], backend='ros'),
                   limits=[(-169., 169.), (-72.45, 89.45), (35.55, 179.45),
                           (-179., 179.), (-99., 99.), (-359., 359.)],
                   fresh=lambda: True, authorized=True, closing=False,
                   control_state_allowed=lambda states: True,
                   speed_input=NS(value=lambda: 60.), acceleration_input=NS(value=lambda: 4.),
                   set_preview_pose=lambda q, **kw: self.preview.append((q, kw)),
                   command=lambda *args: pytest.fail('ROS不得发送Native命令'),
                   release=lambda *args: pytest.fail('正常滚动测试不得停止设备'))
        tab = NS(panel=panel, planner=self.planner, pending=None, executing=False,
                 started_motion=None, execution=None, continuous_enabled=True,
                 request_ready=lambda: True, execution_identity=lambda: (7, 1, 9),
                 current_identity=lambda: (7, 1, 9), scene_ready=lambda: True,
                 scene_revision=lambda: 1, invalidate=lambda: None,
                 preview_timer=NS(stop=lambda: None), collision_timer=NS(start=lambda: None),
                 amplitude=NS(currentData=lambda: 'medium', currentText=lambda: '中'),
                 ik_preference=NS(currentData=lambda: 'nearest', currentText=lambda: '最近'),
                 auto_continuous=NS(isChecked=lambda: True),
                 status=NS(setText=lambda text: None),
                 auto_status=NS(setText=lambda text: None, setStyleSheet=lambda text: None),
                 record=lambda event, **values: self.records.append((event, values)),
                 fill_pose=lambda pose: None,
                 cancel_execution=lambda *args, **kw: pytest.fail('正常滚动不应取消'))
        self.tab = tab
        self.session = AutoTestSession(tab)
        panel.ros_backend = self.transport = Transport(self)
        tab.start_execution = self.start_execution
        from PyQt5.QtWidgets import QMessageBox
        panel.confirm_execution = lambda *args: QMessageBox.Yes
        assert self.session.start(180.)

    def start_execution(self, confirmed):
        assert confirmed
        trajectory = ros_trajectory(self.tab.continuous_path, self.planner.names,
                                    start_tolerance_rad=.003)
        first = trajectory.points[0].time_from_start
        self.session.ros_first_slot_ns = first.sec * 1_000_000_000 + first.nanosec
        assert self.transport.submit_trajectory(trajectory)
        self.tab.executing = True
        self.tab.execution = NS(points=(self.tab.path[0], self.tab.path[-1]))

    @property
    def phase_ns(self):
        return self.session.ros_first_slot_ns + round(self.phase * 1e9)

    def feedback(self, phase):
        self.phase = phase
        value = NS(desired=NS(time_from_start=stamp(self.phase_ns / 1e9)))
        assert self.session.ros_event('progress', value)

    def step(self):
        self.now += .001
        if self.session.chain_live:
            self.feedback(self.phase + .01)
        self.session.tick()
        assert self.session.active
        if self.session.chain_pending is not None:
            self.transport.acknowledge()

    def run_extensions(self, count):
        for _ in range(2500):
            self.step()
            if self.transport.version >= count:
                return
        pytest.fail('真实规划/拼接流水未能在有限tick内续接')

    def finish_candidate(self):
        self.session._drop_planning()
        self.session._begin(self.session.chain_path.points[-1])
        for _ in range(500):
            self.now += .001
            self.session._poll_plan()
            if self.session.queued is not None:
                return
        pytest.fail('修改区未完成碰撞检查')


def test_real_ros_rolling_pipeline_starts_short_then_extends_three_times(monkeypatch):
    h = Harness(monkeypatch)
    h.run_extensions(3)
    assert len(h.transport.mothers) == 1
    assert h.session.chain_live and h.session.chain_version == 3
    assert [item['version'] for item in h.transport.extensions] == [0, 1, 2]
    assert h.transport.mothers[0].points[-1].time_from_start.sec < 20
    assert h.planner.plans >= 6
    assert all(item['samples'] > 2 for item in h.transport.extensions)
    path = h.session.chain_path
    dt = path.period
    velocities = [tuple((b - a) / dt for a, b in zip(p, q))
                  for p, q in zip(path.points, path.points[1:])]
    # 只有整条首末静止，不在每个内部目标处人为塞入全轴停车采样。
    assert all(max(map(abs, v)) > 1e-10 for v in velocities[2:-2])
    assert max(max(map(abs, v)) for v in velocities) <= math.radians(60.)
    assert max(max(abs(b - a) / dt for a, b in zip(p, q))
               for p, q in zip([(0.,) * 6] + velocities[:-1], velocities)) <= 4.
    h.session.update_preview()
    first = h.preview[-1]
    h.feedback(h.phase + .2)
    h.session.update_preview()
    assert h.preview[-1][0] != first[0]
    assert h.preview[-1][1] == dict(announce=False)


def test_real_ros_rolling_late_checked_candidate_keeps_original_safe_tail(monkeypatch):
    h = Harness(monkeypatch)
    h.run_extensions(3)
    h.finish_candidate()
    old = h.session.chain_path
    cut = h.session.queued['join_cut']
    previous_updates = len(h.transport.extensions)
    h.feedback(cut * .02 - .1)
    h.session._chain_tick()
    assert h.session.chain_path is old
    assert h.session.chain_draining and h.session.chain_pending is None
    assert len(h.transport.extensions) == previous_updates
    assert len(h.transport.mothers) == 1
    assert any(event == 'AUTO_TEST_CHAIN_LATE' for event, _ in h.records)


def test_real_ros_rolling_controller_rejection_keeps_confirmed_tail(monkeypatch):
    h = Harness(monkeypatch)
    h.run_extensions(3)
    h.finish_candidate()
    old = h.session.chain_path
    h.session._chain_tick()
    assert h.session.chain_pending is not None
    h.session.ros_event('trajectory_extension_rejected',
                        dict(phase_ns=h.phase_ns, reason='TOO_LATE_AT_CONTROL_TICK'))
    assert h.session.chain_path is old
    assert h.session.chain_pending is None and h.session.chain_draining
    assert h.session.stage is None and not h.session.samples and not h.session.checks
    assert len(h.transport.mothers) == 1 and h.transport.version == 3
