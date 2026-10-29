"""高频反馈不占用操作员日志；故障图仍允许确认释放后重连。"""
import sys
import pytest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import manual_joint_panel as module
from manual_joint_panel import JointPanel


def event_panel():
    messages, forwarded = [], []
    panel = NS(message=messages.append, busy=True,
               moveit_tab=NS(ros_event=lambda *v: forwarded.append(v)),
               ros_runtime=NS(fail=lambda value: None, active=lambda: None))
    return panel, messages, forwarded


def test_feedback_is_summarized_without_serializing_ros_payload(monkeypatch):
    panel, messages, forwarded = event_panel()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 3.)
    class Feedback:
        desired = NS(time_from_start=NS(sec=12, nanosec=500000000))
        def __str__(self):
            raise AssertionError('不能序列化完整高频反馈到操作员日志')
    feedback = Feedback()
    for _ in range(1000):
        JointPanel.ros_event(panel, 'progress', feedback)
    assert len(messages) == 1 and '12.50' in messages[0]
    assert len(forwarded) == 1000


def test_progress_dedup_never_drops_new_states_or_terminal_errors(monkeypatch):
    panel, messages, _ = event_panel()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 3.)
    for _ in range(100):
        JointPanel.ros_event(panel, 'progress', 'READY')
    JointPanel.ros_event(panel, 'progress', 'RELEASE_PENDING')
    JointPanel.ros_event(panel, 'failed', 'TIMEOUT')
    JointPanel.ros_event(panel, 'failed', 'TIMEOUT')
    assert len(messages) == 4 and messages[-1].endswith('TIMEOUT')


def test_graph_error_refreshes_connection_controls_before_return():
    seen = []
    panel = NS(ros_runtime=NS(check_startup_timeout=lambda *v: None),
               ros_started_at=0, cfg={}, closing=False,
               ros_backend=NS(transport=NS(control_graph_error=lambda: 'ACTION_UNAVAILABLE')),
               refresh_ros_connection_controls=lambda: seen.append('controls'),
               show_actual_description=lambda value: None, authorized=True,
               status=NS(setText=lambda value: seen.append('error')))
    JointPanel.ros_tick(panel)
    assert seen == ['controls', 'error'] and not panel.authorized


def test_reconnect_waits_for_release_then_reopens_once(monkeypatch):
    calls, queued = [], []
    panel, _, _ = event_panel()
    panel.backend = 'ros'
    panel.ros_runtime = NS(state='Active', process=object(), stop=lambda value: calls.append('stop') or True,
                           retire_disconnected_owner=lambda: False)
    panel.ros_backend = NS(released=False, transport=NS(), close=lambda: calls.append('close') or True)
    panel.release = lambda value: calls.append('release')
    panel.apply_backend_change = lambda: calls.append('reopen')
    panel.request_reconnect = lambda: JointPanel.request_reconnect(panel)
    monkeypatch.setattr(module.QtCore.QTimer, 'singleShot', lambda delay, fn: queued.append(fn))
    JointPanel.request_reconnect(panel)
    assert calls == ['release'] and panel.ros_reconnect_pending
    panel.ros_backend.released = True
    JointPanel.ros_event(panel, 'released', 'EVIDENCE_INCOMPLETE;RELEASED_IDLE')
    assert len(queued) == 1
    queued.pop()()
    assert calls == ['release', 'stop', 'close', 'reopen']
    assert not panel.ros_reconnect_pending


@pytest.mark.parametrize('callback_cost,expected', [(0., 32), (.0011, 3)])
def test_ros_callback_pump_drains_burst_with_a_gui_time_budget(monkeypatch, callback_cost, expected):
    clock, seen = [0.], []
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.time, 'perf_counter', lambda: clock[0])
    def spin(*args, **kwargs):
        seen.append('callback')
        clock[0] += callback_cost
    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    panel = NS(node=object(), backend='ros', ros_tick=lambda: seen.append('ui'),
               ros_backend=NS(transport=NS(pump=lambda:None)))
    JointPanel.tick(panel)
    assert seen == ['callback'] * expected + ['ui']


def test_native_callback_pump_retains_existing_batch(monkeypatch):
    seen = []
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *a, **k: seen.append(1))
    class AfterCallbacks(Exception):
        pass
    def read():
        raise AfterCallbacks()
    with pytest.raises(AfterCallbacks):
        JointPanel.tick(NS(node=object(), backend='real', read_state=read))
    assert len(seen) == 8
