"""关节直控保持 ROS 权威执行，不依赖 MoveIt 场景或碰撞服务。"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from PyQt5 import QtWidgets

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import manual_moveit_execution as execution
from manual_joint_panel import JointPanel


def joint_panel():
    submitted, messages = [], []
    def forbidden(*args):
        raise AssertionError('关节直控不能调用 MoveIt 或 Native 命令')
    panel = NS(backend='ros', busy=False, closing=False, authorized=True,
        calibration=(1, 2), actual_q=[0.] * 6, limits=[[-90., 90.]] * 6,
        cfg={'joints': [{'name': f'Joint{i+1}', 'nominal_limits_deg': [-100., 100.]}
                        for i in range(6)]},
        inputs=[NS(value=lambda: 30.) for _ in range(6)],
        speed_input=NS(value=lambda: 60.), acceleration_input=NS(value=lambda: 4.),
        obstacle_tab=NS(blocked=True, scene_ready=False),
        moveit_tab=NS(executing=False, auto_test=NS(active=False),
                      start_joint_plan=forbidden, invalidate=lambda: None),
        fresh=lambda: True, control_state_allowed=lambda *args: True,
        confirm_execution=lambda *args: QtWidgets.QMessageBox.Yes,
        message=messages.append, command=forbidden, set_preview_pose=lambda goal: None,
        ros_backend=NS(submit_trajectory=lambda trajectory: submitted.append(trajectory) or True))
    return panel, submitted, messages


def test_ros_joint_control_submits_complete_action_without_moveit_collision_checks():
    panel, submitted, _ = joint_panel()
    JointPanel.start_move(panel)
    assert len(submitted) == 1
    trajectory = submitted[0]
    assert trajectory.joint_names == [f'Joint{i+1}' for i in range(6)]
    assert list(trajectory.points[0].positions) == panel.actual_q
    assert trajectory.points[-1].positions == pytest.approx([math.pi/6] * 6)
    assert len(trajectory.points) > 2 and panel.busy


@pytest.mark.parametrize('reason', ['confirmation', 'stale', 'unauthorized', 'busy', 'auto'])
def test_ros_joint_control_preserves_explicit_confirmation_and_motion_interlocks(reason):
    panel, submitted, _ = joint_panel()
    if reason == 'confirmation': panel.confirm_execution = lambda *args: QtWidgets.QMessageBox.No
    elif reason == 'stale': panel.fresh = lambda: False
    elif reason == 'unauthorized': panel.authorized = False
    elif reason == 'busy': panel.busy = True
    else: panel.moveit_tab.auto_test.active = True
    JointPanel.start_move(panel)
    assert not submitted


def test_ros_joint_control_rechecks_feedback_after_operator_confirmation():
    panel, submitted, _ = joint_panel()
    def confirm(*args):
        panel.fresh = lambda: False
        return QtWidgets.QMessageBox.Yes
    panel.confirm_execution = confirm
    JointPanel.start_move(panel)
    assert not submitted


def test_direct_joint_path_preserves_limits_and_requested_dynamic_caps():
    path = execution.joint_goal_path([0.] * 6, [.8] * 6, [[-90., 90.]] * 6, 60., 4.)
    assert path.points[0] == (0.,) * 6 and path.points[-1] == (.8,) * 6
    previous = (0.,) * 6
    for start, end in zip(path.points, path.points[1:]):
        velocity = tuple((b-a)/path.period for a,b in zip(start, end))
        assert max(map(abs, velocity)) <= math.radians(60.)
        assert max(abs(v-p)/path.period for v,p in zip(velocity, previous)) <= 4.
        assert all(0. <= q <= .8 for q in end)
        previous = velocity
    assert previous == (0.,) * 6 and path.duration < 4.


@pytest.mark.parametrize('bad', [float('nan'), 2.])
def test_direct_joint_path_rejects_nonfinite_and_out_of_range_goals(bad):
    with pytest.raises(ValueError):
        execution.joint_goal_path([0.] * 6, [bad] * 6, [[-90., 90.]] * 6, 60., 4.)
