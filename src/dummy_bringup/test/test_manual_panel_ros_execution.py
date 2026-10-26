"""ROS整轨迹提交保留已检查折线，不走Native逐段命令。"""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))


def test_prepared_trajectory_preserves_all_points_and_time():
    from manual_moveit_execution import ros_trajectory
    points = ((0.,)*6, (.01,)*6, (.005,)*6, (0.,)*6)
    path = NS(points=points, period=.02)
    message = ros_trajectory(path, [f'Joint{i}' for i in range(6)])
    assert tuple(tuple(p.positions) for p in message.points) == points
    assert [p.time_from_start.nanosec for p in message.points] == [20000000,40000000,60000000,80000000]
    assert all(not p.velocities and not p.accelerations for p in message.points)


def test_ros_autotest_requests_cpp_without_python_motion():
    from manual_moveit_autotest import AutoTestSession
    requests = []
    panel = NS(backend='ros', ros_backend=NS(request_calibration_and_test=lambda: requests.append('cpp') or True))
    session = AutoTestSession(NS(panel=panel))
    session._status = lambda *args: None
    assert session.start(60.)
    assert requests == ['cpp']
    assert not session.active


def test_ros_trajectory_rejects_invalid_time():
    from manual_moveit_execution import ros_trajectory
    with pytest.raises(ValueError):
        ros_trajectory(NS(points=((0.,)*6, (1.,)*6), period=0.), list('abcdef'))


def test_duplicate_ros_state_does_not_refresh_arrival():
    from manual_joint_panel import JointPanel
    panel = NS(ros_backend=NS(latest_state=None), last_state={'boot_epoch': 7, 'state_seq': 10}, last_state_ns=123)
    JointPanel.ros_state_received(panel, NS(boot_epoch=7, state_seq=10))
    assert panel.last_state_ns == 123


def test_ros_ui_submits_whole_checked_path_without_native(tmp_path):
    from manual_moveit_tab import MoveItTab
    from manual_moveit_continuous import PreparedPath
    from PyQt5 import QtWidgets
    path = ((0.,)*6, (.001,)*6, (0.,)*6)
    prepared = PreparedPath(path, 1., .1)
    submitted = []
    events = []
    panel = NS(backend='ros', busy=False, actual_q=path[0], limits=[(-180,180)]*6,
               speed_input=NS(value=lambda:1.), acceleration_input=NS(value=lambda:.1),
               confirm_execution=lambda *args:QtWidgets.QMessageBox.Yes,
               ros_backend=NS(submit_trajectory=lambda message:submitted.append(message) or True),
               command=lambda *args:pytest.fail('不得调用Native'))
    tab = NS(panel=panel, path=path, checks=[], samples=[], pending=None, executing=False,
             scene_ready=lambda:True, scene_revision=lambda:3, path_scene_revision=3,
             request_ready=lambda:True, execution_identity=lambda:(1,2,3), revision=4,
             continuous_enabled=True, continuous_path=prepared, times=(0.,.02,.04),
             segment_timeout_s=30., auto_test=NS(active=False),
             planner=NS(names=list('abcdef')), status=NS(setText=lambda value:None),
             record=lambda event,**fields:events.append(event),
             cancel_execution=lambda *args,**kwargs:pytest.fail(str(args)),
             advance_execution=lambda:pytest.fail('不得逐段发末点'))
    MoveItTab.start_execution(tab)
    assert len(submitted) == 1
    assert tuple(tuple(p.positions) for p in submitted[0].points) == path
    assert tab.execution.waiting
    assert 'ROS_TRAJECTORY_SUBMITTED' in events


def test_ros_scene_requires_action_cancel_terminal():
    from manual_scene_obstacles import ObstacleScene
    scene = NS(wait_hold=True, stop_identity=(1,2,3), stop_state_seq=8, hold_marker=None,
               panel=NS(last_state=dict(boot_epoch=1, reference_epoch=2, session_id=3, state_seq=9, motion_epoch=4)))
    ObstacleScene.ros_event(scene, 'accepted', {'status':5})
    assert scene.hold_marker is None
    ObstacleScene.ros_event(scene, 'canceled', {'status':5})
    assert scene.hold_marker == ((1,2,3),9,4)


def test_real_24_point_positions_only_path_starts_at_real_command_slot():
    from manual_moveit_execution import ros_trajectory
    # panel-interface-01已保存的真实24点：保留全部六轴位置，命令周期20ms。
    positions = (
        (-3.681404088114222e-05, -1.0471299231335476, 2.895613308401128, 7.117693326211072e-05, 0.0001227184630308513, 6.381360072118324e-05),
        (-3.681404088114222e-05, -1.0471299231335476, 2.895613308401128, 7.117693326211072e-05, 0.0001227184630308513, 6.381360072118324e-05),
        (-2.1935834302477296e-05, -1.0471295595038579, 2.8956131943970904, 7.109736128078043e-05, 0.00012254684158644367, 6.415792737528089e-05),
        (2.3627184153735917e-05, -1.0471284459243293, 2.8956128452711365, 7.085368005248312e-05, 0.00012202126809089619, 6.521239328907687e-05),
        (0.00010126761256782518, -1.0471265483592735, 2.895612250352505, 7.043844165075926e-05, 0.0001211256788007219, 6.700922738982574e-05),
        (0.00021237804902011795, -1.0471238327730015, 2.8956113989704324, 6.984419814914925e-05, 0.00011984400997243393, 6.958065860478206e-05),
        (0.00035835109159094243, -1.0471202651298246, 2.8956102804541564, 6.906350162119346e-05, 0.00011816019786254532, 7.295891586120038e-05),
        (0.0005405793383606256, -1.0471158113940542, 2.8956088841329164, 6.808890414043235e-05, 0.00011605817872756914, 7.717622808633527e-05),
        (0.0007604553874094952, -1.0471104375300013, 2.895607199335949, 6.691295778040631e-05, 0.00011352188882401848, 8.226482420744124e-05),
        (0.0010185265369199753, -1.047104130161466, 2.8956052218695905, 6.55327354678472e-05, 0.00011054501501837873, 8.823737035809775e-05),
        (0.0013058669463535243, -1.0470971074397832, 2.895603020127962, 6.399597459724109e-05, 0.00010723051768007864, 9.488729561919603e-05),
        (0.0016095555860625899, -1.047089685160407, 2.8956006931182605, 6.237177972930546e-05, 0.0001037274420424682, 0.00010191556832435108),
        (0.0019166712285407362, -1.047082179123627, 2.895598339849202, 6.072925648295148e-05, 0.00010018483562121734, 0.00010902315222812551),
        (0.0022142926462815297, -1.0470749051297334, 2.8955960593294994, 5.9137510477090306e-05, 9.675174593199598e-05, 0.00011591101108508204),
        (0.002489498611778533, -1.047068178979016, 2.895593950567867, 5.7665647330633064e-05, 9.357722049047405e-05, 0.0001222801086497832),
        (0.0027293678975253105, -1.0470623164717638, 2.89559211257302, 5.638277266249094e-05, 9.081030681232151e-05, 0.00012783140867679172),
        (0.002931501494425341, -1.0470573762407795, 2.8955905637273296, 5.530171691066264e-05, 8.847867768216565e-05, 0.00013250939084711024),
        (0.003102623379205835, -1.0470531939492145, 2.8955892525084233, 5.438651874151738e-05, 8.650477142669687e-05, 0.0001364696682984157),
        (0.0032423021664489396, -1.0470497801403122, 2.895588182221786, 5.363948530089525e-05, 8.489356411531688e-05, 0.00013970225746872626),
        (0.0033501018188875544, -1.0470471454710077, 2.895587356208547, 5.3062948613767996e-05, 8.365008547692475e-05, 0.00014219706713821322),
        (0.0034255862992545825, -1.047045300598238, 2.895586777809835, 5.265924070510745e-05, 8.277936524041956e-05, 0.00014394400608704793),
        (0.0034683195702829247, -1.047044256178939, 2.8955864503667788, 5.2430693599885376e-05, 8.228643313470035e-05, 0.00014493298309540168),
        (0.003478635691971859, -1.0470440040485423, 2.8955863713196495, 5.237552067040004e-05, 8.216743574777883e-05, 0.0001451717293210058),
        (0.003478635691971859, -1.0470440040485423, 2.8955863713196495, 5.237552067040004e-05, 8.216743574777883e-05, 0.0001451717293210058),
    )
    trajectory = ros_trajectory(NS(points=positions, period=.02), [f'Joint{i}' for i in range(1, 7)])
    assert len(trajectory.points) == 24
    assert tuple(tuple(point.positions) for point in trajectory.points) == positions
    assert all(not point.velocities and not point.accelerations for point in trajectory.points)
    assert [point.time_from_start.sec + point.time_from_start.nanosec / 1e9 for point in trajectory.points] == pytest.approx([.02 * (i + 1) for i in range(24)])
