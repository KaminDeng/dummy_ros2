"""直接执行生产后端切换与Native tick；不创建Qt/ROS节点或打开设备。"""
from types import SimpleNamespace
import time
from test_manual_panel_fault_readonly import panel


def test_ros_to_native_recreates_measured_joint_state_publisher(tmp_path):
    """ROS切Native后恢复实际关节发布，不能沿用ROS入口的None。"""
    p, scope = panel(tmp_path)
    actual_state = dict(p.last_state)
    p.actual_q = [1.0]*6
    p.backend = 'ros'
    p.pending_backend = 'real'
    p.publisher = None
    p.connection_action = None
    p.worker.deleteLater = lambda: None
    p.backend_choice = SimpleNamespace(findData=lambda value: 0, setCurrentIndex=lambda value: None,
                                      setEnabled=lambda value: None)
    p.feedback_labels = []
    p.create_worker = p.start_worker = lambda: None
    created = []
    def create_publisher(message_type, topic, depth):
        assert message_type is scope['JointState']
        created.append((topic, depth))
        return SimpleNamespace(publish=p.published.append)
    p.node.create_publisher = create_publisher
    p.node.destroy_publisher = lambda publisher: None
    p.apply_backend_change()
    assert p.backend == 'real'
    assert p.publisher is not None
    assert created == [('/joint_states', 10)]
    # 切换先清空旧actual，收到本代新鲜反馈才允许发布。
    assert not hasattr(p, 'actual_q')
    assert p.published == []
    p.last_state = actual_state
    p.last_state_ns = time.monotonic_ns()
    p.calibration = (actual_state['boot_epoch'], actual_state['reference_epoch'])
    p.tick()
    assert p.published[-1].position == actual_state['position_rad']
    assert p.published[-1].name == [f'Joint{i+1}' for i in range(6)]
    assert p.actual_q == actual_state['position_rad']
    count = len(p.published)
    p.last_state_ns = time.monotonic_ns() - 2_000_000_000
    p.tick()
    assert len(p.published) == count
    assert not p.description_visible


def test_native_to_ros_removes_extra_measured_joint_state_publisher(tmp_path):
    """ROS控制链接管后不能保留Native实测publisher造成双来源。"""
    p, _ = panel(tmp_path)
    previous = p.publisher
    p.pending_backend = 'ros'
    p.connection_action = None
    p.worker.deleteLater = lambda: None
    p.backend_choice = SimpleNamespace(findData=lambda value: 0, setCurrentIndex=lambda value: None,
                                      setEnabled=lambda value: None)
    p.feedback_labels = []
    p.create_worker = p.start_worker = lambda: None
    destroyed = []
    p.node.destroy_publisher = destroyed.append
    p.apply_backend_change()
    assert p.publisher is None
    assert destroyed == [previous]
