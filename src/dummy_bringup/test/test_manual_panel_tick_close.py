"""实际Qt面板方法的关闭重入叶模型；不建立ROS/Qt或设备连接。"""
import ast
from pathlib import Path
from types import SimpleNamespace


def methods(scope):
    """从现有生产源码提取方法，不复制其实现。"""
    path = Path(__file__).resolve().parents[1] / 'scripts/manual_joint_panel.py'
    tree = ast.parse(path.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef)
               and any(isinstance(node, ast.FunctionDef) and node.name == 'tick'
                       for node in item.body))
    selected = [node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name in ('tick', 'closeEvent')]
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), scope)
    return scope


def test_closing_timer_never_touches_destroyed_ros_node():
    """已开始关闭时，排队的timer不再查询已销毁的ROS节点。"""
    def invalid_handle():
        raise RuntimeError('ROS node destruction requested')
    scope = methods({'rclpy': SimpleNamespace(ok=invalid_handle)})
    scope['tick'](SimpleNamespace(closing=True))


def test_running_timer_keeps_original_callback_budget():
    """未关闭时保留原ROS批量和ros_tick，不改变运行期门槛。"""
    calls = []
    scope = methods({
        'rclpy': SimpleNamespace(ok=lambda: True,
                                spin_once=lambda *_a, **_k: calls.append('spin')),
        'time': SimpleNamespace(perf_counter=lambda: 0),
    })
    panel = SimpleNamespace(
        closing=False, backend='ros', node=object(),
        ros_backend=SimpleNamespace(transport=SimpleNamespace(
            pump=lambda: calls.append('pump'))),
        ros_tick=lambda: calls.append('ros_tick'),
    )
    scope['tick'](panel)
    assert calls.count('spin') == calls.count('pump') == 32
    assert calls[-1] == 'ros_tick'


def test_close_event_still_explicitly_advances_release():
    """原关闭循环仍显式推进释放；Qt重入timer只退出，不销毁清理流程。"""
    calls = []
    clock = SimpleNamespace(monotonic=lambda: 0)
    scope = methods({
        'rclpy': SimpleNamespace(spin_once=lambda *_a, **_k: calls.append('spin')),
        'time': clock,
    })
    backend = SimpleNamespace(can_close=False, released=False)
    backend.transport = SimpleNamespace(pump=lambda: calls.append('pump'))
    backend.close = lambda: calls.append('backend_close')
    panel = SimpleNamespace(
        closing=False, backend='ros', ros_backend=backend, node=object(), cfg={},
        release=lambda value: calls.append(('release', value)),
        ros_runtime=SimpleNamespace(stop=lambda ready: calls.append(('stop', ready)) or True),
    )
    def explicit_release_tick():
        calls.append('explicit_ros_tick')
        backend.can_close = True
    panel.ros_tick = explicit_release_tick
    scope['QtWidgets'] = SimpleNamespace(QApplication=SimpleNamespace(
        processEvents=lambda: scope['tick'](panel)))
    event = SimpleNamespace(accept=lambda: calls.append('accept'))
    scope['closeEvent'](panel, event)
    assert calls == [('release', 'RELEASE'), 'spin', 'pump',
                     'explicit_ros_tick', ('stop', True), 'backend_close', 'accept']
