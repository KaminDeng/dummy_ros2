"""重复MoveIt不得将场景、规划和碰撞检查路由到不同world。"""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from manual_moveit_planner import MoveItPlanner
from manual_scene_obstacles import ObstacleScene

SERVICES = ('/apply_planning_scene', '/get_planning_scene', '/plan_kinematic_path',
            '/check_state_validity', '/compute_ik', '/compute_fk')
NAMES = [f'Joint{i}' for i in range(1, 7)]


class Graph:
    """仅模拟ROS发现图和请求入队，不创建ROS节点或发送服务。"""
    def __init__(self):
        self.counts = dict.fromkeys(SERVICES, 1)
        self.owners = {('manual_move_group', '/'): set(SERVICES)}
        self.calls = []

    def resolve_service_name(self, name):
        return name

    def count_services(self, name):
        return self.counts[name]

    def get_node_names_and_namespaces(self):
        return list(self.owners)

    def get_service_names_and_types_by_node(self, name, namespace):
        return [(service, ['fixture/type']) for service in self.owners[(name, namespace)]]

    def create_client(self, service_type, name, **kwargs):
        def call(request):
            self.calls.append((name, request))
            return Future()
        return NS(service_name=name, service_is_ready=lambda: True, call_async=call)

    def create_publisher(self, *args):
        return NS(publish=lambda message: None)


@pytest.mark.parametrize('service', SERVICES)
@pytest.mark.parametrize('count', [0, 2])
@pytest.mark.parametrize('operation', ['validity', 'fk', 'ik', 'plan'])
def test_no_request_when_any_moveit_service_is_missing_or_duplicated(service, count, operation):
    graph = Graph()
    graph.counts[service] = count
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    with pytest.raises(ValueError, match='MoveIt'):
        if operation == 'validity':
            planner.request_validity([0.] * 6)
        elif operation == 'fk':
            planner.request_fk([0.] * 6)
        elif operation == 'ik':
            planner._request_ik_single([0.] * 6, [0., 0., 100.], [0.] * 3)
        else:
            planner._request_plan_single([0.] * 6, [.1] * 6)
    assert graph.calls == []


def test_single_counts_with_split_scene_and_planning_owners_are_rejected():
    graph = Graph()
    graph.owners = {('scene_a', '/'): set(SERVICES[:2]),
                    ('planner_b', '/'): set(SERVICES[2:])}
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    with pytest.raises(ValueError, match='MoveIt'):
        planner.request_validity([0.] * 6)
    assert graph.calls == []


def test_single_common_owner_preserves_normal_request():
    graph = Graph()
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    planner.request_validity([0.] * 6)
    assert [name for name, _ in graph.calls] == ['/check_state_validity']


def make_scene(tmp_path):
    graph = Graph()
    changes = []
    panel = NS(node=graph, last_state=None, busy=False,
               moveit_tab=NS(executing=False, auto_test=NS(active=False),
                             scene_change_requested=lambda: changes.append('invalidated')),
               cancel_motion_hold=lambda: changes.append('stop'))
    scene = ObstacleScene(panel, tmp_path/'obstacles.json', {'base_link': (0., 0.)})
    return graph, scene, changes


def test_ready_scene_is_revoked_when_second_server_appears(tmp_path):
    graph, scene, changes = make_scene(tmp_path)
    scene.scene_ready = True
    scene.blocked = False
    graph.counts['/plan_kinematic_path'] = 2
    scene.tick()
    assert scene.blocked and not scene.scene_ready
    assert changes == ['invalidated']
    assert graph.calls == []


def test_late_apply_success_cannot_restore_ready_after_topology_changed(tmp_path):
    graph, scene, changes = make_scene(tmp_path)
    pending = Future()
    scene.phase = 'apply'
    scene.future = pending
    graph.counts['/get_planning_scene'] = 2
    pending.set_result(NS(success=True))
    scene.tick()
    assert scene.blocked and not scene.scene_ready
    assert not scene.path.exists()
    assert changes == ['invalidated']


def test_initial_discovery_waits_but_ready_scene_cannot_ignore_missing_service(tmp_path):
    graph, scene, changes = make_scene(tmp_path)
    graph.counts['/compute_fk'] = 0
    scene.tick()
    assert not scene.error and scene.blocked and not graph.calls
    graph.counts['/compute_fk'] = 1
    scene.tick()
    assert len(graph.calls) == 1
    scene.scene_ready = True
    scene.blocked = False
    graph.counts['/compute_fk'] = 0
    scene.tick()
    assert scene.error and scene.blocked and not scene.scene_ready


def test_graph_query_failure_is_not_treated_as_one_provider():
    graph = Graph()
    def unavailable(name):
        raise RuntimeError('discovery unavailable')
    graph.count_services = unavailable
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    with pytest.raises(ValueError, match='MoveIt'):
        planner.request_validity([0.] * 6)
    assert graph.calls == []


def test_same_node_name_duplicate_is_not_deduplicated_before_rejection():
    graph = Graph()
    graph.counts['/plan_kinematic_path'] = 2
    graph.get_node_names_and_namespaces = lambda: [('manual_move_group', '/')] * 2
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    with pytest.raises(ValueError, match='MoveIt'):
        planner.request_validity([0.] * 6)
    assert not graph.calls


def test_moveit_private_scene_monitor_and_same_process_name_aliases_are_supported():
    graph = Graph()
    graph.owners = {('manual_move_group', '/'): set(SERVICES)-{'/get_planning_scene'},
                    ('manual_move_group_private_12345', '/'): {'/get_planning_scene'}}
    names = list(graph.owners)
    graph.get_node_names_and_namespaces = lambda: names + [('manual_move_group', '/')] * 2
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    planner.request_validity([0.] * 6)
    assert len(graph.calls) == 1


def test_topology_loss_during_motion_requests_hold_and_keeps_old_path_invalid(tmp_path):
    graph, scene, changes = make_scene(tmp_path)
    scene.scene_ready = True
    scene.blocked = False
    scene.panel.busy = True
    graph.counts['/compute_fk'] = 2
    scene.tick()
    assert changes == ['invalidated', 'stop']
    graph.counts['/compute_fk'] = 1
    scene.tick()
    assert scene.blocked and not scene.scene_ready and not graph.calls


def test_changed_owner_needs_explicit_new_scene_transaction(tmp_path):
    graph, scene, changes = make_scene(tmp_path)
    assert scene.validate_topology()
    scene.scene_ready = True
    scene.blocked = False
    graph.owners = {('replacement_move_group', '/'): set(SERVICES)}
    scene.tick()
    assert scene.error and scene.blocked and not scene.scene_ready
    scene.request_change([])
    scene.tick()
    assert not scene.error and not scene.scene_ready and len(graph.calls) == 1


def test_initial_node_disappearance_waits_without_opening_scene(tmp_path):
    from rclpy.node import NodeNameNonExistentError
    graph, scene, changes = make_scene(tmp_path)
    def disappeared(name, namespace):
        raise NodeNameNonExistentError('node disappeared')
    graph.get_service_names_and_types_by_node = disappeared
    scene.tick()
    assert scene.blocked and not scene.scene_ready and not scene.error
    assert not graph.calls


def test_measured_motion_stops_even_if_panel_busy_flag_is_clear(tmp_path):
    graph, scene, changes = make_scene(tmp_path)
    scene.scene_ready = True
    scene.blocked = False
    scene.panel.last_state = {'core_state': 3, 'estimated_velocity_rad_s': [0.] * 6}
    graph.counts['/plan_kinematic_path'] = 2
    scene.tick()
    assert changes == ['invalidated', 'stop']
