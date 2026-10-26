"""重复MoveIt不得将场景、规划和碰撞检查路由到不同world。"""
##
# @file test_manual_moveit_service_topology.py
# @brief MoveIt单服务实例/同world归属及场景失效闭锁的发现图替身测试。
#
# Graph只模拟count/owner/service列表和call_async入队，Future由测试手动完成，不创建ROS节点或发送服务。核公共五服务同owner、Get可属于Jazzy private monitor、先服务计数再去重别名；名称关系通过不构成真实进程身份密码证明。
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


##
# @brief ROS发现/客户端接口替身，所有计数、owner与请求都来自内存，不解析真实DDS图。
class Graph:
    """仅模拟ROS发现图和请求入队，不创建ROS节点或发送服务。"""
    ##
    # @brief 初始六服务各count1并归manual_move_group，calls列表记录模拟入队，不读取ROS发现。
    # @return 无；仅初始化本测试替身字段。
    def __init__(self):
        self.counts = dict.fromkeys(SERVICES, 1)
        self.owners = {('manual_move_group', '/'): set(SERVICES)}
        self.calls = []

    ##
    # @brief 原样返回绝对服务名，省略真实ROS重映射/命名空间解析。
    # @param name 传入服务名，原样返回。
    # @return 传入name原值。
    def resolve_service_name(self, name):
        return name

    ##
    # @brief 从counts字典返回人工服务实例数，可注入0/2，不向DDS查询。
    # @param name SERVICES中的绝对名，查人工counts。
    # @return 人工服务实例数。
    def count_services(self, name):
        return self.counts[name]

    ##
    # @brief 返回owners键列表，允许用例替换以模拟同进程名称别名。
    # @return owners的(name, namespace)键列表。
    def get_node_names_and_namespaces(self):
        return list(self.owners)

    ##
    # @brief 返回指定owner声明的服务及fixture/type，不校验真实类型。
    # @param name owner节点名。
    # @param namespace owner命名空间，夹具默认/。
    # @return 该owner服务名及占位类型列表。
    def get_service_names_and_types_by_node(self, name, namespace):
        return [(service, ['fixture/type']) for service in self.owners[(name, namespace)]]

    ##
    # @brief 返回service_ready固定true的客户端替身；call_async只记calls并给未完成Future。
    # @param service_type 真实调用方传入的服务类型，本替身忽略。
    # @param name 记录到calls的服务绝对名。
    # @param kwargs callback_group等关键字，本替身忽略。
    # @return 带service_name/service_is_ready/call_async的NS替身。
    def create_client(self, service_type, name, **kwargs):
        ##
        # @par 内部回调 call
        # 记录(service_name, request)并返回新Future，永不自动收到真实服务结果。
        # 参数 request：需要记录的服务请求对象。
        # 返回：未完成Future。
        def call(request):
            self.calls.append((name, request))
            return Future()
        return NS(service_name=name, service_is_ready=lambda: True, call_async=call)

    ##
    # @brief 返回丢弃消息的publish替身，无ROS发布。
    # @param args ROS发布器构造位置参数，本替身忽略。
    # @return publish丢弃所有消息的NS。
    def create_publisher(self, *args):
        return NS(publish=lambda message: None)


##
# @brief 六服务×count0/2×四请求入口均先拓扑拒绝，calls必须为空，不能只检查当前所用服务。
# @param service 六项场景/规划/IK/FK服务之一。
# @param count 0缺失或2重复实例。
# @param operation validity/fk/ik/plan请求入口。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 每服务count1但Apply/Get与规划分别归不同owner仍拒绝，避免单计数掩盖两个world。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_single_counts_with_split_scene_and_planning_owners_are_rejected():
    graph = Graph()
    graph.owners = {('scene_a', '/'): set(SERVICES[:2]),
                    ('planner_b', '/'): set(SERVICES[2:])}
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    with pytest.raises(ValueError, match='MoveIt'):
        planner.request_validity([0.] * 6)
    assert graph.calls == []


##
# @brief 唯一公共owner下validity正常入队一次，防止门条件阻断正常路径。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_single_common_owner_preserves_normal_request():
    graph = Graph()
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    planner.request_validity([0.] * 6)
    assert [name for name, _ in graph.calls] == ['/check_state_validity']


##
# @brief 构造真实ObstacleScene配图/面板替身，changes按顺序记录路径失效和Hold请求。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return (Graph, ObstacleScene, changes列表)；场景初始尚未确认同步。
def make_scene(tmp_path):
    graph = Graph()
    changes = []
    panel = NS(node=graph, last_state=None, busy=False,
               moveit_tab=NS(executing=False, auto_test=NS(active=False),
                             scene_change_requested=lambda: changes.append('invalidated')),
               cancel_motion_hold=lambda: changes.append('stop'))
    scene = ObstacleScene(panel, tmp_path/'obstacles.json', {'base_link': (0., 0.)})
    return graph, scene, changes


##
# @brief ready后plan服务count2使blocked、撤scene_ready并通知旧path失效，不能继续用旧场景。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_ready_scene_is_revoked_when_second_server_appears(tmp_path):
    graph, scene, changes = make_scene(tmp_path)
    scene.scene_ready = True
    scene.blocked = False
    graph.counts['/plan_kinematic_path'] = 2
    scene.tick()
    assert scene.blocked and not scene.scene_ready
    assert changes == ['invalidated']
    assert graph.calls == []


##
# @brief Apply Future虽成功，Get已重复时仍撤ready、不落障碍文件，不接受过时事务结果。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 初次FK未发现只等待不置error；发现后可发一次Apply；已ready后再丢FK必须error并撤ready。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief count_services抛异常必须拒绝请求，不能把未知图当唯一服务。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_graph_query_failure_is_not_treated_as_one_provider():
    graph = Graph()
    ##
    # @par 内部回调 unavailable
    # 对任意服务名抛发现不可用异常，供fail-closed测试注入。
    # 参数 name：模拟查询服务/节点名称。
    # 返回：无正常返回；总抛RuntimeError。
    def unavailable(name):
        raise RuntimeError('discovery unavailable')
    graph.count_services = unavailable
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    with pytest.raises(ValueError, match='MoveIt'):
        planner.request_validity([0.] * 6)
    assert graph.calls == []


##
# @brief 即使节点名重复完全相同，只要真实服务count2先拒绝，不能先去重名称抹掉重复实例。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_same_node_name_duplicate_is_not_deduplicated_before_rejection():
    graph = Graph()
    graph.counts['/plan_kinematic_path'] = 2
    graph.get_node_names_and_namespaces = lambda: [('manual_move_group', '/')] * 2
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    with pytest.raises(ValueError, match='MoveIt'):
        planner.request_validity([0.] * 6)
    assert not graph.calls


##
# @brief 公共五服务归manual_move_group、Get归private_12345且公共名有重复别名时，count1允许validity，兼容Jazzy架构。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_moveit_private_scene_monitor_and_same_process_name_aliases_are_supported():
    graph = Graph()
    graph.owners = {('manual_move_group', '/'): set(SERVICES)-{'/get_planning_scene'},
                    ('manual_move_group_private_12345', '/'): {'/get_planning_scene'}}
    names = list(graph.owners)
    graph.get_node_names_and_namespaces = lambda: names + [('manual_move_group', '/')] * 2
    planner = MoveItPlanner(graph, NAMES, [(-180., 180.)] * 6)
    planner.request_validity([0.] * 6)
    assert len(graph.calls) == 1


##
# @brief busy运动时拓扑丢失先invalidated再stop；恢复count1也保持闭锁，不能自动复活旧路径。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief owner换名后即使count1仍拒旧场景，显式request_change才清错并启动新Apply，未立刻ready。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 初次逐节点查询抛NodeNameNonExistentError视发现竞态，等待但不ready、不发送Apply。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_initial_node_disappearance_waits_without_opening_scene(tmp_path):
    from rclpy.node import NodeNameNonExistentError
    graph, scene, changes = make_scene(tmp_path)
    ##
    # @par 内部回调 disappeared
    # 模拟查服务期间节点消失，抛真实rclpy异常类型，不操作节点。
    # 参数 name：模拟查询服务/节点名称。
    # 参数 namespace：模拟节点命名空间。
    # 返回：无正常返回；总抛NodeNameNonExistentError。
    def disappeared(name, namespace):
        raise NodeNameNonExistentError('node disappeared')
    graph.get_service_names_and_types_by_node = disappeared
    scene.tick()
    assert scene.blocked and not scene.scene_ready and not scene.error
    assert not graph.calls


##
# @brief panel.busy=false但State core3显示运动时拓扑重复仍请求Hold，不能只依赖GUI忙标志。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_measured_motion_stops_even_if_panel_busy_flag_is_clear(tmp_path):
    graph, scene, changes = make_scene(tmp_path)
    scene.scene_ready = True
    scene.blocked = False
    scene.panel.last_state = {'core_state': 3, 'estimated_velocity_rad_s': [0.] * 6}
    graph.counts['/plan_kinematic_path'] = 2
    scene.tick()
    assert changes == ['invalidated', 'stop']


def test_unrelated_observer_disappears_without_invalidating_ready_moveit():
    from manual_moveit_planner import require_unique_moveit
    from rclpy.node import NodeNameNonExistentError
    graph = Graph()
    original = graph.get_service_names_and_types_by_node
    graph.owners[('temporary_tf_observer', '/')] = []
    def query(name, namespace):
        if name == 'temporary_tf_observer':
            raise NodeNameNonExistentError('observer exited')
        return original(name, namespace)
    graph.get_service_names_and_types_by_node = query
    assert require_unique_moveit(graph) == (('manual_move_group', '/'), ('manual_move_group', '/'))


@pytest.mark.parametrize('count', [0, 2])
def test_service_instance_change_during_owner_queries_is_rejected(count):
    from manual_moveit_planner import require_unique_moveit
    graph = Graph()
    original = graph.get_service_names_and_types_by_node
    def query(name, namespace):
        result = original(name, namespace)
        graph.counts[SERVICES[0]] = count
        return result
    graph.get_service_names_and_types_by_node = query
    with pytest.raises(ValueError):
        require_unique_moveit(graph)


def test_unknown_node_query_error_remains_rejected():
    from manual_moveit_planner import require_unique_moveit
    graph = Graph()
    graph.get_service_names_and_types_by_node = lambda *args: (_ for _ in ()).throw(RuntimeError('unknown graph failure'))
    with pytest.raises(ValueError, match='来源查询失败'):
        require_unique_moveit(graph)
