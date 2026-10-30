"""装配合同离线测试；从不执行launch或创建串口owner。"""
import importlib.util
import pytest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'launch'/f'{name}.launch.py')
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def test_control_starts_hardware_and_controllers_inactive():
    params = module('manual_ros_control').controller_parameters()
    manager = params['controller_manager']['ros__parameters']
    assert manager['hardware_components_initial_state'] == {'inactive': ['dummy_ref_real_6dof']}
    assert manager['update_rate'] == 50
    assert manager['arm_controller']['type'] == 'dummy_ros_control/GuardedJointTrajectoryController'
    assert not params['arm_controller']['ros__parameters']['allow_partial_joints_goal']


def test_deployment_pin_is_required_before_reading_paths():
    import pytest
    control = module('manual_ros_control')
    with pytest.raises((ValueError, FileNotFoundError)):
        control.read_bound_deployment('/missing', '', '/missing')


@pytest.mark.parametrize("reuse", [False, True])
def test_graph_has_single_planner_single_actual_publisher_and_inactive_spawner(monkeypatch, tmp_path, reuse):
    from launch import LaunchContext
    from launch_ros.actions import Node
    control = module('manual_ros_control')
    names = ['Joint'+str(i) for i in range(1, 7)]
    cfg = {'joints': [{'name': name, 'limits_deg': [-60, 60]} for name in names], 'rest_deg': [0.0]*6}
    model = {'joints': [{'model_limits_rad': [-2.0, 2.0]} for _ in names]}
    identity = {key: 'a'*64 for key in ('contract_sha256', 'model_sha256', 'firmware_sha256')}
    monkeypatch.setattr(control, 'read_bound_deployment', lambda *args: (tmp_path, cfg,
        {'--endpoint': 'serial//test', '--device': '1'*32}, identity, model))
    monkeypatch.setattr(control, 'get_package_share_directory', lambda name: str(tmp_path))

    class Builder:
        robot_description = {'robot_description': '<robot name="test"/>'}
        def __init__(self, *args, **kwargs): pass
        def robot_description(self, *args, **kwargs): return self
        def joint_limits(self, *args, **kwargs): return self
        def to_moveit_configs(self):
            class Config:
                robot_description = {'robot_description': '<robot name="test"/>'}
                def to_dict(self): return self.robot_description
            return Config()
    monkeypatch.setattr(control, 'MoveItConfigsBuilder', Builder)
    context = LaunchContext()
    context.launch_configurations.update({key: '' for key in ('deployment_root', 'deployment_sha256',
        'sdk_prefix', 'report_path', 'raw_evidence_path', 'goal_timeline_path')})
    context.launch_configurations.update(initial_pose='manual_limits', auto_calibrate_and_test='false',
                                        operator_confirmed_double_limits='false', reuse_display_planning=str(reuse).lower(),
                                        cylinder_pose='[]', cylinder_dimensions='[]')
    nodes = [action for action in control.start(context) if isinstance(action, Node)]
    executables = [node.node_executable for node in nodes]
    assert executables.count('move_group') == (0 if reuse else 1)
    assert executables.count('robot_state_publisher') == 1
    assert executables.count('device_action_gate') == 1
    assert executables.count('manual_ros_baseline_node') == 1
    from launch_ros.utilities import evaluate_parameters
    gate = next(node for node in nodes if node.node_executable == 'device_action_gate')
    from launch.utilities import perform_substitutions
    def description_topic(node):
        remappings = [(perform_substitutions(context, source), perform_substitutions(context, target))
                      for source, target in node._Node__remappings]
        return dict(remappings)['/robot_description']
    topic=description_topic(gate)
    publisher=next(node for node in nodes if node.node_executable == 'robot_state_publisher')
    assert description_topic(publisher)==topic
    # 异常退出后旧发布器仍可能存活；下一代不能消费其transient-local硬件装配。
    assert topic!='/manual_control/robot_description'
    next_nodes=[action for action in control.start(context) if isinstance(action,Node)]
    next_gate=next(node for node in next_nodes if node.node_executable=='device_action_gate')
    next_publisher=next(node for node in next_nodes if node.node_executable=='robot_state_publisher')
    assert description_topic(next_gate)==description_topic(next_publisher)
    assert description_topic(next_gate)!=topic
    assert gate._Node__node_name is None  # 不得把同进程controller_manager一起全局改名。
    params = evaluate_parameters(context, gate._Node__parameters)[-1]
    assert params['operator_confirmed_double_limits'] is False
    assert params['manual_reference_mode'] is True
    assert params['permit_remaining_us'] == 900000000
    baseline = next(node for node in nodes if node.node_executable == 'manual_ros_baseline_node')
    params = evaluate_parameters(context, baseline._Node__parameters)[-1]
    assert params['auto_calibrate_and_test'] is False
    assert params['total_timeout_s'] == 840
    assert 'status_max_age_ms' not in params  # SDK默认/上限由C++公布，launch不覆盖。
    spawner = next(node for node in nodes if node.node_executable == 'spawner')
    from launch.utilities import perform_substitutions
    assert '--inactive' in [perform_substitutions(context, item) for item in spawner.cmd[1:]]


@pytest.mark.parametrize('backend', ['real', 'ros', 'simulator'])
def test_each_initial_gui_target_carries_ros_owner_config_with_shared_display(tmp_path, monkeypatch, backend):
    import hashlib
    import json
    import shutil
    from types import SimpleNamespace
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    panel = module('manual_joint_panel')
    share = tmp_path/'share'
    (share/'launch').mkdir(parents=True)
    (share/'config').mkdir()
    shutil.copyfile(ROOT/'launch/manual_ros_control.launch.py', share/'launch/manual_ros_control.launch.py')
    model_path = tmp_path/'robot.xacro'
    model_path.write_text('<robot name="test"/>')
    sdk = tmp_path/'sdk'
    sdk_model = sdk/'share/dummy_ref/robot_contract/model/model.yaml'
    sdk_model.parent.mkdir(parents=True)
    joints = [{'name': 'Joint'+str(i), 'direction': -1, 'gear_ratio': 50,
               'legacy_limits_deg': [-90, 90]} for i in range(1, 7)]
    sdk_model.write_text(json.dumps({'joints': joints}))
    release = tmp_path/'release'
    release.mkdir()
    firmware = b'bound-firmware'
    (release/'firmware.bin').write_bytes(firmware)
    identity = {'contract_sha256': 'a'*64, 'model_sha256': hashlib.sha256(sdk_model.read_bytes()).hexdigest(),
                'profile_sha256': module('manual_ros_control').LIVE_POLICY,
                'firmware_sha256': hashlib.sha256(firmware).hexdigest()}
    cfg = {'description_xacro': str(model_path), 'joints': [dict(j, nominal_limits_deg=j['legacy_limits_deg']) for j in joints],
           'native_arguments': ['--scope', 'manual-home-empty-supported', '--profile', '201', '--mask', '63']}
    for arg, key in (('--contract', 'contract_sha256'), ('--model', 'model_sha256'),
                     ('--profile-sha256', 'profile_sha256'), ('--firmware', 'firmware_sha256')):
        cfg['native_arguments'] += [arg, identity[key]]
    original = release/'panel-config.json'
    original.write_text(json.dumps(cfg))
    (release/'deployment.json').write_text(json.dumps({'format': 'dummy_manual_panel_deployment_v1',
        'identity': identity, 'files': {name: hashlib.sha256((release/name).read_bytes()).hexdigest()
                                      for name in ('panel-config.json', 'firmware.bin')}}))
    simulated = share/'config/manual_joint_panel.json'
    simulated.write_text(json.dumps(cfg))
    def resolve(argument, selected, template):
        return simulated if selected == 'simulator' else original
    def load(path, selected):
        return json.loads(Path(path).read_text())
    monkeypatch.setattr(panel, '_config_helpers', lambda: SimpleNamespace(resolve_config_path=resolve, load_panel_config=load))
    monkeypatch.setattr(panel, 'get_package_share_directory', lambda name: str(share))
    monkeypatch.setattr(panel, 'Node', lambda **kwargs: SimpleNamespace(**kwargs))
    context = LaunchContext()
    context.launch_configurations.update(backend=backend, config='', sdk_prefix=str(sdk), moveit='true',
        report=str(tmp_path/'report'), auto_calibrate_and_test='false', operator_confirmed_double_limits='false',
        cylinder_pose='[]', cylinder_dimensions='[]', initial_pose='manual_limits')
    actions = panel.build_nodes(context)
    gui = next(action for action in actions if getattr(action, 'executable', '') == 'manual_joint_panel.py')
    runtime_path = Path(gui.arguments[1])
    runtime = json.loads(runtime_path.read_text())
    assert runtime['sdk_prefix'] == str(sdk)
    assert Path(runtime['ros_controller_config']).is_file()
    assert runtime['deployment_source_config'] == str(original)
    assert runtime['operator_confirmed_double_limits'] is False
    assert 'reuse_display_planning:=true' in runtime['ros_launch_command']
    assert 'operator_confirmed_double_limits:=false' in runtime['ros_launch_command']
    assert sum(isinstance(action, IncludeLaunchDescription) for action in actions) == 1
    assert sum(getattr(action, 'executable', '') == 'robot_state_publisher' for action in actions) == 2
