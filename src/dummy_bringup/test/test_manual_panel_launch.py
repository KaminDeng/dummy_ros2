"""面板 launch 必须在独立安装区工作，且关闭 MoveIt 时不读取其包。"""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'launch' / (name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def installation(tmp_path, monkeypatch):
    share = tmp_path / 'share/dummy_bringup'
    (share / 'config').mkdir(parents=True)
    (share / 'launch').mkdir()
    model = tmp_path / 'installed-description.xacro'
    model.write_text('<robot name="test"/>')
    template = {'deployment_required': True, 'description_xacro': str(model)}
    (share / 'config/manual_joint_panel.json').write_text(json.dumps(template))
    helper_dir = tmp_path / 'lib/dummy_bringup'
    helper_dir.mkdir(parents=True)
    # 配置加载本身另有集成测试；这里隔离 ament 安装布局与 launch 参数传递。
    (helper_dir / 'manual_panel_config.py').write_text('''import json
from pathlib import Path
def resolve_config_path(argument, backend, template):
    assert backend == 'simulator'
    return Path(argument) if argument else template
def load_panel_config(path, backend, description_xacro=''):
    assert backend == 'simulator'
    return json.loads(path.read_text())
''')
    seen = []
    import ament_index_python.packages
    original_lookup = ament_index_python.packages.get_package_share_directory

    def lookup(package):
        seen.append(package)
        if package == 'dummy_bringup':
            return str(share)
        if package == 'dummy_moveit_config':
            raise ament_index_python.packages.PackageNotFoundError(package)
        return original_lookup(package)

    import ament_index_python.packages
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', lookup)
    monkeypatch.setattr(ament_index_python.packages, 'get_package_prefix', lambda _: str(tmp_path))
    return share, model, seen


def context(**values):
    result = LaunchContext()
    result.launch_configurations.update(config='', backend='simulator', moveit='false', report='/tmp/report')
    result.launch_configurations.update(values)
    return result


def test_simulator_default_uses_installed_template_without_moveit(installation, monkeypatch):
    share, model, seen = installation
    module = load_launch('manual_joint_panel')
    monkeypatch.setattr(module, 'Node', lambda **kw: SimpleNamespace(**kw))
    actions = module.build_nodes(context())
    panels = [node for node in actions if getattr(node, 'executable', '') == 'manual_joint_panel.py']
    assert len(panels) == 1
    args = panels[0].arguments
    assert args[args.index('--config') + 1] == str(share / 'config/manual_joint_panel.json')
    assert args[args.index('--description-xacro') + 1] == str(model)
    assert '--moveit' not in args
    assert seen == ['dummy_bringup']


def test_moveit_is_not_eagerly_included_before_backend_resolution(installation):
    module = load_launch('manual_joint_panel')
    from launch.actions import IncludeLaunchDescription
    entities = module.generate_launch_description().entities
    assert not any(isinstance(action, IncludeLaunchDescription) for action in entities)


def test_missing_moveit_explains_build_and_optional_disable(installation):
    module = load_launch('manual_joint_panel')
    with pytest.raises((RuntimeError, ValueError), match=r'build_manual_panel.sh.*moveit:=false'):
        module.build_nodes(context(moveit='true'))


def test_planning_passes_backend_and_resolved_model(installation, monkeypatch):
    share, model, _ = installation
    config = {'description_xacro': str(model), 'joints': [],
              'motion_speed_max_deg_s': 60., 'motion_acceleration_max_rad_s2': 4.}
    (share / 'config/manual_joint_panel.json').write_text(json.dumps(config))
    module = load_launch('manual_moveit_planning')
    loaded = []

    class Builder:
        def __init__(self, *args, **kwargs):
            pass
        def robot_description(self, file_path):
            loaded.append(file_path)
            return self
        def planning_pipelines(self, **kwargs):
            return self
        def to_moveit_configs(self):
            return SimpleNamespace(joint_limits={'robot_description_planning': {'joint_limits': {}}},
                                   to_dict=lambda: {})

    monkeypatch.setattr(module, '_moveit_builder', lambda: Builder)
    monkeypatch.setattr(module, 'Node', lambda **kw: SimpleNamespace(**kw))
    nodes=module.build_nodes(context())
    assert len(nodes) == 1
    assert nodes[0].parameters[-1]['ompl.totg.path_tolerance']==.01
    parameters=nodes[0].parameters[-1]
    assert parameters['ompl.dummy_arm.default_planner_config']=='RRTConnectFast'
    assert parameters['ompl.planner_configs.RRTConnectFast.longest_valid_segment_fraction']==.01
    assert parameters['ompl.planner_configs.RRTConnectPrecise.longest_valid_segment_fraction']==.0005
    assert loaded == [str(model)]
    arguments = {entity.name for entity in module.generate_launch_description().entities
                 if hasattr(entity, 'name')}
    assert 'backend' in arguments
