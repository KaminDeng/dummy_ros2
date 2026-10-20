"""面板 launch 必须在独立安装区工作，且关闭 MoveIt 时不读取其包。"""
##
# @file test_manual_panel_launch.py
# @brief 安装布局、可选MoveIt依赖及参数构造的离线launch测试。
#
# build_nodes返回的Node被NS替身替换，MoveItConfigsBuilder在相应用例也替换；从不执行LaunchService或启动节点。临时helper只为隔离安装路径，不能把它当真实配置校验覆盖。
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
import pytest

ROOT = Path(__file__).resolve().parents[1]


##
# @brief 按名称从源码launch目录动态导入模块，不执行launch动作。
# @param name launch文件基名，不含.launch.py。
# @return 已执行顶层定义的launch模块。
def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'launch' / (name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


##
# @brief 创建隔离share/lib安装布局及最小helper，拦截ament查找并故意令MoveIt包缺失。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return (临时share, 模型路径, 包查找记录列表)。
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

    ##
    # @par 内部回调 lookup
    # 记录包查找；bringup返回临时share，MoveIt包抛PackageNotFoundError，其它包转原lookup。
    # 参数 package：ament包名。
    # 返回：临时share/原lookup结果，或对MoveIt包抛错。
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


##
# @brief 建立LaunchContext，默认simulator/moveit=false及占位report，再覆盖调用方配置。
# @param values 覆盖默认launch字符串配置的关键字字典。
# @return LaunchContext，仅持有字符串配置，不启动launch。
def context(**values):
    result = LaunchContext()
    result.launch_configurations.update(config='', backend='simulator', moveit='false', report='/tmp/report')
    result.launch_configurations.update(values)
    return result


##
# @brief Node为NS时核面板参数使用安装template/model，moveit关闭不查MoveIt包也不传--moveit。
# @param installation 隔离安装布局fixture：(share, 模型路径, 查找记录)。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 仅检查生成器entities不存在提前IncludeLaunchDescription，保证后端解析前不急加载MoveIt。
# @param installation 隔离安装布局fixture：(share, 模型路径, 查找记录)。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_moveit_is_not_eagerly_included_before_backend_resolution(installation):
    module = load_launch('manual_joint_panel')
    from launch.actions import IncludeLaunchDescription
    entities = module.generate_launch_description().entities
    assert not any(isinstance(action, IncludeLaunchDescription) for action in entities)


##
# @brief moveit=true且包缺失时build_nodes抛可操作错误，指明构建脚本及moveit:=false。
# @param installation 隔离安装布局fixture：(share, 模型路径, 查找记录)。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_missing_moveit_explains_build_and_optional_disable(installation):
    module = load_launch('manual_joint_panel')
    with pytest.raises((RuntimeError, ValueError), match=r'build_manual_panel.sh.*moveit:=false'):
        module.build_nodes(context(moveit='true'))


##
# @brief Builder/Node替身核解析模型路径、backend声明及OMPL/TOTG配置键值；不执行规划或证明避碰分辨率足够。
# @param installation 隔离安装布局fixture：(share, 模型路径, 查找记录)。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_planning_passes_backend_and_resolved_model(installation, monkeypatch):
    share, model, _ = installation
    config = {'description_xacro': str(model), 'joints': [],
              'motion_speed_max_deg_s': 60., 'motion_acceleration_max_rad_s2': 4.}
    (share / 'config/manual_joint_panel.json').write_text(json.dumps(config))
    module = load_launch('manual_moveit_planning')
    loaded = []

    ##
    # @par 内部类型 Builder
    # 局部MoveIt配置Builder替身，只记录传入值并返回最小配置。
    class Builder:
        ##
        # @par 内部回调 __init__
        # 忽略Builder构造参数，不读真实MoveIt配置。
        # 参数 args：额外位置参数，替身忽略。
        # 参数 kwargs：额外关键字参数，替身忽略。
        # 返回：无；仅初始化本测试替身字段。
        def __init__(self, *args, **kwargs):
            pass
        ##
        # @par 内部回调 robot_description
        # 记录file_path并返回self供链式调用，不解析xacro。
        # 参数 file_path：需要记录的模型路径。
        # 返回：self供链式调用。
        def robot_description(self, file_path):
            loaded.append(file_path)
            return self
        ##
        # @par 内部回调 planning_pipelines
        # 忽略规划管线kwargs并返回self，不创建插件。
        # 参数 kwargs：额外关键字参数，替身忽略。
        # 返回：self供链式调用。
        def planning_pipelines(self, **kwargs):
            return self
        ##
        # @par 内部回调 to_moveit_configs
        # 返回最小joint_limits和空to_dict替身，使build_nodes参数装配可独立断言。
        # 返回：最小NS配置替身。
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
