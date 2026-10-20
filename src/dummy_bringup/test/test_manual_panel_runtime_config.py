"""模拟器独立于SDK构建；真机配置只使用明确部署的不可变产物。"""
##
# @file test_manual_panel_runtime_config.py
# @brief 模拟模板与real固定部署入口的配置解析回归。
#
# 仅导入配置模块、建临时文件与修改进程环境；不执行Native或扫描硬件。显式旧配置兼容与带manifest部署摘要校验是不同路径，不能宣称所有real配置都经过同等不可变产物校验。
import importlib.util
import json
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


##
# @brief 从源码独立加载manual_panel_config，避免要求SDK构建或面板GUI。
# @return 配置模块对象；未解析或启动任何后端。
def runtime():
    spec = importlib.util.spec_from_file_location('manual_panel_config', SCRIPTS/'manual_panel_config.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


##
# @brief simulator可从空native模板补安装模型并撤deployment_required，不依赖real部署入口。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_simulator_uses_template_without_native_or_sdk(tmp_path, monkeypatch):
    module = runtime()
    monkeypatch.setenv('XDG_DATA_HOME', str(tmp_path/'data'))
    template = tmp_path/'template.json'
    template.write_text(json.dumps(dict(deployment_required=True, description_xacro='', native_binary='')))
    description = tmp_path/'robot.xacro'; description.write_text('<robot name="fixture"/>')
    monkeypatch.setattr(module, 'installed_description', lambda: description)
    assert module.resolve_config_path('', 'simulator', template) == template
    config = module.load_panel_config(template, 'simulator')
    assert config['description_xacro'] == str(description)
    assert config['native_binary'] == ''
    assert not config.get('deployment_required')


##
# @brief 默认real只解析XDG固定current/panel-config.json；缺部署报错，不回退搜索构建目录。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_real_uses_fixed_current_and_never_scans_builds(tmp_path, monkeypatch):
    module = runtime(); monkeypatch.setenv('XDG_DATA_HOME', str(tmp_path))
    expected = tmp_path/'dummy_ros/manual_panel/current/panel-config.json'
    assert module.resolve_config_path('', 'real', Path('/unused')) == expected
    with pytest.raises(ValueError, match='部署'):
        module.load_panel_config(expected, 'real')


##
# @brief 显式路径的可执行权限fixture、decoder/XML满足旧配置路径，不要求manifest；不运行其native。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_explicit_legacy_config_remains_supported(tmp_path):
    module = runtime(); native = tmp_path/'native'; native.write_text('fixture'); native.chmod(0o755)
    decoder = tmp_path/'decoder'; decoder.mkdir(); (decoder/'join_real_6dof_evidence.py').write_text('')
    description = tmp_path/'robot.xacro'; description.write_text('<robot/>')
    path = tmp_path/'explicit.json'; path.write_text(json.dumps(dict(native_binary=str(native),decoder_directory=str(decoder),description_xacro=str(description))))
    assert module.resolve_config_path(str(path),'real',Path('/unused')) == path
    assert module.load_panel_config(path,'real')['native_binary'] == str(native)


##
# @brief manifest声明全零SHA而native字节changed，real加载必须以校验错误拒绝。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_manifest_native_hash_mismatch_rejected(tmp_path):
    module = runtime(); path = tmp_path/'panel-config.json'
    path.write_text(json.dumps(dict(deployment_manifest='deployment.json')))
    (tmp_path/'deployment.json').write_text(json.dumps(dict(files={'native': '00'*32})))
    (tmp_path/'native').write_text('changed')
    with pytest.raises(ValueError, match='校验'):
        module.load_panel_config(path, 'real')
