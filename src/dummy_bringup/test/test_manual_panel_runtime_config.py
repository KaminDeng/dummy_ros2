"""模拟器独立于SDK构建；真机配置只使用明确部署的不可变产物。"""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


def runtime():
    spec = importlib.util.spec_from_file_location('manual_panel_config', SCRIPTS/'manual_panel_config.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def test_real_uses_fixed_current_and_never_scans_builds(tmp_path, monkeypatch):
    module = runtime(); monkeypatch.setenv('XDG_DATA_HOME', str(tmp_path))
    expected = tmp_path/'dummy_ros/manual_panel/current/panel-config.json'
    assert module.resolve_config_path('', 'real', Path('/unused')) == expected
    with pytest.raises(ValueError, match='部署'):
        module.load_panel_config(expected, 'real')


def test_explicit_legacy_config_remains_supported(tmp_path):
    module = runtime(); native = tmp_path/'native'; native.write_text('fixture'); native.chmod(0o755)
    decoder = tmp_path/'decoder'; decoder.mkdir(); (decoder/'join_real_6dof_evidence.py').write_text('')
    description = tmp_path/'robot.xacro'; description.write_text('<robot/>')
    path = tmp_path/'explicit.json'; path.write_text(json.dumps(dict(native_binary=str(native),decoder_directory=str(decoder),description_xacro=str(description))))
    assert module.resolve_config_path(str(path),'real',Path('/unused')) == path
    assert module.load_panel_config(path,'real')['native_binary'] == str(native)


def test_manifest_native_hash_mismatch_rejected(tmp_path):
    module = runtime(); path = tmp_path/'panel-config.json'
    path.write_text(json.dumps(dict(deployment_manifest='deployment.json')))
    (tmp_path/'deployment.json').write_text(json.dumps(dict(files={'native': '00'*32})))
    (tmp_path/'native').write_text('changed')
    with pytest.raises(ValueError, match='校验'):
        module.load_panel_config(path, 'real')
