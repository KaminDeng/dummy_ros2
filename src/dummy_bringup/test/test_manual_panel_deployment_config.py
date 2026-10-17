"""真实临时产物验证离线部署绑定，不连接ROS或设备。"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/render_manual_panel_config.py'
spec = importlib.util.spec_from_file_location('panel_config_renderer', SCRIPT)
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)


@pytest.fixture
def inputs(tmp_path):
    deployment = {}
    for name in ('native', 'firmware'):
        p = tmp_path / name
        p.write_bytes(('real ' + name).encode())
        deployment[name + '_path'] = str(p)
        deployment[name + '_sha256'] = hashlib.sha256(p.read_bytes()).hexdigest()
    deployment.update(contract_sha256='11' * 32, model_sha256='22' * 32, profile_sha256='33' * 32)
    decoder = tmp_path / 'decoder'
    decoder.mkdir()
    (decoder / 'join_real_6dof_evidence.py').write_text('# fixture decoder\n')
    deployment['decoder_directory'] = str(decoder)
    description = tmp_path / 'model.xacro'
    description.write_text('<robot name="fixture"/>')
    template = json.loads((ROOT / 'config/manual_joint_panel.json').read_text())
    return template, deployment, description


def test_render_binds_real_artifacts_and_preserves_all_motion_parameters(inputs):
    template, deployment, description = inputs
    original = copy.deepcopy(template)
    result = renderer.render_config(template, deployment, description)
    assert template == original
    allowed = {'native_binary', 'native_arguments', 'description_xacro', 'decoder_directory', 'deployment_required'}
    assert {k: v for k, v in result.items() if k not in allowed} == {k: v for k, v in template.items() if k not in allowed}
    for flag, key in (('--firmware', 'firmware_sha256'), ('--contract', 'contract_sha256'),
                      ('--model', 'model_sha256'), ('--profile-sha256', 'profile_sha256')):
        assert result['native_arguments'][result['native_arguments'].index(flag) + 1] == deployment[key]
    assert result['native_binary'] == deployment['native_path']
    assert result['description_xacro'] == str(description)
    assert result['decoder_directory'] == deployment['decoder_directory']


@pytest.mark.parametrize('mode', ['native-hash', 'firmware-hash', 'identity-missing', 'identity-invalid',
                                  'model-missing', 'decoder-missing', 'decoder-entry-missing', 'flag-missing', 'flag-duplicate'])
def test_invalid_deployment_rejected_without_mutating_template(inputs, mode):
    template, deployment, description = inputs
    if mode.endswith('-hash'):
        deployment[mode.split('-')[0] + '_sha256'] = '00' * 32
    elif mode == 'identity-missing':
        deployment.pop('contract_sha256')
    elif mode == 'identity-invalid':
        deployment['profile_sha256'] = 'not-a-hash'
    elif mode == 'model-missing':
        description.unlink()
    elif mode == 'decoder-missing':
        deployment['decoder_directory'] += '/missing'
    elif mode == 'decoder-entry-missing':
        (Path(deployment['decoder_directory']) / 'join_real_6dof_evidence.py').unlink()
    elif mode == 'flag-missing':
        template['native_arguments'].remove('--firmware')
    else:
        template['native_arguments'].extend(['--model', ''])
    original = copy.deepcopy(template)
    with pytest.raises(ValueError):
        renderer.render_config(template, deployment, description)
    assert template == original


def test_cli_creates_output_exclusively(inputs, tmp_path):
    template, deployment, description = inputs
    tp, dp, output = tmp_path / 'template.json', tmp_path / 'deployment.json', tmp_path / 'output.json'
    tp.write_text(json.dumps(template))
    dp.write_text(json.dumps(deployment))
    argv = [sys.executable, str(SCRIPT), '--template', str(tp), '--deployment', str(dp),
            '--description-xacro', str(description), '--output', str(output)]
    assert subprocess.run(argv, check=False).returncode == 0
    before = output.read_bytes()
    assert subprocess.run(argv, check=False).returncode != 0
    assert output.read_bytes() == before
