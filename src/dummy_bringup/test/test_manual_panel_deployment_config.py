"""真实临时产物验证离线部署绑定，不连接ROS或设备。"""
##
# @file test_manual_panel_deployment_config.py
# @brief 部署配置render的文件SHA绑定、只改白名单及排他输出测试。
#
# native/firmware是临时文本文件，XML和decoder也是最小fixture；校验字节与配置合同不等于真实编译产物或硬件身份。CLI用例仅运行render配置脚本写文件，不部署、flash或连接设备。
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


##
# @brief 生成有真实文件SHA的native/firmware文本、固定合同摘要、最小decoder/XML及原面板配置模板。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return (template字典, deployment字典, description路径)，仅存在于tmp_path。
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


##
# @brief render仅改允许的五个部署字段，原模板不变，各身份flag、二进制/模型/decoder路径准确绑定。
# @param inputs inputs fixture返回的模板、临时部署字典和模型路径。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 九种hash/身份/文件/flag缺失或重复均ValueError，且调用方模板不被部分修改。
# @param inputs inputs fixture返回的模板、临时部署字典和模型路径。
# @param mode 九种独立负例之一：摘要/身份/文件/flag缺失或重复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 临时输入运行renderer CLI，首次创建成功，再写同输出必须失败且字节不变；只产生配置文件。
# @param inputs inputs fixture返回的模板、临时部署字典和模型路径。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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


##
# @brief 段watchdog秒数拒绝0/负/非有限/>660、bool及字符串，不能以隐式类型转换放行。
# @param inputs inputs fixture返回的模板、临时部署字典和模型路径。
# @param value 非法秒数或类型，包括bool/字符串，避免隐式转换。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
@pytest.mark.parametrize('value', [0, -1, float('inf'), float('nan'), 661, True, '660'])
def test_render_rejects_unbounded_or_invalid_segment_watchdog(inputs, value):
    template, deployment, description = inputs
    template['moveit_segment_timeout_s'] = value
    with pytest.raises(ValueError, match='moveit_segment_timeout_s'):
        renderer.render_config(template, deployment, description)


##
# @brief 有效模板最终仍发布660.0s段监督上限，不擅改运动参数。
# @param inputs inputs fixture返回的模板、临时部署字典和模型路径。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_render_publishes_validated_segment_watchdog(inputs):
    template, deployment, description = inputs
    assert renderer.render_config(template, deployment, description)['moveit_segment_timeout_s'] == 660.
