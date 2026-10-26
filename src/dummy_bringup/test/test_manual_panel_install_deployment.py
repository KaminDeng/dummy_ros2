"""部署后移走构建目录，固定入口仍能独立校验与运行。"""
##
# @file test_manual_panel_install_deployment.py
# @brief 临时XDG部署副本可脱离原构建路径并拒绝字节漂移。
#
# deploy会真实写tmp_path下部署目录，输入native仅exit0 shell、firmware仅文本、摘要仅fixture常量；测试不运行native/flash，不验证真实设备身份。decoder通过load_decoder导入，但没有调用返回的解码函数。
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
from pathlib import Path
import sys

import pytest

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
sys.path.insert(0,str(SCRIPTS.parents[2]))


##
# @brief 部署到固定current入口后删原native/firmware，real配置仍读副本且decoder可加载；随后篡改副本要求hash校验拒绝。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_deploy_copies_artifacts_and_rejects_unmatched_identity(tmp_path, monkeypatch):
    spec=importlib.util.spec_from_file_location('deploy_manual_panel',SCRIPTS/'deploy_manual_panel.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    native=tmp_path/'build/native';native.parent.mkdir();native.write_bytes(b'#!/bin/sh\nexit 0\n');native.chmod(0o755)
    firmware=tmp_path/'build/firmware';firmware.write_bytes(b'firmware')
    identity=tmp_path/'identity.json';identity.write_text(json.dumps(dict(contract_sha256='11'*32,model_sha256='22'*32,profile_sha256='33'*32)))
    description=tmp_path/'robot.xacro';description.write_text('<robot name="fixture"/>')
    monkeypatch.setenv('XDG_DATA_HOME',str(tmp_path/'data'))
    # 协议输入显式携带已有缓存；部署必须排除缓存，不能改动实际SDK。
    selected=os.environ.get('DPSDK_PROTOCOL_RELEASE')
    if not selected:
        selected=str(Path(os.environ['DPSDK_HOST_SDK_PREFIX'])/'lib/cmake/DpProtocol')
    input_protocol=tmp_path/'protocol-fixture'
    shutil.copytree(selected,input_protocol)
    cache=input_protocol/'include/__pycache__/fixture.pyc'
    cache.parent.mkdir(parents=True,exist_ok=True);cache.write_bytes(b'input-cache')
    (input_protocol/'include/fixture.pyo').write_bytes(b'input-cache')
    monkeypatch.setenv('DPSDK_PROTOCOL_RELEASE',str(input_protocol))
    config_path=module.deploy(native,firmware,identity,description)
    assert config_path==tmp_path/'data/dummy_ros/manual_panel/current/panel-config.json'
    native.unlink();firmware.unlink()
    import manual_panel_config
    config=manual_panel_config.load_panel_config(config_path,'real')
    assert Path(config['native_binary']).read_bytes().startswith(b'#!/bin/sh')
    assert '/build/' not in config['native_binary']
    deployed_protocol=Path(config['decoder_directory'])/'protocol'
    assert cache.read_bytes()==b'input-cache'
    assert not list(deployed_protocol.rglob('__pycache__'))
    assert not list(deployed_protocol.rglob('*.py[co]'))
    deployment=json.loads((config_path.parent/'deployment.json').read_text())
    assert not any('__pycache__' in name or name.endswith(('.pyc','.pyo')) for name in deployment['files'])
    for relative, digest in deployment['files'].items():
        assert hashlib.sha256((config_path.parent/relative).read_bytes()).hexdigest()==digest
    assert callable(manual_panel_config.load_decoder(config))
    # 独立进程在源码/外部发布不可用时仍使用部署自带公共reader实际解码。
    from scripts.dummy.selected_protocol import current_layout
    layout=current_layout('DummyJointStateV2')
    raw=layout.encode({'schema_major':layout.constant('SCHEMA_MAJOR'),
                       'schema_minor':layout.constant('SCHEMA_MINOR')})
    code="""import importlib.util, pathlib, sys
p=pathlib.Path(sys.argv[1])/'join_real_6dof_evidence.py'
s=importlib.util.spec_from_file_location('isolated_panel_decoder',p)
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
assert m._decode('DummyJointStateV2',bytes.fromhex(sys.argv[2]))['schema_minor']==5
"""
    env={**os.environ,'DPSDK_PROTOCOL_RELEASE':'/missing-release',
         'DPSDK_PROTOCOL_SOURCE_ROOT':'/missing-source','PYTHONPATH':''}
    subprocess.run([sys.executable,'-c',code,config['decoder_directory'],raw.hex()],
                   cwd=tmp_path,env=env,check=True)
    assert not list((Path(config['decoder_directory'])/'protocol').rglob('__pycache__'))
    Path(config['native_binary']).write_bytes(b'corrupted')
    with pytest.raises(ValueError,match='校验'):
        manual_panel_config.load_panel_config(config_path,'real')


@pytest.mark.parametrize('endpoint', [None, 'serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_EXAMPLE_DEVICE_0001-if00#baudrate=115200'])
def test_endpoint_override_is_pinned_without_changing_default(tmp_path, monkeypatch, endpoint):
    import deploy_manual_panel as module
    native=tmp_path/'native';native.write_text('#!/bin/sh\nexit 0\n');native.chmod(0o755)
    firmware=tmp_path/'firmware';firmware.write_bytes(b'fixture')
    identity=tmp_path/'identity.json';identity.write_text(json.dumps(dict(contract_sha256='11'*32,model_sha256='22'*32,profile_sha256='33'*32)))
    description=tmp_path/'robot.xacro';description.write_text('<robot name="fixture"/>')
    monkeypatch.setenv('XDG_DATA_HOME',str(tmp_path/'data'))
    template=json.loads(module.resources()[1].read_text())
    original=template['native_arguments'][template['native_arguments'].index('--endpoint')+1]
    config_path=module.deploy(native,firmware,identity,description,endpoint=endpoint)
    from manual_panel_config import load_panel_config
    config=load_panel_config(config_path,'real')
    argv=config['native_arguments']
    assert argv[argv.index('--endpoint')+1]==(original if endpoint is None else endpoint)
    manifest=json.loads((config_path.parent/'deployment.json').read_text())
    assert manifest['files']['panel-config.json']==hashlib.sha256(config_path.read_bytes()).hexdigest()
    for relative, digest in manifest['files'].items():
        assert hashlib.sha256((config_path.parent/relative).read_bytes()).hexdigest()==digest


@pytest.mark.parametrize('endpoint', ['', '   ', '--endpoint=serial//dev/ttyACM0', '--execute'])
def test_endpoint_override_rejects_empty_or_option_style(tmp_path, endpoint):
    import deploy_manual_panel as module
    with pytest.raises(ValueError,match='endpoint'):
        module.deploy('/not-read/native','/not-read/firmware','/not-read/identity','/not-read/description',endpoint=endpoint)


def test_cli_forwards_explicit_endpoint(monkeypatch, capsys):
    import deploy_manual_panel as module
    endpoint='serial//dev/serial/by-id/EXAMPLE_DEVICE_0001#baudrate=115200'
    calls=[]
    monkeypatch.setattr(module,'deploy',lambda *args,**kwargs:calls.append((args,kwargs)) or Path('/fixture/config'))
    monkeypatch.setattr(sys,'argv',['deploy','--native','/n','--firmware','/f','--identity','/i',
                                  '--description-xacro','/d','--endpoint',endpoint])
    module.main()
    assert calls[0][1]=={'endpoint':endpoint}
