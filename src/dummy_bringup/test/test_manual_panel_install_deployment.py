"""部署后移走构建目录，固定入口仍能独立校验与运行。"""
##
# @file test_manual_panel_install_deployment.py
# @brief 临时XDG部署副本可脱离原构建路径并拒绝字节漂移。
#
# deploy会真实写tmp_path下部署目录，输入native仅exit0 shell、firmware仅文本、摘要仅fixture常量；测试不运行native/flash，不验证真实设备身份。decoder通过load_decoder导入，但没有调用返回的解码函数。
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))


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
    config_path=module.deploy(native,firmware,identity,description)
    assert config_path==tmp_path/'data/dummy_ros/manual_panel/current/panel-config.json'
    native.unlink();firmware.unlink()
    import manual_panel_config
    config=manual_panel_config.load_panel_config(config_path,'real')
    assert Path(config['native_binary']).read_bytes().startswith(b'#!/bin/sh')
    assert '/build/' not in config['native_binary']
    assert callable(manual_panel_config.load_decoder(config))
    Path(config['native_binary']).write_bytes(b'corrupted')
    with pytest.raises(ValueError,match='校验'):
        manual_panel_config.load_panel_config(config_path,'real')
