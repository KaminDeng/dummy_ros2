"""正式Root发布部署与Native已接纳DPC1接合；不接设备或运行控制。"""
import ast
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import pytest
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))


def module():
    spec=importlib.util.spec_from_file_location('root_deployment_under_test',SCRIPTS/'deploy_manual_panel.py')
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


@pytest.fixture
def release(tmp_path):
    selected=os.environ.get('DPSDK_TEST_PROTOCOL_RELEASE')
    if not selected:pytest.skip('需明确当前正式DPSDK_TEST_PROTOCOL_RELEASE')
    root=tmp_path/'protocol';shutil.copytree(selected,root);return root


def codec(release):
    path=release/'python/dp_protocol_canonical_record_reader.py'
    value={};exec(compile(path.read_bytes(),str(path),'exec'),value)
    return value['SchemaCodec'].from_directory(release)


def state_record(release):
    reader=codec(release)
    def defaults(name):
        result={}
        for f in reader.fields(name):
            kind=f['type'];base=kind['base_type']
            result[f['name']]=(False if base=='Bool' else 0. if base in ('Float','Double') else
                [defaults(kind['reference'])for _ in range(reader._capacity(name,f))] if base=='Vector'and kind['element']=='Obj' else
                bytes(reader._capacity(name,f))if base=='Vector'and kind['element']=='UByte' else
                [0]*reader._capacity(name,f)if base=='Vector'else 0)
        return result
    name='dp.protocol.dummy.JointState';fields=defaults(name)
    for i,joint in enumerate(fields['joints']):joint['joint_id']=i
    return reader,reader.encode_record(name,fields)


def method(name,scope):
    tree=ast.parse((SCRIPTS/'manual_joint_panel.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and any(isinstance(x,ast.FunctionDef)and x.name==name for x in n.body))
    node=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),'actual-panel-method','exec'),scope)
    return scope[name]


def test_current_release_reader_checked_and_legacy_not_default(release,tmp_path,monkeypatch):
    monkeypatch.delenv('DPSDK_HOST_SDK_PREFIX',raising=False);monkeypatch.setenv('DPSDK_PROTOCOL_RELEASE',str(release))
    assert module().selected_protocol_release(tmp_path)==(release,'flatbuffers')
    (release/'python/dp_protocol_canonical_record_reader.py').write_text('# tampered')
    with pytest.raises(ValueError,match='摘要'):module().selected_protocol_release(tmp_path)
    legacy=tmp_path/'legacy';legacy.mkdir();(legacy/'manifest.json').write_text('{}')
    monkeypatch.setenv('DPSDK_PROTOCOL_RELEASE',str(legacy))
    with pytest.raises(ValueError,match='当前部署只支持'):module().selected_protocol_release(tmp_path)


def test_mixed_prefix_rejected(tmp_path,monkeypatch):
    selected=tmp_path/'sdk';selected.mkdir();other=tmp_path/'other';other.mkdir()
    monkeypatch.setenv('DPSDK_HOST_SDK_PREFIX',str(selected));monkeypatch.setenv('DPSDK_PROTOCOL_RELEASE',str(other))
    with pytest.raises(ValueError,match='混用'):module().selected_protocol_release(tmp_path)


def test_actual_stdout_parser_preserves_full_source_and_sequence_zero(release,tmp_path):
    from manual_panel_root import state_view
    reader,raw=state_record(release)
    record=dict(format='dummy.accepted-state.dpc1',arrival_us=1,
        source=dict(device='ab'*16,boot_id=2**64-1,context_id=91),canonical_record=raw.hex())
    class Worker:
        def readAllStandardOutput(self):return ('panel_state_json='+json.dumps(record)+'\n').encode()
    class Panel:
        backend='real';stdout_buffer='';worker=Worker();report=tmp_path
        received=[]
        def decode(self,name,raw):return reader.decode_record('dp.protocol.dummy.JointState',raw)
        def accept_state(self,state,arrival):self.received.append((state,arrival))
        def message(self,text):raise AssertionError(text)
    obj=Panel();method('stdout_ready',dict(json=json,state_view=state_view))(obj)
    state,arrival=obj.received[0];assert state['boot_epoch']==2**64-1 and state['state_seq']==0 and arrival==1000
    assert state['source_identity']==(tuple(bytes.fromhex('ab'*16)),2**64-1,91)


def test_actual_accept_state_preserves_masked_null_and_new_context_zero_sequence():
    class Panel:
        retired_state_boots=set();state_boot=None;state_sequence=0;pending_authorization=None;authorized=True
    panel=Panel();accept=method('accept_state',{})
    state=dict(boot_epoch=2**64-1,state_seq=0,position_valid_mask=31,
               position_rad=[1.,2.,3.,4.,5.,None],source_identity=('device',2**64-1,91))
    assert accept(panel,state,1000) and panel.last_state['position_rad'][5]is None
    assert accept(panel,{**state,'source_identity':('device',2**64-1,92)},2000)
    assert not panel.authorized and not accept(panel,state,3000)


def test_root_deploy_runs_generic_validation_and_bundles_join_dependencies(release,tmp_path,monkeypatch):
    value=module();source=SCRIPTS.parents[2];template=SCRIPTS.parent/'config/manual_joint_panel.json'
    monkeypatch.setattr(value,'resources',lambda:(source,template))
    native=tmp_path/'native';native.write_text('#!/bin/sh\nexit 0\n');native.chmod(0o755)
    firmware=tmp_path/'firmware';firmware.write_bytes(b'fixture')
    identity=tmp_path/'identity.json';identity.write_text(json.dumps(dict(contract_sha256='11'*32,model_sha256='22'*32,profile_sha256='33'*32)))
    description=tmp_path/'robot.xacro';description.write_text('<robot name="fixture"/>')
    monkeypatch.delenv('DPSDK_HOST_SDK_PREFIX',raising=False);monkeypatch.setenv('DPSDK_PROTOCOL_RELEASE',str(release));monkeypatch.setenv('XDG_DATA_HOME',str(tmp_path/'data'))
    calls=[];original=value.load_panel_config
    def checked(*args):calls.append(args[1]);return original(*args)
    monkeypatch.setattr(value,'load_panel_config',checked)
    config_path=value.deploy(native,firmware,identity,description)
    assert calls==['real'];cfg=json.loads(config_path.read_text())
    assert cfg['native_state_format']=='accepted-state-dpc1' and cfg['evidence_decoder_supported']
    assert cfg['decoder_capabilities']['network_root_decoder'] is False
    assert 'wire_schema_major'not in cfg and 'wire_schema_minor'not in cfg
    for name in ['current_root_records.py','application_e2e.py','build_real_6dof_context.py']:
        assert (config_path.parent/'ros/scripts/dummy'/name).is_file()
    (config_path.parent/'dummy_joint_cli').write_bytes(b'changed')
    with pytest.raises(ValueError,match='校验'):original(config_path,'real')
