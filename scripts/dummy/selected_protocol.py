"""当前中央 Root 发布的 DPC1 记录接口；不把网络Root或历史raw-fixed当记录读。"""
from collections.abc import Mapping
from functools import lru_cache
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from types import MappingProxyType

ROOT = Path(__file__).resolve().parents[2]


@lru_cache(maxsize=1)
def _reader():
    local = Path(__file__).resolve().parent / 'protocol'
    selected = (local if (local / 'release_manifest.json').is_file() else
                os.environ.get('DPSDK_PROTOCOL_RELEASE') or os.environ.get('DPSDK_PROTOCOL_GENERATED_ROOT'))
    if not selected:
        raise ValueError('请指定当前Root发布DPSDK_PROTOCOL_RELEASE或实际生成目录DPSDK_PROTOCOL_GENERATED_ROOT；历史DPRAW01须显式历史reader')
    root = Path(selected).resolve()
    release = (root / 'release_manifest.json').is_file()
    if release:
        path = root / 'python/dp_protocol_canonical_record_reader.py'
        manifest = json.loads((root / 'release_manifest.json').read_text())
        if manifest['files'].get(path.relative_to(root).as_posix()) != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError('当前发布canonical reader SHA不符')
    else:
        source = Path(os.environ.get('DPSDK_PROTOCOL_SOURCE_ROOT', ROOT.parent/'dpsdk/libraries/dp_protocol'))
        path = source / 'tools/dp_protocol_core/canonical_record_reader.py'
    name = '_selected_canonical_' + hashlib.sha256(str(path).encode()).hexdigest()[:16]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    # 发布目录只读；不写pycache，不执行隐式生成或改所选协议。
    exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
    return module.SchemaCodec.from_directory(root)


def _type(name):
    codec = _reader()
    if name in codec.descriptors:
        return name
    for fqn in codec.descriptors:
        parts = fqn.split('.')
        alias = ''.join(part.title() for part in parts[2:-1]) + parts[-1]
        if name == alias:
            return fqn
    raise ValueError('当前Root选择没有类型: ' + name)


def _json_value(value):
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, dict):
        return {key: _json_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_value(item) for item in value]
    return value


def decode(name, raw):
    """返回真实嵌套字段；None为缺失，不增加boot/flat joints等旧DTO别名。"""
    return _json_value(_reader().decode_record(_type(name), raw))


def encode(name, fields):
    """规范记录fixture/离线工具编码；输入字节字段须提供bytes，非网络发送API。"""
    return _reader().encode_record(_type(name), fields)


def evidence_parts(raw):
    """SHA输入严格为previous32+完整中央EvidenceHash目的记录，不置零链字段。"""
    name = _type('DummyAttemptEvidence')
    fields = _reader().decode_record(name, raw)
    previous, chain, lost = fields['previous_chain_sha256'], fields['chain_sha256'], fields['evidence_lost']
    if previous is None or chain is None or len(previous) != 32 or len(chain) != 32 or lost is None:
        raise ValueError('证据链字段缺失或长度非法')
    return {'canonical': _reader().canonical_hash_record(name, raw), 'previous': previous,
            'chain': chain, 'lost': lost, 'fields':_json_value(fields)}


class _RecordLayout:
    """只读真实schema元数据；变长DPC1没有固定offset或encoded_size。"""
    def __init__(self, name):
        self.name = name; self.fqn = _type(name)
        self.interface_id = _reader().descriptors[self.fqn]['type_id']
        self.fields = MappingProxyType({field['name']: MappingProxyType(field)
                                       for field in _reader().fields(self.fqn)})
    def decode(self, raw):
        return _reader().decode_record(self.fqn, raw)
    def encode(self, values):
        return _reader().encode_record(self.fqn, values)


@lru_cache(maxsize=None)
def current_layout(name):
    return _RecordLayout(name)


class _CurrentPayloads(Mapping):
    def __getitem__(self, name):
        layout = current_layout(name)
        return MappingProxyType({'type_id': layout.interface_id, 'format': 'DPC1',
                                 'fields': tuple(layout.fields.values())})
    def __iter__(self):
        return iter(''.join(part.title() for part in fqn.split('.')[2:-1]) + fqn.split('.')[-1]
                    for fqn in _reader().descriptors)
    def __len__(self):
        return len(_reader().descriptors)


LAYOUT = MappingProxyType({'payloads': _CurrentPayloads(), 'format': 'DPC1'})
