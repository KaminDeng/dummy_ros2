"""ROS离线工具只消费当前受管协议发布。"""
from functools import lru_cache
import importlib.util
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
_TYPES = {'uint8': 'B', 'uint16': 'H', 'uint32': 'I', 'uint64': 'Q', 'float64': 'd', 'bool': 'B'}


@lru_cache(maxsize=1)
def _reader():
    local_release = Path(__file__).resolve().parent / 'protocol'
    release = (local_release if (local_release / 'manifest.json').is_file()
               else os.environ.get('DPSDK_PROTOCOL_RELEASE'))
    if release:
        root = Path(release)
        module_path = root / 'include/dp_protocol_layout_reader.py'
        spec = importlib.util.spec_from_file_location('dpsdk_selected_layout_reader', module_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        # 发布包是只读输入；不用SourceFileLoader，避免产生未登记的pycache。
        exec(compile(module_path.read_bytes(), str(module_path), "exec"), module.__dict__)
        return module, root, False
    source = Path(os.environ.get('DPSDK_PROTOCOL_SOURCE_ROOT', ROOT.parent / 'dpsdk/libraries/dp_protocol'))
    if not (source / 'tools/dp_protocol_core/layout_reader.py').is_file():
        raise ValueError('请设置DPSDK_PROTOCOL_RELEASE或DPSDK_PROTOCOL_SOURCE_ROOT选择中央协议')
    sys.path.insert(0, str(source / 'tools'))
    from dp_protocol_core import layout_reader
    return layout_reader, source, True


def _interface(name):
    if name in ('DummyManageV2Request', 'DummyManageV2Response'):
        return 'dpsdk_interfaces/srv/DummyManageV2', 'request' if name.endswith('Request') else 'response'
    return 'dpsdk_interfaces/msg/' + name, 'message'


@lru_cache(maxsize=None)
def current_layout(name):
    """进程启动时固定所选版本；逐页只decode，不重复扫描源码摘要。"""
    module, root, source = _reader()
    fqn, section = _interface(name)
    if source:
        return module.load_source_raw_fixed_layout(root, fqn, section=section, selection={
            'protocol_set': 'dummy', 'profile': 'profile_full', 'codec': 'raw-fixed'})
    return module.load_raw_fixed_layout(root, fqn, section=section)


def _schema(raw):
    # 共同版本前缀的offset/width也来自中央所选字段，不在工具保存<HH字段表。
    layout = current_layout('DummyDeviceStatusV2')
    values = []
    for name in ('schema_major', 'schema_minor'):
        field = layout.field(name)
        if len(raw) < field.offset + field.size:
            raise ValueError('invalid wire schema prefix')
        values.append(int.from_bytes(raw[field.offset:field.offset + field.size], 'little'))
    return tuple(values)


def decode(name, raw):
    schema = _schema(raw)
    layout = current_layout(name)
    if schema != (layout.constant('SCHEMA_MAJOR'), layout.constant('SCHEMA_MINOR')):
        raise ValueError(f'invalid {name} schema')
    result = layout.decode(raw)
    return {name: value.hex() if isinstance(value, bytes) else list(value) if isinstance(value, tuple) else value
            for name, value in result.items()}


def evidence_parts(raw):
    """返回独立SHA算法所需的规范字段片段；不在本层替代验链或来源准入。"""
    name = 'DummyAttemptEvidenceV2'
    fields = decode(name, raw)
    layout = current_layout(name)
    prefix_offset = layout.field('previous_chain_sha256').offset
    lost = layout.slice(raw, 'evidence_lost')
    return {'prefix': raw[:prefix_offset], 'previous': bytes.fromhex(fields['previous_chain_sha256']),
            'chain': bytes.fromhex(fields['chain_sha256']), 'lost': lost, 'fields': fields}


# 通用metadata消费者读取公共对象的惰性只读视图，不保存字段副本。
from collections.abc import Mapping
from types import MappingProxyType


class _CurrentPayloads(Mapping):
    _names = ('DummyDeviceStatusV2', 'DummyJointCommandV2', 'DummyJointStateV2',
              'DummyAttemptEvidenceV2', 'DummyManageV2Request', 'DummyManageV2Response')

    def __getitem__(self, name):
        layout = current_layout(name)
        fields = tuple(MappingProxyType({'name': field.name, 'type': field.type,
                                        'count': field.count, 'offset': field.offset, 'size': field.size})
                       for field in layout.fields.values())
        return MappingProxyType({'encoded_size': layout.size, 'interface_id': layout.interface_id,
                                 'fields': fields})

    def __iter__(self):
        return iter(self._names)

    def __len__(self):
        return len(self._names)


LAYOUT = MappingProxyType({'payloads': _CurrentPayloads()})
TYPES = MappingProxyType(_TYPES)
