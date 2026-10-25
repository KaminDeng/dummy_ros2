"""ROS离线工具的公共协议选择：活动版本消费受管发布，2.4仅读冻结兼容合同。"""
from functools import lru_cache
import importlib.util
import json
import os
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
LEGACY_LAYOUT = json.loads((ROOT / 'compatibility/wire2_4/wire_layout.json').read_text())
_TYPES = {'uint8': 'B', 'uint16': 'H', 'uint32': 'I', 'uint64': 'Q', 'float64': 'd', 'bool': 'B'}


@lru_cache(maxsize=1)
def _reader():
    release = os.environ.get('DPSDK_PROTOCOL_RELEASE')
    if release:
        root = Path(release)
        module_path = root / 'include/dp_protocol_layout_reader.py'
        spec = importlib.util.spec_from_file_location('dpsdk_selected_layout_reader', module_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
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
    if len(raw) < 4:
        raise ValueError('invalid wire schema prefix')
    return struct.unpack_from('<HH', raw)


def decode(name, raw):
    schema = _schema(raw)
    if schema == (2, 4):
        # 历史分支只消费已冻结的2.4字段表，不参与活动生成或替换当前来源。
        layout = LEGACY_LAYOUT['payloads'][name]
        if len(raw) != layout['encoded_size']:
            raise ValueError(f'invalid historical {name} wire size')
        result = {}
        for field in layout['fields']:
            values = struct.unpack_from('<' + _TYPES[field['type']] * field['count'], raw, field['offset'])
            result[field['name']] = (bytes(values).hex() if field['count'] > 1 and field['type'] == 'uint8'
                                     else list(values) if field['count'] > 1 else values[0])
        return result
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
    if _schema(raw) == (2, 4):
        layout = LEGACY_LAYOUT['payloads'][name]
        offsets = {field['name']: field['offset'] for field in layout['fields']}
        prefix_offset = offsets['previous_chain_sha256']
        lost = raw[offsets['evidence_lost']:offsets['evidence_lost'] + 1]
    else:
        layout = current_layout(name)
        prefix_offset = layout.field('previous_chain_sha256').offset
        lost = layout.slice(raw, 'evidence_lost')
    return {'prefix': raw[:prefix_offset], 'previous': bytes.fromhex(fields['previous_chain_sha256']),
            'chain': bytes.fromhex(fields['chain_sha256']), 'lost': lost, 'fields': fields}
