"""活动ROS证据字段从公共所选协议读取；其他未使用版本明确拒绝。"""
import importlib.util
from pathlib import Path
import sys
import pytest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))


def test_current_state_uses_public_layout_and_rejects_other_versions():
    import selected_protocol
    layout = selected_protocol.current_layout('DummyJointStateV2')
    raw = layout.encode({'schema_major': 2, 'schema_minor': 5, 'state_seq': 123})
    assert selected_protocol.decode('DummyJointStateV2', raw)['state_seq'] == 123
    with pytest.raises(ValueError):
        selected_protocol.decode('DummyJointStateV2', raw[:-1])
    invalid = bytearray(raw)
    invalid[layout.field('schema_minor').offset] = 6
    with pytest.raises(ValueError, match='schema'):
        selected_protocol.decode('DummyJointStateV2', invalid)


def test_current_evidence_chain_parts_follow_selected_fields():
    import selected_protocol
    layout = selected_protocol.current_layout('DummyAttemptEvidenceV2')
    raw = layout.encode({'schema_major': 2, 'schema_minor': 5,
        'previous_chain_sha256': bytes([1]) * 32, 'chain_sha256': bytes([2]) * 32})
    parts = selected_protocol.evidence_parts(raw)
    assert parts['prefix'] == raw[:layout.field('previous_chain_sha256').offset]
    assert parts['previous'] == bytes([1]) * 32
    assert parts['chain'] == bytes([2]) * 32
    assert parts['lost'] == b'\0'


def test_metadata_api_is_read_only_view_of_current_public_layout():
    from scripts.dummy import join_real_6dof_evidence as codec
    import selected_protocol
    public = selected_protocol.current_layout('DummyJointStateV2')
    metadata = codec.LAYOUT['payloads']['DummyJointStateV2']
    assert metadata['encoded_size'] == public.size
    assert metadata['interface_id'] == public.interface_id
    for field in metadata['fields']:
        actual = public.field(field['name'])
        assert (field['offset'], field['size'], field['type'], field['count']) == (
            actual.offset, actual.size, actual.type, actual.count)
    with pytest.raises(TypeError):
        metadata['fields'][0]['offset'] = 999
    assert codec.TYPES['uint16'] == 'H'
    assert public.constant('SCHEMA_MINOR') == 5
    assert not hasattr(codec, 'LEGACY_LAYOUT')
