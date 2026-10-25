"""活动ROS证据字段从公共所选协议读取；历史2.4明确走冻结布局。"""
import importlib.util
from pathlib import Path
import sys
import pytest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))


def test_current_state_uses_public_layout_and_preserves_old_version_boundary():
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
