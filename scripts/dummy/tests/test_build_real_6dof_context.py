"""离线生成真实六轴 ROS 证据上下文的契约测试。"""

import hashlib
import json
from pathlib import Path
import struct

import pytest

from scripts.dummy import build_real_6dof_context as builder
from scripts.dummy import join_real_6dof_evidence as joiner


def wire(name, **values):
    layout = joiner.LAYOUT["payloads"][name]
    raw = bytearray(layout["encoded_size"])
    types = {"uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q",
             "float64": "d", "bool": "B"}
    for field in layout["fields"]:
        value = {"schema_major": 2, "schema_minor": 4, **values}.get(field["name"])
        if value is None:
            continue
        if field["count"] > 1:
            value = bytes.fromhex(value) if isinstance(value, str) else value
        else:
            value = [value]
        struct.pack_into("<" + types[field["type"]] * field["count"], raw,
                         field["offset"], *value)
    return bytes(raw)


def record(kind, when, boot, session, run, seq, payload):
    return joiner.RECORD_HEADER.pack(kind, len(payload), when, boot, session,
                                     run, seq) + payload


@pytest.fixture
def capture(tmp_path):
    timeline = tmp_path / "timeline.csv"
    timeline.write_text("event,steady_us\nCOMPLETE,1\n")
    action = tmp_path / "action.json"
    action.write_text('{"scope":"ROS_ACTION_ONLY"}\n')
    client = "11" * 16
    request = wire("DummyManageV2Request", client_instance_id=client,
                   expected_boot_epoch=2, expected_session_id=3, request_seq=1)
    response = wire("DummyManageV2Response", client_instance_id=client,
                    boot_epoch=2, session_id=3)
    states = [wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                   produced_at_us=produced) for produced in (1090, 1180)]
    raw = tmp_path / "raw.bin"
    raw.write_bytes(b"DPRAW01\0" +
                    record(4, 95, 2, 3, 0, 1, request) +
                    record(2, 100, 2, 3, 0, 0, states[0]) +
                    record(5, 150, 2, 3, 0, 0, response) +
                    record(2, 200, 2, 3, 0, 0, states[1]))
    return timeline, raw, action, tmp_path / "context.json"


def build(capture):
    timeline, raw, action, output = capture
    return builder.build_context(
        timeline, raw, action, output,
        path_tolerance_rad=[0.01] * 6, goal_tolerance_rad=[0.003] * 6,
        producer_lag_budget_us=500000, feedback_age_budget_us=10000,
        ros_feedback_lag_budget_us=2000, gate_state_max_age_us=500000)


def test_builds_context_from_parsed_records_and_pins_inputs(capture):
    context = build(capture)
    timeline, raw, action, output = capture
    assert context["client_id"] == "11" * 16
    assert context["producer_lag_us"] == 10
    assert context["producer_lag_method"] == "state_receive_minus_produced_offset_span"
    assert context["producer_lag_budget_us"] == 500000
    assert context["path_tolerance_rad"] == [0.01] * 6
    for name, path in (("timeline", timeline), ("raw", raw), ("action", action)):
        assert context[f"{name}_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert json.loads(output.read_text()) == context


def test_rejects_mismatched_client_without_output(capture):
    raw = capture[1]
    source = raw.read_bytes()
    raw.write_bytes(source.replace(bytes.fromhex("11" * 16),
                                   bytes.fromhex("22" * 16), 1))
    with pytest.raises(ValueError, match="client"):
        build(capture)
    assert not capture[3].exists()


def test_rejects_empty_state_stream(capture):
    raw = capture[1]
    records = joiner._raw_records(raw.read_bytes())
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], item["raw"])
        for item in records if item["kind"] != 2))
    with pytest.raises(ValueError, match="State"):
        build(capture)
    assert not capture[3].exists()


def test_rejects_nonmonotonic_receive_times(capture):
    raw = capture[1]
    records = joiner._raw_records(raw.read_bytes())
    reordered = [records[0], records[2], records[1], records[3]]
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], item["raw"]) for item in reordered))
    with pytest.raises(ValueError, match="nonmonotonic"):
        build(capture)
    assert not capture[3].exists()


def test_rejects_nonmonotonic_state_production_times(capture):
    raw = capture[1]
    records = joiner._raw_records(raw.read_bytes())
    earlier = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                   produced_at_us=1080)
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], earlier if index == 3 else item["raw"])
        for index, item in enumerate(records)))
    with pytest.raises(ValueError, match="nonmonotonic DPRAW01 State"):
        build(capture)
    assert not capture[3].exists()


def test_rejects_budget_below_observed_lag(capture):
    timeline, raw, action, output = capture
    with pytest.raises(ValueError, match="producer lag budget"):
        builder.build_context(
            timeline, raw, action, output,
            path_tolerance_rad=[0.01] * 6, goal_tolerance_rad=[0.003] * 6,
            producer_lag_budget_us=9, feedback_age_budget_us=10000,
            ros_feedback_lag_budget_us=2000, gate_state_max_age_us=500000)
    assert not output.exists()


def test_first_high_offset_still_counts_full_observed_span(capture):
    raw = capture[1]
    records = joiner._raw_records(raw.read_bytes())
    states = [(20000, 10000), (30000, 25000), (40000, 31000)]
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], item["raw"])
        for item in records if item["kind"] in (4, 5)) + b"".join(
        record(2, when, 2, 3, 0, 0, wire("DummyJointStateV2",
            boot_epoch=2, session_id=3, produced_at_us=produced))
        for when, produced in states))
    context = build(capture)
    assert context["producer_lag_us"] == 5000
    assert context["producer_lag_method"] == "state_receive_minus_produced_offset_span"
    timeline, raw, action, output = capture
    with pytest.raises(ValueError, match="producer lag budget"):
        builder.build_context(
            timeline, raw, action, output.with_name("tight-context.json"),
            path_tolerance_rad=[0.01] * 6, goal_tolerance_rad=[0.003] * 6,
            producer_lag_budget_us=4999, feedback_age_budget_us=10000,
            ros_feedback_lag_budget_us=2000, gate_state_max_age_us=500000)


def test_existing_output_is_preserved(capture):
    output = capture[3]
    output.write_text('{"existing":true}\n')
    with pytest.raises(FileExistsError):
        build(capture)
    assert output.read_text() == '{"existing":true}\n'


def test_cli_writes_explicit_budgets(capture, capsys):
    timeline, raw, action, output = capture
    assert builder.main([
        "--timeline", str(timeline), "--raw", str(raw),
        "--action", str(action), "--output", str(output),
        "--path-tolerance-rad", "0.01,0.01,0.01,0.01,0.01,0.01",
        "--goal-tolerance-rad", "0.003,0.003,0.003,0.003,0.003,0.003",
        "--producer-lag-budget-us", "500000",
        "--feedback-age-budget-us", "10000",
        "--ros-feedback-lag-budget-us", "2000",
        "--gate-state-max-age-us", "500000",
    ]) == 0
    cli_result = json.loads(capsys.readouterr().out)
    assert cli_result["producer_lag_us"] == 10
    assert cli_result["producer_lag_method"] == "state_receive_minus_produced_offset_span"
    assert json.loads(output.read_text())["gate_state_max_age_us"] == 500000
