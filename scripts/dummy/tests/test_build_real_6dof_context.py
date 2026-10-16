"""离线生成真实六轴 ROS 证据上下文的契约测试。"""

import csv
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


def window_capture(tmp_path, offsets=(400000, 0, 100, 200, 300, 400, 400000),
                   *, release=True, idle=True, other_boot=False, goal=True,
                   queried_reply=False, other_device=False, other_epoch=False,
                   second_goal=False, early_second_epoch=False):
    """Small raw/timeline pair with State samples before, during, and after motion."""
    timeline = tmp_path / "timeline.csv"
    header = ("event,goal_uuid,trajectory_sha256,steady_us,boot,session,"
              "reference_epoch,motion_epoch,run_id,axis_mask,status_core_state,"
              "state_core_state,release_request_id,release_operation_id")
    def row(event, when, session=3, run=7, request=0, operation=0, motion=1):
        reference = 0 if session == 0 else 1
        return f"{event},{'a' * 32},{'b' * 64},{when},2,{session},{reference},{motion},{run},63,4,4,{request},{operation}"
    first_run = 6 if second_goal else 7
    rows = [row("accepted", 1000000, run=first_run),
            row("before_begin", 1000000, run=first_run),
            row("after_begin", 1100000, run=first_run),
            row("feedback", 1200000, run=first_run),
            row("terminal_success", 2500000, run=first_run),
            row("pre_release_hold", 2600000, run=first_run)] if goal else []
    if second_goal and goal:
        rows.extend((row("accepted", 2600000, motion=2),
                     row("before_begin", 2600000, motion=2),
                     row("after_begin", 2650000, motion=2),
                     row("feedback", 2700000, motion=2),
                     row("terminal_success", 3000000, motion=2),
                     row("pre_release_hold", 3100000, motion=2)))
    if goal:
        rows.append(row("post_release_status", 4000000, 0, 8, 9, 10,
                        motion=3 if second_goal else 1))
    timeline.write_text("\n".join([header, *rows,
                                   f"COMPLETE,{2 if second_goal else 1}", ""]))
    action = tmp_path / "action.json"
    action.write_text('{"scope":"ROS_ACTION_ONLY"}\n')
    client = "11" * 16
    request = wire("DummyManageV2Request", client_instance_id=client,
                   device_id="22" * 16,
                   expected_boot_epoch=2, expected_session_id=3,
                   request_seq=1)
    release_request = wire("DummyManageV2Request", client_instance_id=client,
                           device_id="22" * 16,
                           expected_boot_epoch=2, expected_session_id=3,
                           request_seq=2, request_id=9, operation=10)
    response = wire("DummyManageV2Response", client_instance_id=client,
                    device_id="22" * 16,
                    boot_epoch=2, session_id=0,
                    request_id=11 if queried_reply else 9, operation_id=10,
                    run_id=8, transaction_phase=3, result_code=0,
                    release_committed=1)
    query_request = wire("DummyManageV2Request", client_instance_id=client,
                         expected_boot_epoch=2, expected_session_id=3,
                         request_seq=3, request_id=11, operation=12,
                         queried_operation_id=9)
    terminal = wire("DummyAttemptEvidenceV2", run_id=8, attempt_id=8,
                    category=4, final=1, final_state=8, release_committed=1)
    status = wire("DummyDeviceStatusV2", boot_epoch=2, session_id=0,
                  device_id="22" * 16, reference_epoch=0,
                  motion_epoch=3 if second_goal else 1,
                  run_id=8, core_state=1, release_committed=1, operation_id=10,
                  final_state=8, attempt_kind=3)
    events = [(4, 500000, 2, 3, 0, 1, request)]
    for index, (when, offset) in enumerate(zip((700000, 1300000, 1800000,
                                                 2700000, 3300000, 3900000,
                                                 4500000), offsets)):
        session = 0 if when >= 3800000 else 3
        state = wire("DummyJointStateV2", boot_epoch=3 if other_boot and index == 3 else 2,
                     session_id=session, produced_at_us=when + 100000 - offset,
                     device_id=("33" if other_device and index == 3 else "22") * 16,
                     reference_epoch=0 if session == 0 else 1,
                     motion_epoch=(99 if other_epoch and index == 3 else
                                   3 if second_goal and session == 0 else
                                   2 if early_second_epoch and index in (2, 3) else
                                   2 if second_goal and index == 4 else 1),
                     core_state=1 if session == 0 else 4)
        events.append((2, when, 3 if other_boot and index == 3 else 2,
                       session, 0, 0, state))
    if release:
        events.extend([(4, 3150000 if second_goal else 2900000,
                        2, 3, 0, 2, release_request),
                       (1, 3400000, 2, 3, 8, 0, terminal),
                       (5, 3500000, 2, 0, 8, 0, response)])
        if queried_reply:
            events.append((4, 3450000, 2, 3, 0, 3, query_request))
    if idle:
        events.append((3, 3800000, 2, 0, 8, 0, status))
    raw = tmp_path / "raw.bin"
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        record(*event) for event in sorted(events, key=lambda event: event[1])))
    return timeline, raw, action, tmp_path / "context.json"


def build_window(capture):
    return builder.build_context(
        *capture, path_tolerance_rad=[0.01] * 6,
        goal_tolerance_rad=[0.003] * 6, producer_lag_budget_us=250000,
        feedback_age_budget_us=10000, ros_feedback_lag_budget_us=2000,
        gate_state_max_age_us=1000000, producer_lag_scope="motion_window_v2")


def canceled_window_capture(tmp_path, *, early_stop_epoch=False, stop_page=True,
                            ambiguous_stop_epoch=False, ambiguous_only=False,
                            stale_after_page=False):
    capture = window_capture(tmp_path)
    timeline, raw, _, _ = capture
    text = timeline.read_text()
    text = text.replace("release_request_id,release_operation_id",
                        "release_request_id,release_operation_id,cancel_request_id,cancel_operation_id")
    rows = text.splitlines()
    for index in range(1, len(rows) - 1):
        if rows[index].startswith("terminal_success,"):
            rows[index] = rows[index].replace("terminal_success,", "terminal_canceled,", 1)
            columns = rows[index].split(",")
            columns[7] = "2"
            rows[index] = ",".join(columns) + ",7,7"
        elif rows[index].startswith("pre_release_hold,"):
            columns = rows[index].split(",")
            columns[7] = "2"
            rows[index] = ",".join(columns) + ",0,0"
        elif rows[index].startswith("post_release_status,"):
            columns = rows[index].split(",")
            columns[7] = "3"
            rows[index] = ",".join(columns) + ",0,0"
        else:
            rows[index] += ",0,0"
    timeline.write_text("\n".join(rows) + "\n")
    records = joiner._raw_records(raw.read_bytes())
    rewritten = []
    for item in records:
        payload = item["raw"]
        if item["kind"] == 2:
            fields = item["fields"]
            stop_epoch = item["when"] >= 2100000 or (
                early_stop_epoch and item["when"] == 1800000)
            if item["session"] == 3 and stop_epoch:
                payload = wire("DummyJointStateV2", **{
                    **fields, "motion_epoch": 2})
            elif item["session"] == 0:
                payload = wire("DummyJointStateV2", **{
                    **fields, "motion_epoch": 3})
        elif item["kind"] == 3:
            payload = wire("DummyDeviceStatusV2", **{
                **item["fields"], "motion_epoch": 3})
        rewritten.append(record(item["kind"], item["when"], item["boot"],
                                item["session"], item["run"], item["seq"], payload))
    cancel = wire("DummyManageV2Request", client_instance_id="11" * 16,
                  device_id="22" * 16, expected_boot_epoch=2,
                  expected_session_id=3, expected_motion_epoch=1,
                  request_seq=7, request_id=7, operation=9)
    rewritten.append(record(4, 2000000, 2, 3, 0, 7, cancel))
    receipt = wire("DummyManageV2Response", client_instance_id="11" * 16,
                   device_id="22" * 16, request_id=7, operation_id=7,
                   boot_epoch=2, session_id=3, run_id=7, motion_epoch=2,
                   receipt=1, transaction_phase=3, result_code=0)
    rewritten.append(record(5, 2200000, 2, 3, 7, 0, receipt))
    if stop_page:
        stop = wire("DummyAttemptEvidenceV2", run_id=7, attempt_id=7,
                    attempt_kind=1, category=2, motion_epoch=2,
                    evidence_seq=1)
        rewritten.append(record(1, 2100000, 2, 3, 7, 1, stop))
    if ambiguous_stop_epoch or ambiguous_only:
        state = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                     device_id="22" * 16, reference_epoch=1,
                     motion_epoch=2, core_state=4, produced_at_us=2149900)
        rewritten.append(record(2, 2050000, 2, 3, 0, 0, state))
    if stale_after_page:
        state = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                     device_id="22" * 16, reference_epoch=1,
                     motion_epoch=1, core_state=4, produced_at_us=2449900)
        rewritten.append(record(2, 2350000, 2, 3, 0, 0, state))
    if ambiguous_only:
        rewritten = [item for item in rewritten if not (
            joiner.RECORD_HEADER.unpack_from(item)[0] == 2 and
            joiner.RECORD_HEADER.unpack_from(item)[2] == 1800000)]
    raw.write_bytes(b"DPRAW01\0" + b"".join(sorted(
        rewritten, key=lambda raw_record: joiner.RECORD_HEADER.unpack_from(raw_record)[2])))
    return capture


def test_canceled_window_accepts_stop_epoch_after_bound_stop_page(tmp_path):
    context = build_window(canceled_window_capture(tmp_path))
    assert context["producer_lag_us"] == 400


def test_canceled_window_allows_stop_state_between_request_and_page(tmp_path):
    context = build_window(canceled_window_capture(
        tmp_path, ambiguous_stop_epoch=True))
    assert context["producer_lag_gate_version"] == 2


def test_canceled_window_rejects_stop_epoch_before_bound_stop_page(tmp_path):
    capture = canceled_window_capture(tmp_path, early_stop_epoch=True)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)
    assert not capture[3].exists()


def test_canceled_window_cannot_fill_coverage_with_ambiguous_state(tmp_path):
    capture = canceled_window_capture(tmp_path, ambiguous_only=True)
    with pytest.raises(ValueError, match="State coverage"):
        build_window(capture)
    assert not capture[3].exists()


def test_canceled_window_rejects_old_epoch_after_stop_page(tmp_path):
    capture = canceled_window_capture(tmp_path, stale_after_page=True)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)
    assert not capture[3].exists()


def test_canceled_window_requires_bound_stop_page(tmp_path):
    capture = canceled_window_capture(tmp_path, stop_page=False)
    with pytest.raises(ValueError, match="Stop CAN page"):
        build_window(capture)
    assert not capture[3].exists()


def test_window_gate_excludes_pre_goal_and_post_release_peaks(tmp_path):
    context = build_window(window_capture(tmp_path))
    assert context["producer_lag_us"] == 400
    assert context["producer_lag_environment_us"] == 400000
    assert context["producer_lag_gate_version"] == 2
    assert context["producer_lag_method"] == "state_receive_minus_produced_offset_span_motion_window"


def test_window_gate_accepts_query_linked_release_receipt(tmp_path):
    assert build_window(window_capture(tmp_path, queried_reply=True))["producer_lag_us"] == 400


@pytest.mark.parametrize("queried_reply", [False, True])
def test_window_gate_accepts_release_receipt_before_terminal_page(tmp_path, queried_reply):
    capture = window_capture(tmp_path, queried_reply=queried_reply)
    records = joiner._raw_records(capture[1].read_bytes())
    reordered = [record(
        item["kind"], 3550000 if item["kind"] == 1 else
        3500000 if item["kind"] == 5 and item["fields"].get("release_committed") else
        item["when"], item["boot"], item["session"], item["run"], item["seq"], item["raw"])
        for item in records]
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(sorted(
        reordered, key=lambda raw_record: joiner.RECORD_HEADER.unpack_from(raw_record)[2])))
    assert build_window(capture)["producer_lag_us"] == 400


def test_window_gate_rejects_release_receipt_before_request(tmp_path):
    capture = window_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    reordered = [record(
        item["kind"], 2800000 if item["kind"] == 5 and
        item["fields"].get("release_committed") else item["when"],
        item["boot"], item["session"], item["run"], item["seq"], item["raw"])
        for item in records]
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(sorted(
        reordered, key=lambda raw_record: joiner.RECORD_HEADER.unpack_from(raw_record)[2])))
    with pytest.raises(ValueError, match="Release proof"):
        build_window(capture)


def test_window_gate_rejects_uncommitted_release_terminal(tmp_path):
    capture = window_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"],
        wire("DummyAttemptEvidenceV2", run_id=8, attempt_id=8,
             category=4, final=1, final_state=8, release_committed=0)
        if item["kind"] == 1 else item["raw"]) for item in records))
    with pytest.raises(ValueError, match="Release proof"):
        build_window(capture)


def test_window_gate_rejects_sparse_state_coverage(tmp_path):
    capture = window_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], item["raw"]) for item in records
        if item["kind"] != 2 or item["when"] in (700000, 1300000,
                                                   3900000, 4500000)))
    with pytest.raises(ValueError, match="State coverage"):
        build_window(capture)


@pytest.mark.parametrize("options", [{"other_device": True}, {"other_epoch": True}])
def test_window_gate_rejects_cross_identity_coverage_sample(tmp_path, options):
    capture = window_capture(tmp_path, **options)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)


def test_window_gate_rejects_prior_goal_phase_after_next_goal_acceptance(tmp_path):
    capture = window_capture(tmp_path, second_goal=True)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)


def test_window_gate_rejects_next_goal_phase_before_acceptance(tmp_path):
    capture = window_capture(tmp_path, second_goal=True, early_second_epoch=True)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)


@pytest.mark.parametrize("offsets", [
    (0, 0, 300000, 100, 200, 300, 0),
    (0, 0, 100, 300000, 200, 300, 0),
    (0, 0, 100, 200, 300000, 400, 0),
])
def test_window_gate_rejects_peak_during_motion_hold_or_stop(tmp_path, offsets):
    with pytest.raises(ValueError, match="producer lag budget"):
        build_window(window_capture(tmp_path, offsets))


@pytest.mark.parametrize("options,match", [
    ({"release": False}, "Release"),
    ({"idle": False}, "Idle"),
    ({"other_boot": True}, "identity"),
    ({"goal": False}, "accepted"),
])
def test_window_gate_requires_raw_proof_and_identity(tmp_path, options, match):
    capture = window_capture(tmp_path, **options)
    with pytest.raises(ValueError, match=match):
        build_window(capture)
    assert not capture[3].exists()


def begun_cancel_capture(tmp_path):
    """复现真实accepted103→after_begin104→cancel SafetyHold105角色。"""
    capture = canceled_window_capture(tmp_path)
    timeline, raw, _, _ = capture
    rows = list(csv.DictReader(timeline.read_text().splitlines()[:-1]))
    for row in rows:
        epoch = int(row["motion_epoch"])
        row["motion_epoch"] = str(103 if row["event"] in ("accepted", "before_begin")
                                  else 104 if epoch == 1 else 105 if epoch == 2 else 106)
    with timeline.open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows); stream.write("COMPLETE,1\n")
    rewritten = []
    for item in joiner._raw_records(raw.read_bytes()):
        fields = dict(item["fields"])
        for field in ("motion_epoch", "expected_motion_epoch"):
            if field in fields:
                fields[field] = {0: 0, 1: 104, 2: 105, 3: 106}[fields[field]]
        name = {1: "DummyAttemptEvidenceV2", 2: "DummyJointStateV2",
                3: "DummyDeviceStatusV2", 4: "DummyManageV2Request",
                5: "DummyManageV2Response"}[item["kind"]]
        rewritten.append(record(item["kind"], item["when"], item["boot"],
                                item["session"], item["run"], item["seq"], wire(name, **fields)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(rewritten))
    return capture


def test_cancel_window_binds_active_after_begin_epoch_not_accepted_epoch(tmp_path):
    assert build_window(begun_cancel_capture(tmp_path))["producer_lag_us"] == 400


@pytest.mark.parametrize("mutation", ["missing", "duplicate", "boot", "session", "run",
                                      "reference", "uuid", "trajectory", "late", "old_request"])
def test_cancel_window_rejects_ambiguous_or_unbound_after_begin(tmp_path, mutation):
    capture = begun_cancel_capture(tmp_path)
    timeline, raw, _, _ = capture
    lines = timeline.read_text().splitlines()
    index = next(i for i, line in enumerate(lines) if line.startswith("after_begin,"))
    fields = lines[index].split(",")
    if mutation == "missing": lines.pop(index)
    elif mutation == "duplicate": lines.insert(index, lines[index])
    elif mutation == "late": fields[3] = "2600000"; lines[index] = ",".join(fields)
    elif mutation != "old_request":
        column = {"boot": 4, "session": 5, "run": 8, "reference": 6,
                  "uuid": 1, "trajectory": 2}[mutation]
        fields[column] = "999"; lines[index] = ",".join(fields)
    if mutation == "old_request":
        rewritten = []
        for item in joiner._raw_records(raw.read_bytes()):
            payload = item["raw"]
            if item["kind"] == 4 and item["fields"]["operation"] == 9:
                payload = wire("DummyManageV2Request", **{
                    **item["fields"], "expected_motion_epoch": 103})
            rewritten.append(record(item["kind"], item["when"], item["boot"],
                                    item["session"], item["run"], item["seq"], payload))
        raw.write_bytes(b"DPRAW01\0" + b"".join(rewritten))
    timeline.write_text("\n".join(lines) + "\n")
    with pytest.raises(ValueError): build_window(capture)


@pytest.mark.parametrize("field,value", [("operation_id", 6), ("motion_epoch", 104),
                                         ("request_id", 6)])
def test_cancel_window_keeps_exact_terminal_receipt_binding(tmp_path, field, value):
    capture = begun_cancel_capture(tmp_path)
    raw = capture[1]
    rewritten = []
    for item in joiner._raw_records(raw.read_bytes()):
        payload = item["raw"]
        if item["kind"] == 5 and item["fields"]["operation_id"] == 7:
            payload = wire("DummyManageV2Response", **{**item["fields"], field: value})
        rewritten.append(record(item["kind"], item["when"], item["boot"],
                                item["session"], item["run"], item["seq"], payload))
    raw.write_bytes(b"DPRAW01\0" + b"".join(rewritten))
    with pytest.raises(ValueError, match="Stop CAN page"):
        build_window(capture)


def _rewrite_records(capture, transform):
    rewritten = []
    for item in joiner._raw_records(capture[1].read_bytes()):
        rewritten.append(transform(item))
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(sorted(
        rewritten, key=lambda value: joiner.RECORD_HEADER.unpack_from(value)[2])))


def test_release_idle_channels_can_arrive_before_terminal_evidence(tmp_path):
    capture = window_capture(tmp_path)
    # 已确认本次op10，Idle Status/State先到，终页异步稍后到达。
    _rewrite_records(capture, lambda item: record(item["kind"],
        3950000 if item["kind"] == 1 and item["fields"].get("release_committed") else item["when"],
        item["boot"], item["session"], item["run"], item["seq"], item["raw"]))
    assert build_window(capture)["producer_lag_us"] == 400


def test_cancel_active_state_can_arrive_before_after_begin_timeline(tmp_path):
    capture = begun_cancel_capture(tmp_path)
    lines = capture[0].read_text().splitlines()
    index = next(i for i, line in enumerate(lines) if line.startswith("after_begin,"))
    fields = lines[index].split(","); fields[3] = "1400000"; lines[index] = ",".join(fields)
    capture[0].write_text("\n".join(lines) + "\n")
    assert build_window(capture)["producer_lag_us"] == 400


@pytest.mark.parametrize("mutation", ["before_request", "operation", "run", "boot", "session",
                                      "phase", "permit", "receipt_pending", "no_terminal"])
def test_release_early_idle_still_requires_this_request_and_clean_identity(tmp_path, mutation):
    capture = window_capture(tmp_path)
    def transform(item):
        when = item["when"]
        payload = item["raw"]
        boot, session = item["boot"], item["session"]
        if item["kind"] == 3:
            fields = dict(item["fields"])
            if mutation == "before_request": when = 2800000
            elif mutation == "operation": fields["operation_id"] = 99
            elif mutation == "run": fields["run_id"] = 9
            elif mutation == "boot": fields["boot_epoch"] = 9; boot = 9
            elif mutation == "session": fields["session_id"] = 3; session = 3
            elif mutation == "phase": fields["motion_epoch"] = 9
            elif mutation == "permit": fields["permit_remaining_us"] = 1
            payload = wire("DummyDeviceStatusV2", **fields)
        if item["kind"] == 5 and mutation == "receipt_pending":
            payload = wire("DummyManageV2Response", **{**item["fields"], "transaction_phase": 2})
        return record(item["kind"], when, boot, session, item["run"], item["seq"], payload)
    _rewrite_records(capture, transform)
    if mutation == "no_terminal":
        raw = capture[1]
        raw.write_bytes(b"DPRAW01\0" + b"".join(record(x["kind"], x["when"], x["boot"],
            x["session"], x["run"], x["seq"], x["raw"]) for x in joiner._raw_records(raw.read_bytes())
            if x["kind"] != 1))
    with pytest.raises(ValueError, match="Release"):
        build_window(capture)


def build_strict_window(capture):
    """真实Gate的250ms预算；不沿用老window夹具的1s采样容差。"""
    return builder.build_context(
        *capture, path_tolerance_rad=[0.01] * 6,
        goal_tolerance_rad=[0.003] * 6, producer_lag_budget_us=250000,
        feedback_age_budget_us=10000, ros_feedback_lag_budget_us=2000,
        gate_state_max_age_us=250000, producer_lag_scope="motion_window_v2")


def dense_cancel_capture(tmp_path):
    """20ms真wire角色：103接纳→104运行→105安全停止；Stop页晚300ms。

    Stop期间State仍报告旧事务op501，身份与管理代次已切到105/143。
    完整本run页链和精确Cancel Query回执提供安全相位证明。
    """
    capture = begun_cancel_capture(tmp_path)
    timeline, raw, _, _ = capture
    lines = timeline.read_text().splitlines()
    for index, line in enumerate(lines):
        if line.startswith("terminal_canceled,"):
            fields = line.split(","); fields[-1] = "508"; lines[index] = ",".join(fields)
    timeline.write_text("\n".join(lines) + "\n")
    hashes = {"contract_sha256": "41" * 32, "model_sha256": "42" * 32,
              "profile_sha256": "43" * 32, "firmware_sha256": "44" * 32}
    events = []
    cancel_fields = None
    for item in joiner._raw_records(raw.read_bytes()):
        if item["kind"] == 2 and 1000000 <= item["when"] <= 4000000:
            continue
        if item["kind"] == 1 and item["run"] == 7:
            continue
        if item["kind"] == 5 and item["fields"]["request_id"] == 7:
            continue
        payload = item["raw"]
        if item["kind"] == 4 and item["fields"]["operation"] == 9:
            cancel_fields = {**item["fields"], "expected_reference_epoch": 1,
                             "expected_motion_epoch": 104, "expected_management_epoch": 142,
                             "expected_profile_id": 201, "configured_mask": 63}
            payload = wire("DummyManageV2Request", **cancel_fields)
        elif item["kind"] == 3:
            # Only the later Release Idle Status supplies the immutable digest tuple.
            # Fast canceled goals need not publish any same-run HOLD Status.
            payload = wire("DummyDeviceStatusV2", **{
                **item["fields"], **hashes, "profile_id": 201, "configured_mask": 63})
        events.append(record(item["kind"], item["when"], item["boot"],
                             item["session"], item["run"], item["seq"], payload))
    seed = struct.pack("<QQ", 7, 2) + bytes.fromhex("11" * 16)
    seed += b"".join(bytes.fromhex(hashes[name]) for name in
                     ("contract_sha256", "model_sha256", "profile_sha256", "firmware_sha256"))
    chain = hashlib.sha256(seed.ljust(192, b"\0")).digest()
    stop_fields = dict(run_id=7, attempt_id=7, attempt_kind=1, parent_run_id=0,
                       evidence_seq=0, category=2, motion_epoch=105, group_id=1,
                       group_count=1, producer_seq=1, expected_mask=63,
                       submitted_mask=63, sent_mask=63, failed_mask=0, final=1,
                       queued_at_us=[2000100] * 6, taken_at_us=[2000200] * 6,
                       submitted_at_us=[2000300] * 6, sent_at_us=[2000400] * 6)
    for when, fields in ((2300000, stop_fields), (2350000, dict(
            run_id=7, attempt_id=7, attempt_kind=1, parent_run_id=0,
            evidence_seq=1, category=4, motion_epoch=103, group_count=1,
            final=1, final_state=7, motion_complete=0, release_committed=0))):
        fields = {**fields, "previous_chain_sha256": chain.hex()}
        payload = wire("DummyAttemptEvidenceV2", **fields)
        chain = hashlib.sha256(chain + payload[:307] + payload[371:372]).digest()
        payload = wire("DummyAttemptEvidenceV2", **fields, chain_sha256=chain.hex())
        events.append(record(1, when, 2, 3, 7, fields["evidence_seq"], payload))
    assert cancel_fields is not None
    query = wire("DummyManageV2Request", **{**cancel_fields, "operation": 12,
                 "request_id": 8, "request_seq": 8, "queried_operation_id": 7})
    events.append(record(4, 2360000, 2, 3, 0, 8, query))
    receipt = wire("DummyManageV2Response", client_instance_id="11" * 16,
                   device_id="22" * 16, request_id=8, operation_id=508,
                   boot_epoch=2, session_id=3, reference_epoch=1, motion_epoch=105,
                   management_epoch=143, profile_id=201, run_id=7, receipt=1,
                   transaction_phase=3, result_code=0)
    events.append(record(5, 2400000, 2, 3, 7, 0, receipt))
    for index, when in enumerate(range(1000000, 4000001, 20000)):
        session = 0 if when >= 3400000 else 3
        motion = (106 if session == 0 else 103 if when < 1100000 else
                  104 if when < 2000000 else 105)
        stopping = 2000000 <= when <= 2300000
        core = 1 if session == 0 else 7 if stopping else 5 if motion == 104 else 4
        state = wire("DummyJointStateV2", boot_epoch=2, session_id=session,
                     device_id="22" * 16, reference_epoch=0 if session == 0 else 1,
                     motion_epoch=motion, management_epoch=143 if motion >= 105 else 142,
                     profile_id=201, configured_mask=63, core_state=core,
                     operation_id=501 if stopping or motion < 105 else 508,
                     transaction_phase=3, hold_ready=int(core == 4),
                     position_valid_mask=63, velocity_valid_mask=63,
                     raw_position_valid_mask=63, can_rx_age_us=[1000] * 6,
                     state_seq=index + 1, produced_at_us=when + 100000 - (index % 5) * 100)
        events.append(record(2, when, 2, session, 0, index + 1, state))
    raw.write_bytes(b"DPRAW01\0" + b"".join(sorted(
        events, key=lambda value: joiner.RECORD_HEADER.unpack_from(value)[2])))
    return capture


def test_dense_cancel_stopping_states_with_old_operation_fill_250ms_coverage(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    stopping = [x for x in records if x["kind"] == 2 and
                2000000 <= x["when"] <= 2300000]
    assert len(stopping) == 16
    assert all(x["fields"]["operation_id"] == 501 for x in stopping)
    assert max(b["when"] - a["when"] for a, b in zip(stopping, stopping[1:])) == 20000
    assert build_strict_window(capture)["producer_lag_us"] == 400


def _alter_dense_transition(capture, changes):
    def transform(item):
        fields = dict(item["fields"])
        if item["kind"] == 2 and 2000000 <= item["when"] <= 2300000:
            fields.update(changes)
        name = {1: "DummyAttemptEvidenceV2", 2: "DummyJointStateV2",
                3: "DummyDeviceStatusV2", 4: "DummyManageV2Request",
                5: "DummyManageV2Response"}[item["kind"]]
        return record(item["kind"], item["when"],
                      fields.get("boot_epoch", item["boot"]),
                      fields.get("session_id", item["session"]),
                      item["run"], item["seq"], wire(name, **fields))
    _rewrite_records(capture, transform)


def _alter_dense_pages(capture, category, changes):
    """重新签夹具页链，保证SENT/终态负例不靠错误SHA偶然拒绝。"""
    records = joiner._raw_records(capture[1].read_bytes())
    first = next(x for x in records if x["kind"] == 1 and x["run"] == 7)
    previous = bytes.fromhex(first["fields"]["previous_chain_sha256"])
    rewritten = []
    for item in records:
        payload = item["raw"]
        if item["kind"] == 1 and item["run"] == 7:
            fields = dict(item["fields"])
            if fields["category"] == category:
                fields.update(changes)
            fields["previous_chain_sha256"] = previous.hex()
            payload = wire("DummyAttemptEvidenceV2", **fields)
            previous = hashlib.sha256(previous + payload[:307] + payload[371:372]).digest()
            fields["chain_sha256"] = previous.hex()
            payload = wire("DummyAttemptEvidenceV2", **fields)
        rewritten.append(record(item["kind"], item["when"], item["boot"],
                                item["session"], item["run"], item["seq"], payload))
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(rewritten))


def test_dense_cancel_ready_hold_state_with_old_operation_fills_coverage(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    _alter_dense_transition(capture, {"core_state": 4, "hold_ready": 1})
    assert build_strict_window(capture)["producer_lag_us"] == 400


@pytest.mark.parametrize("changes", [
    {"boot_epoch": 3}, {"session_id": 4}, {"device_id": "33" * 16},
    {"reference_epoch": 2}, {"motion_epoch": 104}, {"management_epoch": 142},
    {"profile_id": 202}, {"configured_mask": 31}, {"core_state": 5},
    {"position_valid_mask": 0}, {"velocity_valid_mask": 31},
    {"raw_position_valid_mask": 0}, {"core_state": 4, "hold_ready": 0},
    {"produced_at_us": 0},
])
def test_dense_cancel_invalid_transition_state_cannot_fill_coverage(tmp_path, changes):
    capture = dense_cancel_capture(tmp_path)
    _alter_dense_transition(capture, changes)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


@pytest.mark.parametrize("changes", [
    {"request_id": 99}, {"operation_id": 507}, {"reference_epoch": 2},
    {"motion_epoch": 104}, {"management_epoch": 142}, {"profile_id": 202},
    {"client_instance_id": "33" * 16}, {"run_id": 9},
    {"transaction_phase": 4, "result_code": 10},
])
def test_dense_cancel_wrong_receipt_cannot_authorize_transition(tmp_path, changes):
    capture = dense_cancel_capture(tmp_path)
    def transform(item):
        if item["kind"] == 5 and item["fields"]["request_id"] == 8:
            fields = {**item["fields"], **changes}
            return record(5, item["when"], item["boot"], item["session"],
                          fields["run_id"], 0, wire("DummyManageV2Response", **fields))
        return record(item["kind"], item["when"], item["boot"],
                      item["session"], item["run"], item["seq"], item["raw"])
    _rewrite_records(capture, transform)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


@pytest.mark.parametrize("category,changes", [
    (2, {"sent_mask": 31}), (2, {"failed_mask": 1}),
    (2, {"submitted_mask": 31}), (2, {"evidence_lost": 1}),
    (4, {"final_state": 6, "motion_complete": 1}),
    (4, {"motion_epoch": 104}), (4, {"first_failure_source": 1}),
])
def test_dense_cancel_invalid_full_chain_cannot_authorize_transition(tmp_path, category, changes):
    capture = dense_cancel_capture(tmp_path)
    _alter_dense_pages(capture, category, changes)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


def test_dense_cancel_corrupt_sha_cannot_authorize_transition(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    def transform(item):
        payload = item["raw"]
        if item["kind"] == 1 and item["run"] == 7 and item["seq"] == 0:
            payload = payload[:339] + bytes([payload[339] ^ 1]) + payload[340:]
        return record(item["kind"], item["when"], item["boot"],
                      item["session"], item["run"], item["seq"], payload)
    _rewrite_records(capture, transform)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


def test_dense_cancel_missing_full_proof_cannot_authorize_transition(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(record(
        x["kind"], x["when"], x["boot"], x["session"], x["run"], x["seq"], x["raw"])
        for x in records if not (x["kind"] == 1 and x["run"] == 7)))
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


def test_dense_cancel_ambiguous_static_identity_quad_cannot_authorize_transition(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    status = next(x for x in records if x["kind"] == 3)
    fields = {**status["fields"], "contract_sha256": "55" * 32}
    # Both Status observations are later unrelated Release Idle observations.
    extra = record(3, 3850000, 2, 0, 8, 0, wire("DummyDeviceStatusV2", **fields))
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(sorted(
        [record(x["kind"], x["when"], x["boot"], x["session"], x["run"], x["seq"], x["raw"])
         for x in records] + [extra], key=lambda x: joiner.RECORD_HEADER.unpack_from(x)[2])))
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


def _alter_dense_cancel_binding(capture, *, mask=None, profile=None, reference=None):
    def transform(item):
        fields = dict(item["fields"])
        wanted = item["kind"] == 4 and (fields["operation"] == 9 or
                 (fields["operation"] == 12 and fields["queried_operation_id"] == 7))
        receipt = item["kind"] == 5 and fields["request_id"] == 8
        status = item["kind"] == 3
        transition = item["kind"] == 2 and 2000000 <= item["when"] <= 2300000
        if mask is not None and (wanted or status):
            fields["configured_mask"] = mask
        if profile is not None and (wanted or receipt or status or transition):
            fields["expected_profile_id" if wanted else "profile_id"] = profile
        if reference is not None and (wanted or receipt):
            fields["expected_reference_epoch" if wanted else "reference_epoch"] = reference
        name = {1: "DummyAttemptEvidenceV2", 2: "DummyJointStateV2",
                3: "DummyDeviceStatusV2", 4: "DummyManageV2Request",
                5: "DummyManageV2Response"}[item["kind"]]
        return record(item["kind"], item["when"], item["boot"], item["session"],
                      item["run"], item["seq"], wire(name, **fields))
    _rewrite_records(capture, transform)


@pytest.mark.parametrize("changes", [{"mask": 1}, {"profile": 0}, {"reference": 2}])
def test_dense_cancel_wanted_authority_must_bind_goal(tmp_path, changes):
    capture = dense_cancel_capture(tmp_path)
    _alter_dense_cancel_binding(capture, **changes)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


def _cancel_disabling_after_verified_release_request(tmp_path,mutation=None,release_state=7):
 c=dense_cancel_capture(tmp_path)
 def transform(row):
  fields=dict(row['fields']);boot=row['boot'];session=row['session'];when=row['when']
  if row['kind']==4 and fields['operation']==10:
   fields.update(expected_reference_epoch=1,expected_motion_epoch=105,expected_management_epoch=143,expected_profile_id=201,configured_mask=63)
  if row['kind']==2 and 2900000<=when<3400000:
   fields.update(motion_epoch=106,management_epoch=144,core_state=release_state,hold_ready=0)
   if mutation=='epoch':fields['motion_epoch']=107
   elif mutation=='management':fields['management_epoch']=145
   elif mutation=='boot':fields['boot_epoch']=boot=3
   elif mutation=='session':fields['session_id']=session=4
   elif mutation=='ref':fields['reference_epoch']=2
   elif mutation=='before':when=2899999
   elif mutation=='stateRun':fields['core_state']=5
   elif mutation=='stateHold':fields['core_state']=4;fields['hold_ready']=1
   elif mutation=='mask':fields['configured_mask']=31
   elif mutation=='profile':fields['profile_id']=202
  if row['kind']==5 and fields['request_id']==9:
   fields['management_epoch']=144
   fields['request_id']=11
   if mutation=='receipt':fields['operation_id']=999
  if row['session']==0 and row['kind'] in (2,3):fields['management_epoch']=144
  if mutation=='noIdle'and row['session']==0 and row['kind']in(2,3):fields['core_state']=4
  if mutation=='sent31'and row['kind']==1 and fields.get('category')==2:fields['sent_mask']=31
  if mutation=='chain'and row['kind']==1 and fields.get('category')==2:fields['chain_sha256']='ff'*32
  name={1:'DummyAttemptEvidenceV2',2:'DummyJointStateV2',3:'DummyDeviceStatusV2',4:'DummyManageV2Request',5:'DummyManageV2Response'}[row['kind']]
  return record(row['kind'],when,boot,session,row['run'],row['seq'],wire(name,**fields))
 _rewrite_records(c,transform)
 records=joiner._raw_records(c[1].read_bytes());root=next(r['fields']for r in records if r['kind']==4 and r['fields']['operation']==10)
 query=wire('DummyManageV2Request',**{**root,'operation':12,'request_id':11,'request_seq':11,'queried_operation_id':9})
 rows=[record(r['kind'],r['when'],r['boot'],r['session'],r['run'],r['seq'],r['raw'])for r in records if not(mutation=='noTypedDisable'and r['kind']==4 and r['fields']['operation']==10)]
 rows.append(record(4,3305000,2,3,0,11,query));c[1].write_bytes(b'DPRAW01\0'+b''.join(sorted(rows,key=lambda b:joiner.RECORD_HEADER.unpack_from(b)[2])))
 return c

def test_cancel_Disabling_after_current_bound_Disable_fills_window(tmp_path):
 c=_cancel_disabling_after_verified_release_request(tmp_path);result=build_strict_window(c)
 assert result['producer_lag_method']==builder.WINDOW_LAG_METHOD
@pytest.mark.parametrize('bad',['before','epoch','boot','session','ref','receipt','chain','sent31','stateRun','stateHold','mask','profile'])
def test_cancel_Disabling_without_bound_release_or_full_stop_proof_is_rejected(tmp_path,bad):
 c=_cancel_disabling_after_verified_release_request(tmp_path,bad)
 with pytest.raises(ValueError):build_strict_window(c)
 assert not c[3].exists()


@pytest.mark.parametrize('release_state',[7,8])
def test_cancel_release_State7_and_actual_Disabling8_bound_receipt_management(tmp_path,release_state):
 c=_cancel_disabling_after_verified_release_request(tmp_path,release_state=release_state)
 result=build_strict_window(c)
 assert result['producer_lag_method']==builder.WINDOW_LAG_METHOD
@pytest.mark.parametrize('bad',['before','epoch','boot','session','ref','management','receipt','chain','sent31','stateRun','stateHold','mask','profile','noTypedDisable','noIdle'])
@pytest.mark.parametrize('release_state',[7,8])
def test_cancel_release_State7_8_must_match_full_Current_Disable_proof(tmp_path,bad,release_state):
 c=_cancel_disabling_after_verified_release_request(tmp_path,bad,release_state=release_state)
 with pytest.raises(ValueError):build_strict_window(c)
 assert not c[3].exists()
