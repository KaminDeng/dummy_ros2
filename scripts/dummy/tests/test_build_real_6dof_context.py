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
                  run_id=8, core_state=1, release_committed=1,
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
