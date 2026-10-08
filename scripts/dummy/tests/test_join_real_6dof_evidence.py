"""离线拼接 Gate CSV 与同一 DeviceTransport 的 DPRAW01 记录。"""

import copy
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import sys
import time

import pytest
from scripts.dummy import build_real_6dof_context as builder


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("join_real_6dof_evidence", ROOT / "join_real_6dof_evidence.py")
joiner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(joiner)
APP_SPEC = importlib.util.spec_from_file_location("application_e2e", ROOT / "application_e2e.py")
application_e2e = importlib.util.module_from_spec(APP_SPEC)
APP_SPEC.loader.exec_module(application_e2e)
LAYOUT = json.loads((ROOT.parents[1] / "contracts/dummy_robot_v2/codec/wire_layout.json").read_text())


def wire(name, **values):
    spec = LAYOUT["payloads"][name]
    raw = bytearray(spec["encoded_size"])
    codes = {"uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q",
             "float64": "d", "bool": "B"}
    values = {"schema_major": 2, "schema_minor": 4, **values}
    for field in spec["fields"]:
        if field["name"] not in values:
            continue
        value = values[field["name"]]
        if field["count"] > 1:
            if isinstance(value, str):
                value = bytes.fromhex(value)
            value = list(value)
        else:
            value = [value]
        struct.pack_into("<" + codes[field["type"]] * field["count"], raw,
                         field["offset"], *value)
    return bytes(raw)


def record(kind, when, boot, session, run, seq, payload):
    return struct.pack("<B H Q Q Q Q I", kind, len(payload), when, boot, session,
                       run, seq) + payload


def test_large_raw_state_lookups_remain_bounded():
    payload = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                   reference_epoch=1, motion_epoch=2)
    raw = b"DPRAW01\0" + b"".join(
        record(2, index * 10, 2, 3, 0, 0, payload) for index in range(20000))
    records = joiner._raw_records(raw)
    start = time.perf_counter()
    for index in range(1, 500):
        observed = joiner._prior(records, 2, index * 400, (2, 3, 1, 2), 1000)
        assert observed["when"] == index * 400
    assert time.perf_counter() - start < 0.12


def test_indexed_raw_record_metadata_cannot_change_after_parsing():
    payload = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                   reference_epoch=1, motion_epoch=2)
    records = joiner._raw_records(b"DPRAW01\0" +
        record(2, 100, 2, 3, 0, 0, payload))
    with pytest.raises(TypeError):
        records[0]["when"] = 200
    with pytest.raises(AttributeError):
        records.append(records[0])
    assert joiner._prior(records, 2, 100, (2, 3, 1, 2), 10)["when"] == 100
    source = dict(records[0])
    indexed = joiner._IndexedRecords([source])
    source["when"] = 200
    assert joiner._prior(indexed, 2, 100, (2, 3, 1, 2), 10)["when"] == 100


def test_equal_time_latest_state_with_wrong_identity_fails_closed():
    first = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                 reference_epoch=1, motion_epoch=2)
    second = wire("DummyJointStateV2", boot_epoch=2, session_id=4,
                  reference_epoch=1, motion_epoch=2)
    records = joiner._raw_records(b"DPRAW01\0" +
        record(2, 100, 2, 3, 0, 0, first) +
        record(2, 100, 2, 4, 0, 0, second))
    with pytest.raises(ValueError, match="ambiguous prior raw state identity"):
        joiner._prior(records, 2, 100, (2, 3, 1, 2), 10)


def test_large_desired_path_interpolation_remains_bounded():
    samples = [(index * 400, [float(index)] * 6) for index in range(500)]
    start = time.perf_counter()
    total = sum(joiner._desired_at(samples, index)[0] for index in range(30000))
    elapsed = time.perf_counter() - start
    assert total == pytest.approx(30000 * 29999 / 800)
    assert elapsed < 0.12


def page(identity, run, boot, seq, previous, *, release=False, epoch=2,
         parent=4, sent_mask=63, group_count=1, group_id=None,
         active_release=False, canceled=False, category=1):
    seed = struct.pack("<QQ", run, boot) + bytes.fromhex(identity["client_id"])
    seed += b"".join(bytes.fromhex(identity[key]) for key in
                     ("contract_hash", "model_hash", "profile_hash", "firmware_hash"))
    if seq == 0:
        previous = hashlib.sha256(seed.ljust(192, b"\0")).digest()
    terminal = seq == group_count
    raw = bytearray(wire("DummyAttemptEvidenceV2", run_id=run, evidence_seq=seq,
        group_id=0 if terminal else group_id or (8 if release else 7), motion_epoch=epoch,
        category=4 if terminal else category, expected_mask=0 if terminal else 63,
        submitted_mask=0 if terminal else 63,
        sent_mask=0 if terminal else sent_mask, final=1,
        group_count=group_count if terminal else seq + 1,
        attempt_id=run, attempt_kind=3 if release else 1,
        parent_run_id=parent if release else 0,
        final_state=(2 if active_release else 8 if release else 7 if canceled else 6)
                    if terminal else 0,
        motion_complete=int(terminal and not (release or active_release or canceled)),
        release_committed=int(terminal and (release or active_release))))
    raw[307:339] = previous
    raw[339:371] = hashlib.sha256(previous + raw[:307] + raw[371:372]).digest()
    return bytes(raw)


@pytest.fixture
def inputs(tmp_path):
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    names = ("event,goal_uuid,trajectory_sha256,steady_us,boot,session,reference_epoch,"
             "motion_epoch,run_id,axis_mask,status_core_state,state_core_state," +
             ",".join(f"feedback_seq_{i}" for i in range(6)) + "," +
             ",".join(f"feedback_age_us_{i}" for i in range(6)) +
             ",release_request_id,release_operation_id")
    rows = []
    events = ("accepted", "before_begin", "after_begin", "feedback",
              "terminal_success", "pre_release_hold", "post_release_status")
    for index, event in enumerate(events):
        rows.append([event, "a" * 32, "b" * 64, (index + 1) * 1000, 2,
                     0 if index == 6 else 3, 0 if index == 6 else 1,
                     3 if index == 6 else 2, 5 if index == 6 else 4, 63,
                     1 if index == 6 else 4 if index == 5 else 3,
                     1 if index == 6 else 4 if index == 5 else 3,
                     *([1] * 6), *([1000] * 6),
                     2 if index == 6 else 0, 2 if index == 6 else 0])
    timeline = tmp_path / "timeline.csv"
    with timeline.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(names.split(","))
        writer.writerows(rows)
        writer.writerow(["COMPLETE", "1"])
    status_common = dict(device_id="22" * 16,
        boot_epoch=2, reference_epoch=1, motion_epoch=2,
        profile_id=201, configured_mask=63,
        contract_sha256=identity["contract_hash"], model_sha256=identity["model_hash"],
        profile_sha256=identity["profile_hash"], firmware_sha256=identity["firmware_hash"])
    state_common = dict(device_id="22" * 16,
        boot_epoch=2, reference_epoch=1, motion_epoch=2,
        profile_id=201, configured_mask=63, position_valid_mask=63,
        feedback_seq=[1] * 6, can_rx_age_us=[1000] * 6,
        position_rad=[0.01] * 6)
    first = page(identity, 4, 2, 0, b"")
    second = page(identity, 4, 2, 1, first[339:371])
    release_first = page(identity, 5, 2, 0, b"", release=True, epoch=3, category=3)
    release_last = page(identity, 5, 2, 1, release_first[339:371], release=True, epoch=2)
    records = [
        record(3, 2900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **status_common,
               session_id=3, run_id=4, core_state=3)),
        record(2, 3000, 2, 3, 0, 0, wire("DummyJointStateV2", **state_common,
               session_id=3, core_state=3)),
        record(1, 3500, 2, 3, 4, 0, first),
        record(2, 4000, 2, 3, 0, 0, wire("DummyJointStateV2", **state_common,
               session_id=3, core_state=3)),
        record(1, 4500, 2, 3, 4, 1, second),
        record(3, 5900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **status_common,
               session_id=3, run_id=4, core_state=4, final_state=6, motion_complete=1,
               attempt_kind=1, evidence_chain_sha256=second[339:371].hex())),
        record(2, 6000, 2, 3, 0, 0, wire("DummyJointStateV2", **state_common,
               session_id=3, core_state=4, hold_ready=1)),
        record(1, 6400, 2, 3, 5, 0, release_first),
        record(1, 6500, 2, 3, 5, 1, release_last),
        record(3, 7000, 2, 0, 0, 0, wire("DummyDeviceStatusV2", **{
               **status_common, "session_id": 0, "reference_epoch": 0,
               "motion_epoch": 3, "run_id": 5, "core_state": 1,
               "attempt_kind": 3, "final_state": 8, "release_committed": 1,
               "evidence_chain_sha256": release_last[339:371].hex()})),
        record(2, 7000, 2, 0, 0, 0, wire("DummyJointStateV2", **{
               **state_common, "session_id": 0, "reference_epoch": 0,
               "motion_epoch": 3, "core_state": 1})),
    ]
    raw = tmp_path / "raw.bin"
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    action = {"scope": "ROS_ACTION_ONLY", "status": "SUCCEEDED",
              "goal_id": "a" * 32, "action_status": 4,
              "trajectory_sha256": "b" * 64,
              "action_started_steady_us": 500,
              "result_error_code": 0, "target_rad": [0.01] * 6,
              "joint_names": [f"Joint{i}" for i in range(1, 7)],
              "stop_confirmation": "CONFIRMED_RELEASED_IDLE",
              "feedback": [{"elapsed_us": 3500, "actual_rad": [0.01] * 6,
                            "desired_rad": [0.01] * 6}],
              "device_can_evidence": "REQUIRED_SEPARATELY",
              "device_qualification": "NOT_CLAIMED"}
    action_path = tmp_path / "action.json"
    action_path.write_text(json.dumps(action))
    context = {"client_id": identity["client_id"],
               "path_tolerance_rad": [0.01] * 6,
               "goal_tolerance_rad": [0.003] * 6,
               "producer_lag_us": 1000, "producer_lag_budget_us": 10000,
               "ros_feedback_lag_budget_us": 2000,
               "gate_state_max_age_us": 250000,
               "feedback_age_budget_us": 10000,
               "timeline_sha256": hashlib.sha256(timeline.read_bytes()).hexdigest(),
               "raw_sha256": hashlib.sha256(raw.read_bytes()).hexdigest(),
               "action_sha256": hashlib.sha256(action_path.read_bytes()).hexdigest()}
    context_path = tmp_path / "context.json"
    context_path.write_text(json.dumps(context))
    repin(timeline, raw, action_path, context_path)
    return timeline, raw, action_path, context_path, records


def repin(timeline, raw, action, context, *, preserve_state_sequence=False):
    timeline_rows = list(csv.DictReader(timeline.open(newline="")))
    release_epochs = {int(row["run_id"]): int(row["motion_epoch"])
                      for row in timeline_rows if row.get("event") == "post_release_status"}
    terminal_times = {int(row["run_id"]): int(row["steady_us"])
                      for row in timeline_rows if row.get("event") == "terminal_success"}
    source = raw.read_bytes()
    if source.startswith(b"DPRAW01\0"):
        chunks = []
        offset = 8
        while offset < len(source):
            kind, size, when, boot, session, run, seq = struct.unpack_from(
                "<B H Q Q Q Q I", source, offset)
            end = offset + 39 + size
            chunks.append((kind, when, boot, session, run, seq,
                           source[offset + 39:end]))
            offset = end
        if not any(item[0] in (4, 5) for item in chunks):
            additions = []
            request_id = 0
            for kind, when, boot, session, run, seq, payload in chunks:
                if kind != 1 or payload[32] != 4 or payload[304] not in (6, 8):
                    continue
                request_id += 1
                operation = 8 if payload[304] == 6 else 10
                request_time = min(when - 1, terminal_times.get(run, when) - 1) \
                    if operation == 8 else min(item[1] for item in chunks if
                                               item[0] == 1 and item[4] == run) - 1
                request = wire("DummyManageV2Request", device_id="22" * 16,
                    client_instance_id="11" * 16, request_id=request_id,
                    request_seq=request_id, operation=operation,
                    expected_boot_epoch=boot, expected_session_id=session,
                    expected_profile_id=201)
                response = wire("DummyManageV2Response", device_id="22" * 16,
                    client_instance_id="11" * 16, request_id=request_id,
                    operation_id=request_id, receipt=1,
                    transaction_phase=3, result_code=0,
                    boot_epoch=boot, session_id=0 if operation == 10 else session,
                    profile_id=201, run_id=run,
                    motion_epoch=release_epochs.get(run, 0) if operation == 10 else 0,
                    motion_complete=int(operation == 8),
                    release_committed=int(operation == 10))
                additions.extend(((request_time, record(4, request_time, boot, session, 0,
                                                   request_id, request)),
                                  (when + 1, record(5, when + 1, boot,
                                                   0 if operation == 10 else session,
                                                   run, 0, response))))
            if additions:
                pending = iter(sorted(additions, key=lambda item: item[0]))
                addition = next(pending, None)
                combined = []
                for item in chunks:
                    while addition is not None and addition[0] <= item[1]:
                        combined.append(addition[1])
                        addition = next(pending, None)
                    combined.append(record(*item))
                while addition is not None:
                    combined.append(addition[1])
                    addition = next(pending, None)
                raw.write_bytes(b"DPRAW01\0" + b"".join(combined))
        if not preserve_state_sequence:
            source = raw.read_bytes()
            rebuilt = []
            offset = 8
            state_seq = 0
            while offset < len(source):
                kind, size, when, boot, session, run, seq = struct.unpack_from(
                    "<B H Q Q Q Q I", source, offset)
                end = offset + 39 + size
                if end > len(source):
                    rebuilt = None
                    break
                payload = bytearray(source[offset + 39:end])
                if kind == 2:
                    state_seq += 1
                    struct.pack_into("<IQ", payload, 52, state_seq, when)
                rebuilt.append(record(kind, when, boot, session, run, seq, payload))
                offset = end
            if rebuilt is not None:
                raw.write_bytes(b"DPRAW01\0" + b"".join(rebuilt))
    value = json.loads(context.read_text())
    for key, path in (("timeline_sha256", timeline), ("raw_sha256", raw),
                      ("action_sha256", action)):
        value[key] = hashlib.sha256(path.read_bytes()).hexdigest()
    context.write_text(json.dumps(value))


def canceled_inputs(inputs):
    timeline, raw, action_path, context, original = inputs
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    rows = list(csv.reader(timeline.open(newline="")))
    rows[5][0], rows[5][7] = "terminal_canceled", "3"
    rows[6][7] = "3"
    rows[7][7] = "4"
    rows[0].extend(("cancel_request_id", "cancel_operation_id"))
    for row in rows[1:-1]:
        row.extend(("1", "1") if row[0] == "terminal_canceled" else ("0", "0"))
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    action = json.loads(action_path.read_text())
    action.update(status="CANCELED", action_status=5,
        cancel={"goal_id": action["goal_id"], "requested": True,
                "acknowledged": True, "motion_feedback_start_index": 0,
                "motion_feedback_trigger_index": 1, "motion_delta_rad": 0.005},
        feedback=[{"elapsed_us": 3400, "actual_rad": [0.0] * 6,
                   "desired_rad": [0.01] * 6, "goal_id": action["goal_id"]},
                  {**action["feedback"][0], "goal_id": action["goal_id"]}])
    action_path.write_text(json.dumps(action))

    first = original[2][39:]
    stop = page(identity, 4, 2, 1, first[339:371], canceled=True,
                epoch=3, group_count=2, group_id=8, category=2)
    terminal = page(identity, 4, 2, 2, stop[339:371], canceled=True,
                    epoch=2, group_count=2)
    status = joiner._decode("DummyDeviceStatusV2", original[5][39:])
    status.update(motion_epoch=3, final_state=7, motion_complete=0,
                  evidence_chain_sha256=terminal[339:371].hex())
    state = joiner._decode("DummyJointStateV2", original[6][39:])
    state["motion_epoch"] = 3
    release_first = page(identity, 5, 2, 0, b"", release=True, epoch=4,
                         category=3)
    release_last = page(identity, 5, 2, 1, release_first[339:371],
                        release=True, epoch=3)
    idle = joiner._decode("DummyDeviceStatusV2", original[9][39:])
    idle["motion_epoch"] = 4
    idle["evidence_chain_sha256"] = release_last[339:371].hex()
    idle_state = joiner._decode("DummyJointStateV2", original[10][39:])
    idle_state["motion_epoch"] = 4
    def management(operation, request_id, when, run, *, response=False):
        if response:
            payload = wire("DummyManageV2Response", device_id="22" * 16,
                client_instance_id=identity["client_id"], request_id=request_id,
                operation_id=request_id, receipt=1, transaction_phase=3,
                result_code=0, boot_epoch=2,
                session_id=0 if operation == 10 else 3,
                profile_id=201, run_id=run,
                motion_epoch=3 if operation == 9 else 4,
                release_committed=int(operation == 10))
            return record(5, when, 2, 0 if operation == 10 else 3,
                          run, 0, payload)
        payload = wire("DummyManageV2Request", device_id="22" * 16,
            client_instance_id=identity["client_id"], request_id=request_id,
            request_seq=request_id, operation=operation,
            expected_boot_epoch=2, expected_session_id=3, expected_profile_id=201,
            expected_motion_epoch=2 if operation == 9 else 3)
        return record(4, when, 2, 3, 0, request_id, payload)
    records = [*original[:4], management(9, 1, 4200, 4),
        record(1, 4500, 2, 3, 4, 1, stop),
        record(1, 4700, 2, 3, 4, 2, terminal),
        management(9, 1, 4800, 4, response=True),
        record(3, 5900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **status)),
        record(2, 6000, 2, 3, 0, 0, wire("DummyJointStateV2", **state)),
        management(10, 2, 6150, 5),
        record(1, 6400, 2, 3, 5, 0, release_first),
        record(1, 6500, 2, 3, 5, 1, release_last),
        management(10, 2, 6600, 5, response=True),
        record(3, 7000, 2, 0, 0, 0, wire("DummyDeviceStatusV2", **idle)),
        record(2, 7000, 2, 0, 0, 0, wire("DummyJointStateV2", **idle_state))]
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action_path, context)
    return timeline, raw, action_path, context, records


def unconfirmed_cancel_report(action):
    report = json.loads(action.read_text())
    report.update(status="UNCONFIRMED_STOP", stop_confirmation="UNCONFIRMED_STOP",
                  action_observation="CANCELED", safety_verdict="NOT_QUALIFIED",
                  raw_stop_release_join="REQUIRED_SEPARATELY",
                  device_can_evidence="REQUIRED_SEPARATELY",
                  device_release_evidence="REQUIRED_SEPARATELY",
                  device_qualification="NOT_CLAIMED",
                  release_observation={"request_reply": "RELEASE_QUEUED",
                                       "bootstrap_reply": "RELEASED_IDLE"})
    action.write_text(json.dumps(report))
    return report


def test_canceled_goal_joins_cancel_and_release_raw_chain(inputs):
    timeline, raw, action, context, _ = canceled_inputs(inputs)
    joined = joiner.join_evidence(timeline, raw, action, context)
    assert joined["goals"][0]["action_status"] == "CANCELED"
    assert joined["goals"][0]["cancel_request_id"] == 1
    assert joined["release"]["terminal"] == "RELEASED"


@pytest.mark.parametrize("terminal_time,match", [
    (4400, "Safety Stop CAN SENT follows Gate terminal"),
    (4600, "cancel committed receipt follows Gate terminal"),
])
def test_canceled_goal_rejects_raw_stop_after_gate_terminal(
        inputs, terminal_time, match):
    timeline, raw, action, context, _ = canceled_inputs(inputs)
    rows = list(csv.reader(timeline.open(newline="")))
    rows[5][3] = str(terminal_time)
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match=match):
        joiner.join_evidence(timeline, raw, action, context)


def test_raw_join_confirms_cli_unconfirmed_cancel_only_after_full_chain(inputs):
    timeline, raw, action, context, _ = canceled_inputs(inputs)
    unconfirmed_cancel_report(action)
    repin(timeline, raw, action, context)

    joined = joiner.join_evidence(timeline, raw, action, context)

    assert joined["goals"][0]["action_status"] == "CANCELED"
    assert joined["release"]["confirmed"] is True


def preempt_inputs(inputs):
    timeline, raw, action_path, context, records = canceled_inputs(inputs)
    report = unconfirmed_cancel_report(action_path)
    a, b, c = report["goal_id"], "c" * 32, "d" * 32
    goals = {"A": {"goal_id": a, "status": "CANCELED",
                   "phase": "ACTION_TERMINAL_RAW_PENDING"},
             "B": {"goal_id": b, "status": "REJECTED",
                   "phase": "A_MOTION_FEEDBACK_OVERLAP",
                   "second_begin_raw_join": "REQUIRED_SEPARATELY"},
             "C": {"goal_id": c, "status": "NOT_SUBMITTED",
                   "phase": "WAITING_FOR_CANCEL_RAW_AND_NEW_PERMIT"}}
    report.update(scope="ROS_PREEMPT_PROBE", preempt_goals=goals,
                  goals=copy.deepcopy(goals),
                  second_begin_raw_join="REQUIRED_SEPARATELY",
                  qualification="NOT_QUALIFIED")
    report["cancel"]["overlap_probe"] = copy.deepcopy(goals["B"])
    action_path.write_text(json.dumps(report))
    begin_request = wire("DummyManageV2Request", device_id="22" * 16,
        client_instance_id="11" * 16, request_id=3, request_seq=3,
        operation=7, expected_boot_epoch=2, expected_session_id=3,
        expected_reference_epoch=1, expected_motion_epoch=2,
        expected_profile_id=201, configured_mask=63)
    query = wire("DummyManageV2Request", device_id="22" * 16,
        client_instance_id="11" * 16, request_id=4, request_seq=4,
        operation=12, queried_operation_id=3, expected_boot_epoch=2,
        expected_session_id=3, expected_reference_epoch=1,
        expected_motion_epoch=2, expected_profile_id=201, configured_mask=63)
    committed = wire("DummyManageV2Response", device_id="22" * 16,
        client_instance_id="11" * 16, request_id=4, operation_id=3,
        receipt=1, transaction_phase=3, result_code=0, boot_epoch=2,
        session_id=3, profile_id=201, motion_epoch=2)
    records[:0] = [record(4, 2100, 2, 3, 0, 3, begin_request),
                    record(4, 2200, 2, 3, 0, 4, query),
                    record(5, 2800, 2, 3, 0, 0, committed)]
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action_path, context)
    return timeline, raw, action_path, context, records


def test_preempt_probe_joins_one_canceled_motion_and_rejected_overlap(inputs):
    timeline, raw, action, context, _ = preempt_inputs(inputs)
    joined = joiner.join_evidence(timeline, raw, action, context)
    assert joined["goals"][0]["action_status"] == "CANCELED"
    assert joined["preempt_goals"]["B"]["status"] == "REJECTED"
    assert joined["preempt_goals"]["C"]["status"] == "NOT_SUBMITTED"
    assert joined["begin_motion"]["request_id"] == 3
    assert joined["release"]["run_id"] == 5


def recovery_inputs(inputs):
    timeline, raw, action_path, context, records = canceled_inputs(inputs)
    rows = list(csv.reader(timeline.open(newline="")))
    second = []
    for index, event in enumerate(("accepted", "before_begin", "after_begin",
                                   "feedback", "terminal_success", "pre_release_hold",
                                   "post_release_status")):
        row = rows[index + 1].copy()
        row[:4] = [event, "c" * 32, "d" * 64, str(8000 + index * 1000)]
        row[7:9] = ["3" if index < 2 else "4", "4" if index < 2 else "5"]
        if index in (4, 5):
            row[7] = "5"
        row[10:12] = ["4" if index == 5 else "3", "4" if index == 5 else "3"]
        row[-4:] = ["0", "0", "0", "0"]
        if event == "post_release_status":
            row[5:9] = ["0", "0", "6", "6"]
            row[10:12] = ["1", "1"]
            row[-4:-2] = ["3", "3"]
        second.append(row)
    with timeline.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(rows[0])
        writer.writerows(rows[1:7] + second)
        writer.writerow(["COMPLETE", "2"])

    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    a = json.loads(action_path.read_text())
    a.update(index=0, stop_confirmation="NOT_REQUESTED")
    c = {**a, "index": 1, "goal_id": "c" * 32, "status": "SUCCEEDED",
         "action_status": 4, "trajectory_sha256": "d" * 64,
         "action_started_steady_us": 7500,
         "feedback": [{"elapsed_us": 3500, "actual_rad": [0.01] * 6,
                       "desired_rad": [0.01] * 6}], "cancel": None}
    b = {"goal_id": "b" * 32, "status": "REJECTED",
         "phase": "A_MOTION_FEEDBACK_OVERLAP",
         "second_begin_raw_join": "REQUIRED_SEPARATELY"}
    report = {"scope": "ROS_PREEMPT_RECOVERY", "status": "SUCCEEDED",
              "stop_confirmation": "CONFIRMED_RELEASED_IDLE",
              "requested_goals": 2, "completed_goals": 2,
              "goals": [a, c], "preempt_goals": {
                  "A": {"goal_id": a["goal_id"], "status": "CANCELED",
                        "phase": "ACTION_TERMINAL_RAW_PENDING"},
                  "B": b, "C": {"goal_id": c["goal_id"], "status": "SUCCEEDED"}},
              "second_begin_raw_join": "REQUIRED_SEPARATELY",
              "qualification": "NOT_QUALIFIED"}
    a["cancel"]["overlap_probe"] = copy.deepcopy(b)
    action_path.write_text(json.dumps(report))

    old_status = joiner._decode("DummyDeviceStatusV2", records[8][39:])
    old_state = joiner._decode("DummyJointStateV2", records[9][39:])
    new_status = {**old_status, "motion_epoch": 4, "run_id": 5,
                  "core_state": 3, "final_state": 0, "motion_complete": 0}
    new_state = {**old_state, "motion_epoch": 4, "core_state": 3,
                 "hold_ready": 0}
    prefix = page(identity, 5, 2, 0, b"", epoch=3, group_count=3, group_id=6)
    first = page(identity, 5, 2, 1, prefix[339:371], epoch=4,
                 group_count=3, group_id=7)
    complete_page = page(identity, 5, 2, 2, first[339:371], epoch=5,
                         group_count=3, group_id=8)
    terminal = page(identity, 5, 2, 3, complete_page[339:371], epoch=3,
                    group_count=3)
    release_first = page(identity, 6, 2, 0, b"", release=True, epoch=6,
                         parent=5, category=3)
    release_last = page(identity, 6, 2, 1, release_first[339:371],
                        release=True, epoch=5, parent=5)

    def management(operation, request_id, when, response_time, run,
                   expected_epoch, response_epoch=0):
        request = wire("DummyManageV2Request", device_id="22" * 16,
            client_instance_id=identity["client_id"], request_id=request_id,
            request_seq=request_id, operation=operation,
            expected_boot_epoch=2, expected_session_id=3,
            expected_reference_epoch=1, expected_motion_epoch=expected_epoch,
            expected_profile_id=201, configured_mask=63)
        response = wire("DummyManageV2Response", device_id="22" * 16,
            client_instance_id=identity["client_id"], request_id=request_id,
            operation_id=request_id, receipt=1, transaction_phase=3,
            result_code=0, boot_epoch=2,
            session_id=0 if operation == 10 else 3,
            profile_id=201, run_id=run, motion_epoch=response_epoch,
            motion_complete=int(operation == 8),
            release_committed=int(operation == 10))
        return (record(4, when, 2, 3, 0, request_id, request),
                record(5, response_time, 2, 0 if operation == 10 else 3,
                       run, 0, response))

    begin_a = management(7, 4, 2100, 2800, 4, 2, 2)
    permit = management(5, 5, 7500, 7600, 0, 3, 3)
    begin_c = management(7, 6, 9100, 9800, 5, 3, 4)
    complete = management(8, 7, 11200, 11700, 5, 4, 5)
    release = management(10, 3, 13150, 13600, 6, 5, 6)
    idle = joiner._decode("DummyDeviceStatusV2", records[-2][39:])
    idle.update(motion_epoch=6, run_id=6,
                evidence_chain_sha256=release_last[339:371].hex())
    idle_state = joiner._decode("DummyJointStateV2", records[-1][39:])
    idle_state["motion_epoch"] = 6
    additions = [*begin_a, *permit, *begin_c, *complete, *release,
        record(1, 9050, 2, 3, 5, 0, prefix),
        record(3, 9900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **new_status)),
        record(2, 10000, 2, 3, 0, 0, wire("DummyJointStateV2", **new_state)),
        record(1, 10500, 2, 3, 5, 1, first),
        record(2, 11000, 2, 3, 0, 0, wire("DummyJointStateV2", **new_state)),
        record(1, 11750, 2, 3, 5, 2, complete_page),
        record(1, 11900, 2, 3, 5, 3, terminal),
        record(3, 12900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **{
            **new_status, "motion_epoch": 5, "core_state": 4, "final_state": 6,
            "motion_complete": 1, "evidence_chain_sha256": terminal[339:371].hex()})),
        record(2, 13000, 2, 3, 0, 0, wire("DummyJointStateV2", **{
            **new_state, "motion_epoch": 5, "core_state": 4, "hold_ready": 1})),
        record(1, 13400, 2, 3, 6, 0, release_first),
        record(1, 13500, 2, 3, 6, 1, release_last),
        record(3, 14000, 2, 0, 0, 0, wire("DummyDeviceStatusV2", **idle)),
        record(2, 14000, 2, 0, 0, 0, wire("DummyJointStateV2", **idle_state))]
    records = sorted(records[:10] + additions, key=lambda item:
        struct.unpack_from("<Q", item, 3)[0])
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action_path, context)
    return timeline, raw, action_path, context, records


def test_preempt_recovery_joins_canceled_a_and_successful_c(inputs):
    timeline, raw, action, context, _ = recovery_inputs(inputs)
    joined = joiner.join_evidence(timeline, raw, action, context)
    assert [goal["action_status"] for goal in joined["goals"]] == ["CANCELED", "SUCCEEDED"]
    assert [item["goal_id"] for item in joined["begin_motion"]] == ["a" * 32, "c" * 32]
    assert joined["release"]["run_id"] == 6


def test_preempt_recovery_allows_initial_a_permit_before_fresh_c_permit(inputs):
    timeline, raw, action, context, records = recovery_inputs(inputs)
    request = next(item for item in records if item[0] == 4 and
                   joiner._decode("DummyManageV2Request", item[39:])["operation"] == 5)
    response = next(item for item in records if item[0] == 5 and
                    joiner._decode("DummyManageV2Response", item[39:])["request_id"] == 5)
    initial_request = wire("DummyManageV2Request", **{
        **joiner._decode("DummyManageV2Request", request[39:]),
        "request_id": 9, "request_seq": 9, "expected_motion_epoch": 2})
    initial_response = wire("DummyManageV2Response", **{
        **joiner._decode("DummyManageV2Response", response[39:]),
        "request_id": 9, "operation_id": 9, "motion_epoch": 2})
    records.extend((record(4, 500, 2, 3, 0, 9, initial_request),
                    record(5, 600, 2, 3, 0, 0, initial_response)))
    records.sort(key=lambda item: struct.unpack_from("<Q", item, 3)[0])
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    assert [goal["action_status"] for goal in
            joiner.join_evidence(timeline, raw, action, context)["goals"]] == [
                "CANCELED", "SUCCEEDED"]


def test_preempt_recovery_rejects_second_permit_in_a_to_c_window(inputs):
    timeline, raw, action, context, records = recovery_inputs(inputs)
    request = next(item for item in records if item[0] == 4 and
                   joiner._decode("DummyManageV2Request", item[39:])["operation"] == 5)
    second = wire("DummyManageV2Request", **{
        **joiner._decode("DummyManageV2Request", request[39:]),
        "request_id": 9, "request_seq": 9})
    records.append(record(4, 7700, 2, 3, 0, 9, second))
    records.sort(key=lambda item: struct.unpack_from("<Q", item, 3)[0])
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="missing or extra recovery management operation"):
        joiner.join_evidence(timeline, raw, action, context)


def test_preempt_recovery_accepts_bounded_get_operation_retry_for_c_begin(inputs):
    timeline, raw, action, context, records = recovery_inputs(inputs)
    query = wire("DummyManageV2Request", device_id="22" * 16,
        client_instance_id="11" * 16, request_id=8, request_seq=8,
        operation=12, queried_operation_id=6, expected_boot_epoch=2,
        expected_session_id=3, expected_reference_epoch=1,
        expected_motion_epoch=3, expected_profile_id=201, configured_mask=63)
    changed = []
    for item in records:
        kind, _, when, boot, session, run, seq = struct.unpack_from(
            "<B H Q Q Q Q I", item)
        payload = item[39:]
        if kind == 5 and when == 9800:
            response = joiner._decode("DummyManageV2Response", payload)
            payload = wire("DummyManageV2Response", **{
                **response, "request_id": 8, "operation_id": 6})
            item = record(kind, when, boot, session, run, seq, payload)
        changed.append(item)
    changed.extend((record(4, 9400, 2, 3, 0, 8, query),
                    record(4, 9500, 2, 3, 0, 8, query)))
    changed.sort(key=lambda item: struct.unpack_from("<Q", item, 3)[0])
    raw.write_bytes(b"DPRAW01\0" + b"".join(changed))
    repin(timeline, raw, action, context)
    joined = joiner.join_evidence(timeline, raw, action, context)
    assert joined["begin_motion"][1]["request_id"] == 6
    assert joined["begin_motion"][1]["operation_id"] == 6


def test_preempt_recovery_accepts_queued_then_committed_receipts(inputs):
    timeline, raw, action, context, records = recovery_inputs(inputs)
    additions = []
    for request_id, at in ((6, 9150), (5, 7520)):
        request = next(item for item in records if item[0] == 4 and
                       joiner._decode("DummyManageV2Request", item[39:])[
                           "request_id"] == request_id)
        fields = joiner._decode("DummyManageV2Request", request[39:])
        response = wire("DummyManageV2Response", device_id="22" * 16,
            client_instance_id="11" * 16, request_id=request_id,
            operation_id=0, receipt=1, transaction_phase=1,
            result_code=23, boot_epoch=2, session_id=3,
            profile_id=201, run_id=0,
            motion_epoch=fields["expected_motion_epoch"])
        additions.append(record(5, at, 2, 3, 0, 0, response))
    records.extend(additions)
    records.sort(key=lambda item: struct.unpack_from("<Q", item, 3)[0])
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    assert len(joiner.join_evidence(timeline, raw, action, context)["begin_motion"]) == 2


def test_preempt_recovery_rejects_wrong_boot_queued_receipt(inputs):
    timeline, raw, action, context, records = recovery_inputs(inputs)
    response = wire("DummyManageV2Response", device_id="22" * 16,
        client_instance_id="11" * 16, request_id=6,
        operation_id=0, receipt=1, transaction_phase=1,
        result_code=23, boot_epoch=99, session_id=3,
        profile_id=201, run_id=0, motion_epoch=3)
    records.append(record(5, 9150, 99, 3, 0, 0, response))
    records.sort(key=lambda item: struct.unpack_from("<Q", item, 3)[0])
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="interim receipt"):
        joiner.join_evidence(timeline, raw, action, context)


def test_preempt_recovery_rejects_other_device_begin_request(inputs):
    timeline, raw, action, context, records = recovery_inputs(inputs)
    changed = []
    for item in records:
        kind, _, when, boot, session, run, seq = struct.unpack_from("<B H Q Q Q Q I", item)
        payload = item[39:]
        if kind == 4:
            fields = joiner._decode("DummyManageV2Request", payload)
            if fields["request_id"] == 6:
                payload = wire("DummyManageV2Request", **{
                    **fields, "device_id": "33" * 16})
        elif kind == 5:
            fields = joiner._decode("DummyManageV2Response", payload)
            if fields["request_id"] == 6:
                payload = wire("DummyManageV2Response", **{
                    **fields, "device_id": "33" * 16})
        changed.append(record(kind, when, boot, session, run, seq, payload))
    raw.write_bytes(b"DPRAW01\0" + b"".join(changed))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="BeginMotion request does not bind"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("mutation", [
    "b_accepted", "missing_c", "duplicate_uuid", "wrong_c_status",
    "stale_c_epoch", "missing_permit", "early_permit", "pending_permit",
    "extra_begin", "duplicate_begin_request", "wrong_c_begin_epoch",
    "missing_complete", "extra_release",
])
def test_preempt_recovery_rejects_unproven_second_motion(inputs, mutation):
    timeline, raw, action, context, records = recovery_inputs(inputs)
    report = json.loads(action.read_text())
    rows = list(csv.reader(timeline.open(newline="")))
    parsed = [(struct.unpack_from("<B H Q Q Q Q I", item), item[39:])
              for item in records]
    if mutation == "b_accepted":
        report["preempt_goals"]["B"]["status"] = "ACCEPTED"
    elif mutation == "missing_c":
        report["goals"].pop()
    elif mutation == "duplicate_uuid":
        report["preempt_goals"]["B"]["goal_id"] = report["goals"][1]["goal_id"]
    elif mutation == "wrong_c_status":
        report["preempt_goals"]["C"]["status"] = "NOT_SUBMITTED"
    elif mutation == "stale_c_epoch":
        for row in rows[8:14]:
            if row[0] != "post_release_status":
                row[7] = "3"
    elif mutation == "missing_permit":
        parsed = [(header, payload) for header, payload in parsed if not
                  (header[0] == 4 and joiner._decode("DummyManageV2Request", payload)[
                      "operation"] == 5)]
    elif mutation == "early_permit":
        parsed = [((kind, size, 5800 if kind == 4 and
                    joiner._decode("DummyManageV2Request", payload)["operation"] == 5
                    else when, boot, session, run, seq), payload)
                  for (kind, size, when, boot, session, run, seq), payload in parsed]
    elif mutation == "pending_permit":
        parsed = [(header, wire("DummyManageV2Response", **{
            **joiner._decode("DummyManageV2Response", payload),
            "transaction_phase": 2}) if header[0] == 5 and
            joiner._decode("DummyManageV2Response", payload)["request_id"] == 5
            else payload) for header, payload in parsed]
    elif mutation == "extra_begin":
        request = next(payload for header, payload in parsed if header[0] == 4 and
                       joiner._decode("DummyManageV2Request", payload)["request_id"] == 6)
        duplicate = wire("DummyManageV2Request", **{
            **joiner._decode("DummyManageV2Request", request),
            "request_id": 8, "request_seq": 8})
        parsed.append(((4, len(duplicate), 9200, 2, 3, 0, 8), duplicate))
    elif mutation == "duplicate_begin_request":
        request = next(payload for header, payload in parsed if header[0] == 4 and
                       joiner._decode("DummyManageV2Request", payload)["request_id"] == 6)
        parsed.append(((4, len(request), 9200, 2, 3, 0, 6), request))
    elif mutation == "wrong_c_begin_epoch":
        parsed = [(header, wire("DummyManageV2Response", **{
            **joiner._decode("DummyManageV2Response", payload),
            "motion_epoch": 3}) if header[0] == 5 and
            joiner._decode("DummyManageV2Response", payload)["request_id"] == 6
            else payload) for header, payload in parsed]
    elif mutation == "missing_complete":
        parsed = [(header, payload) for header, payload in parsed if not
                  (header[0] == 4 and joiner._decode("DummyManageV2Request", payload)[
                      "operation"] == 8)]
    else:
        request = next(payload for header, payload in parsed if header[0] == 4 and
                       joiner._decode("DummyManageV2Request", payload)["operation"] == 10)
        duplicate = wire("DummyManageV2Request", **{
            **joiner._decode("DummyManageV2Request", request),
            "request_id": 8, "request_seq": 8})
        parsed.append(((4, len(duplicate), 13200, 2, 3, 0, 8), duplicate))
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    action.write_text(json.dumps(report))
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        record(kind, when, boot, session, run, seq, payload)
        for (kind, _, when, boot, session, run, seq), payload in
        sorted(parsed, key=lambda item: item[0][2])))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("mutation", [
    "same_uuid", "b_accepted", "c_submitted", "overlap_mismatch",
    "missing_begin", "extra_begin", "pending_begin", "wrong_begin_epoch",
    "wrong_begin_client", "extra_begin_receipt", "extra_motion_run",
    "extra_release_run", "extra_motion_status",
])
def test_preempt_probe_fails_closed_on_ambiguous_overlap_or_motion(inputs, mutation):
    timeline, raw, action, context, records = preempt_inputs(inputs)
    report = json.loads(action.read_text())
    if mutation == "same_uuid":
        report["preempt_goals"]["B"]["goal_id"] = report["goal_id"]
    elif mutation == "b_accepted":
        report["preempt_goals"]["B"]["status"] = "ACCEPTED"
    elif mutation == "c_submitted":
        report["preempt_goals"]["C"]["status"] = "REJECTED"
    elif mutation == "overlap_mismatch":
        report["cancel"]["overlap_probe"]["goal_id"] = "e" * 32
    elif mutation == "missing_begin":
        records.pop(0)
    elif mutation == "extra_begin":
        records.insert(1, record(4, 2150, 2, 3, 0, 5,
            wire("DummyManageV2Request", device_id="22" * 16,
                 client_instance_id="11" * 16, request_id=5, request_seq=5,
                 operation=7, expected_boot_epoch=2, expected_session_id=3,
                 expected_reference_epoch=1, expected_motion_epoch=2,
                 expected_profile_id=201, configured_mask=63)))
    elif mutation in ("pending_begin", "wrong_begin_epoch", "wrong_begin_client"):
        index = 2 if mutation == "pending_begin" else 0
        kind = 5 if mutation == "pending_begin" else 4
        fields = joiner._decode("DummyManageV2Response" if kind == 5 else
                                "DummyManageV2Request", records[index][39:])
        if mutation == "pending_begin":
            fields["transaction_phase"] = 2
        elif mutation == "wrong_begin_epoch":
            fields["expected_motion_epoch"] = 1
        else:
            fields["client_instance_id"] = "33" * 16
        records[index] = record(kind, 2800 if kind == 5 else 2100,
                                2, 3, 0, 0 if kind == 5 else 3,
                                wire("DummyManageV2Response" if kind == 5 else
                                     "DummyManageV2Request", **fields))
    elif mutation == "extra_begin_receipt":
        records.insert(4, record(5, 3100, 2, 3, 0, 0, records[2][39:]))
    elif mutation == "extra_motion_status":
        fields = joiner._decode("DummyDeviceStatusV2", records[3][39:])
        fields["run_id"] = 6
        records.append(record(3, 5700, 2, 3, 0, 0,
                              wire("DummyDeviceStatusV2", **fields)))
    elif mutation in ("extra_motion_run", "extra_release_run"):
        run = 6 if mutation == "extra_motion_run" else 7
        identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
            ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
        extra = page(identity, run, 2, 0, b"", epoch=3, category=1,
                     release=mutation == "extra_release_run")
        records.insert(-2, record(1, 6800, 2, 3, run, 0, extra))
    action.write_text(json.dumps(report))
    records.sort(key=lambda item: struct.unpack_from("<Q", item, 3)[0])
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError):
        joiner.join_evidence(timeline, raw, action, context)


def test_unconfirmed_cancel_claim_cannot_wrap_two_successful_goals(inputs):
    timeline, raw, action, context, _ = two_goal_inputs(inputs)
    report = unconfirmed_cancel_report(action)
    report.update(action_status=5, result_error_code=0)
    action.write_text(json.dumps(report))
    repin(timeline, raw, action, context)

    with pytest.raises(ValueError, match="unconfirmed final ROS release"):
        joiner.join_evidence(timeline, raw, action, context)


def test_cancel_release_idle_must_bind_raw_terminal_chain(inputs):
    timeline, raw, action, context, records = canceled_inputs(inputs)
    unconfirmed_cancel_report(action)
    idle = joiner._decode("DummyDeviceStatusV2", records[14][39:])
    idle["evidence_chain_sha256"] = "00" * 32
    records[14] = record(3, 7000, 2, 0, 0, 0,
                         wire("DummyDeviceStatusV2", **idle))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)

    with pytest.raises(ValueError, match="terminal chain"):
        joiner.join_evidence(timeline, raw, action, context)


def test_zero_idle_chain_does_not_replace_release_committed_receipt(inputs):
    timeline, raw, action, context, records = canceled_inputs(inputs)
    unconfirmed_cancel_report(action)
    idle = joiner._decode("DummyDeviceStatusV2", records[14][39:])
    idle["evidence_chain_sha256"] = "00" * 32
    records[14] = record(3, 7000, 2, 0, 0, 0,
                         wire("DummyDeviceStatusV2", **idle))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records[:13] + records[14:]))
    repin(timeline, raw, action, context)

    with pytest.raises(ValueError, match="committed release receipt"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("field,value", [
    ("action_observation", "SUCCEEDED"),
    ("stop_confirmation", "CONFIRMED_RELEASED_IDLE"),
    ("safety_verdict", "QUALIFIED"),
    ("raw_stop_release_join", "CONFIRMED"),
    ("release_observation", {"request_reply": "RELEASE_QUEUED",
                             "bootstrap_reply": "OBSERVATION_UNAVAILABLE"}),
])
def test_unconfirmed_cancel_diagnostic_claim_must_stay_unqualified(
        inputs, field, value):
    timeline, raw, action, context, _ = canceled_inputs(inputs)
    report = unconfirmed_cancel_report(action)
    report[field] = value
    action.write_text(json.dumps(report))
    repin(timeline, raw, action, context)

    with pytest.raises(ValueError):
        joiner.join_evidence(timeline, raw, action, context)


def canceled_query_inputs(inputs):
    timeline, raw, action, context, records = canceled_inputs(inputs)
    rows = list(csv.reader(timeline.open(newline="")))
    rows[5][rows[0].index("cancel_operation_id")] = "7"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    request = joiner._decode("DummyManageV2Request", records[4][39:])
    response = joiner._decode("DummyManageV2Response", records[7][39:])
    query = {**request, "operation": 12, "request_id": 3,
             "request_seq": 3, "queried_operation_id": request["request_id"]}
    reply = {**response, "request_id": 3, "operation_id": 7}
    records[7:8] = [record(4, 4750, 2, 3, 0, 3,
                           wire("DummyManageV2Request", **query)),
                    record(5, 4800, 2, 3, 4, 0,
                           wire("DummyManageV2Response", **reply))]
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    return timeline, raw, action, context, records


def test_canceled_goal_accepts_strict_get_operation_commit(inputs):
    timeline, raw, action, context, _ = canceled_query_inputs(inputs)
    joined = joiner.join_evidence(timeline, raw, action, context)
    assert joined["goals"][0]["cancel_request_id"] == 1
    assert joined["goals"][0]["cancel_operation_id"] == 7


def test_canceled_query_rejects_wrong_committed_operation_id(inputs):
    timeline, raw, action, context, records = canceled_query_inputs(inputs)
    response = joiner._decode("DummyManageV2Response", records[8][39:])
    response["operation_id"] = 1
    records[8] = record(5, 4800, 2, 3, 4, 0,
                        wire("DummyManageV2Response", **response))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="committed current-run receipt"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("field,value", [
    ("queried_operation_id", 2), ("expected_motion_epoch", 99),
    ("expected_boot_epoch", 3), ("client_instance_id", "33" * 16),
])
def test_canceled_query_rejects_wrong_original_binding(inputs, field, value):
    timeline, raw, action, context, records = canceled_query_inputs(inputs)
    query = joiner._decode("DummyManageV2Request", records[7][39:])
    query[field] = value
    records[7] = record(4, 4750, query["expected_boot_epoch"],
                        query["expected_session_id"], 0, 3,
                        wire("DummyManageV2Request", **query))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError):
        joiner.join_evidence(timeline, raw, action, context)


def test_joiner_release_rejects_motion_complete_on_release_receipt(inputs):
    timeline, raw, action, context, records = canceled_inputs(inputs)
    response = joiner._decode("DummyManageV2Response", records[13][39:])
    response["motion_complete"] = 1
    records[13] = record(5, 6600, 2, 0, 5, 0,
                         wire("DummyManageV2Response", **response))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="committed release receipt"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("mutation", [
    "pending_cancel", "wrong_cancel_run", "old_cancel_receipt",
    "cancel_motion_complete", "cancel_release_committed",
    "missing_cancel_request", "late_cancel_request", "wrong_cancel_client",
    "wrong_cancel_boot", "wrong_cancel_session",
    "duplicate_cancel_request", "missing_stop_axis", "missing_stop_page",
    "missing_cancel_terminal", "automatic_release", "missing_release_terminal",
    "canceled_action_only", "released_idle_only", "unobserved_cancel",
    "wrong_action_goal", "no_cancel_ack", "old_timeline_cancel_key",
    "old_timeline_cancel_operation", "missing_timeline_cancel_columns",
])
def test_canceled_goal_rejects_incomplete_or_stale_raw_chain(inputs, mutation):
    timeline, raw, action_path, context, records = canceled_inputs(inputs)
    if mutation in ("pending_cancel", "wrong_cancel_run", "old_cancel_receipt",
                    "cancel_motion_complete", "cancel_release_committed"):
        fields = joiner._decode("DummyManageV2Response", records[7][39:])
        if mutation == "pending_cancel":
            fields["transaction_phase"] = 2
        elif mutation == "wrong_cancel_run":
            fields["run_id"] = 5
        elif mutation == "cancel_motion_complete":
            fields["motion_complete"] = 1
        elif mutation == "cancel_release_committed":
            fields["release_committed"] = 1
        else:
            fields["request_id"] = 99
        records[7] = record(5, 4800, 2, 3, fields["run_id"], 0,
                            wire("DummyManageV2Response", **fields))
    elif mutation == "duplicate_cancel_request":
        fields = joiner._decode("DummyManageV2Request", records[4][39:])
        fields.update(request_seq=2)
        records.insert(5, record(4, 4300, 2, 3, 0, 2,
                                 wire("DummyManageV2Request", **fields)))
    elif mutation == "missing_cancel_request":
        records.pop(4)
    elif mutation == "late_cancel_request":
        request = records.pop(4)
        records.insert(5, record(4, 4600, 2, 3, 0, 1, request[39:]))
    elif mutation in ("wrong_cancel_client", "wrong_cancel_boot",
                      "wrong_cancel_session"):
        fields = joiner._decode("DummyManageV2Request", records[4][39:])
        if mutation == "wrong_cancel_client":
            fields["client_instance_id"] = "33" * 16
        elif mutation == "wrong_cancel_boot":
            fields["expected_boot_epoch"] = 3
        else:
            fields["expected_session_id"] = 4
        records[4] = record(4, 4200, fields["expected_boot_epoch"],
                            fields["expected_session_id"], 0, 1,
                            wire("DummyManageV2Request", **fields))
    elif mutation == "missing_stop_axis":
        stop = bytearray(records[5][39:])
        stop[35] = 1
        records[5] = record(1, 4500, 2, 3, 4, 1, stop)
    elif mutation == "missing_stop_page":
        records.pop(5)
    elif mutation == "missing_cancel_terminal":
        records.pop(6)
    elif mutation == "automatic_release":
        records.pop(10)
    elif mutation == "missing_release_terminal":
        records.pop(12)
    elif mutation == "canceled_action_only":
        records[:] = records[:4]
    elif mutation == "released_idle_only":
        action = json.loads(action_path.read_text())
        action["status"], action["action_status"] = "SUCCEEDED", 4
        action_path.write_text(json.dumps(action))
    elif mutation in ("old_timeline_cancel_key", "old_timeline_cancel_operation",
                      "missing_timeline_cancel_columns"):
        rows = list(csv.reader(timeline.open(newline="")))
        if mutation == "missing_timeline_cancel_columns":
            rows[0] = rows[0][:-2]
            for row in rows[1:-1]:
                del row[-2:]
        else:
            rows[5][-2 if mutation == "old_timeline_cancel_key" else -1] = "99"
        with timeline.open("w", newline="") as output:
            csv.writer(output).writerows(rows)
    else:
        action = json.loads(action_path.read_text())
        if mutation == "unobserved_cancel":
            action["cancel"]["motion_delta_rad"] = 0.02
        elif mutation == "wrong_action_goal":
            action["cancel"]["goal_id"] = "f" * 32
        else:
            action["cancel"]["acknowledged"] = False
        action_path.write_text(json.dumps(action))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action_path, context)
    with pytest.raises(ValueError):
        joiner.join_evidence(timeline, raw, action_path, context)


def two_goal_inputs(inputs):
    timeline, raw, action_path, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    second = []
    for index, event in enumerate(("accepted", "before_begin", "after_begin",
                                   "feedback", "terminal_success", "pre_release_hold",
                                   "post_release_status")):
        original = rows[index + 1].copy()
        original[:4] = [event, "c" * 32, "d" * 64, str(8000 + index * 1000)]
        original[7:9] = ["3", "5"]
        if event == "post_release_status":
            original[5:9] = ["0", "0", "4", "6"]
            original[-2:] = ["3", "3"]
        second.append(original)
    with timeline.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(rows[0])
        writer.writerows(rows[1:7] + second)
        writer.writerow(["COMPLETE", "2"])

    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    old_status = joiner._decode("DummyDeviceStatusV2", records[0][39:])
    old_state = joiner._decode("DummyJointStateV2", records[1][39:])
    new_status = {**old_status, "motion_epoch": 3, "run_id": 5}
    new_state = {**old_state, "motion_epoch": 3}
    first = page(identity, 5, 2, 0, b"", epoch=3)
    terminal = page(identity, 5, 2, 1, first[339:371], epoch=3)
    release_first = page(identity, 6, 2, 0, b"", release=True, epoch=4,
                         parent=5, category=3)
    release_last = page(identity, 6, 2, 1, release_first[339:371],
                        release=True, epoch=3, parent=5)
    suffix = [
        record(3, 9900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **new_status)),
        record(2, 10000, 2, 3, 0, 0, wire("DummyJointStateV2", **new_state)),
        record(1, 10500, 2, 3, 5, 0, first),
        record(2, 11000, 2, 3, 0, 0, wire("DummyJointStateV2", **new_state)),
        record(1, 11500, 2, 3, 5, 1, terminal),
        record(3, 12900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **{
            **new_status, "core_state": 4, "final_state": 6, "motion_complete": 1,
            "attempt_kind": 1, "evidence_chain_sha256": terminal[339:371].hex()})),
        record(2, 13000, 2, 3, 0, 0, wire("DummyJointStateV2", **{
            **new_state, "core_state": 4, "hold_ready": 1})),
        record(1, 13400, 2, 3, 6, 0, release_first),
        record(1, 13500, 2, 3, 6, 1, release_last),
        record(3, 14000, 2, 0, 0, 0, wire("DummyDeviceStatusV2", **{
            **new_status, "session_id": 0, "reference_epoch": 0,
            "motion_epoch": 4, "run_id": 6, "core_state": 1,
            "attempt_kind": 3, "final_state": 8, "release_committed": 1,
            "evidence_chain_sha256": release_last[339:371].hex()})),
        record(2, 14000, 2, 0, 0, 0, wire("DummyJointStateV2", **{
            **new_state, "session_id": 0, "reference_epoch": 0,
            "motion_epoch": 4, "core_state": 1})),
    ]
    all_records = records[:7] + suffix
    raw.write_bytes(b"DPRAW01\0" + b"".join(all_records))
    first_goal = json.loads(action_path.read_text())
    first_goal["index"] = 0
    first_goal["stop_confirmation"] = "NOT_REQUESTED"
    second_goal = {**first_goal, "index": 1, "goal_id": "c" * 32,
                   "trajectory_sha256": "d" * 64,
                   "action_started_steady_us": 7500}
    action_path.write_text(json.dumps({"scope": "ROS_ACTION_SESSION_ONLY",
        "status": "SUCCEEDED", "requested_goals": 2, "completed_goals": 2,
        "goals": [first_goal, second_goal],
        "stop_confirmation": "CONFIRMED_RELEASED_IDLE"}))
    repin(timeline, raw, action_path, context)
    return timeline, raw, action_path, context, all_records


def fifty_goal_inputs(inputs, *, states_per_goal=830):
    timeline, raw, action_path, context, original = inputs
    source_rows = list(csv.reader(timeline.open(newline="")))
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    begin_status = joiner._decode("DummyDeviceStatusV2", original[0][39:])
    begin_state = joiner._decode("DummyJointStateV2", original[1][39:])
    hold_status = joiner._decode("DummyDeviceStatusV2", original[5][39:])
    hold_state = joiner._decode("DummyJointStateV2", original[6][39:])
    rows = []
    timed_records = []
    actions = []
    for index in range(50):
        base = index * 7000
        run = index + 4
        epoch = index + 2
        goal_id = f"{index + 1:032x}"
        digest = f"{index + 1:064x}"
        for source in source_rows[1:7]:
            row = source.copy()
            row[1:4] = [goal_id, digest, str(int(source[3]) + base)]
            row[7:9] = [str(epoch), str(run)]
            rows.append(row)
        first = page(identity, run, 2, 0, b"", epoch=epoch)
        terminal = page(identity, run, 2, 1, first[339:371], epoch=epoch)
        entries = [
            (base + 2900, record(3, base + 2900, 2, 3, 0, 0,
                wire("DummyDeviceStatusV2", **{**begin_status,
                    "motion_epoch": epoch, "run_id": run}))),
            (base + 3000, record(2, base + 3000, 2, 3, 0, 0,
                wire("DummyJointStateV2", **{**begin_state, "motion_epoch": epoch}))),
            (base + 3500, record(1, base + 3500, 2, 3, run, 0, first)),
            (base + 4000, record(2, base + 4000, 2, 3, 0, 0,
                wire("DummyJointStateV2", **{**begin_state, "motion_epoch": epoch}))),
            (base + 4500, record(1, base + 4500, 2, 3, run, 1, terminal)),
            (base + 5900, record(3, base + 5900, 2, 3, 0, 0,
                wire("DummyDeviceStatusV2", **{**hold_status,
                    "motion_epoch": epoch, "run_id": run,
                    "evidence_chain_sha256": terminal[339:371].hex()}))),
            (base + 6000, record(2, base + 6000, 2, 3, 0, 0,
                wire("DummyJointStateV2", **{**hold_state, "motion_epoch": epoch}))),
        ]
        state_payload = wire("DummyJointStateV2", **{**begin_state,
            "motion_epoch": epoch})
        for extra in range(states_per_goal):
            when = base + (4001 + extra if extra < 415 else 4600 + extra - 415)
            entries.append((when, record(2, when, 2, 3, 0, 0, state_payload)))
        timed_records.extend(entries)
        actions.append({**json.loads(action_path.read_text()),
            "index": index, "goal_id": goal_id, "trajectory_sha256": digest,
            "action_started_steady_us": base + 500,
            "stop_confirmation": "NOT_REQUESTED"})
    release_run = 54
    last_epoch = 51
    release_first = page(identity, release_run, 2, 0, b"", release=True,
                         epoch=last_epoch + 1, parent=53, category=3)
    release_last = page(identity, release_run, 2, 1, release_first[339:371],
                        release=True, epoch=last_epoch, parent=53)
    release_base = 49 * 7000
    for offset, seq, payload in ((6400, 0, release_first), (6500, 1, release_last)):
        timed_records.append((release_base + offset, record(1, release_base + offset,
            2, 3, release_run, seq, payload)))
    idle_status = joiner._decode("DummyDeviceStatusV2", original[9][39:])
    idle_state = joiner._decode("DummyJointStateV2", original[10][39:])
    timed_records.extend((
        (release_base + 7000, record(3, release_base + 7000, 2, 0, 0, 0,
            wire("DummyDeviceStatusV2", **{**idle_status,
                "motion_epoch": last_epoch + 1, "run_id": release_run,
                "evidence_chain_sha256": release_last[339:371].hex()}))),
        (release_base + 7000, record(2, release_base + 7000, 2, 0, 0, 0,
            wire("DummyJointStateV2", **{**idle_state,
                "motion_epoch": last_epoch + 1}))),
    ))
    release_row = source_rows[7].copy()
    release_row[1:4] = [actions[-1]["goal_id"], actions[-1]["trajectory_sha256"],
                        str(release_base + 7000)]
    release_row[7:9] = [str(last_epoch + 1), str(release_run)]
    release_row[-2:] = ["51", "51"]
    rows.append(release_row)
    with timeline.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(source_rows[0])
        writer.writerows(rows)
        writer.writerow(["COMPLETE", "50"])
    raw.write_bytes(b"DPRAW01\0" + b"".join(payload for _, payload in
                    sorted(timed_records, key=lambda item: item[0])))
    action_path.write_text(json.dumps({"scope": "ROS_ACTION_SESSION_ONLY",
        "status": "SUCCEEDED", "requested_goals": 50, "completed_goals": 50,
        "goals": actions, "stop_confirmation": "CONFIRMED_RELEASED_IDLE"}))
    repin(timeline, raw, action_path, context)
    return timeline, raw, action_path, context


def test_synthetic_fifty_goal_capture_joins_with_bounded_runtime(inputs):
    timeline, raw, action, context = fifty_goal_inputs(inputs)
    started = time.perf_counter()
    result = joiner.join_evidence(timeline, raw, action, context)
    elapsed = time.perf_counter() - started
    assert len(result["goals"]) == 50
    assert result["release"]["parent_run_id"] == 53
    assert elapsed < 30.0


def test_synthetic_fifty_goal_capture_rejects_missing_final_terminal(inputs):
    timeline, raw, action, context = fifty_goal_inputs(inputs, states_per_goal=0)
    source = raw.read_bytes()
    records = []
    offset = 8
    while offset < len(source):
        kind, size, when, boot, session, run, seq = joiner.RECORD_HEADER.unpack_from(
            source, offset)
        end = offset + joiner.RECORD_HEADER.size + size
        if not (kind == 1 and run == 53 and seq == 1):
            records.append(source[offset:end])
        offset = end
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="terminal"):
        joiner.join_evidence(timeline, raw, action, context)


def mixed_epoch_inputs(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    for row in rows[1:3]:
        row[7] = "1"
    for row in rows[5:7]:
        row[7] = "3"
    rows[7][7] = "4"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    status = joiner._decode("DummyDeviceStatusV2", records[0][39:])
    state = joiner._decode("DummyJointStateV2", records[1][39:])
    prefix = page(identity, 4, 2, 0, b"", epoch=1, group_count=3, group_id=6)
    goal = page(identity, 4, 2, 1, prefix[339:371], epoch=2,
                group_count=3, group_id=7)
    hold = page(identity, 4, 2, 2, goal[339:371], epoch=3,
                group_count=3, group_id=8)
    terminal = page(identity, 4, 2, 3, hold[339:371], epoch=1, group_count=3)
    hold_status = {**status, "motion_epoch": 3, "core_state": 4,
                   "final_state": 6, "motion_complete": 1, "attempt_kind": 1,
                   "evidence_chain_sha256": terminal[339:371].hex()}
    hold_state = {**state, "motion_epoch": 3, "core_state": 4,
                  "hold_ready": 1}
    release_first = page(identity, 5, 2, 0, b"", release=True, epoch=4,
                         category=3)
    release_last = page(identity, 5, 2, 1, release_first[339:371],
                        release=True, epoch=3)
    records = [
        record(3, 1900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **{
            **status, "motion_epoch": 1})),
        record(1, 1950, 2, 3, 4, 0, prefix),
        record(2, 2000, 2, 3, 0, 0, wire("DummyJointStateV2", **{
            **state, "motion_epoch": 1})),
        records[0],
        record(1, 2950, 2, 3, 4, 1, goal),
        records[1], records[3],
        record(1, 5200, 2, 3, 4, 2, hold),
        record(1, 5500, 2, 3, 4, 3, terminal),
        record(3, 5900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **hold_status)),
        record(2, 6000, 2, 3, 0, 0, wire("DummyJointStateV2", **hold_state)),
        record(1, 6400, 2, 3, 5, 0, release_first),
        record(1, 6500, 2, 3, 5, 1, release_last),
        record(3, 7000, 2, 0, 0, 0, wire("DummyDeviceStatusV2", **{
            **joiner._decode("DummyDeviceStatusV2", records[9][39:]),
            "motion_epoch": 4,
            "evidence_chain_sha256": release_last[339:371].hex()})),
        record(2, 7000, 2, 0, 0, 0, wire("DummyJointStateV2", **{
            **joiner._decode("DummyJointStateV2", records[10][39:]),
            "motion_epoch": 4})),
    ]
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    return timeline, raw, action, context, records


def active_release_inputs(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[-2][7] = "3"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    hold = page(identity, 5, 2, 0, b"", active_release=True,
                group_count=2, group_id=8)
    disable = page(identity, 5, 2, 1, hold[339:371], active_release=True,
                   epoch=3, group_count=2, group_id=9, category=3)
    terminal = page(identity, 5, 2, 2, disable[339:371], active_release=True,
                    group_count=2)
    idle = joiner._decode("DummyDeviceStatusV2", records[9][39:])
    idle.update(attempt_kind=1, final_state=2, motion_epoch=3,
                evidence_chain_sha256=terminal[339:371].hex())
    idle_state = joiner._decode("DummyJointStateV2", records[10][39:])
    idle_state["motion_epoch"] = 3
    def manage_pair(operation, request_id, request_time, response_time, run):
        request = wire("DummyManageV2Request", device_id="22" * 16,
            client_instance_id=identity["client_id"], request_id=request_id,
            request_seq=request_id, operation=operation,
            expected_boot_epoch=2, expected_session_id=3, expected_profile_id=201)
        response = wire("DummyManageV2Response", device_id="22" * 16,
            client_instance_id=identity["client_id"], request_id=request_id,
            operation_id=request_id, receipt=1,
            transaction_phase=3, result_code=0,
            boot_epoch=2, session_id=0 if operation == 10 else 3,
            profile_id=201, run_id=run,
            motion_epoch=3 if operation == 10 else 0,
            motion_complete=int(operation == 8),
            release_committed=int(operation == 10))
        return (record(4, request_time, 2, 3, 0, request_id, request),
                record(5, response_time, 2, 0 if operation == 10 else 3,
                       run, 0, response))
    complete_request, complete_response = manage_pair(8, 1, 4400, 4600, 4)
    release_request, release_response = manage_pair(10, 2, 6150, 6600, 5)
    changed = [*records[:4], complete_request, records[4], complete_response,
               records[5], records[6], release_request,
               record(1, 6200, 2, 3, 5, 0, hold),
               record(1, 6300, 2, 3, 5, 1, disable),
               record(1, 6500, 2, 3, 5, 2, terminal), release_response,
               record(3, 7000, 2, 0, 0, 0, wire("DummyDeviceStatusV2", **idle)),
               record(2, 7000, 2, 0, 0, 0,
                      wire("DummyJointStateV2", **idle_state))]
    raw.write_bytes(b"DPRAW01\0" + b"".join(changed))
    repin(timeline, raw, action, context)
    return timeline, raw, action, context, changed


def test_active_hold_refresh_release_joins_raw_management_and_can(inputs):
    timeline, raw, action, context, _ = active_release_inputs(inputs)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["release"]["terminal"] == "CANCELED_AND_RELEASED"
    assert result["release"]["run_id"] == 5
    assert [item["kind"] for item in result["management_records"]] == [4, 5, 4, 5]


@pytest.mark.parametrize("change", ("wrong_client", "wrong_boot", "wrong_session",
                                   "old_request", "conflicting_receipt"))
def test_release_requires_current_request_and_unambiguous_final_receipt(inputs, change):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    if change == "conflicting_receipt":
        response = joiner._decode("DummyManageV2Response", records[13][39:])
        response["operation_id"] = 99
        records.insert(13, record(5, 6550, 2, 0, 5, 0,
                                  wire("DummyManageV2Response", **response)))
    else:
        request = joiner._decode("DummyManageV2Request", records[9][39:])
        if change == "wrong_client":
            request["client_instance_id"] = "33" * 16
        elif change == "wrong_boot":
            request["expected_boot_epoch"] = 9
        elif change == "wrong_session":
            request["expected_session_id"] = 9
        request_time = 100 if change == "old_request" else 6150
        records[9] = record(4, request_time,
                            request["expected_boot_epoch"],
                            request["expected_session_id"], 0,
                            request["request_seq"],
                            wire("DummyManageV2Request", **request))
        if change == "old_request":
            records.insert(0, records.pop(9))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="release request|release receipt|management receipt|management operation identity"):
        joiner.join_evidence(timeline, raw, action, context)


def test_active_release_joins_safety_deceleration_before_disable(inputs):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    hold = page(identity, 5, 2, 0, b"", active_release=True,
                group_count=3, group_id=8)
    safety = page(identity, 5, 2, 1, hold[339:371], active_release=True,
                  epoch=3, group_count=3, group_id=9, category=2)
    disable = page(identity, 5, 2, 2, safety[339:371], active_release=True,
                   epoch=3, group_count=3, group_id=10, category=3)
    terminal = page(identity, 5, 2, 3, disable[339:371], active_release=True,
                    group_count=3)
    records[10:13] = [record(1, 6200, 2, 3, 5, 0, hold),
                      record(1, 6250, 2, 3, 5, 1, safety),
                      record(1, 6300, 2, 3, 5, 2, disable),
                      record(1, 6500, 2, 3, 5, 3, terminal)]
    status = joiner._decode("DummyDeviceStatusV2", records[15][39:])
    status.update(motion_epoch=3, evidence_chain_sha256=terminal[339:371].hex())
    records[15] = record(3, 7000, 2, 0, 0, 0,
                         wire("DummyDeviceStatusV2", **status))
    state = joiner._decode("DummyJointStateV2", records[16][39:])
    state["motion_epoch"] = 3
    records[16] = record(2, 7000, 2, 0, 0, 0,
                         wire("DummyJointStateV2", **state))
    response = joiner._decode("DummyManageV2Response", records[14][39:])
    response["motion_epoch"] = 3
    records[14] = record(5, 6600, 2, 0, 5, 0,
                         wire("DummyManageV2Response", **response))
    rows = list(csv.reader(timeline.open(newline="")))
    rows[-2][7] = "3"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert [bytes.fromhex(page["raw_hex"])[32] for page in
            result["release"]["evidence_pages"]] == [1, 2, 3, 4]
    assert result["release"]["terminal"] == "CANCELED_AND_RELEASED"


def zero_active_idle_chain(inputs):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    status = joiner._decode("DummyDeviceStatusV2", records[14][39:])
    status["evidence_chain_sha256"] = "00" * 32
    records[14] = record(3, 7000, 2, 0, 0, 0,
                         wire("DummyDeviceStatusV2", **status))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    return timeline, raw, action, context, records


def test_active_release_accepts_cleared_idle_status_chain_with_raw_terminal(inputs):
    timeline, raw, action, context, _ = zero_active_idle_chain(inputs)
    assert joiner.join_evidence(timeline, raw, action, context)["release"]["run_id"] == 5


@pytest.mark.parametrize("mutation,match", [
    ("wrong_nonzero", "terminal chain"),
    ("zero_holding", "HOLD terminal chain"),
    ("bad_terminal_chain", "evidence chain"),
    ("missing_terminal", "terminal evidence"),
    ("wrong_receipt", "committed release receipt"),
    ("wrong_identity", "identity hash"),
])
def test_zero_idle_chain_keeps_other_release_proofs_mandatory(inputs, mutation, match):
    timeline, raw, action, context, records = zero_active_idle_chain(inputs)
    if mutation in ("wrong_nonzero", "wrong_identity"):
        status = joiner._decode("DummyDeviceStatusV2", records[14][39:])
        status["evidence_chain_sha256" if mutation == "wrong_nonzero" else
               "profile_sha256"] = "22" * 32
        records[14] = record(3, 7000, 2, 0, 0, 0,
                             wire("DummyDeviceStatusV2", **status))
    elif mutation == "zero_holding":
        status = joiner._decode("DummyDeviceStatusV2", records[7][39:])
        status["evidence_chain_sha256"] = "00" * 32
        records[7] = record(3, 5900, 2, 3, 0, 0,
                            wire("DummyDeviceStatusV2", **status))
    elif mutation == "bad_terminal_chain":
        terminal = bytearray(records[12][39:])
        terminal[339] ^= 1
        records[12] = record(1, 6500, 2, 3, 5, 2, terminal)
    elif mutation == "missing_terminal":
        records.pop(12)
    else:
        response = joiner._decode("DummyManageV2Response", records[13][39:])
        response["transaction_phase"] = 2
        records[13] = record(5, 6600, 2, 0, 5, 0,
                             wire("DummyManageV2Response", **response))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match=match):
        joiner.join_evidence(timeline, raw, action, context)


def test_independent_release_still_requires_status_chain_hash(inputs):
    timeline, raw, action, context, records = inputs
    status = joiner._decode("DummyDeviceStatusV2", records[9][39:])
    status["evidence_chain_sha256"] = "00" * 32
    records[9] = record(3, 7000, 2, 0, 0, 0,
                        wire("DummyDeviceStatusV2", **status))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="terminal chain"):
        joiner.join_evidence(timeline, raw, action, context)


def test_release_idle_preserves_only_committed_response_motion_epoch(inputs):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    hold = records[10][39:]
    disable = page(identity, 5, 2, 1, hold[339:371], active_release=True,
                   epoch=3, group_count=2, group_id=9, category=3)
    terminal = page(identity, 5, 2, 2, disable[339:371], active_release=True,
                    epoch=2, group_count=2)
    records[11] = record(1, 6300, 2, 3, 5, 1, disable)
    records[12] = record(1, 6500, 2, 3, 5, 2, terminal)
    rows = list(csv.reader(timeline.open(newline="")))
    rows[-2][7] = "3"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    response = joiner._decode("DummyManageV2Response", records[13][39:])
    response["motion_epoch"] = 3
    records[13] = record(5, 6600, 2, 0, 5, 0,
                         wire("DummyManageV2Response", **response))
    for index, kind in ((14, "DummyDeviceStatusV2"), (15, "DummyJointStateV2")):
        fields = joiner._decode(kind, records[index][39:])
        fields["motion_epoch"] = 3
        if index == 14:
            fields["evidence_chain_sha256"] = terminal[339:371].hex()
        records[index] = record(3 if index == 14 else 2, 7000, 2, 0, 0, 0,
                                wire(kind, **fields))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    assert joiner.join_evidence(timeline, raw, action, context)["release"]["run_id"] == 5


def test_active_hold_page_may_precede_user_release_request(inputs):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    hold = records.pop(10)
    records.insert(9, record(1, 6100, 2, 3, 5, 0, hold[39:]))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    assert joiner.join_evidence(timeline, raw, action, context)["release"]["run_id"] == 5


def test_release_dispatch_budget_ignores_prior_hold_but_bounds_late_pages():
    page = lambda when, category: {"when": when, "fields": {"category": category}}
    prior = page(100, 1)
    near_hold = page(205, 1)
    disable = page(210, 3)
    assert joiner._release_dispatch_within_budget(
        [prior, near_hold, disable], 200, 20)
    assert not joiner._release_dispatch_within_budget(
        [prior, near_hold, disable], 200, 9)
    assert not joiner._release_dispatch_within_budget(
        [prior, page(500, 1), page(510, 3)], 200, 20)
    assert not joiner._release_dispatch_within_budget(
        [prior, near_hold], 200, 20)


def test_active_hold_page_may_precede_gate_pre_release_hold(inputs):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    hold = records.pop(10)
    records.insert(7, record(1, 5800, 2, 3, 5, 0, hold[39:]))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["release"]["terminal"] == "CANCELED_AND_RELEASED"


def test_release_request_after_disable_sent_is_rejected(inputs):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    request = joiner._decode("DummyManageV2Request", records[9][39:])
    records.pop(9)
    records.insert(11, record(4, 6400, 2, 3, 0, request["request_seq"],
                              wire("DummyManageV2Request", **request)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="release request does not precede"):
        joiner.join_evidence(timeline, raw, action, context)


def test_idle_epoch_without_same_release_receipt_epoch_is_rejected(inputs):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    rows = list(csv.reader(timeline.open(newline="")))
    rows[-2][7] = "4"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    for index, kind in ((14, "DummyDeviceStatusV2"), (15, "DummyJointStateV2")):
        fields = joiner._decode(kind, records[index][39:])
        fields["motion_epoch"] = 4
        records[index] = record(3 if index == 14 else 2, 7000, 2, 0, 0, 0,
                                wire(kind, **fields))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="Idle motion epoch"):
        joiner.join_evidence(timeline, raw, action, context)


def test_prime_ros_feedback_before_after_begin_is_not_path_evidence(inputs):
    timeline, raw, action, context, _ = inputs
    report = json.loads(action.read_text())
    prime = {**report["feedback"][0], "elapsed_us": 2000,
             "actual_rad": [0.0] * 6, "desired_rad": [0.0] * 6}
    report["feedback"].insert(0, prime)
    action.write_text(json.dumps(report))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["path_error_max_rad"] == [0.0] * 6


def test_prime_only_ros_feedback_cannot_prove_motion_path(inputs):
    timeline, raw, action, context, _ = inputs
    report = json.loads(action.read_text())
    report["feedback"][0]["elapsed_us"] = 2000
    action.write_text(json.dumps(report))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="post-Begin ROS feedback"):
        joiner.join_evidence(timeline, raw, action, context)


def test_later_gate_raw_feedback_covers_ros_feedback_tail_gap(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    late = rows[4].copy()
    late[3] = "4500"
    late[12:18] = ["2"] * 6
    rows.insert(5, late)
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    state = joiner._decode("DummyJointStateV2", records[3][39:])
    state["feedback_seq"] = [2] * 6
    records.insert(4, record(2, 4400, 2, 3, 0, 0,
                             wire("DummyJointStateV2", **state)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    limits = json.loads(context.read_text())
    limits["ros_feedback_lag_budget_us"] = 100
    context.write_text(json.dumps(limits))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["feedback_age_us"] == [1000] * 6


@pytest.mark.parametrize("change", (None, "missing_raw", "late_raw", "wrong_identity",
                                   "replayed_seq", "no_gate_continuation",
                                   "delayed_gate", "five_stale_axes", "partial_seq",
                                   "bad_phase"))
def test_ros_handoff_uses_fresh_raw_state_then_gate_feedback(inputs, change):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[4][3] = "4500"
    rows[4][12:18] = ["3"] * 6
    if change == "delayed_gate":
        rows[4][3] = "4601"
    if change == "no_gate_continuation":
        rows[4][3] = "500000"
        rows[5][3], rows[6][3], rows[7][3] = "501000", "502000", "503000"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    state = joiner._decode("DummyJointStateV2", records[3][39:])
    state["feedback_seq"] = [2] * 6
    state["core_state"] = 5
    if change == "wrong_identity":
        state["motion_epoch"] = 99
    if change == "replayed_seq":
        state["state_seq"] = 1
    if change == "five_stale_axes":
        state["can_rx_age_us"] = [1000] + [11000] * 5
    if change == "partial_seq":
        state["feedback_seq"] = [2] + [1] * 5
    if change == "bad_phase":
        state["core_state"] = 3
    handoff_time = 4450 if change == "late_raw" else 4200
    if change != "missing_raw":
        records.insert(4, record(2, handoff_time, 2, 3, 0, 0,
                                 wire("DummyJointStateV2", **state)))
    if change != "no_gate_continuation":
        gate_state = {**state, "motion_epoch": 2, "feedback_seq": [3] * 6,
                      "can_rx_age_us": [1000] * 6, "core_state": 5}
        records.insert(5 if change != "missing_raw" else 4,
                       record(2, 4601 if change == "delayed_gate" else 4500,
                              2, 3, 0, 0,
                              wire("DummyJointStateV2", **gate_state)))
    if change == "delayed_gate":
        records[6] = record(1, 4700, 2, 3, 4, 1, records[6][39:])
    if change == "no_gate_continuation":
        original = records[5 if change != "missing_raw" else 4]
        records[5 if change != "missing_raw" else 4] = record(1, 500500, 2, 3, 4, 1,
                                                               original[39:])
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    metadata = json.loads(context.read_text())
    metadata["ros_feedback_lag_budget_us"] = 400
    if change == "no_gate_continuation":
        metadata["gate_state_max_age_us"] = 500000
    context.write_text(json.dumps(metadata))
    repin(timeline, raw, action, context, preserve_state_sequence=change == "replayed_seq")
    if change is None:
        assert joiner.join_evidence(timeline, raw, action, context)["goals"][0]["run_id"] == 4
    else:
        message = ("ROS feedback time" if change in
                   ("delayed_gate", "five_stale_axes", "partial_seq", "bad_phase") else None)
        with pytest.raises(ValueError, match=message):
            joiner.join_evidence(timeline, raw, action, context)


def test_long_terminal_tail_uses_nearby_ros_gate_handoff_and_fresh_raw(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    middle = rows[4].copy()
    middle[3] = "250000"
    middle[12:18] = ["2"] * 6
    late = rows[4].copy()
    late[3] = "505000"
    late[12:18] = ["3"] * 6
    rows[5:5] = [middle, late]
    rows[7][3], rows[8][3], rows[9][3] = "506000", "507000", "508000"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    state = joiner._decode("DummyJointStateV2", records[3][39:])
    state["feedback_seq"] = [2] * 6
    middle_state = record(2, 249900, 2, 3, 0, 0,
                          wire("DummyJointStateV2", **state))
    state["feedback_seq"] = [3] * 6
    late_state = record(2, 505000, 2, 3, 0, 0,
                        wire("DummyJointStateV2", **state))
    updated = records[:5] + [middle_state, late_state]
    for index, when in ((5, 506900), (6, 507000), (7, 507400),
                        (8, 507500), (9, 508000), (10, 508000)):
        original = records[index]
        kind, size, _, boot, session, run, seq = struct.unpack_from(
            "<B H Q Q Q Q I", original)
        updated.append(record(kind, when, boot, session, run, seq, original[39:]))
    raw.write_bytes(b"DPRAW01\0" + b"".join(updated))
    metadata = json.loads(context.read_text())
    metadata["gate_state_max_age_us"] = 500000
    context.write_text(json.dumps(metadata))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["timeline"][-4]["steady_us"] == 505000


def test_long_terminal_tail_without_continuous_gate_raw_path_is_rejected(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    late = rows[4].copy()
    late[3] = "505000"
    late[12:18] = ["2"] * 6
    rows.insert(5, late)
    rows[6][3], rows[7][3], rows[8][3] = "506000", "507000", "508000"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    state = joiner._decode("DummyJointStateV2", records[3][39:])
    state["feedback_seq"] = [2] * 6
    updated = records[:5] + [record(2, 505000, 2, 3, 0, 0,
                                    wire("DummyJointStateV2", **state))]
    for index, when in ((5, 506900), (6, 507000), (7, 507400),
                        (8, 507500), (9, 508000), (10, 508000)):
        original = records[index]
        kind, size, _, boot, session, run, seq = struct.unpack_from(
            "<B H Q Q Q Q I", original)
        updated.append(record(kind, when, boot, session, run, seq, original[39:]))
    raw.write_bytes(b"DPRAW01\0" + b"".join(updated))
    metadata = json.loads(context.read_text())
    metadata["gate_state_max_age_us"] = 500000
    context.write_text(json.dumps(metadata))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="unobserved raw path gap"):
        joiner.join_evidence(timeline, raw, action, context)


def test_release_receipt_may_follow_final_status_callback(inputs):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    response = joiner._decode("DummyManageV2Response", records[13][39:])
    records.pop(13)
    records.append(record(5, 7100, 2, 0, 5, 0,
                          wire("DummyManageV2Response", **response)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    assert joiner.join_evidence(timeline, raw, action, context)["release"]["run_id"] == 5


@pytest.mark.parametrize("field,value", [
    ("real_motion_permitted", 1), ("permit_remaining_us", 1000),
    ("reference_epoch", 1),
])
def test_active_release_requires_cleared_idle_authority(inputs, field, value):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    status = joiner._decode("DummyDeviceStatusV2", records[14][39:])
    status[field] = value
    records[14] = record(3, 7000, 2, 0, 0, 0,
                         wire("DummyDeviceStatusV2", **status))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError):
        joiner.join_evidence(timeline, raw, action, context)


def test_old_timeline_without_release_key_cannot_join_positive_capture(inputs):
    timeline, raw, action, context, _ = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[0] = rows[0][:-2]
    rows[1:-1] = [row[:-2] for row in rows[1:-1]]
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="release request key"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("mutation,match", [
    ("missing_receipt", "receipt"),
    ("pending_receipt", "receipt"),
    ("wrong_request", "receipt"),
    ("wrong_run", "receipt"),
    ("partial_disable", "six-axis CAN"),
    ("duplicate_terminal", "terminal|evidence"),
    ("wrong_timeline_key", "release management request"),
])
def test_active_release_raw_failures_are_not_positive(inputs, mutation, match):
    timeline, raw, action, context, records = active_release_inputs(inputs)
    if mutation == "missing_receipt":
        records.pop(13)
    elif mutation in ("pending_receipt", "wrong_request", "wrong_run"):
        fields = joiner._decode("DummyManageV2Response", records[13][39:])
        if mutation == "pending_receipt":
            fields["transaction_phase"] = 2
        elif mutation == "wrong_request":
            fields["request_id"] = 99
        else:
            fields["run_id"] = 4
        records[13] = record(5, 6600, 2, 0, fields["run_id"], 0,
                             wire("DummyManageV2Response", **fields))
    elif mutation == "partial_disable":
        identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
            ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
        hold = records[10][39:]
        disable = page(identity, 5, 2, 1, hold[339:371], active_release=True,
                       epoch=3, group_count=2, group_id=9, category=3,
                       sent_mask=31)
        terminal = page(identity, 5, 2, 2, disable[339:371], active_release=True,
                        group_count=2)
        records[11] = record(1, 6300, 2, 3, 5, 1, disable)
        records[12] = record(1, 6500, 2, 3, 5, 2, terminal)
        status = joiner._decode("DummyDeviceStatusV2", records[14][39:])
        status["evidence_chain_sha256"] = terminal[339:371].hex()
        records[14] = record(3, 7000, 2, 0, 0, 0,
                             wire("DummyDeviceStatusV2", **status))
    elif mutation == "duplicate_terminal":
        records.insert(13, records[12])
    else:
        rows = list(csv.reader(timeline.open(newline="")))
        rows[-2][-2] = "99"
        with timeline.open("w", newline="") as output:
            csv.writer(output).writerows(rows)
    if mutation != "wrong_timeline_key":
        raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match=match):
        joiner.join_evidence(timeline, raw, action, context)


def test_mixed_epoch_raw_chain_preserves_hold_prefix_and_goal_group(inputs):
    timeline, raw, action, context, records = mixed_epoch_inputs(inputs)
    assert struct.unpack_from("<Q", records[1], 3)[0] < 2000
    assert 2000 < struct.unpack_from("<Q", records[4], 3)[0] < 3000
    result = joiner.join_evidence(timeline, raw, action, context)
    assert [group["motion_epoch"] for group in result["goals"][0]["groups"]] == [1, 2, 3]
    assert result["pre_release_hold"]["motion_epoch"] == 3


def test_goal_epoch_raw_page_cannot_precede_before_begin(inputs):
    timeline, raw, action, context, records = mixed_epoch_inputs(inputs)
    early_goal = record(1, 1990, 2, 3, 4, 1, records[4][39:])
    records = [*records[:2], early_goal, *records[2:4], *records[5:]]
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="goal-epoch raw page predates Begin"):
        joiner.join_evidence(timeline, raw, action, context)


def test_late_ros_feedback_binds_complete_hold_raw_state(inputs):
    timeline, raw, action, context, records = mixed_epoch_inputs(inputs)
    rows = list(csv.reader(timeline.open(newline="")))
    late = rows[4].copy()
    late[3], late[7] = "5700", "3"
    rows.insert(5, late)
    rows[6][3] = "5800"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    hold_state = joiner._decode("DummyJointStateV2", records[10][39:])
    records.insert(9, record(2, 5600, 2, 3, 0, 0,
                             wire("DummyJointStateV2", **hold_state)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    report = json.loads(action.read_text())
    report["feedback"].append({"elapsed_us": 5150, "actual_rad": [0.01] * 6,
                               "desired_rad": [0.01] * 6})
    action.write_text(json.dumps(report))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["timeline"][-4]["motion_epoch"] == 3


def test_joins_two_goals_one_session_and_final_release(inputs):
    timeline, raw, action, context, _ = two_goal_inputs(inputs)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert [(goal["goal_id"], goal["run_id"], goal["motion_epoch"]) for goal in
            result["goals"]] == [("a" * 32, 4, 2), ("c" * 32, 5, 3)]
    assert [row["event"] for row in result["goals"][0]["timeline"]][-1] == "pre_release_hold"
    assert result["release"]["parent_run_id"] == 5


@pytest.mark.parametrize("mutation,match", [
    ("missing_second_terminal", "terminal"),
    ("second_cross_session", "session"),
    ("duplicate_goal_run", "run|status"),
    ("second_bad_chain", "chain"),
    ("second_partial_can", "CAN|six-axis"),
    ("second_hash_drift", "identity"),
    ("early_release", "RELEASE"),
    ("footer_count", "count"),
    ("forged_completed_count", "action"),
    ("forged_second_success", "unsuccessful"),
])
def test_two_goal_capture_fails_closed(inputs, mutation, match):
    timeline, raw, action, context, records = two_goal_inputs(inputs)
    if mutation == "missing_second_terminal":
        records.pop(11)
    elif mutation == "second_cross_session":
        records[9] = record(1, 10500, 2, 9, 5, 0, records[9][39:])
    elif mutation == "duplicate_goal_run":
        rows = list(csv.reader(timeline.open(newline="")))
        for row in rows[7:13]:
            row[8] = "4"
        with timeline.open("w", newline="") as output:
            csv.writer(output).writerows(rows)
    elif mutation == "second_bad_chain":
        status = joiner._decode("DummyDeviceStatusV2", records[12][39:])
        status["evidence_chain_sha256"] = "00" * 32
        records[12] = record(3, 12900, 2, 3, 0, 0,
                             wire("DummyDeviceStatusV2", **status))
    elif mutation == "second_partial_can":
        identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
            ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
        first = page(identity, 5, 2, 0, b"", epoch=3, sent_mask=31)
        terminal = page(identity, 5, 2, 1, first[339:371], epoch=3)
        records[9] = record(1, 10500, 2, 3, 5, 0, first)
        records[11] = record(1, 11500, 2, 3, 5, 1, terminal)
        status = joiner._decode("DummyDeviceStatusV2", records[12][39:])
        status["evidence_chain_sha256"] = terminal[339:371].hex()
        records[12] = record(3, 12900, 2, 3, 0, 0,
                             wire("DummyDeviceStatusV2", **status))
    elif mutation == "second_hash_drift":
        status = joiner._decode("DummyDeviceStatusV2", records[7][39:])
        status["profile_sha256"] = "22" * 32
        records[7] = record(3, 9900, 2, 3, 0, 0,
                            wire("DummyDeviceStatusV2", **status))
    elif mutation == "early_release":
        identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
            ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
        first = page(identity, 7, 2, 0, b"", release=True)
        last = page(identity, 7, 2, 1, first[339:371], release=True)
        records[7:7] = [record(1, 6500, 2, 3, 7, 0, first),
                        record(1, 6600, 2, 3, 7, 1, last)]
    elif mutation == "forged_completed_count":
        report = json.loads(action.read_text())
        report["completed_goals"] = 1
        action.write_text(json.dumps(report))
    elif mutation == "forged_second_success":
        report = json.loads(action.read_text())
        report["goals"][1]["action_status"] = 6
        action.write_text(json.dumps(report))
    elif mutation == "footer_count":
        timeline.write_text(timeline.read_text().replace("COMPLETE,2", "COMPLETE,3"))
    if mutation in ("missing_second_terminal", "second_cross_session",
                    "second_bad_chain", "second_partial_can", "second_hash_drift",
                    "early_release"):
        raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match=match):
        joiner.join_evidence(timeline, raw, action, context)


def test_joins_real_wire_pages_and_validates(inputs):
    timeline, raw, action, context, _ = inputs
    result = joiner.join_evidence(timeline, raw, action, context)
    assert len(result["goals"][0]["evidence_pages"]) == 2
    assert result["goals"][0]["groups"][0]["sent_mask"] == 63
    assert result["release"]["parent_run_id"] == 4
    assert result["provenance"]["raw_sha256"] == json.loads(context.read_text())["raw_sha256"]


def test_joiner_rejects_unverified_window_version(inputs):
    timeline, raw, action, context, _ = inputs
    metadata = json.loads(context.read_text())
    metadata.update(producer_lag_gate_version=2,
                    producer_lag_method="state_receive_minus_produced_offset_span_motion_window",
                    producer_lag_budget_us=250000,
                    producer_lag_environment_us=300000,
                    producer_lag_window={"start_steady_us": 1000,
                                         "end_steady_us": 7000,
                                         "state_samples": 3,
                                         "release_run_id": 5})
    context.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="unverified window producer lag"):
        joiner.join_evidence(timeline, raw, action, context)


def test_joiner_rejects_sha_pinned_over_budget_window(inputs):
    timeline, raw, action, context, _ = inputs
    records = joiner._raw_records(raw.read_bytes())
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"],
        wire("DummyJointStateV2", **{**item["fields"],
            "produced_at_us": item["when"] +
            (300000 if item["when"] >= 6000 else 0)})
        if item["kind"] == 2 else item["raw"]) for item in records))
    repin(timeline, raw, action, context, preserve_state_sequence=True)
    records = joiner._raw_records(raw.read_bytes())
    metadata = json.loads(context.read_text())
    start, end, states, release_run = builder._motion_window(
        timeline.read_bytes(), records, metadata["client_id"],
        metadata["gate_state_max_age_us"])
    offsets = [item["when"] - item["fields"]["produced_at_us"] for item in states]
    metadata.update(producer_lag_gate_version=2,
                    producer_lag_method=builder.WINDOW_LAG_METHOD,
                    producer_lag_us=max(offsets) - min(offsets),
                    producer_lag_budget_us=250000,
                    producer_lag_environment_us=300000,
                    producer_lag_window={"start_steady_us": start,
                                         "end_steady_us": end,
                                         "state_samples": len(states),
                                         "release_run_id": release_run})
    context.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="producer lag budget"):
        joiner._verify_window_context(metadata, timeline.read_bytes(), records)


@pytest.mark.parametrize("session_text", ["3", "03"])
def test_window_context_joins_valid_raw_capture(inputs, session_text):
    timeline, raw, action, context, _ = inputs
    if session_text != "3":
        rows = list(csv.reader(timeline.open(newline="")))
        for row in rows[1:-1]:
            if row[5] == "3":
                row[5] = session_text
        with timeline.open("w", newline="") as output_stream:
            csv.writer(output_stream).writerows(rows)
    output = context.with_name("window-context.json")
    metadata = builder.build_context(
        timeline, raw, action, output,
        path_tolerance_rad=[0.01] * 6, goal_tolerance_rad=[0.003] * 6,
        producer_lag_budget_us=250000, feedback_age_budget_us=10000,
        ros_feedback_lag_budget_us=2000, gate_state_max_age_us=250000,
        producer_lag_scope="motion_window_v2")
    joined = joiner.join_evidence(timeline, raw, action, output)
    application_e2e.validate_evidence(joined)
    assert joined["environment"]["producer_lag_gate_version"] == 2
    assert joined["environment"]["producer_lag_us"] == metadata["producer_lag_us"]


def test_window_context_rejects_nonnumeric_session(inputs):
    timeline, raw, action, context, _ = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[1][5] = "not-a-session"
    with timeline.open("w", newline="") as output_stream:
        csv.writer(output_stream).writerows(rows)
    output = context.with_name("invalid-window-context.json")
    with pytest.raises(ValueError, match="invalid ROS motion window timeline"):
        builder.build_context(
            timeline, raw, action, output,
            path_tolerance_rad=[0.01] * 6, goal_tolerance_rad=[0.003] * 6,
            producer_lag_budget_us=250000, feedback_age_budget_us=10000,
            ros_feedback_lag_budget_us=2000, gate_state_max_age_us=250000,
            producer_lag_scope="motion_window_v2")
    assert not output.exists()


def test_accepts_real_hold_wire_state_four(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[6][10:12] = ["4", "4"]
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    hold_status = joiner._decode("DummyDeviceStatusV2", records[5][39:])
    hold_status["core_state"] = 4
    hold_state = joiner._decode("DummyJointStateV2", records[6][39:])
    hold_state["core_state"] = 4
    records[5] = record(3, 5900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **hold_status))
    records[6] = record(2, 6000, 2, 3, 0, 0, wire("DummyJointStateV2", **hold_state))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    assert joiner.join_evidence(timeline, raw, action, context)["goals"][0]["hold_confirmed"]


@pytest.mark.parametrize("change", [
    {"evidence_chain_sha256": "00" * 32}, {"attempt_kind": 3},
    {"final_state": 7}, {"evidence_lost": 1}, {"run_id": 9},
])
def test_hold_status_must_bind_verified_terminal_chain(inputs, change):
    timeline, raw, action, context, records = inputs
    status = joiner._decode("DummyDeviceStatusV2", records[5][39:])
    status.update(change)
    changed = record(3, 5900, 2, 3, 0, 0, wire("DummyDeviceStatusV2", **status))
    raw.write_bytes(b"DPRAW01\0" + b"".join([*records[:5], changed, *records[6:]]))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="terminal|HOLD|run|status"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("change", [
    {"evidence_chain_sha256": "00" * 32},
    {"attempt_kind": 1},
    {"final_state": 6},
    {"evidence_lost": 1},
])
def test_release_idle_status_must_bind_verified_terminal(inputs, change):
    timeline, raw, action, context, records = inputs
    status = joiner._decode("DummyDeviceStatusV2", records[9][39:])
    records[9] = record(3, 7000, 2, 0, 0, 0,
                        wire("DummyDeviceStatusV2", **{**status, **change}))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="RELEASE|release"):
        joiner.join_evidence(timeline, raw, action, context)


def test_release_idle_status_cannot_precede_terminal_page(inputs):
    timeline, raw, action, context, records = inputs
    early_idle = record(3, 6300, 2, 0, 0, 0, records[9][39:])
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        [*records[:7], early_idle, *records[7:9], records[10]]))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="RELEASE|release"):
        joiner.join_evidence(timeline, raw, action, context)


def test_positive_cli_requires_rejoined_sha_pinned_capture(inputs, tmp_path, monkeypatch):
    timeline, raw, action, context, _ = inputs
    evidence = tmp_path / "joined.json"
    evidence.write_text(json.dumps(joiner.join_evidence(timeline, raw, action, context)))
    report = tmp_path / "report.json"
    monkeypatch.setattr(sys, "argv", ["application_e2e", "--evidence", str(evidence),
        "--report", str(report)])
    assert application_e2e.main() == 0
    assert json.loads(report.read_text())["qualification"] == "UNQUALIFIED"
    monkeypatch.setattr(sys, "argv", ["application_e2e", "--evidence", str(evidence),
        "--report", str(report), "--timeline", str(timeline), "--raw", str(raw),
        "--action", str(action), "--context", str(context)])
    assert application_e2e.main() == 0
    positive = json.loads(report.read_text())
    assert positive["status"] == "ROS_BENCH_E2E_POSITIVE"
    assert positive["qualification"] == "UNQUALIFIED"
    assert positive["hardware_qualification"] == "NOT_ASSERTED"
    forged = json.loads(evidence.read_text())
    forged["provenance"]["raw_sha256"] = "0" * 64
    evidence.write_text(json.dumps(forged))
    assert application_e2e.main() == 1
    assert json.loads(report.read_text())["status"] == "FAIL"


def test_positive_cli_rejects_partial_capture_and_environment_mismatch(
        inputs, tmp_path, monkeypatch):
    timeline, raw, action, context, _ = inputs
    evidence = tmp_path / "joined.json"
    evidence.write_text(json.dumps(joiner.join_evidence(timeline, raw, action, context)))
    report = tmp_path / "report.json"
    base = ["application_e2e", "--evidence", str(evidence), "--report", str(report)]
    monkeypatch.setattr(sys, "argv", base + ["--raw", str(raw)])
    assert application_e2e.main() == 1
    assert json.loads(report.read_text())["qualification"] == "UNQUALIFIED"
    metadata = json.loads(context.read_text())
    metadata["producer_lag_us"] = metadata["producer_lag_budget_us"] + 1
    context.write_text(json.dumps(metadata))
    monkeypatch.setattr(sys, "argv", base + ["--timeline", str(timeline),
        "--raw", str(raw), "--action", str(action), "--context", str(context)])
    assert application_e2e.main() == 1
    assert json.loads(report.read_text())["status"] == "FAIL"


def test_accepts_multiple_gate_feedback_rows(inputs):
    timeline, raw, action, context, _ = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    extra = copy.copy(rows[4])
    extra[3] = "4500"
    rows.insert(5, extra)
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert [row["event"] for row in result["goals"][0]["timeline"]].count("feedback") == 2


def test_rejects_intermediate_raw_position_off_planned_path(inputs):
    timeline, raw, action, context, records = inputs
    intermediate = joiner._decode("DummyJointStateV2", records[3][39:])
    intermediate["position_rad"] = [0.08] * 6
    observed = record(2, 3600, 2, 3, 0, 0, wire("DummyJointStateV2", **intermediate))
    raw.write_bytes(b"DPRAW01\0" + b"".join([*records[:3], observed, *records[3:]]))
    action_report = json.loads(action.read_text())
    action_report["feedback"].insert(0, {"elapsed_us": 3100,
        "actual_rad": [0.01] * 6, "desired_rad": [0.01] * 6})
    action.write_text(json.dumps(action_report))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="raw.*path|ROS feedback outside bounded raw"):
        joiner.join_evidence(timeline, raw, action, context)


def test_replayed_raw_state_between_feedback_rows_is_rejected(inputs):
    timeline, raw, action, context, records = inputs
    first = joiner._decode("DummyJointStateV2", records[1][39:])
    next_state = joiner._decode("DummyJointStateV2", records[3][39:])
    first.update(state_seq=10, produced_at_us=100)
    next_state.update(state_seq=11, produced_at_us=200)
    replay = {**next_state, "state_seq": 10, "produced_at_us": 201}
    records[1] = record(2, 3000, 2, 3, 0, 0, wire("DummyJointStateV2", **first))
    records[3] = record(2, 4000, 2, 3, 0, 0, wire("DummyJointStateV2", **next_state))
    records.insert(4, record(2, 4100, 2, 3, 0, 0,
                             wire("DummyJointStateV2", **replay)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context, preserve_state_sequence=True)
    with pytest.raises(ValueError, match="replay|sequence"):
        joiner.join_evidence(timeline, raw, action, context)


def test_old_raw_state_epoch_cannot_reappear_after_complete_phase(inputs):
    timeline, raw, action, context, records = mixed_epoch_inputs(inputs)
    old = joiner._decode("DummyJointStateV2", records[2][39:])
    records.insert(9, record(2, 5700, 2, 3, 0, 0,
                             wire("DummyJointStateV2", **old)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="raw State outside goal authority epochs"):
        joiner.join_evidence(timeline, raw, action, context)


def test_equal_producer_timestamps_with_increasing_state_sequence_are_valid(inputs):
    timeline, raw, action, context, records = inputs
    for index, sequence, produced in ((1, 10, 100), (3, 11, 100), (6, 12, 200)):
        fields = joiner._decode("DummyJointStateV2", records[index][39:])
        fields.update(state_seq=sequence, produced_at_us=produced)
        records[index] = record(2, {1: 3000, 3: 4000, 6: 6000}[index],
                                2, 3, 0, 0, wire("DummyJointStateV2", **fields))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context, preserve_state_sequence=True)
    assert joiner.join_evidence(timeline, raw, action, context)["goals"][0]["run_id"] == 4


def test_regressing_producer_timestamp_is_rejected(inputs):
    timeline, raw, action, context, records = inputs
    for index, sequence, produced in ((1, 10, 200), (3, 11, 100), (6, 12, 300)):
        fields = joiner._decode("DummyJointStateV2", records[index][39:])
        fields.update(state_seq=sequence, produced_at_us=produced)
        records[index] = record(2, {1: 3000, 3: 4000, 6: 6000}[index],
                                2, 3, 0, 0, wire("DummyJointStateV2", **fields))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context, preserve_state_sequence=True)
    with pytest.raises(ValueError, match="producer time regression"):
        joiner.join_evidence(timeline, raw, action, context)


def test_raw_state_excursion_after_last_gate_feedback_is_rejected(inputs):
    timeline, raw, action, context, records = inputs
    state = joiner._decode("DummyJointStateV2", records[3][39:])
    excursion = record(2, 4200, 2, 3, 0, 0,
                       wire("DummyJointStateV2", **{
                           **state, "position_rad": [0.08] * 6}))
    recovery = record(2, 4300, 2, 3, 0, 0, records[3][39:])
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        [*records[:4], excursion, recovery, *records[4:]]))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="raw State outside bounded trajectory path"):
        joiner.join_evidence(timeline, raw, action, context)


def test_unobserved_final_gate_to_terminal_state_gap_is_rejected(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[5][3], rows[6][3], rows[7][3] = "600000", "601000", "602000"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    state = joiner._decode("DummyJointStateV2", records[3][39:])
    late = record(2, 600000, 2, 3, 0, 0,
                  wire("DummyJointStateV2", **state))
    updated = records[:5] + [late]
    for index, when in ((5, 600900), (6, 601000), (7, 601400),
                        (8, 601500), (9, 602000), (10, 602000)):
        original = records[index]
        kind, size, _, boot, session, run, seq = struct.unpack_from(
            "<B H Q Q Q Q I", original)
        updated.append(record(kind, when, boot, session, run, seq, original[39:]))
    raw.write_bytes(b"DPRAW01\0" + b"".join(updated))
    metadata = json.loads(context.read_text())
    metadata["gate_state_max_age_us"] = 500000
    context.write_text(json.dumps(metadata))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="terminal/HOLD has unobserved raw State gap"):
        joiner.join_evidence(timeline, raw, action, context)


def test_gate_feedback_raw_position_contributes_to_path_error(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    middle = rows[4].copy()
    middle[3] = "4500"
    restored = rows[4].copy()
    restored[3] = "4900"
    rows[5:5] = [middle, restored]
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    state = joiner._decode("DummyJointStateV2", records[3][39:])
    excursion = record(2, 4300, 2, 3, 0, 0,
                       wire("DummyJointStateV2", **{**state, "position_rad": [0.08] * 6}))
    restored_state = record(2, 4800, 2, 3, 0, 0,
                            wire("DummyJointStateV2", **state))
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        [*records[:4], excursion, records[4], restored_state, *records[5:]]))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="trajectory path|trajectory tolerance"):
        joiner.join_evidence(timeline, raw, action, context)


def test_gate_feedback_uses_interpolated_ros_desired_position(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    middle = rows[4].copy()
    middle[3] = "4400"
    final = rows[4].copy()
    final[3] = "4900"
    rows[5:5] = [middle, final]
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    first_state = joiner._decode("DummyJointStateV2", records[3][39:])
    moving = record(2, 4300, 2, 3, 0, 0, wire("DummyJointStateV2", **{
        **first_state, "position_rad": [0.02] * 6}))
    arrived = record(2, 4800, 2, 3, 0, 0, wire("DummyJointStateV2", **{
        **first_state, "position_rad": [0.03] * 6}))
    hold_state = joiner._decode("DummyJointStateV2", records[6][39:])
    records[6] = record(2, 6000, 2, 3, 0, 0, wire("DummyJointStateV2", **{
        **hold_state, "position_rad": [0.03] * 6}))
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        [*records[:4], moving, records[4], arrived, *records[5:]]))
    report = json.loads(action.read_text())
    report["target_rad"] = [0.03] * 6
    report["feedback"].append({"elapsed_us": 4300, "actual_rad": [0.03] * 6,
                               "desired_rad": [0.03] * 6})
    action.write_text(json.dumps(report))
    metadata = json.loads(context.read_text())
    metadata["path_tolerance_rad"] = [0.005] * 6
    context.write_text(json.dumps(metadata))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["path_error_max_rad"] == pytest.approx([0.0025] * 6)


def test_final_raw_position_must_reach_target_within_goal_tolerance(inputs):
    timeline, raw, action, context, records = inputs
    report = json.loads(action.read_text())
    report["feedback"][0]["actual_rad"] = [0.0125] * 6
    action.write_text(json.dumps(report))
    for index in (1, 3, 6):
        state = joiner._decode("DummyJointStateV2", records[index][39:])
        records[index] = record(2, {1: 3000, 3: 4000, 6: 6000}[index], 2, 3, 0, 0,
                                wire("DummyJointStateV2", **{
                                    **state, "position_rad": [0.015] * 6}))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="goal tolerance"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("change", (None, "missing_near", "stale_near",
                                   "wrong_identity", "stale_axes",
                                   "sample_mismatch", "hold_outside",
                                   "path_excursion"))
def test_final_ros_sample_and_later_hold_use_their_own_raw_positions(inputs, change):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    if change == "stale_axes":
        rows[4][3] = "3990"
    late = rows[4].copy()
    late[3] = "5000"
    late[12:18] = ["2"] * 6
    rows.insert(5, late)
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    report = json.loads(action.read_text())
    report["target_rad"] = [0.015] * 6
    report["feedback"][0]["actual_rad"] = ([0.0175] * 6 if change == "sample_mismatch"
                                            else [0.01382] * 6)
    report["feedback"][0]["desired_rad"] = [0.015] * 6
    action.write_text(json.dumps(report))
    near = joiner._decode("DummyJointStateV2", records[3][39:])
    near["position_rad"] = [0.01382] * 6
    if change == "wrong_identity":
        near["motion_epoch"] = 99
    if change == "stale_axes":
        near["can_rx_age_us"] = [11000] * 6
    near_time = 3900 if change == "stale_near" else 4000
    records[3] = record(2, near_time, 2, 3, 0, 0,
                        wire("DummyJointStateV2", **near))
    if change == "missing_near":
        records.pop(3)
    gate = joiner._decode("DummyJointStateV2", records[6 if change != "missing_near" else 5][39:])
    gate["feedback_seq"] = [2] * 6
    gate["position_rad"] = [0.01752] * 6
    gate["can_rx_age_us"] = [1000] * 6
    if change == "path_excursion":
        excursion = {**gate, "position_rad": [0.08] * 6}
        records.insert(4 if change != "missing_near" else 3,
                       record(2, 4300, 2, 3, 0, 0,
                              wire("DummyJointStateV2", **excursion)))
    hold_index = next(index for index, item in enumerate(records) if
                      item[0] == 2 and struct.unpack_from("<Q", item, 3)[0] == 6000)
    hold_position = [0.019] * 6 if change == "hold_outside" else gate["position_rad"]
    records[hold_index] = record(2, 6000, 2, 3, 0, 0,
                                 wire("DummyJointStateV2", **{
                                     **gate, "position_rad": hold_position,
                                     "hold_ready": 1}))
    gate_state = {**gate, "core_state": 5, "hold_ready": 0}
    records.insert(hold_index - 1, record(2, 4900, 2, 3, 0, 0,
                                           wire("DummyJointStateV2", **gate_state)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    metadata = json.loads(context.read_text())
    metadata["ros_feedback_lag_budget_us"] = 40
    context.write_text(json.dumps(metadata))
    repin(timeline, raw, action, context)
    if change is None:
        assert joiner.join_evidence(timeline, raw, action, context)["goals"][0]["run_id"] == 4
    else:
        matches = {"missing_near": "missing fresh raw State near",
                   "stale_near": "missing fresh raw State near",
                   "wrong_identity": "ambiguous prior raw state identity",
                   "stale_axes": "stale raw State near",
                   "sample_mismatch": "ROS feedback outside bounded raw device position envelope",
                   "hold_outside": "raw device position outside goal tolerance",
                   "path_excursion": "raw State outside bounded trajectory path"}
        with pytest.raises(ValueError, match=matches[change]):
            joiner.join_evidence(timeline, raw, action, context)


def test_rejects_reference_epoch_drift_in_gate_timeline(inputs):
    timeline, raw, action, context, _ = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[4][6] = "9"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="reference"):
        joiner.join_evidence(timeline, raw, action, context)


def test_rejects_status_hash_drift_before_hold(inputs):
    timeline, raw, action, context, records = inputs
    changed = bytearray(records[5])
    changed[39 + 113] ^= 1
    raw.write_bytes(b"DPRAW01\0" + b"".join(records[:5] +
                    [bytes(changed)] + records[6:]))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="identity"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("mutation,match", [
    ("missing_terminal", "terminal"),
    ("cross_session", "session"),
    ("ambiguous", "ambiguous"),
    ("truncated", "truncated"),
    ("hash", "hash"),
    ("missing_complete", "COMPLETE"),
])
def test_rejects_unbound_or_incomplete_capture(inputs, mutation, match):
    timeline, raw, action, context, records = inputs
    if mutation == "missing_terminal":
        raw.write_bytes(b"DPRAW01\0" + b"".join(records[:4] + records[5:]))
    elif mutation == "cross_session":
        raw.write_bytes(b"DPRAW01\0" + b"".join(records[:2] +
            [record(1, 3500, 2, 9, 4, 0, records[2][39:])] + records[3:]))
    elif mutation == "ambiguous":
        conflicting = record(3, 2950, 9, 3, 0, 0,
            wire("DummyDeviceStatusV2", boot_epoch=9, session_id=3,
                 reference_epoch=1, motion_epoch=2, run_id=4, profile_id=201,
                 configured_mask=63, core_state=3))
        raw.write_bytes(b"DPRAW01\0" + records[0] + conflicting + b"".join(records[1:]))
    elif mutation == "truncated":
        raw.write_bytes(raw.read_bytes()[:-4])
    elif mutation == "hash":
        value = json.loads(context.read_text())
        value["raw_sha256"] = "0" * 64
        context.write_text(json.dumps(value))
    elif mutation == "missing_complete":
        timeline.write_text(timeline.read_text().replace("COMPLETE,1\n", ""))
    if mutation != "hash":
        repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match=match):
        joiner.join_evidence(timeline, raw, action, context)


def test_missing_external_metadata_fails_closed(inputs):
    timeline, raw, action, context, _ = inputs
    value = json.loads(context.read_text())
    del value["client_id"]
    context.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="MISSING_INPUT.*client_id"):
        joiner.join_evidence(timeline, raw, action, context)


def test_trajectory_digest_must_bind_to_gate(inputs):
    timeline, raw, action, context, _ = inputs
    value = json.loads(action.read_text())
    value["trajectory_sha256"] = "c" * 64
    action.write_text(json.dumps(value))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="trajectory"):
        joiner.join_evidence(timeline, raw, action, context)


def test_rejects_stale_ros_feedback_elapsed_time(inputs):
    timeline, raw, action, context, _ = inputs
    value = json.loads(action.read_text())
    value["feedback"][0]["elapsed_us"] = 1000000
    action.write_text(json.dumps(value))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="ROS feedback time"):
        joiner.join_evidence(timeline, raw, action, context)


def test_missing_action_clock_anchor_fails_closed(inputs):
    timeline, raw, action, context, _ = inputs
    value = json.loads(action.read_text())
    del value["action_started_steady_us"]
    action.write_text(json.dumps(value))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="MISSING_INPUT.*action_started_steady_us"):
        joiner.join_evidence(timeline, raw, action, context)


def test_release_page_before_motion_terminal_is_rejected(inputs):
    timeline, raw, action, context, records = inputs
    early_release = record(1, 4100, 2, 3, 5, 0, records[7][39:])
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        [*records[:4], early_release, records[4], records[5], records[6], *records[8:]]))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="RELEASE.*motion terminal"):
        joiner.join_evidence(timeline, raw, action, context)


def test_ros_feedback_and_raw_state_need_only_bounded_agreement(inputs):
    timeline, raw, action, context, _ = inputs
    report = json.loads(action.read_text())
    report["feedback"][0]["actual_rad"] = [0.0101] * 6
    action.write_text(json.dumps(report))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["feedback_rad"] == [0.0101] * 6


def test_same_cycle_feedback_receive_time_can_precede_after_begin(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[4][3] = "2950"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    prior_state = record(2, 2940, 2, 3, 0, 0, records[1][39:])
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        [records[0], prior_state, *records[1:]]))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["timeline"][3]["steady_us"] == 2950


def test_async_raw_pages_can_straddle_gate_callbacks(inputs):
    timeline, raw, action, context, records = inputs
    first = record(1, 2950, 2, 3, 4, 0, records[2][39:])
    terminal = record(1, 5500, 2, 3, 4, 1, records[4][39:])
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        [records[0], first, records[1], records[3], terminal, *records[5:]]))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["evidence_pages"][0]["raw_hex"] == records[2][39:].hex()


def test_final_ros_feedback_requires_nearby_gate_handoff(inputs):
    timeline, raw, action, context, _ = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    extra = copy.copy(rows[4])
    extra[3] = "4500"
    rows.insert(5, extra)
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    report = json.loads(action.read_text())
    report["feedback"][0]["elapsed_us"] = 2600
    action.write_text(json.dumps(report))
    metadata = json.loads(context.read_text())
    metadata["ros_feedback_lag_budget_us"] = 500
    context.write_text(json.dumps(metadata))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="ROS feedback time"):
        joiner.join_evidence(timeline, raw, action, context)


def test_gate_uses_prior_cached_state_and_status(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    for row, when in zip(rows[1:-1], (80000, 90000, 600000, 601000,
                                      602000, 603000, 604000)):
        row[3] = str(when)
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    times = (100000, 550000, 600500, 601000, 601500, 602900,
             603000, 603400, 603500, 604000, 604000)
    rewritten = [record(item[0], when, 2, 0 if index >= 9 else 3,
                        4 if index in (2, 4) else 5 if index in (7, 8) else 0,
                        1 if index in (4, 8) else 0, item[39:])
                 for index, (item, when) in enumerate(zip(records, times))]
    raw.write_bytes(b"DPRAW01\0" + b"".join(rewritten))
    report = json.loads(action.read_text())
    report["action_started_steady_us"] = 79000
    report["feedback"][0]["elapsed_us"] = 522000
    action.write_text(json.dumps(report))
    repin(timeline, raw, action, context)
    result = joiner.join_evidence(timeline, raw, action, context)
    assert result["goals"][0]["run_id"] == 4


def test_hold_rejects_unverified_later_run_despite_fresh_old_observations(inputs):
    timeline, raw, action, context, records = inputs
    rows = list(csv.reader(timeline.open(newline="")))
    rows[5][3], rows[6][3], rows[7][3] = "6800", "6900", "10000"
    with timeline.open("w", newline="") as output:
        csv.writer(output).writerows(rows)
    later_status = joiner._decode("DummyDeviceStatusV2", records[5][39:])
    later_status.update(run_id=5, final_state=0, motion_complete=0,
                        evidence_chain_sha256="00" * 32)
    records.insert(9, record(3, 6700, 2, 3, 0, 0,
                             wire("DummyDeviceStatusV2", **later_status)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="unverified.*successor"):
        joiner._joined_goal(
            joiner._timeline(timeline.read_bytes())[0],
            joiner._raw_records(raw.read_bytes()), json.loads(action.read_text()),
            json.loads(context.read_text()))


@pytest.mark.parametrize("change", ("stale", "missing_old_status", "missing_later_page",
                                   "wrong_hash", "wrong_session", "wrong_run",
                                   "early_state"))
def test_later_run_status_cannot_rescue_unverified_hold(inputs, change):
    _, _, _, _, raw_records = inputs
    records = joiner._raw_records(b"DPRAW01\0" + b"".join(raw_records))
    old_status, old_state, later_page = records[5], records[6], records[7]
    later_fields = {**old_status["fields"], "run_id": 5,
                    "final_state": 0, "motion_complete": 0}
    if change == "wrong_hash":
        later_fields["firmware_sha256"] = "22" * 32
    if change == "wrong_run":
        later_fields["run_id"] = 9
    later_status = {**old_status, "when": 6700, "fields": later_fields}
    if change == "wrong_session":
        later_status = {**later_status, "session": 7}
    if change == "early_state":
        old_state = {**old_state, "when": 4400}
    if change == "stale":
        old_status = {**old_status, "when": 51909203083,
                      "fields": {**old_status["fields"], "run_id": 8}}
        old_state = {**old_state, "when": 51909218740}
        later_page = {**later_page, "when": 51909239140, "run": 9}
        later_status = {**later_status, "when": 51909767985,
                        "fields": {**later_status["fields"], "run_id": 9}}
    selected = [old_status, old_state, later_page, later_status]
    if change == "missing_old_status":
        selected.remove(old_status)
    if change == "missing_later_page":
        selected.remove(later_page)
    identity = (2, 3, 1, 2)
    when = 51909770401 if change == "stale" else 6900
    with pytest.raises(ValueError, match="status|HOLD|identity"):
        joiner._hold_observations(selected, when, identity,
                                  8 if change == "stale" else 4, 500000,
                                  51909182709 if change == "stale" else 4500)


def refresh_hold_records(inputs):
    _, _, _, _, raw_records = inputs
    records = joiner._raw_records(b"DPRAW01\0" + b"".join(raw_records))
    old_terminal, old_status, old_state = records[4:7]
    old_status = {**old_status, "when": 5900}
    old_state = {**old_state, "when": 6000}
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    next_raw = page(identity, 5, 2, 0, b"", epoch=2, parent=0)
    next_fields = joiner._decode("DummyAttemptEvidenceV2", next_raw)
    next_fields["sent_at_us"] = [6200] * 6
    next_raw = bytearray(wire("DummyAttemptEvidenceV2", **next_fields))
    next_raw[339:371] = hashlib.sha256(
        next_raw[307:339] + next_raw[:307] + next_raw[371:372]).digest()
    next_raw = bytes(next_raw)
    next_page = {"kind": 1, "when": 6400, "boot": 2, "session": 3,
                 "run": 5, "seq": 0, "raw": next_raw,
                 "fields": joiner._decode("DummyAttemptEvidenceV2", next_raw)}
    next_status = {**old_status, "when": 400000,
                   "fields": {**old_status["fields"], "run_id": 5,
                              "attempt_id": 5, "parent_run_id": 0,
                              "final_state": 0, "motion_complete": 0,
                              "evidence_chain_sha256": "00" * 32}}
    next_state = {**old_state, "when": 400010,
                  "fields": {**old_state["fields"],
                             "produced_at_us": 6600, "state_seq": 2}}
    return [old_terminal, old_status, old_state, next_page,
            next_status, next_state]


def test_canceled_hold_terminal_and_successor_can_prove_missing_old_status(inputs):
    _, _, _, _, raw_records = inputs
    original = joiner._raw_records(b"DPRAW01\0" + b"".join(raw_records))
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    first = original[2]
    final_raw = page(identity, 4, 2, 1, first["raw"][339:371], canceled=True)
    terminal = {**original[4], "raw": final_raw,
                "fields": joiner._decode("DummyAttemptEvidenceV2", final_raw)}
    records = refresh_hold_records(inputs)
    records = [first, terminal, records[2], *records[3:]]

    status, state = joiner._hold_observations(
        records, 400020, (2, 3, 1, 2), 4, 250000, 4500, "11" * 16, 7)
    assert status["proof_source"] == "terminal_state_successor"
    assert status["fields"]["final_state"] == 7
    assert state["when"] == 6000


def completed_hold_without_old_status(inputs):
    _, _, _, _, raw_records = inputs
    original = joiner._raw_records(b"DPRAW01\0" + b"".join(raw_records))
    records = refresh_hold_records(inputs)
    return [original[2], records[0], records[2], *records[3:]]


def completed_hold_with_leading_group_counts(inputs):
    records = completed_hold_without_old_status(inputs)
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    first = bytearray(page(identity, 4, 2, 0, b"", group_count=3, group_id=7))
    struct.pack_into("<I", first, 281, 2)
    first[339:371] = hashlib.sha256(
        first[307:339] + first[:307] + first[371:372]).digest()
    second = page(identity, 4, 2, 1, first[339:371], group_count=3, group_id=8)
    third = page(identity, 4, 2, 2, second[339:371], group_count=3, group_id=9)
    terminal = page(identity, 4, 2, 3, third[339:371], group_count=3)
    pages = [(3000, first), (3500, second), (4000, third), (4500, terminal)]
    motion = [{"kind": 1, "when": when, "boot": 2, "session": 3,
               "run": 4, "seq": seq, "raw": bytes(raw),
               "fields": joiner._decode("DummyAttemptEvidenceV2", raw)}
              for seq, (when, raw) in enumerate(pages)]
    successor_raw = bytearray(records[3]["raw"])
    struct.pack_into("<I", successor_raw, 281, 2)
    successor_raw[339:371] = hashlib.sha256(
        successor_raw[307:339] + successor_raw[:307] +
        successor_raw[371:372]).digest()
    successor = {**records[3], "raw": bytes(successor_raw),
                 "fields": joiner._decode("DummyAttemptEvidenceV2", successor_raw)}
    return [*motion, records[2], successor, *records[4:]]


def test_completed_hold_accepts_leading_plateau_groups_and_successor_count_two(inputs):
    status, state = joiner._hold_observations(
        completed_hold_with_leading_group_counts(inputs), 400020,
        (2, 3, 1, 2), 4, 250000, 4500, "11" * 16)
    assert status["proof_source"] == "terminal_state_successor"
    assert state["when"] == 6000


@pytest.mark.parametrize("change", ("missing_page", "reordered_page",
                                   "decreasing_count", "wrong_terminal_total"))
def test_completed_hold_leading_groups_rejects_broken_sequence(inputs, change):
    records = completed_hold_with_leading_group_counts(inputs)
    if change == "missing_page":
        records.pop(1)
    elif change == "reordered_page":
        records[1], records[2] = records[2], records[1]
    elif change == "decreasing_count":
        records[1] = {**records[1], "fields": {
            **records[1]["fields"], "group_count": 1}}
    else:
        records[3] = {**records[3], "fields": {
            **records[3]["fields"], "group_count": 2}}
    with pytest.raises(ValueError):
        joiner._hold_observations(records, 400020, (2, 3, 1, 2),
                                  4, 250000, 4500, "11" * 16)


def test_completed_hold_terminal_and_successor_can_prove_missing_old_status(inputs):
    status, state = joiner._hold_observations(
        completed_hold_without_old_status(inputs), 400020,
        (2, 3, 1, 2), 4, 250000, 4500, "11" * 16)
    assert status["proof_source"] == "terminal_state_successor"
    assert (status["fields"]["run_id"], status["fields"]["final_state"]) == (4, 6)
    assert status["fields"]["motion_complete"] == 1
    assert state["when"] == 6000


def test_completed_hold_successor_state_can_prove_missing_intermediate_state(inputs):
    records = completed_hold_without_old_status(inputs)
    records.pop(2)
    status, state = joiner._hold_observations(
        records, 400020, (2, 3, 1, 2), 4, 250000, 4500, "11" * 16)
    assert status["proof_source"] == "terminal_state_successor"
    assert state["when"] == 400010
    records.pop()
    with pytest.raises(ValueError):
        joiner._hold_observations(
            records, 400020, (2, 3, 1, 2), 4, 250000, 4500, "11" * 16)


def test_completed_hold_rejects_delayed_pre_successor_state(inputs):
    records = completed_hold_without_old_status(inputs)
    records.pop(2)
    records[-1] = {**records[-1], "fields": {
        **records[-1]["fields"], "produced_at_us": 6100}}
    with pytest.raises(ValueError, match="successor|HOLD"):
        joiner._hold_observations(
            records, 400020, (2, 3, 1, 2), 4, 250000, 4500, "11" * 16)


@pytest.mark.parametrize("field,value", (("first_failure_source", 1),
                                         ("first_failure_axis_mask", 1),
                                         ("terminal_reason", 1)))
def test_completed_hold_rejects_rehashed_failure_terminal(inputs, field, value):
    records = completed_hold_without_old_status(inputs)
    raw = bytearray(records[1]["raw"])
    offset = {"first_failure_source": 38, "first_failure_axis_mask": 40,
              "terminal_reason": 305}[field]
    struct.pack_into("<B" if field == "first_failure_axis_mask" else "<H",
                     raw, offset, value)
    raw[339:371] = hashlib.sha256(
        raw[307:339] + raw[:307] + raw[371:372]).digest()
    records[1] = {**records[1], "raw": bytes(raw),
                  "fields": joiner._decode("DummyAttemptEvidenceV2", raw)}
    with pytest.raises(ValueError, match="terminal"):
        joiner._hold_observations(records, 400020, (2, 3, 1, 2),
                                  4, 250000, 4500, "11" * 16)


@pytest.mark.parametrize("change", ("missing_state", "foreign_state", "partial_state",
                                   "stale_state", "broken_terminal_chain",
                                   "partial_terminal", "wrong_group_count",
                                   "missing_first_page", "wrong_first_run",
                                   "broken_successor_chain", "foreign_status",
                                   "wrong_source_hash", "stale_successor_status"))
def test_completed_hold_without_old_status_rejects_incomplete_proof(inputs, change):
    records = completed_hold_without_old_status(inputs)
    if change == "missing_state":
        records.pop(5)
        records.pop(2)
    elif change == "foreign_state":
        records[2] = {**records[2], "session": 99}
    elif change == "partial_state":
        records[2] = {**records[2], "fields": {
            **records[2]["fields"], "position_valid_mask": 31}}
    elif change == "stale_state":
        records[2] = {**records[2], "when": 4501}
        records[3] = {**records[3], "when": 400000}
        records[5] = {**records[5], "when": 7000}
    elif change == "broken_terminal_chain":
        corrupted = bytearray(records[1]["raw"])
        corrupted[339] ^= 1
        records[1] = {**records[1], "raw": bytes(corrupted),
                      "fields": joiner._decode("DummyAttemptEvidenceV2", corrupted)}
    elif change == "partial_terminal":
        records[1] = {**records[1], "fields": {
            **records[1]["fields"], "final": 0}}
    elif change == "wrong_group_count":
        records[1] = {**records[1], "fields": {
            **records[1]["fields"], "group_count": 2}}
    elif change == "missing_first_page":
        records.pop(3)
    elif change == "wrong_first_run":
        records[3] = {**records[3], "run": 6}
    elif change == "broken_successor_chain":
        corrupted = bytearray(records[3]["raw"])
        corrupted[339] ^= 1
        records[3] = {**records[3], "raw": bytes(corrupted)}
    elif change == "foreign_status":
        records[4] = {**records[4], "session": 99}
    elif change == "wrong_source_hash":
        records[4] = {**records[4], "fields": {
            **records[4]["fields"], "firmware_sha256": "33" * 32}}
    else:
        records[4] = {**records[4], "when": 100000}
    with pytest.raises(ValueError):
        joiner._hold_observations(records, 400020, (2, 3, 1, 2),
                                  4, 250000, 4500, "11" * 16)


def test_canceled_hold_without_old_status_rejects_other_device_successor(inputs):
    _, _, _, _, raw_records = inputs
    original = joiner._raw_records(b"DPRAW01\0" + b"".join(raw_records))
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    first = original[2]
    final_raw = page(identity, 4, 2, 1, first["raw"][339:371], canceled=True)
    terminal = {**original[4], "raw": final_raw,
                "fields": joiner._decode("DummyAttemptEvidenceV2", final_raw)}
    records = [first, terminal, *refresh_hold_records(inputs)[2:]]
    successor_status = records[4]
    records[4] = {**successor_status, "fields": {
        **successor_status["fields"], "device_id": "33" * 16}}
    with pytest.raises(ValueError, match="device"):
        joiner._hold_observations(
            records, 400020, (2, 3, 1, 2), 4, 250000, 4500, "11" * 16, 7)


def test_goal_rejects_other_device_begin_status(inputs):
    timeline, raw, action, context, records = inputs
    status = joiner._decode("DummyDeviceStatusV2", records[0][39:])
    records[0] = record(3, 2900, 2, 3, 0, 0, wire(
        "DummyDeviceStatusV2", **{**status, "device_id": "33" * 16}))
    raw.write_bytes(b"DPRAW01\0" + b"".join(records))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="device"):
        joiner.join_evidence(timeline, raw, action, context)


@pytest.mark.parametrize("change", ("missing_state", "broken_terminal_chain",
                                   "wrong_successor_session", "missing_successor_page"))
def test_canceled_hold_without_old_status_rejects_incomplete_raw_proof(inputs, change):
    _, _, _, _, raw_records = inputs
    original = joiner._raw_records(b"DPRAW01\0" + b"".join(raw_records))
    identity = {"client_id": "11" * 16, **{key: "11" * 32 for key in
        ("contract_hash", "model_hash", "profile_hash", "firmware_hash")}}
    first = original[2]
    final_raw = page(identity, 4, 2, 1, first["raw"][339:371], canceled=True)
    terminal = {**original[4], "raw": final_raw,
                "fields": joiner._decode("DummyAttemptEvidenceV2", final_raw)}
    records = [first, terminal, *refresh_hold_records(inputs)[2:]]
    if change == "missing_state":
        records.pop(5)
        records.pop(2)
    elif change == "broken_terminal_chain":
        corrupted = bytearray(final_raw)
        corrupted[339] ^= 1
        records[1] = {**terminal, "raw": bytes(corrupted),
                      "fields": joiner._decode("DummyAttemptEvidenceV2", bytes(corrupted))}
    elif change == "wrong_successor_session":
        records[4] = {**records[4], "session": 99}
    else:
        records.pop(3)
    with pytest.raises(ValueError):
        joiner._hold_observations(
            records, 400020, (2, 3, 1, 2), 4, 250000, 4500, "11" * 16, 7)


def test_stale_old_status_can_close_verified_fresh_refresh_hold(inputs):
    records = refresh_hold_records(inputs)
    status, state = joiner._hold_observations(
        records, 400020, (2, 3, 1, 2), 4, 250000, 4500, "11" * 16)
    assert (status["fields"]["run_id"], state["when"]) == (4, 6000)


@pytest.mark.parametrize("change", ("valid", "broken_chain", "missing_axis",
                                   "wrong_attempt_run", "wrong_epoch"))
def test_fresh_old_hold_still_verifies_visible_successor(inputs, change):
    records = refresh_hold_records(inputs)
    records[4] = {**records[4], "when": 6800}
    records[5] = {**records[5], "when": 6810}
    if change == "broken_chain":
        raw = records[3]["raw"]
        records[3]["raw"] = raw[:339] + bytes([raw[339] ^ 1]) + raw[340:]
    elif change == "missing_axis":
        records[3]["fields"]["sent_mask"] = 31
    elif change == "wrong_attempt_run":
        records[3]["fields"]["attempt_id"] = 6
    elif change == "wrong_epoch":
        records[3]["fields"]["motion_epoch"] = 3
    if change == "valid":
        status, state = joiner._hold_observations(
            records, 6900, (2, 3, 1, 2), 4, 250000, 4500, "11" * 16)
        assert (status["fields"]["run_id"], state["when"]) == (4, 6000)
    else:
        with pytest.raises(ValueError):
            joiner._hold_observations(records, 6900, (2, 3, 1, 2),
                                      4, 250000, 4500, "11" * 16)


@pytest.mark.parametrize("change", ("wrong_parent", "wrong_epoch", "wrong_chain",
                                   "stale_old_status", "invalid_next_run",
                                   "stale_next_status", "invalid_next_state",
                                   "missing_axis", "broken_successor_chain",
                                   "missing_submitted_axis", "failed_axis",
                                   "incomplete_group"))
def test_refresh_hold_rejects_unlinked_or_stale_successor(inputs, change):
    records = refresh_hold_records(inputs)
    if change == "wrong_parent":
        records[3]["fields"]["parent_run_id"] = 4
    elif change == "wrong_epoch":
        records[3]["fields"]["motion_epoch"] = 3
    elif change == "wrong_chain":
        records[1]["fields"]["evidence_chain_sha256"] = "00" * 32
    elif change == "stale_old_status":
        records[3]["when"] = 300000
        records[2]["when"] = 299990
    elif change == "invalid_next_run":
        records[4]["fields"]["run_id"] = 6
    elif change == "stale_next_status":
        records[4]["when"] = 400020 - 250001
    elif change == "invalid_next_state":
        records[5]["fields"]["hold_ready"] = 0
    elif change == "missing_axis":
        records[5]["fields"]["position_valid_mask"] = 31
    elif change == "broken_successor_chain":
        raw = records[3]["raw"]
        records[3]["raw"] = raw[:339] + bytes([raw[339] ^ 1]) + raw[340:]
    elif change == "missing_submitted_axis":
        records[3]["fields"]["submitted_mask"] = 31
    elif change == "failed_axis":
        records[3]["fields"]["failed_mask"] = 1
    elif change == "incomplete_group":
        records[3]["fields"]["final"] = 0
    with pytest.raises(ValueError):
        joiner._hold_observations(records, 400020, (2, 3, 1, 2),
                                  4, 250000, 4500, "11" * 16)


def test_future_only_raw_status_cannot_confirm_begin(inputs):
    timeline, raw, action, context, records = inputs
    late_status = record(3, 3100, 2, 3, 0, 0, records[0][39:])
    raw.write_bytes(b"DPRAW01\0" + b"".join([records[1], late_status, *records[2:]]))
    repin(timeline, raw, action, context)
    with pytest.raises(ValueError, match="prior.*status"):
        joiner.join_evidence(timeline, raw, action, context)
