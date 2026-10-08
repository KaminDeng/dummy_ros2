#!/usr/bin/env python3
"""将同一次 ROS Gate 时间线与 DeviceTransport DPRAW01 原始流离线关联。"""

import argparse
from bisect import bisect_left, bisect_right
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import struct
from types import MappingProxyType


ROOT = Path(__file__).resolve().parents[2]
LAYOUT = json.loads((ROOT / "contracts/dummy_robot_v2/codec/wire_layout.json").read_text())
TYPES = {"uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q",
         "float64": "d", "bool": "B"}
RECORD_HEADER = struct.Struct("<B H Q Q Q Q I")
STATUS_MAX_AGE_US = 1000000


def _snapshot(path, expected=None):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        before = os.fstat(fd)
        with os.fdopen(fd, "rb", closefd=False) as source:
            data = source.read()
        after = os.fstat(fd)
    finally:
        os.close(fd)
    if ((before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns,
         before.st_ctime_ns) !=
            (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns,
             after.st_ctime_ns) or len(data) != after.st_size):
        raise ValueError(f"file changed during read: {path}")
    digest = hashlib.sha256(data).hexdigest()
    if expected is not None and (not isinstance(expected, str) or len(expected) != 64 or
                                 digest != expected):
        raise ValueError(f"file hash mismatch: {path}")
    return data, digest


def _decode(name, raw):
    layout = LAYOUT["payloads"][name]
    if len(raw) != layout["encoded_size"]:
        raise ValueError(f"invalid {name} wire size")
    result = {}
    for field in layout["fields"]:
        count = field["count"]
        values = struct.unpack_from("<" + TYPES[field["type"]] * count,
                                    raw, field["offset"])
        result[field["name"]] = (bytes(values).hex() if count > 1 and
            field["type"] == "uint8" else list(values) if count > 1 else values[0])
    if (result["schema_major"], result["schema_minor"]) != (2, 4):
        raise ValueError(f"invalid {name} schema")
    return result


def _timeline(data):
    rows = list(csv.reader(io.StringIO(data.decode("utf-8", errors="strict"))))
    if (not rows or len(rows[-1]) != 2 or rows[-1][0] != "COMPLETE" or
            not rows[-1][1].isdigit() or int(rows[-1][1]) <= 0):
        raise ValueError("missing Gate COMPLETE footer")
    goal_count = int(rows[-1][1])
    header = rows[0]
    required = {"event", "goal_uuid", "trajectory_sha256", "steady_us", "boot",
                "session", "reference_epoch", "motion_epoch", "run_id", "axis_mask",
                "status_core_state", "state_core_state"}
    release_columns = {"release_request_id", "release_operation_id"}
    if release_columns.issubset(header):
        required.update(release_columns)
    elif release_columns.intersection(header):
        raise ValueError("partial Gate release request key")
    cancel_columns = {"cancel_request_id", "cancel_operation_id"}
    if cancel_columns.issubset(header):
        required.update(cancel_columns)
    elif cancel_columns.intersection(header):
        raise ValueError("partial Gate cancel request key")
    required.update(f"feedback_seq_{i}" for i in range(6))
    required.update(f"feedback_age_us_{i}" for i in range(6))
    if len(header) != len(set(header)) or not required.issubset(header):
        raise ValueError("invalid Gate timeline header")
    if len(rows) < 9 or any(len(row) != len(header) for row in rows[1:-1]):
        raise ValueError("incomplete Gate timeline")
    output = []
    for values in rows[1:-1]:
        source = dict(zip(header, values))
        row = {key: int(source[key]) for key in required -
               {"event", "goal_uuid", "trajectory_sha256"} -
               {f"feedback_seq_{i}" for i in range(6)} -
               {f"feedback_age_us_{i}" for i in range(6)}}
        row.update(event=source["event"], goal_uuid=source["goal_uuid"],
                   trajectory_sha256=source["trajectory_sha256"])
        row["boot_epoch"] = row.pop("boot")
        row["session_id"] = row.pop("session")
        if row["event"] == "feedback":
            row["feedback_seq"] = [int(source[f"feedback_seq_{i}"]) for i in range(6)]
            row["feedback_age_us"] = [int(source[f"feedback_age_us_{i}"]) for i in range(6)]
        output.append(row)
    groups = []
    current = []
    for row in output:
        if row["event"] == "accepted" and current:
            groups.append(current)
            current = []
        current.append(row)
    if current:
        groups.append(current)
    if len(groups) != goal_count:
        raise ValueError("Gate COMPLETE goal count mismatch")
    for index, group in enumerate(groups):
        events = [row["event"] for row in group]
        tail = ["pre_release_hold"]
        if index == goal_count - 1:
            tail.append("post_release_status")
        terminal_event = events[-len(tail) - 1] if len(events) > len(tail) else None
        if (events[:3] != ["accepted", "before_begin", "after_begin"] or
                terminal_event not in ("terminal_success", "terminal_canceled") or
                events[-len(tail):] != tail or
                not events[3:-len(tail) - 1] or
                any(event != "feedback" for event in events[3:-len(tail) - 1])):
            raise ValueError("incomplete Gate event sequence")
        if (len({row["goal_uuid"] for row in group}) != 1 or
                len({row["trajectory_sha256"] for row in group}) != 1):
            raise ValueError("ambiguous Gate goal binding")
    if len({group[0]["goal_uuid"] for group in groups}) != goal_count:
        raise ValueError("duplicate Gate goal UUID")
    for group in groups:
        for row in group[:-1]:
            if row.get("release_request_id", 0) != 0 or row.get("release_operation_id", 0) != 0:
                raise ValueError("release key appears before final Gate status")
        for row in group:
            request_id = row.get("cancel_request_id", 0)
            operation_id = row.get("cancel_operation_id", 0)
            if row["event"] == "terminal_canceled":
                if request_id <= 0 or operation_id <= 0:
                    raise ValueError("missing Gate cancel request key")
            elif request_id != 0 or operation_id != 0:
                raise ValueError("cancel key appears outside Gate canceled terminal")
        final = group[-1]
        if final["event"] == "post_release_status" and (
                final.get("release_request_id", 0) <= 0 or
                final.get("release_operation_id", 0) <= 0):
            raise ValueError("missing final Gate release request key")
    return groups


class _IndexedRecords(tuple):
    def __new__(cls, records):
        return super().__new__(cls, (MappingProxyType(record.copy()) for record in records))

    def __init__(self, records):
        self.by_kind = MappingProxyType({
            kind: tuple(record for record in self if record["kind"] == kind)
            for kind in (2, 3)})
        self.times_by_kind = MappingProxyType({
            kind: tuple(record["when"] for record in items)
            for kind, items in self.by_kind.items()})


def _raw_records(data):
    if not data.startswith(b"DPRAW01\0"):
        raise ValueError("invalid DPRAW01 header")
    records = []
    offset = 8
    while offset < len(data):
        if len(data) - offset < RECORD_HEADER.size:
            raise ValueError("truncated DPRAW01 record header")
        kind, size, when, boot, session, run, seq = RECORD_HEADER.unpack_from(data, offset)
        offset += RECORD_HEADER.size
        if kind not in (1, 2, 3, 4, 5) or size == 0 or len(data) - offset < size:
            raise ValueError("truncated or invalid DPRAW01 record")
        raw = data[offset:offset + size]
        offset += size
        name = {1: "DummyAttemptEvidenceV2", 2: "DummyJointStateV2",
                3: "DummyDeviceStatusV2", 4: "DummyManageV2Request",
                5: "DummyManageV2Response"}[kind]
        fields = _decode(name, raw)
        if kind == 1:
            if fields["run_id"] != run or fields["evidence_seq"] != seq:
                raise ValueError("unlinked evidence record metadata")
        elif kind in (2, 3, 5) and (fields["boot_epoch"], fields["session_id"]) != (boot, session):
            raise ValueError("unlinked state/status record metadata")
        if kind == 4 and (boot, session, run, seq) != (
                fields["expected_boot_epoch"], fields["expected_session_id"],
                0, fields["request_seq"]):
            raise ValueError("unlinked management request metadata")
        if kind == 5 and (run, seq) != (fields["run_id"], 0):
            raise ValueError("unlinked management response metadata")
        records.append({"kind": kind, "when": when, "boot": boot,
                        "session": session, "run": run, "seq": seq,
                        "fields": fields, "raw": raw})
    if any(right["when"] < left["when"] for left, right in zip(records, records[1:])):
        raise ValueError("nonmonotonic DPRAW01 receive times")
    return _IndexedRecords(records)


def _latest(records, kind, when):
    if isinstance(records, _IndexedRecords) and kind in records.by_kind:
        index = bisect_right(records.times_by_kind[kind], when) - 1
        return records.by_kind[kind][index] if index >= 0 else None
    return next((record for record in reversed(records) if
                 record["kind"] == kind and record["when"] <= when), None)


def _prior(records, kind, when, identity, max_age_us, *, run=None):
    latest = _latest(records, kind, when)
    name = "status" if kind == 3 else "state"
    if latest is None or when - latest["when"] > max_age_us:
        raise ValueError(f"missing prior raw {name} observation")
    fields = latest["fields"]
    key = (latest["boot"], latest["session"], fields["reference_epoch"],
           fields["motion_epoch"])
    if key != identity or (run is not None and fields["run_id"] != run):
        raise ValueError(f"ambiguous prior raw {name} identity")
    return latest


def _hold_observations(records, when, identity, run, state_max_age, terminal_when,
                       client_id=None, final_state=6):
    latest_status = _prior(records, 3, when, identity, STATUS_MAX_AGE_US)
    if latest_status["fields"]["run_id"] == run:
        return latest_status, _prior(records, 2, when, identity, state_max_age)

    later_run = latest_status["fields"]["run_id"]
    later_pages = [record for record in records if record["kind"] == 1 and
                   record["run"] == later_run and record["boot"] == identity[0] and
                   record["session"] == identity[1] and record["when"] <= latest_status["when"]]
    if later_run <= run or not later_pages:
        raise ValueError("ambiguous prior raw status identity")
    cutoff = later_pages[0]["when"]
    terminal = next((record for record in records if record["kind"] == 1 and
                     record["run"] == run and record["when"] == terminal_when and
                     record["fields"]["category"] == 4), None)
    old_statuses = [record for record in records if record["kind"] == 3 and
                    record["fields"]["run_id"] == run and
                    terminal_when <= record["when"] < cutoff and
                    cutoff - record["when"] <= state_max_age]
    old_states = [record for record in records if record["kind"] == 2 and
                  terminal_when <= record["when"] < cutoff and
                  cutoff - record["when"] <= state_max_age]
    successor = later_pages[0]
    if terminal is None or (not old_states and
                           (final_state not in (6, 7) or client_id is None)):
        raise ValueError("missing fresh completed-run HOLD observations")
    fresh_state = _prior(records, 2, when, identity, state_max_age)
    next_status = latest_status["fields"]
    if old_statuses:
        status = old_statuses[-1]
    elif final_state in (6, 7) and client_id is not None:
        # The successor can be published before the completed run's Status.
        # Authenticate its terminal chain before deriving the missing snapshot.
        pages = _pages(records, run, identity[0], identity[1], 1, 0)
        seed = struct.pack("<QQ", run, identity[0]) + bytes.fromhex(client_id)
        seed += b"".join(bytes.fromhex(next_status[name]) for name in
                         ("contract_sha256", "model_sha256", "profile_sha256",
                          "firmware_sha256"))
        previous_chain = hashlib.sha256(seed.ljust(192, b"\0")).digest()
        previous_group_count = 0
        total_groups = len(pages) - 1
        for index, item in enumerate(pages):
            raw_page = item["raw"]
            fields = item["fields"]
            if (len(raw_page) != 372 or raw_page[307:339] != previous_chain or
                    raw_page[371] != 0 or
                    raw_page[339:371] != hashlib.sha256(
                        previous_chain + raw_page[:307] + raw_page[371:372]).digest() or
                    fields["group_count"] < previous_group_count or
                    fields["group_count"] > total_groups or
                    (index < len(pages) - 1 and
                     (fields["group_count"] < index + 1 or
                      (fields["expected_mask"], fields["submitted_mask"],
                      fields["sent_mask"], fields["failed_mask"], fields["final"]) !=
                      (63, 63, 63, 0, 1))) or
                    (index < len(pages) - 1 and fields["category"] not in
                     ((1,) if final_state == 6 else (1, 2))) or
                    (index < len(pages) - 1 and fields["group_id"] == 0)):
                raise ValueError("unverified completed-run terminal chain")
            previous_chain = raw_page[339:371]
            previous_group_count = fields["group_count"]
        terminal_fields = terminal["fields"]
        if (pages[-1] is not terminal or terminal["boot"] != identity[0] or
                terminal["session"] != identity[1] or
                terminal_fields["final_state"] != final_state or
                terminal_fields["motion_complete"] != int(final_state == 6) or
                terminal_fields["release_committed"] or
                (final_state == 6 and
                 (terminal_fields["first_failure_source"] != 0 or
                  terminal_fields["first_failure_axis_mask"] != 0 or
                  terminal_fields["terminal_reason"] != 0)) or
                terminal_fields["group_id"] != 0 or
                terminal_fields["group_count"] != len(pages) - 1 or
                terminal_fields["expected_mask"] != 0 or
                terminal_fields["submitted_mask"] != 0 or
                terminal_fields["sent_mask"] != 0 or
                terminal_fields["failed_mask"] != 0 or
                terminal_fields["final"] != 1):
            raise ValueError("unlinked completed-run terminal page")
        proof_fields = {**next_status, "run_id": run, "attempt_id": run,
                        "core_state": 4, "attempt_kind": 1, "parent_run_id": 0,
                        "final_state": final_state,
                        "motion_complete": int(final_state == 6),
                        "evidence_lost": 0,
                        "evidence_chain_sha256": terminal["fields"]["chain_sha256"]}
        status = {"when": terminal_when, "boot": identity[0],
                  "session": identity[1], "fields": proof_fields,
                  "proof_source": "terminal_state_successor"}
    else:
        raise ValueError("missing fresh completed-run HOLD observations")
    state = old_states[-1] if old_states else fresh_state
    if not old_states:
        sent_times = successor["fields"]["sent_at_us"]
        produced = fresh_state["fields"]["produced_at_us"]
        if (len(sent_times) != 6 or any(type(stamp) is not int or stamp <= 0
                                        for stamp in sent_times) or
                type(produced) is not int or produced <= max(sent_times)):
            raise ValueError("unverified successor HOLD State production")
    old = status["fields"]
    new_state = fresh_state["fields"]
    page = successor["fields"]
    if client_id is None:
        raise ValueError("missing client identity for HOLD refresh successor")
    seed = struct.pack("<QQ", later_run, identity[0]) + bytes.fromhex(client_id)
    seed += b"".join(bytes.fromhex(old[name]) for name in
                     ("contract_sha256", "model_sha256", "profile_sha256",
                      "firmware_sha256"))
    previous = hashlib.sha256(seed.ljust(192, b"\0")).digest()
    raw = successor["raw"]
    if (later_run != run + 1 or successor["when"] <= terminal_when or
            successor["seq"] != 0 or page["evidence_seq"] != 0 or
            page["attempt_id"] != later_run or page["attempt_kind"] != 1 or
            page["parent_run_id"] != 0 or page["category"] != 1 or
            page["motion_epoch"] != identity[3] or page["evidence_lost"] or
            page["group_id"] == 0 or page["group_count"] < 1 or
            page["expected_mask"] != 63 or page["submitted_mask"] != 63 or
            page["sent_mask"] != 63 or page["failed_mask"] != 0 or
            page["final"] != 1 or page["final_state"] != 0 or
            page["motion_complete"] or page["release_committed"] or
            len(raw) != 372 or raw[371] != 0 or raw[307:339] != previous or
            raw[339:371] != hashlib.sha256(
                previous + raw[:307] + raw[371:372]).digest() or
            terminal["fields"]["final_state"] != final_state or
            terminal["fields"]["motion_complete"] != (final_state == 6) or
            old["evidence_chain_sha256"] != terminal["fields"]["chain_sha256"] or
            (old["core_state"], old["final_state"], old["motion_complete"],
             old["attempt_kind"], old["parent_run_id"], old["evidence_lost"]) !=
            (4, final_state, int(final_state == 6), 1, 0, 0) or
            (state["fields"]["core_state"], state["fields"]["hold_ready"],
             state["fields"]["position_valid_mask"]) != (4, 1, 63) or
            when - latest_status["when"] > state_max_age or
            latest_status["when"] < cutoff or fresh_state["when"] < cutoff or
            (next_status["core_state"], next_status["final_state"],
             next_status["motion_complete"], next_status["attempt_kind"],
             next_status["parent_run_id"], next_status["evidence_lost"]) !=
            (4, 0, 0, 1, 0, 0) or
            (new_state["core_state"], new_state["hold_ready"],
             new_state["position_valid_mask"]) != (4, 1, 63) or
            any(next_status[name] != old[name] for name in
                ("contract_sha256", "model_sha256", "profile_sha256", "firmware_sha256"))):
        raise ValueError("unverified completed-run HOLD refresh successor")
    for candidate in (status, state):
        fields = candidate["fields"]
        if (candidate["boot"], candidate["session"], fields["reference_epoch"],
                fields["motion_epoch"]) != identity:
            raise ValueError("ambiguous prior raw HOLD identity")
    device_id = latest_status["fields"]["device_id"]
    if any(candidate["fields"]["device_id"] != device_id for candidate in
           (status, state, fresh_state)):
        raise ValueError("HOLD successor changed device identity")
    if any(latest_status["fields"][name] != status["fields"][name] for name in
           ("contract_sha256", "model_sha256", "profile_sha256", "firmware_sha256")):
        raise ValueError("device identity hash changed before later run status")
    return status, state


def _six(values, name):
    if not isinstance(values, list) or len(values) != 6 or any(
            type(value) not in (int, float) or not math.isfinite(value) for value in values):
        raise ValueError(f"invalid six-axis {name}")
    return values


def _desired_at(samples, when):
    index = bisect_left(samples, when, key=lambda sample: sample[0])
    if index == 0:
        return samples[0][1]
    if index == len(samples):
        return samples[-1][1]
    start_time, start = samples[index - 1]
    end_time, end = samples[index]
    fraction = (when - start_time) / (end_time - start_time)
    return [a + fraction * (b - a) for a, b in zip(start, end)]


def _raw_ros_handoff(records, sample_time, sample_state, actual, identity,
                     ros_budget, path_tolerance, raw_states, gate_feedback,
                     feedback_age_budget):
    following = next((record for record in records if record["kind"] == 2 and
                      sample_time < record["when"]), None)
    if (following is None or following["when"] - sample_time > ros_budget or
            following not in raw_states):
        return False
    fields = following["fields"]
    if ((following["boot"], following["session"], fields["reference_epoch"],
         fields["motion_epoch"]) != identity or fields["position_valid_mask"] != 63 or
            fields["state_seq"] == 0 or fields["produced_at_us"] == 0 or
            fields["core_state"] not in (4, 5) or
            any(age > feedback_age_budget for age in fields["can_rx_age_us"])):
        return False
    old = sample_state["fields"]
    seq_delta = (fields["state_seq"] - old["state_seq"]) & 0xffffffff
    if (fields["motion_epoch"] != old["motion_epoch"] or
            not 0 < seq_delta < 0x80000000 or
            fields["produced_at_us"] < old["produced_at_us"] or
            any(new <= previous for new, previous in
                zip(fields["feedback_seq"], old["feedback_seq"]))):
        return False
    position = _six(fields["position_rad"], "raw ROS handoff")
    if any(abs(observed - measured) > limit for observed, measured, limit in
           zip(actual, position, path_tolerance)):
        return False
    continuation = next(((when, state) for when, state in gate_feedback if
                         when >= following["when"]), None)
    if continuation is None or continuation[0] - following["when"] > ros_budget:
        return False
    gate_fields = continuation[1]["fields"]
    if gate_fields["core_state"] not in (fields["core_state"], 4) or any(
            age > feedback_age_budget for age in gate_fields["can_rx_age_us"]):
        return False
    gate_seq = gate_fields["feedback_seq"]
    return (all(new >= previous for new, previous in zip(gate_seq, fields["feedback_seq"]))
            and any(new > previous for new, previous in
                    zip(gate_seq, fields["feedback_seq"])))


def _goal_raw_states(records, before, begin, hold, boot, session, reference,
                     allowed_epochs, desired_samples, path_tolerance):
    states = [record for record in records if record["kind"] == 2 and
              record["boot"] == boot and record["session"] == session and
              record["fields"]["reference_epoch"] == reference and
              before <= record["when"] <= hold]
    previous = {}
    last_epoch = None
    path_error = [0.0] * 6
    for record in states:
        fields = record["fields"]
        epoch = fields["motion_epoch"]
        if epoch not in allowed_epochs or (last_epoch is not None and epoch < last_epoch):
            raise ValueError("raw State outside goal authority epochs")
        last_epoch = epoch
        if fields["state_seq"] == 0 or fields["produced_at_us"] == 0:
            raise ValueError("raw State missing producer sequence or timestamp")
        earlier = previous.get(epoch)
        if earlier is not None:
            difference = (fields["state_seq"] - earlier["state_seq"]) & 0xffffffff
            if not 0 < difference < 0x80000000:
                raise ValueError("raw State sequence replay")
            if fields["produced_at_us"] < earlier["produced_at_us"]:
                raise ValueError("raw State producer time regression")
        previous[epoch] = fields
        if record["when"] < begin:
            continue
        if fields["position_valid_mask"] != 63:
            raise ValueError("invalid raw State path position")
        position = _six(fields["position_rad"], "raw State path")
        desired = _desired_at(desired_samples, record["when"])
        path_error = [max(old, abs(actual - planned)) for old, actual, planned in
                      zip(path_error, position, desired)]
        if any(error > limit for error, limit in zip(path_error, path_tolerance)):
            raise ValueError("raw State outside bounded trajectory path")
    return states, path_error


def _near_ros_state(records, sample_time, identity, lag_budget, age_budget):
    states = (record for record in records if record["kind"] == 2 and
              abs(record["when"] - sample_time) <= lag_budget)
    nearest = min(states, key=lambda record: abs(record["when"] - sample_time),
                  default=None)
    if nearest is None:
        raise ValueError("missing fresh raw State near final ROS feedback")
    fields = nearest["fields"]
    if ((nearest["boot"], nearest["session"], fields["reference_epoch"],
         fields["motion_epoch"]) != identity or fields["position_valid_mask"] != 63 or
            fields["state_seq"] == 0 or fields["produced_at_us"] == 0 or
            any(seq <= 0 for seq in fields["feedback_seq"]) or
            any(age > age_budget for age in fields["can_rx_age_us"])):
        raise ValueError("unlinked or stale raw State near final ROS feedback")
    return nearest


def _pages(records, run, boot, session, kind, parent):
    pages = [record for record in records if record["kind"] == 1 and record["run"] == run]
    if len(pages) < 2:
        raise ValueError("missing CAN or terminal evidence page")
    for index, page in enumerate(pages):
        fields = page["fields"]
        if ((page["boot"], page["session"], page["seq"]) != (boot, session, index) or
                (fields["attempt_kind"], fields["parent_run_id"], fields["attempt_id"]) !=
                (kind, parent, run) or fields["evidence_lost"]):
            raise ValueError("cross-session or incomplete evidence pages")
        if (fields["category"] == 4) != (index == len(pages) - 1):
            raise ValueError("missing or duplicate terminal evidence page")
    return pages


def _context(path):
    data, digest = _snapshot(Path(path))
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("MISSING_INPUT: external context object")
    required = ("timeline_sha256", "raw_sha256", "action_sha256", "client_id",
                "path_tolerance_rad", "goal_tolerance_rad", "producer_lag_us",
                "producer_lag_budget_us", "feedback_age_budget_us",
                "ros_feedback_lag_budget_us", "gate_state_max_age_us")
    for key in required:
        if key not in value or value[key] is None:
            raise ValueError(f"MISSING_INPUT: {key}")
    return value, digest


def _verify_window_context(context, timeline_data, records):
    if "producer_lag_gate_version" not in context:
        return
    try:
        from scripts.dummy import build_real_6dof_context as builder
    except ImportError:
        import build_real_6dof_context as builder
    if (context["producer_lag_gate_version"] != 2 or
            context.get("producer_lag_method") != builder.WINDOW_LAG_METHOD or
            context["producer_lag_budget_us"] != 250000):
        raise ValueError("invalid window producer lag version")
    start, end, window_states, release_run = builder._motion_window(
        timeline_data, records, context["client_id"], context["gate_state_max_age_us"])
    states = [item for item in records if item["kind"] == 2]
    produced = [item["fields"]["produced_at_us"] for item in states]
    if (any(stamp <= 0 for stamp in produced) or
            any(right < left for left, right in zip(produced, produced[1:]))):
        raise ValueError("nonmonotonic window producer lag State time")
    environment_offsets = [item["when"] - item["fields"]["produced_at_us"]
                           for item in states]
    offsets = [item["when"] - item["fields"]["produced_at_us"]
               for item in window_states]
    expected_window = {"start_steady_us": start, "end_steady_us": end,
                       "state_samples": len(window_states), "release_run_id": release_run}
    if (context.get("producer_lag_window") != expected_window or
            context.get("producer_lag_environment_us") !=
            max(environment_offsets) - min(environment_offsets) or
            context["producer_lag_us"] != max(offsets) - min(offsets)):
        raise ValueError("unverified window producer lag context")
    if context["producer_lag_us"] > context["producer_lag_budget_us"]:
        raise ValueError("window producer lag budget exceeded")


def _unconfirmed_cancel_observation(action):
    release = action.get("release_observation")
    return (action.get("status") == "UNCONFIRMED_STOP" and
            action.get("stop_confirmation") == "UNCONFIRMED_STOP" and
            action.get("action_observation") == "CANCELED" and
            action.get("action_status") == 5 and
            action.get("result_error_code") == 0 and
            action.get("safety_verdict") == "NOT_QUALIFIED" and
            action.get("raw_stop_release_join") == "REQUIRED_SEPARATELY" and
            action.get("device_can_evidence") == "REQUIRED_SEPARATELY" and
            action.get("device_release_evidence") == "REQUIRED_SEPARATELY" and
            action.get("device_qualification") == "NOT_CLAIMED" and
            isinstance(release, dict) and
            release.get("request_reply") == "RELEASE_QUEUED" and
            release.get("bootstrap_reply") == "RELEASED_IDLE")


def _preempt_probe(action, rows, records, goal, release, client):
    reported = action.get("preempt_goals")
    mirrored = action.get("goals")
    cancel = action.get("cancel")
    overlap = cancel.get("overlap_probe") if isinstance(cancel, dict) else None
    if (not isinstance(reported, dict) or set(reported) != {"A", "B", "C"} or
            mirrored != reported or not isinstance(overlap, dict)):
        raise ValueError("ambiguous ROS preempt goal report")
    a, b, c = (reported[key] for key in ("A", "B", "C"))
    ids = [item.get("goal_id") for item in (a, b, c) if isinstance(item, dict)]
    if (len(ids) != 3 or any(not isinstance(value, str) or len(value) != 32 or
                            any(char not in "0123456789abcdef" for char in value)
                            for value in ids) or len(set(ids)) != 3 or
            ids[0] != goal["goal_id"] or a.get("status") != "CANCELED" or
            b.get("status") != "REJECTED" or
            b.get("phase") != "A_MOTION_FEEDBACK_OVERLAP" or
            c.get("status") != "NOT_SUBMITTED" or
            overlap != b or goal["action_status"] != "CANCELED" or
            action.get("second_begin_raw_join") != "REQUIRED_SEPARATELY" or
            action.get("qualification") != "NOT_QUALIFIED"):
        raise ValueError("unlinked or incomplete ROS preempt observations")

    requests = [item for item in records if item["kind"] == 4 and
                item["fields"]["operation"] == 7]
    if len(requests) != 1:
        raise ValueError("missing or ambiguous BeginMotion request")
    request = requests[0]
    fields = request["fields"]
    before, after = rows[1], rows[2]
    binding = ("device_id", "client_instance_id", "expected_boot_epoch",
               "expected_session_id", "expected_reference_epoch",
               "expected_motion_epoch", "expected_management_epoch",
               "expected_profile_id", "configured_mask")
    if (not before["steady_us"] <= request["when"] <= after["steady_us"] or
            (request["boot"], request["session"]) !=
            (goal["boot_epoch"], goal["session_id"]) or
            fields["client_instance_id"] != client or
            (fields["expected_reference_epoch"], fields["expected_motion_epoch"],
             fields["expected_profile_id"], fields["configured_mask"]) !=
            (before["reference_epoch"], before["motion_epoch"], 201, 63) or
            fields["request_id"] <= 0):
        raise ValueError("BeginMotion request does not bind A")
    queries = [item for item in records if item["kind"] == 4 and
               item["fields"]["operation"] == 12 and
               item["fields"]["queried_operation_id"] == fields["request_id"] and
               request["when"] <= item["when"] <= after["steady_us"] and
               all(item["fields"][key] == fields[key] for key in binding)]
    response_ids = {fields["request_id"],
                    *(item["fields"]["request_id"] for item in queries)}
    receipts = [item for item in records if item["kind"] == 5 and
                item["fields"]["request_id"] in response_ids and
                request["when"] <= item["when"] and
                item["fields"]["transaction_phase"] == 3 and
                item["fields"]["result_code"] == 0]
    if (len(receipts) != 1 or receipts[0]["when"] > after["steady_us"] or
            receipts[0]["fields"]["receipt"] != 1 or
            receipts[0]["fields"]["operation_id"] <= 0 or
            (receipts[0]["boot"], receipts[0]["session"],
             receipts[0]["fields"]["motion_epoch"]) !=
            (goal["boot_epoch"], goal["session_id"], goal["motion_epoch"]) or
            receipts[0]["fields"]["run_id"] not in (0, goal["run_id"]) or
            receipts[0]["fields"]["client_instance_id"] != client or
            receipts[0]["fields"]["device_id"] != fields["device_id"]):
        raise ValueError("BeginMotion lacks one committed A receipt")
    runs = {item["run"] for item in records if item["kind"] == 1}
    if (runs != {goal["run_id"], release["run_id"]} or
            release["run_id"] != goal["run_id"] + 1):
        raise ValueError("preempt capture contains another motion or release run")
    release_requests = [item for item in records if item["kind"] == 4 and
                        item["fields"]["operation"] == 10 and
                        item["fields"]["request_id"] == release["request_id"]]
    if (len(release_requests) != 1 or
            any(item["fields"]["run_id"] not in (0, goal["run_id"])
                for item in records if item["kind"] == 3 and
                request["when"] <= item["when"] < release_requests[0]["when"])):
        raise ValueError("second motion status before A release")
    return {"request_id": fields["request_id"],
            "operation_id": receipts[0]["fields"]["operation_id"],
            "run_id": goal["run_id"], "goal_id": goal["goal_id"]}


def _preempt_recovery(action, timelines, records, goals, release, client):
    reported = action.get("preempt_goals")
    if (not isinstance(reported, dict) or set(reported) != {"A", "B", "C"} or
            action.get("second_begin_raw_join") != "REQUIRED_SEPARATELY" or
            action.get("qualification") != "NOT_QUALIFIED"):
        raise ValueError("incomplete ROS preempt recovery report")
    a, b, c = (reported[key] for key in ("A", "B", "C"))
    if (any(not isinstance(item, dict) for item in (a, b, c)) or
            [item.get("goal_id") for item in (a, b, c)] !=
            [goals[0]["goal_id"], b.get("goal_id"), goals[1]["goal_id"]] or
            not isinstance(b.get("goal_id"), str) or len(b["goal_id"]) != 32 or
            any(char not in "0123456789abcdef" for char in b["goal_id"]) or
            len({a["goal_id"], b["goal_id"], c["goal_id"]}) != 3 or
            a.get("status") != "CANCELED" or b.get("status") != "REJECTED" or
            b.get("phase") != "A_MOTION_FEEDBACK_OVERLAP" or
            c.get("status") != "SUCCEEDED" or
            b.get("second_begin_raw_join") != "REQUIRED_SEPARATELY" or
            goals[0]["action_status"] != "CANCELED" or
            goals[1]["action_status"] != "SUCCEEDED" or
            action["goals"][0].get("cancel", {}).get("overlap_probe") != b):
        raise ValueError("unlinked ROS preempt recovery goals")
    if (goals[0]["boot_epoch"], goals[0]["session_id"]) != \
            (goals[1]["boot_epoch"], goals[1]["session_id"]) or \
            timelines[0][2]["reference_epoch"] != timelines[1][2]["reference_epoch"] or \
            goals[1]["motion_epoch"] != goals[0]["stop_epoch"] + 1 or \
            goals[1]["run_id"] != goals[0]["run_id"] + 1 or \
            release["run_id"] != goals[1]["run_id"] + 1:
        raise ValueError("stale recovery session, epoch, or run")
    if {record["run"] for record in records if record["kind"] == 1} != \
            {goals[0]["run_id"], goals[1]["run_id"], release["run_id"]}:
        raise ValueError("unexpected recovery raw motion run")

    def operation_records(operation, kind):
        return [record for record in records if record["kind"] == kind and
                record["fields"].get("operation") == operation]

    begins = operation_records(7, 4)
    hold_time = timelines[0][-1]["steady_us"]
    permits = [record for record in operation_records(5, 4) if
               hold_time < record["when"] < begins[1]["when"]] if len(begins) == 2 else []
    if (len(begins) != 2 or len(permits) != 1 or
            len(operation_records(8, 4)) != 1 or
            len(operation_records(9, 4)) != 1 or
            len(operation_records(10, 4)) != 1):
        raise ValueError("missing or extra recovery management operation")
    requests_by_id = {}
    for request in (record for record in records if record["kind"] == 4):
        request_id = request["fields"]["request_id"]
        previous = requests_by_id.get(request_id)
        if request_id <= 0 or (previous is not None and
                (request["fields"]["operation"] != 12 or
                 previous[0]["raw"] != request["raw"] or previous[1] >= 4 or
                 any(response["kind"] == 5 and
                     response["fields"]["request_id"] == request_id and
                     response["when"] < request["when"] for response in records))):
            raise ValueError("duplicate recovery management request")
        requests_by_id[request_id] = (request, 1 if previous is None else previous[1] + 1)

    binding_fields = ("device_id", "client_instance_id", "expected_boot_epoch",
                      "expected_session_id", "expected_reference_epoch",
                      "expected_motion_epoch", "expected_management_epoch",
                      "expected_profile_id", "configured_mask")

    def committed_receipts(request, latest):
        fields = request["fields"]
        queries = [item for item in operation_records(12, 4) if
                   item["fields"]["queried_operation_id"] == fields["request_id"] and
                   request["when"] <= item["when"] <= latest and
                   all(item["fields"][key] == fields[key] for key in binding_fields)]
        related = [receipt for receipt in records if receipt["kind"] == 5 and
                   request["when"] <= receipt["when"] <= latest and
                   (receipt["fields"]["request_id"] == fields["request_id"] or
                    any(query["when"] <= receipt["when"] and
                        receipt["fields"]["request_id"] == query["fields"]["request_id"]
                        for query in queries))]
        if any(receipt["fields"]["transaction_phase"] != 3 and
               (receipt["fields"]["transaction_phase"] not in (1, 2) or
                receipt["fields"]["receipt"] != 1 or
                receipt["fields"]["result_code"] != 23 or
                receipt["fields"]["operation_id"] != 0 or
                receipt["fields"]["client_instance_id"] != fields["client_instance_id"] or
                receipt["fields"]["device_id"] != fields["device_id"] or
                receipt["boot"] != request["boot"] or
                receipt["session"] != request["session"] or
                receipt["fields"]["profile_id"] != fields["expected_profile_id"] or
                receipt["fields"]["run_id"] != 0 or
                receipt["fields"]["motion_epoch"] not in (
                    fields["expected_motion_epoch"],
                    fields["expected_motion_epoch"] +
                    int(fields["operation"] == 7 and
                        receipt["fields"]["transaction_phase"] == 2)))
               for receipt in related):
            raise ValueError("invalid recovery interim receipt")
        return [receipt for receipt in related if
                receipt["fields"]["transaction_phase"] == 3]
    bindings = []
    for rows, goal, request in zip(timelines, goals, begins):
        fields = request["fields"]
        before, after = rows[1:3]
        if (not before["steady_us"] <= request["when"] <= after["steady_us"] or
                (request["boot"], request["session"]) !=
                (goal["boot_epoch"], goal["session_id"]) or
                fields["device_id"] != goal["device_id"] or
                fields["client_instance_id"] != client or
                fields["request_id"] <= 0 or
                (fields["expected_reference_epoch"], fields["expected_motion_epoch"],
                 fields["expected_profile_id"], fields["configured_mask"]) !=
                (before["reference_epoch"], before["motion_epoch"], 201, 63)):
            raise ValueError("recovery BeginMotion request does not bind A/C")
        matches = committed_receipts(request, after["steady_us"])
        if (len(matches) != 1 or matches[0]["fields"]["receipt"] != 1 or
                matches[0]["fields"]["operation_id"] <= 0 or
                (matches[0]["fields"]["transaction_phase"],
                 matches[0]["fields"]["result_code"]) != (3, 0) or
                (matches[0]["boot"], matches[0]["session"],
                 matches[0]["fields"]["motion_epoch"]) !=
                (goal["boot_epoch"], goal["session_id"], goal["motion_epoch"]) or
                matches[0]["fields"]["run_id"] not in (0, goal["run_id"]) or
                (matches[0]["fields"]["motion_complete"],
                 matches[0]["fields"]["release_committed"]) != (0, 0) or
                matches[0]["fields"]["client_instance_id"] != client or
                matches[0]["fields"]["device_id"] != fields["device_id"]):
            raise ValueError("recovery BeginMotion lacks committed A/C receipt")
        bindings.append({"goal_id": goal["goal_id"], "run_id": goal["run_id"],
                         "request_id": fields["request_id"],
                         "operation_id": matches[0]["fields"]["operation_id"]})
    if bindings[0]["operation_id"] == bindings[1]["operation_id"]:
        raise ValueError("duplicate recovery BeginMotion operation")

    permit = permits[0]
    fields = permit["fields"]
    receipts = committed_receipts(permit, begins[1]["when"])
    if (len(receipts) != 1 or
            not hold_time < permit["when"] < receipts[0]["when"] < begins[1]["when"] or
            (permit["boot"], permit["session"]) !=
            (goals[0]["boot_epoch"], goals[0]["session_id"]) or
            fields["client_instance_id"] != client or
            (fields["expected_reference_epoch"], fields["expected_motion_epoch"],
             fields["expected_profile_id"], fields["configured_mask"]) !=
            (timelines[0][2]["reference_epoch"], goals[0]["stop_epoch"], 201, 63) or
            (receipts[0]["boot"], receipts[0]["session"],
             receipts[0]["fields"]["motion_epoch"]) !=
            (goals[0]["boot_epoch"], goals[0]["session_id"], goals[0]["stop_epoch"]) or
            (receipts[0]["fields"]["transaction_phase"],
             receipts[0]["fields"]["result_code"], receipts[0]["fields"]["receipt"]) !=
            (3, 0, 1) or receipts[0]["fields"]["operation_id"] <= 0 or
            receipts[0]["fields"]["run_id"] not in (0, goals[0]["run_id"]) or
            (receipts[0]["fields"]["motion_complete"],
             receipts[0]["fields"]["release_committed"]) != (0, 0) or
            receipts[0]["fields"]["client_instance_id"] != client or
            receipts[0]["fields"]["device_id"] != fields["device_id"]):
        raise ValueError("missing fresh committed recovery bench permit")
    return bindings


def _joined_goal(rows, records, action, context):
    state_max_age = context["gate_state_max_age_us"]
    goal_id = rows[0]["goal_uuid"]
    canceled = rows[-3 if rows[-1]["event"] == "post_release_status" else -2]["event"] == \
        "terminal_canceled"
    expected_status = ("CANCELED", 5) if canceled else ("SUCCEEDED", 4)
    observed_status = (action.get("action_observation") if
                       action.get("status") == "UNCONFIRMED_STOP" else action.get("status"))
    if ((observed_status, action.get("action_status")) != expected_status or
            action.get("goal_id") != goal_id or
            (not canceled and action.get("result_error_code") != 0)):
        raise ValueError("unlinked or unsuccessful ROS action report")
    cancel = action.get("cancel")
    if canceled and (not isinstance(cancel, dict) or cancel.get("goal_id") != goal_id or
                     cancel.get("requested") is not True or
                     cancel.get("acknowledged") is not True):
        raise ValueError("unconfirmed ROS cancel request")
    if action.get("trajectory_sha256") is None:
        raise ValueError("MISSING_INPUT: trajectory_sha256")
    if (not isinstance(action["trajectory_sha256"], str) or
            len(action["trajectory_sha256"]) != 64 or
            action["trajectory_sha256"] != rows[0]["trajectory_sha256"]):
        raise ValueError("unlinked trajectory SHA-256")
    started = action.get("action_started_steady_us")
    if type(started) is not int or started <= 0:
        raise ValueError("MISSING_INPUT: action_started_steady_us")
    final = rows[-1]["event"] == "post_release_status"
    begin = rows[2]
    feedback_rows = rows[3:-3] if final else rows[3:-2]
    hold_row = rows[-2] if final else rows[-1]
    feedback_row = feedback_rows[-1]
    boot, session, reference, motion, run = (begin[key] for key in
        ("boot_epoch", "session_id", "reference_epoch", "motion_epoch", "run_id"))
    if (reference == 0 or any(row["reference_epoch"] != reference for row in
                              (rows[:-1] if final else rows)) or
            (final and rows[-1]["reference_epoch"] != 0)):
        raise ValueError("Gate reference epoch changed within goal")
    identity_key = (boot, session, reference, motion)
    before = rows[1]
    before_identity = (boot, session, reference, before["motion_epoch"])
    before_status = None
    before_state = None
    if before_identity != identity_key:
        before_status = _prior(records, 3, before["steady_us"], before_identity,
                               STATUS_MAX_AGE_US, run=before["run_id"])
        before_state = _prior(records, 2, before["steady_us"],
                              before_identity, state_max_age)
    status = _prior(records, 3, begin["steady_us"], identity_key,
                    STATUS_MAX_AGE_US, run=run)
    state = _prior(records, 2, begin["steady_us"], identity_key, state_max_age)
    device_id = status["fields"]["device_id"]
    if (device_id == "00" * 16 or any(candidate is not None and
            candidate["fields"]["device_id"] != device_id for candidate in
            (state, before_status, before_state))):
        raise ValueError("goal observations changed device identity")
    if (status["fields"]["profile_id"] != 201 or status["fields"]["configured_mask"] != 63 or
            state["fields"]["profile_id"] != 201 or state["fields"]["configured_mask"] != 63):
        raise ValueError("non-six-axis device binding")
    pages = _pages(records, run, boot, session, 1, 0)
    goal_pages = [page for page in pages[:-1] if page["fields"]["motion_epoch"] == motion]
    if not goal_pages or goal_pages[0]["when"] < before["steady_us"]:
        raise ValueError("goal-epoch raw page predates Begin request binding")
    feedback = None
    gate_feedback = []
    for row in feedback_rows:
        feedback_identity = (boot, session, reference, row["motion_epoch"])
        feedback = _prior(records, 2, row["steady_us"], feedback_identity,
                          state_max_age)
        if (feedback["fields"]["device_id"] != device_id or
                feedback["fields"]["position_valid_mask"] != 63 or
                feedback["fields"]["feedback_seq"] != row["feedback_seq"] or
                feedback["fields"]["can_rx_age_us"] != row["feedback_age_us"]):
            raise ValueError("unlinked six-axis feedback")
        gate_feedback.append((row["steady_us"], feedback))
    hold_identity = (boot, session, reference, hold_row["motion_epoch"])
    hold_status, hold_state = _hold_observations(
        records, hold_row["steady_us"], hold_identity, run, state_max_age,
        pages[-1]["when"], context["client_id"], 7 if canceled else 6)
    if (hold_status["fields"]["device_id"] != device_id or
            hold_state["fields"]["device_id"] != device_id):
        raise ValueError("HOLD observations changed device identity")
    if (hold_status["fields"]["core_state"], hold_state["fields"]["core_state"],
            hold_status["fields"]["motion_complete"], hold_state["fields"]["hold_ready"]) != \
            (4, 4, 0 if canceled else 1, 1):
        raise ValueError("unconfirmed pre-release HOLD")
    if (hold_status["when"] < pages[-1]["when"] or
            hold_status["fields"]["attempt_kind"] != 1 or
            hold_status["fields"]["final_state"] != (7 if canceled else 6) or
            hold_status["fields"]["evidence_lost"] or
            hold_status["fields"]["evidence_chain_sha256"] !=
            pages[-1]["fields"]["chain_sha256"]):
        raise ValueError("unlinked HOLD terminal chain")
    hashes = {key: status["fields"][name] for key, name in
              (("contract_hash", "contract_sha256"), ("model_hash", "model_sha256"),
               ("profile_hash", "profile_sha256"), ("firmware_hash", "firmware_sha256"))}
    if any(candidate["fields"][name] != hashes[key]
           for candidate in (hold_status,) + ((before_status,) if before_status else ())
           for key, name in (("contract_hash", "contract_sha256"),
                             ("model_hash", "model_sha256"),
                             ("profile_hash", "profile_sha256"),
                             ("firmware_hash", "firmware_sha256"))):
        raise ValueError("device identity hash changed during goal")
    client = context["client_id"]
    if not isinstance(client, str) or len(client) != 32:
        raise ValueError("missing externally captured client identity")
    try:
        bytes.fromhex(client)
    except ValueError as error:
        raise ValueError("invalid client identity") from error
    samples = action.get("feedback")
    if not isinstance(samples, list) or not samples:
        raise ValueError("missing ROS action feedback")
    if canceled:
        start = cancel.get("motion_feedback_start_index")
        trigger = cancel.get("motion_feedback_trigger_index")
        delta = cancel.get("motion_delta_rad")
        if (type(start) is not int or type(trigger) is not int or
                not 0 <= start < trigger < len(samples) or
                type(delta) not in (int, float) or not math.isfinite(delta) or delta <= 0 or
                any(sample.get("goal_id") != goal_id for sample in
                    samples[start:trigger + 1]) or
                max(abs(a - b) for a, b in zip(
                    _six(samples[start].get("actual_rad"), "cancel origin"),
                    _six(samples[trigger].get("actual_rad"), "cancel trigger"))) < delta):
            raise ValueError("unlinked or unobserved mid-motion ROS cancel")
    ros_budget = context["ros_feedback_lag_budget_us"]
    if type(ros_budget) is not int or ros_budget <= 0 or started > rows[0]["steady_us"]:
        raise ValueError("invalid ROS feedback clock or budget")
    path_tolerance = _six(context["path_tolerance_rad"], "path tolerance")
    if any(limit <= 0 for limit in path_tolerance):
        raise ValueError("invalid path tolerance")
    path_error = [0.0] * 6
    desired_samples = []
    last_feedback_time = -1
    last_raw_feedback_epoch = motion
    last_ros_feedback_seq = None
    for sample in samples:
        elapsed = sample.get("elapsed_us")
        if type(elapsed) is not int or elapsed < 0:
            raise ValueError("invalid ROS feedback time")
        sample_time = started + elapsed
        if (sample_time <= last_feedback_time or sample_time < before["steady_us"] or
                sample_time > rows[-3 if final else -2]["steady_us"] + ros_budget):
            raise ValueError("ROS feedback time outside Gate action window")
        last_feedback_time = sample_time
        actual = _six(sample.get("actual_rad"), "ROS feedback")
        desired = _six(sample.get("desired_rad"), "ROS desired")
        if sample_time < begin["steady_us"]:
            continue
        desired_samples.append((sample_time, desired))
        latest_state = _latest(records, 2, sample_time)
        if latest_state is None or sample_time - latest_state["when"] > state_max_age:
            raise ValueError("missing prior raw feedback state")
        raw_epoch = latest_state["fields"]["motion_epoch"]
        if raw_epoch not in (motion, hold_row["motion_epoch"]) or \
                raw_epoch < last_raw_feedback_epoch:
            raise ValueError("raw feedback outside goal/Complete epoch phase")
        last_raw_feedback_epoch = raw_epoch
        raw_state = _prior(records, 2, sample_time,
                           (boot, session, reference, raw_epoch), state_max_age)
        if raw_state["fields"]["position_valid_mask"] != 63:
            raise ValueError("invalid raw feedback position")
        last_ros_feedback_seq = raw_state["fields"]["feedback_seq"]
        raw_position = _six(raw_state["fields"]["position_rad"], "raw feedback")
        if any(abs(a - position) > limit for a, position, limit in
               zip(actual, raw_position, path_tolerance)):
            raise ValueError("ROS feedback outside bounded raw path position")
        path_error = [max(previous, abs(position - d)) for previous, position, d in
                      zip(path_error, raw_position, desired)]
    if not desired_samples:
        raise ValueError("missing post-Begin ROS feedback")
    for gate_time, gate_state in gate_feedback:
        desired = _desired_at(desired_samples, gate_time)
        raw_position = _six(gate_state["fields"]["position_rad"], "Gate raw feedback")
        path_error = [max(previous, abs(position - d)) for previous, position, d in
                      zip(path_error, raw_position, desired)]
    raw_states, raw_path_error = _goal_raw_states(
        records, before["steady_us"], begin["steady_us"], hold_row["steady_us"],
        boot, session, reference,
        {before["motion_epoch"], motion, hold_row["motion_epoch"]},
        desired_samples, path_tolerance)
    if any(record["fields"]["device_id"] != device_id for record in raw_states):
        raise ValueError("raw path changed device identity")
    path_error = [max(a, b) for a, b in zip(path_error, raw_path_error)]
    if last_ros_feedback_seq is None:
        raise ValueError("missing post-Begin ROS feedback")
    actual = _six(samples[-1]["actual_rad"], "final ROS feedback")
    tail_gap = feedback_row["steady_us"] - last_feedback_time
    if any(next_time - previous_time > state_max_age for
           (previous_time, _), (next_time, _) in zip(gate_feedback, gate_feedback[1:])):
        raise ValueError("Gate feedback has unobserved raw path gap")
    terminal_time = rows[-3 if final else -2]["steady_us"]
    tail_observations = [feedback_row["steady_us"]] + [record["when"] for record in
        raw_states if feedback_row["steady_us"] < record["when"] <= hold_row["steady_us"]]
    tail_observations += [terminal_time, hold_row["steady_us"]]
    tail_observations.sort()
    if any(right - left > state_max_age for left, right in
           zip(tail_observations, tail_observations[1:])):
        raise ValueError("terminal/HOLD has unobserved raw State gap")
    nearby_gate = any(abs(gate_time - last_feedback_time) <= ros_budget for
                      gate_time, _ in gate_feedback)
    if not nearby_gate and not _raw_ros_handoff(
            records, last_feedback_time, raw_state, actual,
            (boot, session, reference, raw_epoch), ros_budget,
            path_tolerance, raw_states, gate_feedback,
            context["feedback_age_budget_us"]):
        raise ValueError("ROS feedback time has no bounded fresh Gate feedback")
    if (tail_gap < -ros_budget or
            (tail_gap > ros_budget and
             (any(new < old for new, old in zip(feedback["fields"]["feedback_seq"],
                                                last_ros_feedback_seq)) or
              not any(new > old for new, old in zip(feedback["fields"]["feedback_seq"],
                                                  last_ros_feedback_seq))))):
        raise ValueError("ROS feedback time has no bounded fresh Gate feedback")
    goal_tolerance = _six(context["goal_tolerance_rad"], "goal tolerance")
    near_state = _near_ros_state(records, last_feedback_time,
                                 (boot, session, reference, raw_epoch),
                                 ros_budget, context["feedback_age_budget_us"])
    if any(limit <= 0 or abs(a - b) > limit for a, b, limit in
           zip(actual, _six(near_state["fields"]["position_rad"],
                            "near ROS raw feedback"), goal_tolerance)):
        raise ValueError("ROS feedback outside bounded raw device position envelope")
    target = _six(action.get("target_rad"), "target")
    if not canceled and any(abs(position - planned) > limit for fields in
           (feedback["fields"], hold_state["fields"]) for position, planned, limit in
           zip(_six(fields["position_rad"], "terminal raw position"),
               target, goal_tolerance)):
        raise ValueError("raw device position outside goal tolerance")
    groups = [{"group_id": page["fields"]["group_id"], "run_id": run,
               "session_id": session, "motion_epoch": page["fields"]["motion_epoch"],
               "axis_mask": page["fields"]["expected_mask"],
               "sent_mask": page["fields"]["sent_mask"],
               "committed": page["fields"]["final"] == 1} for page in pages[:-1]]
    page_json = lambda items: [{"raw_hex": item["raw"].hex(), "boot_epoch": item["boot"],
                                "session_id": item["session"]} for item in items]
    goal = {"goal_id": goal_id, "action_status": "CANCELED" if canceled else "SUCCEEDED",
            "result_error_code": action.get("result_error_code"),
            "device_id": device_id,
            "run_id": run, "session_id": session,
            "boot_epoch": boot, "motion_epoch": motion,
            "target_rad": target,
            "feedback_rad": actual, "path_error_max_rad": path_error,
            "path_tolerance_rad": path_tolerance,
            "goal_tolerance_rad": goal_tolerance,
            "feedback_age_us": feedback_row["feedback_age_us"],
            "feedback_age_budget_us": context["feedback_age_budget_us"],
            "timeline": [{key: value for key, value in row.items() if
                          key != "trajectory_sha256"} for row in rows],
            "evidence_pages": page_json(pages), "groups": groups,
            "terminal": "CANCELED_HOLDING" if canceled else "MOTION_COMPLETED_HOLDING",
            "hold_confirmed": True,
            "hold_proof_source": hold_status.get("proof_source", "completed_run_status")}
    if canceled:
        stop_epoch = hold_row["motion_epoch"]
        terminal_row = rows[-3 if final else -2]
        terminal_time = terminal_row["steady_us"]
        requests = [record for record in records if record["kind"] == 4 and
                    record["fields"]["operation"] == 9 and
                    record["fields"]["request_id"] ==
                    terminal_row["cancel_request_id"] and
                    record["fields"]["client_instance_id"] == client and
                    (record["boot"], record["session"]) == (boot, session) and
                    begin["steady_us"] <= record["when"] <= terminal_time]
        if len(requests) != 1:
            raise ValueError("missing or ambiguous current cancel request")
        request = requests[0]
        stop_pages = [page for page in pages[:-1] if
                      page["fields"]["motion_epoch"] == stop_epoch and
                      page["fields"]["category"] == 2]
        if not stop_pages or request["when"] > stop_pages[0]["when"]:
            raise ValueError("cancel request does not precede new-epoch Hold SENT")
        if stop_pages[0]["when"] > terminal_time:
            raise ValueError("Safety Stop CAN SENT follows Gate terminal")
        binding_fields = ("device_id", "client_instance_id", "expected_boot_epoch",
                          "expected_session_id", "expected_reference_epoch",
                          "expected_motion_epoch", "expected_management_epoch",
                          "expected_profile_id", "configured_mask")
        queries = [record for record in records if record["kind"] == 4 and
                   record["fields"]["operation"] == 12 and
                   record["fields"]["request_id"] != request["fields"]["request_id"] and
                   record["fields"]["queried_operation_id"] ==
                   request["fields"]["request_id"] and
                   request["when"] <= record["when"] <= hold_row["steady_us"] and
                   all(record["fields"][name] == request["fields"][name]
                       for name in binding_fields)]
        responses = [record for record in records if record["kind"] == 5 and
                     request["when"] <= record["when"] <= hold_row["steady_us"] and
                     (record["fields"]["request_id"] == request["fields"]["request_id"] or
                      any(query["when"] <= record["when"] and
                          query["fields"]["request_id"] == record["fields"]["request_id"]
                          for query in queries)) and
                     record["fields"]["client_instance_id"] == client and
                     record["fields"]["device_id"] == request["fields"]["device_id"] and
                     (record["boot"], record["session"], record["run"]) ==
                     (boot, session, run) and
                     (record["fields"]["transaction_phase"],
                      record["fields"]["result_code"]) == (3, 0) and
                     record["fields"]["motion_complete"] == 0 and
                     record["fields"]["release_committed"] == 0 and
                     record["fields"]["operation_id"] > 0 and
                     record["fields"]["operation_id"] ==
                     terminal_row["cancel_operation_id"] and
                     record["fields"]["motion_epoch"] == stop_epoch]
        if len(responses) != 1 or stop_epoch <= motion:
            raise ValueError("cancel lacks matching committed current-run receipt")
        if responses[0]["when"] > terminal_time:
            raise ValueError("cancel committed receipt follows Gate terminal")
        goal.update(stop_confirmed=True, stop_epoch=stop_epoch,
                    cancel_request_id=terminal_row["cancel_request_id"],
                    cancel_operation_id=terminal_row["cancel_operation_id"])
    return goal, hashes, hold_row, pages[-1], pages[0]


def _release_dispatch_within_budget(pages, request_when, budget_us):
    if request_when <= 0 or budget_us <= 0:
        return False
    disable = next((page for page in pages if page["fields"]["category"] == 3), None)
    if (disable is None or disable["when"] < request_when or
            disable["when"] - request_when > budget_us):
        return False
    return all(page["when"] <= disable["when"] and
               page["when"] - request_when <= budget_us
               for page in pages if page["when"] >= request_when and
               page["fields"]["category"] in (1, 2))


def _release(records, goal, hold_row, terminal, state_max_age, hashes):
    boot, session, run, motion = (goal[key] for key in
                                  ("boot_epoch", "session_id", "run_id", "motion_epoch"))
    idle_row = goal["timeline"][-1]
    release_runs = {record["run"] for record in records if record["kind"] == 1 and
                    record["run"] > run}
    if len(release_runs) != 1:
        raise ValueError("missing or ambiguous RELEASE attempt")
    release_run = release_runs.pop()
    if goal["action_status"] == "CANCELED" and release_run != run + 1:
        raise ValueError("cancel release must use next attempt run")
    first = next(record for record in records if record["kind"] == 1 and
                 record["run"] == release_run)
    active = first["fields"]["attempt_kind"] == 1
    release_pages = _pages(records, release_run, boot, session,
                           1 if active else 3, 0 if active else run)
    if active and release_run <= run:
        raise ValueError("active release run did not advance")
    if release_pages[0]["when"] <= terminal["when"]:
        raise ValueError("RELEASE page precedes motion terminal page")
    if ((not active and release_pages[0]["when"] <= hold_row["steady_us"]) or
            release_pages[-1]["when"] >= idle_row["steady_us"]):
        raise ValueError("RELEASE attempt out of order")
    request_id = idle_row["release_request_id"]
    operation_id = idle_row["release_operation_id"]
    requests = [record for record in records if record["kind"] == 4 and
                record["fields"]["operation"] == 10 and
                record["fields"]["request_id"] == request_id]
    if len(requests) != 1:
        raise ValueError("missing or ambiguous release management request")
    request = requests[0]
    if request["fields"]["device_id"] != goal["device_id"]:
        raise ValueError("release request changed device identity")
    responses = []
    for record in records:
        if record["kind"] != 5 or record["when"] < request["when"]:
            continue
        fields = record["fields"]
        direct = fields["request_id"] == request_id
        queried = any(query["kind"] == 4 and query["fields"]["operation"] == 12 and
                      query["fields"]["request_id"] == fields["request_id"] and
                      query["fields"]["queried_operation_id"] == request_id and
                      query["fields"]["client_instance_id"] ==
                      request["fields"]["client_instance_id"] and
                      request["when"] <= query["when"] <= record["when"]
                      for query in records)
        if ((direct or queried) and fields["operation_id"] == operation_id and
                fields["run_id"] == release_run and fields["boot_epoch"] == boot and
                fields["session_id"] == 0 and fields["transaction_phase"] == 3 and
                fields["result_code"] == 0 and
                (fields["motion_complete"], fields["release_committed"]) == (0, 1) and
                fields["device_id"] == request["fields"]["device_id"] and
                fields["client_instance_id"] == request["fields"]["client_instance_id"]):
            responses.append(record)
    if len(responses) != 1 or responses[0]["fields"]["motion_epoch"] != \
            idle_row["motion_epoch"]:
        raise ValueError("Idle motion epoch lacks matching committed release receipt")
    idle = (boot, 0, 0, idle_row["motion_epoch"])
    idle_status = _prior(records, 3, idle_row["steady_us"], idle,
                         state_max_age, run=release_run)
    idle_state = _prior(records, 2, idle_row["steady_us"], idle, state_max_age)
    if (idle_status["fields"]["device_id"] != goal["device_id"] or
            idle_state["fields"]["device_id"] != goal["device_id"]):
        raise ValueError("release observation changed device identity")
    if (idle_status["fields"]["core_state"], idle_state["fields"]["core_state"],
            idle_status["fields"]["release_committed"],
            idle_status["fields"]["real_motion_permitted"],
            idle_status["fields"]["permit_remaining_us"]) != (1, 1, 1, 0, 0):
        raise ValueError("unconfirmed RELEASE Idle")
    status_chain = idle_status["fields"]["evidence_chain_sha256"]
    terminal_chain = release_pages[-1]["fields"]["chain_sha256"]
    if (idle_status["when"] < release_pages[-1]["when"] or
            idle_status["fields"]["attempt_kind"] != (1 if active else 3) or
            idle_status["fields"]["final_state"] != (2 if active else 8) or
            idle_status["fields"]["evidence_lost"] or
            status_chain not in ((terminal_chain, "00" * 32) if active else
                                 (terminal_chain,))):
        raise ValueError("unlinked RELEASE terminal chain")
    if any(idle_status["fields"][name] != hashes[key] for key, name in
           (("contract_hash", "contract_sha256"), ("model_hash", "model_sha256"),
            ("profile_hash", "profile_sha256"), ("firmware_hash", "firmware_sha256"))):
        raise ValueError("device identity hash changed during goal")
    page_json = lambda items: [{"raw_hex": item["raw"].hex(), "boot_epoch": item["boot"],
                                "session_id": item["session"]} for item in items]
    return {"run_id": release_run, "parent_run_id": run,
            "session_id": session, "boot_epoch": boot,
            "terminal": "CANCELED_AND_RELEASED" if active else "RELEASED",
            "confirmed": True,
            "evidence_pages": page_json(release_pages)}


def join_evidence(timeline_path, raw_path, action_path, context_path):
    context, context_hash = _context(context_path)
    state_max_age = context["gate_state_max_age_us"]
    if type(state_max_age) is not int or not 0 < state_max_age <= STATUS_MAX_AGE_US:
        raise ValueError("invalid Gate state maximum age")
    csv_bytes, csv_hash = _snapshot(Path(timeline_path), context["timeline_sha256"])
    raw_bytes, raw_hash = _snapshot(Path(raw_path), context["raw_sha256"])
    action_bytes, action_hash = _snapshot(Path(action_path), context["action_sha256"])
    action = json.loads(action_bytes)
    if not isinstance(action, dict):
        raise ValueError("invalid ROS action report")
    timelines = _timeline(csv_bytes)
    records = _raw_records(raw_bytes)
    _verify_window_context(context, csv_bytes, records)
    if len(timelines) == 1 and action.get("scope") in (
            "ROS_ACTION_ONLY", "ROS_PREEMPT_PROBE"):
        actions = [action]
    elif action.get("scope") in ("ROS_ACTION_SESSION_ONLY", "ROS_PREEMPT_RECOVERY"):
        actions = action.get("goals")
        if (not isinstance(actions, list) or len(actions) != len(timelines) or
                (action.get("scope") == "ROS_PREEMPT_RECOVERY" and len(actions) != 2) or
                type(action.get("requested_goals")) is not int or
                type(action.get("completed_goals")) is not int or
                action["requested_goals"] != len(timelines) or
                action["completed_goals"] != len(timelines) or
                any(not isinstance(item, dict) or item.get("index") != index
                    for index, item in enumerate(actions))):
            raise ValueError("unlinked multi-goal ROS action report")
    else:
        raise ValueError("unlinked ROS action scope")
    previously_confirmed = (action.get("status") in ("SUCCEEDED", "CANCELED") and
                            action.get("stop_confirmation") == "CONFIRMED_RELEASED_IDLE")
    unconfirmed_single_cancel = (
        action.get("scope") in ("ROS_ACTION_ONLY", "ROS_PREEMPT_PROBE") and
        len(timelines) == 1 and
        timelines[0][-3]["event"] == "terminal_canceled" and
        _unconfirmed_cancel_observation(action))
    if not previously_confirmed and not unconfirmed_single_cancel:
        raise ValueError("unconfirmed final ROS release")
    goals = []
    hashes = None
    holds = []
    for rows, item in zip(timelines, actions):
        goal, goal_hashes, hold_row, terminal, first_page = _joined_goal(rows, records, item, context)
        if hashes is not None and hashes != goal_hashes:
            raise ValueError("device identity hash changed across goals")
        hashes = goal_hashes
        if goals and (goal["boot_epoch"], goal["session_id"]) != \
                (goals[0]["boot_epoch"], goals[0]["session_id"]):
            raise ValueError("cross-session ROS goals")
        if goals and goal["device_id"] != goals[0]["device_id"]:
            raise ValueError("cross-device ROS goals")
        if goals and goal["run_id"] <= goals[-1]["run_id"]:
            raise ValueError("duplicate or regressing device run across goals")
        if holds and first_page["when"] <= holds[-1][1]["when"]:
            raise ValueError("next motion run predates prior terminal evidence")
        goals.append(goal)
        holds.append((hold_row, terminal))
    release = _release(records, goals[-1], *holds[-1], state_max_age, hashes)
    release = {**release,
               "request_id": goals[-1]["timeline"][-1]["release_request_id"],
               "operation_id": goals[-1]["timeline"][-1]["release_operation_id"]}
    preempt = None
    if action.get("scope") == "ROS_PREEMPT_PROBE":
        preempt = _preempt_probe(action, timelines[0], records, goals[0], release,
                                 context["client_id"])
    elif action.get("scope") == "ROS_PREEMPT_RECOVERY":
        preempt = _preempt_recovery(action, timelines, records, goals, release,
                                    context["client_id"])
    release_requests = [record for record in records if record["kind"] == 4 and
                        record["fields"]["operation"] == 10 and
                        record["fields"]["request_id"] == release["request_id"]]
    release_groups = [record for record in records if record["kind"] == 1 and
                      record["run"] == release["run_id"] and
                      record["fields"]["category"] != 4]
    required_groups = [record for record in release_groups if
                       not (release["terminal"] == "CANCELED_AND_RELEASED" and
                            record["fields"]["category"] == 1)]
    if not required_groups:
        raise ValueError("release has no Disable CAN group")
    first_required = min(record["when"] for record in required_groups)
    if len(release_requests) != 1 or release_requests[0]["when"] > first_required:
        raise ValueError("release request does not precede release CAN chain")
    if (release["terminal"] == "CANCELED_AND_RELEASED" and
            not _release_dispatch_within_budget(
                release_groups, release_requests[0]["when"], state_max_age)):
        raise ValueError("active release request-to-Disable observation budget exceeded")
    last = goals[-1]
    result = {"source": "ros_moveit_jtc_usb_ref_can", "profile_id": 201,
              "configured_mask": 63, "identity": {"client_id": context["client_id"],
                                                     "device_id": goals[0]["device_id"],
                                                     **hashes},
              "environment": {"producer_lag_us": context["producer_lag_us"],
                              "producer_lag_budget_us": context["producer_lag_budget_us"],
                              **({key: context[key] for key in
                                  ("producer_lag_gate_version", "producer_lag_method",
                                   "producer_lag_environment_us", "producer_lag_window")}
                                 if "producer_lag_gate_version" in context else {})},
              "provenance": {"timeline_sha256": csv_hash, "raw_sha256": raw_hash,
                             "action_sha256": action_hash, "context_sha256": context_hash,
                             "feedback_binding": "bounded_temporal_and_position_envelope"},
              "goals": goals,
              "management_records": [{"kind": record["kind"], "when": record["when"],
                                      "boot": record["boot"], "session": record["session"],
                                      "run": record["run"], "seq": record["seq"],
                                      "raw_hex": record["raw"].hex()} for record in records
                                     if record["kind"] in (4, 5)],
              "pre_release_hold": {"run_id": last["run_id"], "session_id": last["session_id"],
                                   "boot_epoch": last["boot_epoch"],
                                   "motion_epoch": holds[-1][0]["motion_epoch"],
                                   "confirmed": True},
              "release": release}
    if preempt is not None:
        result.update(preempt_goals=action["preempt_goals"], begin_motion=preempt)
    spec = importlib.util.spec_from_file_location("application_e2e", Path(__file__).with_name("application_e2e.py"))
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    validator.validate_evidence(result)
    return result


def main():
    parser = argparse.ArgumentParser(description="拼接真实六轴 ROS 与 DeviceTransport 原始证据")
    parser.add_argument("--timeline", type=Path, required=True)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--action", type=Path, required=True)
    parser.add_argument("--context", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = join_evidence(args.timeline, args.raw, args.action, args.context)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
