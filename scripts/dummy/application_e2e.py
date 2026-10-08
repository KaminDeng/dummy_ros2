#!/usr/bin/env python3
"""六轴 ROS/MoveIt 无负载 E2E 原始证据门禁。"""

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct


LAYOUT = json.loads((Path(__file__).resolve().parents[2] /
    "contracts/dummy_robot_v2/codec/wire_layout.json").read_text())
WIRE_TYPES = {"uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q",
              "float64": "d", "bool": "B"}


def _manage_fields(item):
    kind = item.get("kind") if isinstance(item, dict) else None
    if kind not in (4, 5):
        raise ValueError("invalid management record kind")
    name = "DummyManageV2Request" if kind == 4 else "DummyManageV2Response"
    layout = LAYOUT["payloads"][name]
    raw = _hex_bytes(item, "raw_hex", layout["encoded_size"])
    fields = {}
    for field in layout["fields"]:
        values = struct.unpack_from("<" + WIRE_TYPES[field["type"]] * field["count"],
                                    raw, field["offset"])
        fields[field["name"]] = (bytes(values).hex() if field["count"] > 1 and
            field["type"] == "uint8" else list(values) if field["count"] > 1 else values[0])
    if (fields["schema_major"], fields["schema_minor"]) != (2, 4):
        raise ValueError("invalid management wire schema")
    return fields


def _validate_management(evidence, goals, release):
    records = _required(evidence, "management_records")
    if not isinstance(records, list) or not records:
        raise ValueError("missing management receipts")
    decoded = []
    request_ids = {}
    answered_ids = set()
    previous_when = 0
    for item in records:
        fields = _manage_fields(item)
        if (type(item.get("when")) is not int or item["when"] <= 0 or
                item.get("seq") != (fields["request_seq"] if item["kind"] == 4 else 0) or
                item.get("boot") != (fields["expected_boot_epoch"] if item["kind"] == 4
                                     else fields["boot_epoch"]) or
                item.get("session") != (fields["expected_session_id"] if item["kind"] == 4
                                        else fields["session_id"]) or
                item.get("run") != (0 if item["kind"] == 4 else fields["run_id"]) or
                fields["client_instance_id"] != evidence["identity"]["client_id"] or
                fields["device_id"] != evidence["identity"]["device_id"]):
            raise ValueError("unlinked management record identity")
        if item["when"] < previous_when:
            raise ValueError("regressing management record time")
        previous_when = item["when"]
        if item["kind"] == 4:
            request_key = (fields["client_instance_id"], fields["request_id"])
            if fields["request_id"] == 0:
                raise ValueError("duplicate management request identity")
            previous = request_ids.get(request_key)
            if previous is not None:
                raw, attempts = previous
                if (fields["operation"] != 12 or bytes.fromhex(item["raw_hex"]) != raw or
                        request_key in answered_ids or attempts >= 4):
                    raise ValueError("duplicate management request identity")
                request_ids[request_key] = (raw, attempts + 1)
                continue
            request_ids[request_key] = (bytes.fromhex(item["raw_hex"]), 1)
        else:
            answered_ids.add((fields["client_instance_id"], fields["request_id"]))
        decoded.append((item, fields))

    query_binding_fields = ("device_id", "client_instance_id", "expected_boot_epoch",
                            "expected_session_id", "expected_reference_epoch",
                            "expected_motion_epoch", "expected_management_epoch",
                            "expected_profile_id", "configured_mask")
    terminal_flags = {8: (1, 0), 9: (0, 0), 10: (0, 1)}

    def committed(operation, run, session, before=None, after=0):
        requests = [(item, fields) for item, fields in decoded if item["kind"] == 4 and
                    fields["operation"] == operation and
                    item["when"] >= after and
                    fields["expected_boot_epoch"] == release["boot_epoch"] and
                    fields["expected_session_id"] == session and
                    fields["expected_profile_id"] == 201]
        matches = []
        for request_item, request in requests:
            if before is not None and request_item["when"] > before:
                continue
            operation_ids = set()
            for response_item, response in decoded:
                if response_item["kind"] != 5 or response_item["when"] < request_item["when"]:
                    continue
                direct = response["request_id"] == request["request_id"]
                query = any(query_item["when"] >= request_item["when"] and
                            query_item["when"] <= response_item["when"] and
                            query["operation"] == 12 and
                            query["request_id"] == response["request_id"] and
                            query["queried_operation_id"] == request["request_id"] and
                            all(query[name] == request[name] for name in query_binding_fields)
                            for query_item, query in decoded if query_item["kind"] == 4)
                if not (direct or query):
                    continue
                terminal = response["transaction_phase"] in (3, 4)
                if terminal and (response["receipt"] not in (1, 2) or
                                 response["operation_id"] == 0 or
                                 response["run_id"] != run or
                                 response["boot_epoch"] != release["boot_epoch"] or
                                 response["session_id"] != (0 if operation == 10 else session) or
                                 response["device_id"] != request["device_id"] or
                                 response["profile_id"] != 201 or
                                 response["transaction_phase"] != 3 or
                                 response["result_code"] != 0 or
                                 (response["motion_complete"],
                                  response["release_committed"]) != terminal_flags[operation]):
                    raise ValueError("conflicting terminal management receipt")
                if response["operation_id"]:
                    operation_ids.add(response["operation_id"])
                committed_flag = (response["motion_complete"],
                                  response["release_committed"]) == terminal_flags[operation]
                if ((response["transaction_phase"], response["result_code"]) == (3, 0) and
                        committed_flag):
                    matches.append((request_item, request, response_item, response))
            if len(operation_ids) > 1:
                raise ValueError("conflicting management operation identity")
        if len(matches) != 1:
            raise ValueError("missing or ambiguous committed management receipt")
        return matches[0]

    completes = []
    for goal in goals:
        terminal_time = goal["timeline"][-3 if goal is goals[-1] else -2]["steady_us"]
        operation = 9 if goal["action_status"] == "CANCELED" else 8
        receipt = committed(operation, goal["run_id"], goal["session_id"], terminal_time,
                            goal["timeline"][2]["steady_us"])
        if operation == 9 and receipt[2]["when"] > terminal_time:
            raise ValueError("cancel committed receipt follows Gate terminal")
        if operation == 9 and (
                goal.get("cancel_request_id") != receipt[1]["request_id"] or
                goal.get("cancel_operation_id") != receipt[3]["operation_id"] or
                receipt[3]["motion_epoch"] != goal["stop_epoch"]):
            raise ValueError("cancel request or operation key does not match committed receipt")
        if operation == 9 and receipt[1]["expected_motion_epoch"] != goal["motion_epoch"]:
            raise ValueError("cancel request motion epoch does not match goal")
        if operation == 9 and any(
                row["event"] == "feedback" and
                row["motion_epoch"] == goal["stop_epoch"] and
                row["steady_us"] < receipt[0]["when"] for row in goal["timeline"]):
            raise ValueError("stop feedback precedes cancel request")
        completes.append(receipt)
    stop = committed(10, release["run_id"], release["session_id"],
                     goals[-1]["timeline"][-1]["steady_us"])
    if (release.get("request_id") != stop[1]["request_id"] or
            release.get("operation_id") != stop[3]["operation_id"]):
        raise ValueError("release request or operation key does not match committed receipt")
    if goals[-1]["timeline"][-1]["motion_epoch"] != stop[3]["motion_epoch"]:
        raise ValueError("Idle motion epoch does not match committed release receipt")
    if completes[-1][2]["when"] > stop[0]["when"]:
        raise ValueError("CompleteMotion receipt follows release request")
    if stop[0]["when"] < goals[-1]["timeline"][-2]["steady_us"]:
        raise ValueError("release request precedes final HOLD observation")


def _required(record, name):
    if not isinstance(record, dict):
        raise ValueError(f"invalid record for {name}")
    value = record.get(name)
    if value is None or value == "" or value == []:
        raise ValueError(f"missing {name}")
    return value


def _positive_int(record, name):
    value = _required(record, name)
    if type(value) is not int or value <= 0:
        raise ValueError(f"invalid {name}")
    return value


def _six_finite(record, name, positive=False):
    values = _required(record, name)
    if (not isinstance(values, list) or len(values) != 6 or
            any(type(value) not in (int, float) or not math.isfinite(value) or
                (positive and value <= 0) for value in values)):
        raise ValueError(f"invalid {name}")
    return values


def _hex_bytes(record, name, size):
    value = _required(record, name)
    if not isinstance(value, str) or len(value) != size * 2:
        raise ValueError(f"invalid {name}")
    try:
        return bytes.fromhex(value)
    except ValueError as error:
        raise ValueError(f"invalid {name}") from error


def _validate_pages(attempt, identity, *, run, session, boot, epoch, kind,
                    parent, final_state, group_ids=None, group_epochs=None,
                    terminal_epoch=None):
    pages = _required(attempt, "evidence_pages")
    if not isinstance(pages, list) or len(pages) < 2:
        raise ValueError("missing CAN or terminal evidence page")
    seed = struct.pack("<QQ", run, boot) + _hex_bytes(identity, "client_id", 16)
    seed += b"".join(_hex_bytes(identity, name, 32) for name in
                     ("contract_hash", "model_hash", "profile_hash", "firmware_hash"))
    previous = hashlib.sha256(seed.ljust(192, b"\0")).digest()
    observed_groups = []
    observed_group_ids = set()
    previous_group_count = 0
    for seq, page in enumerate(pages):
        raw = _hex_bytes(page, "raw_hex", 372)
        if page.get("boot_epoch") != boot or page.get("session_id") != session:
            raise ValueError("unlinked evidence identity")
        major, minor, page_run, page_seq, group_id, _, page_epoch = struct.unpack_from(
            "<HHQIQII", raw)
        category, expected, submitted, sent, failed, final = raw[32:38]
        group_count, attempt_id, page_kind, page_parent, motion_complete, release_committed, state, _ = \
            struct.unpack_from("<IQBQBBBH", raw, 281)
        terminal = seq == len(pages) - 1
        if (group_count > len(pages) - 1 or group_count < previous_group_count or
                (terminal and group_count != len(pages) - 1) or
                (not terminal and group_count < seq + 1)):
            raise ValueError("inconsistent evidence group count")
        previous_group_count = group_count
        expected_epoch = (epoch if terminal_epoch is None else terminal_epoch) if terminal \
            else epoch if group_epochs is None else group_epochs[seq]
        if attempt_id != run:
            raise ValueError("unlinked raw attempt id")
        if (major, minor, page_run, page_seq, page_epoch, page_kind, page_parent) != \
                (2, 4, run, seq, expected_epoch, kind, parent) or raw[371] != 0:
            raise ValueError("unlinked or lost evidence page")
        if raw[307:339] != previous or hashlib.sha256(
                previous + raw[:307] + raw[371:372]).digest() != raw[339:371]:
            raise ValueError("invalid evidence chain")
        previous = raw[339:371]
        if not terminal:
            if (category not in (1, 2, 3) or group_id == 0 or group_id in observed_group_ids or
                    expected != 63 or
                    submitted != 63 or sent != 63 or failed != 0 or final != 1 or
                    state != 0 or motion_complete or release_committed or
                    group_count <= seq):
                raise ValueError("incomplete six-axis CAN evidence")
            observed_groups.append(group_id)
            observed_group_ids.add(group_id)
        elif (category != 4 or group_id != 0 or expected or submitted or sent or failed or
              final != 1 or group_count != len(observed_groups) or state != final_state or
              motion_complete != int(final_state == 6) or
              release_committed != int(final_state in (2, 8))):
            raise ValueError("missing or invalid terminal evidence")
    if group_ids is not None and observed_groups != group_ids:
        raise ValueError("CAN groups do not match terminal chain")


def _validate_timeline(goal, final_goal, lag_budget):
    rows = _required(goal, "timeline")
    terminal = {"SUCCEEDED": "terminal_success", "CANCELED": "terminal_canceled",
                "ABORTED": "terminal_aborted"}[goal["action_status"]]
    events = [row.get("event") if isinstance(row, dict) else None for row in rows] \
        if isinstance(rows, list) else []
    tail = [terminal, "pre_release_hold"] + (["post_release_status"] if final_goal else [])
    feedback_events = events[3:-len(tail)]
    if (events[:3] != ["accepted", "before_begin", "after_begin"] or
            not feedback_events or any(event != "feedback" for event in feedback_events) or
            events[-len(tail):] != tail):
        raise ValueError("incomplete or unordered goal timeline")
    release_index = len(rows) - 1 if final_goal else -1
    hold_index = len(rows) - 2 if final_goal else len(rows) - 1
    goal_epoch = goal["motion_epoch"]
    before_epoch = rows[1].get("motion_epoch")
    hold_epoch = rows[hold_index].get("motion_epoch")
    mixed_success = goal["action_status"] == "SUCCEEDED"
    if goal["action_status"] == "CANCELED" and rows[0].get("motion_epoch") != before_epoch:
        raise ValueError("accepted motion epoch differs from before Begin")
    if mixed_success and (type(before_epoch) is not int or
                          type(hold_epoch) is not int or
                          before_epoch not in (goal_epoch - 1, goal_epoch) or
                          hold_epoch not in (goal_epoch, goal_epoch + 1) or
                          rows[0].get("motion_epoch") != before_epoch):
        raise ValueError("invalid Begin/Complete motion epoch transition")
    seen_hold_feedback = False
    seen_goal_feedback = False
    seen_stop_feedback = False
    for index, row in enumerate(rows):
        canceled_terminal = goal["action_status"] == "CANCELED" and index == hold_index - 1
        cancel_request_id = row.get("cancel_request_id", 0)
        cancel_operation_id = row.get("cancel_operation_id", 0)
        if canceled_terminal:
            if (type(cancel_request_id) is not int or
                    type(cancel_operation_id) is not int or
                    cancel_request_id <= 0 or cancel_operation_id <= 0 or
                    cancel_request_id != goal.get("cancel_request_id") or
                    cancel_operation_id != goal.get("cancel_operation_id")):
                raise ValueError("cancel Gate key does not match current action")
        elif cancel_request_id != 0 or cancel_operation_id != 0:
            raise ValueError("cancel Gate key appears outside canceled terminal")
        if ((index != release_index and
             (row.get("release_request_id", 0) != 0 or
              row.get("release_operation_id", 0) != 0)) or
            (index == release_index and
             (type(row.get("release_request_id")) is not int or
              type(row.get("release_operation_id")) is not int or
              row["release_request_id"] <= 0 or row["release_operation_id"] <= 0))):
            raise ValueError("release request key appears on wrong Gate row")
        if (row.get("goal_uuid") != goal["goal_id"] or
                row.get("session_id") != (0 if index == release_index else goal["session_id"]) or
                row.get("boot_epoch") != goal["boot_epoch"] or
                row.get("axis_mask") != 63 or
                type(row.get("steady_us")) is not int or row["steady_us"] < 0):
            raise ValueError("unlinked goal timeline")
        expected_epoch = goal.get("stop_epoch", goal_epoch) \
            if index == hold_index or (goal["action_status"] == "CANCELED" and
                                       index == hold_index - 1) else goal_epoch
        observed_epoch = row.get("motion_epoch")
        if mixed_success and 3 <= index < hold_index - 1:
            if observed_epoch == goal_epoch and not seen_hold_feedback:
                seen_goal_feedback = True
            elif observed_epoch == hold_epoch:
                seen_hold_feedback = True
            else:
                raise ValueError("old motion epoch resumed in feedback")
            expected_epoch = observed_epoch
        elif goal["action_status"] == "CANCELED" and 3 <= index < hold_index - 1:
            if observed_epoch == goal_epoch and not seen_stop_feedback:
                expected_epoch = goal_epoch
            elif observed_epoch == goal["stop_epoch"]:
                seen_stop_feedback = True
                expected_epoch = observed_epoch
            else:
                raise ValueError("old motion epoch resumed in canceled feedback")
        elif mixed_success and hold_index - 1 <= index <= hold_index:
            expected_epoch = hold_epoch
        if 2 <= index <= hold_index and (row.get("run_id") != goal["run_id"] or
                                         observed_epoch != expected_epoch):
            raise ValueError("unlinked post-Begin goal timeline")
        if index >= hold_index and (row.get("status_core_state"), row.get("state_core_state")) != \
                ((1, 1) if index == release_index else (4, 4)):
            raise ValueError("unconfirmed timeline HOLD or release")
    if mixed_success and not seen_goal_feedback:
        raise ValueError("missing goal-epoch six-axis feedback")
    event_times = [rows[index]["steady_us"] for index in
                   (0, 1, 2, hold_index - 1, hold_index)]
    if final_goal:
        event_times.append(rows[release_index]["steady_us"])
    if any(later < earlier for earlier, later in zip(event_times, event_times[1:])):
        raise ValueError("regressing goal event time")
    feedback_times = [row["steady_us"] for row in rows[3:hold_index - 1]]
    if (any(later < earlier for earlier, later in zip(feedback_times, feedback_times[1:])) or
            feedback_times[0] < rows[0]["steady_us"] or
            feedback_times[0] < rows[2]["steady_us"] - lag_budget or
            feedback_times[-1] > rows[hold_index - 1]["steady_us"]):
        raise ValueError("feedback receive time outside goal window or producer lag budget")
    for feedback in rows[3:hold_index - 1]:
        ages = _six_finite(feedback, "feedback_age_us")
        sequences = _six_finite(feedback, "feedback_seq", True)
        if (any(age < 0 or age > goal["feedback_age_budget_us"] for age in ages) or
                any(type(seq) is not int for seq in sequences)):
            raise ValueError("stale six-axis timeline feedback")
    if ages != goal["feedback_age_us"]:
        raise ValueError("unlinked final six-axis feedback")


def validate_evidence(evidence):
    if not isinstance(evidence, dict) or evidence.get("source") != "ros_moveit_jtc_usb_ref_can":
        raise ValueError("real ROS/MoveIt/JTC/CAN source required")
    if evidence.get("profile_id") != 201 or evidence.get("configured_mask") != 63:
        raise ValueError("six-axis bench identity required")
    environment = _required(evidence, "environment")
    if not isinstance(environment, dict):
        raise ValueError("invalid environment")
    lag = environment.get("producer_lag_us")
    budget = environment.get("producer_lag_budget_us")
    if type(lag) is not int or type(budget) is not int or lag < 0 or budget <= 0:
        raise ValueError("invalid producer lag evidence")
    version = environment.get("producer_lag_gate_version")
    if version is not None:
        window = environment.get("producer_lag_window")
        if (version != 2 or
                environment.get("producer_lag_method") !=
                "state_receive_minus_produced_offset_span_motion_window" or
                budget != 250000 or
                type(environment.get("producer_lag_environment_us")) is not int or
                environment["producer_lag_environment_us"] < lag or
                not isinstance(window, dict) or
                any(type(window.get(key)) is not int or window[key] <= 0 for key in
                    ("start_steady_us", "end_steady_us", "release_run_id")) or
                type(window.get("state_samples")) is not int or
                window["state_samples"] < 2 or
                window["start_steady_us"] >= window["end_steady_us"]):
            raise ValueError("invalid window producer lag evidence")
        if lag > budget:
            raise ValueError("window producer lag budget exceeded")
    goals = _required(evidence, "goals")
    if not isinstance(goals, list):
        raise ValueError("invalid goals")
    identity = _required(evidence, "identity")
    if not isinstance(identity, dict):
        raise ValueError("invalid identity")
    device_id = _hex_bytes(identity, "device_id", 16).hex()
    if device_id == "00" * 16:
        raise ValueError("invalid device identity")
    goal_ids = [goal["goal_id"] for goal in goals if isinstance(goal, dict) and
                isinstance(goal.get("goal_id"), str)]
    if len(goal_ids) != len(set(goal_ids)):
        raise ValueError("duplicate ROS goal id")
    run_keys = [(goal.get("boot_epoch"), goal.get("session_id"), goal.get("run_id"))
                for goal in goals if isinstance(goal, dict) and
                all(type(goal.get(name)) is int for name in
                    ("boot_epoch", "session_id", "run_id"))]
    if len(run_keys) != len(set(run_keys)):
        raise ValueError("duplicate device run across ROS goals")
    seen = set()
    previous_hold_time = -1
    for goal_index, goal in enumerate(goals):
        if not isinstance(goal, dict) or goal.get("device_id") != device_id:
            raise ValueError("goal changed device identity")
        goal_id = _required(goal, "goal_id")
        if (not isinstance(goal_id, str) or len(goal_id) != 32 or
                any(character not in "0123456789abcdef" for character in goal_id) or
                goal_id in seen):
            raise ValueError("invalid or duplicate ROS goal id")
        seen.add(goal_id)
        run = _positive_int(goal, "run_id")
        session = _positive_int(goal, "session_id")
        boot = _positive_int(goal, "boot_epoch")
        epoch = _positive_int(goal, "motion_epoch")
        groups = _required(goal, "groups")
        if not isinstance(groups, list):
            raise ValueError("invalid groups")
        for group in groups:
            if (_positive_int(group, "group_id") <= 0 or group.get("run_id") != run or
                    group.get("session_id") != session or group.get("axis_mask") != 63 or
                    group.get("sent_mask") != 63 or group.get("committed") is not True):
                raise ValueError("unlinked or incomplete six-axis CAN group")
        status = _required(goal, "action_status")
        if status != "ABORTED" or goal.get("fault_domain") != "ENVIRONMENT":
            ages = _six_finite(goal, "feedback_age_us")
            age_budget = _positive_int(goal, "feedback_age_budget_us")
            if any(age < 0 or age > age_budget for age in ages):
                raise ValueError("stale six-axis feedback")
            _validate_timeline(goal, goal_index == len(goals) - 1, budget)
            if goal["timeline"][0]["steady_us"] < previous_hold_time:
                raise ValueError("cross-goal timeline regression")
            previous_hold_time = goal["timeline"][-2 if goal_index == len(goals) - 1
                                                       else -1]["steady_us"]
        if status == "SUCCEEDED":
            if (goal.get("result_error_code") != 0 or
                    goal.get("terminal") != "MOTION_COMPLETED_HOLDING" or
                    goal.get("hold_confirmed") is not True or lag > budget):
                raise ValueError("unconfirmed ROS success or producer lag")
            target = _six_finite(goal, "target_rad")
            feedback = _six_finite(goal, "feedback_rad")
            path_error = _six_finite(goal, "path_error_max_rad")
            path_limit = _six_finite(goal, "path_tolerance_rad", True)
            goal_limit = _six_finite(goal, "goal_tolerance_rad", True)
            if any(abs(actual - planned) > tolerance for planned, actual, tolerance in
                   zip(target, feedback, goal_limit)) or any(
                       error < 0 or error > tolerance for error, tolerance in zip(path_error, path_limit)):
                raise ValueError("six-axis trajectory tolerance exceeded")
            before_epoch = goal["timeline"][1]["motion_epoch"]
            hold_epoch = goal["timeline"][-2 if goal_index == len(goals) - 1 else -1]["motion_epoch"]
            group_epochs = [group.get("motion_epoch") for group in groups]
            if ((before_epoch != epoch or hold_epoch != epoch) and
                    (before_epoch, hold_epoch) != (epoch - 1, epoch + 1)):
                raise ValueError("invalid Hold/goal/Complete epoch transition")
            if (not group_epochs or group_epochs[0] != before_epoch or any(group_epoch not in
                    (before_epoch, epoch, hold_epoch) for group_epoch in group_epochs) or
                    group_epochs != sorted(group_epochs) or epoch not in group_epochs or
                    (hold_epoch != epoch and hold_epoch not in group_epochs)):
                raise ValueError("invalid Hold/goal/Complete evidence phase")
            terminal_state = 6
        elif status == "CANCELED":
            stop_epoch = _positive_int(goal, "stop_epoch")
            group_epochs = [group.get("motion_epoch") for group in groups]
            before_epoch = goal["timeline"][1]["motion_epoch"]
            if stop_epoch != epoch + 1:
                raise ValueError("cancel stop epoch must advance once")
            if (goal.get("terminal") != "CANCELED_HOLDING" or
                    goal.get("stop_confirmed") is not True or
                    stop_epoch not in group_epochs):
                raise ValueError("cancel has no confirmed new-epoch stop")
            first_stop = group_epochs.index(stop_epoch)
            if (before_epoch not in (epoch - 1, epoch) or
                    group_epochs[0] != before_epoch or
                    group_epochs != sorted(group_epochs) or
                    epoch not in group_epochs[:first_stop] or
                    any(group_epoch != stop_epoch for group_epoch in group_epochs[first_stop:])):
                raise ValueError("old motion epoch resumed after cancel stop")
            categories = [bytes.fromhex(page["raw_hex"])[32] for page in
                          goal["evidence_pages"][:-1]]
            if (len(categories) != len(groups) or
                    any(category != 1 for category in categories[:first_stop]) or
                    categories[first_stop] != 2 or
                    any(category not in (1, 2) for category in categories[first_stop:]) or
                    any(bytes.fromhex(page["raw_hex"])[38:41] != b"\0\0\0" for page in
                        goal["evidence_pages"][first_stop:-1])):
                raise ValueError("cancel lacks new-epoch six-axis Hold SENT")
            terminal_state = 7
        elif status == "ABORTED":
            if lag > budget and goal.get("fault_domain") != "ENVIRONMENT":
                raise ValueError("producer lag attributed to device")
            if lag <= budget and goal.get("fault_domain") == "ENVIRONMENT":
                raise ValueError("device fault masked as environment")
            terminal_state = {"FAULT_LATCHED_AND_RELEASED": 4,
                              "FAULT_LATCHED_UNCOMMUNICABLE": 5,
                              "FAULT_LATCHED_HOLDING": 9}.get(goal.get("terminal"))
            if terminal_state is None and goal.get("fault_domain") != "ENVIRONMENT":
                raise ValueError("invalid abort terminal")
        else:
            raise ValueError("unsupported ROS action status")
        if terminal_state is not None:
            _validate_pages(goal, identity, run=run, session=session, boot=boot,
                            epoch=epoch,
                            kind=1, parent=0, final_state=terminal_state,
                            group_ids=[group["group_id"] for group in groups],
                            group_epochs=[group["motion_epoch"] for group in groups],
                            terminal_epoch=before_epoch if status in ("SUCCEEDED", "CANCELED") else None)
    if all(goal["action_status"] == "ABORTED" and
           goal.get("fault_domain") == "ENVIRONMENT" for goal in goals):
        raise ValueError("environment-only evidence cannot validate device motion")
    last = goals[-1]
    release_epoch = last.get("stop_epoch", last["timeline"][-2]["motion_epoch"]
                             if last["action_status"] == "SUCCEEDED" else last["motion_epoch"])
    hold = _required(evidence, "pre_release_hold")
    if (not isinstance(hold, dict) or hold.get("confirmed") is not True or
            any(hold.get(name) != last.get(name) for name in
                ("run_id", "session_id", "boot_epoch")) or
            hold.get("motion_epoch") != release_epoch):
        raise ValueError("missing pre-release HOLD confirmation")
    release = _required(evidence, "release")
    if version == 2:
        window = environment["producer_lag_window"]
        if (not isinstance(release, dict) or
                window["start_steady_us"] != goals[0]["timeline"][0]["steady_us"] or
                window["end_steady_us"] != last["timeline"][-1]["steady_us"] or
                window["release_run_id"] != release.get("run_id")):
            raise ValueError("window producer lag identity mismatch")
    if (not isinstance(release, dict) or release.get("confirmed") is not True or
            release.get("terminal") not in ("RELEASED", "CANCELED_AND_RELEASED") or
            release.get("parent_run_id") != last["run_id"] or
            release.get("session_id") != last["session_id"] or
            release.get("boot_epoch") != last["boot_epoch"] or
            _positive_int(release, "run_id") <= last["run_id"]):
        raise ValueError("unconfirmed or unlinked release")
    if last["timeline"][-1].get("run_id") != release["run_id"]:
        raise ValueError("release timeline run is unlinked")
    if (last["timeline"][-1].get("release_request_id") != release.get("request_id") or
            last["timeline"][-1].get("release_operation_id") != release.get("operation_id")):
        raise ValueError("release timeline request key is unlinked")
    active = release["terminal"] == "CANCELED_AND_RELEASED"
    if last["action_status"] == "CANCELED" and (
            active or release["run_id"] != last["run_id"] + 1 or
            not any(bytes.fromhex(page["raw_hex"])[32] == 3 for page in
                              release["evidence_pages"][:-1])):
        raise ValueError("cancel release lacks next-run six-axis Disable SENT")
    group_epochs = None
    if active:
        group_pages = release["evidence_pages"][:-1]
        categories = [bytes.fromhex(page["raw_hex"])[32] for page in group_pages]
        group_epochs = [struct.unpack_from("<I", bytes.fromhex(page["raw_hex"]), 28)[0]
                        for page in group_pages]
        if (release_epoch >= 0xffffffff or len(categories) < 2 or
                1 not in categories or
                3 not in categories or
                categories != sorted(categories) or
                any(category not in (1, 2, 3) for category in categories) or
                any(category == 2 and (
                    struct.unpack_from("<H", bytes.fromhex(page["raw_hex"]), 38)[0] != 0 or
                    bytes.fromhex(page["raw_hex"])[40] != 0)
                    for category, page in zip(categories, group_pages)) or
                any(group_epoch != release_epoch for category, group_epoch in
                    zip(categories, group_epochs) if category == 1) or
                any(group_epoch != release_epoch + 1 for category, group_epoch in
                    zip(categories, group_epochs) if category == 2) or
                group_epochs[-1] not in (release_epoch, release_epoch + 1) or
                any(group_epoch != release_epoch + 1 for category, group_epoch in
                    zip(categories, group_epochs) if category == 3) or
                group_epochs != sorted(group_epochs) or
                last["timeline"][-1]["motion_epoch"] != release_epoch + 1):
            raise ValueError("active release requires Hold then Disable motion epoch phase")
    else:
        group_pages = release["evidence_pages"][:-1]
        if (release_epoch >= 0xffffffff or len(group_pages) != 1 or
                bytes.fromhex(group_pages[0]["raw_hex"])[32] != 3 or
                last["timeline"][-1]["motion_epoch"] != release_epoch + 1):
            raise ValueError("release lacks next-epoch six-axis Disable SENT")
        group_epochs = [release_epoch + 1]
    _validate_pages(release, identity, run=release["run_id"], session=release["session_id"],
                    boot=release["boot_epoch"], epoch=release_epoch,
                    kind=1 if active else 3, parent=0 if active else last["run_id"],
                    final_state=2 if active else 8,
                    group_epochs=group_epochs, terminal_epoch=release_epoch)
    if not any(goal["action_status"] in ("SUCCEEDED", "CANCELED") for goal in goals):
        raise ValueError("confirmed six-axis trajectory or cancel evidence required")
    _validate_management(evidence, goals, release)


def main():
    parser = argparse.ArgumentParser(description="校验真实六轴 ROS 原始证据关联")
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    for name in ("timeline", "raw", "action", "context"):
        parser.add_argument(f"--{name}", type=Path)
    args = parser.parse_args()
    try:
        evidence = json.loads(args.evidence.read_text())
        validate_evidence(evidence)
        captures = (args.timeline, args.raw, args.action, args.context)
        if any(captures) and not all(captures):
            raise ValueError("positive E2E requires timeline/raw/action/context capture")
        status = "SOFTWARE_EVIDENCE_VALIDATED"
        if all(captures):
            spec = importlib.util.spec_from_file_location(
                "join_real_6dof_evidence", Path(__file__).with_name("join_real_6dof_evidence.py"))
            joiner = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(joiner)
            if joiner.join_evidence(*captures) != evidence:
                raise ValueError("evidence differs from SHA-pinned raw capture join")
            status = "ROS_BENCH_E2E_POSITIVE"
        result = {"status": status, "goals": len(evidence["goals"]),
                  "qualification": "UNQUALIFIED", "hardware_qualification": "NOT_ASSERTED"}
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        result = {"status": "FAIL", "reason": str(error), "qualification": "UNQUALIFIED",
                  "hardware_qualification": "NOT_ASSERTED"}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["status"] != "FAIL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
