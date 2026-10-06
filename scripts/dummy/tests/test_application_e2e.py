"""P8 六轴 ROS 原始证据的最小拒绝门禁。"""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
from time import perf_counter

import pytest


MODULE = Path(__file__).resolve().parents[1] / "application_e2e.py"
SPEC = importlib.util.spec_from_file_location("application_e2e", MODULE)
application_e2e = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(application_e2e)
LAYOUT = json.loads((Path(__file__).resolve().parents[3] /
    "contracts/dummy_robot_v2/codec/wire_layout.json").read_text())


def manage(kind, when, *, operation=0, request_id=1, run=0, session=3,
           phase=3, result=0, operation_id=1, client="11" * 16,
           queried_operation_id=0, motion_epoch=0, expected_motion_epoch=0,
           request_seq=None,
           motion_complete=None, release_committed=None):
    name = "DummyManageV2Request" if kind == 4 else "DummyManageV2Response"
    raw = bytearray(LAYOUT["payloads"][name]["encoded_size"])
    values = {"schema_major": 2, "schema_minor": 4, "client_instance_id": client,
              "device_id": "22" * 16, "request_id": request_id,
              "request_seq": request_id if request_seq is None else request_seq,
              "operation": operation,
              "queried_operation_id": queried_operation_id,
              "expected_boot_epoch": 2, "expected_session_id": 3,
              "expected_motion_epoch": expected_motion_epoch,
              "expected_profile_id": 201, "transaction_phase": phase,
              "result_code": result, "operation_id": operation_id,
              "receipt": 1,
              "motion_complete": int(kind == 5 and session != 0)
                                 if motion_complete is None else motion_complete,
              "release_committed": int(kind == 5 and session == 0)
                                   if release_committed is None else release_committed,
              "boot_epoch": 2, "session_id": session, "profile_id": 201,
              "run_id": run, "motion_epoch": motion_epoch}
    codes = {"uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q",
             "float64": "d", "bool": "B"}
    for field in LAYOUT["payloads"][name]["fields"]:
        value = values.get(field["name"])
        if value is None:
            continue
        if field["count"] > 1:
            value = list(bytes.fromhex(value) if isinstance(value, str) else value)
        else:
            value = [value]
        struct.pack_into("<" + codes[field["type"]] * field["count"], raw,
                         field["offset"], *value)
    return {"kind": kind, "when": when, "boot": 2,
            "session": 3 if kind == 4 else session,
            "run": 0 if kind == 4 else run,
            "seq": (request_id if request_seq is None else request_seq)
            if kind == 4 else 0, "raw_hex": raw.hex()}


def pages(identity, *, run, session, boot=2, epoch=2, kind=1, parent=0,
          final_state=6, group_id=7, group_ids=None, group_epochs=None,
          terminal_epoch=None, group_counts=None, attempt_id=None,
          categories=None, safety_failure_source=0):
    group_ids = [group_id] if group_ids is None else group_ids
    group_epochs = [epoch] * len(group_ids) if group_epochs is None else group_epochs
    group_counts = list(range(1, len(group_ids) + 1)) if group_counts is None else group_counts
    categories = [1] * len(group_ids) if categories is None else categories
    seed = struct.pack("<QQ", run, boot) + bytes.fromhex(identity["client_id"])
    seed += b"".join(bytes.fromhex(identity[name]) for name in
                     ("contract_hash", "model_hash", "profile_hash", "firmware_hash"))
    previous = hashlib.sha256(seed.ljust(192, b"\0")).digest()
    result = []
    for seq in range(len(group_ids) + 1):
        terminal = seq == len(group_ids)
        raw = bytearray(372)
        struct.pack_into("<HHQIQII", raw, 0, 2, 4, run, seq,
                         0 if terminal else group_ids[seq], 0 if terminal else seq + 1,
                         (epoch if terminal_epoch is None else terminal_epoch) if terminal
                         else group_epochs[seq])
        raw[32:38] = bytes((4, 0, 0, 0, 0, 1)) if terminal else bytes(
            (categories[seq], 63, 63, 63, 0, 1))
        if not terminal and categories[seq] == 2:
            struct.pack_into("<H", raw, 38, safety_failure_source)
        struct.pack_into("<IQB QBBB H", raw, 281,
                         len(group_ids) if terminal else group_counts[seq],
                         run if attempt_id is None else attempt_id, kind, parent,
                         int(terminal and final_state == 6),
                         int(terminal and final_state in (2, 8)),
                         final_state if terminal else 0, 0)
        raw[307:339] = previous
        digest = hashlib.sha256(previous + raw[:307] + raw[371:372]).digest()
        raw[339:371] = digest
        result.append({"raw_hex": raw.hex(), "boot_epoch": boot, "session_id": session})
        previous = digest
    return result


@pytest.fixture
def evidence():
    identity = {name: "11" * width for name, width in
                (("client_id", 16), ("contract_hash", 32), ("model_hash", 32),
                 ("profile_hash", 32), ("firmware_hash", 32))}
    identity["device_id"] = "22" * 16
    result = {
        "source": "ros_moveit_jtc_usb_ref_can",
        "profile_id": 201,
        "configured_mask": 63,
        "identity": identity,
        "environment": {"producer_lag_us": 1000, "producer_lag_budget_us": 10000},
        "goals": [{
            "goal_id": "a" * 32, "action_status": "SUCCEEDED", "result_error_code": 0,
            "device_id": identity["device_id"],
            "run_id": 4, "session_id": 3, "boot_epoch": 2, "motion_epoch": 2,
            "target_rad": [0.01, -0.02, 1.59, -0.01, 0.02, -0.03],
            "feedback_rad": [0.01, -0.02, 1.59, -0.01, 0.02, -0.03],
            "path_error_max_rad": [0.001] * 6,
            "path_tolerance_rad": [0.01] * 6,
            "goal_tolerance_rad": [0.003] * 6,
            "feedback_age_us": [1000] * 6, "feedback_age_budget_us": 10000,
            "timeline": [
                {"event": event, "goal_uuid": "a" * 32, "boot_epoch": 2,
                 "session_id": 0 if event == "post_release_status" else 3,
                 "run_id": 5 if event == "post_release_status" else 4,
                 "motion_epoch": 3 if event == "post_release_status" else 2,
                 "steady_us": 1000 * (index + 1), "axis_mask": 63,
                 "release_request_id": 2 if event == "post_release_status" else 0,
                 "release_operation_id": 2 if event == "post_release_status" else 0,
                 "status_core_state": 1 if event == "post_release_status" else 4 if event == "pre_release_hold" else 3,
                 "state_core_state": 1 if event == "post_release_status" else 4 if event == "pre_release_hold" else 3,
                 **({"feedback_age_us": [1000] * 6,
                     "feedback_seq": [1] * 6} if event == "feedback" else {})}
                for index, event in enumerate(
                    ("accepted", "before_begin", "after_begin", "feedback",
                     "terminal_success", "pre_release_hold", "post_release_status"))],
            "evidence_pages": pages(identity, run=4, session=3),
            "groups": [{"group_id": 7, "run_id": 4, "session_id": 3,
                        "motion_epoch": 2, "axis_mask": 63, "sent_mask": 63,
                        "committed": True}],
            "terminal": "MOTION_COMPLETED_HOLDING", "hold_confirmed": True,
        }],
        "pre_release_hold": {"run_id": 4, "session_id": 3, "boot_epoch": 2,
                             "motion_epoch": 2, "confirmed": True},
        "management_records": [manage(4, 4800, operation=8, request_id=1),
                               manage(5, 5000, request_id=1, run=4),
                               manage(4, 6200, operation=10, request_id=2),
                               manage(5, 6700, request_id=2, run=5, session=0,
                                      operation_id=2, motion_epoch=3)],
            "release": {"run_id": 5, "parent_run_id": 4, "session_id": 3,
                    "boot_epoch": 2, "terminal": "RELEASED", "confirmed": True,
                    "request_id": 2, "operation_id": 2,
                    "evidence_pages": pages(identity, run=5, session=3, kind=3,
                                            parent=4, final_state=8, group_id=8,
                                            group_epochs=[3], terminal_epoch=2,
                                            categories=[3])},
    }
    return result


def test_six_axis_evidence_passes(evidence):
    application_e2e.validate_evidence(evidence)


def test_other_device_goal_cannot_share_application_evidence(evidence):
    evidence["goals"][0]["device_id"] = "33" * 16
    with pytest.raises(ValueError, match="device"):
        application_e2e.validate_evidence(evidence)


def test_other_device_management_receipt_cannot_share_application_evidence(evidence):
    for item in evidence["management_records"][:2]:
        raw = bytearray(bytes.fromhex(item["raw_hex"]))
        name = "DummyManageV2Request" if item["kind"] == 4 else "DummyManageV2Response"
        offset = next(field["offset"] for field in
            LAYOUT["payloads"][name]["fields"] if field["name"] == "device_id")
        raw[offset:offset + 16] = bytes.fromhex("33" * 16)
        item["raw_hex"] = raw.hex()
    with pytest.raises(ValueError, match="management record identity"):
        application_e2e.validate_evidence(evidence)


def test_release_without_management_receipts_is_not_positive(evidence):
    del evidence["management_records"]
    with pytest.raises(ValueError, match="management|receipt"):
        application_e2e.validate_evidence(evidence)


def active_release(evidence):
    evidence["release"].update(terminal="CANCELED_AND_RELEASED",
        evidence_pages=pages(evidence["identity"], run=5, session=3,
                             final_state=2, group_ids=[8, 9],
                             group_epochs=[2, 3], terminal_epoch=2,
                             categories=[1, 3]))
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    return evidence


def test_active_hold_refresh_release_requires_matching_committed_receipt(evidence):
    active_release(evidence)
    application_e2e.validate_evidence(evidence)


def test_active_release_rejects_disable_sent_without_new_epoch(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, final_state=2, group_ids=[8, 9],
        group_epochs=[2, 2], terminal_epoch=2, categories=[1, 3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 2
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=2)
    with pytest.raises(ValueError, match="Hold then Disable motion epoch phase"):
        application_e2e.validate_evidence(evidence)


def test_active_release_receipt_cannot_override_observed_hold(evidence):
    active_release(evidence)
    status = evidence["goals"][0]["timeline"][-1]
    status["status_core_state"] = 4
    status["state_core_state"] = 4
    with pytest.raises(ValueError, match="unconfirmed timeline HOLD or release"):
        application_e2e.validate_evidence(evidence)


def test_active_release_hold_disable_epochs_advance_then_terminal_uses_birth(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9],
        group_epochs=[2, 3], terminal_epoch=2, categories=[1, 3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    application_e2e.validate_evidence(evidence)


def test_active_release_accepts_bounded_safety_deceleration_before_disable(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9, 10],
        group_epochs=[2, 3, 3], terminal_epoch=2, categories=[1, 2, 3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("categories,epochs", [
    ([1, 3, 2], [2, 3, 3]),
    ([2, 1, 3], [3, 2, 3]),
    ([1, 2, 2], [2, 3, 3]),
    ([1, 2, 3], [2, 2, 3]),
])
def test_active_release_rejects_wrong_safety_stop_phase(evidence, categories, epochs):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9, 10],
        group_epochs=epochs, terminal_epoch=2, categories=categories)
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    with pytest.raises(ValueError, match="Hold then Disable|phase"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_fault_source_on_safety_deceleration(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9, 10],
        group_epochs=[2, 3, 3], terminal_epoch=2, categories=[1, 2, 3],
        safety_failure_source=17)
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    with pytest.raises(ValueError, match="Hold then Disable"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_more_than_ledger_group_capacity(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2,
        group_ids=list(range(8, 17)), group_epochs=[2] + [3] * 8,
        terminal_epoch=2, categories=[1] + [2] * 7 + [3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    with pytest.raises(ValueError, match="Hold then Disable"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("group_epochs,terminal_epoch", [
    ([2, 4], 2), ([3, 2], 2), ([2, 3], 3),
])
def test_active_release_rejects_skipped_regressing_or_wrong_terminal_epoch(
        evidence, group_epochs, terminal_epoch):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9],
        group_epochs=group_epochs, terminal_epoch=terminal_epoch,
        categories=[1, 3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = group_epochs[-1]
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=group_epochs[-1])
    with pytest.raises(ValueError, match="phase|evidence"):
        application_e2e.validate_evidence(evidence)


def test_active_release_accepts_query_result_for_same_request(evidence):
    active_release(evidence)
    evidence["management_records"][-1:] = [
        manage(4, 6600, operation=12, request_id=3, queried_operation_id=2),
        manage(5, 6700, request_id=3, run=5, session=0, operation_id=2,
               motion_epoch=3)]
    application_e2e.validate_evidence(evidence)


def retrying_release_query(evidence):
    active_release(evidence)
    query = manage(4, 6500, operation=12, request_id=15082018622492830540,
                   request_seq=2093819891, queried_operation_id=2)
    evidence["management_records"][-1:] = [
        query, {**query, "when": 6600},
        manage(5, 6700, request_id=15082018622492830540,
               run=5, session=0, operation_id=2,
               motion_epoch=3)]
    return query


def test_active_release_accepts_identical_query_retry_before_one_result(evidence):
    query = retrying_release_query(evidence)
    application_e2e.validate_evidence(evidence)
    assert evidence["management_records"][-3]["raw_hex"] == query["raw_hex"]
    assert evidence["management_records"][-2]["raw_hex"] == query["raw_hex"]


def test_active_release_rejects_mutated_query_retry(evidence):
    retrying_release_query(evidence)
    retry = evidence["management_records"][-2]
    raw = bytearray.fromhex(retry["raw_hex"])
    raw[-1] ^= 1
    retry["raw_hex"] = raw.hex()
    with pytest.raises(ValueError, match="duplicate management request identity"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_query_replay_after_response(evidence):
    retrying_release_query(evidence)
    records = evidence["management_records"]
    records[-2], records[-1] = {**records[-1], "when": 6600}, {**records[-2], "when": 6700}
    with pytest.raises(ValueError, match="duplicate management request identity"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_backdated_query_retry(evidence):
    retrying_release_query(evidence)
    evidence["management_records"][-2]["when"] = 6499
    with pytest.raises(ValueError, match="regressing management record time"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_query_beyond_three_retries(evidence):
    retrying_release_query(evidence)
    records = evidence["management_records"]
    records[-1:-1] = [{**records[-2], "when": when} for when in (6620, 6640, 6660)]
    with pytest.raises(ValueError, match="duplicate management request identity"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_repeated_release_mutation(evidence):
    active_release(evidence)
    original = evidence["management_records"][-2]
    evidence["management_records"].insert(-1, {**original, "when": 6500})
    with pytest.raises(ValueError, match="duplicate management request identity"):
        application_e2e.validate_evidence(evidence)


def test_active_release_retried_query_rejects_duplicate_committed_result(evidence):
    retrying_release_query(evidence)
    response = evidence["management_records"][-1]
    evidence["management_records"].append({**response, "when": 6800})
    with pytest.raises(ValueError, match="ambiguous committed management receipt"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("operation,receipt", [
    (8, 0), (8, 3), (8, 4), (10, 0), (10, 3), (10, 4)])
def test_committed_management_requires_confirmed_receipt(evidence, operation, receipt):
    index = 1 if operation == 8 else 3
    response = evidence["management_records"][index]
    raw = bytearray.fromhex(response["raw_hex"])
    offset = next(field["offset"] for field in
                  LAYOUT["payloads"]["DummyManageV2Response"]["fields"]
                  if field["name"] == "receipt")
    raw[offset] = receipt
    response["raw_hex"] = raw.hex()
    with pytest.raises(ValueError, match="receipt|identity"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("receipt", [1, 2])
def test_committed_management_accepts_confirmed_receipt_codes(evidence, receipt):
    response = evidence["management_records"][-1]
    raw = bytearray.fromhex(response["raw_hex"])
    offset = next(field["offset"] for field in
                  LAYOUT["payloads"]["DummyManageV2Response"]["fields"]
                  if field["name"] == "receipt")
    raw[offset] = receipt
    response["raw_hex"] = raw.hex()
    application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("field,value", [
    ("device_id", "33" * 16),
    ("expected_boot_epoch", 9),
    ("expected_session_id", 9),
    ("expected_reference_epoch", 9),
    ("expected_motion_epoch", 9),
    ("expected_management_epoch", 9),
    ("expected_profile_id", 9),
    ("configured_mask", 63),
])
def test_release_query_must_preserve_original_request_identity(evidence, field, value):
    active_release(evidence)
    query = manage(4, 6600, operation=12, request_id=3, queried_operation_id=2)
    raw = bytearray.fromhex(query["raw_hex"])
    layout = next(item for item in LAYOUT["payloads"]["DummyManageV2Request"]["fields"]
                  if item["name"] == field)
    if layout["count"] > 1:
        raw[layout["offset"]:layout["offset"] + layout["count"]] = bytes.fromhex(value)
    else:
        struct.pack_into("<" + {"uint8": "B", "uint16": "H", "uint32": "I",
                                "uint64": "Q"}[layout["type"]], raw, layout["offset"], value)
    query["raw_hex"] = raw.hex()
    if field == "expected_boot_epoch":
        query["boot"] = value
    elif field == "expected_session_id":
        query["session"] = value
    evidence["management_records"][-1:] = [
        query, manage(5, 6700, request_id=3, run=5, session=0, operation_id=2)]
    with pytest.raises(ValueError, match="receipt|identity"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_query_for_another_request(evidence):
    active_release(evidence)
    evidence["management_records"][-1:] = [
        manage(4, 6600, operation=12, request_id=3, queried_operation_id=1),
        manage(5, 6700, request_id=3, run=5, session=0, operation_id=2)]
    with pytest.raises(ValueError, match="receipt"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_duplicate_committed_receipt(evidence):
    active_release(evidence)
    evidence["management_records"].append(copy.deepcopy(evidence["management_records"][-1]))
    with pytest.raises(ValueError, match="ambiguous"):
        application_e2e.validate_evidence(evidence)


def test_active_release_rejects_safety_owned_attempt(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, kind=2, final_state=2,
        group_ids=[8, 9], categories=[2, 3])
    with pytest.raises(ValueError, match="Hold then Disable|evidence"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("field", ["request_id", "operation_id"])
def test_active_release_timeline_key_must_match_raw_receipt(evidence, field):
    active_release(evidence)
    evidence["release"][field] = 99
    with pytest.raises(ValueError, match="request key"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("change", [
    lambda records: records.pop(),
    lambda records: records[-1].update(raw_hex=manage(5, 6700, request_id=2,
        run=4, session=0, operation_id=2)["raw_hex"]),
    lambda records: records[-1].update(raw_hex=manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, phase=2)["raw_hex"]),
    lambda records: records[-1].update(raw_hex=manage(5, 6700, request_id=1,
        run=5, session=0, operation_id=2)["raw_hex"]),
])
def test_active_release_rejects_missing_cross_run_pending_or_old_receipt(evidence, change):
    active_release(evidence)
    change(evidence["management_records"])
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("categories", [[1, 1], [3, 3], [3, 1]])
def test_active_release_requires_hold_then_disable_sent(evidence, categories):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, final_state=2, group_ids=[8, 9], categories=categories)
    with pytest.raises(ValueError, match="Hold then Disable"):
        application_e2e.validate_evidence(evidence)


def mixed_epoch_success(evidence):
    goal = evidence["goals"][0]
    for row in goal["timeline"][:2]:
        row["motion_epoch"] = 1
    goal["timeline"][-3]["motion_epoch"] = 3
    goal["timeline"][-2]["motion_epoch"] = 3
    goal["timeline"][-1]["motion_epoch"] = 4
    goal["groups"] = [{**goal["groups"][0], "group_id": group_id,
                       "motion_epoch": epoch} for group_id, epoch in
                      ((6, 1), (7, 2), (8, 3))]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[6, 7, 8], group_epochs=[1, 2, 3], terminal_epoch=1)
    evidence["pre_release_hold"]["motion_epoch"] = 3
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=3, kind=3, parent=4, final_state=8,
        group_id=9, group_epochs=[4], terminal_epoch=3, categories=[3])
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=4)
    return evidence


def test_full_chain_keeps_pre_begin_hold_and_complete_hold_outside_goal(evidence):
    mixed_epoch_success(evidence)
    application_e2e.validate_evidence(evidence)


def test_mixed_success_requires_birth_epoch_raw_prefix(evidence):
    mixed_epoch_success(evidence)
    goal = evidence["goals"][0]
    goal["groups"] = [{**goal["groups"][0], "group_id": group_id,
                       "motion_epoch": epoch} for group_id, epoch in ((7, 2), (8, 3))]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8], group_epochs=[2, 3], terminal_epoch=2)
    with pytest.raises(ValueError, match="phase|evidence"):
        application_e2e.validate_evidence(evidence)


def test_mixed_success_requires_complete_epoch_after_goal(evidence):
    mixed_epoch_success(evidence)
    goal = evidence["goals"][0]
    goal["timeline"][-3]["motion_epoch"] = 2
    goal["timeline"][-2]["motion_epoch"] = 2
    goal["groups"][-1]["motion_epoch"] = 2
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[6, 7, 8], group_epochs=[1, 2, 2], terminal_epoch=1)
    evidence["pre_release_hold"]["motion_epoch"] = 2
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, kind=3, parent=4, final_state=8, group_id=9)
    with pytest.raises(ValueError, match="epoch"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("epochs,terminal_epoch,match", [
    ([1, 2, 1, 3], 1, "phase"),
    ([1, 3, 2], 1, "phase"),
    ([1, 1, 3], 1, "phase"),
    ([1, 2, 3], 2, "evidence"),
])
def test_mixed_epoch_chain_rejects_phase_replay_or_wrong_birth_epoch(
        evidence, epochs, terminal_epoch, match):
    mixed_epoch_success(evidence)
    goal = evidence["goals"][0]
    goal["groups"] = [{**goal["groups"][0], "group_id": 6 + index,
                       "motion_epoch": epoch} for index, epoch in enumerate(epochs)]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[6 + index for index in range(len(epochs))],
        group_epochs=epochs, terminal_epoch=terminal_epoch)
    with pytest.raises(ValueError, match=match):
        application_e2e.validate_evidence(evidence)


def test_two_goals_share_session_and_release_only_after_last(evidence):
    first = evidence["goals"][0]
    second = copy.deepcopy(first)
    first["timeline"].pop()
    second.update(goal_id="b" * 32, run_id=5,
                  evidence_pages=pages(evidence["identity"], run=5, session=3, group_id=8))
    second["groups"][0].update(group_id=8, run_id=5)
    for row in second["timeline"]:
        row["goal_uuid"] = second["goal_id"]
        row["steady_us"] += 10000
        if row["event"] not in ("accepted", "before_begin"):
            row["run_id"] = 6 if row["event"] == "post_release_status" else 5
        if row["event"] == "post_release_status":
            row.update(release_request_id=4, release_operation_id=4)
    evidence["goals"].append(second)
    evidence["pre_release_hold"]["run_id"] = 5
    evidence["release"].update(run_id=6, parent_run_id=5,
        request_id=4, operation_id=4,
        evidence_pages=pages(evidence["identity"], run=6, session=3,
                             kind=3, parent=5, final_state=8, group_id=9,
                             group_epochs=[3], terminal_epoch=2, categories=[3]))
    evidence["management_records"] = evidence["management_records"][:2] + [
        manage(4, 14800, operation=8, request_id=3),
        manage(5, 15000, request_id=3, run=5, operation_id=3),
        manage(4, 16200, operation=10, request_id=4),
        manage(5, 16700, request_id=4, run=6, session=0, operation_id=4,
               motion_epoch=3)]
    application_e2e.validate_evidence(evidence)
    overlap = copy.deepcopy(evidence)
    overlap["goals"][1]["timeline"][0]["steady_us"] = 5999
    with pytest.raises(ValueError, match="cross-goal"):
        application_e2e.validate_evidence(overlap)


def test_multi_group_evidence_count_grows_to_terminal_total(evidence):
    goal = evidence["goals"][0]
    goal["groups"].append({**goal["groups"][0], "group_id": 8})
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
                                    group_ids=[7, 8], group_counts=[1, 2])
    application_e2e.validate_evidence(evidence)


def test_large_page_chain_validation_scales_linearly(evidence):
    identity = evidence["identity"]
    durations = []
    for count in (3000, 30000):
        attempt = {"evidence_pages": pages(identity, run=4, session=3,
                                             group_ids=list(range(1, count + 1)))}
        start = perf_counter()
        application_e2e._validate_pages(
            attempt, identity, run=4, session=3, boot=2, epoch=2,
            kind=1, parent=0, final_state=6,
            group_ids=list(range(1, count + 1)))
        durations.append(perf_counter() - start)
    assert durations[1] < durations[0] * 25, durations


@pytest.mark.parametrize("mutation,match", [
    (lambda ids: [*ids[:-1], ids[0]], "six-axis CAN"),
    (lambda ids: list(reversed(ids)), "CAN groups"),
])
def test_page_chain_rejects_duplicate_or_reordered_groups(evidence, mutation, match):
    identity = evidence["identity"]
    expected_ids = list(range(1, 65))
    attempt = {"evidence_pages": pages(identity, run=4, session=3,
                                         group_ids=mutation(expected_ids))}
    with pytest.raises(ValueError, match=match):
        application_e2e._validate_pages(
            attempt, identity, run=4, session=3, boot=2, epoch=2,
            kind=1, parent=0, final_state=6, group_ids=expected_ids)


def test_page_chain_rejects_invalid_digest_after_many_valid_groups(evidence):
    identity = evidence["identity"]
    attempt = {"evidence_pages": pages(identity, run=4, session=3,
                                         group_ids=list(range(1, 65)))}
    raw = bytearray.fromhex(attempt["evidence_pages"][-2]["raw_hex"])
    raw[40] ^= 1
    attempt["evidence_pages"][-2]["raw_hex"] = raw.hex()
    with pytest.raises(ValueError, match="invalid evidence chain"):
        application_e2e._validate_pages(
            attempt, identity, run=4, session=3, boot=2, epoch=2,
            kind=1, parent=0, final_state=6, group_ids=list(range(1, 65)))


def test_wire_chain_seed_includes_zero_initialized_tail(evidence):
    identity = evidence["identity"]
    seed = struct.pack("<QQ", 4, 2) + bytes.fromhex(identity["client_id"])
    seed += b"".join(bytes.fromhex(identity[name]) for name in
                     ("contract_hash", "model_hash", "profile_hash", "firmware_hash"))
    raw = bytes.fromhex(evidence["goals"][0]["evidence_pages"][0]["raw_hex"])
    assert raw[307:339] == hashlib.sha256(seed.ljust(192, b"\0")).digest()


@pytest.mark.parametrize("field", ["goal_id", "run_id", "session_id", "motion_epoch", "groups"])
def test_missing_action_to_device_link_rejected(evidence, field):
    del evidence["goals"][0][field]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_duplicate_goal_uuid_rejected(evidence):
    evidence["goals"].append(copy.deepcopy(evidence["goals"][0]))
    with pytest.raises(ValueError, match="duplicate ROS goal id"):
        application_e2e.validate_evidence(evidence)


def test_duplicate_device_run_across_distinct_goals_rejected(evidence):
    first = evidence["goals"][0]
    first["timeline"].pop()
    second = copy.deepcopy(evidence["goals"][0])
    second["goal_id"] = "b" * 32
    for row in second["timeline"]:
        row["goal_uuid"] = second["goal_id"]
    second["timeline"].append({**evidence["goals"][0]["timeline"][-1],
                               "event": "post_release_status", "session_id": 0,
                               "steady_us": 7000, "status_core_state": 1,
                               "state_core_state": 1, "goal_uuid": second["goal_id"]})
    evidence["goals"].append(second)
    with pytest.raises(ValueError, match="duplicate device run"):
        application_e2e.validate_evidence(evidence)


def test_malformed_unhashable_run_is_validation_error(evidence):
    evidence["goals"][0]["run_id"] = [4]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("field,value", [
    ("run_id", 5), ("session_id", 4), ("motion_epoch", 3),
    ("axis_mask", 1), ("sent_mask", 62), ("committed", False),
])
def test_group_must_bind_all_six_axes_to_goal(evidence, field, value):
    evidence["goals"][0]["groups"][0][field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_empty_can_groups_rejected(evidence):
    evidence["goals"][0]["groups"] = []
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("field", ["evidence_pages", "feedback_age_us"])
def test_missing_raw_goal_evidence_rejected(evidence, field):
    del evidence["goals"][0][field]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("mutation", [
    lambda pages: pages.pop(0),
    lambda pages: pages.pop(),
    lambda pages: pages.append(copy.deepcopy(pages[-1])),
    lambda pages: pages[0].update(raw_hex=pages[0]["raw_hex"][:-2] + "01"),
    lambda pages: pages[0].update(session_id=9),
])
def test_raw_chain_gap_duplicate_tamper_or_wrong_session_rejected(evidence, mutation):
    mutation(evidence["goals"][0]["evidence_pages"])
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_raw_group_count_must_match_terminal_even_with_valid_chain(evidence):
    records = evidence["goals"][0]["evidence_pages"]
    first = bytearray.fromhex(records[0]["raw_hex"])
    struct.pack_into("<I", first, 281, 2)
    first[339:371] = hashlib.sha256(first[307:339] + first[:307] + first[371:372]).digest()
    records[0]["raw_hex"] = first.hex()
    terminal = bytearray.fromhex(records[1]["raw_hex"])
    terminal[307:339] = first[339:371]
    terminal[339:371] = hashlib.sha256(
        terminal[307:339] + terminal[:307] + terminal[371:372]).digest()
    records[1]["raw_hex"] = terminal.hex()
    with pytest.raises(ValueError, match="group count"):
        application_e2e.validate_evidence(evidence)


def test_raw_attempt_id_must_match_run_even_with_valid_chain(evidence):
    goal = evidence["goals"][0]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
                                    attempt_id=99)
    with pytest.raises(ValueError, match="attempt"):
        application_e2e.validate_evidence(evidence)


def test_partial_raw_can_sent_mask_rejected_even_when_json_claims_full(evidence):
    records = evidence["goals"][0]["evidence_pages"]
    first = bytearray.fromhex(records[0]["raw_hex"])
    first[35] = 62
    first[339:371] = hashlib.sha256(first[307:339] + first[:307] + first[371:372]).digest()
    records[0]["raw_hex"] = first.hex()
    terminal = bytearray.fromhex(records[1]["raw_hex"])
    terminal[307:339] = first[339:371]
    terminal[339:371] = hashlib.sha256(
        terminal[307:339] + terminal[:307] + terminal[371:372]).digest()
    records[1]["raw_hex"] = terminal.hex()
    with pytest.raises(ValueError, match="six-axis CAN"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("field,value", [
    ("feedback_age_us", [1000, 1000, 1000, 1000, 1000, 10001]),
    ("feedback_age_us", [1000] * 5),
    ("feedback_age_budget_us", 0),
])
def test_stale_or_incomplete_six_axis_feedback_rejected(evidence, field, value):
    evidence["goals"][0][field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_missing_pre_release_hold_rejected(evidence):
    del evidence["pre_release_hold"]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("field,value", [
    ("confirmed", False), ("parent_run_id", 2), ("session_id", 9),
    ("terminal", "UNCONFIRMED_STOP"),
])
def test_unconfirmed_or_unlinked_release_rejected(evidence, field, value):
    evidence["release"][field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_missing_release_terminal_rejected(evidence):
    evidence["release"]["evidence_pages"].pop()
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("field,value", [
    ("goal_uuid", "b" * 32), ("run_id", 9), ("session_id", 9),
    ("boot_epoch", 9), ("motion_epoch", 9), ("axis_mask", 1),
])
def test_timeline_event_must_bind_to_same_goal_and_run(evidence, field, value):
    evidence["goals"][0]["timeline"][3][field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("event", ["feedback", "terminal_success", "pre_release_hold"])
def test_timeline_requires_feedback_terminal_and_hold(evidence, event):
    evidence["goals"][0]["timeline"] = [row for row in
        evidence["goals"][0]["timeline"] if row["event"] != event]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_timeline_stale_feedback_rejected(evidence):
    evidence["goals"][0]["timeline"][3]["feedback_age_us"][5] = 10001
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_same_cycle_feedback_receive_time_can_precede_after_begin(evidence):
    evidence["environment"].update(producer_lag_us=50, producer_lag_budget_us=100)
    evidence["goals"][0]["timeline"][3]["steady_us"] = 2950
    application_e2e.validate_evidence(evidence)


def test_causal_rows_may_share_microsecond_timestamp(evidence):
    rows = evidence["goals"][0]["timeline"]
    rows[1]["steady_us"] = rows[0]["steady_us"]
    rows[5]["steady_us"] = rows[4]["steady_us"]
    application_e2e.validate_evidence(evidence)


def test_feedback_receive_time_outside_lag_budget_rejected(evidence):
    evidence["environment"].update(producer_lag_us=50, producer_lag_budget_us=100)
    evidence["goals"][0]["timeline"][3]["steady_us"] = 2800
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_feedback_receive_time_cannot_predate_goal_acceptance(evidence):
    evidence["environment"].update(producer_lag_us=50, producer_lag_budget_us=3000)
    evidence["goals"][0]["timeline"][3]["steady_us"] = 900
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_feedback_receive_times_cannot_run_backward(evidence):
    rows = evidence["goals"][0]["timeline"]
    later = copy.deepcopy(rows[3])
    later["steady_us"] = 3900
    rows[3]["steady_us"] = 3950
    rows.insert(4, later)
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_multiple_fresh_feedback_rows_accepted(evidence):
    rows = evidence["goals"][0]["timeline"]
    intermediate = copy.deepcopy(rows[3])
    intermediate["steady_us"] = 4500
    intermediate["feedback_seq"] = [2] * 6
    rows.insert(4, intermediate)
    application_e2e.validate_evidence(evidence)


def test_stale_intermediate_feedback_rejected(evidence):
    rows = evidence["goals"][0]["timeline"]
    intermediate = copy.deepcopy(rows[3])
    intermediate["steady_us"] = 4500
    intermediate["feedback_age_us"][0] = 10001
    rows.insert(4, intermediate)
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_post_release_status_must_clear_session(evidence):
    evidence["goals"][0]["timeline"][-1]["session_id"] = 3
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_post_release_status_must_bind_release_child_run(evidence):
    evidence["goals"][0]["timeline"][-1]["run_id"] = 99
    with pytest.raises(ValueError, match="release.*run"):
        application_e2e.validate_evidence(evidence)


def test_post_release_status_must_confirm_both_idle(evidence):
    evidence["goals"][0]["timeline"][-1]["state_core_state"] = 3
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_pre_release_hold_must_confirm_both_hold(evidence):
    evidence["goals"][0]["timeline"][-2]["status_core_state"] = 1
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_native_only_claim_rejected(evidence):
    evidence["source"] = "native_cli"
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("change", [
    lambda goal: goal.update(feedback_rad=[0.1] * 6),
    lambda goal: goal.update(path_error_max_rad=[0.1] * 6),
    lambda goal: goal["groups"][0].update(sent_mask=1),
    lambda goal: goal["groups"][0].update(motion_epoch=1),
    lambda goal: goal.update(hold_confirmed=False),
])
def test_false_success_rejected(evidence, change):
    change(evidence["goals"][0])
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_cancel_with_old_epoch_or_unconfirmed_stop_rejected(evidence):
    goal = copy.deepcopy(evidence["goals"][0])
    evidence["goals"][0]["timeline"].pop()
    goal.update(goal_id="b" * 32, run_id=5, action_status="CANCELED",
                terminal="CANCELED_HOLDING", stop_confirmed=True, stop_epoch=3)
    goal["groups"][0].update(group_id=8, run_id=5, motion_epoch=3)
    goal["evidence_pages"] = pages(evidence["identity"], run=5, session=3,
                                    epoch=2, group_epochs=[3], terminal_epoch=2,
                                    final_state=7, group_id=8)
    for row in goal["timeline"]:
        row["goal_uuid"] = goal["goal_id"]
        row["steady_us"] += 10000
        if row["event"] not in ("accepted", "before_begin"):
            row["run_id"] = 5
    goal["timeline"][4]["event"] = "terminal_canceled"
    goal["timeline"][4]["motion_epoch"] = 3
    goal["timeline"][5]["motion_epoch"] = 3
    evidence["goals"].append(goal)
    evidence["pre_release_hold"].update(run_id=5, motion_epoch=3)
    evidence["release"].update(run_id=6, parent_run_id=5,
        evidence_pages=pages(evidence["identity"], run=6, session=3,
                             epoch=3, kind=3, parent=5, final_state=8, group_id=9))
    goal["timeline"][-1]["run_id"] = 6
    with pytest.raises(ValueError, match="cancel Gate key"):
        application_e2e.validate_evidence(evidence)
    stale_release = copy.deepcopy(evidence)
    stale_release["release"]["evidence_pages"] = pages(evidence["identity"],
        run=6, session=3, epoch=2, kind=3, parent=5, final_state=8, group_id=9)
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(stale_release)
    for changed in (dict(stop_confirmed=False), dict(stop_epoch=2)):
        invalid = copy.deepcopy(evidence)
        invalid["goals"][1].update(changed)
        with pytest.raises(ValueError):
            application_e2e.validate_evidence(invalid)


def test_cancel_allows_motion_group_then_new_epoch_stop_group(evidence):
    goal = copy.deepcopy(evidence["goals"][0])
    evidence["goals"][0]["timeline"].pop()
    goal.update(goal_id="b" * 32, run_id=5, action_status="CANCELED",
                terminal="CANCELED_HOLDING", stop_confirmed=True, stop_epoch=3)
    goal["groups"] = [
        {**goal["groups"][0], "group_id": 8, "run_id": 5, "motion_epoch": 2},
        {**goal["groups"][0], "group_id": 9, "run_id": 5, "motion_epoch": 3},
    ]
    goal["evidence_pages"] = pages(evidence["identity"], run=5, session=3,
        group_ids=[8, 9], group_epochs=[2, 3], terminal_epoch=2, final_state=7)
    for row in goal["timeline"]:
        row["goal_uuid"] = goal["goal_id"]
        row["steady_us"] += 10000
        if row["event"] not in ("accepted", "before_begin"):
            row["run_id"] = 5
    goal["timeline"][4]["event"] = "terminal_canceled"
    goal["timeline"][4]["motion_epoch"] = 3
    goal["timeline"][5]["motion_epoch"] = 3
    evidence["goals"].append(goal)
    evidence["pre_release_hold"].update(run_id=5, motion_epoch=3)
    evidence["release"].update(run_id=6, parent_run_id=5,
        evidence_pages=pages(evidence["identity"], run=6, session=3,
                             epoch=3, kind=3, parent=5, final_state=8, group_id=10))
    goal["timeline"][-1]["run_id"] = 6
    with pytest.raises(ValueError, match="cancel Gate key"):
        application_e2e.validate_evidence(evidence)
    invalid = copy.deepcopy(evidence)
    invalid["goals"][1]["groups"].append({**goal["groups"][0], "group_id": 10})
    invalid["goals"][1]["evidence_pages"] = pages(evidence["identity"], run=5,
        session=3, group_ids=[8, 9, 10], group_epochs=[2, 3, 2],
        terminal_epoch=2, final_state=7)
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(invalid)


def test_canceled_report_without_stop_raw_is_not_positive_motion_evidence(evidence):
    goal = evidence["goals"][0]
    goal.update(action_status="CANCELED", terminal="CANCELED_HOLDING",
                stop_confirmed=True, stop_epoch=3)
    goal["groups"][0]["motion_epoch"] = 3
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
                                    group_epochs=[3], terminal_epoch=2, final_state=7)
    goal["timeline"][4]["event"] = "terminal_canceled"
    goal["timeline"][4]["motion_epoch"] = 3
    goal["timeline"][5]["motion_epoch"] = 3
    evidence["pre_release_hold"]["motion_epoch"] = 3
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=3, kind=3, parent=4, final_state=8, group_id=8)
    with pytest.raises(ValueError, match="cancel Gate key"):
        application_e2e.validate_evidence(evidence)


def canceled_chain(evidence):
    goal = evidence["goals"][0]
    goal.update(action_status="CANCELED", terminal="CANCELED_HOLDING",
                stop_confirmed=True, stop_epoch=3,
                cancel_request_id=1, cancel_operation_id=1)
    goal["groups"] = [
        {**goal["groups"][0], "motion_epoch": 2},
        {**goal["groups"][0], "group_id": 8, "motion_epoch": 3},
    ]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8], group_epochs=[2, 3], terminal_epoch=2,
        categories=[1, 2], final_state=7)
    goal["timeline"][4].update(event="terminal_canceled", motion_epoch=3)
    for row in goal["timeline"]:
        row.update(cancel_request_id=0, cancel_operation_id=0)
    goal["timeline"][4].update(cancel_request_id=1, cancel_operation_id=1)
    goal["timeline"][5]["motion_epoch"] = 3
    evidence["pre_release_hold"]["motion_epoch"] = 3
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=3, kind=3, parent=4, final_state=8,
        group_id=9, group_epochs=[4], terminal_epoch=3, categories=[3])
    goal["timeline"][-1]["motion_epoch"] = 4
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=4)
    evidence["management_records"][0] = manage(4, 4800, operation=9,
        request_id=1, expected_motion_epoch=2)
    evidence["management_records"][1] = manage(5, 5000, request_id=1, run=4,
        motion_epoch=3, motion_complete=0)
    return evidence


def test_canceled_chain_requires_new_epoch_stop_and_explicit_release(evidence):
    application_e2e.validate_evidence(canceled_chain(evidence))


def test_canceled_chain_rejects_commit_after_gate_terminal(evidence):
    canceled_chain(evidence)
    evidence["goals"][0]["timeline"][4]["steady_us"] = 4900
    with pytest.raises(ValueError, match="cancel committed receipt follows Gate terminal"):
        application_e2e.validate_evidence(evidence)


def test_canceled_chain_rejects_accepted_epoch_mismatch(evidence):
    canceled_chain(evidence)
    evidence["goals"][0]["timeline"][0]["motion_epoch"] = 1
    with pytest.raises(ValueError, match="accepted motion epoch"):
        application_e2e.validate_evidence(evidence)


def test_canceled_chain_rejects_skipped_stop_epoch_even_with_matching_receipts(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    goal["stop_epoch"] = 4
    goal["groups"][1]["motion_epoch"] = 4
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8], group_epochs=[2, 4], terminal_epoch=2,
        categories=[1, 2], final_state=7)
    goal["timeline"][4]["motion_epoch"] = 4
    goal["timeline"][5]["motion_epoch"] = 4
    goal["timeline"][-1]["motion_epoch"] = 5
    evidence["pre_release_hold"]["motion_epoch"] = 4
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=4, kind=3, parent=4, final_state=8,
        group_id=9, group_epochs=[5], terminal_epoch=4, categories=[3])
    evidence["management_records"][1] = manage(5, 5000, request_id=1,
        run=4, motion_epoch=4, motion_complete=0)
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=5)
    with pytest.raises(ValueError, match="cancel stop epoch must advance once"):
        application_e2e.validate_evidence(evidence)


def test_canceled_chain_rejects_cancel_request_bound_to_old_motion_epoch(evidence):
    canceled_chain(evidence)
    evidence["management_records"][0] = manage(4, 4800, operation=9,
        request_id=1, expected_motion_epoch=1)
    with pytest.raises(ValueError, match="cancel request motion epoch"):
        application_e2e.validate_evidence(evidence)


def test_canceled_goal_allows_monotonic_stop_epoch_feedback_before_action_result(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    goal["timeline"].insert(4, {**goal["timeline"][3],
        "steady_us": 4900, "motion_epoch": 3, "feedback_seq": [2] * 6})
    application_e2e.validate_evidence(evidence)


def test_canceled_goal_rejects_motion_epoch_after_stop_feedback(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    goal["timeline"].insert(4, {**goal["timeline"][3],
        "steady_us": 4700, "motion_epoch": 3, "feedback_seq": [2] * 6})
    goal["timeline"].insert(5, {**goal["timeline"][3],
        "steady_us": 4900, "motion_epoch": 2, "feedback_seq": [3] * 6})
    with pytest.raises(ValueError, match="old motion epoch resumed"):
        application_e2e.validate_evidence(evidence)


def test_canceled_goal_rejects_stop_feedback_before_cancel_request(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    goal["timeline"].insert(4, {**goal["timeline"][3],
        "steady_us": 4700, "motion_epoch": 3, "feedback_seq": [2] * 6})
    with pytest.raises(ValueError, match="stop feedback precedes cancel request"):
        application_e2e.validate_evidence(evidence)


def canceled_chain_with_hold_refresh(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    original = goal["groups"][0]
    goal["groups"] = [{**original, "group_id": group_id, "motion_epoch": epoch}
        for group_id, epoch in ((7, 1), (8, 2), (9, 3), (10, 3))]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8, 9, 10], group_epochs=[1, 2, 3, 3],
        terminal_epoch=1, categories=[1, 1, 2, 1], final_state=7)
    goal["timeline"][0]["motion_epoch"] = 1
    goal["timeline"][1]["motion_epoch"] = 1
    return evidence


def test_canceled_chain_accepts_prebegin_and_post_stop_fixed_hold_refresh(evidence):
    application_e2e.validate_evidence(canceled_chain_with_hold_refresh(evidence))


def test_canceled_chain_still_requires_safety_group_before_hold_refresh(evidence):
    canceled_chain_with_hold_refresh(evidence)
    goal = evidence["goals"][0]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8, 9, 10], group_epochs=[1, 2, 3, 3],
        terminal_epoch=1, categories=[1, 1, 1, 1], final_state=7)
    with pytest.raises(ValueError, match="Hold SENT"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("operation", (8, 10))
def test_committed_receipt_rejects_conflicting_terminal_flags(evidence, operation):
    index = 1 if operation == 8 else -1
    original = evidence["management_records"][index]
    evidence["management_records"][index] = manage(5, original["when"],
        request_id=1 if operation == 8 else 2,
        operation_id=1 if operation == 8 else 2,
        run=4 if operation == 8 else 5,
        session=3 if operation == 8 else 0,
        motion_complete=1, release_committed=1)
    with pytest.raises(ValueError, match="management receipt"):
        application_e2e.validate_evidence(evidence)


@pytest.mark.parametrize("mutation", [
    "pending_cancel", "wrong_cancel_run", "old_cancel_key", "old_cancel_operation",
    "missing_stop_axis",
    "missing_cancel_terminal", "missing_release_receipt", "missing_release_page",
    "release_without_disable", "active_cancel_release", "skipped_release_run",
    "cancel_motion_complete", "cancel_release_committed",
    "old_timeline_cancel_key", "old_timeline_cancel_operation", "missing_timeline_cancel_key",
])
def test_canceled_chain_rejects_incomplete_raw_proof(evidence, mutation):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    if mutation == "pending_cancel":
        evidence["management_records"][1] = manage(5, 5000, request_id=1,
            run=4, phase=2, motion_epoch=3)
    elif mutation == "wrong_cancel_run":
        evidence["management_records"][1] = manage(5, 5000, request_id=1,
            run=6, motion_epoch=3)
    elif mutation == "cancel_motion_complete":
        evidence["management_records"][1] = manage(5, 5000, request_id=1,
            run=4, motion_epoch=3, motion_complete=1)
    elif mutation == "cancel_release_committed":
        evidence["management_records"][1] = manage(5, 5000, request_id=1,
            run=4, motion_epoch=3, motion_complete=0, release_committed=1)
    elif mutation == "old_cancel_key":
        goal["cancel_request_id"] = 99
    elif mutation == "old_cancel_operation":
        goal["cancel_operation_id"] = 99
    elif mutation == "old_timeline_cancel_key":
        goal["timeline"][4]["cancel_request_id"] = 99
    elif mutation == "old_timeline_cancel_operation":
        goal["timeline"][4]["cancel_operation_id"] = 99
    elif mutation == "missing_timeline_cancel_key":
        del goal["timeline"][4]["cancel_request_id"]
    elif mutation == "missing_stop_axis":
        raw = bytearray.fromhex(goal["evidence_pages"][1]["raw_hex"])
        raw[34] = 1
        goal["evidence_pages"][1]["raw_hex"] = raw.hex()
    elif mutation == "missing_cancel_terminal":
        goal["evidence_pages"].pop()
    elif mutation == "missing_release_receipt":
        evidence["management_records"].pop()
    elif mutation == "release_without_disable":
        evidence["release"]["evidence_pages"] = pages(evidence["identity"],
            run=5, session=3, epoch=3, kind=3, parent=4, final_state=8,
            group_id=9, categories=[1])
    elif mutation == "active_cancel_release":
        evidence["release"].update(terminal="CANCELED_AND_RELEASED",
            evidence_pages=pages(evidence["identity"], run=5, session=3,
                epoch=3, kind=1, parent=0, final_state=2, group_ids=[9, 10],
                categories=[1, 3]))
    elif mutation == "skipped_release_run":
        evidence["release"]["run_id"] = 6
        goal["timeline"][-1]["run_id"] = 6
        evidence["release"]["evidence_pages"] = pages(evidence["identity"],
            run=6, session=3, epoch=3, kind=3, parent=4, final_state=8,
            group_id=9, categories=[3])
        evidence["management_records"][-1] = manage(5, 6700, request_id=2,
            run=6, session=0, operation_id=2)
    else:
        evidence["release"]["evidence_pages"].pop()
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_environment_lag_cannot_be_called_device_fault(evidence):
    evidence["environment"].update(producer_lag_us=20000)
    evidence["goals"][0].update(action_status="ABORTED", result_error_code=-4,
                                terminal="PRODUCER_LAG", fault_domain="DEVICE")
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_environment_only_fault_cannot_be_reported_as_device_success(evidence):
    evidence["environment"]["producer_lag_us"] = 20000
    evidence["goals"][0].update(action_status="ABORTED", result_error_code=-4,
                                terminal="PRODUCER_LAG", fault_domain="ENVIRONMENT")
    with pytest.raises(ValueError, match="environment-only"):
        application_e2e.validate_evidence(evidence)


def test_environment_abort_without_release_timeline_fails_as_validation_error(evidence):
    evidence["environment"]["producer_lag_us"] = 20000
    goal = evidence["goals"][0]
    goal.update(action_status="ABORTED", result_error_code=-4,
                terminal="PRODUCER_LAG", fault_domain="ENVIRONMENT")
    del goal["timeline"]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)




@pytest.mark.parametrize("field,value", [("environment", []), ("goals", [None]),
                                         ("goals", [{"goal_id": "a" * 32, "groups": [None]}])])
def test_malformed_evidence_fails_as_validation_error(evidence, field, value):
    evidence[field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)
