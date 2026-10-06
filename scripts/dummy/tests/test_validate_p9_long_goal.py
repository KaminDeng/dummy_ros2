"""P9 long-goal qualification stays tied to strict raw ROS evidence."""

import copy
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("validate_p9_long_goal", ROOT / "validate_p9_long_goal.py")
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)

POLICY = "aa" * 32
FIRMWARE = "bb" * 32


def joined():
    feedback_rows = [{"event": "feedback", "steady_us": 1000 + second * 1_000_000}
                     for second in range(1, 600)]
    return {
        "source": "ros_moveit_jtc_usb_ref_can", "profile_id": 201,
        "configured_mask": 63, "identity": {"firmware_hash": FIRMWARE},
        "provenance": {"raw_sha256": "cc" * 32,
                       "timeline_sha256": "dd" * 32,
                       "action_sha256": "ee" * 32,
                       "context_sha256": "ff" * 32},
        "goals": [{"action_status": "SUCCEEDED", "terminal": "MOTION_COMPLETED_HOLDING",
                   "hold_confirmed": True, "run_id": 4,
                   "path_tolerance_rad": [0.003] * 6,
                   "groups": [{"axis_mask": 63, "sent_mask": 63, "committed": True}],
                   "timeline": [{"event": "accepted", "steady_us": 1000},
                                {"event": "before_begin", "steady_us": 1100},
                                {"event": "after_begin", "steady_us": 1200},
                                *feedback_rows,
                                {"event": "terminal_success", "steady_us": 600_001_200},
                                {"event": "pre_release_hold", "steady_us": 600_002_000},
                                {"event": "post_release_status", "steady_us": 600_003_000}]}],
        "pre_release_hold": {"confirmed": True, "run_id": 4},
        "release": {"terminal": "RELEASED", "parent_run_id": 4,
                    "evidence_pages": [{"raw_hex": "00"}, {"raw_hex": "11"}]},
    }


def action():
    return {"scope": "ROS_ACTION_ONLY", "status": "SUCCEEDED",
            "policy_sha256": POLICY, "duration_s": 600,
            "planned_points": 301, "total_travel_rad": 3.6,
            "action_started_steady_us": 900,
            "action_accepted_steady_us": 1000, "action_elapsed_us": 600_000_000,
            "feedback": [{"elapsed_us": second * 1_000_000 + 100,
                          "desired_rad": [0.01 * min(second % 20, 20 - second % 20) / 10] * 6,
                          "actual_rad": [0.01 * min(second % 20, 20 - second % 20) / 10] * 6}
                         for second in range(1, 600)]}


def witness():
    return {"start_us": 1200, "end_us": 600_001_200,
            "max_gap_us": 20_000,
            "window_ranges_rad": [[0.01] * 6 for _ in range(30)]}


def check(evidence=None, report=None):
    return validator.validate_long_goal(evidence or joined(), report or action(),
                                        policy_sha256=POLICY, firmware_sha256=FIRMWARE,
                                        raw_terminal_us=600_001_200,
                                        raw_motion=witness())


def test_accepts_exact_600_second_gate_span_only_as_unqualified_evidence():
    result = check()
    assert result["status"] == "ROS_SINGLE_GOAL_600S_MOTION_EVIDENCE_POSITIVE"
    assert result["evidence_scope"] == "GENERIC_SIX_AXIS_LONG_MOTION"
    assert result["trajectory_shape"] == "NOT_VERIFIED"
    assert result["qualification_scope"] == "NOT_QUALIFIED"
    assert result["accepted_to_terminal_us"] == 600_000_200
    assert result["active_to_terminal_us"] == 600_000_000
    assert result["active_to_raw_terminal_us"] == 600_000_000


@pytest.mark.parametrize("change", [
    lambda e, a: a.update(duration_s=9999, action_elapsed_us=999999999),
    lambda e, a: e["goals"][0]["timeline"][-3].update(steady_us=600_000_999),
])
def test_action_claim_cannot_hide_short_gate_span(change):
    evidence, report = joined(), action()
    evidence["goals"][0]["timeline"][-3]["steady_us"] = 600_000_999
    change(evidence, report)
    with pytest.raises(ValueError, match="600000000"):
        check(evidence, report)


@pytest.mark.parametrize("field", ["raw_sha256", "timeline_sha256", "action_sha256",
                                     "context_sha256"])
def test_missing_capture_digest_fails_closed(field):
    evidence = joined()
    del evidence["provenance"][field]
    with pytest.raises(ValueError, match=field):
        check(evidence)


@pytest.mark.parametrize("mutation", [
    lambda e, a: e["identity"].update(firmware_hash="12" * 32),
    lambda e, a: a.update(policy_sha256="12" * 32),
    lambda e, a: a.update(scope="ROS_ACTION_SESSION_ONLY"),
    lambda e, a: e["goals"].append(copy.deepcopy(e["goals"][0])),
    lambda e, a: e["goals"][0].update(hold_confirmed=False),
    lambda e, a: e["release"].update(terminal="UNKNOWN"),
    lambda e, a: e["goals"][0].update(groups=[]),
])
def test_rejects_identity_scope_or_missing_chain(mutation):
    evidence, report = joined(), action()
    mutation(evidence, report)
    with pytest.raises(ValueError):
        check(evidence, report)


def test_raw_terminal_duration_is_required():
    with pytest.raises(ValueError, match="raw motion terminal"):
        validator.validate_long_goal(joined(), action(), policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE, raw_motion=witness())
    with pytest.raises(ValueError, match="raw motion terminal"):
        validator.validate_long_goal(joined(), action(), policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE,
                                     raw_terminal_us=600_000_999, raw_motion=witness())


def test_confirmed_active_release_terminal_is_supported():
    evidence = joined()
    evidence["release"]["terminal"] = "CANCELED_AND_RELEASED"
    assert check(evidence=evidence)["qualification_scope"] == "NOT_QUALIFIED"
    report = action()
    report["action_accepted_steady_us"] = 1
    with pytest.raises(ValueError, match="raw motion terminal"):
        validator.validate_long_goal(joined(), report, policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE,
                                     raw_terminal_us=600_000_999, raw_motion=witness())


@pytest.mark.parametrize("mutation", [
    lambda e, a, w: a.update(duration_s=10),
    lambda e, a, w: a.update(planned_points=2),
    lambda e, a, w: a.update(action_elapsed_us=10_000_000),
    lambda e, a, w: a["feedback"].__setitem__(slice(10, -10), []),
    lambda e, a, w: [sample.update(desired_rad=[0.0] * 6,
                                   actual_rad=[0.0] * 6) for sample in a["feedback"]],
    lambda e, a, w: [sample.update(actual_rad=[0.0] * 6) for sample in a["feedback"]],
    lambda e, a, w: a["feedback"][200].update(actual_rad=[0.1] * 6),
    lambda e, a, w: w["window_ranges_rad"][15].__setitem__(2, 0.0),
    lambda e, a, w: w.update(max_gap_us=10_000_000),
    lambda e, a, w: e["goals"][0]["timeline"].__setitem__(slice(10, -10), []),
])
def test_rejects_delayed_completion_or_static_and_sparse_motion(mutation):
    evidence, report, raw_motion = joined(), action(), witness()
    mutation(evidence, report, raw_motion)
    with pytest.raises(ValueError):
        validator.validate_long_goal(evidence, report, policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE,
                                     raw_terminal_us=600_001_200,
                                     raw_motion=raw_motion)


def test_short_active_span_fails_even_when_acceptance_span_is_long():
    evidence = joined()
    evidence["goals"][0]["timeline"][2]["steady_us"] = 2000
    with pytest.raises(ValueError, match="active motion"):
        check(evidence=evidence)


def test_one_pulse_per_window_is_not_continuous_motion():
    evidence, report, raw_motion = joined(), action(), witness()
    for second, sample in enumerate(report["feedback"], 1):
        pulse = 0.01 if second % 20 == 10 else 0.0
        sample.update(desired_rad=[pulse] * 6, actual_rad=[pulse] * 6)
    raw_motion["window_ranges_rad"] = [[0.01] * 6 for _ in range(30)]
    with pytest.raises(ValueError, match="sparse or static"):
        validator.validate_long_goal(evidence, report, policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE,
                                     raw_terminal_us=600_001_200,
                                     raw_motion=raw_motion)


def test_quantized_actual_motion_is_accepted():
    report = action()
    for sample in report["feedback"]:
        sample["actual_rad"] = [round(value, 3) for value in sample["desired_rad"]]
    assert check(report=report)["qualification_scope"] == "NOT_QUALIFIED"


def test_structured_raw_state_witness_covers_all_six_axes_for_600_seconds():
    records = [{"kind": 2, "boot": 2, "session": 3, "when": 1100,
                "fields": {"position_valid_mask": 63, "configured_mask": 63,
                           "profile_id": 201, "motion_epoch": 1,
                           "position_rad": [0.0] * 6}}]
    for tick in range(30_001):
        when = 1200 + tick * 20_000
        phase = (tick % 1000) / 1000
        position = 0.01 * (1 - abs(2 * phase - 1))
        records.append({"kind": 2, "boot": 2, "session": 3, "when": when,
                        "fields": {"position_valid_mask": 63, "configured_mask": 63,
                                   "profile_id": 201, "motion_epoch": 2,
                                   "position_rad": [position] * 6}})
    goal = {"boot_epoch": 2, "session_id": 3, "motion_epoch": 2}
    observed = validator._raw_motion(records, goal, 1200)
    assert observed["max_gap_us"] == 20_000
    assert len(observed["window_ranges_rad"]) == 30
    assert all(min(ranges) > 0.004 for ranges in observed["window_ranges_rad"])
    for item in records:
        if 1200 + 15 * 20_000_000 <= item["when"] < 1200 + 16 * 20_000_000:
            item["fields"] = {**item["fields"], "position_rad": [0.0] * 6}
    observed = validator._raw_motion(records, goal, 1200)
    assert observed["window_ranges_rad"][15] == [0.0] * 6
    records[100]["fields"] = {**records[100]["fields"], "motion_epoch": 1}
    with pytest.raises(ValueError, match="unlinked six-axis raw motion"):
        validator._raw_motion(records, goal, 1200)


def test_raw_one_sample_pulses_fail_robust_window_motion():
    records = []
    for tick in range(30_001):
        position = 0.01 if tick % 1000 == 500 else 0.0
        records.append({"kind": 2, "boot": 2, "session": 3,
                        "when": 1200 + tick * 20_000,
                        "fields": {"position_valid_mask": 63, "configured_mask": 63,
                                   "profile_id": 201, "motion_epoch": 2,
                                   "position_rad": [position] * 6}})
    goal = {"boot_epoch": 2, "session_id": 3, "motion_epoch": 2}
    observed = validator._raw_motion(records, goal, 1200)
    assert observed["window_ranges_rad"] == [[0.0] * 6 for _ in range(30)]


def test_cli_paths_replay_strict_join_and_reject_short_real_fixture(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "existing_join_tests", ROOT / "tests/test_join_real_6dof_evidence.py")
    fixtures = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixtures)
    timeline, raw, action_path, context, _ = fixtures.inputs.__wrapped__(tmp_path)
    report = json.loads(action_path.read_text())
    report.update(policy_sha256=POLICY, action_accepted_steady_us=1000)
    action_path.write_text(json.dumps(report))
    fixtures.repin(timeline, raw, action_path, context)
    evidence = fixtures.joiner.join_evidence(timeline, raw, action_path, context)
    joined_path = tmp_path / "joined.json"
    joined_path.write_text(json.dumps(evidence))
    with pytest.raises(ValueError, match="motion observations"):
        validator.verify_paths(joined=joined_path, timeline=timeline, raw=raw,
                               action=action_path, context=context,
                               policy_sha256=POLICY, firmware_sha256="11" * 32)
    evidence["provenance"]["raw_sha256"] = "00" * 32
    joined_path.write_text(json.dumps(evidence))
    with pytest.raises(ValueError, match="strict SHA-pinned raw replay"):
        validator.verify_paths(joined=joined_path, timeline=timeline, raw=raw,
                               action=action_path, context=context,
                               policy_sha256=POLICY, firmware_sha256="11" * 32)
