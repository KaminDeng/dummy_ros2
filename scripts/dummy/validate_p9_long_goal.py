#!/usr/bin/env python3
"""Offline P9 single-goal duration check over strict joined ROS/raw evidence."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import re


MIN_DURATION_US = 600_000_000
WINDOW_US = 20_000_000
MAX_RAW_GAP_US = 250_000
MAX_FEEDBACK_GAP_US = 2_000_000
MIN_AXIS_RANGE_RAD = 0.002
MIN_ACTUAL_RANGE_RAD = 0.001
SHA256 = re.compile(r"[0-9a-f]{64}\Z")


def _joiner():
    path = Path(__file__).with_name("join_real_6dof_evidence.py")
    spec = importlib.util.spec_from_file_location("join_real_6dof_evidence", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _hash(value, name):
    if not isinstance(value, str) or not SHA256.fullmatch(value):
        raise ValueError(f"missing or invalid {name}")
    return value


def _six_positions(sample, name):
    values = sample.get(name) if isinstance(sample, dict) else None
    if (not isinstance(values, list) or len(values) != 6 or
            any(type(value) not in (int, float) or not math.isfinite(value)
                for value in values)):
        raise ValueError(f"invalid six-axis {name}")
    return values


def _window_ranges(samples, start_us):
    ranges = []
    for index in range(MIN_DURATION_US // WINDOW_US):
        start = start_us + index * WINDOW_US
        end = start + WINDOW_US
        positions = [values for when, values in samples if start <= when < end]
        if len(positions) < 10:
            raise ValueError("missing motion observations in 20s window")
        axis_ranges = []
        for axis in range(6):
            ordered = sorted(row[axis] for row in positions)
            quartile = len(ordered) // 4
            axis_ranges.append(ordered[-quartile - 1] - ordered[quartile])
        ranges.append(axis_ranges)
    return ranges


def _raw_motion(records, goal, active_start):
    deadline = active_start + MIN_DURATION_US
    samples = []
    for item in records:
        if (item["kind"] != 2 or item["boot"] != goal["boot_epoch"] or
                item["session"] != goal["session_id"] or
                not active_start <= item["when"] <= deadline):
            continue
        fields = item["fields"]
        if (fields["position_valid_mask"] != 63 or fields["configured_mask"] != 63 or
                fields["profile_id"] != 201 or
                fields["motion_epoch"] != goal["motion_epoch"]):
            raise ValueError("unlinked six-axis raw motion observation")
        samples.append((item["when"], _six_positions(fields, "position_rad")))
    if not samples:
        raise ValueError("missing raw motion observations")
    times = [active_start, *(when for when, _ in samples), deadline]
    return {"start_us": active_start, "end_us": deadline,
            "max_gap_us": max(right - left for left, right in zip(times, times[1:])),
            "window_ranges_rad": _window_ranges(samples, active_start)}


def validate_long_goal(evidence, action, *, policy_sha256, firmware_sha256,
                       raw_terminal_us=None, raw_motion=None):
    """Check P9-specific facts after the strict joiner has validated the capture."""
    _hash(policy_sha256, "expected policy SHA-256")
    _hash(firmware_sha256, "expected firmware SHA-256")
    if not isinstance(evidence, dict) or not isinstance(action, dict):
        raise ValueError("structured joined evidence and action report required")
    if (evidence.get("source") != "ros_moveit_jtc_usb_ref_can" or
            evidence.get("profile_id") != 201 or evidence.get("configured_mask") != 63 or
            action.get("scope") != "ROS_ACTION_ONLY" or
            action.get("status") != "SUCCEEDED"):
        raise ValueError("unsupported P9 single six-axis ROS action scope")
    provenance = evidence.get("provenance")
    if not isinstance(provenance, dict):
        raise ValueError("missing capture provenance")
    for name in ("raw_sha256", "timeline_sha256", "action_sha256", "context_sha256"):
        _hash(provenance.get(name), name)
    identity = evidence.get("identity")
    if not isinstance(identity, dict) or _hash(identity.get("firmware_hash"), "firmware_hash") != firmware_sha256:
        raise ValueError("firmware identity mismatch")
    if _hash(action.get("policy_sha256"), "policy_sha256") != policy_sha256:
        raise ValueError("long-goal policy identity mismatch")
    goals = evidence.get("goals")
    if not isinstance(goals, list) or len(goals) != 1:
        raise ValueError("exactly one joined ROS goal required")
    goal = goals[0]
    rows = goal.get("timeline") if isinstance(goal, dict) else None
    if (goal.get("action_status") != "SUCCEEDED" or
            goal.get("terminal") != "MOTION_COMPLETED_HOLDING" or
            goal.get("hold_confirmed") is not True or
            not isinstance(rows, list) or len(rows) < 7 or
            [row.get("event") for row in rows[:3]] !=
            ["accepted", "before_begin", "after_begin"] or
            [row.get("event") for row in rows[-3:]] !=
            ["terminal_success", "pre_release_hold", "post_release_status"]):
        raise ValueError("missing joined motion terminal or Hold chain")
    groups = goal.get("groups")
    if (not isinstance(groups, list) or not groups or
            any(group.get("axis_mask") != 63 or group.get("sent_mask") != 63 or
                group.get("committed") is not True for group in groups)):
        raise ValueError("missing committed six-axis motion evidence")
    hold, release = evidence.get("pre_release_hold"), evidence.get("release")
    if (not isinstance(hold, dict) or hold.get("confirmed") is not True or
            hold.get("run_id") != goal.get("run_id") or
            not isinstance(release, dict) or
            release.get("terminal") not in ("RELEASED", "CANCELED_AND_RELEASED") or
            release.get("parent_run_id") != goal.get("run_id") or
            not isinstance(release.get("evidence_pages"), list) or
            len(release["evidence_pages"]) < 2):
        raise ValueError("missing confirmed Hold/Release evidence chain")
    accepted = rows[0].get("steady_us")
    active_start = rows[2].get("steady_us")
    terminal = rows[-3].get("steady_us")
    action_accepted = action.get("action_accepted_steady_us")
    if (any(type(value) is not int or value <= 0 for value in
            (accepted, active_start, terminal, action_accepted)) or
            action_accepted > accepted or active_start < accepted or terminal < active_start):
        raise ValueError("invalid action/Gate accepted-to-terminal clock binding")
    duration = terminal - accepted
    if duration < MIN_DURATION_US:
        raise ValueError("Gate accepted-to-terminal below 600000000 us")
    if terminal - active_start < MIN_DURATION_US:
        raise ValueError("Gate active motion below 600000000 us from after_begin")
    if (type(raw_terminal_us) is not int or raw_terminal_us > terminal or
            raw_terminal_us - active_start < MIN_DURATION_US):
        raise ValueError("raw motion terminal below 600000000 us from after_begin")
    if (action.get("duration_s") != 600 or type(action.get("planned_points")) is not int or
            action["planned_points"] != 301 or
            type(action.get("action_elapsed_us")) is not int or
            action["action_elapsed_us"] < MIN_DURATION_US or
            type(action.get("total_travel_rad")) not in (int, float) or
            not math.isfinite(action["total_travel_rad"]) or
            action["total_travel_rad"] < 3.0):
        raise ValueError("unbound P9 600s/301-point six-axis action shape")
    feedback = action.get("feedback")
    started = action.get("action_started_steady_us")
    if not isinstance(feedback, list) or type(started) is not int or started <= 0:
        raise ValueError("missing long-goal ROS feedback")
    tolerance = _six_positions(goal, "path_tolerance_rad")
    if any(value <= 0 for value in tolerance):
        raise ValueError("invalid bounded six-axis path tolerance")
    tolerance = [min(value, 0.005) for value in tolerance]
    ros_samples = []
    actual_samples = []
    for item in feedback:
        elapsed = item.get("elapsed_us") if isinstance(item, dict) else None
        if type(elapsed) is not int or elapsed < 0:
            raise ValueError("invalid ROS feedback clock")
        when = started + elapsed
        if active_start <= when <= terminal:
            actual = _six_positions(item, "actual_rad")
            desired = _six_positions(item, "desired_rad")
            if any(abs(a - d) > limit for a, d, limit in
                   zip(actual, desired, tolerance)):
                raise ValueError("ROS actual feedback diverges from commanded path")
            ros_samples.append((when, desired))
            actual_samples.append((when, actual))
    ros_times = [active_start, *(when for when, _ in ros_samples), terminal]
    if (len(ros_samples) < 300 or
            any(right <= left or right - left > MAX_FEEDBACK_GAP_US
                for left, right in zip(ros_times, ros_times[1:])) or
            any(axis_range < MIN_AXIS_RANGE_RAD for ranges in
                _window_ranges(ros_samples, active_start) for axis_range in ranges) or
            any(axis_range < MIN_ACTUAL_RANGE_RAD for ranges in
                _window_ranges(actual_samples, active_start) for axis_range in ranges)):
        raise ValueError("sparse or static six-axis ROS feedback over 600s")
    gate_times = [row.get("steady_us") for row in rows if row.get("event") == "feedback" and
                  row.get("steady_us", -1) >= active_start]
    if (any(type(when) is not int for when in gate_times) or
            any(right <= left or right - left > MAX_FEEDBACK_GAP_US
                for left, right in zip([active_start, *gate_times, terminal],
                                       [*gate_times, terminal]))):
        raise ValueError("sparse Gate feedback over 600s")
    if (not isinstance(raw_motion, dict) or raw_motion.get("start_us") != active_start or
            raw_motion.get("end_us") != active_start + MIN_DURATION_US or
            type(raw_motion.get("max_gap_us")) is not int or
            raw_motion["max_gap_us"] > MAX_RAW_GAP_US or
            not isinstance(raw_motion.get("window_ranges_rad"), list) or
            len(raw_motion["window_ranges_rad"]) != MIN_DURATION_US // WINDOW_US or
            any(not isinstance(ranges, list) or len(ranges) != 6 or
                any(type(value) not in (int, float) or not math.isfinite(value) or
                    value < MIN_AXIS_RANGE_RAD for value in ranges)
                for ranges in raw_motion["window_ranges_rad"])):
        raise ValueError("missing continuous six-axis raw motion over 600s")
    return {"status": "ROS_SINGLE_GOAL_600S_MOTION_EVIDENCE_POSITIVE",
            "evidence_scope": "GENERIC_SIX_AXIS_LONG_MOTION",
            "trajectory_shape": "NOT_VERIFIED",
            "qualification_scope": "NOT_QUALIFIED", "hardware_qualification": "NOT_ASSERTED",
            "accepted_to_terminal_us": duration,
            "active_to_terminal_us": terminal - active_start,
            "active_to_raw_terminal_us": raw_terminal_us - active_start,
            "raw_terminal_us": raw_terminal_us,
            "policy_sha256": policy_sha256, "firmware_sha256": firmware_sha256,
            "raw_sha256": provenance["raw_sha256"]}


def verify_paths(*, joined, timeline, raw, action, context, policy_sha256,
                 firmware_sha256):
    joiner = _joiner()
    joined_bytes, _ = joiner._snapshot(Path(joined))
    evidence = json.loads(joined_bytes)
    rebuilt = joiner.join_evidence(timeline, raw, action, context)
    if evidence != rebuilt:
        raise ValueError("joined evidence differs from strict SHA-pinned raw replay")
    action_bytes, action_hash = joiner._snapshot(Path(action))
    if action_hash != evidence["provenance"]["action_sha256"]:
        raise ValueError("action SHA-256 mismatch")
    raw_bytes, raw_hash = joiner._snapshot(Path(raw))
    if raw_hash != evidence["provenance"]["raw_sha256"]:
        raise ValueError("raw SHA-256 mismatch")
    records = joiner._raw_records(raw_bytes)
    goal = evidence["goals"][0]
    terminals = [item["when"] for item in records if item["kind"] == 1 and
                 item["run"] == goal["run_id"] and
                 item["fields"]["category"] == 4 and
                 item["fields"]["final_state"] == 6]
    if len(terminals) != 1:
        raise ValueError("missing unique raw motion terminal")
    raw_motion = _raw_motion(records, goal, goal["timeline"][2]["steady_us"])
    return validate_long_goal(evidence, json.loads(action_bytes),
                              policy_sha256=policy_sha256,
                              firmware_sha256=firmware_sha256,
                              raw_terminal_us=terminals[0], raw_motion=raw_motion)


def main():
    parser = argparse.ArgumentParser(description="P9 单次六轴 ROS 600s 原始证据门禁")
    for name in ("joined", "timeline", "raw", "action", "context", "report"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--policy-sha256", required=True)
    parser.add_argument("--firmware-sha256", required=True)
    args = parser.parse_args()
    try:
        result = verify_paths(joined=args.joined, timeline=args.timeline, raw=args.raw,
                              action=args.action, context=args.context,
                              policy_sha256=args.policy_sha256,
                              firmware_sha256=args.firmware_sha256)
        code = 0
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
        result = {"status": "NOT_QUALIFIED", "qualification_scope": "NOT_QUALIFIED",
                  "hardware_qualification": "NOT_ASSERTED", "reason": str(error)}
        code = 1
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
