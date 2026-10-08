#!/usr/bin/env python3
"""从同一次 ROS 原始采集生成 join_real_6dof_evidence 所需的上下文。"""

import argparse
import csv
import io
import json
import math
import os
from pathlib import Path
import tempfile

try:
    from . import join_real_6dof_evidence as joiner
except ImportError:
    import join_real_6dof_evidence as joiner


PRODUCER_LAG_METHOD = "state_receive_minus_produced_offset_span"
WINDOW_LAG_METHOD = PRODUCER_LAG_METHOD + "_motion_window"


def _motion_window(timeline_data, records, client, state_max_age):
    try:
        reader = csv.DictReader(io.StringIO(timeline_data.decode("utf-8")))
        rows = [row for row in reader if row.get("event") != "COMPLETE"]
        goals = [row for row in rows if row.get("event") == "accepted"]
        if not goals:
            raise ValueError("missing ROS accepted goal")
        final = rows[-1]
        if final.get("event") != "post_release_status":
            raise ValueError("missing final ROS Release observation")
        start = int(goals[0]["steady_us"])
        end = int(final["steady_us"])
        boot, session = int(goals[0]["boot"]), int(goals[0]["session"])
        motion_runs = [int(row["run_id"]) for row in goals]
        release_run = int(final["run_id"])
        request_id = int(final["release_request_id"])
        operation_id = int(final["release_operation_id"])
    except (UnicodeDecodeError, TypeError, KeyError, IndexError, ValueError) as error:
        if isinstance(error, ValueError) and ("accepted" in str(error) or
                                              "Release" in str(error)):
            raise
        raise ValueError("invalid ROS motion window timeline") from error
    if (start <= 0 or end <= start or boot <= 0 or session <= 0 or
            release_run <= max(motion_runs) or request_id <= 0 or operation_id <= 0 or
            any(int(row["boot"]) != boot or int(row["session"]) != session or
                int(row["run_id"]) <= 0 for row in goals)):
        raise ValueError("ROS motion window identity or ordering mismatch")
    requests = [item for item in records if item["kind"] == 4 and
                item["fields"]["operation"] == 10 and
                item["fields"]["request_id"] == request_id and
                item["fields"]["client_instance_id"] == client and
                (item["boot"], item["session"]) == (boot, session)]
    terminals = [item for item in records if item["kind"] == 1 and
                 item["fields"]["category"] == 4 and
                 item["fields"]["release_committed"] == 1 and
                 item["fields"]["final_state"] in (2, 8) and
                 (item["boot"], item["session"], item["run"]) ==
                 (boot, session, release_run)]
    queries = [item for item in records if item["kind"] == 4 and
               item["fields"]["operation"] == 12 and
               item["fields"]["queried_operation_id"] == request_id and
               item["fields"]["client_instance_id"] == client and
               (item["boot"], item["session"]) == (boot, session) and
               requests and item["when"] >= requests[0]["when"]]
    responses = [item for item in records if item["kind"] == 5 and
                 (item["fields"]["request_id"] == request_id or
                  any(query["fields"]["request_id"] == item["fields"]["request_id"] and
                      query["when"] <= item["when"] for query in queries)) and
                 item["fields"]["operation_id"] == operation_id and
                 item["fields"]["client_instance_id"] == client and
                 item["fields"]["transaction_phase"] == 3 and
                 item["fields"]["result_code"] == 0 and
                 item["fields"]["release_committed"] == 1 and
                 (item["boot"], item["session"], item["run"]) ==
                 (boot, 0, release_run)]
    if (len(requests) != 1 or len(terminals) != 1 or len(responses) != 1 or
            not start < requests[0]["when"] <= terminals[0]["when"] <=
            responses[0]["when"] <= end):
        raise ValueError("missing or unlinked raw Release proof")
    idle_statuses = [item for item in records if item["kind"] == 3 and
                     (item["boot"], item["session"], item["fields"]["run_id"]) ==
                     (boot, 0, release_run) and
                     item["fields"]["core_state"] == 1 and
                     item["fields"]["release_committed"] == 1 and
                     terminals[0]["when"] <= item["when"] <= end and
                     end - item["when"] <= state_max_age]
    idle_states = [item for item in records if item["kind"] == 2 and
                   (item["boot"], item["session"]) == (boot, 0) and
                   item["fields"]["core_state"] == 1 and
                   terminals[0]["when"] <= item["when"] <= end and
                   end - item["when"] <= state_max_age]
    if not idle_statuses or not idle_states:
        raise ValueError("missing fresh raw Release Idle observation")
    window_states = [item for item in records if item["kind"] == 2 and
                     start <= item["when"] <= end]
    if len(window_states) < 2:
        raise ValueError("insufficient motion window State samples")
    observations = [start, *(item["when"] for item in window_states), end]
    if any(later - earlier > state_max_age for earlier, later in
           zip(observations, observations[1:])):
        raise ValueError("insufficient motion window State coverage")
    if any(item["boot"] != boot or item["session"] not in (session, 0) or
           (item["session"] == 0 and
            (item["when"] < requests[0]["when"] or
             item["fields"]["core_state"] != 1)) or
           (item["session"] == session and item["when"] > terminals[0]["when"])
           for item in window_states):
        raise ValueError("motion window State identity mismatch")
    return start, end, window_states, release_run


def _six_positive(values, name):
    if (not isinstance(values, (list, tuple)) or len(values) != 6 or
            any(type(value) not in (int, float) or not math.isfinite(value) or
                value <= 0 for value in values)):
        raise ValueError(f"invalid {name}: six positive finite values required")
    return list(values)


def _positive_int(value, name):
    if type(value) is not int or value <= 0:
        raise ValueError(f"invalid {name}: positive integer required")
    return value


def build_context(timeline_path, raw_path, action_path, output_path, *,
                  path_tolerance_rad, goal_tolerance_rad,
                  producer_lag_budget_us, feedback_age_budget_us,
                  ros_feedback_lag_budget_us, gate_state_max_age_us,
                  producer_lag_scope="full_stream_v1"):
    """校验采集和显式限值，再独占、原子地写入 context.json。"""
    paths = [Path(path) for path in (timeline_path, raw_path, action_path)]
    output = Path(output_path)
    if output.resolve() in {path.resolve() for path in paths}:
        raise ValueError("output must differ from every capture input")
    path_tolerance_rad = _six_positive(path_tolerance_rad, "path tolerance")
    goal_tolerance_rad = _six_positive(goal_tolerance_rad, "goal tolerance")
    producer_lag_budget_us = _positive_int(producer_lag_budget_us, "producer lag budget")
    feedback_age_budget_us = _positive_int(feedback_age_budget_us, "feedback age budget")
    ros_feedback_lag_budget_us = _positive_int(ros_feedback_lag_budget_us,
                                               "ROS feedback lag budget")
    gate_state_max_age_us = _positive_int(gate_state_max_age_us,
                                          "Gate state maximum age")
    if gate_state_max_age_us > joiner.STATUS_MAX_AGE_US:
        raise ValueError("Gate state maximum age exceeds joiner limit")
    if producer_lag_scope not in ("full_stream_v1", "motion_window_v2"):
        raise ValueError("invalid producer lag scope")
    if producer_lag_scope == "motion_window_v2" and producer_lag_budget_us != 250000:
        raise ValueError("motion window producer lag budget must be 250000us")

    snapshots = [joiner._snapshot(path) for path in paths]
    raw_records = joiner._raw_records(snapshots[1][0])
    states = [item for item in raw_records if item["kind"] == 2]
    if not states:
        raise ValueError("empty DPRAW01 State stream")
    management = [item for item in raw_records if item["kind"] in (4, 5)]
    requests = [item for item in management if item["kind"] == 4]
    if not requests:
        raise ValueError("missing DPRAW01 management request client identity")
    clients = {item["fields"]["client_instance_id"] for item in management}
    if len(clients) != 1 or "00" * 16 in clients:
        raise ValueError("mismatched or zero DPRAW01 management client identity")
    offsets = []
    previous_produced = None
    for item in states:
        produced = item["fields"]["produced_at_us"]
        if produced <= 0 or (previous_produced is not None and
                             produced < previous_produced):
            raise ValueError("nonmonotonic DPRAW01 State produced time")
        previous_produced = produced
        offsets.append(item["when"] - produced)
    environment_lag = max(offsets) - min(offsets)
    try:
        action = json.loads(snapshots[2][0])
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("invalid action JSON") from error
    if not isinstance(action, dict):
        raise ValueError("invalid action JSON object")

    window = None
    producer_lag = environment_lag
    if producer_lag_scope == "motion_window_v2":
        start, end, window_states, release_run = _motion_window(
            snapshots[0][0], raw_records, next(iter(clients)), gate_state_max_age_us)
        window_offsets = [item["when"] - item["fields"]["produced_at_us"]
                          for item in window_states]
        producer_lag = max(window_offsets) - min(window_offsets)
        window = {"start_steady_us": start, "end_steady_us": end,
                  "state_samples": len(window_states), "release_run_id": release_run}
    if producer_lag > producer_lag_budget_us:
        raise ValueError("observed State producer lag exceeds producer lag budget")

    context = {
        "timeline_sha256": snapshots[0][1], "raw_sha256": snapshots[1][1],
        "action_sha256": snapshots[2][1], "client_id": next(iter(clients)),
        "path_tolerance_rad": path_tolerance_rad,
        "goal_tolerance_rad": goal_tolerance_rad,
        "producer_lag_us": producer_lag,
        "producer_lag_method": (WINDOW_LAG_METHOD if window is not None else
                                PRODUCER_LAG_METHOD),
        "producer_lag_budget_us": producer_lag_budget_us,
        "feedback_age_budget_us": feedback_age_budget_us,
        "ros_feedback_lag_budget_us": ros_feedback_lag_budget_us,
        "gate_state_max_age_us": gate_state_max_age_us,
    }
    if window is not None:
        context = {**context, "producer_lag_gate_version": 2,
                   "producer_lag_environment_us": environment_lag,
                   "producer_lag_window": window}
    encoded = (json.dumps(context, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode()
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=output.parent, prefix=f".{output.name}.",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return context


def _six_argument(value):
    try:
        return [float(part) for part in value.split(",")]
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected six comma-separated numbers") from error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("timeline", "raw", "action", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    for name in ("path-tolerance-rad", "goal-tolerance-rad"):
        parser.add_argument(f"--{name}", type=_six_argument, required=True)
    for name in ("producer-lag-budget-us", "feedback-age-budget-us",
                 "ros-feedback-lag-budget-us", "gate-state-max-age-us"):
        parser.add_argument(f"--{name}", type=int, required=True)
    parser.add_argument("--producer-lag-scope", default="full_stream_v1",
                        choices=("full_stream_v1", "motion_window_v2"))
    args = parser.parse_args(argv)
    try:
        result = build_context(
            args.timeline, args.raw, args.action, args.output,
            path_tolerance_rad=args.path_tolerance_rad,
            goal_tolerance_rad=args.goal_tolerance_rad,
            producer_lag_budget_us=args.producer_lag_budget_us,
            feedback_age_budget_us=args.feedback_age_budget_us,
            ros_feedback_lag_budget_us=args.ros_feedback_lag_budget_us,
            gate_state_max_age_us=args.gate_state_max_age_us,
            producer_lag_scope=args.producer_lag_scope)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(args.output.resolve()),
                      "producer_lag_us": result["producer_lag_us"],
                      "producer_lag_method": result["producer_lag_method"]},
                     sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
