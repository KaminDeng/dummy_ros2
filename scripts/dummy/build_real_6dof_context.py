#!/usr/bin/env python3
"""从同一次 ROS 原始采集生成 join_real_6dof_evidence 所需的上下文。"""

import argparse
from bisect import bisect_right
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
        goal_times = [int(row["steady_us"]) for row in goals]
        goal_phases = []
        for row in rows:
            if row["event"] == "accepted":
                goal_phases.append(set())
            if int(row["session"]) == session:
                goal_phases[-1].add((int(row["reference_epoch"]),
                                     int(row["motion_epoch"])))
        phase_markers = [(int(row["steady_us"]), int(row["motion_epoch"]))
                         for row in rows if row["event"] in
                         ("accepted", "after_begin", "terminal_success",
                          "terminal_canceled", "pre_release_hold",
                          "post_release_status")]
        release_phase = (int(final["reference_epoch"]), int(final["motion_epoch"]))
        releasing_phase = (int(goals[-1]["reference_epoch"]), release_phase[1])
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
    if any(later[0] < earlier[0] or later[1] < earlier[1] for earlier, later in
           zip(phase_markers, phase_markers[1:])):
        raise ValueError("ROS motion window timeline phase regression")
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
    device_id = requests[0]["fields"]["device_id"]
    if device_id == "00" * 16:
        raise ValueError("motion window device identity missing")
    cancel_bounds = {}
    for goal_index, goal in enumerate(goals):
        goal_rows = rows[rows.index(goal):rows.index(goals[goal_index + 1])
                         if goal_index + 1 < len(goals) else len(rows)]
        canceled = [row for row in goal_rows if row["event"] == "terminal_canceled"]
        if not canceled:
            continue
        if len(canceled) != 1:
            raise ValueError("ambiguous ROS canceled goal phase")
        terminal = canceled[0]
        old_epoch = int(goal["motion_epoch"])
        stop_epoch = int(terminal["motion_epoch"])
        request_id = int(terminal["cancel_request_id"])
        operation_id = int(terminal["cancel_operation_id"])
        terminal_time = int(terminal["steady_us"])
        cancel_requests = [item for item in records if item["kind"] == 4 and
                           item["fields"]["operation"] == 9 and
                           item["fields"]["request_id"] == request_id and
                           item["fields"]["client_instance_id"] == client and
                           item["fields"]["device_id"] == device_id and
                           item["fields"]["expected_motion_epoch"] == old_epoch and
                           (item["boot"], item["session"]) == (boot, session) and
                           goal_times[goal_index] <= item["when"] <= terminal_time]
        stop_pages = [item for item in records if item["kind"] == 1 and
                      (item["boot"], item["session"], item["run"]) ==
                      (boot, session, int(goal["run_id"])) and
                      (item["fields"]["attempt_id"], item["fields"]["attempt_kind"],
                       item["fields"]["category"], item["fields"]["motion_epoch"]) ==
                      (int(goal["run_id"]), 1, 2, stop_epoch) and
                      cancel_requests and
                      cancel_requests[0]["when"] <= item["when"] <= terminal_time]
        query_binding = ("expected_boot_epoch", "expected_session_id",
                         "expected_reference_epoch", "expected_motion_epoch",
                         "expected_management_epoch", "expected_profile_id",
                         "configured_mask")
        cancel_queries = [item for item in records if item["kind"] == 4 and
                          item["fields"]["operation"] == 12 and
                          item["fields"]["queried_operation_id"] == request_id and
                          item["fields"]["client_instance_id"] == client and
                          item["fields"]["device_id"] == device_id and
                          (item["boot"], item["session"]) == (boot, session) and
                          cancel_requests and
                          all(item["fields"][name] ==
                              cancel_requests[0]["fields"][name]
                              for name in query_binding) and
                          cancel_requests[0]["when"] <= item["when"] <= terminal_time]
        receipts = [item for item in records if item["kind"] == 5 and
                    (item["fields"]["request_id"] == request_id or
                     any(query["fields"]["request_id"] ==
                         item["fields"]["request_id"] and
                         query["when"] <= item["when"] for query in cancel_queries)) and
                    item["fields"]["operation_id"] == operation_id and
                    item["fields"]["client_instance_id"] == client and
                    item["fields"]["device_id"] == device_id and
                    item["fields"]["transaction_phase"] == 3 and
                    item["fields"]["result_code"] == 0 and
                    item["fields"]["receipt"] == 1 and
                    item["fields"]["motion_complete"] == 0 and
                    item["fields"]["release_committed"] == 0 and
                    item["fields"]["motion_epoch"] == stop_epoch and
                    (item["boot"], item["session"], item["run"]) ==
                    (boot, session, int(goal["run_id"])) and
                    goal_times[goal_index] <= item["when"] <= terminal_time]
        if (stop_epoch != old_epoch + 1 or request_id <= 0 or operation_id <= 0 or
                len(cancel_requests) != 1 or not stop_pages or len(receipts) != 1 or
                not cancel_requests[0]["when"] <= receipts[0]["when"]):
            raise ValueError("missing or unlinked raw cancel Stop CAN page")
        cancel_bounds[goal_index] = (cancel_requests[0]["when"],
                                     stop_pages[0]["when"], old_epoch, stop_epoch)
    idle_statuses = [item for item in records if item["kind"] == 3 and
                     (item["boot"], item["session"], item["fields"]["run_id"]) ==
                     (boot, 0, release_run) and
                     item["fields"]["device_id"] == device_id and
                     item["fields"]["core_state"] == 1 and
                     item["fields"]["release_committed"] == 1 and
                     terminals[0]["when"] <= item["when"] <= end and
                     end - item["when"] <= state_max_age]
    idle_states = [item for item in records if item["kind"] == 2 and
                   (item["boot"], item["session"]) == (boot, 0) and
                   item["fields"]["device_id"] == device_id and
                   (item["fields"]["reference_epoch"],
                    item["fields"]["motion_epoch"]) == release_phase and
                   item["fields"]["core_state"] == 1 and
                   terminals[0]["when"] <= item["when"] <= end and
                   end - item["when"] <= state_max_age]
    if not idle_statuses or not idle_states:
        raise ValueError("missing fresh raw Release Idle observation")
    window_states = [item for item in records if item["kind"] == 2 and
                     start <= item["when"] <= end]
    if len(window_states) < 2:
        raise ValueError("insufficient motion window State samples")
    if any(item["boot"] != boot or item["session"] not in (session, 0) or
           item["fields"]["device_id"] != device_id or
           (item["session"] == 0 and
            ((item["fields"]["reference_epoch"],
              item["fields"]["motion_epoch"]) != release_phase or
             item["when"] < requests[0]["when"] or
             item["fields"]["core_state"] != 1)) or
           (item["session"] == session and
            ((item["fields"]["reference_epoch"],
              item["fields"]["motion_epoch"]) not in
             goal_phases[bisect_right(goal_times, item["when"]) - 1] and
             not (item["when"] >= requests[0]["when"] and
                  bisect_right(goal_times, item["when"]) == len(goals) and
                  (item["fields"]["reference_epoch"],
                   item["fields"]["motion_epoch"]) == releasing_phase) or
             item["when"] > terminals[0]["when"]))
           for item in window_states):
        raise ValueError("motion window State identity mismatch")
    coverage_states = []
    for item in window_states:
        if item["session"] != session:
            coverage_states.append(item)
            continue
        goal_index = bisect_right(goal_times, item["when"]) - 1
        bound = cancel_bounds.get(goal_index)
        if bound is None:
            coverage_states.append(item)
            continue
        request_time, stop_time, old_epoch, stop_epoch = bound
        epoch = item["fields"]["motion_epoch"]
        if ((item["when"] < request_time and epoch != old_epoch) or
                (item["when"] > stop_time and epoch != stop_epoch)):
            raise ValueError("motion window State identity phase mismatch")
        if not request_time <= item["when"] <= stop_time:
            coverage_states.append(item)
    marker_index = 0
    phase_floor = phase_markers[0][1]
    previous_epoch = phase_floor
    for item in window_states:
        while (marker_index + 1 < len(phase_markers) and
               phase_markers[marker_index + 1][0] <= item["when"]):
            marker_index += 1
            phase_floor = phase_markers[marker_index][1]
        epoch = item["fields"]["motion_epoch"]
        if epoch < phase_floor or epoch < previous_epoch:
            raise ValueError("motion window State identity phase regression")
        previous_epoch = epoch
    observations = [start, *(item["when"] for item in coverage_states), end]
    if any(later - earlier > state_max_age for earlier, later in
           zip(observations, observations[1:])):
        raise ValueError("insufficient motion window State coverage")
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
