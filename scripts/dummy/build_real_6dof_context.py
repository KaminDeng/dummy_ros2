#!/usr/bin/env python3
"""从同一次 ROS 原始采集生成 join_real_6dof_evidence 所需的上下文。"""

import argparse
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
                  ros_feedback_lag_budget_us, gate_state_max_age_us):
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
    producer_lag = max(offsets) - min(offsets)
    if producer_lag > producer_lag_budget_us:
        raise ValueError("observed State producer lag exceeds producer lag budget")
    try:
        action = json.loads(snapshots[2][0])
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("invalid action JSON") from error
    if not isinstance(action, dict):
        raise ValueError("invalid action JSON object")

    context = {
        "timeline_sha256": snapshots[0][1], "raw_sha256": snapshots[1][1],
        "action_sha256": snapshots[2][1], "client_id": next(iter(clients)),
        "path_tolerance_rad": path_tolerance_rad,
        "goal_tolerance_rad": goal_tolerance_rad,
        "producer_lag_us": producer_lag,
        "producer_lag_method": PRODUCER_LAG_METHOD,
        "producer_lag_budget_us": producer_lag_budget_us,
        "feedback_age_budget_us": feedback_age_budget_us,
        "ros_feedback_lag_budget_us": ros_feedback_lag_budget_us,
        "gate_state_max_age_us": gate_state_max_age_us,
    }
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
    args = parser.parse_args(argv)
    try:
        result = build_context(
            args.timeline, args.raw, args.action, args.output,
            path_tolerance_rad=args.path_tolerance_rad,
            goal_tolerance_rad=args.goal_tolerance_rad,
            producer_lag_budget_us=args.producer_lag_budget_us,
            feedback_age_budget_us=args.feedback_age_budget_us,
            ros_feedback_lag_budget_us=args.ros_feedback_lag_budget_us,
            gate_state_max_age_us=args.gate_state_max_age_us)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(args.output.resolve()),
                      "producer_lag_us": result["producer_lag_us"],
                      "producer_lag_method": result["producer_lag_method"]},
                     sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
