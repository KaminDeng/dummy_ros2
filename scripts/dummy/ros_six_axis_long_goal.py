#!/usr/bin/env python3
"""Bounded BENCH_SESSION six-axis long goal; ROS action success is not hardware qualification."""

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import time
from typing import NamedTuple


JOINTS = tuple(f"Joint{axis}" for axis in range(1, 7))
DURATION_NS = 600_000_000_000
KNOT_NS = 2_000_000_000
CYCLE_NS = 20_000_000_000
AMPLITUDE_RAD = 0.01
MAX_POINTS = 301
MAX_TRAVEL_RAD = 10.0
MAX_START_AGE_S = 1.0
MAX_START_VELOCITY_RAD_S = 0.01
MAX_SERIALIZED_BYTES = 1024 * 1024
LONG_TIMING = {"control_period_us": 20_000,
               "max_goal_duration_us": 630_000_000,
               "trial_max_duration_us": 660_000_000,
               "motion_permit_us": 650_000_000,
               "session_lease_us": 30_000_000}


class Point(NamedTuple):
    time_ns: int
    positions: tuple
    velocities: tuple
    accelerations: tuple


class TrajectoryPath(NamedTuple):
    joint_names: tuple
    points: tuple
    travel_rad: float


def _load_neighbor(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + ".py"))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"required neighboring module unavailable: {name}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_long_policy(policy):
    if not isinstance(policy, dict):
        raise ValueError("bound long-goal policy object required")
    timing = policy.get("timing")
    if (not isinstance(timing, dict) or
            any(type(timing.get(key)) is not int or timing[key] != expected
                for key, expected in LONG_TIMING.items())):
        raise ValueError("long-goal timing must match bound 630/660/650/30s, 20ms candidate")
    limits = policy.get("limits")
    if (policy.get("reference_kind") != "BENCH_SESSION" or
            policy.get("no_load_bench") is not True or
            policy.get("gravity_loaded") is not False or
            policy.get("active_axes") != list(JOINTS) or
            not isinstance(limits, dict) or
            limits.get("max_trial_travel_rad") != MAX_TRAVEL_RAD):
        raise ValueError("bound no-load six-axis long-goal policy required")


def validate_long_policy_binding(policy, expected_sha256):
    validate_long_policy(policy)
    canonical = json.dumps({key: value for key, value in policy.items() if key != "note"},
                           sort_keys=True, separators=(",", ":")).encode()
    actual = hashlib.sha256(canonical).hexdigest()
    if (not isinstance(expected_sha256, str) or
            not re.fullmatch(r"[0-9a-f]{64}", expected_sha256) or
            actual != expected_sha256):
        raise ValueError("explicit long policy hash differs from bound candidate")
    return actual


def load_bound_candidate(limits_path, candidate_manifest, long_policy_path):
    converter = _load_neighbor("convert_application_ros_limits")
    bound = converter.validate_bound(Path(limits_path), Path(candidate_manifest))
    source_path = Path(long_policy_path)
    policy = converter.json_object(converter.read_snapshot(source_path), source_path)
    validate_long_policy_binding(policy, bound.limits["policy_sha256"])
    if (bound.gate_lease.permit_remaining_us != LONG_TIMING["motion_permit_us"] or
            bound.gate_lease.requested_session_lease_us != LONG_TIMING["session_lease_us"]):
        raise ValueError("bound Gate permit/lease mismatch")
    rows = tuple((joint["planning_position_min_rad"], joint["planning_position_max_rad"],
                  joint["velocity_max_rad_s"], joint["acceleration_max_rad_s2"])
                 for joint in bound.limits["joints"])
    return rows, bound.limits["policy_sha256"]


def _validate_start(start, limits):
    if (not isinstance(start, (list, tuple)) or len(start) != 6 or
            not isinstance(limits, (list, tuple)) or len(limits) != 6 or
            any(not _finite_number(value) for value in start)):
        raise ValueError("six finite BENCH_SESSION start positions required")
    for value, row in zip(start, limits):
        if (not isinstance(row, (list, tuple)) or len(row) != 4 or
                any(not _finite_number(entry) for entry in row) or
                not row[0] < row[1] or row[2] <= 0 or row[3] <= 0 or
                not row[0] <= value <= row[1]):
            raise ValueError("start or per-axis limits invalid")
    return tuple(float(value) for value in start)


def build_path(start, limits, *, amplitude_rad=AMPLITUDE_RAD):
    start = _validate_start(start, limits)
    if not _finite_number(amplitude_rad) or not 0 < amplitude_rad <= 0.04:
        raise ValueError("finite bounded amplitude required")
    signs = tuple(1 if value + amplitude_rad <= row[1] else -1
                  for value, row in zip(start, limits))
    if any(not row[0] <= value + sign * amplitude_rad <= row[1]
           for value, sign, row in zip(start, signs, limits)):
        raise ValueError("insufficient room for out-and-back excursion")
    points = []
    omega = 2 * math.pi / (CYCLE_NS / 1e9)
    for time_ns in range(0, DURATION_NS + 1, KNOT_NS):
        if time_ns in (0, DURATION_NS):
            displacement = velocity = acceleration = 0.0
        else:
            theta = omega * (time_ns / 1e9)
            cosine, sine = math.cos(theta), math.sin(theta)
            displacement = amplitude_rad * (1 - cosine) ** 2 / 4
            velocity = amplitude_rad * omega * (1 - cosine) * sine / 2
            acceleration = amplitude_rad * omega ** 2 * (sine ** 2 +
                            (1 - cosine) * cosine) / 2
        point = Point(time_ns,
                      tuple(value + sign * displacement for value, sign in zip(start, signs)),
                      tuple(sign * velocity for sign in signs),
                      tuple(sign * acceleration for sign in signs))
        if any(not math.isfinite(value) for values in
               (point.positions, point.velocities, point.accelerations) for value in values):
            raise ValueError("non-finite trajectory point")
        for axis, row in enumerate(limits):
            if (not row[0] <= point.positions[axis] <= row[1] or
                    abs(point.velocities[axis]) > row[2] or
                    abs(point.accelerations[axis]) > row[3]):
                raise ValueError("trajectory exceeds per-axis bound")
        points.append(point)
    travel = sum(abs(current.positions[axis] - previous.positions[axis])
                 for axis in range(6) for previous, current in zip(points, points[1:]))
    if len(points) > MAX_POINTS or travel > MAX_TRAVEL_RAD:
        raise ValueError("trajectory point or trial travel overflow")
    return TrajectoryPath(JOINTS, tuple(points), travel)


def validate_start_sample(sample, limits):
    if not isinstance(sample, dict) or tuple(sample.get("names", ())) != JOINTS:
        raise ValueError("fresh six-axis joint order required")
    age = sample.get("age_s")
    velocities = sample.get("velocities")
    if (not _finite_number(age) or not 0 <= age <= MAX_START_AGE_S or
            not isinstance(velocities, (list, tuple)) or len(velocities) != 6 or
            any(not _finite_number(value) or abs(value) > MAX_START_VELOCITY_RAD_S
                for value in velocities)):
        raise ValueError("fresh stationary joint state required")
    return _validate_start(sample.get("positions"), limits)


def _fresh_start(node, deadline, limits, movej):
    from sensor_msgs.msg import JointState

    samples = []

    def on_state(message):
        now_ns = node.get_clock().now().nanoseconds
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        sample = {"names": tuple(message.name), "positions": tuple(message.position),
                  "velocities": tuple(message.velocity),
                  "age_s": (now_ns - stamp_ns) / 1e9}
        try:
            position = validate_start_sample(sample, limits)
        except ValueError:
            return
        samples.append((time.monotonic(), position))

    subscriber = node.create_subscription(JointState, "/joint_states", on_state, 10)
    try:
        while len(samples) < 2:
            import rclpy
            rclpy.spin_once(node, timeout_sec=min(0.05, movej.remaining_time(deadline)))
        if (samples[-1][0] - samples[-2][0] > MAX_START_AGE_S or
                any(abs(a - b) > 0.003 for a, b in zip(samples[-1][1], samples[-2][1]))):
            raise ValueError("fresh stationary BENCH_SESSION state not confirmed")
        return samples[-1][1]
    finally:
        node.destroy_subscription(subscriber)


def _ros_trajectory(path):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    trajectory = JointTrajectory()
    trajectory.joint_names = list(path.joint_names)
    for point in path.points:
        item = JointTrajectoryPoint()
        item.positions = list(point.positions)
        item.velocities = list(point.velocities)
        item.accelerations = list(point.accelerations)
        item.time_from_start.sec, item.time_from_start.nanosec = divmod(point.time_ns, 1_000_000_000)
        trajectory.points.append(item)
    return trajectory


def success_report(path, digest, policy_sha256, goal_id, result,
                   started_ns, accepted_ns, ended_ns, feedback):
    if not started_ns <= accepted_ns <= ended_ns:
        raise RuntimeError("invalid long ROS action clock order")
    elapsed_us = (ended_ns - accepted_ns) // 1000
    if elapsed_us < DURATION_NS // 1000:
        raise RuntimeError("long ROS action returned before 600s")
    return {"scope": "ROS_ACTION_ONLY", "status": "SUCCEEDED", "goal_id": goal_id,
            "joint_names": list(JOINTS), "target_rad": list(path.points[-1].positions),
            "duration_s": DURATION_NS / 1e9, "planned_points": len(path.points),
            "total_travel_rad": path.travel_rad,
            "trajectory_sha256": digest, "policy_sha256": policy_sha256,
            "action_started_steady_us": started_ns // 1000,
            "action_accepted_steady_us": accepted_ns // 1000,
            "action_elapsed_us": elapsed_us,
            "action_status": result.status, "result_error_code": result.result.error_code,
            "feedback": feedback, "device_can_evidence": "REQUIRED_SEPARATELY",
            "device_qualification": "NOT_CLAIMED"}


def _send_goal(node, client, adapter, path, deadline, policy_sha256, movej):
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from rclpy.serialization import serialize_message

    trajectory = _ros_trajectory(path)
    serialized = serialize_message(trajectory)
    if len(serialized) > MAX_SERIALIZED_BYTES:
        raise ValueError("long trajectory serialized byte overflow")
    digest = movej.trajectory_digest(trajectory)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    feedback = []
    feedback_error = []
    started = time.monotonic_ns()
    last_sample_s = -1

    def on_feedback(message):
        nonlocal last_sample_s
        sample = message.feedback
        if (tuple(sample.joint_names) != JOINTS or len(sample.actual.positions) != 6 or
                len(sample.desired.positions) != 6 or
                any(not math.isfinite(value) for value in
                    (*sample.actual.positions, *sample.desired.positions))):
            feedback_error.append("invalid six-axis ROS feedback")
            return
        elapsed_s = (time.monotonic_ns() - started) // 1_000_000_000
        if elapsed_s > last_sample_s:
            last_sample_s = elapsed_s
            feedback.append({"elapsed_us": (time.monotonic_ns() - started) // 1000,
                             "actual_rad": list(sample.actual.positions),
                             "desired_rad": list(sample.desired.positions)})

    result = None
    goal_id = None
    try:
        for attempt in range(movej.MAX_ADMISSION_RETRIES + 1):
            started = time.monotonic_ns()
            handle = movej.wait(node, client.send_goal_async(goal, feedback_callback=on_feedback),
                                movej.remaining_time(deadline))
            if handle.accepted:
                accepted = time.monotonic_ns()
                goal_id = bytes(handle.goal_id.uuid).hex()
                break
            if (feedback or feedback_error or attempt == movej.MAX_ADMISSION_RETRIES or
                    not movej.has_matching_renewal_receipt(adapter, handle.goal_id, deadline) or
                    not movej.wait_for_admission_ready(adapter, deadline)):
                raise RuntimeError("guarded long goal rejected")
        result = movej.await_accepted_goal(node, handle, movej.remaining_time(deadline))
        ended = time.monotonic_ns()
        if (result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0 or
                feedback_error or not feedback):
            raise RuntimeError("long ROS action failed or feedback invalid")
        report = success_report(path, digest, policy_sha256, goal_id, result,
                                started, accepted, ended, feedback)
    except Exception as error:
        failed = {"scope": "ROS_ACTION_ONLY", "status": "FAILED", "reason": str(error),
                  "trajectory_sha256": digest, "policy_sha256": policy_sha256,
                  "device_qualification": "NOT_CLAIMED"}
        if goal_id is not None:
            failed["goal_id"] = goal_id
        raise movej.GoalAttemptError(str(error), failed) from error
    return report


def capture(limits, candidate_manifest, activation_sha256, device_id, policy_sha256,
            *, activate_controller=False):
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient

    movej = _load_neighbor("ros_six_axis_movej")
    rclpy.init()
    node = rclpy.create_node("ros_six_axis_long_goal")
    deadline = time.monotonic() + 640.0
    adapter = movej.RosControlAdapter(node, deadline)
    owned = []
    bound = False
    release_attempted = False
    try:
        movej.verify_live_gate(adapter, activation_sha256, device_id, candidate_manifest)
        bound = True
        if activate_controller:
            movej.activate_controller_stack(adapter, owned)
        else:
            movej.verify_active_stack(adapter)
        start = _fresh_start(node, deadline, limits, movej)
        path = build_path(start, limits)
        client = ActionClient(node, FollowJointTrajectory,
                              "/arm_controller/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=movej.remaining_time(deadline)):
            raise RuntimeError("public guarded action unavailable")
        report = _send_goal(node, client, adapter, path, deadline, policy_sha256, movej)
        release_attempted = True
        return movej.complete_success(adapter, owned, report)
    except BaseException as error:
        if bound and not release_attempted:
            try:
                active, unknown = movej.recover_after_failure(adapter, owned)
            except movej.UnconfirmedStopError as stop_error:
                raise movej.UnconfirmedStopError(
                    f"{stop_error}; original failure: {error}",
                    stop_error.still_active_components,
                    stop_error.unresolved_components) from error
            except BaseException as cleanup_error:
                raise movej.UnconfirmedStopError(
                    f"UNCONFIRMED_STOP: cleanup interrupted: {cleanup_error}; "
                    f"original failure: {error}") from error
            raise movej.ConfirmedStopError(
                str(error), active,
                failed_goal_report=getattr(error, "goal_report", None)) from error
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="六轴 BENCH_SESSION 长目标，默认仅离线检查")
    parser.add_argument("--candidate-manifest", type=Path, required=True)
    parser.add_argument("--ros-limits-json", type=Path, required=True)
    parser.add_argument("--long-policy", type=Path, required=True)
    parser.add_argument("--activation-sha256", required=True)
    parser.add_argument("--device-id", required=True)
    parser.add_argument("--start", nargs=6, type=float, metavar="RAD")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--activate-controller", action="store_true")
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if args.execute == (args.start is not None):
        parser.error("offline check requires --start; --execute requires fresh live state")
    if args.activate_controller and not args.execute:
        parser.error("controller activation requires --execute")
    if (not re.fullmatch(r"[0-9a-f]{64}", args.activation_sha256) or
            args.activation_sha256 == "0" * 64 or
            not re.fullmatch(r"[0-9a-f]{32}", args.device_id) or
            args.device_id == "0" * 32):
        parser.error("pinned activation SHA-256 and device ID required")
    try:
        limits, policy_sha256 = load_bound_candidate(args.ros_limits_json,
                                                      args.candidate_manifest,
                                                      args.long_policy)
        if args.execute:
            report = capture(limits, args.candidate_manifest, args.activation_sha256,
                             args.device_id, policy_sha256,
                             activate_controller=args.activate_controller)
        else:
            path = build_path(args.start, limits)
            report = {"scope": "OFFLINE_PLAN_ONLY", "status": "VALIDATED",
                      "joint_names": list(path.joint_names),
                      "duration_s": DURATION_NS / 1e9,
                      "planned_points": len(path.points),
                      "total_travel_rad": path.travel_rad,
                      "policy_sha256": policy_sha256,
                      "ros_action_functional": "NOT_RUN",
                      "device_qualification": "NOT_CLAIMED"}
    except (RuntimeError, ValueError) as error:
        status = "UNCONFIRMED_STOP" if "UNCONFIRMED_STOP" in str(error) else "FAIL"
        report = {"scope": "ROS_ACTION_ONLY" if args.execute else "OFFLINE_PLAN_ONLY",
                  "status": status, "reason": str(error),
                  "stop_confirmation": ("UNCONFIRMED_STOP" if status == "UNCONFIRMED_STOP"
                                        else "CONFIRMED_RELEASED_IDLE" if
                                        type(error).__name__ == "ConfirmedStopError"
                                        else "NOT_REQUESTED"),
                  "ros_action_functional": "NOT_CONFIRMED",
                  "device_qualification": "NOT_CLAIMED"}
        if getattr(error, "failed_goal_report", None) is not None:
            report["failed_goal"] = error.failed_goal_report
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 0 if report["status"] in ("VALIDATED", "SUCCEEDED") else 1


if __name__ == "__main__":
    raise SystemExit(main())
