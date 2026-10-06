#!/usr/bin/env python3
"""通过真实 MoveIt 规划服务和公开 FollowJointTrajectory 采集六轴 ROS 记录。"""

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import struct
import sys
import time
import uuid


JOINTS = tuple(f"Joint{axis}" for axis in range(1, 7))
LIMITS = ((-2.9670597, 2.9670597), (-1.2740904, 1.5707963),
          (0.6108652, 3.1415927), (-3.1415927, 3.1415927),
          (-2.0943952, 2.0943952), (-12.5663706, 12.5663706))
STOP_CONFIRM_TIMEOUT = 5.0
PREEMPT_B_PROBE_TIMEOUT = 2.0
POST_CANCEL_SUBMISSION_TIMEOUT = 4.0
TERMINAL_ACTION_STATUSES = frozenset((4, 5, 6))  # GoalStatus: succeeded, canceled, aborted.
HARDWARE_NAME = "dummy_ref_real_6dof"
CONTROLLERS = ("joint_state_broadcaster", "arm_controller")
ALL_COMPONENTS = ("hardware",) + CONTROLLERS
GATE_PARAMETERS = ("activation_sha256", "device_id", "contract_bundle",
                   "profile_name", "guarded_controller")
MAX_DIAGNOSTIC_POINTS = 512
MAX_SERIALIZED_TRAJECTORY_BYTES = 1024 * 1024
MAX_MULTI_GOALS = 50
MAX_TARGETS_JSON_BYTES = 65536
MAX_MULTI_GOAL_TIMEOUT_S = 120.0
MAX_FEEDBACK_SAMPLES = 1024
MAX_ADMISSION_RETRIES = 8
MID_MOTION_DELTA_RAD = 0.001
ADMISSION_PENDING_STATUSES = frozenset(("LEASE_RENEWAL_PENDING", "BOOTSTRAP_PENDING"))
STOP_REPLY_CATEGORIES = frozenset((
    "RELEASE_QUEUED", "RELEASE_UNAVAILABLE", "RELEASE_PENDING", "RELEASED_IDLE",
    "RELEASE_UNCONFIRMED", "RELEASE_OBSERVATION_UNAVAILABLE",
    "BOOTSTRAP_PENDING", "ENABLED_HOLD", "OPERATION_COMPLETE",
    "LEASE_RENEWAL_PENDING", "LEASE_RENEWED",
    "LEASE_RENEWAL_UNCONFIRMED", "FAULT", "OTHER", "NOT_OBSERVED"))
RELEASE_RETRY_STATUSES = frozenset((
    "BOOTSTRAP_PENDING", "ENABLED_HOLD", "OPERATION_COMPLETE",
    "LEASE_RENEWAL_PENDING", "LEASE_RENEWED"))
PERMIT_RETRY_STATUSES = frozenset((
    "BOOTSTRAP_PENDING", "ENABLED_HOLD", "OPERATION_COMPLETE",
    "LEASE_RENEWAL_PENDING", "LEASE_RENEWED"))
PERMIT_FAILURE_STATUSES = frozenset((
    "LEASE_RENEWAL_UNCONFIRMED", "FAULT", "AUTHORITY_CHANGED"))
STOP_PHASE_CATEGORIES = frozenset((
    "NOT_STARTED", "RELEASE_CALL", "RELEASE_CALL_FAILED", "BOOTSTRAP_CALL",
    "BOOTSTRAP_CALL_FAILED", "BOOTSTRAP_WAIT"))


class UnconfirmedStopError(RuntimeError):
    """Action 未返回终态，不能确认运动已经停止。"""

    def __init__(self, message, still_active_components=(), unresolved_components=(),
                 completed_goals=(), failed_goal_index=None, failed_goal_report=None):
        super().__init__(message)
        self.still_active_components = list(still_active_components)
        self.unresolved_components = list(unresolved_components)
        self.completed_goals = list(completed_goals)
        self.failed_goal_index = failed_goal_index
        self.failed_goal_report = failed_goal_report


class ConfirmedStopError(RuntimeError):
    """守护门已确认 RELEASED_IDLE，但采集仍失败。"""

    def __init__(self, message, still_active_components=(), completed_goals=(),
                 failed_goal_index=None, failed_goal_report=None):
        super().__init__(message)
        self.still_active_components = list(still_active_components)
        self.completed_goals = list(completed_goals)
        self.failed_goal_index = failed_goal_index
        self.failed_goal_report = failed_goal_report


class RejectedPlanError(RuntimeError):
    """Guarded action rejected the exact MoveIt trajectory sent to it."""

    def __init__(self, diagnostic):
        super().__init__("guarded action rejected MoveIt plan")
        self.diagnostic = diagnostic


class GoalAttemptError(RuntimeError):
    def __init__(self, message, goal_report):
        super().__init__(message)
        self.goal_report = goal_report


class CancelAttemptError(RuntimeError):
    def __init__(self, message, result=None, cancel=None):
        super().__init__(message)
        self.result = result
        self.cancel = cancel


def load_bench_limits(limits_path, candidate_manifest):
    script = Path(__file__).with_name("convert_application_ros_limits.py")
    spec = importlib.util.spec_from_file_location("convert_application_ros_limits", script)
    if spec is None or spec.loader is None:
        raise ValueError("no-load limits verifier unavailable")
    converter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(converter)
    data = converter.validate(Path(limits_path), Path(candidate_manifest))
    return tuple((joint["planning_position_min_rad"], joint["planning_position_max_rad"])
                 for joint in data["joints"])


def validate_target(values, limits=LIMITS):
    if len(values) != 6 or any(type(value) not in (float, int) or not math.isfinite(value)
                               for value in values):
        raise ValueError("six finite joint targets required")
    if any(not lower <= value <= upper for value, (lower, upper) in zip(values, limits)):
        raise ValueError("joint target outside selected reference limits")
    return tuple(values)


def load_targets_json(path, limits=LIMITS):
    path = Path(path)
    if path.stat().st_size > MAX_TARGETS_JSON_BYTES:
        raise ValueError("targets JSON exceeds size limit")
    raw = path.read_bytes()
    if len(raw) > MAX_TARGETS_JSON_BYTES:
        raise ValueError("targets JSON exceeds size limit")
    values = json.loads(raw)
    if not isinstance(values, list) or not 1 <= len(values) <= MAX_MULTI_GOALS:
        raise ValueError("targets JSON requires one to fifty six-axis targets")
    if any(not isinstance(value, list) for value in values):
        raise ValueError("targets JSON requires six-axis arrays")
    return tuple(validate_target(value, limits) for value in values)


def validate_plan(trajectory, target=None, limits=LIMITS):
    if tuple(trajectory.joint_names) != JOINTS or not trajectory.points:
        raise ValueError("MoveIt plan missing or reordering six joints")
    if any(len(point.positions) != 6 or any(not math.isfinite(value) for value in point.positions)
           for point in trajectory.points):
        raise ValueError("MoveIt plan has invalid six-axis positions")
    for point in trajectory.points:
        validate_target(point.positions, limits)
    if target is not None and any(abs(actual - requested) > 0.003 for actual, requested in
                                  zip(trajectory.points[-1].positions, target)):
        raise ValueError("MoveIt plan terminal differs from requested target")


def trajectory_digest(trajectory):
    # Versioned semantic encoding; ROS CDR buffers may contain allocator padding.
    encoded = bytearray(b"DPSDK-JT-1\0")

    def u32(value):
        if type(value) is not int or not 0 <= value <= 0xffffffff:
            raise ValueError("trajectory integer outside uint32 range")
        encoded.extend(struct.pack("<I", value))

    def stamp(value):
        if type(value.sec) is not int or not -(1 << 31) <= value.sec < (1 << 31):
            raise ValueError("trajectory sec outside int32 range")
        encoded.extend(struct.pack("<i", value.sec))
        if type(value.nanosec) is not int or not 0 <= value.nanosec < 1_000_000_000:
            raise ValueError("trajectory nanosec outside range")
        u32(value.nanosec)

    def string(value):
        if not isinstance(value, str):
            raise ValueError("trajectory string required")
        try:
            raw = value.encode("utf-8")
        except UnicodeEncodeError as error:
            raise ValueError("trajectory string must be UTF-8") from error
        u32(len(raw))
        encoded.extend(raw)

    def doubles(values):
        u32(len(values))
        for value in values:
            if not math.isfinite(value):
                raise ValueError("trajectory values must be finite")
            encoded.extend(struct.pack("<d", 0.0 if value == 0.0 else value))

    stamp(trajectory.header.stamp)
    string(trajectory.header.frame_id)
    u32(len(trajectory.joint_names))
    for name in trajectory.joint_names:
        string(name)
    u32(len(trajectory.points))
    for point in trajectory.points:
        for values in (point.positions, point.velocities, point.accelerations, point.effort):
            doubles(values)
        stamp(point.time_from_start)
    return hashlib.sha256(encoded).hexdigest()


def rejection_diagnostic(trajectory, serialized):
    if len(trajectory.points) > MAX_DIAGNOSTIC_POINTS:
        raise RuntimeError("MoveIt plan exceeds bounded diagnostic point count")
    if len(serialized) > MAX_SERIALIZED_TRAJECTORY_BYTES:
        raise RuntimeError("MoveIt plan exceeds bounded diagnostic byte count")
    points = []
    time_deltas = []
    previous_ns = 0
    axis_maxima = [{"joint_name": name, "abs_position_rad": 0.0,
                    "abs_velocity_rad_s": None, "abs_acceleration_rad_s2": None}
                   for name in trajectory.joint_names]
    for point in trajectory.points:
        stamp = point.time_from_start
        current_ns = stamp.sec * 1000000000 + stamp.nanosec
        time_deltas.append(current_ns - previous_ns)
        previous_ns = current_ns
        points.append({"positions": list(point.positions),
                       "velocities": list(point.velocities),
                       "accelerations": list(point.accelerations),
                       "time_from_start": {"sec": stamp.sec, "nanosec": stamp.nanosec}})
        for index, maximum in enumerate(axis_maxima):
            maximum["abs_position_rad"] = max(maximum["abs_position_rad"],
                                               abs(point.positions[index]))
            for field, values in (("abs_velocity_rad_s", point.velocities),
                                  ("abs_acceleration_rad_s2", point.accelerations)):
                if index < len(values):
                    maximum[field] = max(maximum[field] or 0.0, abs(values[index]))
    return {"trajectory_sha256": trajectory_digest(trajectory),
            "serialized_bytes": len(serialized), "joint_names": list(trajectory.joint_names),
            "points": points, "time_deltas_ns": time_deltas, "axis_maxima": axis_maxima}


def rejection_diagnostic_from_error(error):
    while error is not None:
        if isinstance(error, RejectedPlanError):
            return error.diagnostic
        error = error.__cause__
    return None


def wait(node, future, timeout):
    import rclpy
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError("ROS request timed out or failed")
    return future.result()


def remaining_time(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError("ROS operation deadline expired")
    return remaining


class RosControlAdapter:
    def __init__(self, node, deadline):
        from controller_manager_msgs.srv import (ListControllers, ListHardwareComponents,
                                                 SetHardwareComponentState, SwitchController)
        from lifecycle_msgs.msg import State
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterType
        from std_srvs.srv import Trigger

        self.node = node
        self.deadline = deadline
        self.ListControllers = ListControllers
        self.ListHardwareComponents = ListHardwareComponents
        self.SetHardwareComponentState = SetHardwareComponentState
        self.SwitchController = SwitchController
        self.State = State
        self.Trigger = Trigger
        self.GetParameters = GetParameters
        self.ParameterType = ParameterType
        self.pending_transitions = []

    def call(self, name, service_type, request, deadline=None, transition=False):
        deadline = self.deadline if deadline is None else deadline
        client = self.node.create_client(service_type, name)
        future = None
        try:
            if not client.wait_for_service(timeout_sec=remaining_time(deadline)):
                raise RuntimeError(f"ROS service unavailable: {name}")
            future = client.call_async(request)
            if transition:
                self.pending_transitions.append((client, future))
            response = wait(self.node, future, remaining_time(deadline))
            if response is None:
                raise RuntimeError(f"ROS service returned no response: {name}")
            return response
        finally:
            if transition and future is not None and future.done():
                self.pending_transitions.remove((client, future))
            if not transition or future is None or future.done():
                self.node.destroy_client(client)

    def settle_pending(self, deadline):
        for client, future in tuple(self.pending_transitions):
            try:
                wait(self.node, future, remaining_time(deadline))
            except RuntimeError:
                pass
            if not future.done():
                return False
            self.pending_transitions.remove((client, future))
            self.node.destroy_client(client)
        return True

    def gate_parameters(self):
        response = self.call("/device_action_gate/get_parameters", self.GetParameters,
                             self.GetParameters.Request(names=list(GATE_PARAMETERS)))
        if len(response.values) != len(GATE_PARAMETERS) or any(
                value.type != self.ParameterType.PARAMETER_STRING for value in response.values):
            raise RuntimeError("live gate identity parameters unavailable")
        return dict(zip(GATE_PARAMETERS, (value.string_value for value in response.values)))

    def confirmed_hold(self):
        response = self.call("/device_action_gate/bootstrap_status", self.Trigger,
                             self.Trigger.Request())
        return response.success and response.message == "ENABLED_HOLD"

    def renew_motion_permit(self):
        set_reply = "NOT_ATTEMPTED"
        bootstrap_reply = "NOT_OBSERVED"

        def time_left():
            try:
                return remaining_time(self.deadline)
            except RuntimeError as error:
                raise RuntimeError(
                    "new motion permit deadline expired "
                    f"(set_permit={set_reply}, bootstrap={bootstrap_reply})") from error

        while True:
            time_left()
            queued = self.call("/device_action_gate/set_bench_permit", self.Trigger,
                               self.Trigger.Request())
            set_reply = "QUEUED" if queued.success else "UNAVAILABLE"
            if queued.success:
                break
            time_left()
            status = self.call("/device_action_gate/bootstrap_status", self.Trigger,
                               self.Trigger.Request())
            bootstrap_reply = permit_status_category(status.message)
            if bootstrap_reply not in PERMIT_RETRY_STATUSES:
                raise RuntimeError("new motion permit unavailable "
                                   f"(set_permit={set_reply}, bootstrap={bootstrap_reply})")
            time.sleep(min(0.05, time_left()))
        while True:
            time_left()
            status = self.call("/device_action_gate/bootstrap_status", self.Trigger,
                               self.Trigger.Request())
            bootstrap_reply = permit_status_category(status.message)
            if status.success and status.message == "ENABLED_HOLD":
                return
            if bootstrap_reply not in PERMIT_RETRY_STATUSES:
                raise RuntimeError("new motion permit unconfirmed "
                                   f"(set_permit={set_reply}, bootstrap={bootstrap_reply})")
            time.sleep(min(0.05, time_left()))

    def set_hardware_state(self, name, state, deadline=None):
        request = self.SetHardwareComponentState.Request(name=name)
        request.target_state.id = (self.State.PRIMARY_STATE_ACTIVE if state == "active"
                                   else self.State.PRIMARY_STATE_INACTIVE)
        request.target_state.label = state
        response = self.call("/controller_manager/set_hardware_component_state",
                             self.SetHardwareComponentState, request, deadline, transition=True)
        if not response.ok:
            raise RuntimeError(f"hardware {state} transition rejected: {name}")

    def activate_hardware(self, name):
        self.set_hardware_state(name, "active")

    def deactivate_hardware(self, name):
        deadline = self.deactivation_deadline
        self.set_hardware_state(name, "inactive", deadline)
        if self.hardware_state(name, deadline) != "inactive":
            raise RuntimeError(f"hardware remains active: {name}")

    def hardware_state(self, name, deadline=None):
        response = self.call("/controller_manager/list_hardware_components",
                             self.ListHardwareComponents, self.ListHardwareComponents.Request(), deadline)
        matches = [component for component in response.component if component.name == name]
        return matches[0].state.label if len(matches) == 1 else None

    def switch_controller(self, name, activate, deadline=None):
        request = self.SwitchController.Request()
        if activate:
            request.activate_controllers = [name]
        else:
            request.deactivate_controllers = [name]
        request.strictness = self.SwitchController.Request.STRICT
        request.timeout.sec = min(5, max(1, math.ceil(remaining_time(
            self.deadline if deadline is None else deadline))))
        response = self.call("/controller_manager/switch_controller", self.SwitchController,
                             request, deadline, transition=True)
        if not response.ok:
            raise RuntimeError(f"controller transition rejected: {name}")

    def activate_controller(self, name):
        self.switch_controller(name, True)

    def deactivate_controller(self, name):
        deadline = self.deactivation_deadline
        self.switch_controller(name, False, deadline)
        if self.controller_state(name, deadline) != "inactive":
            raise RuntimeError(f"controller remains active: {name}")

    def controller_state(self, name, deadline=None):
        response = self.call("/controller_manager/list_controllers", self.ListControllers,
                             self.ListControllers.Request(), deadline)
        matches = [controller for controller in response.controller if controller.name == name]
        return matches[0].state if len(matches) == 1 else None

    def confirmed_stop(self):
        deadline = self.stop_deadline
        self.stop_release_reply = "NOT_OBSERVED"
        self.stop_bootstrap_reply = "NOT_OBSERVED"
        release_pending = False
        release_requested = False
        while True:
            remaining_time(deadline)
            if not release_pending:
                self.stop_phase = "RELEASE_CALL"
                try:
                    response = self.call("/device_action_gate/disable_and_release", self.Trigger,
                                         self.Trigger.Request(), deadline)
                except Exception:
                    self.stop_phase = "RELEASE_CALL_FAILED"
                    raise
                self.stop_release_reply = stop_reply_category(response.message)
                if response.message in ("RELEASE_QUEUED", "RELEASED_IDLE"):
                    release_pending = True
                    release_requested = True
                elif response.message != "RELEASE_UNAVAILABLE":
                    return False
            self.stop_phase = "BOOTSTRAP_CALL"
            try:
                status = self.call("/device_action_gate/bootstrap_status", self.Trigger,
                                   self.Trigger.Request(), deadline)
            except Exception:
                self.stop_phase = "BOOTSTRAP_CALL_FAILED"
                raise
            self.stop_bootstrap_reply = stop_reply_category(status.message)
            if status.success and status.message == "RELEASED_IDLE":
                return True
            if status.message == "RELEASE_PENDING":
                release_pending = True
            elif status.message == "RELEASE_OBSERVATION_UNAVAILABLE" and release_requested:
                pass
            elif release_pending or status.message not in RELEASE_RETRY_STATUSES:
                return False
            self.stop_phase = "BOOTSTRAP_WAIT"
            time.sleep(min(0.05, remaining_time(deadline)))


def verify_live_gate(adapter, activation_sha256, device_id, candidate_manifest):
    values = adapter.gate_parameters()
    expected = {"activation_sha256": activation_sha256, "device_id": device_id,
                "contract_bundle": str(Path(candidate_manifest).parent.resolve()),
                "profile_name": "f405_6dof_bench", "guarded_controller": "arm_controller"}
    if any(values.get(key) != value for key, value in expected.items()):
        raise RuntimeError("live gate identity does not match pinned BENCH_SESSION inputs")


def component_name(component):
    return HARDWARE_NAME if component == "hardware" else component


def stop_reply_category(message):
    return message if isinstance(message, str) and message in STOP_REPLY_CATEGORIES else "OTHER"


def permit_status_category(message):
    if isinstance(message, str):
        if message in PERMIT_RETRY_STATUSES or message in PERMIT_FAILURE_STATUSES:
            return message
        if message.startswith("EVIDENCE_INCOMPLETE;"):
            return "EVIDENCE_INCOMPLETE"
    return "OTHER"


def observed_active_components(adapter, owned, deadline):
    active = []
    unresolved = []
    for component in owned:
        name = component_name(component)
        try:
            state = (adapter.hardware_state(name, deadline) if component == "hardware"
                     else adapter.controller_state(name, deadline))
            if state == "active":
                active.append(name)
            elif state != "inactive":
                unresolved.append(name)
        except Exception:
            unresolved.append(name)
    return active, unresolved


def recover_after_failure(adapter, owned):
    try:
        settled = adapter.settle_pending(time.monotonic() + STOP_CONFIRM_TIMEOUT)
    except Exception:
        settled = False
        settle_result = "ERROR"
    else:
        settle_result = "SETTLED" if settled else "PENDING"
    adapter.stop_deadline = time.monotonic() + STOP_CONFIRM_TIMEOUT
    try:
        stopped = adapter.confirmed_stop()
    except Exception:
        stopped = False
        stop_result = "ERROR"
    else:
        stop_result = "CONFIRMED" if stopped else "NOT_CONFIRMED"
    if not settled or not stopped:
        active, unknown = observed_active_components(
            adapter, ALL_COMPONENTS, time.monotonic() + STOP_CONFIRM_TIMEOUT)
        unresolved = list(dict.fromkeys(
            [component_name(item) for item in owned] + unknown))
        release_reply = stop_reply_category(getattr(adapter, "stop_release_reply", "NOT_OBSERVED"))
        bootstrap_reply = stop_reply_category(getattr(adapter, "stop_bootstrap_reply", "NOT_OBSERVED"))
        phase = getattr(adapter, "stop_phase", "NOT_STARTED")
        if phase not in STOP_PHASE_CATEGORIES:
            phase = "NOT_STARTED"
        raise UnconfirmedStopError(
            "UNCONFIRMED_STOP: pending lifecycle transition or guarded release not confirmed "
            f"(settle_pending={settle_result}, confirmed_stop={stop_result}, "
            f"release_reply={release_reply}, bootstrap_reply={bootstrap_reply}, "
            f"stop_phase={phase})",
            active, unresolved)

    adapter.deactivation_deadline = time.monotonic() + STOP_CONFIRM_TIMEOUT
    errors = []
    for component in reversed(owned):
        try:
            if component == "hardware":
                if adapter.hardware_state(HARDWARE_NAME, adapter.deactivation_deadline) == "active":
                    adapter.deactivate_hardware(HARDWARE_NAME)
            else:
                if adapter.controller_state(component, adapter.deactivation_deadline) == "active":
                    adapter.deactivate_controller(component)
        except Exception as error:
            errors.append(str(error))
    active, unknown = observed_active_components(adapter, ALL_COMPONENTS,
                                                 adapter.deactivation_deadline)
    if errors:
        raise UnconfirmedStopError(
            "UNCONFIRMED_STOP: guarded release confirmed but ROS deactivation failed: "
            + "; ".join(errors), active, unknown)
    if active or unknown:
        raise UnconfirmedStopError(
            "UNCONFIRMED_STOP: lifecycle components not confirmed inactive after release",
            active, unknown)
    return active, unknown


def complete_success(adapter, owned, report):
    active, unknown = recover_after_failure(adapter, owned)
    return {**report, "stop_confirmation": "CONFIRMED_RELEASED_IDLE",
            "still_active_components": active, "unresolved_components": unknown}


def cancel_release_context(adapter):
    return {"release_observation": {
                "request_reply": stop_reply_category(
                    getattr(adapter, "stop_release_reply", "NOT_OBSERVED")),
                "bootstrap_reply": stop_reply_category(
                    getattr(adapter, "stop_bootstrap_reply", "NOT_OBSERVED"))},
            "raw_stop_release_join": "REQUIRED_SEPARATELY",
            "safety_verdict": "NOT_QUALIFIED",
            "stop_confirmation": "UNCONFIRMED_STOP"}


def complete_cancel_observation(adapter, owned, report):
    observed = complete_success(adapter, owned, report)
    return {**observed, **cancel_release_context(adapter),
            "status": "UNCONFIRMED_STOP", "action_observation": "CANCELED"}


def run_guarded_sequence(adapter, owned, targets, perform_goal, per_goal_timeout=None):
    completed = []
    goal_ids = set()
    for index, target in enumerate(targets):
        try:
            if per_goal_timeout is not None:
                adapter.deadline = time.monotonic() + per_goal_timeout
            if index:
                adapter.renew_motion_permit()
            goal = perform_goal(target, index)
            goal_id = goal.get("goal_id") if isinstance(goal, dict) else None
            if (goal.get("status") != "SUCCEEDED" or not isinstance(goal_id, str) or
                    not re.fullmatch(r"[0-9a-f]{32}", goal_id) or goal_id in goal_ids):
                raise RuntimeError("unconfirmed or duplicate ROS goal result")
            goal_ids.add(goal_id)
            completed.append({**goal, "index": index})
        except BaseException as error:
            failed_report = ({**error.goal_report, "index": index}
                             if isinstance(error, GoalAttemptError) else None)
            try:
                active, unknown = recover_after_failure(adapter, owned)
            except UnconfirmedStopError as stop_error:
                raise UnconfirmedStopError(
                    f"{stop_error}; original failure: {error}",
                    stop_error.still_active_components, stop_error.unresolved_components,
                    completed, index, failed_report) from error
            except RuntimeError as cleanup_error:
                raise ConfirmedStopError(f"{error}; {cleanup_error}",
                                         completed_goals=completed,
                                         failed_goal_index=index,
                                         failed_goal_report=failed_report) from error
            except BaseException as cleanup_error:
                raise UnconfirmedStopError(
                    f"UNCONFIRMED_STOP: cleanup interrupted: {cleanup_error}; "
                    f"original failure: {error}", completed_goals=completed,
                    failed_goal_index=index, failed_goal_report=failed_report) from error
            raise ConfirmedStopError(str(error), active, completed, index,
                                     failed_report) from error
    try:
        active, unknown = recover_after_failure(adapter, owned)
    except UnconfirmedStopError as error:
        raise UnconfirmedStopError(str(error), error.still_active_components,
                                   error.unresolved_components, completed,
                                   len(targets)) from error
    except BaseException as error:
        raise UnconfirmedStopError(f"UNCONFIRMED_STOP: final release interrupted: {error}",
                                   completed_goals=completed,
                                   failed_goal_index=len(targets)) from error
    return {"scope": "ROS_ACTION_SESSION_ONLY", "status": "SUCCEEDED",
            "requested_goals": len(targets), "completed_goals": len(completed),
            "goals": completed, "stop_confirmation": "CONFIRMED_RELEASED_IDLE",
            "still_active_components": active, "unresolved_components": unknown,
            "device_can_evidence": "REQUIRED_SEPARATELY",
            "device_qualification": "NOT_CLAIMED"}


def verify_active_stack(adapter):
    if not adapter.confirmed_hold():
        raise RuntimeError("confirmed ENABLED_HOLD required before planning")
    if adapter.hardware_state(HARDWARE_NAME) != "active":
        raise RuntimeError(f"hardware is not active: {HARDWARE_NAME}")
    for name in CONTROLLERS:
        if adapter.controller_state(name) != "active":
            raise RuntimeError(f"controller is not active: {name}")


def activate_controller_stack(adapter, owned=None):
    if owned is None:
        owned = []
    if not adapter.confirmed_hold():
        raise RuntimeError("confirmed ENABLED_HOLD required before ROS activation")
    initial = adapter.hardware_state(HARDWARE_NAME)
    if initial not in ("inactive", "active"):
        raise RuntimeError(f"hardware state unavailable: {HARDWARE_NAME}")
    if initial == "inactive":
        owned.append("hardware")
        adapter.activate_hardware(HARDWARE_NAME)
    if adapter.hardware_state(HARDWARE_NAME) != "active":
        raise RuntimeError(f"hardware is not active: {HARDWARE_NAME}")
    for name in CONTROLLERS:
        initial = adapter.controller_state(name)
        if initial not in ("inactive", "active"):
            raise RuntimeError(f"controller state unavailable: {name}")
        if initial == "inactive":
            owned.append(name)
            adapter.activate_controller(name)
        if adapter.controller_state(name) != "active":
            raise RuntimeError(f"controller is not active: {name}")


def await_accepted_goal(node, handle, timeout):
    result_future = handle.get_result_async()
    try:
        return wait(node, result_future, timeout)
    except RuntimeError as error:
        cleanup_deadline = time.monotonic() + STOP_CONFIRM_TIMEOUT
        cancel_detail = "cancel request not confirmed"
        try:
            cancel = wait(node, handle.cancel_goal_async(), remaining_time(cleanup_deadline))
            cancel_detail = ("cancel accepted" if any(
                bytes(item.goal_id.uuid) == bytes(handle.goal_id.uuid)
                for item in cancel.goals_canceling)
                             else "cancel response did not accept goal")
        except Exception as cancel_error:
            cancel_detail = f"cancel request failed: {cancel_error}"

        try:
            result = wait(node, result_future, remaining_time(cleanup_deadline))
        except RuntimeError as result_error:
            raise UnconfirmedStopError(
                f"UNCONFIRMED_STOP: action result unavailable after timeout; "
                f"{cancel_detail}; {result_error}") from error
        if result.status not in TERMINAL_ACTION_STATUSES:
            raise UnconfirmedStopError(
                f"UNCONFIRMED_STOP: action status {result.status} is not terminal; "
                f"{cancel_detail}") from error
        raise RuntimeError(
            f"action result timed out; {cancel_detail}; terminal action status {result.status}; "
            "physical stop requires device evidence") from error


def ros_goal_uuid(goal_id):
    from unique_identifier_msgs.msg import UUID
    return UUID(uuid=list(bytes.fromhex(goal_id)))


def probe_overlap_goal(node, client, goal, deadline, goal_id):
    report = {"goal_id": goal_id, "phase": "A_MOTION_FEEDBACK_OVERLAP",
              "second_begin_raw_join": "REQUIRED_SEPARATELY"}
    if time.monotonic() >= deadline:
        return {**report, "status": "NOT_SUBMITTED", "phase": "BUDGET_EXHAUSTED",
                "reason": "B probe budget unavailable"}
    handle = None
    try:
        admission_timeout = remaining_time(deadline)
        handle = wait(node, client.send_goal_async(
            goal, goal_uuid=ros_goal_uuid(goal_id)), admission_timeout)
        identity_matches = bytes(handle.goal_id.uuid).hex() == goal_id
        if not identity_matches and handle.accepted:
            cleanup = cancel_accepted_on_interrupt(node, handle, stop_deadline=deadline)
            return {**report, "status": "ADMISSION_IDENTITY_MISMATCH", "cleanup": cleanup}
        if not identity_matches:
            return {**report, "status": "ADMISSION_IDENTITY_MISMATCH"}
        if not handle.accepted:
            return {**report, "status": "REJECTED"}
        cleanup = cancel_accepted_on_interrupt(node, handle, stop_deadline=deadline)
        return {**report, "status": "ACCEPTED_UNEXPECTED", "cleanup": cleanup}
    except BaseException as error:
        uncertain = {**report, "status": "ADMISSION_UNCONFIRMED", "reason": str(error)}
        if isinstance(error, (KeyboardInterrupt, SystemExit)):
            uncertain["interrupted"] = True
        if handle is not None and handle.accepted:
            try:
                uncertain["cleanup"] = cancel_accepted_on_interrupt(
                    node, handle, stop_deadline=deadline)
            except BaseException as cleanup_error:
                uncertain["cleanup_error"] = str(cleanup_error)
        return uncertain


def await_mid_motion_cancel(node, handle, feedback, motion_delta_rad, deadline,
                            accepted_feedback_index=0, overlap_probe=None,
                            overlap_goal_id=None):
    goal_id = bytes(handle.goal_id.uuid).hex()
    try:
        result_future = handle.get_result_async()
    except BaseException as error:
        cleanup = cancel_accepted_on_interrupt(node, handle)
        raise CancelAttemptError("UNCONFIRMED_STOP: action result future unavailable",
                                 cancel={"goal_id": goal_id, **cleanup}) from error
    origin = None
    while True:
        for index, sample in enumerate(feedback[accepted_feedback_index:],
                                       accepted_feedback_index):
            actual = sample.get("actual_rad")
            if (sample.get("goal_id") != goal_id or not isinstance(actual, list) or
                    len(actual) != 6 or any(type(value) not in (int, float) or
                                            not math.isfinite(value) for value in actual)):
                continue
            if origin is None:
                origin = actual
            elif max(abs(a - b) for a, b in zip(actual, origin)) >= motion_delta_rad:
                break
        else:
            if result_future.done():
                raise CancelAttemptError("action finished before observed mid-motion cancel",
                                         result_future.result())
            try:
                timeout = min(0.05, remaining_time(deadline))
            except RuntimeError as error:
                raise CancelAttemptError("no observed motion before cancel deadline") from error
            import rclpy
            rclpy.spin_once(node, timeout_sec=timeout)
            continue
        break

    overlap = None
    if overlap_probe is not None:
        try:
            overlap = overlap_probe()
        except BaseException as error:
            overlap = {"goal_id": overlap_goal_id, "status": "ADMISSION_UNCONFIRMED",
                       "reason": str(error)}
            if isinstance(error, (KeyboardInterrupt, SystemExit)):
                overlap["interrupted"] = True
    cancel = {"goal_id": goal_id, "requested": True, "acknowledged": False,
              "motion_delta_rad": motion_delta_rad,
              "motion_feedback_start_index": accepted_feedback_index,
              "motion_feedback_trigger_index": index,
              **({"overlap_probe": overlap} if overlap_probe is not None else {})}
    try:
        cancel_reply = wait(node, handle.cancel_goal_async(), remaining_time(deadline))
        cancel["acknowledged"] = any(
            bytes(item.goal_id.uuid) == bytes(handle.goal_id.uuid)
            for item in cancel_reply.goals_canceling)
    except (RuntimeError, AttributeError, TypeError, ValueError) as error:
        cancel["reply_error"] = str(error)
    try:
        result = wait(node, result_future, remaining_time(deadline))
    except RuntimeError as error:
        raise CancelAttemptError("UNCONFIRMED_STOP: canceled action has no terminal result",
                                 cancel=cancel) from error
    if result.status not in TERMINAL_ACTION_STATUSES:
        raise CancelAttemptError("UNCONFIRMED_STOP: canceled action result is not terminal",
                                 result, cancel)
    if not cancel["acknowledged"] or result.status != 5:
        raise CancelAttemptError("cancel reply or CANCELED action result not confirmed",
                                 result, cancel)
    if overlap is not None and overlap.get("interrupted"):
        raise CancelAttemptError("UNCONFIRMED_STOP: overlap probe interrupted",
                                 result, cancel)
    return result, cancel


def cancel_accepted_on_interrupt(node, handle, result_future=None, stop_deadline=None):
    deadline = time.monotonic() + STOP_CONFIRM_TIMEOUT
    if stop_deadline is not None:
        deadline = min(deadline, stop_deadline)
    goal_id = bytes(handle.goal_id.uuid).hex()
    observation = {"goal_id": goal_id, "cancel_requested": False,
                   "cancel_acknowledged": False, "action_result_confirmed": False}
    if result_future is None:
        try:
            result_future = handle.get_result_async()
        except BaseException as error:
            observation["result_error"] = str(error)
    try:
        observation["cancel_requested"] = True
        reply = wait(node, handle.cancel_goal_async(), remaining_time(deadline))
        observation["cancel_acknowledged"] = any(
            bytes(item.goal_id.uuid) == bytes(handle.goal_id.uuid)
            for item in reply.goals_canceling)
    except BaseException as error:
        observation["cancel_error"] = str(error)
    if result_future is not None:
        try:
            result = wait(node, result_future, remaining_time(deadline))
            observation["action_status"] = result.status
            observation["result_error_code"] = result.result.error_code
            observation["result_error_string"] = result.result.error_string
            observation["action_result_confirmed"] = result.status in TERMINAL_ACTION_STATUSES
        except BaseException as error:
            observation["result_error"] = str(error)
    return observation


def wait_for_admission_ready(adapter, deadline):
    remaining_time(deadline)
    status = adapter.call("/device_action_gate/bootstrap_status", adapter.Trigger,
                          adapter.Trigger.Request(), deadline=deadline)
    if status.success and status.message == "ENABLED_HOLD":
        return True
    if status.success or status.message not in ADMISSION_PENDING_STATUSES:
        return False
    while True:
        time.sleep(min(0.05, remaining_time(deadline)))
        remaining_time(deadline)
        status = adapter.call("/device_action_gate/bootstrap_status", adapter.Trigger,
                              adapter.Trigger.Request(), deadline=deadline)
        if status.success and status.message == "ENABLED_HOLD":
            return True
        if status.success or status.message not in ADMISSION_PENDING_STATUSES:
            return False


def has_matching_renewal_receipt(adapter, rejected_goal_id, deadline):
    try:
        goal_uuid = bytes(rejected_goal_id.uuid)
        if len(goal_uuid) != 16:
            return False
        response = adapter.call("/device_action_gate/last_goal_rejection", adapter.Trigger,
                                adapter.Trigger.Request(), deadline=deadline)
    except Exception:
        return False
    message = getattr(response, "message", None)
    return (getattr(response, "success", None) is True and
            isinstance(message, str) and
            re.fullmatch(r"RENEWAL_PENDING:[0-9a-f]{32}", message) is not None and
            message.removeprefix("RENEWAL_PENDING:") == goal_uuid.hex())


def execute_goal(node, planner, client, target, limits, deadline, admission_adapter=None,
                 cancel_mid_motion=False, preempt_probe=False, preempt_goal_ids=None,
                 goal_uuid_hex=None, submission_deadline=None):
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.serialization import serialize_message

    request = GetMotionPlan.Request()
    motion = MotionPlanRequest()
    motion.start_state.is_diff = True
    motion.group_name = "dummy_arm"
    motion.num_planning_attempts = 2
    motion.allowed_planning_time = 5.0
    motion.max_velocity_scaling_factor = 0.1
    motion.max_acceleration_scaling_factor = 0.1
    motion.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=name, position=value, tolerance_above=0.001,
                        tolerance_below=0.001, weight=1.0)
        for name, value in zip(JOINTS, target)])]
    request.motion_plan_request = motion
    admission_deadline = min(deadline, submission_deadline) if submission_deadline else deadline
    response = wait(node, planner.call_async(request), remaining_time(admission_deadline))
    if response.motion_plan_response.error_code.val != 1:
        raise RuntimeError(f"MoveIt planning failed: {response.motion_plan_response.error_code.val}")
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    validate_plan(trajectory, target, limits)
    if len(trajectory.points) > MAX_DIAGNOSTIC_POINTS:
        raise RuntimeError("MoveIt plan exceeds bounded diagnostic point count")
    serialized = serialize_message(trajectory)
    if len(serialized) > MAX_SERIALIZED_TRAJECTORY_BYTES:
        raise RuntimeError("MoveIt plan exceeds bounded diagnostic byte count")
    planned_digest = trajectory_digest(trajectory)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    feedback = []
    feedback_errors = []
    started = time.monotonic_ns()

    def on_feedback(message):
        sample = message.feedback
        if (tuple(sample.joint_names) != JOINTS or len(sample.actual.positions) != 6 or
                len(sample.desired.positions) != 6):
            if not feedback_errors:
                feedback_errors.append("ROS feedback missing six axes")
            return
        if len(feedback) >= MAX_FEEDBACK_SAMPLES:
            if not feedback_errors:
                feedback_errors.append("ROS feedback exceeds bounded sample count")
            return
        if (cancel_mid_motion or preempt_probe) and any(
                type(value) not in (int, float) or not math.isfinite(value)
                for value in (*sample.actual.positions, *sample.desired.positions)):
            if not feedback_errors:
                feedback_errors.append("ROS feedback has non-finite positions")
            return
        feedback_goal_id = None
        try:
            uuid = bytes(message.goal_id.uuid)
            if len(uuid) == 16:
                feedback_goal_id = uuid.hex()
        except (AttributeError, TypeError, ValueError):
            pass
        feedback.append({"elapsed_us": (time.monotonic_ns() - started) // 1000,
                         "actual_rad": list(sample.actual.positions),
                         "desired_rad": list(sample.desired.positions),
                         **({"goal_id": feedback_goal_id}
                            if cancel_mid_motion or preempt_probe else {})})

    goal_id = None
    result = None
    cancel_report = None
    accepted_feedback_index = None
    handle = None
    probe_ids = preempt_goal_ids or {name: uuid.uuid4().hex for name in "ABC"} \
        if preempt_probe else None

    def probe_goals():
        if probe_ids is None:
            return None
        overlap = cancel_report.get("overlap_probe") if isinstance(cancel_report, dict) else None
        return {"A": {"goal_id": probe_ids["A"],
                       "status": "CANCELED" if result is not None and result.status == 5 else
                                 "UNCONFIRMED", "phase": "ACTION_TERMINAL_RAW_PENDING"},
                "B": overlap or {"goal_id": probe_ids["B"], "status": "NOT_SUBMITTED",
                                 "phase": "AWAIT_A_MOTION"},
                "C": {"goal_id": probe_ids["C"], "status": "NOT_SUBMITTED",
                      "phase": "WAITING_FOR_CANCEL_RAW_AND_NEW_PERMIT"}}

    try:
        for attempt in range(MAX_ADMISSION_RETRIES + 1):
            started = time.monotonic_ns()
            reserved_id = probe_ids["A"] if preempt_probe else goal_uuid_hex
            send_options = ({"goal_uuid": ros_goal_uuid(reserved_id)}
                            if reserved_id is not None else {})
            handle = wait(node, client.send_goal_async(
                goal, feedback_callback=on_feedback, **send_options),
                          remaining_time(admission_deadline))
            if handle.accepted:
                goal_id = bytes(handle.goal_id.uuid).hex()
                if reserved_id is not None and goal_id != reserved_id:
                    cancel_accepted_on_interrupt(node, handle)
                    raise RuntimeError("reserved goal UUID mismatch")
                accepted_feedback_index = len(feedback)
                break
            if (admission_adapter is None or feedback or feedback_errors or
                    attempt == MAX_ADMISSION_RETRIES or preempt_probe or
                    not has_matching_renewal_receipt(
                        admission_adapter, handle.goal_id, admission_deadline) or
                    not wait_for_admission_ready(admission_adapter, admission_deadline)):
                raise RejectedPlanError(rejection_diagnostic(trajectory, serialized))
        if cancel_mid_motion or preempt_probe:
            overlap_probe = (lambda: probe_overlap_goal(
                node, client, goal,
                min(deadline - STOP_CONFIRM_TIMEOUT,
                    time.monotonic() + PREEMPT_B_PROBE_TIMEOUT),
                probe_ids["B"])) if preempt_probe else None
            result, cancel_report = await_mid_motion_cancel(
                node, handle, feedback, MID_MOTION_DELTA_RAD, deadline,
                accepted_feedback_index,
                **({"overlap_probe": overlap_probe, "overlap_goal_id": probe_ids["B"]}
                   if preempt_probe else {}))
        else:
            result = await_accepted_goal(node, handle, remaining_time(deadline))
        if feedback_errors or not feedback:
            raise RuntimeError(feedback_errors[0] if feedback_errors else "no six-axis ROS feedback")
        if not (cancel_mid_motion or preempt_probe) and (result.status != GoalStatus.STATUS_SUCCEEDED or
                                      result.result.error_code != 0):
            raise RuntimeError(f"guarded action failed: status={result.status} "
                               f"code={result.result.error_code}")
    except BaseException as error:
        if isinstance(error, CancelAttemptError):
            result = error.result
            cancel_report = error.cancel
        interrupted = None
        if goal_id is not None:
            if isinstance(error, (KeyboardInterrupt, SystemExit)):
                interrupted = cancel_accepted_on_interrupt(node, handle)
            elif goal_uuid_hex is not None and result is None:
                interrupted = cancel_accepted_on_interrupt(
                    node, handle, stop_deadline=deadline)
        failed = {"scope": "ROS_ACTION_ONLY",
                  "target_rad": list(target), "trajectory_sha256": planned_digest,
                  "action_started_steady_us": started // 1000,
                  "status": "FAILED", "reason": str(error), "feedback": feedback,
                  "device_can_evidence": "REQUIRED_SEPARATELY",
                  "device_qualification": "NOT_CLAIMED"}
        if goal_id is not None:
            failed["goal_id"] = goal_id
        if result is not None:
            failed.update(action_status=result.status,
                          result_error_code=result.result.error_code,
                          result_error_string=result.result.error_string)
        if cancel_mid_motion or preempt_probe:
            failed.update(cancel=cancel_report,
                          device_stop_evidence="REQUIRED_SEPARATELY",
                          device_release_evidence="REQUIRED_SEPARATELY")
        if preempt_probe:
            failed.update(preempt_goals=probe_goals())
        if interrupted is not None:
            failed.update(interrupted)
        raise GoalAttemptError(str(error), failed) from error
    return {"scope": "ROS_ACTION_ONLY", "goal_id": goal_id,
            "joint_names": list(JOINTS), "target_rad": list(target),
            "trajectory_sha256": planned_digest,
            "action_started_steady_us": started // 1000,
            "planned_points": len(trajectory.points), "result_error_code": result.result.error_code,
            "result_error_string": result.result.error_string,
            "action_status": result.status,
            "status": "CANCELED" if cancel_mid_motion or preempt_probe else "SUCCEEDED",
            "feedback": feedback,
            "device_can_evidence": "REQUIRED_SEPARATELY",
            "device_qualification": "NOT_CLAIMED",
            **({"cancel": cancel_report, "device_stop_evidence": "REQUIRED_SEPARATELY",
                "device_release_evidence": "REQUIRED_SEPARATELY"}
               if cancel_mid_motion or preempt_probe else {}),
            **({"preempt_goals": probe_goals()} if preempt_probe else {})}


def capture(target, timeout, limits, activate_controller=False, activation_sha256=None,
            device_id=None, candidate_manifest=None, guarded_session=False,
            cancel_mid_motion=False, preempt_probe=False, preempt_goal_ids=None,
            preempt_then_goal=False, post_cancel_target=None):
    preempt_mode = preempt_probe or preempt_then_goal
    cancel_mode = cancel_mid_motion or preempt_mode
    if cancel_mode and not guarded_session:
        raise ValueError("mid-motion cancel requires guarded session")
    if cancel_mode and (not math.isfinite(timeout) or
                              not 0 < timeout <= MAX_MULTI_GOAL_TIMEOUT_S):
        raise ValueError("mid-motion cancel requires bounded timeout <= 120s")
    if sum((cancel_mid_motion, preempt_probe, preempt_then_goal)) > 1:
        raise ValueError("cancel and preempt probes are distinct modes")
    if preempt_then_goal:
        if post_cancel_target is None:
            raise ValueError("post-cancel six-axis target required")
        post_cancel_target = validate_target(post_cancel_target, limits)
        if (not isinstance(preempt_goal_ids, dict) or set(preempt_goal_ids) != set("ABC") or
                any(not isinstance(value, str) or
                    re.fullmatch(r"[0-9a-f]{32}", value) is None
                    for value in preempt_goal_ids.values()) or
                len(set(preempt_goal_ids.values())) != 3):
            raise ValueError("three distinct reserved preempt goal UUIDs required")
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node("ros_six_axis_movej_capture")
    deadline = time.monotonic() + timeout
    adapter = None
    owned = []
    bound = False
    release_attempted = False
    report = None
    try:
        if guarded_session or activate_controller:
            adapter = RosControlAdapter(node, deadline)
            verify_live_gate(adapter, activation_sha256, device_id, candidate_manifest)
            bound = True
            if activate_controller:
                activate_controller_stack(adapter, owned)
            else:
                verify_active_stack(adapter)
        planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        if not planner.wait_for_service(timeout_sec=remaining_time(deadline)):
            raise RuntimeError("MoveIt planning service unavailable")
        client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=remaining_time(deadline)):
            raise RuntimeError("public guarded action unavailable")
        report = execute_goal(node, planner, client, target, limits, deadline, adapter,
                              **({"preempt_probe": True,
                                  "preempt_goal_ids": preempt_goal_ids}
                                 if preempt_mode else
                                 {"cancel_mid_motion": True} if cancel_mid_motion else {}))
        if preempt_then_goal:
            observed = report.get("preempt_goals", {})
            if (report.get("goal_id") != preempt_goal_ids["A"] or
                    report.get("status") != "CANCELED" or
                    report.get("action_status") != 5 or
                    report.get("result_error_code") != 0 or
                    observed.get("A", {}).get("status") != "CANCELED" or
                    observed.get("B", {}).get("goal_id") != preempt_goal_ids["B"] or
                    observed.get("B", {}).get("status") != "REJECTED"):
                raise RuntimeError("A cancel or B overlap rejection unconfirmed")
            post_cancel_submission_deadline = min(
                deadline, time.monotonic() + POST_CANCEL_SUBMISSION_TIMEOUT)
            adapter.deadline = post_cancel_submission_deadline
            remaining_time(post_cancel_submission_deadline)
            adapter.renew_motion_permit()
            remaining_time(post_cancel_submission_deadline)
            try:
                second = execute_goal(
                    node, planner, client, post_cancel_target, limits, deadline, adapter,
                    goal_uuid_hex=preempt_goal_ids["C"],
                    submission_deadline=post_cancel_submission_deadline)
            except GoalAttemptError as error:
                failed_c = error.goal_report
                error.goal_report = {**failed_c,
                                     "preempt_goals": {**observed, "C": failed_c},
                                     "goals": [{key: value for key, value in report.items()
                                                if key != "preempt_goals"} | {"index": 0},
                                               {**failed_c, "index": 1}]}
                raise
            if (second.get("goal_id") != preempt_goal_ids["C"] or
                    second.get("status") != "SUCCEEDED" or
                    second.get("action_status") != 4 or
                    second.get("result_error_code") != 0):
                raise RuntimeError("post-cancel goal C result unconfirmed")
            report = {"scope": "ROS_PREEMPT_RECOVERY", "status": "SUCCEEDED",
                      "requested_goals": 2, "completed_goals": 2,
                      "goals": [{key: value for key, value in report.items()
                                 if key != "preempt_goals"} | {"index": 0},
                                {**second, "index": 1}],
                      "preempt_goals": {**observed, "C": {**second, "status": "SUCCEEDED"}},
                      "second_begin_raw_join": "REQUIRED_SEPARATELY",
                      "qualification": "NOT_QUALIFIED"}
        if adapter is not None and bound:
            try:
                release_attempted = True
                completed = (complete_cancel_observation(adapter, owned, report)
                             if cancel_mid_motion or preempt_probe else
                             complete_success(adapter, owned, report))
            except UnconfirmedStopError as error:
                if not cancel_mode:
                    raise
                raise UnconfirmedStopError(
                    str(error), error.still_active_components,
                    error.unresolved_components,
                    failed_goal_report={**report, **cancel_release_context(adapter)}) from error
            except BaseException as error:
                if not cancel_mode:
                    raise
                raise UnconfirmedStopError(
                    f"UNCONFIRMED_STOP: release interrupted: {error or type(error).__name__}",
                    failed_goal_report={**report, **cancel_release_context(adapter)}) from error
            if preempt_probe:
                return {**completed, "scope": "ROS_PREEMPT_PROBE",
                        "goals": report["preempt_goals"],
                        "second_begin_raw_join": "REQUIRED_SEPARATELY",
                        "qualification": "NOT_QUALIFIED"}
            return completed
        return {**report, "stop_confirmation": "NOT_REQUESTED"}
    except BaseException as error:
        if adapter is not None and bound and not release_attempted:
            original_report = getattr(error, "goal_report", None) or report
            try:
                active, unknown = recover_after_failure(adapter, owned)
            except UnconfirmedStopError as stop_error:
                failed = original_report
                if failed is not None and cancel_mode:
                    failed = {**failed, **cancel_release_context(adapter)}
                raise UnconfirmedStopError(
                    f"{stop_error}; original failure: {error}",
                    stop_error.still_active_components,
                    stop_error.unresolved_components,
                    failed_goal_report=failed) from error
            except RuntimeError as cleanup_error:
                failed = original_report
                if cancel_mode:
                    raise UnconfirmedStopError(
                        f"UNCONFIRMED_STOP: {error}; {cleanup_error}",
                        failed_goal_report=failed) from error
                raise ConfirmedStopError(f"{error}; {cleanup_error}",
                                         failed_goal_report=failed) from error
            except BaseException as cleanup_error:
                raise UnconfirmedStopError(
                    f"UNCONFIRMED_STOP: cleanup interrupted: {cleanup_error}; "
                    f"original failure: {error}",
                    failed_goal_report=original_report) from error
            failed = original_report
            if cancel_mode:
                if failed is not None:
                    failed = {**failed, **cancel_release_context(adapter)}
                raise UnconfirmedStopError(f"UNCONFIRMED_STOP: {error}", active, unknown,
                                           failed_goal_report=failed) from error
            raise ConfirmedStopError(str(error), active,
                                     failed_goal_report=failed) from error
        raise
    finally:
        prior_error = sys.exc_info()[1]
        teardown_errors = []
        for cleanup in (node.destroy_node, rclpy.shutdown):
            try:
                cleanup()
            except BaseException as error:
                teardown_errors.append(error)
        if teardown_errors:
            detail = "; ".join(str(error) or type(error).__name__
                               for error in teardown_errors)
            if isinstance(prior_error, UnconfirmedStopError):
                if prior_error.failed_goal_report is None and report is not None:
                    prior_error.failed_goal_report = {
                        **report, **cancel_release_context(adapter)}
                prior_error.add_note(f"ROS teardown failed: {detail}")
            elif cancel_mode:
                original = (getattr(prior_error, "failed_goal_report", None) or
                            getattr(prior_error, "goal_report", None) or report)
                if original is not None:
                    original = {**original, **cancel_release_context(adapter)}
                raise UnconfirmedStopError(
                    f"UNCONFIRMED_STOP: ROS teardown failed: {detail}",
                    failed_goal_report=original) from teardown_errors[0]
            else:
                raise teardown_errors[0]


def capture_many(targets, timeout, limits, activate_controller=False,
                 activation_sha256=None, device_id=None, candidate_manifest=None):
    if (not math.isfinite(timeout) or not 0 < timeout <= MAX_MULTI_GOAL_TIMEOUT_S or
            not 1 <= len(targets) <= MAX_MULTI_GOALS):
        raise ValueError("multi-goal count or per-goal timeout outside bounds")
    targets = tuple(validate_target(target, limits) for target in targets)
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node("ros_six_axis_movej_capture")
    adapter = None
    owned = []
    bound = False
    sequence_started = False
    try:
        adapter = RosControlAdapter(node, time.monotonic() + timeout)
        verify_live_gate(adapter, activation_sha256, device_id, candidate_manifest)
        bound = True
        if activate_controller:
            activate_controller_stack(adapter, owned)
        else:
            verify_active_stack(adapter)
        planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        if not planner.wait_for_service(timeout_sec=remaining_time(adapter.deadline)):
            raise RuntimeError("MoveIt planning service unavailable")
        client = ActionClient(node, FollowJointTrajectory,
                              "/arm_controller/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=remaining_time(adapter.deadline)):
            raise RuntimeError("public guarded action unavailable")
        sequence_started = True
        return run_guarded_sequence(adapter, owned, targets,
            lambda target, index: execute_goal(node, planner, client, target, limits,
                                               adapter.deadline, adapter), timeout)
    except BaseException as error:
        if adapter is not None and bound and not sequence_started:
            try:
                active, unknown = recover_after_failure(adapter, owned)
            except UnconfirmedStopError as stop_error:
                raise UnconfirmedStopError(
                    f"{stop_error}; original failure: {error}",
                    stop_error.still_active_components,
                    stop_error.unresolved_components) from error
            except RuntimeError as cleanup_error:
                raise ConfirmedStopError(f"{error}; {cleanup_error}") from error
            except BaseException as cleanup_error:
                raise UnconfirmedStopError(
                    f"UNCONFIRMED_STOP: cleanup interrupted: {cleanup_error}; "
                    f"original failure: {error}") from error
            raise ConfirmedStopError(str(error), active) from error
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="采集真实六轴 MoveIt→ROS action 记录")
    target_input = parser.add_mutually_exclusive_group(required=True)
    target_input.add_argument("--target", nargs=6, type=float, metavar="RAD")
    target_input.add_argument("--targets-json", type=Path)
    parser.add_argument("--reference-frame", choices=("ABSOLUTE_MODEL", "BENCH_SESSION"),
                        required=True)
    parser.add_argument("--candidate-manifest", type=Path)
    parser.add_argument("--ros-limits-json", type=Path)
    parser.add_argument("--activate-controller", action="store_true")
    parser.add_argument("--activation-sha256")
    parser.add_argument("--device-id")
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--cancel-mid-motion", action="store_true")
    parser.add_argument("--preempt-probe", action="store_true")
    parser.add_argument("--preempt-then-goal", action="store_true")
    parser.add_argument("--post-cancel-target", nargs=6, type=float, metavar="RAD")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be positive")
    bench = args.reference_frame == "BENCH_SESSION"
    multi = args.targets_json is not None
    if multi and (not bench or args.timeout > MAX_MULTI_GOAL_TIMEOUT_S):
        parser.error("bounded multi-goal mode requires BENCH_SESSION and timeout <= 120s")
    if args.cancel_mid_motion and (not bench or multi or args.timeout > MAX_MULTI_GOAL_TIMEOUT_S):
        parser.error("mid-motion cancel requires single BENCH_SESSION goal and timeout <= 120s")
    if args.preempt_probe and (not bench or multi or args.cancel_mid_motion or
                               args.preempt_then_goal or
                               args.timeout > MAX_MULTI_GOAL_TIMEOUT_S):
        parser.error("preempt probe requires single BENCH_SESSION goal and timeout <= 120s")
    if args.preempt_then_goal and (not bench or multi or args.cancel_mid_motion or
                                   args.post_cancel_target is None or
                                   args.timeout > MAX_MULTI_GOAL_TIMEOUT_S):
        parser.error("post-cancel goal requires single bounded BENCH_SESSION goal and target")
    if args.post_cancel_target is not None and not args.preempt_then_goal:
        parser.error("--post-cancel-target requires --preempt-then-goal")
    if bench != bool(args.candidate_manifest) or bench != bool(args.ros_limits_json):
        parser.error("BENCH_SESSION requires candidate manifest and ROS limits JSON")
    if args.activate_controller and not bench:
        parser.error("--activate-controller requires BENCH_SESSION and verified limits")
    if bench and (
            not isinstance(args.activation_sha256, str) or
            not re.fullmatch(r"[0-9a-f]{64}", args.activation_sha256) or
            args.activation_sha256 == "0" * 64 or
            not isinstance(args.device_id, str) or
            not re.fullmatch(r"[0-9a-f]{32}", args.device_id) or
            args.device_id == "0" * 32):
        parser.error("BENCH_SESSION requires pinned activation SHA-256 and device ID")
    targets = ()
    preempt_goal_ids = ({name: uuid.uuid4().hex for name in "ABC"}
                        if args.preempt_probe or args.preempt_then_goal else None)
    try:
        limits = (load_bench_limits(args.ros_limits_json, args.candidate_manifest)
                  if bench else LIMITS)
        if multi:
            targets = load_targets_json(args.targets_json, limits)
            report = capture_many(targets, args.timeout, limits,
                                  activate_controller=args.activate_controller,
                                  activation_sha256=args.activation_sha256,
                                  device_id=args.device_id,
                                  candidate_manifest=args.candidate_manifest)
        else:
            target = validate_target(args.target, limits)
            post_cancel_target = (validate_target(args.post_cancel_target, limits)
                                  if args.preempt_then_goal else None)
            report = (capture(target, args.timeout, limits, guarded_session=True,
                              activate_controller=args.activate_controller,
                              activation_sha256=args.activation_sha256,
                              device_id=args.device_id,
                              candidate_manifest=args.candidate_manifest,
                              **({"preempt_then_goal": True,
                                  "post_cancel_target": post_cancel_target,
                                  "preempt_goal_ids": preempt_goal_ids}
                                 if args.preempt_then_goal else
                                 {"preempt_probe": True,
                                  "preempt_goal_ids": preempt_goal_ids}
                                 if args.preempt_probe else
                                 {"cancel_mid_motion": True} if args.cancel_mid_motion else {}))
                      if bench else capture(target, args.timeout, limits))
    except (RuntimeError, ValueError, KeyboardInterrupt, SystemExit) as error:
        status = "UNCONFIRMED_STOP" if isinstance(error, UnconfirmedStopError) else "FAIL"
        report = {"scope": "ROS_ACTION_SESSION_ONLY" if multi else "ROS_ACTION_ONLY",
                  "status": status, "reason": str(error) or type(error).__name__,
                  "stop_confirmation": ("UNCONFIRMED_STOP" if isinstance(error, UnconfirmedStopError)
                                        else "CONFIRMED_RELEASED_IDLE" if isinstance(error, ConfirmedStopError)
                                        else "NOT_REQUESTED"),
                  "still_active_components": (error.still_active_components
                                              if isinstance(error, (UnconfirmedStopError,
                                                                    ConfirmedStopError)) else []),
                  "unresolved_components": (error.unresolved_components
                                            if isinstance(error, UnconfirmedStopError) else []),
                  "device_can_evidence": "REQUIRED_SEPARATELY",
                  "device_qualification": "NOT_CLAIMED"}
        if multi:
            completed = list(getattr(error, "completed_goals", ()))
            failed = getattr(error, "failed_goal_report", None)
            report.update(requested_goals=len(targets), completed_goals=len(completed),
                          goals=completed + ([failed] if failed is not None else []),
                          failed_goal_index=getattr(error, "failed_goal_index", None))
        elif getattr(error, "failed_goal_report", None) is not None:
            report["goal"] = error.failed_goal_report
        if args.cancel_mid_motion or args.preempt_probe or args.preempt_then_goal:
            report.update(raw_stop_release_join="REQUIRED_SEPARATELY",
                          safety_verdict="NOT_QUALIFIED")
            if isinstance(report.get("goal"), dict) and "release_observation" in report["goal"]:
                report["release_observation"] = report["goal"]["release_observation"]
        diagnostic = rejection_diagnostic_from_error(error)
        if diagnostic is not None:
            report["rejected_plan"] = diagnostic
    if args.preempt_probe:
        observed_goals = report.get("goals")
        if not isinstance(observed_goals, dict):
            failed = report.get("goal")
            observed_goals = failed.get("preempt_goals") if isinstance(failed, dict) else None
        report.update(scope="ROS_PREEMPT_PROBE", status="UNCONFIRMED_STOP",
                      stop_confirmation="UNCONFIRMED_STOP", qualification="NOT_QUALIFIED",
                      goals=observed_goals or {
                          name: {"goal_id": goal_id, "status": "NOT_SUBMITTED",
                                 "phase": "SETUP_UNCONFIRMED"}
                          for name, goal_id in preempt_goal_ids.items()},
                      second_begin_raw_join="REQUIRED_SEPARATELY",
                      raw_stop_release_join="REQUIRED_SEPARATELY")
    if args.preempt_then_goal and report.get("status") != "SUCCEEDED":
        failed = report.get("goal")
        observed = failed.get("preempt_goals") if isinstance(failed, dict) else None
        completed = failed.get("goals", []) if isinstance(failed, dict) else []
        report.update(scope="ROS_PREEMPT_RECOVERY", qualification="NOT_QUALIFIED",
                      second_begin_raw_join="REQUIRED_SEPARATELY",
                      requested_goals=2,
                      completed_goals=sum(goal.get("status") in ("CANCELED", "SUCCEEDED")
                                          for goal in completed),
                      goals=completed,
                      preempt_goals=observed or {
                          name: {"goal_id": goal_id, "status": "NOT_SUBMITTED",
                                 "phase": "SETUP_UNCONFIRMED"}
                          for name, goal_id in preempt_goal_ids.items()})
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    printed = report
    if "rejected_plan" in report:
        diagnostic = report["rejected_plan"]
        printed = {key: value for key, value in report.items() if key != "rejected_plan"}
        printed["rejected_plan"] = {
            "report_path": str(args.report),
            "trajectory_sha256": diagnostic["trajectory_sha256"],
            "point_count": len(diagnostic["points"])}
    print(json.dumps(printed))
    return 0 if report.get("status") == "SUCCEEDED" and not (
        args.cancel_mid_motion or args.preempt_probe) else 1


if __name__ == "__main__":
    raise SystemExit(main())
