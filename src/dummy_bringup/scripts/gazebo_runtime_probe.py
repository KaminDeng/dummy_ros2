#!/usr/bin/python3
"""Collect one R02 observation from an already-running canonical Gazebo launch."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import time
from typing import Any

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [f"Joint{index}" for index in range(1, 7)]
PUBLIC_ACTION = "/arm_controller/follow_joint_trajectory"
LOCKED_CONTROLLER = "_internal_arm_controller"
LOCKED_CONTROLLER_TYPE = "joint_trajectory_controller/JointTrajectoryController"


class RuntimeUnavailable(RuntimeError):
    """The canonical Gazebo launch is not available to observe."""


class ObservationNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_r02_gazebo_runtime_probe")
        self.started_at = time.monotonic()
        self.joint_samples: list[dict[str, Any]] = []
        self.clock_samples: list[float] = []
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self.create_subscription(Clock, "/clock", self._on_clock, 50)

    def _on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in JOINTS):
            return
        self.joint_samples.append({
            "time_sec": time.monotonic() - self.started_at,
            "joint1_rad": positions["Joint1"],
            "joint_positions": {name: positions[name] for name in JOINTS},
        })

    def _on_clock(self, message: Clock) -> None:
        stamp = message.clock
        self.clock_samples.append(stamp.sec + stamp.nanosec / 1_000_000_000)


def _spin(node: Node, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))


def _wait_for_clock_quiescence(node: ObservationNode, timeout: float = 5.0) -> tuple[float, int]:
    """Discard pre-pause transport backlog before measuring frozen simulation time."""
    if not node.clock_samples:
        raise RuntimeUnavailable("/clock has no sample after Gazebo pause")
    latest = node.clock_samples[-1]
    last_advance = time.monotonic()
    settled_start = len(node.clock_samples)
    deadline = last_advance + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.clock_samples and node.clock_samples[-1] > latest:
            latest = node.clock_samples[-1]
            last_advance = time.monotonic()
            settled_start = len(node.clock_samples)
        if time.monotonic() - last_advance >= 0.3:
            return latest, settled_start
    raise RuntimeUnavailable("/clock kept advancing after Gazebo pause")


def _wait(node: Node, future, timeout: float = 12.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeUnavailable("ROS request did not complete against the canonical Gazebo launch")
    return future.result()


def _require_joint_samples(node: ObservationNode, minimum: int = 2) -> list[dict[str, Any]]:
    deadline = time.monotonic() + 10.0
    while len(node.joint_samples) < minimum and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(node.joint_samples) < minimum:
        raise RuntimeUnavailable("/joint_states from the canonical Gazebo launch is unavailable")
    return node.joint_samples


def _list_controllers(node: Node) -> list[dict[str, str]]:
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeUnavailable("/controller_manager/list_controllers is unavailable")
    response = _wait(node, client.call_async(ListControllers.Request()))
    return [{"name": item.name, "type": item.type, "state": item.state} for item in response.controller]


def _hardware_components(node: Node) -> list[dict[str, Any]]:
    client = node.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeUnavailable("/controller_manager/list_hardware_components is unavailable")
    response = _wait(node, client.call_async(ListHardwareComponents.Request()))
    return [
        {"name": item.name, "type": item.type, "state": {"id": item.state.id, "label": item.state.label}}
        for item in response.component
    ]


def _is_gazebo_component(item: dict[str, Any]) -> bool:
    return "gazebo" in f"{item.get('name', '')} {item.get('type', '')}".lower()


def _native_gazebo_identity(node: Node) -> dict[str, Any]:
    components = _hardware_components(node)
    component_text = json.dumps(components, sort_keys=True).lower()
    gazebo_count = sum(_is_gazebo_component(component) for component in components)
    return {
        "hardware_components": components,
        "hardware_component_count": len(components),
        "gazebo_hardware_component_count": gazebo_count,
        "gazebo_hardware_component_present": gazebo_count > 0,
        "native_gazebo_backend_only": bool(components) and gazebo_count == len(components),
        "dummy_ros_control_device_present": "dummy_ros_control" in component_text,
        "dpsdk_device_dependencies_present": "dpsdk" in component_text,
    }


def _public_action(node: Node) -> ActionClient:
    client = ActionClient(node, FollowJointTrajectory, PUBLIC_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        client.destroy()
        raise RuntimeUnavailable(f"public FollowJointTrajectory action is unavailable: {PUBLIC_ACTION}")
    return client


def _goal(current: dict[str, float], joint1_target: float) -> FollowJointTrajectory.Goal:
    return FollowJointTrajectory.Goal(trajectory=JointTrajectory(
        joint_names=JOINTS,
        points=[JointTrajectoryPoint(
            positions=[joint1_target if name == "Joint1" else current[name] for name in JOINTS],
            time_from_start=Duration(sec=2),
        )],
    ))


def _gz_services() -> list[str]:
    try:
        listed = subprocess.run(
            ["gz", "service", "-l"], capture_output=True, text=True, check=False, timeout=8,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        raise RuntimeUnavailable(f"Gazebo transport CLI unavailable: {error}") from error
    if listed.returncode:
        raise RuntimeUnavailable(listed.stderr.strip() or "Gazebo transport service discovery failed")
    return listed.stdout.splitlines()


def _world_services() -> tuple[str, str]:
    for service in _gz_services():
        matched = re.fullmatch(r"/world/([^/]+)/control", service.strip())
        if matched:
            return matched.group(1), service.strip()
    raise RuntimeUnavailable("Gazebo world control service is unavailable")


def _gz_control(service: str, pause: bool) -> None:
    try:
        completed = subprocess.run(
            ["gz", "service", "-s", service, "--reqtype", "gz.msgs.WorldControl",
             "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
             "--req", f"pause: {'true' if pause else 'false'}"],
            capture_output=True, text=True, check=False, timeout=8,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        raise RuntimeUnavailable(f"Gazebo world control command failed: {error}") from error
    if completed.returncode or "data: true" not in completed.stdout.lower():
        raise RuntimeUnavailable(completed.stderr.strip() or completed.stdout.strip() or "Gazebo rejected world control")


def _balanced_protobuf_blocks(payload: str, field: str) -> list[str]:
    blocks: list[str] = []
    expression = re.compile(rf"\b{re.escape(field)}\s*\{{")
    start = 0
    while match := expression.search(payload, start):
        depth = 0
        for index in range(match.end() - 1, len(payload)):
            if payload[index] == "{":
                depth += 1
            elif payload[index] == "}":
                depth -= 1
                if depth == 0:
                    blocks.append(payload[match.start():index + 1])
                    start = index + 1
                    break
        else:
            raise ValueError(f"unterminated protobuf {field} block")
    return blocks


def _parse_base_link_pose(payload: str) -> dict[str, Any]:
    matches = [
        block for block in _balanced_protobuf_blocks(payload, "pose")
        if re.search(r'name:\s*"base_link"', block)
    ]
    if len(matches) != 1:
        raise ValueError("pose-info response must contain exactly one base_link pose block")
    positions = _balanced_protobuf_blocks(matches[0], "position")
    if len(positions) != 1:
        raise ValueError("base_link pose must contain exactly one position block")
    values: dict[str, Any] = {}
    defaulted_axes: list[str] = []
    for axis in ("x", "y", "z"):
        match = re.search(rf"\b{axis}:\s*(\S+)", positions[0])
        if match is None:
            values[axis] = 0.0
            defaulted_axes.append(axis)
        else:
            try:
                value = float(match.group(1))
            except ValueError as error:
                raise ValueError(f"base_link position {axis} is not numeric") from error
            if not math.isfinite(value):
                raise ValueError(f"base_link position {axis} is not finite")
            values[axis] = value
    values["protobuf_defaulted_axes"] = defaulted_axes
    return values


def _base_pose(world: str) -> dict[str, Any]:
    completed = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/pose/info"],
        capture_output=True, text=True, check=False, timeout=8,
    )
    if completed.returncode:
        raise RuntimeUnavailable(completed.stderr.strip() or "Gazebo pose-info topic read failed")
    try:
        return _parse_base_link_pose(completed.stdout)
    except ValueError as error:
        raise RuntimeUnavailable(str(error)) from error


def _fre11(node: ObservationNode) -> dict[str, Any]:
    identity = _native_gazebo_identity(node)
    _, control_service = _world_services()
    deadline = time.monotonic() + 8.0
    while len(node.clock_samples) < 2 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(node.clock_samples) < 2 or node.clock_samples[-1] <= node.clock_samples[0]:
        raise RuntimeUnavailable("/clock is not advancing before Gazebo pause")
    before_pause = len(node.clock_samples)
    _gz_control(control_service, True)
    try:
        frozen_clock, settled_start = _wait_for_clock_quiescence(node)
        _spin(node, 0.5)
        paused = node.clock_samples[settled_start:]
        _gz_control(control_service, False)
        resumed_start = len(node.clock_samples)
        _spin(node, 1.2)
    except BaseException:
        _gz_control(control_service, False)
        raise
    resumed = node.clock_samples[resumed_start:]
    paused_advanced = bool(paused) and max(paused) > min(paused)
    return {
        **identity,
        "gazebo_world_control_available": True, "gazebo_pause_requested": True,
        "gazebo_resume_requested": True, "clock_samples_before_pause": before_pause,
        "clock_advanced_before_pause": node.clock_samples[before_pause - 1] > node.clock_samples[0],
        "clock_samples_while_paused": len(paused), "clock_advanced_while_paused": paused_advanced,
        "clock_stopped_while_paused": not paused_advanced,
        "clock_samples_after_resume": len(resumed),
        "clock_advanced_after_resume": bool(resumed) and resumed[-1] > frozen_clock,
    }


def _ros01(node: ObservationNode) -> dict[str, Any]:
    controllers = _list_controllers(node)
    identity = _native_gazebo_identity(node)
    _require_joint_samples(node)
    action = _public_action(node)
    action.destroy()
    active = [item for item in controllers if item["state"] == "active"]
    return {
        **identity,
        "controller_manager_available": True, "loaded_controllers": controllers,
        "active_controller_count": len(active), "joint_state_sample_count": len(node.joint_samples),
        "public_action_available": True,
    }


def _ros02(node: ObservationNode) -> dict[str, Any]:
    identity = _native_gazebo_identity(node)
    samples = _require_joint_samples(node, 3)
    start_index = len(samples)
    initial = dict(samples[-1]["joint_positions"])
    target = initial["Joint1"] - 0.15 if initial["Joint1"] > 0.0 else initial["Joint1"] + 0.15
    action = _public_action(node)
    try:
        handle = _wait(node, action.send_goal_async(_goal(initial, target)))
        if not handle.accepted:
            return {**identity, "public_action_available": True, "goal_accepted": False, "goal_terminal_status": None,
                    "joint_state_time_series": [], "actual_joint1_change_rad": 0.0,
                    "final_joint1_abs_error_rad": math.inf}
        result = _wait(node, handle.get_result_async(), timeout=15.0)
        _spin(node, 0.4)
    finally:
        action.destroy()
    trajectory_samples = node.joint_samples[start_index:]
    values = [sample["joint1_rad"] for sample in trajectory_samples]
    final = values[-1] if values else initial["Joint1"]
    return {
        **identity,
        "public_action_available": True, "goal_accepted": True, "goal_terminal_status": result.status,
        "joint1_target_rad": target, "joint_state_time_series": trajectory_samples,
        "actual_joint1_change_rad": max(values) - min(values) if values else 0.0,
        "final_joint1_abs_error_rad": abs(final - target),
    }


def _ros03(node: ObservationNode) -> dict[str, Any]:
    identity = _native_gazebo_identity(node)
    world, _ = _world_services()
    start_joint = len(node.joint_samples)
    base_samples: list[dict[str, Any]] = []
    for _ in range(3):
        base_samples.append({"time_sec": time.monotonic() - node.started_at, **_base_pose(world)})
        _spin(node, 0.6)
    joint_samples = node.joint_samples[start_joint:]
    if len(joint_samples) < 3:
        raise RuntimeUnavailable("insufficient no-command /joint_states samples")
    axis_drift = {
        name: max(sample["joint_positions"][name] for sample in joint_samples)
        - min(sample["joint_positions"][name] for sample in joint_samples)
        for name in JOINTS
    }
    origin = base_samples[0]
    drift = max(math.sqrt(sum((sample[axis] - origin[axis]) ** 2 for axis in ("x", "y", "z"))) for sample in base_samples)
    return {
        **identity,
        "no_command_sent": True, "joint_state_time_series": joint_samples,
        "joint_axis_drift_rad": axis_drift,
        "max_joint_drift_rad": max(axis_drift.values()),
        "base_pose_time_series": base_samples, "max_base_translation_drift_m": drift,
        "protobuf_defaulted_axes": sorted({
            axis for sample in base_samples for axis in sample["protobuf_defaulted_axes"]
        }),
        "base_is_fixed": drift < 0.002,
    }


def _ros11(node: ObservationNode) -> dict[str, Any]:
    controllers = _list_controllers(node)
    identity = _native_gazebo_identity(node)
    active_names = [item["name"] for item in controllers if item["state"] == "active"]
    locked = next((item for item in controllers if item["name"] == LOCKED_CONTROLLER), None)
    accepted = locked is not None and locked["state"] == "active" and locked["type"] == LOCKED_CONTROLLER_TYPE
    return {
        **identity,
        "controller_manager_available": True, "loaded_controllers": controllers,
        "active_controller_names": active_names, "locked_controller_name": LOCKED_CONTROLLER,
        "locked_controller_type": LOCKED_CONTROLLER_TYPE, "locked_configuration_accepted": accepted,
    }


CASES = {"FRE11": _fre11, "ROS01": _ros01, "ROS02": _ros02, "ROS03": _ros03, "ROS11": _ros11}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=sorted(CASES), required=True)
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    rclpy.init()
    node = ObservationNode()
    try:
        observations = CASES[arguments.case](node)
    except RuntimeUnavailable as error:
        node.get_logger().error(str(error))
        return 3
    except Exception as error:
        node.get_logger().error(f"runtime probe failed: {error}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    arguments.report.write_text(json.dumps(observations, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
