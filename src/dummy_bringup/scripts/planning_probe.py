#!/usr/bin/python3
"""Read-only MoveIt planning service probe for R05 acceptance."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


JOINT_NAMES = tuple(f"Joint{index}" for index in range(1, 7))
START = (0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0)
MODEL = Path(__file__).resolve().parents[3] / "contracts/dummy_robot_v1/model/model.yaml"


def validate_joint_target(positions: tuple[float, ...]) -> bool:
    if len(positions) != len(JOINT_NAMES) or any(not math.isfinite(value) for value in positions):
        return False
    joints = yaml.safe_load(MODEL.read_text())["joints"]
    if tuple(joint["name"] for joint in joints) != JOINT_NAMES:
        raise ValueError("C02 model joint order differs from the ROS axis set")
    return all(
        joint["model_limits_rad"][0] <= value <= joint["model_limits_rad"][1]
        for value, joint in zip(positions, joints)
    )


def call(node: Node, service_type, name: str, request, timeout: float = 15.0):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"service unavailable: {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError(f"service request failed: {name}")
    return future.result()


def joint_sample(node: Node, timeout: float = 8.0) -> float:
    samples: list[float] = []

    def observe(message: JointState) -> None:
        if "Joint1" in message.name:
            samples.append(message.position[message.name.index("Joint1")])

    subscription = node.create_subscription(JointState, "/joint_states", observe, 10)
    try:
        deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not samples and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not samples:
            raise RuntimeError("no Joint1 feedback received")
        return samples[-1]
    finally:
        node.destroy_subscription(subscription)


def motion_request(joint1: float) -> GetMotionPlan.Request:
    goal = (joint1,) + START[1:]
    if not validate_joint_target(goal):
        raise ValueError("target is outside the C02 joint limits")
    constraints = Constraints(
        joint_constraints=[
            JointConstraint(
                joint_name=name,
                position=value,
                tolerance_above=0.001,
                tolerance_below=0.001,
                weight=1.0,
            )
            for name, value in zip(JOINT_NAMES, goal)
        ]
    )
    return GetMotionPlan.Request(
        motion_plan_request=MotionPlanRequest(
            start_state=RobotState(
                joint_state=JointState(name=list(JOINT_NAMES), position=list(START))
            ),
            goal_constraints=[constraints],
            group_name="dummy_arm",
            num_planning_attempts=2,
            allowed_planning_time=5.0,
            max_velocity_scaling_factor=0.1,
            max_acceleration_scaling_factor=0.1,
        )
    )


def cartesian_request() -> GetCartesianPath.Request:
    far_pose = Pose()
    far_pose.position.x = 5.0
    far_pose.orientation.w = 1.0
    return GetCartesianPath.Request(
        start_state=RobotState(
            joint_state=JointState(name=list(JOINT_NAMES), position=list(START))
        ),
        group_name="dummy_arm",
        link_name="link6_1_1",
        waypoints=[far_pose],
        max_step=0.01,
        avoid_collisions=True,
        max_velocity_scaling_factor=0.1,
        max_acceleration_scaling_factor=0.1,
    )


def run(node: Node) -> dict[str, int | float | bool]:
    initial = joint_sample(node)
    valid = call(node, GetMotionPlan, "/plan_kinematic_path", motion_request(0.15))
    out_of_bounds_rejected = not validate_joint_target((5.0,) + START[1:])
    cartesian = call(node, GetCartesianPath, "/compute_cartesian_path", cartesian_request())
    final = joint_sample(node)
    return {
        "valid_joint_plan_error_code": valid.motion_plan_response.error_code.val,
        "valid_joint_trajectory_points": len(valid.motion_plan_response.trajectory.joint_trajectory.points),
        "out_of_bounds_rejected_by_guard": out_of_bounds_rejected,
        "out_of_bounds_service_sent": not out_of_bounds_rejected,
        "unreachable_cartesian_fraction": cartesian.fraction,
        "initial_joint1_rad": initial,
        "final_joint1_rad": final,
        "trajectory_executed": abs(final - initial) > 0.005,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    rclpy.init()
    node = Node("dummy_r05_planning_probe")
    try:
        observations = run(node)
        arguments.report.parent.mkdir(parents=True, exist_ok=True)
        arguments.report.write_text(json.dumps(observations, indent=2, sort_keys=True) + "\n")
        return 0
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
