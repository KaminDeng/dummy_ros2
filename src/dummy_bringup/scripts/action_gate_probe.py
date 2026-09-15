#!/usr/bin/python3
"""Exercise BUSY, cancel, and post-cancel behavior of the public FJT gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [f"Joint{index}" for index in range(1, 7)]


def goal(joint1: float, duration_seconds: int) -> FollowJointTrajectory.Goal:
    return FollowJointTrajectory.Goal(
        trajectory=JointTrajectory(
            joint_names=JOINTS,
            points=[
                JointTrajectoryPoint(
                    positions=[joint1, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0],
                    velocities=[0.0] * 6,
                    time_from_start=Duration(sec=duration_seconds),
                )
            ],
        )
    )


def wait(node: Node, future, timeout: float = 15.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError("action future did not complete")
    return future.result()


def run(node: Node) -> dict[str, int | bool]:
    positions: list[float] = []

    def observe(message: JointState) -> None:
        if "Joint1" in message.name:
            positions.append(message.position[message.name.index("Joint1")])

    subscription = node.create_subscription(JointState, "/joint_states", observe, 10)
    client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    internal_client = ActionClient(
        node, FollowJointTrajectory, "/_internal_arm_controller/follow_joint_trajectory"
    )
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("public action server unavailable")
    if not internal_client.wait_for_server(timeout_sec=15.0):
        raise RuntimeError("internal trajectory controller unavailable")
    deadline = time.monotonic() + 15.0
    while not positions and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not positions:
        raise RuntimeError("joint state feedback unavailable before first goal")
    first = wait(node, client.send_goal_async(goal(0.2, 8)))
    second = wait(node, client.send_goal_async(goal(-0.2, 2)))
    if not first.accepted:
        raise RuntimeError("first action goal rejected")
    wait(node, first.cancel_goal_async())
    first_result = wait(node, first.get_result_async())
    third = wait(node, client.send_goal_async(goal(-0.1, 2)))
    if not third.accepted:
        raise RuntimeError("third action goal rejected after cancel")
    third_result = wait(node, third.get_result_async())
    out_of_bounds = wait(node, client.send_goal_async(goal(5.0, 2)))
    if out_of_bounds.accepted:
        wait(node, out_of_bounds.cancel_goal_async())
        wait(node, out_of_bounds.get_result_async())
    position_change = max(positions) - min(positions) if positions else 0.0
    observations = {
        "first_goal_accepted": first.accepted,
        "second_goal_accepted": second.accepted,
        "first_goal_terminal_status": first_result.status,
        "third_goal_accepted": third.accepted,
        "third_goal_terminal_status": third_result.status,
        "out_of_bounds_goal_accepted": out_of_bounds.accepted,
        "joint_state_sample_count": len(positions),
        "joint1_position_change_rad": position_change,
    }
    node.destroy_subscription(subscription)
    internal_client.destroy()
    return observations


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    rclpy.init()
    node = Node("dummy_r05_action_gate_probe")
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
