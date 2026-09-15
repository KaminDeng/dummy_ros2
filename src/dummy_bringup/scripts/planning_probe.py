#!/usr/bin/python3
"""Read-only MoveIt planning service probe for R05 acceptance."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as element_tree

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest, PlanningSceneComponents,
    PositionIKRequest, RobotState,
)
from moveit_msgs.srv import (
    GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK,
    GetStateValidity,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import yaml


JOINT_NAMES = tuple(f"Joint{index}" for index in range(1, 7))
START = (0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0)
SOURCE_MODEL = Path(__file__).resolve().parents[3] / "contracts/dummy_robot_v1/model/model.yaml"
MODEL = SOURCE_MODEL
if not MODEL.is_file():
    MODEL = (
        Path(get_package_share_directory("dummy_bringup"))
        / "contracts/dummy_robot_v1/model/model.yaml"
    )
SOURCE_SCENE = Path(__file__).resolve().parents[1] / "config/scene.yaml"
SOURCE_WORLD = Path(__file__).resolve().parents[2] / "dummy_moveit_config/worlds/empty.sdf"
SCENE = SOURCE_SCENE
WORLD = SOURCE_WORLD
if not SCENE.is_file():
    share = Path(get_package_share_directory("dummy_bringup"))
    SCENE = share / "config/scene.yaml"
    WORLD = Path(get_package_share_directory("dummy_moveit_config")) / "worlds/empty.sdf"


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


def motion_request(goal: tuple[float, ...]) -> GetMotionPlan.Request:
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


def find_barrier_collision(node: Node) -> tuple[float, ...] | None:
    model = yaml.safe_load(MODEL.read_text())["joints"]
    generator = random.Random(20260914)
    candidates = [
        tuple(generator.uniform(*joint["model_limits_rad"]) for joint in model)
        for _ in range(160)
    ]
    for positions in candidates:
        response = call(
            node,
            GetStateValidity,
            "/check_state_validity",
            GetStateValidity.Request(
                robot_state=RobotState(
                    joint_state=JointState(name=list(JOINT_NAMES), position=list(positions))
                ),
                group_name="dummy_arm",
            ),
        )
        if any(
            contact.contact_body_1 == "planning_barrier" or contact.contact_body_2 == "planning_barrier"
            for contact in response.contacts
        ):
            return positions
    return None


def scene_matches_canonical(node: Node) -> tuple[bool, bool]:
    canonical = yaml.safe_load(SCENE.read_text())
    expected = canonical["objects"][0]
    response = call(
        node,
        GetPlanningScene,
        "/get_planning_scene",
        GetPlanningScene.Request(
            components=PlanningSceneComponents(
                components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
        ),
    )
    objects = {item.id: item for item in response.scene.world.collision_objects}
    actual = objects.get(expected["id"])
    planning_matches = False
    if actual is not None and actual.header.frame_id in {canonical["frame_id"], "base_link"}:
        primitive = actual.primitives[0] if actual.primitives else None
        if primitive is not None:
            pose = actual.pose
            expected_pose = expected["pose_xyz_rpy"]
            planning_matches = (
                all(abs(value - target) < 1e-9 for value, target in zip(
                    primitive.dimensions, expected["size"]
                ))
                and all(abs(value - target) < 1e-9 for value, target in zip(
                    (pose.position.x, pose.position.y, pose.position.z), expected_pose[:3]
                ))
            )
    world = element_tree.parse(WORLD).getroot().find(
        f"./world/model[@name='{expected['id']}']"
    )
    world_matches = False
    if world is not None:
        pose = [float(value) for value in world.findtext("pose", "").split()]
        size = [float(value) for value in world.findtext("./link/collision/geometry/box/size", "").split()]
        world_matches = pose == expected["pose_xyz_rpy"] and size == expected["size"]
    return planning_matches, bool(canonical.get("scene_revision")) and planning_matches and world_matches


def solve_nearby_pose(node: Node) -> tuple[int, bool]:
    state = RobotState(joint_state=JointState(name=list(JOINT_NAMES), position=list(START)))
    fk = call(
        node,
        GetPositionFK,
        "/compute_fk",
        GetPositionFK.Request(
            header=Header(frame_id="base_link"),
            fk_link_names=["link6_1_1"],
            robot_state=state,
        ),
    )
    if fk.error_code.val != 1 or not fk.pose_stamped:
        return fk.error_code.val, False
    target = PoseStamped(header=Header(frame_id="base_link"), pose=fk.pose_stamped[0].pose)
    target.pose.position.y += 0.005
    ik = call(
        node,
        GetPositionIK,
        "/compute_ik",
        GetPositionIK.Request(
            ik_request=PositionIKRequest(
                group_name="dummy_arm",
                robot_state=state,
                avoid_collisions=True,
                ik_link_name="link6_1_1",
                pose_stamped=target,
                timeout=Duration(sec=2),
            )
        ),
    )
    by_name = dict(zip(ik.solution.joint_state.name, ik.solution.joint_state.position))
    solution = tuple(by_name.get(name, math.nan) for name in JOINT_NAMES)
    return ik.error_code.val, validate_joint_target(solution)


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
    valid = call(node, GetMotionPlan, "/plan_kinematic_path", motion_request((0.15,) + START[1:]))
    out_of_bounds_rejected = not validate_joint_target((5.0,) + START[1:])
    cartesian = call(node, GetCartesianPath, "/compute_cartesian_path", cartesian_request())
    collision_state = find_barrier_collision(node)
    planning_scene_matches, world_scene_revision_matches = scene_matches_canonical(node)
    collision_plan = (
        call(node, GetMotionPlan, "/plan_kinematic_path", motion_request(collision_state))
        if collision_state is not None
        else None
    )
    pose_ik_error, pose_ik_within_limits = solve_nearby_pose(node)
    final = joint_sample(node)
    return {
        "valid_joint_plan_error_code": valid.motion_plan_response.error_code.val,
        "valid_joint_trajectory_points": len(valid.motion_plan_response.trajectory.joint_trajectory.points),
        "out_of_bounds_rejected_by_guard": out_of_bounds_rejected,
        "out_of_bounds_service_sent": not out_of_bounds_rejected,
        "unreachable_cartesian_fraction": cartesian.fraction,
        "barrier_collision_state_found": collision_state is not None,
        "planning_scene_matches_canonical": planning_scene_matches,
        "world_scene_revision_matches": world_scene_revision_matches,
        "barrier_collision_goal_error_code": (
            collision_plan.motion_plan_response.error_code.val if collision_plan is not None else 0
        ),
        "pose_ik_error_code": pose_ik_error,
        "pose_ik_solution_within_contract_limits": pose_ik_within_limits,
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
