#!/usr/bin/python3
"""Apply the canonical Dummy scene to MoveIt's planning scene."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import yaml


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def load_scene(path: Path) -> PlanningScene:
    data = yaml.safe_load(path.read_text())
    frame_id = data["frame_id"]
    if frame_id != "world":
        raise ValueError("scene objects must use the world frame")
    objects = []
    identifiers: set[str] = set()
    for item in data["objects"]:
        if item["id"] in identifiers:
            raise ValueError(f"duplicate scene object: {item['id']}")
        identifiers.add(item["id"])
        if item["shape"] != "box" or len(item["size"]) != 3 or len(item["pose_xyz_rpy"]) != 6:
            raise ValueError(f"unsupported scene object: {item['id']}")
        if any(not math.isfinite(value) for value in (*item["size"], *item["pose_xyz_rpy"])):
            raise ValueError(f"non-finite scene object: {item['id']}")
        if any(value <= 0.0 for value in item["size"]):
            raise ValueError(f"non-positive scene object: {item['id']}")
        primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(item["size"]))
        xyz = item["pose_xyz_rpy"][:3]
        quaternion = quaternion_from_rpy(*item["pose_xyz_rpy"][3:])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        objects.append(
            CollisionObject(
                header=Header(frame_id=frame_id),
                id=item["id"],
                primitives=[primitive],
                primitive_poses=[pose],
                operation=CollisionObject.ADD,
            )
        )
    return PlanningScene(
        is_diff=True,
        world=PlanningSceneWorld(collision_objects=objects),
    )


def main() -> int:
    rclpy.init()
    node = Node("dummy_scene_publisher")
    node.declare_parameter("scene_file", "")
    scene_file = Path(node.get_parameter("scene_file").value)
    try:
        scene = load_scene(scene_file)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        node.get_logger().error(f"invalid scene file: {error}")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=20.0):
        node.get_logger().error("/apply_planning_scene unavailable")
        node.destroy_node()
        rclpy.shutdown()
        return 3
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    response = future.result()
    success = response is not None and response.success
    node.get_logger().info(f"scene applied: {success}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 4


if __name__ == "__main__":
    sys.exit(main())
