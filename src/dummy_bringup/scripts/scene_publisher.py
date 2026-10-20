#!/usr/bin/python3
##
# @file scene_publisher.py
# @brief 把公共 YAML 箱体场景作为差量提交到 MoveIt。
# 运行时调用 /apply_planning_scene，但不提交运动，不启动机器人或控制器。该固定箱体场景与手工面板的可编辑圆柱事务不同；应用成功表示 MoveIt 接受场景差量。
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


##
# @brief 将弧度制 RPY 转为 ROS 顺序四元数。
# 此函数不单独校验数值；load_scene 在调用前检查 YAML 位姿有限性。
# @param roll 绕 X 轴 roll，rad。
# @param pitch 绕 Y 轴 pitch，rad。
# @param yaw 绕 Z 轴 yaw，rad。
# @return tuple[float,float,float,float]：(x,y,z,w)，无量纲。
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


##
# @brief 校验公共场景并创建只增加箱体的 PlanningScene 差量。
# ID 不得重复；只支持 box、三个正尺寸和六维有限 xyz/rpy。尺寸/xyz 为米，rpy 为弧度；此处未发送消息，差量也不移除未列出的既有对象。
# @param path 场景 YAML 路径。
# @return PlanningScene：is_diff=True，每个对象为 world 下 CollisionObject.ADD。
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


##
# @brief 读取 scene_file 参数，等待服务并提交一次场景差量。
# 等服务最多 20 s，等待应用响应最多 10 s；未收到有效成功响应即返回失败。不会因应用失败自动执行或清理机器人动作。
# @return int：成功 0，场景输入错误 2，服务不可用 3，未确认应用成功 4。
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
    ##
    # 这是场景写服务；响应 success 只证明本次差量被接受，后续规划仍应使用与实际环境一致的场景。
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
