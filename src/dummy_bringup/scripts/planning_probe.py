#!/usr/bin/python3
##
# @file planning_probe.py
# @brief 通过只读 MoveIt 服务验证规划、逆解与公共场景一致性。
# 不向 action 控制器发送轨迹，也不直接打开 USB；读取 /joint_states 比较调用前后 Joint1 变化。服务成功、返回完整路径与是否产生运动分别报告，单个布尔字段不是完整硬件安全证明。
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


##
# 测试固定六轴轴序与 START 都使用 rad；SOURCE_* 优先源码资源，不存在时从 ament 已安装包定位。
JOINT_NAMES = tuple(f"Joint{index}" for index in range(1, 7))
START = (0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0)
SOURCE_MODEL = Path(__file__).resolve().parents[3] / "contracts/dummy_robot_v2/model/model.yaml"
MODEL = SOURCE_MODEL
if not MODEL.is_file():
    MODEL = (
        Path(get_package_share_directory("dummy_bringup"))
        / "contracts/dummy_robot_v2/model/model.yaml"
    )
SOURCE_SCENE = Path(__file__).resolve().parents[1] / "config/scene.yaml"
SOURCE_WORLD = Path(__file__).resolve().parents[2] / "dummy_moveit_config/worlds/empty.sdf"
SCENE = SOURCE_SCENE
WORLD = SOURCE_WORLD
if not SCENE.is_file():
    share = Path(get_package_share_directory("dummy_bringup"))
    SCENE = share / "config/scene.yaml"
    WORLD = Path(get_package_share_directory("dummy_moveit_config")) / "worlds/empty.sdf"


##
# @brief 按契约模型轴序和 model_limits_rad 检查六轴目标。
# 每次读取 MODEL；契约轴序与 ROS 轴集不一致时抛 ValueError，不默默重排。这里的模型限位不同于面板为停车余量内收的工作范围。
# @param positions 按 Joint1..Joint6 排列的六轴角度，rad。
# @return bool：维数、有限性及逐轴模型限位均符合要求。
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


##
# @brief 等待一个 MoveIt 服务并有界读取响应。
# 不可用、等待未完成或 future 异常抛 RuntimeError；本函数不提交执行 action，也不因超时自动撤销远端计算。
# @param node 负责创建客户端及 spin 的 ROS 节点。
# @param service_type 对应 ROS 服务类型。
# @param name 完整服务名。
# @param request 该服务的 Request 对象。
# @param timeout 等待服务和等待响应分别使用的秒数，默认 15。
# @return 响应对象：future.result()。
def call(node: Node, service_type, name: str, request, timeout: float = 15.0):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"service unavailable: {name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError(f"service request failed: {name}")
    return future.result()


##
# @brief 临时订阅并取得至少一个 Joint1 位置。
# finally 销毁临时订阅；无样本抛 RuntimeError。截止使用 node.get_clock()，若使用仿真时间且时钟暂停，宿主等待可能长于给定秒数。
# @param node 负责订阅和 ROS 时钟截止判断的节点。
# @param timeout ROS 节点时钟中的超时秒数，默认 8。
# @return float：最后收到的 Joint1 位置 rad。
def joint_sample(node: Node, timeout: float = 8.0) -> float:
    samples: list[float] = []

    ##
    # @par 内部回调 joint_sample.observe
    # 按消息名称提取 Joint1 样本。
    # 只要求 Joint1 名称存在；不在本回调核对完整六轴或消息时间戳。
    # 参数 message：JointState，位置 rad。
    # 返回：None；执行上述状态更新、界面或日志操作。
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


##
# @brief 构造从固定 START 到合法六轴目标的只规划请求。
# 目标容差 0.001 rad，最多两次尝试、5 s 规划时间，速度/加速度缩放均为 0.1。START 是测试固定姿态，不自动使用实机当前位置；返回路径不会自动执行。
# @param goal 完整六轴目标 rad，必须通过契约模型范围检查。
# @return GetMotionPlan.Request：dummy_arm 的关节约束请求。
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


##
# @brief 以固定随机种子寻找接触 planning_barrier 的状态。
# 在契约模型限位内生成候选，检查 contacts 中对象名称；单纯 invalid 但未指向 barrier 不算命中。所有请求只计算状态有效性。
# @param node 用于调用 GetStateValidity 的 ROS 节点。
# @return tuple 或 None：首次被报告接触 barrier 的六轴 rad；160 个候选均未命中则 None。
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


##
# @brief 比较首个公共场景对象与 MoveIt/Gazebo 文件几何。
# 只比较 YAML 第一个对象；MoveIt 部分接受 world/base_link，核对 primitive 尺寸与 object.pose 位置，未比较完整场景或旋转。世界文件部分按原始 pose 和 size 列表比较；不是完整几何等价证明。
# @param node 用于 GetPlanningScene 的 ROS 节点。
# @return tuple[bool,bool]：规划场景尺寸/位置匹配，以及同时具有 scene_revision 且世界文件匹配。
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


##
# @brief 从固定 START 做 FK，再对 Y 偏移 5 mm 的目标做避碰 IK。
# FK 失败立即返回其错误码和 False；IK 解按 Joint1..6 重排，缺轴用 NaN 使范围检查失败。IK 超时为服务侧 2 s，不执行运动。
# @param node 用于 FK 和 IK 服务请求的 ROS 节点。
# @return tuple[int,bool]：FK/IK 错误码和逆解是否位于契约模型范围。
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


##
# @brief 构造目标远离机械臂工作区的笛卡尔路径请求。
# max_step=0.01 m、避碰开启，速度和加速度比例 0.1；用于观察不可达路径 fraction，不调用执行接口。
# @return GetCartesianPath.Request：base 起点 START、末端 link6_1_1、X=5 m 的目标。
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


##
# @brief 顺序执行规划、越界、不可达、障碍和逆解观测。
# 越界目标先在本地拒绝，不发送该目标的规划服务。trajectory_executed 仅表示两次 Joint1 样本差超过 0.005 rad 的观测启发式，不能定位运动来源或覆盖其他轴。
# @param node 已可访问相应 MoveIt 服务和关节状态的节点。
# @return dict：各服务错误码、完整路径点数、场景匹配、逆解范围与调用前后 Joint1 rad。
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


##
# @brief 创建探针节点并输出只读规划观测报告。
# 要求 --report；finally 销毁节点并关闭 ROS 上下文，其他输入/文件异常继续传播。
# @return int：成功 0，RuntimeError 表示运行服务条件未满足时为 3。
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
