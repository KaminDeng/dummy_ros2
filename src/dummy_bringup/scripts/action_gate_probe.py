#!/usr/bin/python3
##
# @file action_gate_probe.py
# @brief 对已经运行的公开 FollowJointTrajectory 门控执行忙碌与取消探测。
# 此探针会提交真实 ROS action 目标并请求取消，必须在调用方明确准备好的测试运行环境中使用；它不负责启动后端，也不直接访问 USB。报告记录观测结果，不在本文件内把观测值判为验收通过。
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


##
# @brief 构造以 Joint1 为变量、其余轴为固定姿态的单点目标。
# 其余关节采用 [0,π/2,0,0,0] rad；构造本身不检查当前姿态、限位或碰撞，也不发送目标。
# @param joint1 Joint1 目标角度，rad。
# @param duration_seconds 该单点的 time_from_start 整数秒数。
# @return FollowJointTrajectory.Goal：完整六轴位置和六轴零目标速度。
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


##
# @brief 驱动 ROS 回调并读取一个 action future 的结果。
# @exception RuntimeError 超时未完成或 future 带异常。超时不会自动取消远端目标；此处等待返回不等于物理停车已确认。
# @param node 用于 spin_until_future_complete 的 ROS 节点。
# @param future 发送目标、取消或等待结果产生的异步 future。
# @param timeout 本地等待上限，秒，默认 15。
# @return future.result() 的响应对象。
def wait(node: Node, future, timeout: float = 15.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError("action future did not complete")
    return future.result()


##
# @brief 顺序探测首次接收、并发忙碌、取消后再提交及越界拒绝。
# 先等待控制器与关节反馈，再向公开门控先后提交两个目标；取消首目标并等终态后提交第三个。最后尝试 Joint1=5 rad；若被接受则请求取消并等结果。内部 action 客户端只验证服务存在，不绕过公开门控发目标。
# @param node 已创建且可访问公开/内部控制器的 ROS 节点。
# @return dict：各目标接受状态、终态、Joint1 样本数及角差 rad。
def run(node: Node) -> dict[str, int | bool]:
    positions: list[float] = []

    ##
    # @par 内部回调 run.observe
    # 从关节状态消息累计 Joint1 位置样本。
    # 记录值单位 rad；本回调不做六轴完整性或有限性校验，样本仅用于本探针的变化量观测。
    # 参数 message：JointState 消息，按名称定位 Joint1 而非假定轴序。
    # 返回：None；执行上述状态更新、界面或日志操作。
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
    ##
    # 第二个目标在首目标尚未完成时发送，用于观察 BUSY 门控；代码保留 accepted 原值，不在此断言一定被拒绝。
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


##
# @brief 运行探测并将观测字典保存为 JSON 报告。
# 命令行要求 --report；ROS 节点和上下文在 finally 清理。JSON 保存的是观测，不是设备安全证明。
# @return int：成功写报告返回 0；RuntimeError 返回 3。
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
