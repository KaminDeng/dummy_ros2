#!/usr/bin/python3
##
# @file gazebo_runtime_probe.py
# @brief 从已启动的 Gazebo 标准运行环境收集指定验收观测。
# 探针不启动 launch；不同 case 可能暂停/恢复仿真或向公开轨迹 action 发目标，运行它并非纯文件检查。设备身份由 controller_manager 观测名称/类型推断，不直接访问 USB，也不代表在线固件身份校验。
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


##
# 全部关节位置统一使用 rad；PUBLIC_ACTION 是公开门控，LOCKED_CONTROLLER 只用于查询所期望的内部控制器。
JOINTS = [f"Joint{index}" for index in range(1, 7)]
PUBLIC_ACTION = "/arm_controller/follow_joint_trajectory"
LOCKED_CONTROLLER = "_internal_arm_controller"
LOCKED_CONTROLLER_TYPE = "joint_trajectory_controller/JointTrajectoryController"


##
# @class RuntimeUnavailable
# @brief 区分运行环境未就绪与探针自身其他错误。
# main 将此异常映射为退出码 3；用于服务、反馈、时钟和 Gazebo transport 不可观测等情况。
class RuntimeUnavailable(RuntimeError):
    """The canonical Gazebo launch is not available to observe."""


##
# @class ObservationNode
# @brief 收集完整六轴反馈和仿真时钟的 ROS 观察节点。
# joint_samples 记录宿主单调相对秒数及各轴 rad；clock_samples 记录 /clock 仿真秒数。两种时间基准不能混用，暂停时宿主时间仍推进。
class ObservationNode(Node):
    ##
    # @brief 建立关节反馈和时钟订阅并开始宿主计时。
    # 两个订阅队列深度均为 50；started_at 是 monotonic 秒，样本列表只在本次探针生命周期内累积。
    # @return None；执行上述状态更新、界面或日志操作。
    def __init__(self) -> None:
        super().__init__("dummy_r02_gazebo_runtime_probe")
        self.started_at = time.monotonic()
        self.joint_samples: list[dict[str, Any]] = []
        self.clock_samples: list[float] = []
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self.create_subscription(Clock, "/clock", self._on_clock, 50)

    ##
    # @brief 只接收六轴名称齐全且位置有限的反馈。
    # 按名称映射为固定 JOINTS 顺序；附加自探针启动起的宿主秒数，未要求消息头时间或速度字段。
    # @param message JointState，关节位置单位 rad。
    # @return None；执行上述状态更新、界面或日志操作。
    def _on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in JOINTS):
            return
        self.joint_samples.append({
            "time_sec": time.monotonic() - self.started_at,
            "joint1_rad": positions["Joint1"],
            "joint_positions": {name: positions[name] for name in JOINTS},
        })

    ##
    # @brief 保存 /clock 消息的仿真时间。
    # @param message Clock 消息，其 sec 与 nanosec 换算成浮点秒。
    # @return None；执行上述状态更新、界面或日志操作。
    def _on_clock(self, message: Clock) -> None:
        stamp = message.clock
        self.clock_samples.append(stamp.sec + stamp.nanosec / 1_000_000_000)


##
# @brief 在给定宿主时长内持续处理 ROS 回调。
# 每次阻塞最多 0.1 s，避免仿真暂停导致基于 /clock 的等待永不结束。
# @param node 参与 spin_once 的节点。
# @param seconds 宿主 monotonic 计时的时长，秒。
# @return None；执行上述状态更新、界面或日志操作。
def _spin(node: Node, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))


##
# @brief 排空暂停前的时钟传输积压并等待停止推进。
# 每当最新时钟继续增加就重置安静计时；连续 0.3 s 未前进视为稳定。无样本或一直前进抛 RuntimeUnavailable；仅表示观测稳定，不要求暂停后持续发布相同时间戳。
# @param node 已收集至少一个时钟样本的 ObservationNode。
# @param timeout 最大宿主等待秒数，默认 5。
# @return tuple[float,int]：稳定仿真秒值和稳定区间起始样本索引。
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


##
# @brief 有界等待一次 ROS 服务或 action 异步结果。
# 未完成或 future 异常转为 RuntimeUnavailable；不自动撤销已经发送的服务或运动目标。
# @param node 负责驱动回调的节点。
# @param future 待完成的 ROS future。
# @param timeout 宿主等待上限秒数，默认 12。
# @return future 的响应对象。
def _wait(node: Node, future, timeout: float = 12.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeUnavailable("ROS request did not complete against the canonical Gazebo launch")
    return future.result()


##
# @brief 等待关节反馈累计达到指定数量。
# 最多等待宿主 10 s；数量不足抛 RuntimeUnavailable。这里保证数量，不额外判定最后样本的新鲜时间。
# @param node ObservationNode，样本由订阅回调累计。
# @param minimum 本节点启动以来的最少完整六轴样本数。
# @return list[dict]：节点原有样本列表，后续回调还会追加。
def _require_joint_samples(node: ObservationNode, minimum: int = 2) -> list[dict[str, Any]]:
    deadline = time.monotonic() + 10.0
    while len(node.joint_samples) < minimum and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(node.joint_samples) < minimum:
        raise RuntimeUnavailable("/joint_states from the canonical Gazebo launch is unavailable")
    return node.joint_samples


##
# @brief 查询 controller_manager 并提取已加载控制器状态。
# 等待服务最多 8 s，响应由 _wait 有界等待；不存在时抛 RuntimeUnavailable。
# @param node 用于创建 ListControllers 客户端的 ROS 节点。
# @return list[dict]：每项 name、type、state 字符串。
def _list_controllers(node: Node) -> list[dict[str, str]]:
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeUnavailable("/controller_manager/list_controllers is unavailable")
    response = _wait(node, client.call_async(ListControllers.Request()))
    return [{"name": item.name, "type": item.type, "state": item.state} for item in response.controller]


##
# @brief 查询 controller_manager 的硬件组件列表。
# @param node 用于创建 ListHardwareComponents 客户端的 ROS 节点。
# @return list[dict]：组件 name/type 及生命周期 state 的 id/label。
def _hardware_components(node: Node) -> list[dict[str, Any]]:
    client = node.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeUnavailable("/controller_manager/list_hardware_components is unavailable")
    response = _wait(node, client.call_async(ListHardwareComponents.Request()))
    return [
        {"name": item.name, "type": item.type, "state": {"id": item.state.id, "label": item.state.label}}
        for item in response.component
    ]


##
# @brief 用名称及类型文本识别 Gazebo 组件。
# 这是可审计的字符串启发式，不检查动态库实际加载内容。
# @param item 至少可包含 name、type 的组件字典。
# @return bool：组合文本中是否包含不区分大小写的 gazebo。
def _is_gazebo_component(item: dict[str, Any]) -> bool:
    return "gazebo" in f"{item.get('name', '')} {item.get('type', '')}".lower()


##
# @brief 整理后端组件观测及 Gazebo 专用性标记。
# native_gazebo_backend_only 要求列表非空且每项都命中 Gazebo 文本；依赖标记仅代表组件描述文本命中，不是系统依赖扫描。
# @param node 可查询硬件组件的节点。
# @return dict：原组件表、Gazebo 计数及 dummy_ros_control/dpsdk 文本出现标记。
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


##
# @brief 等待并返回公开轨迹 action 客户端。
# 10 s 内不可用时先销毁客户端，再抛 RuntimeUnavailable；不向内部控制器绕行。
# @param node 拥有 ActionClient 的 ROS 节点。
# @return ActionClient：/arm_controller/follow_joint_trajectory。
def _public_action(node: Node) -> ActionClient:
    client = ActionClient(node, FollowJointTrajectory, PUBLIC_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        client.destroy()
        raise RuntimeUnavailable(f"public FollowJointTrajectory action is unavailable: {PUBLIC_ACTION}")
    return client


##
# @brief 构造只改变 Joint1、其余轴保持当前位置的 2 s 目标。
# 仅构造消息；当前字典完整性由先前反馈采样保证。
# @param current 按 JOINTS 名称索引的当前六轴位置字典，rad。
# @param joint1_target Joint1 目标位置，rad。
# @return FollowJointTrajectory.Goal：完整六轴单点目标。
def _goal(current: dict[str, float], joint1_target: float) -> FollowJointTrajectory.Goal:
    return FollowJointTrajectory.Goal(trajectory=JointTrajectory(
        joint_names=JOINTS,
        points=[JointTrajectoryPoint(
            positions=[joint1_target if name == "Joint1" else current[name] for name in JOINTS],
            time_from_start=Duration(sec=2),
        )],
    ))


##
# @brief 通过 Gazebo transport CLI 枚举服务。
# CLI 最长等待 8 s；缺命令、超时或非零退出转为 RuntimeUnavailable。
# @return list[str]：gz service -l 输出的各行服务名。
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


##
# @brief 从服务列表取第一个世界控制入口。
# 只匹配完整 world control 路径；多世界环境按 CLI 列举顺序选择，没有额外世界名筛选。
# @return tuple[str,str]：world 名称及 `/world/<name>/control` 服务路径。
def _world_services() -> tuple[str, str]:
    for service in _gz_services():
        matched = re.fullmatch(r"/world/([^/]+)/control", service.strip())
        if matched:
            return matched.group(1), service.strip()
    raise RuntimeUnavailable("Gazebo world control service is unavailable")


##
# @brief 向 Gazebo 世界发送暂停或恢复请求并核对布尔回执。
# transport 请求超时 3000 ms，CLI 进程上限 8 s；非零退出或输出不含 data:true 则抛 RuntimeUnavailable。此操作会改变仿真运行状态。
# @param service 已发现的世界 control 服务路径。
# @param pause True 暂停，False 恢复仿真。
# @return None；执行上述状态更新、界面或日志操作。
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


##
# @brief 按花括号深度提取指定 protobuf 文本字段块。
# 用于已知 Gazebo 消息结构，不是通用 protobuf 解析器；没有处理引号内花括号转义，未闭合块抛 ValueError。
# @param payload gz 输出的 protobuf 文本。
# @param field 要匹配的消息字段名，作为正则字面量转义。
# @return list[str]：包含字段名和配对花括号的文本块。
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


##
# @brief 从 pose-info 文本提取唯一 base_link 的有限位置。
# 要求恰好一个 base_link pose 和一个 position 块；proto3 未打印的轴按 0.0 处理并记录，不把缺省数值误判为没有位置块。无效数量/非数值/非有限值抛 ValueError。
# @param payload Gazebo pose/info 的 protobuf 文本。
# @return dict：x/y/z 米及 protobuf_defaulted_axes 缺省轴名列表。
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


##
# @brief 从指定世界的 pose/info topic 读取一次基座位置。
# CLI 读取一条消息，进程超时 8 s；非零退出或解析错误转为 RuntimeUnavailable。
# @param world 已发现的 Gazebo 世界名。
# @return dict：由 _parse_base_link_pose 验证的位置，单位米。
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


##
# @brief 观察 Gazebo 暂停前、暂停稳定后和恢复后的时钟推进。
# 先确认时钟前进，再暂停并排空积压，观察 0.5 s 后恢复并观察 1.2 s。暂停后的异常路径会尝试恢复仿真再重抛；不会发关节目标。
# @param node 收集时钟与后端组件的 ObservationNode。
# @return dict：后端观测、暂停/恢复请求标记与各阶段时钟样本统计。
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


##
# @brief 收集控制器、后端、关节反馈与公开 action 可用性。
# 只查询与等待，不提交运动目标。
# @param node ObservationNode。
# @return dict：已加载控制器、active 数量、样本数和公开接口可用标记。
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


##
# @brief 经公开 action 执行 Joint1 小幅目标并记录反馈曲线。
# 目标相对当前 Joint1 向零侧改变 0.15 rad，其他轴冻结；等待结果最多 15 s 后再观察 0.4 s。拒绝目标时返回空曲线和无穷终点误差，保持原始观测供上层判定。
# @param node 已取得至少三个完整反馈样本的 ObservationNode。
# @return dict：目标接受/终态、反馈曲线、实际角差和终点误差 rad。
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


##
# @brief 在无新运动命令下测量关节和基座漂移。
# 读取三次基座位置并各处理 0.6 s 回调；要求至少三个关节样本。base_is_fixed 采用 0.002 m 门限，只测平移，不验证姿态固定。
# @param node 持续收集完整六轴状态的 ObservationNode。
# @return dict：各轴漂移 rad、基座平移漂移 m、时间序列与 base_is_fixed 标记。
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


##
# @brief 核对内部锁定控制器的名称、类型和 active 状态。
# 同时保留硬件后端文本观测；不加载、切换或启动控制器。
# @param node 可查询 controller_manager 的 ObservationNode。
# @return dict：控制器列表与 locked_configuration_accepted 布尔观测。
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


##
# FRE11 会暂停/恢复仿真，ROS02 会提交公开运动目标；ROS01/03/11 主要收集查询及反馈，不能一概称所有case只读。
CASES = {"FRE11": _fre11, "ROS01": _ros01, "ROS02": _ros02, "ROS03": _ros03, "ROS11": _ros11}


##
# @brief 按 --case 收集一组运行观测并写 JSON。
# 要求 --case 和 --report；采集 finally 销毁节点并 shutdown。报告目录创建和写入在采集成功后执行，文件系统失败继续传播。
# @return int：成功写入 0，运行环境不可观测 3，其他采集异常 1。
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
