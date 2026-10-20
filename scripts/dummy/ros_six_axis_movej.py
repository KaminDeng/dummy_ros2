#!/usr/bin/env python3
##
# @file ros_six_axis_movej.py
# @brief 通过实际MoveIt规划及公开Action采集单/多目标、取消和抢占证据。
#
# CLI没有--execute或dry-run开关：一旦参数通过就创建ROS客户端，可能激活hardware/controller、发运动Action、取消及DisableAndRelease。USB只由现有Gate持有，本脚本不直接开串口。
# BENCH模式绑定在线Gate身份和候选限制，ABSOLUTE_MODEL走历史硬编码LIMITS而不做该Gate输入绑定；两者不是同一资格等级。
# Action终态、ROS inactive、Gate RELEASED_IDLE与原始CAN/终链证明分别记录；取消报告仍强制NOT_QUALIFIED/UNCONFIRMED_STOP直到离线raw join。
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
##
# @brief ABSOLUTE_MODEL历史硬编码rad界限，不是从当前URDF或实机自动加载；尤其J5仍约±120°。
# BENCH模式改用受候选SHA绑定的相对planning范围。所有STOP_CONFIRM_TIMEOUT等是Host monotonic秒，设备Source时钟仅出现在显式witness字段。
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
##
# @brief 诊断最多512点/1MiB、targets JSON64KiB/最多50目标、单目标预算≤120s、反馈最多1024条；限制的是宿主资源和测试范围。
# MAX_ADMISSION_RETRIES=8只适用于精确续租拒绝见证，不能无限重发目标。
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


##
# @brief 无法确认停止/生命周期收尾的异常，保留仍active与未知组件、已完成目标和首失败报告；不能因抛异常便认为设备停了。
class UnconfirmedStopError(RuntimeError):
    """Action 未返回终态，不能确认运动已经停止。"""

    ##
    # @brief 保存停止不确定性及已有目标证据。
    # @param message 首失败文字。
    # @param still_active_components 观测仍active组件名。
    # @param unresolved_components 状态无法确认的组件名。
    # @param completed_goals 已完成的目标报告序列。
    # @param failed_goal_index 可选首失败序号。
    # @param failed_goal_report 可选已采失败目标证据。
    # @return None，列表字段复制，报告对象不深拷贝。
    def __init__(self, message, still_active_components=(), unresolved_components=(),
                 completed_goals=(), failed_goal_index=None, failed_goal_report=None):
        super().__init__(message)
        self.still_active_components = list(still_active_components)
        self.unresolved_components = list(unresolved_components)
        self.completed_goals = list(completed_goals)
        self.failed_goal_index = failed_goal_index
        self.failed_goal_report = failed_goal_report


##
# @brief Gate已确认RELEASED_IDLE但任务本身失败，保留原失败及组件观测；不自动升为硬件资格。
class ConfirmedStopError(RuntimeError):
    """守护门已确认 RELEASED_IDLE，但采集仍失败。"""

    ##
    # @brief 保留已确认release后的任务失败。
    # @param message 原失败文字。
    # @param still_active_components 仍active的观测列表。
    # @param completed_goals 已完成目标。
    # @param failed_goal_index 首失败索引。
    # @param failed_goal_report 失败目标已有报告。
    # @return None；异常本身不触发ROS或硬件操作。
    def __init__(self, message, still_active_components=(), completed_goals=(),
                 failed_goal_index=None, failed_goal_report=None):
        super().__init__(message)
        self.still_active_components = list(still_active_components)
        self.completed_goals = list(completed_goals)
        self.failed_goal_index = failed_goal_index
        self.failed_goal_report = failed_goal_report


##
# @brief 保留实际被拒MoveIt轨迹诊断，未修改目标/时间/摘要来争取通过。
class RejectedPlanError(RuntimeError):
    """Guarded action rejected the exact MoveIt trajectory sent to it."""

    ##
    # @brief 附带被拒原轨迹的有界诊断。
    # @param diagnostic 语义SHA/点/时刻及逐轴最大值映射。
    # @return None。
    def __init__(self, diagnostic):
        super().__init__("guarded action rejected MoveIt plan")
        self.diagnostic = diagnostic


##
# @brief 包装单次发送/执行首错与已采集goal_report供外层恢复和归档。
class GoalAttemptError(RuntimeError):
    ##
    # @brief 保存单次Goal失败报告。
    # @param message 失败文字。
    # @param goal_report 保留原UUID/轨迹SHA/反馈等证据。
    # @return None。
    def __init__(self, message, goal_report):
        super().__init__(message)
        self.goal_report = goal_report


##
# @brief 保留取消观察、Action结果及错误，不将acknowledged响应当物理停止。
class CancelAttemptError(RuntimeError):
    ##
    # @brief 保存取消首错和可能存在的终态。
    # @param message 错误文字。
    # @param result 可选Action结果。
    # @param cancel 可选请求/ack/位移触发上下文。
    # @return None。
    def __init__(self, message, result=None, cancel=None):
        super().__init__(message)
        self.result = result
        self.cancel = cancel


##
# @brief 从邻近转换器验证候选并取相对BENCH边界。
# @param limits_path NO_LOAD_DEV限制JSON。
# @param candidate_manifest 绑定清单。
# @return 六轴(min,max)tuple；只返回位置界，源验证异常传播。
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


##
# @brief 核完整六轴有限位置与选定参考界限。
# @param values 六轴rad序列，bool拒绝。
# @param limits 六行闭区间，默认历史ABSOLUTE_MODEL表。
# @return 六float tuple；不做碰撞、速度或许可验证。
def validate_target(values, limits=LIMITS):
    if len(values) != 6 or any(type(value) not in (float, int) or not math.isfinite(value)
                               for value in values):
        raise ValueError("six finite joint targets required")
    if any(not lower <= value <= upper for value, (lower, upper) in zip(values, limits)):
        raise ValueError("joint target outside selected reference limits")
    return tuple(float(value) for value in values)


##
# @brief 有界读取最多50个六轴目标。
# @param path JSON路径，stat与实际bytes均≤65536。
# @param limits 实际选定参考界限。
# @return 目标tuple；结构/数量/位置越界拒绝。
# 未在此固定源文件SHA或对重复JSON键做额外保护；多目标身份由后续Action/原始证据分别绑定。
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


##
# @brief 核MoveIt输出的六轴顺序、有限位置和末点。
# @param trajectory ROS JointTrajectory。
# @param target 可选目标rad，末点容差0.003rad。
# @param limits 选定位置范围。
# @return None；不符ValueError。
# 此局部函数不检查完整时间/v/a/jerk或独立碰撞重采样，不能把它称为完整可执行性校验；实际守护控制器还有独立准入。
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


##
# @brief 生成与C++约定一致的版本化语义轨迹SHA。
# @param trajectory 含header/joint_names/各点四数组及相对时间的消息。
# @return 64字符小写SHA；整数域、时间纳秒、字符串编码或非有限数错误拒绝。
# 采用DPSDK-JT-1、小端和长度前缀；−0归一0，避免ROS CDR分配padding导致不稳定摘要。这是内容绑定，不验证机械安全。
def trajectory_digest(trajectory):
    # Versioned semantic encoding; ROS CDR buffers may contain allocator padding.
    encoded = bytearray(b"DPSDK-JT-1\0")

    ##
    # @par 内部回调 u32
    # value须精确uint32，追加小端4字节到encoded，返回None。
    def u32(value):
        if type(value) is not int or not 0 <= value <= 0xffffffff:
            raise ValueError("trajectory integer outside uint32 range")
        encoded.extend(struct.pack("<I", value))

    ##
    # @par 内部回调 stamp
    # value含int32 sec及0≤nanosec<10^9，写入encoded并调用u32；返回None，不把ROS stamp当Host steady。
    def stamp(value):
        if type(value.sec) is not int or not -(1 << 31) <= value.sec < (1 << 31):
            raise ValueError("trajectory sec outside int32 range")
        encoded.extend(struct.pack("<i", value.sec))
        if type(value.nanosec) is not int or not 0 <= value.nanosec < 1_000_000_000:
            raise ValueError("trajectory nanosec outside range")
        u32(value.nanosec)

    ##
    # @par 内部回调 string
    # value为UTF-8字符串，先写字节数再正文；错误ValueError，返回None。
    def string(value):
        if not isinstance(value, str):
            raise ValueError("trajectory string required")
        try:
            raw = value.encode("utf-8")
        except UnicodeEncodeError as error:
            raise ValueError("trajectory string must be UTF-8") from error
        u32(len(raw))
        encoded.extend(raw)

    ##
    # @par 内部回调 doubles
    # values为数值序列，写长度及有限float64，−0规范为0；返回None，不在这里检验位置限位。
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


##
# @brief 保留原拒绝轨迹的有界点表/步长与逐轴峰值。
# @param trajectory 待归档原消息。
# @param serialized 该消息CDR字节，仅核容量并记录长度。
# @return 含语义SHA、点列表、time_deltas_ns和axis_maxima的dict。
# 最多512点/1MiB；Δtime只是记录，不在此纠正或判全部时序。缺速度/加速度数组的峰值为None，不能当零。
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


##
# @brief 沿显式异常cause链找原RejectedPlanError诊断。
# @param error 首异常，可为None。
# @return 原diagnostic或None，不跟随任意__context__。
def rejection_diagnostic_from_error(error):
    while error is not None:
        if isinstance(error, RejectedPlanError):
            return error.diagnostic
        error = error.__cause__
    return None


##
# @brief spin等待已有ROS future。
# @param node ROS节点。
# @param future 异步请求或Action future。
# @param timeout 等待秒数，由调用处剩余预算提供。
# @return future实际result；未done或future异常RuntimeError。
# 超时不cancel future，不证明远端服务未执行；生命周期请求因此必须在adapter.pending_transitions保留到收尾。
def wait(node, future, timeout):
    import rclpy
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception() is not None:
        raise RuntimeError("ROS request timed out or failed")
    return future.result()


##
# @brief 计算Host单调截止剩余秒数。
# @param deadline time.monotonic绝对秒。
# @return 正剩余秒；到期RuntimeError，不创建新预算。
def remaining_time(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError("ROS operation deadline expired")
    return remaining


##
# @brief ROS service适配器，拥有临时client和未结束生命周期transition列表；deadline为Host monotonic绝对秒。
class RosControlAdapter:
    ##
    # @brief 缓存ROS消息类型与共享截止，不立即发服务。
    # @param node 客户端所属ROS节点。
    # @param deadline 原Host绝对截止秒。
    # @return None；pending_transitions为空列表，后续保存(client,future)直到实际结束。
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

    ##
    # @brief 发现服务并发送请求；生命周期超时保留未结束客户端。
    # @param name 完整服务名。
    # @param service_type ROS服务类型。
    # @param request 请求消息。
    # @param deadline 可选绝对秒覆盖，缺省self.deadline。
    # @param transition true标记可能晚完成的状态变更。
    # @return 非None响应；不可用/超时/失败抛异常。
    # transition且future未done时finally不销毁client、不从pending删除；非transition请求finally销毁client。超时不能推导操作未发生。
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

    ##
    # @brief 逐个等待迟到生命周期future并销毁已结束client。
    # @param deadline 本次收尾共享Host绝对秒。
    # @return 所有future实际done为True，否则False；done但异常不等于转换成功，后续还需状态观测。
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

    ##
    # @brief 读取在线Gate固定身份参数。
    # @return 五个字符串字段dict；数量/类型不符拒绝，不在此重算设备固件hash。
    def gate_parameters(self):
        response = self.call("/device_action_gate/get_parameters", self.GetParameters,
                             self.GetParameters.Request(names=list(GATE_PARAMETERS)))
        if len(response.values) != len(GATE_PARAMETERS) or any(
                value.type != self.ParameterType.PARAMETER_STRING for value in response.values):
            raise RuntimeError("live gate identity parameters unavailable")
        return dict(zip(GATE_PARAMETERS, (value.string_value for value in response.values)))

    ##
    # @brief 查bootstrap_status精确ENABLED_HOLD。
    # @return success且文字精确相等True；只是Gate报告，不替代raw Hold证明。
    def confirmed_hold(self):
        response = self.call("/device_action_gate/bootstrap_status", self.Trigger,
                             self.Trigger.Request())
        return response.success and response.message == "ENABLED_HOLD"

    ##
    # @brief 原目标deadline内排新SetBenchPermit并等确认Hold。
    # @return None；非可重试状态、失效或到期RuntimeError。
    # 一旦queued成功不重复SetPermit，改轮询bootstrap；所有sleep与call沿原self.deadline，不拿30s会话租约替代目标/运动permit。
    def renew_motion_permit(self):
        set_reply = "NOT_ATTEMPTED"
        bootstrap_reply = "NOT_OBSERVED"

        ##
        # @par 内部回调 time_left
        # 无参数；返回self.deadline剩余秒。到期异常保留set_reply/bootstrap_reply类别，便于区分未排队与未确认，不重启时钟。
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

    ##
    # @brief 通过controller_manager发实际硬件生命周期转换。
    # @param name 硬件组件名。
    # @param state 调用者使用active/inactive；实现其它文本也选inactive ID，不能当任意状态解析器。
    # @param deadline 可选Host截止。
    # @return None；response.ok为false拒绝；此处不独立回读状态。
    def set_hardware_state(self, name, state, deadline=None):
        request = self.SetHardwareComponentState.Request(name=name)
        request.target_state.id = (self.State.PRIMARY_STATE_ACTIVE if state == "active"
                                   else self.State.PRIMARY_STATE_INACTIVE)
        request.target_state.label = state
        response = self.call("/controller_manager/set_hardware_component_state",
                             self.SetHardwareComponentState, request, deadline, transition=True)
        if not response.ok:
            raise RuntimeError(f"hardware {state} transition rejected: {name}")

    ##
    # @brief 请求硬件active。
    # @param name 组件名。
    # @return None；使用原adapter deadline。
    def activate_hardware(self, name):
        self.set_hardware_state(name, "active")

    ##
    # @brief 请求inactive并回读精确inactive。
    # @param name 组件名。
    # @return None；使用外部预设deactivation_deadline，未确认即失败。
    def deactivate_hardware(self, name):
        deadline = self.deactivation_deadline
        self.set_hardware_state(name, "inactive", deadline)
        if self.hardware_state(name, deadline) != "inactive":
            raise RuntimeError(f"hardware remains active: {name}")

    ##
    # @brief 列举硬件并要求唯一同名匹配。
    # @param name 组件名。
    # @param deadline 可选Host截止。
    # @return 唯一state.label或None；None表示缺失/重复，不当作inactive。
    def hardware_state(self, name, deadline=None):
        response = self.call("/controller_manager/list_hardware_components",
                             self.ListHardwareComponents, self.ListHardwareComponents.Request(), deadline)
        matches = [component for component in response.component if component.name == name]
        return matches[0].state.label if len(matches) == 1 else None

    ##
    # @brief STRICT切换单个控制器活动状态。
    # @param name 控制器名。
    # @param activate true激活，false失活。
    # @param deadline 可选Host截止。
    # @return None；response.ok=false拒绝。
    # 服务内timeout整数秒裁为1..5，但外层future仍受剩余Host预算限制，可能晚完成，故标transition保存。
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

    ##
    # @brief 实际请求单控制器激活。
    # @param name 控制器名。
    # @return None；使用adapter原截止。
    def activate_controller(self, name):
        self.switch_controller(name, True)

    ##
    # @brief 请求控制器inactive并回读确认。
    # @param name 控制器名。
    # @return None；使用deactivation_deadline，未知/active均失败。
    def deactivate_controller(self, name):
        deadline = self.deactivation_deadline
        self.switch_controller(name, False, deadline)
        if self.controller_state(name, deadline) != "inactive":
            raise RuntimeError(f"controller remains active: {name}")

    ##
    # @brief 列举控制器状态。
    # @param name 精确组件名。
    # @param deadline 可选Host截止。
    # @return 唯一匹配state字符串，否则None。
    def controller_state(self, name, deadline=None):
        response = self.call("/controller_manager/list_controllers", self.ListControllers,
                             self.ListControllers.Request(), deadline)
        matches = [controller for controller in response.controller if controller.name == name]
        return matches[0].state if len(matches) == 1 else None

    ##
    # @brief 请求Gate DisableAndRelease并等精确RELEASED_IDLE。
    # @return 精确success/RELEASED_IDLE为True；不可接受类别False，超时/调用失败异常。
    # 沿原stop_deadline，queued/pending后仅观察，不盲重发；RELEASE_UNAVAILABLE仅在允许bootstrap状态才重试。
    # 记录stop_phase和release/bootstrap类别供失败报告；这一service观察仍需raw停止/释放链独立核，不是物理停止认证。
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


##
# @brief 把在线Gate参数与显式激活/设备/候选目录精确比较。
# @param adapter ROS服务适配器。
# @param activation_sha256 期望激活原文件SHA。
# @param device_id 期望128bit设备ID文本。
# @param candidate_manifest 取其parent resolve为contract_bundle。
# @return None；任一字符串不符RuntimeError；不重算在线进程实际加载文件或固件。
def verify_live_gate(adapter, activation_sha256, device_id, candidate_manifest):
    values = adapter.gate_parameters()
    expected = {"activation_sha256": activation_sha256, "device_id": device_id,
                "contract_bundle": str(Path(candidate_manifest).parent.resolve()),
                "profile_name": "f405_6dof_bench", "guarded_controller": "arm_controller"}
    if any(values.get(key) != value for key, value in expected.items()):
        raise RuntimeError("live gate identity does not match pinned BENCH_SESSION inputs")


##
# @brief 将内部hardware标记映射真实硬件名。
# @param component hardware或控制器名。
# @return HARDWARE_NAME或原控制器字符串。
def component_name(component):
    return HARDWARE_NAME if component == "hardware" else component


##
# @brief 将停止回执文字压到已知有限类别。
# @param message 任意响应文字。
# @return 白名单原文字或OTHER，避免把任意服务文本误认成功。
def stop_reply_category(message):
    return message if isinstance(message, str) and message in STOP_REPLY_CATEGORIES else "OTHER"


##
# @brief 分类许可轮询状态及证据未完成前缀。
# @param message bootstrap响应文字。
# @return 已知许可类别、EVIDENCE_INCOMPLETE或OTHER。
def permit_status_category(message):
    if isinstance(message, str):
        if message in PERMIT_RETRY_STATUSES or message in PERMIT_FAILURE_STATUSES:
            return message
        if message.startswith("EVIDENCE_INCOMPLETE;"):
            return "EVIDENCE_INCOMPLETE"
    return "OTHER"


##
# @brief 逐个回读组件并分离active与未知。
# @param adapter 可调用状态服务的适配器。
# @param owned 待观察组件序列，此处不只限本调用者实际激活的组件。
# @param deadline 共享Host截止。
# @return (active真实名列表,unresolved真实名列表)；任何异常归未知而非静默inactive。
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


##
# @brief 先收敛迟到生命周期请求，再Gate释放，最后反向失活本调用者owned组件。
# @param adapter 已绑定在线Gate的适配器。
# @param owned 本调用者尝试激活的内部标记列表。
# @return (active,unknown)，成功时均空；任何未收敛/未释放/失活未知抛UnconfirmedStopError。
# settle、停止确认、失活各建立自己的5s Host预算，不能合称一个5s总期限。owned在发激活前登记，防超时晚完成未被清理；最终回读ALL_COMPONENTS而不只owned。
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


##
# @brief 给成功Action报告补独立release/失活收尾。
# @param adapter 已绑定适配器。
# @param owned 本次激活责任列表。
# @param report 原Action报告。
# @return 新dict含CONFIRMED_RELEASED_IDLE及组件列表；恢复失败异常。
def complete_success(adapter, owned, report):
    active, unknown = recover_after_failure(adapter, owned)
    return {**report, "stop_confirmation": "CONFIRMED_RELEASED_IDLE",
            "still_active_components": active, "unresolved_components": unknown}


##
# @brief 提取取消场景有限service观察并保留未合格标签。
# @param adapter 可能含stop_*阶段字段的适配器。
# @return release_observation及raw join要求dict，stop_confirmation固定UNCONFIRMED_STOP。
def cancel_release_context(adapter):
    return {"release_observation": {
                "request_reply": stop_reply_category(
                    getattr(adapter, "stop_release_reply", "NOT_OBSERVED")),
                "bootstrap_reply": stop_reply_category(
                    getattr(adapter, "stop_bootstrap_reply", "NOT_OBSERVED"))},
            "raw_stop_release_join": "REQUIRED_SEPARATELY",
            "safety_verdict": "NOT_QUALIFIED",
            "stop_confirmation": "UNCONFIRMED_STOP"}


##
# @brief 执行正常释放收尾，但保持取消证据未确认标签。
# @param adapter Gate适配器。
# @param owned 本次激活组件。
# @param report 取消Action报告。
# @return Action观察CANCELED但status/stop_confirmation仍UNCONFIRMED_STOP的报告。
# 即使Gate服务见RELEASED_IDLE，也不能将Action取消报告直接写成安全确认；原CAN Stop/Release必须由raw join。
def complete_cancel_observation(adapter, owned, report):
    observed = complete_success(adapter, owned, report)
    return {**observed, **cancel_release_context(adapter),
            "status": "UNCONFIRMED_STOP", "action_observation": "CANCELED"}


##
# @brief 同一会话顺序执行多个目标，失败即收尾并停止后续。
# @param adapter 已绑定Gate/生命周期适配器。
# @param owned 本调用者激活责任列表。
# @param targets 有界目标序列。
# @param perform_goal 接收(target,index)并返回报告的真实执行回调。
# @param per_goal_timeout 可选每目标Host预算秒，每轮开始重设adapter.deadline。
# @return 全部目标成功及最终释放报告；失败异常保留completed/failed索引与原报告。
# 第2目标开始前重新请求permit；所有成功UUID必须唯一。这里每目标预算可以刷新，但停止收尾独立且不掩盖原错误。
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


##
# @brief 只读要求已ENABLED_HOLD且硬件/两控制器均active。
# @param adapter ROS适配器。
# @return None；任一条件不符RuntimeError，不自动激活。
def verify_active_stack(adapter):
    if not adapter.confirmed_hold():
        raise RuntimeError("confirmed ENABLED_HOLD required before planning")
    if adapter.hardware_state(HARDWARE_NAME) != "active":
        raise RuntimeError(f"hardware is not active: {HARDWARE_NAME}")
    for name in CONTROLLERS:
        if adapter.controller_state(name) != "active":
            raise RuntimeError(f"controller is not active: {name}")


##
# @brief Hold确认后按硬件→broadcaster→arm激活并登记责任。
# @param adapter ROS适配器。
# @param owned 可选可变责任列表，仅把原inactive组件加入。
# @return None；失败传播，owned仍保留已尝试项供后续清理。
# 在服务发送前append，避免超时晚完成后遗失owner责任；原已active组件不属于本次owned，但最终收尾仍检查所有组件状态。
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


##
# @brief 等Action结果，超时后有界取消并再等真实终态。
# @param node ROS节点。
# @param handle 已接受goal handle。
# @param timeout 原结果等待秒。
# @return 正常结果；第一次超时即使后见终态也以RuntimeError保留失败；无终态UnconfirmedStopError。
# 恢复Cancel和再等结果共用新5s截止；只接受goals_canceling中同UUID，终态4/5/6不等于物理停止，仍需设备证据。
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


##
# @brief 把hex UUID转换ROS UUID消息。
# @param goal_id 调用者应提供32个hex字符。
# @return UUID；bytes.fromhex或消息类型校验错误传播，不生成随机ID。
def ros_goal_uuid(goal_id):
    from unique_identifier_msgs.msg import UUID
    return UUID(uuid=list(bytes.fromhex(goal_id)))


##
# @brief A运动时有界提交保留UUID的B以观察守护拒绝。
# @param node ROS节点。
# @param client 公开Action客户端。
# @param goal B复用的轨迹目标。
# @param deadline B提交/异常取消共用Host截止。
# @param goal_id B预留UUID。
# @return B拒绝/意外接受/身份错/不确定报告；意外接受尝试取消，不能据此称第二Begin未发。
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


##
# @brief 只在同UUID实际反馈有位移后发Cancel并等待CANCELED终态。
# @param node ROS节点。
# @param handle 已接受A handle。
# @param feedback 回调持续追加的样本list。
# @param motion_delta_rad 相对首有效actual样本最大轴位移门槛rad。
# @param deadline 共用Host截止秒。
# @param accepted_feedback_index 只看接受之后的反馈索引起点。
# @param overlap_probe 可选先执行B重叠观察回调。
# @param overlap_goal_id 用于B回调异常报告的UUID。
# @return (Action终态,取消上下文)；缺运动/ack/同UUIDCANCELED或中断失败CancelAttemptError。
# 只证明ROS观察；回执列goals_canceling匹配不等于设备Cancel COMMITTED，raw Stop新epoch链另核。
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


##
# @brief 在中断或异常接受后尽力取消并记录真实结果。
# @param node ROS节点。
# @param handle 已接受goal。
# @param result_future 可选已有结果future，缺省重新取得。
# @param stop_deadline 可选更早外部Host截止，裁小默认5s。
# @return 观察dict，所有取消/结果异常记字段而不伪装成功；不直接确认设备Stop。
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


##
# @brief 在原准入截止内只等待明确Pending状态到ENABLED_HOLD。
# @param adapter Gate适配器。
# @param deadline 原Host准入截止。
# @return 精确成功True，非允许状态False，到期异常；不重发未明原因目标。
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


##
# @brief 核被拒UUID有精确续租Pending见证。
# @param adapter Gate适配器。
# @param rejected_goal_id 被拒handle的16字节UUID对象。
# @param deadline 原Host准入截止。
# @return last_goal_rejection精确RENEWAL_PENDING:同UUID且success时True，否则False。
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


##
# @brief 专用硬件失活采集错误，保留触发witness和原结果，不由此类自动发恢复。
class HardwareDeactivationAttemptError(RuntimeError):
    """保留服务/Action首错与原触发上下文，不发恢复请求。"""
    ##
    # @brief 附带失活触发原上下文，不做后续修复。
    # @param message 服务/Action首错。
    # @param trigger 原Host截止/witness/root映射。
    # @param result 可选Action结果。
    # @return None。
    def __init__(self, message, trigger, result=None):
        super().__init__(message)
        self.trigger = trigger
        self.result = result


##
# @brief 核可信wrapper提供的当前Run typed witness，不从位移猜Run。
# @param witness 同goal的root、两State、Status与after_begin Host锚映射。
# @param goal_id 精确Action UUID。
# @param now_us 当前Host monotonic µs。
# @return root副本；错身份、旧/未来时刻、非Run5、缺有效permit或Source不推进拒绝。
# State250ms、Status1s Host年龄门槛；produced_at_us只在设备域比较前进，不与now_us绝对相减。本函数信任wrapper来源前提，不验证任意手写dict原字节签名。
def validate_hardware_deactivation_witness(witness, goal_id, now_us):
    """仅可信采集 wrapper 可提供原 v9 byte/source-bound live witness。

    when/after_begin_us 为 Host monotonic 微秒；produced_at_us 属 Device Port 域。
    此函数不从位移猜 RUN，不读取设备，也不签 stop/资格。
    """
    ##
    # @par 内部回调 require
    # ok为当前条件、reason为具体前提；不符抛带STOP trigger witness前缀RuntimeError，满足返回None。
    def require(ok, reason):
        if not ok:
            raise RuntimeError('STOP trigger witness: ' + reason)
    require(type(now_us) is int and now_us > 0 and isinstance(witness, dict), 'missing current witness')
    require(witness.get('goal_id') == goal_id, 'goal identity mismatch')
    marker = witness.get('after_begin_us')
    require(type(marker) is int and 0 < marker <= now_us, 'afterBegin missing/future')
    root = witness.get('root')
    keys = ('boot_epoch', 'session_id', 'reference_epoch', 'motion_epoch',
            'management_epoch', 'profile_id', 'configured_mask', 'run_id')
    require(isinstance(root, dict) and all(type(root.get(k)) is int and root[k] > 0 for k in keys), 'root unknown')
    require(root['profile_id'] == 201 and root['configured_mask'] == 63, 'profile/mask mismatch')
    states = witness.get('states')
    require(isinstance(states, list) and len(states) == 2, 'two current State rows required')
    for row, budget in [(states[0], 250000), (states[1], 250000), (witness.get('status'), 1000000)]:
        require(isinstance(row, dict) and type(row.get('when')) is int
                and marker <= row['when'] <= now_us and now_us-row['when'] <= budget, 'stale/future/preBegin row')
        fields = row.get('fields')
        require(isinstance(fields, dict) and all(type(fields.get(k)) is int and fields[k] == root[k] for k in keys if k != 'run_id'), 'current full root mismatch')
        require(type(fields.get('core_state')) is int and fields['core_state'] == 5
                and type(fields.get('first_failure_reason')) is int and fields['first_failure_reason'] == 0, 'not active RUN')
    first, last = (row['fields'].get('produced_at_us') for row in states)
    require(type(first) is int and type(last) is int and 0 < first < last
            and states[0]['when'] < states[1]['when'], 'State source clock did not advance')
    status = witness['status']['fields']
    require(type(status.get('run_id')) is int and status['run_id'] == root['run_id']
            and status.get('attempt_id') == root['run_id']
            and status.get('attempt_kind') == 1 and status.get('parent_run_id') == 0
            and status.get('final_state') == 0 and status.get('evidence_lost') == 0, 'not current active Motion run')
    require(type(status.get('real_motion_permitted')) is int and status['real_motion_permitted'] == 1
            and type(status.get('permit_remaining_us')) is int and status['permit_remaining_us'] > 0, 'permit snapshot absent')
    return dict(root)


##
# @brief 可信Run witness与实际位移均成立后发两个原生命周期失活服务。
# @param node ROS节点。
# @param handle A的已接受handle。
# @param feedback 同goal有限实际反馈。
# @param deadline 目标原Host截止。
# @param accepted_feedback_index 接受之后的反馈起索引。
# @param adapter 必须由可信wrapper注入hardware_deactivation_witness hook。
# @return (ABORTED6结果,原触发上下文)；缺hook或服务/结果错误保留首错异常。
# 先停arm_controller再hardware，共用一次采锚的最多5s截止；不发Cancel/新授权、不调用普通恢复把原失败洗成停止成功。
def await_mid_motion_hardware_deactivation(node, handle, feedback, deadline,
                                          accepted_feedback_index, adapter):
    """测试采集：两原生命周期服务，共享原5s；不发 Cancel/新授权。

    hook 必须来自固定可信 wrapper，返回上方原 typed witness 或 None（尚未就绪）。
    缺 hook/错身份/失鲜为硬拒；None 仅在原45s及5s预留内等待。
    """
    hook = getattr(adapter, 'hardware_deactivation_witness', None)
    if not callable(hook):
        raise RuntimeError('STOP capture requires trusted live typed witness hook')
    goal_id = bytes(handle.goal_id.uuid).hex()
    result_future = handle.get_result_async()
    while True:
        if result_future.done():
            raise RuntimeError('action finished before hardware deactivation trigger')
        remaining_time(deadline - STOP_CONFIRM_TIMEOUT)
        selected = []
        for index, sample in enumerate(feedback[accepted_feedback_index:], accepted_feedback_index):
            if sample.get('goal_id') != goal_id:
                raise RuntimeError('STOP feedback goal identity mismatch')
            actual = sample.get('actual_rad')
            if not isinstance(actual, list) or len(actual) != 6 or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in actual):
                raise RuntimeError('STOP feedback missing finite six axes')
            selected.append((index, actual))
        if len(selected) >= 2 and max(abs(a-b) for a, b in zip(selected[0][1], selected[-1][1])) >= MID_MOTION_DELTA_RAD:
            witness = hook(goal_id)
            if witness is not None:
                now_ns = time.monotonic_ns()
                root = validate_hardware_deactivation_witness(witness, goal_id, now_ns//1000)
                if result_future.done():
                    raise RuntimeError('action finished before hardware deactivation service')
                # 同一Host采锚导出原5s整数微秒截止，不能由第二次采时扩窗。
                stop_deadline_us = min(int(deadline * 1000000), now_ns//1000 + int(STOP_CONFIRM_TIMEOUT * 1000000))
                stop_deadline = stop_deadline_us / 1000000
                trigger = dict(goal_id=goal_id, requested_steady_us=now_ns//1000,
                               outer_deadline_steady_us=stop_deadline_us,
                               root=root, live_witness=witness,
                               motion_delta_threshold_rad=MID_MOTION_DELTA_RAD,
                               motion_feedback_start_index=selected[0][0],
                               motion_feedback_trigger_index=selected[-1][0],
                               device_stop_confirmation='REQUIRED_SEPARATELY')
                # 不调用普通 complete_success/recovery，也不把 ROS service ok 当 Device 确认。
                result = None
                try:
                    adapter.switch_controller('arm_controller', False, stop_deadline)
                    adapter.set_hardware_state(HARDWARE_NAME, 'inactive', stop_deadline)
                    result = wait(node, result_future, remaining_time(stop_deadline))
                    if result is None or result.status != 6:
                        raise RuntimeError('hardware deactivation did not retain ABORTED6 action')
                except BaseException as error:
                    raise HardwareDeactivationAttemptError(
                        str(error) or type(error).__name__, trigger, result) from error
                return result, trigger
        import rclpy
        rclpy.spin_once(node, timeout_sec=min(0.05, remaining_time(deadline - STOP_CONFIRM_TIMEOUT)))


##
# @brief 请求真实MoveIt计划并原样通过公开守护Action提交。
# @param node ROS节点。
# @param planner GetMotionPlan服务client。
# @param client FollowJointTrajectory client。
# @param target 六轴目标rad。
# @param limits 实际参考边界。
# @param deadline 本目标原Host总截止。
# @param admission_adapter 可选Gate适配器供续租拒绝见证/测试hook。
# @param cancel_mid_motion 实际位移后取消A。
# @param preempt_probe A位移后提交B观察再取消A。
# @param preempt_goal_ids A/B/C保留UUID映射。
# @param goal_uuid_hex 可选单目标保留UUID。
# @param submission_deadline 可选更早提交截止，只裁规划/准入，不改变后续执行总deadline。
# @param hardware_deactivate_mid_motion 专用可信witness失活模式，与普通取消收尾区分。
# @return ROS范围报告；任何失败包装GoalAttemptError保留原计划摘要、反馈和首错。
# MoveIt start_state.is_diff使用服务端当前状态，规划缩放0.1、目标容差0.001rad；局部validate_plan不是完整时间/碰撞验证。
# 最多8次续租特例重试，要求无反馈/错误且精确UUID见证，preempt禁重试；accepted后异常取消仅覆盖明确分支，不能宣称所有异常均已停止。
def execute_goal(node, planner, client, target, limits, deadline, admission_adapter=None,
                 cancel_mid_motion=False, preempt_probe=False, preempt_goal_ids=None,
                 goal_uuid_hex=None, submission_deadline=None, hardware_deactivate_mid_motion=False):
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

    ##
    # @par 内部回调 on_feedback
    # message为Action反馈；核六轴名称及两位置数组长度，最多1024条。取消/抢占/失活模式额外核所有数值有限并记录UUID；普通模式此回调未额外核有限性。返回None，错误仅记首条。
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
        if (cancel_mid_motion or preempt_probe or hardware_deactivate_mid_motion) and any(
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
                            if cancel_mid_motion or preempt_probe or hardware_deactivate_mid_motion else {})})

    goal_id = None
    result = None
    cancel_report = None
    deactivation_report = None
    accepted_feedback_index = None
    handle = None
    probe_ids = preempt_goal_ids or {name: uuid.uuid4().hex for name in "ABC"} \
        if preempt_probe else None

    ##
    # @par 内部回调 probe_goals
    # 无参数；从当前result/cancel_report派生A观察、B重叠报告与C未提交占位dict；不启动C或构造raw第二Begin证明。
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
        if hardware_deactivate_mid_motion:
            result, deactivation_report = await_mid_motion_hardware_deactivation(
                node, handle, feedback, deadline, accepted_feedback_index, admission_adapter)
        elif cancel_mid_motion or preempt_probe:
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
        if not (cancel_mid_motion or preempt_probe or hardware_deactivate_mid_motion) and (result.status != GoalStatus.STATUS_SUCCEEDED or
                                      result.result.error_code != 0):
            raise RuntimeError(f"guarded action failed: status={result.status} "
                               f"code={result.result.error_code}")
    except BaseException as error:
        if isinstance(error, HardwareDeactivationAttemptError):
            result = error.result
            deactivation_report = error.trigger
        if isinstance(error, CancelAttemptError):
            result = error.result
            cancel_report = error.cancel
        interrupted = None
        if goal_id is not None and not hardware_deactivate_mid_motion:
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
        if hardware_deactivate_mid_motion:
            failed.update(hardware_deactivation=deactivation_report,
                          raw_stop_release_join="REQUIRED_SEPARATELY")
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
            "status": "UNCONFIRMED_STOP" if hardware_deactivate_mid_motion else
                      "CANCELED" if cancel_mid_motion or preempt_probe else "SUCCEEDED",
            **({"hardware_deactivation": deactivation_report}
               if hardware_deactivate_mid_motion else {}),
            "feedback": feedback,
            "device_can_evidence": "REQUIRED_SEPARATELY",
            "device_qualification": "NOT_CLAIMED",
            **({"cancel": cancel_report, "device_stop_evidence": "REQUIRED_SEPARATELY",
                "device_release_evidence": "REQUIRED_SEPARATELY"}
               if cancel_mid_motion or preempt_probe else {}),
            **({"preempt_goals": probe_goals()} if preempt_probe else {})}


##
# @brief 编排单目标及可选取消/抢占恢复/硬件失活采集。
# @param target A目标rad。
# @param timeout Host总预算秒。
# @param limits 位置界限。
# @param activate_controller 是否允许激活原inactive组件。
# @param activation_sha256 预期激活文件SHA。
# @param device_id 预期设备ID文本。
# @param candidate_manifest 候选manifest，取parent绑定在线Gate。
# @param guarded_session true先核Gate/Hold及active栈。
# @param cancel_mid_motion A实际运动后取消。
# @param preempt_probe B重叠拒绝观察，C不提交。
# @param preempt_goal_ids 保留A/B/C UUID。
# @param preempt_then_goal A取消/B拒绝后在新permit下提交C。
# @param post_cancel_target C目标，须显式给。
# @param hardware_deactivate_mid_motion 可信wrapper专用失活捕获，最多45s且不能混其它模式。
# @return 范围受限报告；已绑定后按原失败进入恢复，未确认收尾抛UnconfirmedStopError。
# 普通capture无guarded_session也会发Action，不能把false叫dry-run。抢占C提交须在取消观察后额外4s窗内重新permit/规划/准入；其执行仍受原总deadline。
# 硬件失活模式刻意不走普通恢复；ROS teardown错误保留取消未确认状态而不覆盖成成功。
def capture(target, timeout, limits, activate_controller=False, activation_sha256=None,
            device_id=None, candidate_manifest=None, guarded_session=False,
            cancel_mid_motion=False, preempt_probe=False, preempt_goal_ids=None,
            preempt_then_goal=False, post_cancel_target=None, hardware_deactivate_mid_motion=False):
    if hardware_deactivate_mid_motion and (not guarded_session or cancel_mid_motion or
            preempt_probe or preempt_then_goal or not 0 < timeout <= 45.0):
        raise ValueError("hardware deactivation capture requires one guarded goal and timeout <= 45s")
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
            if hardware_deactivate_mid_motion and not callable(
                    getattr(adapter, "hardware_deactivation_witness", None)):
                raise RuntimeError("STOP capture requires trusted live typed witness hook before services")
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
                              **({"hardware_deactivate_mid_motion": True}
                                 if hardware_deactivate_mid_motion else
                                 {"preempt_probe": True,
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
        if hardware_deactivate_mid_motion:
            return {**report, "scope": "ROS_HARDWARE_DEACTIVATION_CAPTURE",
                    "qualification": "NOT_QUALIFIED", "raw_stop_release_join": "REQUIRED_SEPARATELY"}
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
        if hardware_deactivate_mid_motion:
            # 采集首失败不可用后续恢复洗成 STOP 成功；安全恢复由 Root 独立记录。
            raise UnconfirmedStopError(
                str(error) or type(error).__name__,
                failed_goal_report=getattr(error, "goal_report", None) or report) from error
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


##
# @brief 在同一ROS节点/会话依次执行有界目标并最终释放。
# @param targets 1..50个六轴目标。
# @param timeout 每目标预算秒，有限且≤120。
# @param limits 选定BENCH位置界。
# @param activate_controller 是否激活本调用责任组件。
# @param activation_sha256 激活期望SHA。
# @param device_id 设备ID。
# @param candidate_manifest 候选目录身份。
# @return 多目标报告；setup失败先恢复，sequence_started后由run_guarded_sequence负责原失败收尾。
# 每目标重设执行预算、目标间新permit；lambda perform_goal只调用execute_goal，不从目标数量推导硬件资格。
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


##
# @brief 解析实际执行模式并归档有界Action/错误证据。
# @return 仅SUCCEEDED且非cancel_mid_motion/preempt_probe时0，其它1；参数错误argparse退出。
# 无dry-run：ABSOLUTE_MODEL不绑定BENCH输入，BENCH要求候选/limits/非零激活SHA/device，并可--activate-controller。
# 抢占probe报告一律UNCONFIRMED_STOP/NOT_QUALIFIED；硬件失活需外部可信hook，直接CLI没有hook会在服务动作前拒绝。
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
    parser.add_argument("--hardware-deactivate-mid-motion", action="store_true",
                        help="测试采集模式；必须由可信 wrapper 安装 live typed witness hook")
    parser.add_argument("--cancel-mid-motion", action="store_true")
    parser.add_argument("--preempt-probe", action="store_true")
    parser.add_argument("--preempt-then-goal", action="store_true")
    parser.add_argument("--post-cancel-target", nargs=6, type=float, metavar="RAD")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be positive")
    if args.hardware_deactivate_mid_motion and (args.reference_frame != "BENCH_SESSION" or
            args.targets_json is not None or args.cancel_mid_motion or args.preempt_probe or
            args.preempt_then_goal or args.timeout > 45.0):
        parser.error("hardware deactivation capture requires one BENCH_SESSION goal, timeout <= 45s, exclusive mode")
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
                              **({"hardware_deactivate_mid_motion": True}
                                 if args.hardware_deactivate_mid_motion else
                                 {"preempt_then_goal": True,
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
