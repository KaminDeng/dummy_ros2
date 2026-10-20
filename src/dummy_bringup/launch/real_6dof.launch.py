##
# @file real_6dof.launch.py
# @brief 对显式绑定产物与设备身份做预检的无负载六轴开发入口。
# 作用域固定 NO_LOAD_DEV、资格为 UNQUALIFIED；启动前验证外部固定 activation 摘要、候选清单、文件路径、USB 标识和绑定 ROS 限制。随后创建 gate 与可选完整 ROS 图，硬件和控制器初始 inactive；创建节点不等于完成运行时授权、参考建立、使能或运动。
"""显式绑定的无负载六轴开发入口；默认不使能运动。"""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import secrets
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

import yaml


##
# 路径清单要求恰好这些文件角色及目录角色，不允许用未登记额外键暗示另一产物；manifest内容的深入一致性还由预检器、限制转换器及gate分层校验。
FILE_KEYS = (
    "prebuild_bundle_manifest_path", "compiled_policy_manifest_path",
    "firmware_image_path", "elf_path", "host_executable_path", "map_file_path",
    "resource_report_path", "d_regression_report_path",
    "d_regression_provenance_path", "hil_report_path", "hil_raw_index_path",
)
DIRECTORY_KEYS = ("candidate_bundle_root", "source_bundle_root",
                  "resource_base_root", "hil_evidence_root")
HEX_SHA = re.compile(r"[0-9a-f]{64}\Z")
HEX_ID = re.compile(r"[0-9a-f]{32}\Z")
STM32_ENDPOINT = re.compile(
    r"serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_"
    r"Virtual_ComPort_in_FS_Mode_([A-Za-z0-9]+)-if00#baudrate=115200\Z"
)
##
# 时间单位均为µs：默认250ms，显式诊断500ms；两者都不是许可剩余时间，也不代表测试/资格完成。
NO_LOAD_STATE_MAX_AGE_US = 250000
DIAGNOSTIC_STATE_MAX_AGE_US = 500000


##
# @brief 从已验证的限制对象取得门控许可与会话租期。
# 不自造更长租约、不从 launch 默认常量覆盖已验证限制；设备是否授予仍由运行时门控协议决定。
# @param validated_limits convert_application_ros_limits.validate_bound 返回的绑定校验对象。
# @return dict：permit_remaining_us 和 requested_session_lease_us，单位均为 µs。
def gate_lease_parameters(validated_limits):
    lease = validated_limits.gate_lease
    return {"permit_remaining_us": lease.permit_remaining_us,
            "requested_session_lease_us": lease.requested_session_lease_us}


##
# @brief 只接受默认新鲜度或明确诊断新鲜度参数。
# 其他格式/数值抛 ValueError。500 ms 作为 UNQUALIFIED_DIAGNOSTIC 记录，不据此升级开发资格或延长设备许可。
# @param value 精确文本 250000 或 500000，单位 µs。
# @return int：250000（250 ms）或 500000（500 ms）。
def validate_host_state_max_age_us(value):
    if value not in (str(NO_LOAD_STATE_MAX_AGE_US), str(DIAGNOSTIC_STATE_MAX_AGE_US)):
        raise ValueError("host_state_max_age_us must be 250000 or explicit diagnostic 500000")
    return int(value)


##
# @brief 校验独立停车确认总预算的受控选项。
# 超出白名单、带其他格式或布尔值均抛 ValueError；此预算与反馈年龄及会话租期是不同参数。
# @param value Python 精确 int 或 str，仅允许 0/300000，不接受 bool。
# @return int：本函数允许0或300000；300000表示300ms确认预算。0在此仅为兼容输入，不能证明下游已回退：当前Xacro显式输出0，而transport解析器拒绝显式0。
def validate_stop_confirmation_timeout_us(value):
    # 保留既有0/300000输入白名单；运行期可接受性还须核Xacro输出和transport的非零参数校验。
    if type(value) is int and value in (0, 300000):
        return value
    if type(value) is str and value in ("0", "300000"):
        return int(value)
    raise ValueError("stop_confirmation_timeout_us must be 0 or approved 300000")


##
# @brief 构建 JSON 对象并拒绝重复键造成的隐式覆盖。
# 重复键抛 ValueError；作为 object_pairs_hook 也作用于嵌套 JSON 对象。
# @param pairs json.loads 传入的有序键值对序列。
# @return dict：所有键仅出现一次的映射。
def _unique_pairs(pairs):
    values = {}
    for key, value in pairs:
        if key in values:
            raise ValueError("duplicate JSON key")
        values[key] = value
    return values


##
# @brief 读取末级路径为普通文件且非符号链接的内容。
# 缺文件、非普通文件或末级符号链接抛 ValueError；不是对所有父目录及并发替换的完整原子防护。
# @param path 待读取文件路径，可转换为 Path。
# @return bytes：原始文件内容，供精确 SHA-256 和 JSON 解析使用。
def _read_bytes(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"missing regular file: {path}")
    return path.read_bytes()


##
# @brief 解析拒绝重复键的 JSON 并要求顶层对象。
# 数组/标量或重复键会拒绝；调用方之后还要核对具体 schema、字段和身份。
# @param data JSON 原始 bytes 或可由 json.loads 接受的文本。
# @param path 报错时标识输入来源的路径。
# @return dict：解析所得 JSON 顶层对象。
def _parse_object(data, path):
    value = json.loads(data, object_pairs_hook=_unique_pairs)
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


##
# @brief 按普通文件约束读取一个严格 JSON 对象。
# @param path 输入文件路径。
# @return dict：_read_bytes 和 _parse_object 连续验证后的对象。
def _read_object(path):
    return _parse_object(_read_bytes(path), path)


##
# @brief 验证外部钉住的开发activation、候选身份和明确设备路径。
# 要求路径清单键集完全一致，文件/目录为绝对且末级非符号链接。核对 activation 原始字节摘要、NO_LOAD_DEV/UNQUALIFIED 作用域与候选 manifest 摘要；候选必须 dummy-wire-v2.4、runtime_enabled=False、profile 201、mask 63。设备非零32位小写ID、by-id STM32端点串号和activation字段须匹配后才调用 USB verifier；不在此授予电机控制。
# @param paths_file 部署路径清单 JSON 文件，schema 必须为 no_load_dev_deployment_paths_v1。
# @param expected_activation_sha256 外部给定非零 64 位小写 SHA-256，不能由待验证文件自我决定。
# @param usb_verifier 可调用对象，接收 endpoint/usb_serial；生产入口传实际本机 STM32 USB 预检器。
# @return tuple[dict,dict]：验证后的 paths 和 activation。
def load_deployment(paths_file, expected_activation_sha256, usb_verifier):
    if not isinstance(expected_activation_sha256, str) or not HEX_SHA.fullmatch(
            expected_activation_sha256) or expected_activation_sha256 == "0" * 64:
        raise ValueError("external activation SHA-256 pin required")
    paths = _read_object(paths_file)
    expected_keys = set(FILE_KEYS) | set(DIRECTORY_KEYS) | {
        "schema", "activation_path", "device_id", "endpoint", "usb_serial",
    }
    if paths.get("schema") != "no_load_dev_deployment_paths_v1" or set(paths) != expected_keys:
        raise ValueError("deployment paths schema mismatch")
    for key in FILE_KEYS + ("activation_path",):
        value = paths[key]
        if not isinstance(value, str) or not Path(value).is_absolute():
            raise ValueError(f"{key}: absolute path required")
        if Path(value).is_symlink() or not Path(value).is_file():
            raise ValueError(f"{key}: regular file required")
    for key in DIRECTORY_KEYS:
        value = paths[key]
        if not isinstance(value, str) or not Path(value).is_absolute():
            raise ValueError(f"{key}: absolute directory required")
        if Path(value).is_symlink() or not Path(value).is_dir():
            raise ValueError(f"{key}: directory required")
    activation_path = Path(paths["activation_path"])
    activation_bytes = _read_bytes(activation_path)
    if hashlib.sha256(activation_bytes).hexdigest() != expected_activation_sha256:
        raise ValueError("activation SHA-256 mismatch")
    activation = _parse_object(activation_bytes, activation_path)
    candidate_path = Path(paths["candidate_bundle_root"]) / "manifest.json"
    candidate_bytes = _read_bytes(candidate_path)
    candidate = _parse_object(candidate_bytes, candidate_path)
    if (activation.get("schema") != "dummy_no_load_dev_activation_v1" or
            activation.get("scope") != "NO_LOAD_DEV" or
            activation.get("basis") not in ("NO_LOAD_DEV_FUNCTIONAL", "NO_LOAD_DEV_FUNCTIONAL_LIMITED") or
            activation.get("qualification") != "UNQUALIFIED" or
            activation.get("candidate_manifest_sha256") != hashlib.sha256(
                candidate_bytes).hexdigest() or
            candidate.get("format") != "dummy_wire_release_v2" or
            candidate.get("release_id") != "dummy-wire-v2.4" or
            candidate.get("runtime_enabled") is not False or
            candidate.get("profile_id") != 201 or candidate.get("configured_mask") != 63):
        raise ValueError("unqualified no-load candidate identity mismatch")
    match = (STM32_ENDPOINT.fullmatch(paths["endpoint"])
             if isinstance(paths["endpoint"], str) else None)
    if (not isinstance(paths["device_id"], str) or
            not HEX_ID.fullmatch(paths["device_id"]) or
            paths["device_id"] == "0" * 32 or match is None or
            match.group(1) != paths["usb_serial"] or
            any(activation.get(key) != paths[key] for key in
                ("device_id", "endpoint", "usb_serial"))):
        raise ValueError("deployment device or USB identity mismatch")
    ##
    # 此处才调用注入的本机USB验证器；离线测试注入假实现，真实启动会核实实际端点/串号，但仍未获得控制许可。
    usb_verifier(paths["endpoint"], paths["usb_serial"])
    return paths, activation


##
# @brief 从安装区显式路径加载开发预检/限制转换模块。
# 导入执行模块顶层代码；spec/loader 缺失时给出 ValueError，不搜索源码或其他构建副本。
# @param name 本次动态导入模块名。
# @param path 已安装 Python 文件路径。
# @return module：成功执行导入的模块。
def _load_script(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ValueError(f"missing installed development verifier: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


##
# @brief 核对转换所得 controller_manager 的保护性装配约定。
# 要求 update_rate=50 Hz、唯一初始 inactive 硬件名 dummy_ref_real_6dof、arm_controller 使用 GuardedJointTrajectoryController，状态广播器类型正确；只是配置验证，不执行 activate。
# @param manager_params ros2_controllers.yaml 中 controller_manager.ros__parameters 字典。
# @return bool：符合所有约束返回 True，否则抛 ValueError。
def validate_controller_config(manager_params):
    if (manager_params.get("update_rate") != 50 or
            manager_params.get("hardware_components_initial_state", {}).get("inactive") !=
            ["dummy_ref_real_6dof"] or
            manager_params.get("arm_controller", {}).get("type") !=
            "dummy_ros_control/GuardedJointTrajectoryController" or
            manager_params.get("joint_state_broadcaster", {}).get("type") !=
            "joint_state_broadcaster/JointStateBroadcaster"):
        raise ValueError("development controller or inactive hardware mismatch")
    return True


##
# @brief 按模式选择 gate 最小图或完整显示/控制器/规划图。
# 两种模式都不在此直接激活硬件或发送关节目标。
# @param mode gate_only 或 full，其他文本拒绝。
# @param gate_actions gate 节点和 gate 退出联动关闭处理。
# @param full_actions 状态发布器、inactive spawner 和 MoveIt 启动动作。
# @return list：复制 gate 动作；full 时再追加完整图动作。
def select_startup_actions(mode, gate_actions, full_actions):
    if mode == "gate_only":
        return list(gate_actions)
    if mode == "full":
        return list(gate_actions) + list(full_actions)
    raise ValueError("startup_mode must be gate_only or full")


##
# @brief 检查一次性原始证据输出路径位于既有报告目录之下。
# 拒绝空/点/父目录路径段、控制字符、超过256个UTF-8字节、根目录为/、文件等于根或根外路径。检查根与输出父目录祖先无符号链接、目录已存在且目标从未存在（包括断链）；这里只验证不创建输出文件。
# @param path 可空绝对输出文件路径；空串表示关闭此可选输出。
# @param report_root 非空输出时必须提供的绝对报告根目录。
# @return str：通过验证的原路径，或空串。
def validate_raw_evidence_path(path, report_root):
    if not path:
        return ""
    if not report_root:
        raise ValueError("raw evidence report root required")
    for label, value in (("raw evidence path", path), ("report root", report_root)):
        if (not isinstance(value, str) or not value.startswith("/") or
                any(part in ("", ".", "..") for part in value[1:].split("/")) or
                any(char in value for char in "\x00\n\r\t")):
            raise ValueError(f"{label}: canonical absolute path required")
    if len(path.encode("utf-8")) > 256:
        raise ValueError("raw evidence path exceeds transport limit")
    target, root = Path(path), Path(report_root)
    if root == Path("/") or target == root or not target.is_relative_to(root):
        raise ValueError("raw evidence path must be beneath report root")
    for candidate in (root, *root.parents, target.parent,
                      *target.parent.parents):
        if candidate.is_symlink():
            raise ValueError(f"raw evidence symlink component: {candidate}")
    if not root.is_dir() or not target.parent.is_dir():
        raise ValueError("raw evidence report directory missing")
    if os.path.lexists(path):
        raise ValueError("raw evidence destination already exists")
    return path


##
# @brief 校验一次性目标时间线输出并与原始证据文件区分。
# 复用原始证据路径的目录、格式、链接和不存在性约束；时间线与原始transport包含不同证据，不能写入同一文件。
# @param path 可空时间线绝对路径。
# @param report_root 同一报告根目录。
# @param raw_evidence_path 已验证原始证据路径，不得与时间线相同。
# @return str：独立输出路径或空串。
def validate_goal_timeline_path(path, report_root, raw_evidence_path):
    if not path:
        return ""
    validated = validate_raw_evidence_path(path, report_root)
    if validated == raw_evidence_path:
        raise ValueError("goal timeline and raw evidence paths must differ")
    return validated


##
# @brief 把绑定身份、设备限制和时间预算转换为 xacro 文本映射。
# 逐轴限制用逗号连接，保留设备域数值；max_state_age_us 与 heartbeat_stop_budget_us 传同一已选年龄预算。历史参数名 acceleration_max_rad_s 的实际值单位是 rad/s²，名称不在此重写。返回映射不产生会话许可或在线身份确认。
# @param joints 六轴已验证限制字典，含 device_* rad/rad/s/rad/s² 值。
# @param activation 已验证 activation，提供契约、模型、profile、固件和编译策略摘要。
# @param endpoint 明确 by-id STM32 端点。
# @param device 非零32位小写设备 ID。
# @param client 本次非零32位小写客户端实例 ID。
# @param control_period_us 设备命令周期，µs。
# @param state_publish_period_us 设备状态发布周期，µs。
# @param raw_evidence_path 可选已验证原始transport输出路径。
# @param host_state_max_age_us 宿主新鲜度上限，仅250000/500000 µs。
# @param stop_confirmation_timeout_us 独立确认总预算，仅0/300000 µs。
# @return dict[str,str]：供设备 URDF/xacro 插件使用的字符串参数。
def hardware_mappings(joints, activation, endpoint, device, client,
                      control_period_us, state_publish_period_us,
                      raw_evidence_path="", host_state_max_age_us=NO_LOAD_STATE_MAX_AGE_US,
                      stop_confirmation_timeout_us=0):
    host_state_max_age_us = validate_host_state_max_age_us(str(host_state_max_age_us))
    stop_confirmation_timeout_us = validate_stop_confirmation_timeout_us(
        stop_confirmation_timeout_us)
    mappings = {
        "endpoint": endpoint, "device_id": device,
        "client_instance_id": client, "profile_id": "201", "configured_mask": "63",
        "contract_sha256": activation["source_contract_sha256"],
        "model_sha256": activation["model_sha256"],
        "profile_sha256": activation["live_profile_sha256"],
        "firmware_sha256": activation["firmware_sha256"],
        "compiled_policy_sha256": activation["compiled_policy_sha256"],
        "position_min_rad": ",".join(str(joint["device_position_min_rad"]) for joint in joints),
        "position_max_rad": ",".join(str(joint["device_position_max_rad"]) for joint in joints),
        "velocity_max_rad_s": ",".join(str(joint["device_velocity_max_rad_s"]) for joint in joints),
        "acceleration_max_rad_s": ",".join(
            str(joint["device_acceleration_max_rad_s2"]) for joint in joints),
        "control_period_us": str(control_period_us),
        "state_publish_period_us": str(state_publish_period_us),
        "max_state_age_us": str(host_state_max_age_us),
        "heartbeat_stop_budget_us": str(host_state_max_age_us),
        "stop_confirmation_timeout_us": str(stop_confirmation_timeout_us),
    }
    if raw_evidence_path:
        mappings["raw_evidence_path"] = raw_evidence_path
    return mappings


##
# @brief 按固定六轴顺序生成与设备限制分开的规划位置域。
# 不把规划内收范围替换为设备硬限位；轴序不匹配抛 ValueError。
# @param joints 已验证限制中的六个关节字典，名称必须 Joint1..Joint6。
# @return dict[str,str]：JointN_lower/upper，对应 planning_position_min/max_rad。
def planning_mappings(joints):
    if [joint.get("name") for joint in joints] != [f"Joint{i}" for i in range(1, 7)]:
        raise ValueError("six-axis planning order mismatch")
    return {
        f"{joint['name']}_{suffix}": str(joint[field])
        for joint in joints
        for suffix, field in (
            ("lower", "planning_position_min_rad"),
            ("upper", "planning_position_max_rad"),
        )
    }


##
# @brief 在运行上下文内完成预检并创建受保护开发启动动作。
# 加载安装预检器、校验明确USB及绑定限制，生成本次临时控制器/规划配置。客户端可随机16字节，session随机非零且小于2^63；LIMITED资格必须有钉住的独立时间线和完整raw路径。gate退出触发整个launch关闭；full模式仅以--inactive加载两个控制器，实际激活/授权必须走运行时显式流程。
# @param context launch上下文，提供部署/activation/限制、客户端/会话及证据和启动模式参数。
# @return list：全部预检成功时所选动作；任何异常则仅 Shutdown，原因带 NO_LOAD_DEV blocked。
def _start_real(context):
    try:
        share = Path(get_package_share_directory("dummy_bringup"))
        usb = _load_script("real_six_axis_preflight", share / "scripts" /
                           "real_six_axis_preflight.py")
        converter = _load_script("convert_application_ros_limits", share / "scripts" /
                                 "convert_application_ros_limits.py")
        paths, activation = load_deployment(
            LaunchConfiguration("deployment_paths").perform(context),
            LaunchConfiguration("activation_sha256").perform(context),
            usb.verify_live_stm32_usb,
        )
        limits_file = Path(LaunchConfiguration("ros_limits_json").perform(context))
        validated_limits = converter.validate_bound(
            limits_file, Path(paths["candidate_bundle_root"]) / "manifest.json")
        limits = validated_limits.limits
        client = LaunchConfiguration("client_instance_id").perform(context) or secrets.token_hex(16)
        session_text = LaunchConfiguration("session_id").perform(context)
        session = int(session_text) if session_text else secrets.randbelow(2**63 - 1) + 1
        if not HEX_ID.fullmatch(client) or client == "0" * 32 or session <= 0 or session >= 2**63:
            raise ValueError("client/session identity invalid")
        ##
        # 临时配置由绑定限制重新转换，dev_cadence_1to1显式用于本开发装配；不修改仓库模型/合同文件，也不选择“最新”构建目录。
        generated = Path(tempfile.mkdtemp(prefix="dummy-no-load-dev-")) / "config"
        converter.convert(limits_file, Path(paths["candidate_bundle_root"]) / "manifest.json",
                          generated, dev_cadence_1to1=True)
        controllers = yaml.safe_load((generated / "ros2_controllers.yaml").read_text())
        manager_params = controllers["controller_manager"]["ros__parameters"]
        validate_controller_config(manager_params)
        joints = limits["joints"]
        report_root = LaunchConfiguration("raw_evidence_report_root").perform(context)
        raw_evidence_path = validate_raw_evidence_path(
            LaunchConfiguration("raw_evidence_path").perform(context), report_root)
        goal_timeline_path = validate_goal_timeline_path(
            LaunchConfiguration("goal_timeline_path").perform(context),
            report_root, raw_evidence_path)
        ##
        # LIMITED分支把一次性时间线路径也与activation钉住；缺raw或时间线任一项就阻断，不静默降级到无证据运行。
        if activation.get("basis") == "NO_LOAD_DEV_FUNCTIONAL_LIMITED":
            if (not raw_evidence_path or not goal_timeline_path or
                    activation.get("goal_timeline_path") != goal_timeline_path):
                raise ValueError("LIMITED requires pinned one-shot timeline and complete raw capture")
        host_state_max_age_us = validate_host_state_max_age_us(
            LaunchConfiguration("host_state_max_age_us").perform(context))
        mappings = {
            **hardware_mappings(
                joints, activation, paths["endpoint"], paths["device_id"], client,
                limits["control_period_us"], limits["state_publish_period_us"],
                raw_evidence_path, host_state_max_age_us,
                validate_stop_confirmation_timeout_us(
                    LaunchConfiguration("stop_confirmation_timeout_us").perform(context))),
            **planning_mappings(joints),
        }
        xacro_file = share / "config" / "dummy_ref_real_6dof.urdf.xacro"
        moveit = (MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
                  .robot_description(file_path=str(xacro_file), mappings=mappings)
                  .joint_limits(file_path=str(generated / "joint_limits.yaml"))
                  .to_moveit_configs())
        ##
        # gate接收候选契约、产物/报告路径及本次身份，运行时继续校验并申请会话；传参本身不表示设备接受身份、许可或已激活硬件。
        gate_parameters = {
            "contract_bundle": paths["candidate_bundle_root"],
            "profile_name": "f405_6dof_bench", "device_id": paths["device_id"],
            "client_instance_id": client, "endpoint": paths["endpoint"],
            "profile_id": 201, "configured_mask": 63, "session_id": session,
            **gate_lease_parameters(validated_limits),
            "state_max_age_us": host_state_max_age_us, "guarded_controller": "arm_controller",
            "activation_sha256": LaunchConfiguration("activation_sha256").perform(context),
        }
        for key in FILE_KEYS + DIRECTORY_KEYS + ("activation_path", "usb_serial"):
            if key != "candidate_bundle_root":
                gate_parameters[key] = paths[key]
        if goal_timeline_path:
            gate_parameters["goal_timeline_path"] = goal_timeline_path
        print(json.dumps({"event": "host_state_age", "effective_us": host_state_max_age_us,
                          "scope": "UNQUALIFIED_DIAGNOSTIC" if
                          host_state_max_age_us == DIAGNOSTIC_STATE_MAX_AGE_US else
                          "NO_LOAD_DEV_DEFAULT"}), flush=True)
        gate_node = Node(package="dummy_motion_client", executable="device_action_gate",
                         output="screen", parameters=[gate_parameters,
                                                      str(generated / "ros2_controllers.yaml"),
                                                      moveit.robot_description])
        gate_actions = [
            gate_node,
            RegisterEventHandler(OnProcessExit(
                target_action=gate_node,
                on_exit=[EmitEvent(event=Shutdown(reason="device action gate exited"))])),
        ]
        ##
        # 两个spawner都使用--inactive；launch装配控制器与后续明确activate分开，robot_state_publisher和MoveIt节点也不等于电机已使能。
        full_actions = [
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 output="screen", parameters=[moveit.robot_description]),
            ExecuteProcess(cmd=["ros2", "run", "controller_manager", "spawner",
                                "arm_controller", "--controller-manager", "/controller_manager",
                                "--inactive"], output="screen"),
            ExecuteProcess(cmd=["ros2", "run", "controller_manager", "spawner",
                                "joint_state_broadcaster", "--controller-manager",
                                "/controller_manager", "--inactive"], output="screen"),
        ]
        full_actions.extend(generate_move_group_launch(moveit).entities)
        return select_startup_actions(
            LaunchConfiguration("startup_mode").perform(context),
            gate_actions, full_actions)
    except Exception as error:
        return [EmitEvent(event=Shutdown(reason=f"NO_LOAD_DEV blocked: {error}"))]


##
# @brief 声明显式部署绑定及可选诊断/证据参数。
# deployment_paths、activation_sha256、ros_limits_json 必须提供；客户端/session留空时由预检回调随机生成。startup_mode默认full（也支持gate_only），证据路径默认关闭；host_state_max_age_us默认250000、stop_confirmation_timeout_us默认0。OpaqueFunction不会在仅导入/构造描述时执行实际USB预检。
# @return LaunchDescription：声明参数和延迟求值的启动动作；构造描述本身不等于节点已运行。
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("deployment_paths", description="Bound D artifact path manifest"),
        DeclareLaunchArgument("activation_sha256", description="Externally pinned activation SHA-256"),
        DeclareLaunchArgument("ros_limits_json", description="Bound UNQUALIFIED ROS limits JSON"),
        DeclareLaunchArgument("client_instance_id", default_value=""),
        DeclareLaunchArgument("session_id", default_value=""),
        DeclareLaunchArgument("startup_mode", default_value="full"),
        DeclareLaunchArgument("raw_evidence_path", default_value=""),
        DeclareLaunchArgument("raw_evidence_report_root", default_value=""),
        DeclareLaunchArgument("goal_timeline_path", default_value=""),
        DeclareLaunchArgument("host_state_max_age_us", default_value="250000"),
        DeclareLaunchArgument("stop_confirmation_timeout_us", default_value="0"),
        OpaqueFunction(function=_start_real),
    ])
