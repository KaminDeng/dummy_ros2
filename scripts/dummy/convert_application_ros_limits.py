#!/usr/bin/env python3
##
# @file convert_application_ros_limits.py
# @brief 把绑定候选身份的未合格无负载限制转换成隔离ROS配置。
#
# 只有schema/scope/qualification与源文件原始SHA等关系通过才写MoveIt/JTC YAML；输出仍为UNQUALIFIED NO_LOAD_DEV，硬件初始inactive。
# 校验冻结候选数据不等于读取当前设备Status、认证真实机械模型或签发permit；FUNCTIONAL/生成成功也不能替代运行时Gate准入。
"""Convert bound no-load development limits to isolated MoveIt/JTC YAML."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
from types import MappingProxyType
from typing import NamedTuple

import yaml


##
# @brief 六轴顺序Joint1..6；PAYLOADS限定恰三个候选文件及排序。
# 默认permit20,000,000µs、session lease30,000,000µs，仅从受hash绑定策略导出GateLease，不接受未hash的limits顶层覆盖。
JOINTS = [f"Joint{index}" for index in range(1, 7)]
SHA256 = re.compile(r"[0-9a-f]{64}\Z")
PAYLOADS = ("model/model.yaml", "policy/application_hold_no_load.json",
            "profiles/f405_6dof_bench.yaml")
DEFAULT_MOTION_PERMIT_US = 20_000_000
SESSION_LEASE_US = 30_000_000


##
# @brief 不可变租约二元组，保存供gate后续使用的µs预算，创建对象本身不授予租约。
class GateLease(NamedTuple):
    ##
    # @brief 绑定策略选定的运动许可时长µs；不会在此向设备发送SetPermit。
    permit_remaining_us: int
    ##
    # @brief 请求的会话租约µs，固定匹配30s；与单目标/permit时限独立。
    requested_session_lease_us: int


##
# @brief 验证快照与租约预算的只读组合；limits递归冻结，不等于外部文件或设备状态从此不会变化。
class ValidatedLimits(NamedTuple):
    ##
    # @brief 原limits JSON的递归只读快照，dict变MappingProxyType、list变tuple。
    limits: object
    ##
    # @brief 从同一验证策略抽出的租约值，防止转换后重新读取未绑定的覆盖字段。
    gate_lease: GateLease


##
# @brief 递归把JSON容器改成不可变快照。
# @param value dict/list或标量；只递归这两种容器。
# @return dict为只读映射、list为tuple，其余原值；不冻结任意自定义对象。
def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


##
# @brief 把失败条件转换为输入校验异常。
# @param condition 为假时拒绝。
# @param message ValueError诊断文字。
# @return 满足条件返回None，否则抛ValueError。
def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


##
# @brief 读取一次普通文件字节供摘要与解析共用。
# @param path 要求is_file且最终路径不是symlink的输入。
# @return 完整bytes；形状不符ValueError，读取失败OSError。
# 此处不逐个拒绝父目录symlink、不做原子open/stat快照，也不限制候选路径根归属。
def read_snapshot(path: Path) -> bytes:
    require(path.is_file() and not path.is_symlink(), f"{path}: regular file required")
    return path.read_bytes()


##
# @brief 从同一字节快照解析顶层JSON对象。
# @param snapshot UTF-8原文字节。
# @param path 仅用于报错上下文的路径。
# @return dict；非标准NaN/Infinity、非object或解析/解码失败抛异常。
# parse_constant内部回调调用require(False)拒绝非有限常量；未启用object_pairs_hook，因此不额外拒绝重复JSON键。
def json_object(snapshot: bytes, path: Path) -> dict:
    data = json.loads(snapshot.decode("utf-8"),
                      parse_constant=lambda value: require(False, f"non-finite {value}"))
    require(isinstance(data, dict), f"{path}: JSON object required")
    return data


##
# @brief 要求精确int/float且数值有限。
# @param value 待核对象；bool被type检查排除。
# @param field 错误定位字段名。
# @return 原数值（未强制转float），不在本函数独立要求正数。
def number(value: object, field: str) -> float:
    require(type(value) in (int, float) and math.isfinite(value),
            f"{field}: finite number required")
    return value


##
# @brief 验证候选文件绑定、六轴边界及预算并冻结结果。
# @param limits_path ROS限制JSON输入，必须声明NO_LOAD_DEV/UNQUALIFIED且runtime与实机许可false。
# @param manifest_path 固定dummy-wire-v2.4候选manifest，恰含PAYLOADS三个文件。
# @return ValidatedLimits(只读limits,GateLease)；任何关系不符抛ValueError，I/O或解析异常向外传播。
# 先比较manifest原字节SHA，再读取三份payload快照并重算排序聚合SHA；model/profile使用文件SHA，policy去顶层note后规范JSON SHA，三者含义不同。
# 这里只核传入候选之间自洽与列出的模型/策略字段，没有独立外部信任锚或完整compile_policy动态约束验证，也没有设备在线实测。
def validate_bound(limits_path: Path, manifest_path: Path) -> ValidatedLimits:
    data = json_object(read_snapshot(limits_path), limits_path)
    require("motion_permit_us" not in data and "requested_session_lease_us" not in data,
            "unhashed limits permit or lease override forbidden")
    manifest_snapshot = read_snapshot(manifest_path)
    manifest = json_object(manifest_snapshot, manifest_path)
    require(data.get("schema") == "dummy_application_ros_limits_v1" and
            data.get("scope") == "NO_LOAD_DEV" and
            data.get("planning_reference_kind") == "BENCH_SESSION" and
            data.get("qualification") == "UNQUALIFIED" and
            data.get("runtime_enabled") is False and
            data.get("motion_permitted_on_real_hardware") is False,
            "unqualified no-load development limits required")
    require(data.get("candidate_manifest_sha256") == hashlib.sha256(manifest_snapshot).hexdigest(),
            "candidate manifest SHA mismatch")
    require(manifest.get("format") == "dummy_wire_release_v2" and
            manifest.get("release_id") == "dummy-wire-v2.4",
            "unsupported candidate manifest")
    for key in ("candidate_payload_sorted_sha256", "model_sha256", "policy_sha256",
                "profile_sha256"):
        require(isinstance(data.get(key), str) and SHA256.fullmatch(data[key]) and
                data[key] != "0" * 64, f"{key}: SHA256 required")
    entries = manifest.get("payloads")
    require(isinstance(entries, list) and len(entries) == len(PAYLOADS) and
            [entry.get("path") for entry in entries if isinstance(entry, dict)] == list(PAYLOADS),
            "candidate payload list mismatch")
    payload_hashes = {}
    payload_snapshots = {}
    for entry in entries:
        snapshot = read_snapshot(manifest_path.parent / entry["path"])
        actual = hashlib.sha256(snapshot).hexdigest()
        require(entry.get("sha256") == actual, f"{entry['path']}: payload SHA mismatch")
        payload_hashes[entry["path"]] = actual
        payload_snapshots[entry["path"]] = snapshot
    listing = "\n".join(f"{name}:{payload_hashes[name]}" for name in PAYLOADS)
    aggregate = hashlib.sha256(listing.encode()).hexdigest()
    require(manifest.get("payload_sorted_sha256") == aggregate and
            data["candidate_payload_sorted_sha256"] == aggregate,
            "candidate payload aggregate SHA mismatch")
    require(manifest.get("model_sha256") == data["model_sha256"] == payload_hashes[PAYLOADS[0]] and
            data["profile_sha256"] == payload_hashes[PAYLOADS[2]],
            "candidate model/profile SHA mismatch")
    model = yaml.safe_load(payload_snapshots[PAYLOADS[0]].decode("utf-8"))
    policy = json_object(payload_snapshots[PAYLOADS[1]], manifest_path.parent / PAYLOADS[1])
    policy_sha = hashlib.sha256(json.dumps(
        {key: value for key, value in policy.items() if key != "note"},
        sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    require(data["policy_sha256"] == policy_sha == manifest.get("compiled_policy_sha256") and
            policy_sha != "0" * 64, "candidate policy SHA mismatch")
    profile = yaml.safe_load(payload_snapshots[PAYLOADS[2]].decode("utf-8"))
    require(isinstance(model, dict) and isinstance(model.get("joints"), list),
            "candidate model joints required")
    require(isinstance(profile, dict) and profile.get("active_axes") == JOINTS,
            "candidate profile axis order mismatch")
    model_joints = model["joints"]
    joints = data.get("joints")
    require(isinstance(joints, list) and len(joints) == 6 and
            len(model_joints) == 6 and
            isinstance(policy.get("axes"), list) and len(policy["axes"]) == 6 and
            [joint.get("name") for joint in joints if isinstance(joint, dict)] == JOINTS and
            [joint.get("name") for joint in model_joints if isinstance(joint, dict)] == JOINTS,
            "six-axis order mismatch")
    require(policy.get("bench_acceleration_advisory_only") is True and
            policy.get("reference_kind") == "BENCH_SESSION",
            "candidate BENCH_SESSION acceleration advisory required")
    device_limits = policy.get("limits")
    require(isinstance(device_limits, dict), "candidate device limits required")
    ##
    # @brief 按相同六轴槽核绝对模型界限、设备Status预期能力和相对BENCH试验边界。
    # J1设备位置能力取公共device_limits，其余轴取模型界限；这里的device_*来自候选文件，不能当成已从当前设备读到的Status。
    for index, (joint, model_joint, policy_axis) in enumerate(
            zip(joints, model_joints, policy["axes"])):
        name = joint["name"]
        require(isinstance(policy_axis, dict), f"{name}: policy axis required")
        bounds = model_joint.get("model_limits_rad")
        require(isinstance(bounds, list) and len(bounds) == 2,
                f"{name}: absolute model limits required")
        lower, upper = number(bounds[0], f"{name}.model_min"), number(bounds[1], f"{name}.model_max")
        require(lower < upper and joint.get("position_min_rad") == lower and
                joint.get("position_max_rad") == upper,
                f"{name}: absolute model bounds mismatch")
        device_lower = device_limits.get("position_min_rad") if index == 0 else lower
        device_upper = device_limits.get("position_max_rad") if index == 0 else upper
        require(joint.get("device_position_min_rad") == device_lower and
                joint.get("device_position_max_rad") == device_upper and
                number(device_lower, f"{name}.device_min") <= lower and
                number(device_upper, f"{name}.device_max") >= upper,
                f"{name}: device status bounds mismatch")
        require(joint.get("device_velocity_max_rad_s") ==
                device_limits.get("velocity_max_rad_s") and
                joint.get("device_acceleration_max_rad_s2") ==
                device_limits.get("acceleration_max_rad_s2") and
                number(joint["device_velocity_max_rad_s"], f"{name}.device_velocity") >=
                number(joint.get("velocity_max_rad_s"), f"{name}.planning_velocity") and
                number(joint["device_acceleration_max_rad_s2"], f"{name}.device_acceleration") >=
                number(joint.get("acceleration_max_rad_s2"), f"{name}.planning_acceleration"),
                f"{name}: device dynamic capability mismatch")
        ##
        # @brief planning_position使用相对BENCH_SESSION试验范围，不将其冒充绝对模型限位。
        # 逐轴effective速度/加速度必须与策略相同，acceleration只ADVISORY、jerk仍None/UNQUALIFIED，不能因输出YAML有加速度项就宣称实机加速度已资格化。
        trial_min = number(joint.get("trial_relative_min_rad"), f"{name}.trial_min")
        trial_max = number(joint.get("trial_relative_max_rad"), f"{name}.trial_max")
        require(trial_min < trial_max, f"{name}: invalid relative trial")
        require(joint.get("planning_position_min_rad") == trial_min and
                joint.get("planning_position_max_rad") == trial_max and
                number(device_limits.get("position_min_rad"), "device_min") <= trial_min and
                trial_max <= number(device_limits.get("position_max_rad"), "device_max"),
                f"{name}: BENCH_SESSION position bounds mismatch")
        require(trial_min == policy_axis.get("trial_min_rad") and
                trial_max == policy_axis.get("trial_max_rad") and
                joint.get("velocity_max_rad_s") == policy_axis.get("effective_velocity_max_rad_s") and
                joint.get("acceleration_max_rad_s2") == policy_axis.get("acceleration_max_rad_s2") and
                policy_axis.get("jerk_max_rad_s3") is None,
                f"{name}: policy dynamics mismatch")
        require(number(joint.get("velocity_max_rad_s"), f"{name}.velocity") > 0 and
                number(joint.get("acceleration_max_rad_s2"), f"{name}.acceleration") > 0 and
                joint.get("acceleration_qualification") == "ADVISORY" and
                joint.get("jerk_max_rad_s3") is None and
                joint.get("jerk_qualification") == "UNQUALIFIED",
                f"{name}: dynamics qualification mismatch")
    for field in ("tracking_tolerance_rad", "hold_position_tolerance_rad"):
        require(number(data.get(field), field) > 0, f"{field}: positive tolerance required")
    require(isinstance(policy.get("limits"), dict) and
            data["tracking_tolerance_rad"] == policy["limits"].get("max_tracking_error_rad") and
            data["hold_position_tolerance_rad"] == policy["limits"].get("hold_position_tolerance_rad"),
            "policy tolerances mismatch")
    for field in ("control_period_us", "state_publish_period_us"):
        require(type(data.get(field)) is int and data[field] > 0,
                f"{field}: positive integer required")
        require(isinstance(policy.get("timing"), dict) and
                data[field] == policy["timing"].get(field), f"{field}: policy timing mismatch")
    timing = policy["timing"]
    for field in ("max_goal_duration_us", "stop_max_duration_us",
                  "completion_max_duration_us", "local_hold_budget_us",
                  "trial_max_duration_us", "session_lease_us"):
        require(type(timing.get(field)) is int and timing[field] > 0,
                f"{field}: positive integer required")
    require(timing["session_lease_us"] == SESSION_LEASE_US,
            "candidate session lease mismatch")
    ##
    # @brief permit只读受hash绑定策略，缺省20s；必须覆盖goal+stop+completion+hold且不超过trial。
    # 顶层limits的未hash permit/lease覆盖已提前禁止；此处只算输入预算，不续租或产生运行授权。
    permit = timing.get("motion_permit_us", DEFAULT_MOTION_PERMIT_US)
    require(type(permit) is int and
            timing["max_goal_duration_us"] + timing["stop_max_duration_us"] +
            timing["completion_max_duration_us"] + timing["local_hold_budget_us"] <= permit <=
            timing["trial_max_duration_us"],
            "candidate motion permit must cover goal and stop/completion/hold within trial")
    return ValidatedLimits(_freeze(data), GateLease(permit, timing["session_lease_us"]))


##
# @brief 提供只读limits兼容入口。
# @param limits_path limits JSON路径。
# @param manifest_path 候选manifest路径。
# @return validate_bound的递归只读limits，丢弃GateLease包装但不重新读文件。
def validate(limits_path: Path, manifest_path: Path):
    return validate_bound(limits_path, manifest_path).limits


##
# @brief 只向全新目录写两份带资格边界的ROS YAML。
# @param limits_path 未合格开发限制JSON。
# @param candidate_manifest 对应绑定候选清单。
# @param output_dir 必须尚不存在且不是symlink的输出目录。
# @param dev_cadence_1to1 true额外要求控制/State周期相同且对应整数Hz，写controller_manager update_rate。
# @return None；校验或写入异常向外传播，失败可留下已创建/已写部分目录。
# MoveIt写相对planning位置、有效速度及advisory加速度，jerk关闭；JTC要求完整六轴、终点速度为零，轨迹/goal容差分别取绑定值。
# controller_manager把dummy_ref_real_6dof初始设inactive；生成配置不会加载或激活controller，更不会向电机发目标。
def convert(limits_path: Path, candidate_manifest: Path, output_dir: Path,
            *, dev_cadence_1to1: bool = False) -> None:
    limits_path, candidate_manifest, output_dir = (
        Path(limits_path), Path(candidate_manifest), Path(output_dir))
    data = validate(limits_path, candidate_manifest)
    require(not output_dir.exists() and not output_dir.is_symlink(),
            "development output directory already exists")
    if dev_cadence_1to1:
        require(data["control_period_us"] == data["state_publish_period_us"] and
                1_000_000 % data["control_period_us"] == 0,
                "1:1 development cadence requires equal integral-Hz periods")
    moveit = {"joint_limits": {}}
    for joint in data["joints"]:
        moveit["joint_limits"][joint["name"]] = {
            "has_position_limits": True,
            "min_position": joint["planning_position_min_rad"],
            "max_position": joint["planning_position_max_rad"],
            "has_velocity_limits": True,
            "max_velocity": joint["velocity_max_rad_s"],
            "has_acceleration_limits": True,
            "max_acceleration": joint["acceleration_max_rad_s2"],
            "has_jerk_limits": False,
        }
    manager = {"hardware_components_initial_state": {"inactive": ["dummy_ref_real_6dof"]},
               "arm_controller": {"type":
               "dummy_ros_control/GuardedJointTrajectoryController"},
               "joint_state_broadcaster": {"type":
               "joint_state_broadcaster/JointStateBroadcaster"}}
    if dev_cadence_1to1:
        manager["update_rate"] = 1_000_000 // data["control_period_us"]
    params = {
        "joints": JOINTS, "command_interfaces": ["position"],
        "state_interfaces": ["position", "velocity"],
        "allow_partial_joints_goal": False,
        "allow_nonzero_velocity_at_trajectory_end": False,
        "constraints": {joint["name"]: {
            "trajectory": data["tracking_tolerance_rad"],
            "goal": data["hold_position_tolerance_rad"],
        } for joint in data["joints"]},
    }
    controllers = {"controller_manager": {"ros__parameters": manager},
                   "arm_controller": {"ros__parameters": params}}
    header = ("# UNQUALIFIED NO_LOAD_DEV BENCH_SESSION planning; acceleration ADVISORY; jerk UNQUALIFIED.\n"
              f"# candidate_manifest_sha256: {data['candidate_manifest_sha256']}\n")
    output_dir.mkdir(parents=True)
    (output_dir / "joint_limits.yaml").write_text(
        header + yaml.safe_dump(moveit, sort_keys=False), encoding="utf-8")
    (output_dir / "ros2_controllers.yaml").write_text(
        header + yaml.safe_dump(controllers, sort_keys=False), encoding="utf-8")


##
# @brief 解析隔离输出参数，转换校验错误为argparse错误。
# @return 成功0；OSError/ValueError/YAMLError由parser.error非零退出，其它异常可传播。
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limits", type=Path, required=True)
    parser.add_argument("--candidate-manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dev-cadence-1to1", action="store_true")
    args = parser.parse_args()
    try:
        convert(args.limits, args.candidate_manifest, args.output_dir,
                dev_cadence_1to1=args.dev_cadence_1to1)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
