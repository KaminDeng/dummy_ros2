"""Case-specific evidence adapter for the native Gazebo R02 runtime."""
##
# @file r02.py
# @brief 按单case实际Gazebo探针观测生成证据的R02适配器。
#
# 先ros2 service list确认controller_manager，再运行gazebo_runtime_probe.py；入口不自行启动Gazebo。runtime缺失/退出3为NOT_RUN，已执行但观测不全为FAIL。source哈希只绑定所列当前文件，不证明运行进程由该源码新鲜构建。

from __future__ import annotations

from hashlib import sha256
import json
from pathlib import Path
import subprocess
import tempfile
from typing import Any

from dummy_test_support.task_adapters import not_run_case, run_unittest_case


ROOT = Path(__file__).resolve().parents[5]
PROBE = ROOT / "src/dummy_bringup/scripts/gazebo_runtime_probe.py"
TARGETS = {
    "MOD05": "src.dummy_moveit_config.test.test_model_alignment.ModelAlignmentTest.test_all_zero_pose_is_rejected_by_joint3_contract_range",
}
SOURCES = (
    "src/dummy_bringup/launch/gazebo_6dof.launch.py",
    "src/dummy_bringup/scripts/gazebo_runtime_probe.py",
    "src/dummy_bringup/test/test_gazebo_runtime.launch.py",
    "src/dummy_bringup/CMakeLists.txt",
    "src/dummy_bringup/package.xml",
    "src/dummy_moveit_config/launch/gz_moveit.launch.py",
    "src/dummy_moveit_config/launch/gz_sim.launch.py",
    "src/dummy_moveit_config/config/ros2_controllers.yaml",
    "src/dummy_moveit_config/config/dummy-ros2.gz_sim.xacro",
    "src/dummy_moveit_config/config/dummy-ros2.urdf.xacro",
    "src/dummy-ros2_description/urdf/dummy-ros2.xacro",
    "src/dummy_moveit_config/worlds/empty.sdf",
    "src/dummy_motion_client/src/native_action_gate.cpp",
    "src/dummy_moveit_config/test/test_r02_gazebo_contract.py",
)
RUNTIME_CASES = {"FRE11", "ROS01", "ROS02", "ROS03", "ROS11"}


##
# @brief 依固定SOURCES拼相对名和原字节，缺失文件用missing占位；本函数不判文件是否足够完整。
# @return 所列文件内容/缺失占位的SHA-256。
def _source_hash() -> str:
    digest = sha256()
    for relative in SOURCES:
        path = ROOT / relative
        digest.update(relative.encode())
        digest.update(path.read_bytes() if path.is_file() else b"<missing>")
    return digest.hexdigest()


##
# @brief 用8s服务列表检查再以75s期限跑单case探针、读临时JSON；区分_runtime_unavailable与_probe_failure。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @return probe观测dict，或带_runtime_unavailable/_probe_failure的字典。
def _run_runtime_probe(case_id: str) -> dict[str, Any]:
    """Run exactly one live observation; exit code 3 means no usable runtime."""
    try:
        availability = subprocess.run(
            ["ros2", "service", "list"], cwd=ROOT, capture_output=True, text=True,
            check=False, timeout=8,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        return {"_runtime_unavailable": f"ROS graph unavailable: {error}"}
    if (
        availability.returncode != 0
        or "/controller_manager/list_controllers" not in availability.stdout
    ):
        detail = (availability.stdout + availability.stderr)[-2000:]
        return {"_runtime_unavailable": "canonical Gazebo controller manager unavailable: " + detail}
    with tempfile.TemporaryDirectory(prefix=f"r02_{case_id.lower()}_") as temporary:
        report = Path(temporary) / "observations.json"
        try:
            completed = subprocess.run(
                ["/usr/bin/python3", str(PROBE), "--case", case_id, "--report", str(report)],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=75,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            return {"_runtime_unavailable": str(error)}
        output = completed.stdout + completed.stderr
        try:
            observations = json.loads(report.read_text()) if report.is_file() else {}
        except json.JSONDecodeError as error:
            observations = {}
            output += f"\ninvalid probe JSON: {error}"
        if completed.returncode == 3:
            return {"_runtime_unavailable": output[-4000:] or "Gazebo runtime is unavailable"}
        if completed.returncode != 0:
            return {"_probe_failure": output[-4000:] or f"probe exited {completed.returncode}"}
        if not isinstance(observations, dict):
            return {"_probe_failure": "probe observations must be a JSON object"}
        return observations


##
# @brief 接受int/float排除bool，不额外核finite。
# @param value 待核对象，仅类型检查，bool排除。
# @return 是否int/float且非bool；NaN/inf仍可能True。
def _is_number(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


##
# @brief 仅要求list长度及每项dict/time_sec数值形状；不核单调、有限或真实采样来源。
# @param value 待核list/dict/time_sec形状对象。
# @param minimum 时序list最少样本数，默认2。
# @return 是否满足最小长度和time_sec类型形状。
def _valid_time_series(value: object, minimum: int = 2) -> bool:
    return isinstance(value, list) and len(value) >= minimum and all(
        isinstance(sample, dict) and _is_number(sample.get("time_sec")) for sample in value
    )


##
# @brief 核所有component名称/类型含gazebo、计数一致及三个后端边界布尔；是报告字段校验，不是独立鉴别运行进程。
# @param observed 已读取或人工注入的观测字典，字段不是本函数独立测量。
# @return 报告声明/计数是否同时满足native后端边界。
def _valid_native_gazebo_identity(observed: dict[str, Any]) -> bool:
    components = observed.get("hardware_components")
    if not isinstance(components, list) or not components:
        return False
    gazebo_count = sum(
        isinstance(component, dict)
        and "gazebo" in f"{component.get('name', '')} {component.get('type', '')}".lower()
        for component in components
    )
    return (
        observed.get("hardware_component_count") == len(components)
        and observed.get("gazebo_hardware_component_count") == gazebo_count
        and gazebo_count == len(components)
        and observed.get("native_gazebo_backend_only") is True
        and observed.get("dummy_ros_control_device_present") is False
        and observed.get("dpsdk_device_dependencies_present") is False
    )


##
# @brief 核Gazebo身份、pause/resume请求及暂停前后clock推进/暂停停止标记，重复clock消息不等于时间推进。
# @param observed 已读取或人工注入的观测字典，字段不是本函数独立测量。
# @return 该case所列字段门槛是否全部满足。
def _validate_fre11(observed: dict[str, Any]) -> bool:
    return (
        _valid_native_gazebo_identity(observed)
        and
        observed.get("gazebo_world_control_available") is True
        and observed.get("gazebo_pause_requested") is True
        and observed.get("gazebo_resume_requested") is True
        and observed.get("clock_samples_before_pause", 0) >= 2
        and observed.get("clock_advanced_before_pause") is True
        and observed.get("clock_samples_while_paused", 0) >= 0
        and observed.get("clock_advanced_while_paused") is False
        and observed.get("clock_stopped_while_paused") is True
        and observed.get("clock_advanced_after_resume") is True
    )


##
# @brief 核controller_manager、控制器列表/active计数、至少两State样本及公开Action/Gazebo组件声明。
# @param observed 已读取或人工注入的观测字典，字段不是本函数独立测量。
# @return 该case所列字段门槛是否全部满足。
def _validate_ros01(observed: dict[str, Any]) -> bool:
    return (
        _valid_native_gazebo_identity(observed)
        and
        observed.get("controller_manager_available") is True
        and isinstance(observed.get("loaded_controllers"), list)
        and observed.get("active_controller_count", 0) >= 1
        and observed.get("joint_state_sample_count", 0) >= 2
        and observed.get("public_action_available") is True
        and observed.get("gazebo_hardware_component_present") is True
    )


##
# @brief 核目标accepted/状态4、至少三时序样本、J1变化>0.02rad及末误差<0.05rad。
# @param observed 已读取或人工注入的观测字典，字段不是本函数独立测量。
# @return 该case所列字段门槛是否全部满足。
def _validate_ros02(observed: dict[str, Any]) -> bool:
    return (
        _valid_native_gazebo_identity(observed)
        and
        observed.get("public_action_available") is True
        and observed.get("goal_accepted") is True
        and observed.get("goal_terminal_status") == 4
        and _valid_time_series(observed.get("joint_state_time_series"), 3)
        and _is_number(observed.get("actual_joint1_change_rad"))
        and observed["actual_joint1_change_rad"] > 0.02
        and _is_number(observed.get("final_joint1_abs_error_rad"))
        and observed["final_joint1_abs_error_rad"] < 0.05
    )


##
# @brief 核无发命令、六轴漂移<0.02rad、基座平移<0.002m、固定基座和protobuf缺省轴声明。
# @param observed 已读取或人工注入的观测字典，字段不是本函数独立测量。
# @return 该case所列字段门槛是否全部满足。
def _validate_ros03(observed: dict[str, Any]) -> bool:
    axis_drift = observed.get("joint_axis_drift_rad")
    return (
        _valid_native_gazebo_identity(observed)
        and
        observed.get("no_command_sent") is True
        and _valid_time_series(observed.get("joint_state_time_series"), 3)
        and _valid_time_series(observed.get("base_pose_time_series"), 3)
        and _is_number(observed.get("max_joint_drift_rad"))
        and observed["max_joint_drift_rad"] < 0.02
        and isinstance(axis_drift, dict)
        and set(axis_drift) == {f"Joint{index}" for index in range(1, 7)}
        and all(_is_number(axis_drift[name]) and axis_drift[name] < 0.02 for name in axis_drift)
        and _is_number(observed.get("max_base_translation_drift_m"))
        and observed["max_base_translation_drift_m"] < 0.002
        and isinstance(observed.get("protobuf_defaulted_axes"), list)
        and set(observed["protobuf_defaulted_axes"]).issubset({"x", "y", "z"})
        and observed.get("base_is_fixed") is True
    )


##
# @brief 核锁定_internal_arm_controller及JTC类型/配置接受声明，不仅凭列表非空就PASS。
# @param observed 已读取或人工注入的观测字典，字段不是本函数独立测量。
# @return 该case所列字段门槛是否全部满足。
def _validate_ros11(observed: dict[str, Any]) -> bool:
    return (
        _valid_native_gazebo_identity(observed)
        and
        observed.get("controller_manager_available") is True
        and isinstance(observed.get("loaded_controllers"), list)
        and isinstance(observed.get("active_controller_names"), list)
        and observed.get("locked_controller_name") == "_internal_arm_controller"
        and observed.get("locked_controller_type")
        == "joint_trajectory_controller/JointTrajectoryController"
        and observed.get("locked_configuration_accepted") is True
    )


VALIDATORS = {
    "FRE11": _validate_fre11,
    "ROS01": _validate_ros01,
    "ROS02": _validate_ros02,
    "ROS03": _validate_ros03,
    "ROS11": _validate_ros11,
}


##
# @brief 不可用观测转NOT_RUN，其余按case专属validator归PASS/FAIL并保留probe观测及JSON trace。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param observations 要验证/嵌入结果的观测字典，保留其原有证据层级。
# @return 含Gazebo结果、trace和原观测的证据字典。
def _runtime_result(case_id: str, observations: dict[str, Any]) -> dict[str, Any]:
    source_hash = _source_hash()
    ##
    # @par 未运行与失败
    # 运行时缺失单独NOT_RUN；已有probe执行却字段不全/失败为FAIL且executed=True。不能把两者都显示为通过。
    unavailable = observations.get("_runtime_unavailable")
    if isinstance(unavailable, str):
        return not_run_case(
            case_id, {"case_id": case_id, "hardware": "none", "test_only": True}, SOURCES,
            unavailable, sut_kind="gazebo_runtime",
        )
    failure = observations.get("_probe_failure")
    valid = not isinstance(failure, str) and VALIDATORS[case_id](observations)
    trace = json.dumps(observations, ensure_ascii=True, sort_keys=True)
    return {
        "case_id": case_id,
        "executed": True,
        "status": "PASS" if valid else "FAIL",
        "reason": "" if valid else (failure or "case validator rejected probe observations"),
        "sut_kind": "gazebo_runtime",
        "sut_source_sha256": source_hash,
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "_trace_text": trace,
        "observations": {
            "case_id": case_id,
            "case_target": f"{case_id}:{PROBE.relative_to(ROOT)}",
            "requirement_observed": valid,
            "probe_observations": observations,
        },
    }


##
# @brief 严格校验三字段stimulus；运行时case真跑probe，MOD05转离线J3范围unittest，未知case拒绝。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 该suite所选执行路径返回的证据字典，状态/范围不自动升级。
def run_r02_case(case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("R02 stimulus is invalid")
    if case_id in RUNTIME_CASES:
        return _runtime_result(case_id, _run_runtime_probe(case_id))
    try:
        target = TARGETS[case_id]
    except KeyError as error:
        raise ValueError(f"unsupported R02 case: {case_id}") from error
    return run_unittest_case(case_id, stimulus, target, SOURCES)
