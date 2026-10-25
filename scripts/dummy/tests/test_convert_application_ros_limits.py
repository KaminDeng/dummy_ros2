import copy
##
# @file test_convert_application_ros_limits.py
# @brief 候选/限制快照绑定及开发态MoveIt/JTC配置转换回归。
#
# 在tmp_path创建合成model/policy/profile/manifest；SHA自洽与freeze不代表真实机械模型或实机批准。关节模型绝对界、设备能力界和BENCH相对±0.6rad规划界分开；acceleration ADVISORY、jerk UNQUALIFIED保留。仅写临时YAML/JSON，不运行ROS/控制器。
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


MODULE = Path(__file__).resolve().parents[1] / "convert_application_ros_limits.py"
SPEC = importlib.util.spec_from_file_location("convert_application_ros_limits", MODULE)
converter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(converter)


##
# @brief 生成六轴合成候选与limits：J3绝对35..180°、planning±0.6rad、设备速度0.5rad/s，绑定原文件/规范policy/聚合摘要。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param timing_override 要并入合成policy timing的覆盖字段；None用短时默认。
# @return (manifest路径, limits路径, 原limits字典)，文件均为合成输入。
def source(tmp_path, timing_override=None):
    manifest = tmp_path / "manifest.json"
    model_path = tmp_path / "model/model.yaml"
    model_path.parent.mkdir()
    policy_path = tmp_path / "policy/application_hold_no_load.json"
    policy_path.parent.mkdir()
    profile_path = tmp_path / "profiles/f405_6dof_bench.yaml"
    profile_path.parent.mkdir()
    joints = []
    for index in range(1, 7):
        lower, upper = (0.6108652381980153, 3.141592653589793) if index == 3 else (-2.0, 2.0)
        joints.append({
            "name": f"Joint{index}", "position_min_rad": lower,
            "position_max_rad": upper, "trial_relative_min_rad": -0.6,
            "trial_relative_max_rad": 0.6,
            "planning_position_min_rad": -0.6,
            "planning_position_max_rad": 0.6,
            "velocity_max_rad_s": 0.15079644737231007,
            "device_position_min_rad": -3.2 if index == 1 else lower,
            "device_position_max_rad": 3.2 if index == 1 else upper,
            "device_velocity_max_rad_s": 0.5,
            "device_acceleration_max_rad_s2": 1.0,
            "acceleration_max_rad_s2": 1.0,
            "acceleration_qualification": "ADVISORY",
            "jerk_max_rad_s3": None, "jerk_qualification": "UNQUALIFIED",
        })
    model = {"joints": [{"name": joint["name"],
                         "model_limits_rad": [joint["position_min_rad"], joint["position_max_rad"]]}
                        for joint in joints]}
    model_path.write_text(yaml.safe_dump(model))
    model_sha = hashlib.sha256(model_path.read_bytes()).hexdigest()
    policy = {
        "axes": [{"effective_velocity_max_rad_s": 0.15079644737231007,
                  "acceleration_max_rad_s2": 1.0, "jerk_max_rad_s3": None,
                  "trial_min_rad": -0.6, "trial_max_rad": 0.6} for _ in joints],
        "limits": {"position_min_rad": -3.2, "position_max_rad": 3.2,
                   "velocity_max_rad_s": 0.5, "acceleration_max_rad_s2": 1.0,
                   "max_tracking_error_rad": 0.05,
                   "hold_position_tolerance_rad": 0.003},
        "timing": {"control_period_us": 20000, "state_publish_period_us": 20000,
                   "stop_max_duration_us": 1500000,
                   "completion_max_duration_us": 3000000,
                   "local_hold_budget_us": 5000000,
                   "trial_max_duration_us": 660000000,
                   "max_goal_duration_us": 10000000,
                   "session_lease_us": 30000000,
                   **(timing_override or {})},
        "bench_acceleration_advisory_only": True,
        "reference_kind": "BENCH_SESSION",
    }
    policy_path.write_text(json.dumps(policy))
    policy_sha = hashlib.sha256(json.dumps(policy, sort_keys=True,
                                           separators=(",", ":")).encode()).hexdigest()
    profile_path.write_text(yaml.safe_dump({"active_axes": [f"Joint{i}" for i in range(1, 7)]}))
    profile_sha = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    payloads = [{"path": str(path.relative_to(tmp_path)),
                 "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                for path in (model_path, policy_path, profile_path)]
    aggregate = hashlib.sha256("\n".join(
        f'{item["path"]}:{item["sha256"]}' for item in payloads).encode()).hexdigest()
    manifest.write_text(json.dumps({
        "format": "dummy_wire_release_v2", "release_id": "dummy-wire-v2.5",
        "payloads": payloads, "model_sha256": model_sha,
        "compiled_policy_sha256": policy_sha, "payload_sorted_sha256": aggregate,
    }) + "\n")
    limits = {
        "schema": "dummy_application_ros_limits_v1", "scope": "NO_LOAD_DEV",
        "planning_reference_kind": "BENCH_SESSION",
        "qualification": "UNQUALIFIED", "runtime_enabled": False,
        "motion_permitted_on_real_hardware": False,
        "candidate_manifest_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest(),
        "candidate_payload_sorted_sha256": aggregate,
        "model_sha256": model_sha, "policy_sha256": policy_sha,
        "profile_sha256": profile_sha, "joints": joints,
        "tracking_tolerance_rad": 0.05,
        "hold_position_tolerance_rad": 0.003,
        "control_period_us": 20000, "state_publish_period_us": 20000,
    }
    input_path = tmp_path / "limits.json"
    input_path.write_text(json.dumps(limits))
    return manifest, input_path, limits


##
# @brief 转换保BENCH J3±0.6/设备绝对界分离、无jerk强限/默认不改update_rate、GuardedJTC+inactive hardware+公开Action映射及容差。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_generates_dev_only_moveit_and_jtc_without_cadence_override(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    output = tmp_path / "dev_limits"
    converter.convert(input_path, manifest, output)
    moveit = yaml.safe_load((output / "joint_limits.yaml").read_text())
    jtc = yaml.safe_load((output / "ros2_controllers.yaml").read_text())
    joint3 = moveit["joint_limits"]["Joint3"]
    assert joint3["min_position"] == -0.6
    assert joint3["max_position"] == 0.6
    assert joint3["max_velocity"] == pytest.approx(0.15079644737231007)
    assert converter.validate(input_path, manifest)["joints"][2]["device_position_min_rad"] == (
        pytest.approx(0.6108652381980153))
    assert joint3["max_acceleration"] == 1.0
    assert joint3["has_jerk_limits"] is False
    assert "max_jerk" not in joint3
    assert "update_rate" not in jtc["controller_manager"]["ros__parameters"]
    assert jtc["controller_manager"]["ros__parameters"]["arm_controller"]["type"] == (
        "dummy_ros_control/GuardedJointTrajectoryController")
    assert jtc["controller_manager"]["ros__parameters"]["joint_state_broadcaster"]["type"] == (
        "joint_state_broadcaster/JointStateBroadcaster")
    assert jtc["controller_manager"]["ros__parameters"]["hardware_components_initial_state"] == {
        "inactive": ["dummy_ref_real_6dof"]}
    assert "_internal_arm_controller" not in jtc
    moveit_mapping = yaml.safe_load((Path(__file__).resolve().parents[3] /
                                     "src/dummy_moveit_config/config/moveit_controllers.yaml").read_text())
    assert moveit_mapping["moveit_simple_controller_manager"]["controller_names"] == ["arm_controller"]
    assert moveit_mapping["moveit_simple_controller_manager"]["arm_controller"]["action_ns"] == (
        "follow_joint_trajectory")
    params = jtc["arm_controller"]["ros__parameters"]
    assert params["joints"] == [f"Joint{i}" for i in range(1, 7)]
    assert params["constraints"]["Joint3"] == {"trajectory": 0.05, "goal": 0.003}
    assert "ADVISORY" in (output / "joint_limits.yaml").read_text()
    assert "UNQUALIFIED" in (output / "joint_limits.yaml").read_text()


##
# @brief 显式dev_cadence_1to1才由20000µs周期生成50Hz，不把默认配置暗改。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_explicit_one_to_one_cadence_generates_50hz(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    output = tmp_path / "dev_limits"
    converter.convert(input_path, manifest, output, dev_cadence_1to1=True)
    jtc = yaml.safe_load((output / "ros2_controllers.yaml").read_text())
    assert jtc["controller_manager"]["ros__parameters"]["update_rate"] == 50


##
# @brief 已验证短policy默认permit20s、session lease30s，单位µs；只核参数不申请许可。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_short_policy_uses_default_gate_timing(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    result = converter.validate_bound(input_path, manifest)
    assert result.gate_lease.permit_remaining_us == 20000000
    assert result.gate_lease.requested_session_lease_us == 30000000


##
# @brief 630s目标/650s permit来自已hash policy，session仍30s；Bound值与深层limits均禁止改写，不代表运行中续租已成功。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_hashed_long_policy_provides_goal_sized_gate_timing(tmp_path):
    manifest, input_path, _ = source(tmp_path, {
        "max_goal_duration_us": 630000000, "motion_permit_us": 650000000})
    result = converter.validate_bound(input_path, manifest)
    assert result.gate_lease.permit_remaining_us == 650000000
    assert result.gate_lease.requested_session_lease_us == 30000000
    with pytest.raises((AttributeError, TypeError)):
        result.gate_lease.permit_remaining_us = 20000000
    with pytest.raises(TypeError):
        result.limits["joints"][0]["name"] = "ForgedJoint"


##
# @brief permit不足/超trial/bool或session20s等即便hash自洽仍因时序策略拒绝。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param timing_override 要并入合成policy timing的覆盖字段；None用短时默认。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("timing_override", [
    {"max_goal_duration_us": 630000000, "motion_permit_us": 639000000},
    {"max_goal_duration_us": 630000000, "motion_permit_us": 660000001},
    {"max_goal_duration_us": 630000000, "motion_permit_us": True},
    {"max_goal_duration_us": 630000000, "motion_permit_us": 650000000,
     "session_lease_us": 20000000},
])
def test_rejects_invalid_hashed_long_gate_timing(tmp_path, timing_override):
    manifest, input_path, _ = source(tmp_path, timing_override)
    with pytest.raises(ValueError, match="permit|lease"):
        converter.validate_bound(input_path, manifest)


##
# @brief 只在limits外层塞650s permit，未绑定policy来源，不能放大授权预算。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_rejects_unhashed_limits_permit_override(tmp_path):
    manifest, input_path, limits = source(tmp_path)
    input_path.write_text(json.dumps({**limits, "motion_permit_us": 650000000}))
    with pytest.raises(ValueError, match="permit"):
        converter.validate_bound(input_path, manifest)


##
# @brief 改policy timing却不更新manifest payload SHA，必须拒绝文件漂移。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_rejects_policy_hash_mismatch_for_long_permit(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    policy_path = tmp_path / "policy/application_hold_no_load.json"
    policy = json.loads(policy_path.read_text())
    policy["timing"]["motion_permit_us"] = 650000000
    policy_path.write_text(json.dumps(policy))
    with pytest.raises(ValueError, match="payload SHA"):
        converter.validate_bound(input_path, manifest)


##
# @brief schema/资格/参考系/轴顺序/三类边界/速度/jerk/ADVISORY破坏均在输出目录创建前拒绝。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param mutation 参数化变异函数或分支名，逐例只制造指定错误。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("mutation", [
    lambda data: data.update(schema="wrong"),
    lambda data: data.update(qualification="QUALIFIED"),
    lambda data: data.update(planning_reference_kind="ABSOLUTE_HOME"),
    lambda data: data["joints"].reverse(),
    lambda data: data["joints"][2].update(position_min_rad=-0.6, position_max_rad=0.6),
    lambda data: data["joints"][2].update(planning_position_min_rad=0.6108652381980153),
    lambda data: data["joints"][2].update(jerk_max_rad_s3=1.0),
    lambda data: data["joints"][2].update(velocity_max_rad_s=0.2),
    lambda data: data["joints"][0].update(device_position_min_rad=-0.6),
    lambda data: data["joints"][1].update(device_velocity_max_rad_s=0.15079644737231007),
    lambda data: data["joints"][2].update(acceleration_qualification="QUALIFIED"),
])
def test_rejects_invalid_limits_without_output(tmp_path, mutation):
    manifest, input_path, limits = source(tmp_path)
    changed = copy.deepcopy(limits)
    mutation(changed)
    input_path.write_text(json.dumps(changed))
    output = tmp_path / "dev_limits"
    with pytest.raises(ValueError):
        converter.convert(input_path, manifest, output)
    assert not output.exists()


##
# @brief manifest原字节改变即使是可解析JSON也因pin不符拒绝，不新建输出。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_rejects_changed_candidate_manifest(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    manifest.write_text('{"release_id":"other"}\n')
    with pytest.raises(ValueError, match="manifest SHA"):
        converter.convert(input_path, manifest, tmp_path / "dev_limits")


##
# @brief 读model/policy/profile后立刻改路径为非法YAML，validate仍解析已核摘要的原快照，避免重读竞争。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param monkeypatch pytest属性替换fixture，退出作用域自动恢复。
# @param relative_path 待替换的model/policy/profile相对文件名。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("relative_path", [
    "model/model.yaml", "policy/application_hold_no_load.json",
    "profiles/f405_6dof_bench.yaml",
])
def test_payload_validation_uses_bytes_captured_before_replacement(
        tmp_path, monkeypatch, relative_path):
    manifest, input_path, _ = source(tmp_path)
    payload_path = tmp_path / relative_path
    original_read_bytes = Path.read_bytes

    ##
    # @par 内部回调 replace_after_read
    # 包装read_bytes先取原snapshot，再对目标payload路径写坏文本，返回原字节。
    # 参数 path：目标文件Path，读取原字节；是否原始被替换见函数职责。
    # 返回：路径被改写前读取的原snapshot bytes。
    def replace_after_read(path):
        snapshot = original_read_bytes(path)
        if path == payload_path:
            payload_path.write_text("invalid: [")
        return snapshot

    monkeypatch.setattr(Path, "read_bytes", replace_after_read)
    assert converter.validate(input_path, manifest)["scope"] == "NO_LOAD_DEV"


##
# @brief 分别统计manifest/limits read_bytes一次，任何再read_text重开两输入都会pytest.fail。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param monkeypatch pytest属性替换fixture，退出作用域自动恢复。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_manifest_and_limits_are_each_read_once(tmp_path, monkeypatch):
    manifest, input_path, _ = source(tmp_path)
    original_read_bytes = Path.read_bytes
    original_read_text = Path.read_text
    reads = {manifest: 0, input_path: 0}

    ##
    # @par 内部回调 count_bytes
    # 只对两入口文件累计read_bytes次数后转原方法，不修改返回内容。
    # 参数 path：目标文件Path，读取原字节；是否原始被替换见函数职责。
    # 返回：原read_bytes结果。
    def count_bytes(path):
        if path in reads:
            reads[path] += 1
        return original_read_bytes(path)

    ##
    # @par 内部回调 reject_second_read
    # 两入口文件若被read_text再读立即fail，其它路径交原read_text。
    # 参数 path：目标文件Path，读取原字节；是否原始被替换见函数职责。
    # 参数 args：其它位置参数，转交原read_text。
    # 参数 kwargs：其它关键字参数，转交原read_text。
    # 返回：其它文件原read_text结果；受监控输入重开即pytest.fail。
    def reject_second_read(path, *args, **kwargs):
        if path in reads:
            pytest.fail(f"{path}: input reopened for parsing")
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_bytes", count_bytes)
    monkeypatch.setattr(Path, "read_text", reject_second_read)
    converter.convert(input_path, manifest, tmp_path / "dev_limits")
    assert reads == {manifest: 1, input_path: 1}


@pytest.mark.parametrize('mutation', ['none', 'missing-marker', 'wire', 'selection', 'snapshot', 'missing-payload', 'extra-payload'])
def test_current_protocol_closure_is_bound_before_planning_conversion(tmp_path, mutation):
    """完整当前发布闭包可转换；自洽重签仍不能绕过来源/选择/成员检查。"""
    import shutil
    from scripts.dummy.installed_sdk import robot_bundle
    manifest_path, limits_path, limits = source(tmp_path)
    selected = robot_bundle()
    current = json.loads((selected / 'manifest.json').read_text())
    shutil.copytree(selected / 'protocol', tmp_path / 'protocol')
    manifest = json.loads(manifest_path.read_text())
    manifest.update({key: current[key] for key in ('contract_sha256', 'protocol_release', 'wire_contract')})
    manifest['runtime_contract_sha256'] = current['contract_sha256']
    manifest['payloads'] += [dict(item) for item in current['payloads'] if item['path'].startswith('protocol/')]
    if mutation == 'missing-marker':
        manifest.pop('protocol_release')
    elif mutation == 'wire':
        manifest['contract_sha256'] = 'ab' * 32
    elif mutation == 'selection':
        manifest['protocol_release']['selection']['codec'] = 'protobuf'
    elif mutation == 'snapshot':
        manifest['protocol_release']['source_snapshot'] = {'kind': 'invented'}
    elif mutation == 'missing-payload':
        manifest['payloads'] = [item for item in manifest['payloads'] if item['path'] != 'protocol/include/wire_layouts.json']
    elif mutation == 'extra-payload':
        (tmp_path / 'protocol/unlisted').write_text('unlisted')
        manifest['payloads'].append({'path': 'protocol/unlisted', 'sha256': hashlib.sha256(b'unlisted').hexdigest()})
    manifest['payloads'].sort(key=lambda item: item['path'])
    aggregate = hashlib.sha256('\n'.join(f"{item['path']}:{item['sha256']}" for item in manifest['payloads']).encode()).hexdigest()
    manifest['payload_sorted_sha256'] = aggregate
    manifest_path.write_text(json.dumps(manifest))
    limits.update(candidate_manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                  candidate_payload_sorted_sha256=aggregate)
    limits_path.write_text(json.dumps(limits))
    if mutation == 'none':
        converter.convert(limits_path, manifest_path, tmp_path / 'converted')
    else:
        with pytest.raises(ValueError, match='protocol'):
            converter.convert(limits_path, manifest_path, tmp_path / 'converted')
