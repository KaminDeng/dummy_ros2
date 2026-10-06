import copy
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
        "format": "dummy_wire_release_v2", "release_id": "dummy-wire-v2.4",
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


def test_explicit_one_to_one_cadence_generates_50hz(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    output = tmp_path / "dev_limits"
    converter.convert(input_path, manifest, output, dev_cadence_1to1=True)
    jtc = yaml.safe_load((output / "ros2_controllers.yaml").read_text())
    assert jtc["controller_manager"]["ros__parameters"]["update_rate"] == 50


def test_short_policy_uses_default_gate_timing(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    result = converter.validate_bound(input_path, manifest)
    assert result.gate_lease.permit_remaining_us == 20000000
    assert result.gate_lease.requested_session_lease_us == 30000000


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


def test_rejects_unhashed_limits_permit_override(tmp_path):
    manifest, input_path, limits = source(tmp_path)
    input_path.write_text(json.dumps({**limits, "motion_permit_us": 650000000}))
    with pytest.raises(ValueError, match="permit"):
        converter.validate_bound(input_path, manifest)


def test_rejects_policy_hash_mismatch_for_long_permit(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    policy_path = tmp_path / "policy/application_hold_no_load.json"
    policy = json.loads(policy_path.read_text())
    policy["timing"]["motion_permit_us"] = 650000000
    policy_path.write_text(json.dumps(policy))
    with pytest.raises(ValueError, match="payload SHA"):
        converter.validate_bound(input_path, manifest)


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


def test_rejects_changed_candidate_manifest(tmp_path):
    manifest, input_path, _ = source(tmp_path)
    manifest.write_text('{"release_id":"other"}\n')
    with pytest.raises(ValueError, match="manifest SHA"):
        converter.convert(input_path, manifest, tmp_path / "dev_limits")


@pytest.mark.parametrize("relative_path", [
    "model/model.yaml", "policy/application_hold_no_load.json",
    "profiles/f405_6dof_bench.yaml",
])
def test_payload_validation_uses_bytes_captured_before_replacement(
        tmp_path, monkeypatch, relative_path):
    manifest, input_path, _ = source(tmp_path)
    payload_path = tmp_path / relative_path
    original_read_bytes = Path.read_bytes

    def replace_after_read(path):
        snapshot = original_read_bytes(path)
        if path == payload_path:
            payload_path.write_text("invalid: [")
        return snapshot

    monkeypatch.setattr(Path, "read_bytes", replace_after_read)
    assert converter.validate(input_path, manifest)["scope"] == "NO_LOAD_DEV"


def test_manifest_and_limits_are_each_read_once(tmp_path, monkeypatch):
    manifest, input_path, _ = source(tmp_path)
    original_read_bytes = Path.read_bytes
    original_read_text = Path.read_text
    reads = {manifest: 0, input_path: 0}

    def count_bytes(path):
        if path in reads:
            reads[path] += 1
        return original_read_bytes(path)

    def reject_second_read(path, *args, **kwargs):
        if path in reads:
            pytest.fail(f"{path}: input reopened for parsing")
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_bytes", count_bytes)
    monkeypatch.setattr(Path, "read_text", reject_second_read)
    converter.convert(input_path, manifest, tmp_path / "dev_limits")
    assert reads == {manifest: 1, input_path: 1}
