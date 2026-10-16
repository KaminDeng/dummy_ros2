"""开发态真实六轴 launch 在创建 ROS 节点前校验外部身份。"""

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


MODULE = Path(__file__).resolve().parents[1] / "launch" / "real_6dof.launch.py"
SPEC = importlib.util.spec_from_file_location("real_6dof_launch", MODULE)
launch_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launch_module)


def deployment(tmp_path):
    candidate = tmp_path / "candidate"
    candidate.mkdir()
    (candidate / "manifest.json").write_text(json.dumps({
        "format": "dummy_wire_release_v2", "release_id": "dummy-wire-v2.4",
        "runtime_enabled": False, "profile_id": 201, "configured_mask": 63,
    }))
    activation = tmp_path / "activation.json"
    endpoint = ("serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_"
                "Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200")
    activation.write_text(json.dumps({
        "schema": "dummy_no_load_dev_activation_v1", "scope": "NO_LOAD_DEV",
        "basis": "NO_LOAD_DEV_FUNCTIONAL", "qualification": "UNQUALIFIED",
        "device_id": "01000000000000000000000000000000",
        "endpoint": endpoint, "usb_serial": "DEMO00000001",
        "candidate_manifest_sha256": hashlib.sha256(
            (candidate / "manifest.json").read_bytes()).hexdigest(),
    }))
    artifact = tmp_path / "artifact.bin"
    artifact.write_bytes(b"fixture")
    root = tmp_path / "root"
    root.mkdir()
    evidence = tmp_path / "evidence"
    evidence.mkdir()
    paths = {key: str(artifact) for key in launch_module.FILE_KEYS}
    paths.update({key: str(root) for key in launch_module.DIRECTORY_KEYS})
    paths.update({
        "schema": "no_load_dev_deployment_paths_v1",
        "activation_path": str(activation),
        "candidate_bundle_root": str(candidate),
        "hil_evidence_root": str(evidence),
        "device_id": "01000000000000000000000000000000",
        "endpoint": endpoint, "usb_serial": "DEMO00000001",
    })
    deployment_path = tmp_path / "deployment.json"
    deployment_path.write_text(json.dumps(paths))
    return deployment_path, hashlib.sha256(activation.read_bytes()).hexdigest(), paths


def test_deployment_requires_external_pin_and_live_usb(tmp_path):
    path, digest, paths = deployment(tmp_path)
    seen = []
    loaded, activation = launch_module.load_deployment(
        path, digest, lambda endpoint, serial: seen.append((endpoint, serial)))
    assert loaded == paths
    assert activation["scope"] == "NO_LOAD_DEV"
    assert seen == [(paths["endpoint"], paths["usb_serial"])]
    with pytest.raises(ValueError, match="activation"):
        launch_module.load_deployment(path, "0" * 64, lambda *_: None)


def test_deployment_rejects_noncanonical_endpoint_and_runtime_flip(tmp_path):
    path, digest, paths = deployment(tmp_path)
    paths["endpoint"] = "serial//dev/ttyACM1#baudrate=115200"
    path.write_text(json.dumps(paths))
    with pytest.raises(ValueError):
        launch_module.load_deployment(path, digest, lambda *_: None)
    paths["endpoint"] = json.loads(Path(paths["activation_path"]).read_text())["endpoint"]
    path.write_text(json.dumps(paths))
    manifest = Path(paths["candidate_bundle_root"]) / "manifest.json"
    data = json.loads(manifest.read_text())
    data["runtime_enabled"] = True
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        launch_module.load_deployment(path, digest, lambda *_: None)


def test_deployment_rejects_non_string_endpoint(tmp_path):
    path, digest, paths = deployment(tmp_path)
    paths["endpoint"] = 12
    path.write_text(json.dumps(paths))
    with pytest.raises(ValueError):
        launch_module.load_deployment(path, digest, lambda *_: None)


def test_activation_uses_verified_byte_snapshot_after_path_changes(tmp_path, monkeypatch):
    path, digest, paths = deployment(tmp_path)
    activation_path = Path(paths["activation_path"])
    original_read = Path.read_bytes
    reads = []

    def replace_after_read(self):
        data = original_read(self)
        if self == activation_path:
            reads.append(data)
            if len(reads) == 1:
                changed = json.loads(data)
                changed["scope"] = "REAL_LOAD"
                replacement = self.with_suffix(".replacement")
                replacement.write_text(json.dumps(changed))
                replacement.replace(self)
        return data

    monkeypatch.setattr(Path, "read_bytes", replace_after_read)
    loaded, activation = launch_module.load_deployment(path, digest, lambda *_: None)
    assert loaded == paths
    assert activation["scope"] == "NO_LOAD_DEV"
    assert len(reads) == 1


def test_candidate_uses_hashed_byte_snapshot_after_path_changes(tmp_path, monkeypatch):
    path, digest, paths = deployment(tmp_path)
    candidate_path = Path(paths["candidate_bundle_root"]) / "manifest.json"
    original_read = Path.read_bytes
    reads = []

    def replace_after_read(self):
        data = original_read(self)
        if self == candidate_path:
            reads.append(data)
            if len(reads) == 1:
                changed = json.loads(data)
                changed["runtime_enabled"] = True
                replacement = self.with_suffix(".replacement")
                replacement.write_text(json.dumps(changed))
                replacement.replace(self)
        return data

    monkeypatch.setattr(Path, "read_bytes", replace_after_read)
    loaded, activation = launch_module.load_deployment(path, digest, lambda *_: None)
    assert loaded == paths
    assert activation["scope"] == "NO_LOAD_DEV"
    assert len(reads) == 1


def test_controller_config_keeps_real_hardware_inactive():
    manager = {
        "update_rate": 50,
        "hardware_components_initial_state": {"inactive": ["dummy_ref_real_6dof"]},
        "arm_controller": {"type": "dummy_ros_control/GuardedJointTrajectoryController"},
        "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
    }
    assert launch_module.validate_controller_config(manager)
    with pytest.raises(ValueError):
        launch_module.validate_controller_config({
            **manager, "hardware_components_initial_state": {"inactive": []}})


def test_gate_renews_for_full_candidate_session_lease(tmp_path):
    fixture_path = (Path(__file__).resolve().parents[3] / "scripts/dummy/tests" /
                    "test_convert_application_ros_limits.py")
    fixture_spec = importlib.util.spec_from_file_location("converter_fixture", fixture_path)
    fixture = importlib.util.module_from_spec(fixture_spec)
    fixture_spec.loader.exec_module(fixture)
    manifest, limits_path, _ = fixture.source(tmp_path)
    short = fixture.converter.validate_bound(limits_path, manifest)
    assert launch_module.gate_lease_parameters(short) == {
        "permit_remaining_us": 20000000,
        "requested_session_lease_us": 30000000,
    }
    long_root = tmp_path / "long"
    long_root.mkdir()
    manifest, limits_path, _ = fixture.source(long_root, {
        "max_goal_duration_us": 630000000, "motion_permit_us": 650000000})
    long = fixture.converter.validate_bound(limits_path, manifest)
    assert launch_module.gate_lease_parameters(long) == {
        "permit_remaining_us": 650000000,
        "requested_session_lease_us": 30000000,
    }
    with pytest.raises((AttributeError, TypeError)):
        launch_module.gate_lease_parameters({"permit_remaining_us": 650000000})


def test_hardware_parameters_use_device_capability_not_planning_limits():
    joints = [{"name": f"Joint{index}", "position_min_rad": -2.9,
               "position_max_rad": 2.9, "velocity_max_rad_s": 0.15,
               "acceleration_max_rad_s2": 1.0,
               "device_position_min_rad": -3.2 if index == 1 else 0.6108652381980153 if index == 3 else -2.9,
               "device_position_max_rad": 3.2 if index == 1 else 3.141592653589793 if index == 3 else 2.9,
               "device_velocity_max_rad_s": 0.5,
               "device_acceleration_max_rad_s2": 1.0}
              for index in range(1, 7)]
    activation = {"source_contract_sha256": "a" * 64, "model_sha256": "b" * 64,
                  "live_profile_sha256": "c" * 64, "firmware_sha256": "d" * 64,
                  "compiled_policy_sha256": "e" * 64}
    mappings = launch_module.hardware_mappings(
        joints, activation, "serial//dev/ttyACM1#baudrate=115200",
        "01" * 16, "02" * 16, 20000, 20000)
    assert mappings["position_min_rad"].split(",")[0] == "-3.2"
    assert mappings["position_max_rad"].split(",")[0] == "3.2"
    assert mappings["position_min_rad"].split(",")[2] == "0.6108652381980153"
    assert mappings["velocity_max_rad_s"] == ",".join(["0.5"] * 6)
    assert mappings["max_state_age_us"] == "250000"
    assert mappings["heartbeat_stop_budget_us"] == "250000"
    diagnostic = launch_module.hardware_mappings(
        joints, activation, "serial//dev/ttyACM1#baudrate=115200",
        "01" * 16, "02" * 16, 20000, 20000,
        host_state_max_age_us=500000)
    assert diagnostic["max_state_age_us"] == "500000"
    assert diagnostic["heartbeat_stop_budget_us"] == "500000"
    assert "raw_evidence_path" not in mappings
    capture = launch_module.hardware_mappings(
        joints, activation, "serial//dev/ttyACM1#baudrate=115200",
        "01" * 16, "02" * 16, 20000, 20000,
        raw_evidence_path="/report/transport.raw")
    assert capture["raw_evidence_path"] == "/report/transport.raw"
    assert capture["endpoint"] == mappings["endpoint"]
    assert joints[0]["position_min_rad"] == -2.9
    assert joints[0]["velocity_max_rad_s"] == 0.15


def test_bench_description_maps_all_policy_relative_bounds():
    joints = [{"name": f"Joint{index}", "planning_position_min_rad": -0.1 * index,
               "planning_position_max_rad": 0.1 * index} for index in range(1, 7)]
    mappings = launch_module.planning_mappings(joints)
    assert mappings["Joint3_lower"] == "-0.30000000000000004"
    assert mappings["Joint3_upper"] == "0.30000000000000004"
    assert len(mappings) == 12


def test_gate_only_diagnostic_starts_no_planning_or_hardware_description():
    gate = ["device_gate", "shutdown_on_exit"]
    full = ["robot_description", "controller_spawners", "move_group"]
    assert launch_module.select_startup_actions("gate_only", gate, full) == gate
    assert launch_module.select_startup_actions("full", gate, full) == gate + full
    with pytest.raises(ValueError):
        launch_module.select_startup_actions("unknown", gate, full)


def test_raw_capture_is_disabled_by_default_and_requires_report_root(tmp_path):
    assert launch_module.validate_raw_evidence_path("", "") == ""
    arguments = {item.name: item for item in
                 launch_module.generate_launch_description().entities
                 if hasattr(item, "name")}
    assert "".join(item.perform(None) for item in
                   arguments["raw_evidence_path"].default_value) == ""
    assert "".join(item.perform(None) for item in
                   arguments["raw_evidence_report_root"].default_value) == ""
    with pytest.raises(ValueError, match="report root"):
        launch_module.validate_raw_evidence_path(str(tmp_path / "raw.bin"), "")


def test_raw_capture_accepts_fresh_absolute_path_beneath_explicit_root(tmp_path):
    report_root = tmp_path / "report"
    report_root.mkdir()
    child = report_root / "capture"
    child.mkdir()
    target = child / "transport.raw"
    assert launch_module.validate_raw_evidence_path(str(target), str(report_root)) == str(target)
    assert not target.exists()


@pytest.mark.parametrize("target", ["relative.raw", "/tmp/other.raw"])
def test_raw_capture_rejects_relative_or_outside_path(tmp_path, target):
    report_root = tmp_path / "report"
    report_root.mkdir()
    with pytest.raises(ValueError):
        launch_module.validate_raw_evidence_path(target, str(report_root))


def test_raw_capture_rejects_traversal_existing_path_and_symlinks(tmp_path):
    report_root = tmp_path / "report"
    report_root.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    existing = report_root / "existing.raw"
    existing.write_bytes(b"preserve")
    linked_dir = report_root / "linked"
    linked_dir.symlink_to(outside, target_is_directory=True)
    linked_target = report_root / "linked.raw"
    linked_target.symlink_to(outside / "missing.raw")
    for target in (
            existing, linked_dir / "fresh.raw", linked_target,
            report_root / ".." / "outside" / "fresh.raw"):
        with pytest.raises(ValueError):
            launch_module.validate_raw_evidence_path(str(target), str(report_root))
    assert existing.read_bytes() == b"preserve"


def test_raw_capture_rejects_symlink_report_root_and_missing_parent(tmp_path):
    report_root = tmp_path / "report"
    report_root.mkdir()
    linked_root = tmp_path / "linked_report"
    linked_root.symlink_to(report_root, target_is_directory=True)
    with pytest.raises(ValueError):
        launch_module.validate_raw_evidence_path(str(linked_root / "fresh.raw"),
                                                 str(linked_root))
    with pytest.raises(ValueError):
        launch_module.validate_raw_evidence_path(str(report_root / "missing" / "fresh.raw"),
                                                 str(report_root))


def test_goal_timeline_is_opt_in_and_bound_to_fresh_report_path(tmp_path):
    arguments = {item.name: item for item in
                 launch_module.generate_launch_description().entities
                 if hasattr(item, "name")}
    assert "".join(item.perform(None) for item in
                   arguments["goal_timeline_path"].default_value) == ""
    report_root = tmp_path / "report"
    report_root.mkdir()
    target = report_root / "action_timeline.csv"
    assert launch_module.validate_goal_timeline_path(
        str(target), str(report_root), "") == str(target)
    assert not target.exists()
    target.write_text("existing evidence")
    with pytest.raises(ValueError):
        launch_module.validate_goal_timeline_path(str(target), str(report_root), "")
    assert target.read_text() == "existing evidence"


def test_goal_timeline_rejects_cross_root_and_raw_capture_collision(tmp_path):
    report_root = tmp_path / "report"
    report_root.mkdir()
    target = report_root / "action_timeline.csv"
    with pytest.raises(ValueError):
        launch_module.validate_goal_timeline_path(str(target), "", "")
    with pytest.raises(ValueError):
        launch_module.validate_goal_timeline_path(str(tmp_path / "outside.csv"),
                                                  str(report_root), "")
    with pytest.raises(ValueError):
        launch_module.validate_goal_timeline_path(str(target), str(report_root), str(target))


def test_host_state_age_diagnostic_is_explicit_and_bounded():
    arguments = {item.name: item for item in
                 launch_module.generate_launch_description().entities
                 if hasattr(item, "name")}
    assert "".join(item.perform(None) for item in
                   arguments["host_state_max_age_us"].default_value) == "250000"
    assert launch_module.validate_host_state_max_age_us("250000") == 250000
    assert launch_module.validate_host_state_max_age_us("500000") == 500000
    for invalid in ("0", "500001", "0500000", "250001", "abc"):
        with pytest.raises(ValueError):
            launch_module.validate_host_state_max_age_us(invalid)


def test_independent_stop_confirmation_keeps_state_and_heartbeat_250ms():
    joints = [dict(device_position_min_rad=-1, device_position_max_rad=1,
                   device_velocity_max_rad_s=.5, device_acceleration_max_rad_s2=1)] * 6
    activation = dict(source_contract_sha256="a"*64, model_sha256="b"*64,
                      live_profile_sha256="c"*64, firmware_sha256="d"*64,
                      compiled_policy_sha256="e"*64)
    mappings = launch_module.hardware_mappings(joints, activation,
        "serial//dev/serial/by-id/test#baudrate=115200", "01"*16, "02"*16, 20000, 20000,
        stop_confirmation_timeout_us=300000)
    assert mappings["stop_confirmation_timeout_us"] == "300000"
    assert mappings["max_state_age_us"] == mappings["heartbeat_stop_budget_us"] == "250000"
    fallback = launch_module.hardware_mappings(joints, activation,
        "serial//dev/serial/by-id/test#baudrate=115200", "01"*16, "02"*16, 20000, 20000)
    assert fallback["stop_confirmation_timeout_us"] == "0"
    assert fallback["max_state_age_us"] == "250000"


@pytest.mark.parametrize("value", [-1, 250000, 300001, 1000000, "300000.0", True])
def test_launch_stop_confirmation_requires_approved_independent_value(value):
    with pytest.raises(ValueError):
        launch_module.validate_stop_confirmation_timeout_us(value)
