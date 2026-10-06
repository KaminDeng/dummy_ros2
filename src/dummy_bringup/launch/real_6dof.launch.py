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
NO_LOAD_STATE_MAX_AGE_US = 250000
DIAGNOSTIC_STATE_MAX_AGE_US = 500000


def gate_lease_parameters(validated_limits):
    lease = validated_limits.gate_lease
    return {"permit_remaining_us": lease.permit_remaining_us,
            "requested_session_lease_us": lease.requested_session_lease_us}


def validate_host_state_max_age_us(value):
    if value not in (str(NO_LOAD_STATE_MAX_AGE_US), str(DIAGNOSTIC_STATE_MAX_AGE_US)):
        raise ValueError("host_state_max_age_us must be 250000 or explicit diagnostic 500000")
    return int(value)


def _unique_pairs(pairs):
    values = {}
    for key, value in pairs:
        if key in values:
            raise ValueError("duplicate JSON key")
        values[key] = value
    return values


def _read_bytes(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"missing regular file: {path}")
    return path.read_bytes()


def _parse_object(data, path):
    value = json.loads(data, object_pairs_hook=_unique_pairs)
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def _read_object(path):
    return _parse_object(_read_bytes(path), path)


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
            activation.get("basis") != "NO_LOAD_DEV_FUNCTIONAL" or
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
    usb_verifier(paths["endpoint"], paths["usb_serial"])
    return paths, activation


def _load_script(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ValueError(f"missing installed development verifier: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def select_startup_actions(mode, gate_actions, full_actions):
    if mode == "gate_only":
        return list(gate_actions)
    if mode == "full":
        return list(gate_actions) + list(full_actions)
    raise ValueError("startup_mode must be gate_only or full")


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


def validate_goal_timeline_path(path, report_root, raw_evidence_path):
    if not path:
        return ""
    validated = validate_raw_evidence_path(path, report_root)
    if validated == raw_evidence_path:
        raise ValueError("goal timeline and raw evidence paths must differ")
    return validated


def hardware_mappings(joints, activation, endpoint, device, client,
                      control_period_us, state_publish_period_us,
                      raw_evidence_path="", host_state_max_age_us=NO_LOAD_STATE_MAX_AGE_US):
    host_state_max_age_us = validate_host_state_max_age_us(str(host_state_max_age_us))
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
    }
    if raw_evidence_path:
        mappings["raw_evidence_path"] = raw_evidence_path
    return mappings


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
        host_state_max_age_us = validate_host_state_max_age_us(
            LaunchConfiguration("host_state_max_age_us").perform(context))
        mappings = {
            **hardware_mappings(
                joints, activation, paths["endpoint"], paths["device_id"], client,
                limits["control_period_us"], limits["state_publish_period_us"],
                raw_evidence_path, host_state_max_age_us),
            **planning_mappings(joints),
        }
        xacro_file = share / "config" / "dummy_ref_real_6dof.urdf.xacro"
        moveit = (MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
                  .robot_description(file_path=str(xacro_file), mappings=mappings)
                  .joint_limits(file_path=str(generated / "joint_limits.yaml"))
                  .to_moveit_configs())
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
        OpaqueFunction(function=_start_real),
    ])
