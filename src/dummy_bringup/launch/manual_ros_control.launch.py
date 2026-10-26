"""当前manual-home部署的ROS链：inactive通信，参考/REST后由C++协调激活。"""
import hashlib
import json
import secrets
import tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

LIVE_POLICY = "650cde3ea8f690b65f2a4273603053499ba786f7d108d80d9088ccd9d6bef996"

def read_bound_deployment(root, pin, sdk):
    root = Path(root).resolve(strict=True)
    raw = (root / "deployment.json").read_bytes()
    if not pin or hashlib.sha256(raw).hexdigest() != pin:
        raise ValueError("manual deployment SHA256 mismatch")
    manifest = json.loads(raw)
    if manifest.get("format") != "dummy_manual_panel_deployment_v1":
        raise ValueError("unsupported manual deployment")
    for name, expected in manifest["files"].items():
        relative = Path(name)
        target = root / relative
        if relative.is_absolute() or ".." in relative.parts or target.is_symlink() or root not in target.resolve(strict=True).parents:
            raise ValueError("unsafe deployment payload")
        if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise ValueError("deployment payload digest mismatch: " + name)
    cfg = json.loads((root / "panel-config.json").read_bytes())
    argv = cfg["native_arguments"]
    args = {argv[i]: argv[i+1] for i in range(len(argv)-1)
            if argv[i].startswith("--") and argv[i] != "--execute"}
    if args.get("--scope") != "manual-home-empty-supported" or args.get("--profile") != "201" or args.get("--mask") != "63":
        raise ValueError("manual scope/profile/mask mismatch")
    identity = manifest["identity"]
    for arg, key in (("--contract", "contract_sha256"), ("--model", "model_sha256"), ("--profile-sha256", "profile_sha256"), ("--firmware", "firmware_sha256")):
        if args[arg] != identity[key]:
            raise ValueError("deployment identity mismatch")
    if identity["profile_sha256"] != LIVE_POLICY:
        raise ValueError("unsupported live policy")
    bundle = Path(sdk) / "share/dummy_ref/robot_contract"
    model_raw = (bundle / "model/model.yaml").read_bytes()
    if hashlib.sha256(model_raw).hexdigest() != identity["model_sha256"]:
        raise ValueError("SDK model mismatch")
    model = yaml.safe_load(model_raw)
    if len(cfg["joints"]) != 6 or len(model["joints"]) != 6:
        raise ValueError("six axes required")
    for i, (joint, expected) in enumerate(zip(cfg["joints"], model["joints"])):
        if (joint["name"] != "Joint"+str(i+1) or joint["direction"] != expected["direction"] or joint["gear_ratio"] != expected["gear_ratio"] or joint["nominal_limits_deg"] != expected["legacy_limits_deg"]):
            raise ValueError("deployment transmission/model mismatch")
    return root, cfg, args, identity, model

def controller_parameters():
    return {
      "controller_manager": {"ros__parameters": {
        "update_rate": 50,
        "hardware_components_initial_state": {"inactive": ["dummy_ref_real_6dof"]},
        "arm_controller": {"type": "dummy_ros_control/GuardedJointTrajectoryController"},
        "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"}}},
      "arm_controller": {"ros__parameters": {
        "joints": ["Joint"+str(i) for i in range(1,7)],
        "command_interfaces": ["position"], "state_interfaces": ["position", "velocity"],
        "allow_partial_joints_goal": False, "open_loop_control": False}}}

def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    root, cfg, args, identity, model = read_bound_deployment(value("deployment_root"), value("deployment_sha256"), value("sdk_prefix"))
    if value("initial_pose") != "manual_limits":
        raise ValueError("initial_pose must be manual_limits")
    share = Path(get_package_share_directory("dummy_bringup"))
    rad = 3.141592653589793 / 180
    lower = [j["limits_deg"][0]*rad for j in cfg["joints"]]
    upper = [j["limits_deg"][1]*rad for j in cfg["joints"]]
    device_lower = [j["model_limits_rad"][0] for j in model["joints"]]
    device_upper = [j["model_limits_rad"][1] for j in model["joints"]]
    device_lower[0], device_upper[0] = -3.2, 3.2
    client = secrets.token_hex(16)
    session = secrets.randbelow(2**63-1)+1
    raw_path, timeline = value("raw_evidence_path"), value("goal_timeline_path")
    mappings = {
      "endpoint": args["--endpoint"], "device_id": args["--device"], "client_instance_id": client,
      "profile_id": "201", "configured_mask": "63", "contract_sha256": identity["contract_sha256"],
      "model_sha256": identity["model_sha256"], "profile_sha256": LIVE_POLICY,
      "firmware_sha256": identity["firmware_sha256"], "compiled_policy_sha256": LIVE_POLICY,
      "position_min_rad": ",".join(map(str,device_lower)), "position_max_rad": ",".join(map(str,device_upper)),
      "velocity_max_rad_s": ",".join(["1.0995574287564276"]*6), "acceleration_max_rad_s": ",".join(["4.0"]*6),
      "control_period_us": "20000", "state_publish_period_us": "20000", "max_state_age_us": "250000",
      "heartbeat_stop_budget_us": "250000", "stop_confirmation_timeout_us": "0", "manual_reference_mode": "true", "raw_evidence_path": raw_path}
    for i in range(6):
        mappings["Joint"+str(i+1)+"_lower"] = str(lower[i])
        mappings["Joint"+str(i+1)+"_upper"] = str(upper[i])
    generated = Path(tempfile.mkdtemp(prefix="dummy-manual-ros-"))
    controllers = generated / "controllers.yaml"
    controllers.write_text(yaml.safe_dump(controller_parameters()))
    limits = {"joint_limits": {j["name"]: {"has_velocity_limits": True, "max_velocity": 1.0995574287564276, "has_acceleration_limits": True, "max_acceleration": 4.0} for j in cfg["joints"]}}
    (generated / "joint_limits.yaml").write_text(yaml.safe_dump(limits))
    moveit = (MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
      .robot_description(file_path=str(share / "config/dummy_ref_real_6dof.urdf.xacro"), mappings=mappings)
      .joint_limits(file_path=str(generated / "joint_limits.yaml")).to_moveit_configs())
    # ControllerManager is embedded in Gate. Its transient-local description
    # must never compete with the parent panel's display-only URDF.
    control_description_remap = [("/robot_description", "/manual_control/robot_description")]
    gate = Node(package="dummy_motion_client", executable="device_action_gate", output="screen",
      remappings=control_description_remap,
      parameters=[str(controllers), moveit.robot_description, {
        "contract_bundle": str(Path(value("sdk_prefix"))/"share/dummy_ref/robot_contract"),
        "profile_name": "manual-home-empty-supported", "manual_reference_mode": True,
        "manual_deployment_root": str(root), "manual_deployment_sha256": value("deployment_sha256"),
        "operator_confirmed_double_limits": value("operator_confirmed_double_limits") == "true",
        "device_id": args["--device"], "client_instance_id": client, "endpoint": args["--endpoint"],
        "profile_id": 201, "configured_mask": 63, "session_id": session,
        "requested_session_lease_us": 30000000, "permit_remaining_us": 900000000,
        "state_max_age_us": 250000, "guarded_controller": "arm_controller",
        "goal_timeline_path": timeline}])
    baseline = Node(package="dummy_motion_client", executable="manual_ros_baseline_node", name="manual_ros_baseline", output="screen", parameters=[{
      "auto_calibrate_and_test": value("auto_calibrate_and_test")=="true", "initial_pose": "manual_limits",
      "joint_names": [j["name"] for j in cfg["joints"]], "lower_rad": lower, "upper_rad": upper,
      "rest_rad": [x*rad for x in cfg["rest_deg"]], "velocity_limits_rad_s": [1.0995574287564276]*6,
      "acceleration_limits_rad_s2": [4.0]*6, "feedback_max_age_ms": 250,
      "report_path": value("report_path"), "goal_timeline_path": timeline, "raw_evidence_path": raw_path,
      "total_timeout_s": 840,
      "cylinder_frame": "base_link",
      **({"cylinder_pose": list(map(float, json.loads(value("cylinder_pose"))))} if json.loads(value("cylinder_pose")) else {}),
      **({"cylinder_dimensions": list(map(float, json.loads(value("cylinder_dimensions"))))} if json.loads(value("cylinder_dimensions")) else {})}])
    control_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
      name="manual_control_state_publisher", parameters=[moveit.robot_description],
      remappings=control_description_remap)
    actions = [control_state_publisher, gate, baseline,
      RegisterEventHandler(OnProcessExit(target_action=gate,on_exit=[EmitEvent(event=Shutdown(reason="manual ROS owner exited"))])),
      Node(package="controller_manager",executable="spawner",arguments=["arm_controller","joint_state_broadcaster","--inactive","--controller-manager","/controller_manager"],output="screen")]
    # MoveIt仅规划，完整轨迹由面板/基线提交到同一个公开FJT守卫。
    display_actions = [Node(package="moveit_ros_move_group", executable="move_group", name="manual_move_group",
      output="screen", parameters=[moveit.to_dict(), {
        "allow_trajectory_execution": False, "publish_robot_description_semantic": True,
        "publish_planning_scene": True, "publish_geometry_updates": True,
        "publish_state_updates": True, "publish_transforms_updates": True,
        "ompl.totg.resample_dt": 0.02, "ompl.totg.path_tolerance": 0.01}])]
    if value("reuse_display_planning") != "true":
        actions.extend(display_actions)
    return actions

def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument("deployment_root"), DeclareLaunchArgument("deployment_sha256"), DeclareLaunchArgument("sdk_prefix"),
      DeclareLaunchArgument("auto_calibrate_and_test",default_value="false",choices=["true","false"]),
      DeclareLaunchArgument("initial_pose",default_value="manual_limits",choices=["manual_limits"]),
      DeclareLaunchArgument("reuse_display_planning",default_value="false",choices=["true","false"]),
      DeclareLaunchArgument("operator_confirmed_double_limits",default_value="false",choices=["true","false"]),
      DeclareLaunchArgument("report_path",default_value=""), DeclareLaunchArgument("raw_evidence_path",default_value=""),
      DeclareLaunchArgument("goal_timeline_path",default_value=""),
      # 空场景默认无法通过碰撞基线；自动验收必须显式提供已验证的圆柱场景。
      DeclareLaunchArgument("cylinder_pose",default_value="[]"), DeclareLaunchArgument("cylinder_dimensions",default_value="[]"),
      OpaqueFunction(function=start)])
