"""仅软件故障注入：启动真实 C++ 基线与假 ROS 端点，不连接 SDK/串口。"""
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

import yaml

mode = sys.argv[2] if len(sys.argv) > 2 else "denied"
assert mode in ("released_fault", "released_fault_wrong_reference", "released_fault_no_commit", "released_fault_wrong_tag", "external_release", "external_release_no_authority", "denied", "stale", "shutdown", "unconfirmed", "retained", "wrongref", "owner", "noowner", "status_cadence", "status_qualification_cadence", "state_stale_status_live", "can_stale", "status_stale", "calibration_motion", "release_retry", "release_unavailable", "release_pending_evidence", "planning_duplicate_nodes", "planning_duplicate_servers", "planning_private_scene", "planning_unrelated_scene", "planning_bad_private_suffix", "planning_wrong_scene_namespace", "renewal_retry", "renewal_wrong_uuid", "renewal_no_receipt", "renewal_owner_drift", "renewal_pending", "consumed_permit", "consumed_permit_denied", "consumed_owner_drift", "scene_normalized", "scene_wrong_pose", "scene_wrong_dimensions", "scene_wrong_frame", "scene_equivalent_local", "scene_wrong_orientation", "scene_full_positive_origin", "scene_full_negative_origin", "scene_cancel_evidence_failure", "scene_unrelated_plan_error", "scene_restore_plan_failure", "scene_clear_plan_failure", "scene_blocked_plan_success")
os.environ.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))

import rclpy
from dpsdk_interfaces.msg import DummyDeviceStatusV2, DummyJointStateV2
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.executors import SingleThreadedExecutor
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController, ListControllers
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, ApplyPlanningScene, GetStateValidity

rclpy.init()
node = Node("baseline_software_peer")
fault_release_mode = mode.startswith("released_fault")
external_mode = mode.startswith("external_release")
planning_mode = external_mode or mode in ("planning_duplicate_nodes", "planning_duplicate_servers", "planning_private_scene", "planning_unrelated_scene", "planning_bad_private_suffix", "planning_wrong_scene_namespace", "renewal_retry", "renewal_wrong_uuid", "renewal_no_receipt", "renewal_owner_drift", "renewal_pending", "consumed_permit", "consumed_permit_denied", "consumed_owner_drift", "scene_normalized", "scene_wrong_pose", "scene_wrong_dimensions", "scene_wrong_frame", "scene_equivalent_local", "scene_wrong_orientation", "scene_full_positive_origin", "scene_full_negative_origin", "scene_cancel_evidence_failure", "scene_unrelated_plan_error", "scene_restore_plan_failure", "scene_clear_plan_failure", "scene_blocked_plan_success")
full_scene_mode = mode in ("scene_full_positive_origin", "scene_full_negative_origin", "scene_cancel_evidence_failure", "scene_unrelated_plan_error", "scene_restore_plan_failure", "scene_clear_plan_failure", "scene_blocked_plan_success")
scene_mode = mode.startswith("scene_")
origin_noise = (3.19e-5 if mode == "scene_full_positive_origin" else -3.19e-5) if full_scene_mode else 0.0
cancel_running = False
cancel_requested = False
cancel_future = None
cancel_handle = None
cancel_receipts = 0
obstacle_rejections = 0
obstacle_probe_requests = []
collision_target = None
scene_object = None
scene_validity_calls = 0
consumed_mode = mode.startswith("consumed_") or scene_mode
simulated_positions = [origin_noise, 0.0, 0.0, 0.0, 0.0, 0.0]
permit_consumed = False
permit_requests = 0
permit_consumed_at = None
permit_refresh_count = 0
permit_requested_at = None
renewal_mode = mode.startswith("renewal_")
rejected_uuid = ""
action_calls = 0
renewal_at = None
action_server = None
planning_peers = []
planning_executor = None
held = False
plan_calls = 0


def spin_once():
    if planning_executor is not None:
        planning_executor.spin_once(timeout_sec=0.02)
    else:
        rclpy.spin_once(node, timeout_sec=0.02)

status_pub = node.create_publisher(DummyDeviceStatusV2, "/device_action_gate/device_status", 10)
state_pub = node.create_publisher(DummyJointStateV2, "/device_action_gate/device_state", 10)
released = False
freeze = False
calls = {"calibrate": 0, "release": 0}
sequence = 0
last_status_published = 0.0
status_period_index = 0
calibration_at = 0.0
calibration_denied = fault_release_mode or mode in ("denied", "unconfirmed", "retained", "wrongref", "release_retry", "release_unavailable", "release_pending_evidence")


def publish():
    global permit_consumed, permit_refresh_count, cancel_running, permit_consumed_at
    if cancel_requested and cancel_handle is not None and cancel_handle.is_cancel_requested and not cancel_future.done():
        cancel_running = False
        permit_consumed = True
        permit_consumed_at = time.monotonic()
        from control_msgs.action import FollowJointTrajectory
        result = FollowJointTrajectory.Result()
        if mode == "scene_cancel_evidence_failure":
            cancel_handle.abort()
            result.error_code = -4
            result.error_string = "EVIDENCE_INCOMPLETE"
        else:
            cancel_handle.canceled()
        cancel_future.set_result(result)
    if (scene_mode or (mode == "consumed_permit" and permit_requests == 1 and permit_requested_at is not None)) and permit_consumed and time.monotonic() - (permit_requested_at if mode == "consumed_permit" else permit_consumed_at) >= 0.15:
        assert scene_mode or action_calls == 1, "无许可期间不许发送下一Goal"
        permit_consumed = False
        permit_refresh_count += 1
    global sequence, last_status_published, status_period_index
    if freeze:
        return
    sequence += 1
    status = DummyDeviceStatusV2()
    status.boot_epoch = 23
    status.device_id = [1] * 16
    if fault_release_mode:
        status.reference_epoch = 8 if released and mode=="released_fault_wrong_reference" else 7
        if released:
            status.core_state=9
            status.cleanup_state=3
    if mode in ("retained", "wrongref"):
        status.reference_epoch = 8 if mode == "wrongref" and released else 7
    if planning_mode:
        status.reference_epoch = 1
        status.profile_id = 201
        status.position_min_rad = [-1.0] * 6
        status.position_max_rad = [1.0] * 6
        status.velocity_max_rad_s = [0.03] * 6
        status.acceleration_max_rad_s = [0.1] * 6
        if calls["calibrate"] and not released:
            status.session_id = 42
            status.real_motion_permitted = not permit_consumed
            status.permit_remaining_us = 0 if permit_consumed else 60000000
            status.motion_epoch = (2 * action_calls - (1 if cancel_running else 0)) if consumed_mode else 0
            status.management_epoch = permit_refresh_count
    status.can_sent_mask = 63
    status.core_state = 5 if cancel_running and not released else 4 if planning_mode and held and not released else (2 if status.reference_epoch else 1)
    status.configured_mask = 63
    status.run_id = 9 if cancel_running else 0
    status.release_committed = released and mode!="released_fault_no_commit"
    if fault_release_mode and released:
        status.core_state=9
    if mode == "owner":
        status.session_id = 42
        status.real_motion_permitted = True
        status.permit_remaining_us = 60000000
    state = DummyJointStateV2()
    state.boot_epoch = 23
    state.device_id = [1] * 16
    state.reference_epoch = status.reference_epoch
    state.session_id = status.session_id
    state.profile_id = status.profile_id
    state.motion_epoch = status.motion_epoch
    state.management_epoch = status.management_epoch
    state.can_sent_mask = 63
    if planning_mode:
        state.hold_ready = held and not released and not cancel_running
        state.position_rad = simulated_positions if consumed_mode else ([0.0] * 6 if renewal_mode else [0.0, 0.01, 0.0, 0.0, 0.0, 0.0])
        if ((mode == "renewal_owner_drift" and renewal_at is not None) or (mode == "consumed_owner_drift" and permit_consumed)) and not released:
            state.session_id = 43
    state.core_state = status.core_state
    state.configured_mask = 63
    state.position_valid_mask = 63
    state.velocity_valid_mask = 63
    state.raw_position_valid_mask = 63
    state.state_seq = sequence
    state.feedback_seq = [sequence * 2] * 6
    if mode == "calibration_motion" and calls["calibrate"] and not released:
        state.raw_motor_position_rev = [0.001 * sequence] * 6
        state.position_rad = [0.0001 * sequence] * 6
        state.estimated_velocity_rad_s = [0.01] * 6
    state.can_rx_age_us = [251000 if mode == "can_stale" and calls["calibrate"] and not released else 11000] * 6
    # 实际run5资格阶段Status约500ms；保留调度抖动，不能套普通阶段250ms周期。
    status_periods = (0.50, 0.52, 0.54) if mode == "status_qualification_cadence" else (0.25, 0.26)
    if mode not in ("status_cadence", "status_qualification_cadence") or time.monotonic() - last_status_published >= status_periods[status_period_index % len(status_periods)]:
        if mode != "status_stale" or not calls["calibrate"] or released:
            status_pub.publish(status)
        last_status_published = time.monotonic()
        status_period_index += 1
    if mode != "state_stale_status_live" or not calls["calibrate"] or released:
        state_pub.publish(state)


def calibration(_request, response):
    global freeze, calibration_at
    calls["calibrate"] += 1
    calibration_at = time.monotonic()
    response.success = not calibration_denied
    response.message = "FAULT_INJECTED_CALIBRATION_DENIED" if calibration_denied else "CALIBRATION_ACCEPTED"
    if mode == "stale":
        freeze = True
    return response


def release(_request, response):
    global released, freeze
    calls["release"] += 1
    if (external_mode and calls["release"] > 1) or mode == "release_unavailable" or (mode == "release_retry" and calls["release"] < 3):
        response.success = False
        response.message = "RELEASE_UNAVAILABLE"
        return response
    released = mode != "unconfirmed"
    freeze = False
    response.success = False
    response.message = "EVIDENCE_INCOMPLETE;RELEASE_PENDING" if mode == "release_pending_evidence" else "RELEASE_QUEUED"
    return response


def bootstrap_status(_request, response):
    if mode in ("consumed_permit", "consumed_permit_denied") and permit_consumed and not released:
        response.success = True
        response.message = "ENABLED_HOLD_PERMIT_REQUIRED"
        return response
    if fault_release_mode and released:
        response.success=False
        response.message="EVIDENCE_INCOMPLETE;RELEASED_FAULT"+(" extra" if mode=="released_fault_wrong_tag" else "")
        return response
    if mode == "external_release_no_authority" and released:
        response.success = False
        response.message = "RELEASE_PENDING"
        return response
    if renewal_mode and renewal_at is not None and not released:
        pending = mode == "renewal_pending" or time.monotonic() - renewal_at < 0.08
        response.success = not pending
        response.message = "LEASE_RENEWAL_PENDING" if pending else "ENABLED_HOLD"
        return response
    response.success = (released or (planning_mode and held)) and mode != "release_pending_evidence"
    response.message = ("EVIDENCE_INCOMPLETE;RELEASED_IDLE" if mode == "release_pending_evidence" else "RELEASED_IDLE") if released else ("HOLD" if planning_mode and held else "READ_ONLY")
    return response


def reference_status(_request, response):
    response.success = planning_mode
    response.message = "READY" if planning_mode else "QUEUED"
    return response


services = [
    node.create_service(Trigger, "/device_action_gate/manual_reference_status", reference_status),
    node.create_service(Trigger, "/device_action_gate/calibrate_manual_reference", calibration),
    node.create_service(Trigger, "/device_action_gate/disable_and_release", release),
    node.create_service(Trigger, "/device_action_gate/bootstrap_status", bootstrap_status),
]
if planning_mode:
    planning_executor = SingleThreadedExecutor()
    planning_executor.add_node(node)
    # 一个move_group的内部节点可复用完全相同FQN，但每种公开服务只有一个server。
    planning_peers = [Node("planning_provider") for _ in range(4)]
    scene_peer = planning_peers[0]
    if mode in ("planning_private_scene", "planning_unrelated_scene", "planning_bad_private_suffix", "planning_wrong_scene_namespace"):
        scene_name = {
            "planning_private_scene": "planning_provider_private_101611612567568",
            "planning_unrelated_scene": "unrelated_scene_provider",
            "planning_bad_private_suffix": "planning_provider_private_101junk",
            "planning_wrong_scene_namespace": "planning_provider_private_101611612567568",
        }[mode]
        scene_peer = Node(scene_name, namespace="/unrelated" if mode == "planning_wrong_scene_namespace" else "/")
        planning_peers.append(scene_peer)
    for peer in planning_peers:
        planning_executor.add_node(peer)

    def accept_hardware(request, response):
        response.ok = True
        response.state.id = request.target_state.id
        return response

    def accept_controllers(_request, response):
        response.ok = True
        return response

    def list_controllers(_request, response):
        from controller_manager_msgs.msg import ControllerState
        response.controller = [ControllerState(name=name, state="active") for name in ("joint_state_broadcaster", "arm_controller")]
        return response

    def enable_hold(_request, response):
        global held
        held = True
        response.success = True
        response.message = "HOLD_QUEUED"
        return response

    def reject_plan(request, response):
        global plan_calls
        plan_calls += 1
        if full_scene_mode and action_calls == 16:
            import json
            from rosidl_runtime_py.convert import message_to_ordereddict
            obstacle_probe_requests.append(json.dumps(message_to_ordereddict(request), sort_keys=True, separators=(",", ":")))
            from moveit_msgs.msg import MoveItErrorCodes
            if (mode == "scene_clear_plan_failure" and len(obstacle_probe_requests) == 1) or (mode == "scene_restore_plan_failure" and len(obstacle_probe_requests) == 3):
                response.motion_plan_response.error_code.val = MoveItErrorCodes.FAILURE
                return response
        if full_scene_mode and scene_object is not None:
            global obstacle_rejections
            from moveit_msgs.msg import MoveItErrorCodes
            target = [joint.position for joint in request.motion_plan_request.goal_constraints[0].joint_constraints]
            assert collision_target is not None and target == collision_target
            obstacle_rejections += 1
            response.motion_plan_response.error_code.val = MoveItErrorCodes.INVALID_GROUP_NAME if mode == "scene_unrelated_plan_error" else (MoveItErrorCodes.SUCCESS if mode == "scene_blocked_plan_success" else MoveItErrorCodes.FAILURE)
            return response
        if scene_mode:
            from trajectory_msgs.msg import JointTrajectoryPoint
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
            trajectory.joint_names = ["Joint" + str(axis) for axis in range(1, 7)]
            target = [joint.position for joint in request.motion_plan_request.goal_constraints[0].joint_constraints]
            for positions, seconds in [(simulated_positions, 0), (target, 4)]:
                point = JointTrajectoryPoint()
                point.positions = list(positions)
                point.velocities = [0.0] * 6
                point.accelerations = [0.0] * 6
                point.time_from_start.sec = seconds
                trajectory.points.append(point)
            response.motion_plan_response.error_code.val = 1
        else:
            response.motion_plan_response.error_code.val = -1
        return response

    def read_scene(_request, response):
        import copy
        if scene_object is not None:
            obj = copy.deepcopy(scene_object)
            if mode == "scene_wrong_pose":
                obj.pose.position.x += 0.01
            elif mode == "scene_wrong_dimensions":
                obj.primitives[0].dimensions[1] += 0.01
            elif mode == "scene_wrong_frame":
                obj.header.frame_id = "unrelated_link"
            elif mode == "scene_wrong_orientation":
                obj.pose.orientation.x = 2.0 ** -0.5
                obj.pose.orientation.w = 2.0 ** -0.5
            elif mode == "scene_equivalent_local":
                # 非平凡旋转/平移分解仍须以T_object*T_primitive还原同一header pose。
                obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = 0.2, 0.0, 0.4
                obj.pose.orientation.z = 2.0 ** -0.5
                obj.pose.orientation.w = 2.0 ** -0.5
                local = obj.primitive_poses[0]
                local.position.x, local.position.y, local.position.z = 0.1, -0.1, 0.0
                local.orientation.z = -(2.0 ** -0.5)
                local.orientation.w = 2.0 ** -0.5
            response.scene.world.collision_objects.append(obj)
        return response

    def apply_scene(request, response):
        global scene_object
        import copy
        from geometry_msgs.msg import Pose
        obj = copy.deepcopy(request.scene.world.collision_objects[0])
        if obj.operation == obj.REMOVE:
            scene_object = None
        else:
            # 真MoveIt run12: 全局shape pose归入object.pose，primitive_pose归一identity。
            if abs(obj.pose.orientation.w) < 0.5:
                obj.pose = copy.deepcopy(obj.primitive_poses[0])
            local = Pose()
            local.orientation.w = 1.0
            obj.primitive_poses = [local]
            scene_object = obj
        response.success = True
        return response

    def scene_validity(request, response):
        global scene_validity_calls, collision_target
        scene_validity_calls += 1
        if full_scene_mode:
            if scene_validity_calls == 1:
                response.valid = True
            else:
                from moveit_msgs.msg import ContactInformation
                import math
                collision_target = list(request.robot_state.joint_state.position)
                nominal_target = [math.pi / 180.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                collision = all(abs(a-b) < 1e-12 for a,b in zip(collision_target, nominal_target))
                response.valid = not collision
                if collision:
                    contact = ContactInformation()
                    contact.contact_body_1 = "manual_ros_baseline_cylinder"
                    contact.contact_body_2 = "link3_1_1"
                    response.contacts.append(contact)
        else:
            # 此回归只证明严格场景往返；在下一独立屏障受控终止，不伪称整项通过。
            response.valid = False
        return response

    services.extend([
        node.create_service(SetHardwareComponentState, "/controller_manager/set_hardware_component_state", accept_hardware),
        node.create_service(SwitchController, "/controller_manager/switch_controller", accept_controllers),
        node.create_service(ListControllers, "/controller_manager/list_controllers", list_controllers),
        node.create_service(Trigger, "/device_action_gate/enable_hold", enable_hold),
        planning_peers[0].create_service(GetMotionPlan, "/plan_kinematic_path", reject_plan),
        scene_peer.create_service(GetPlanningScene, "/get_planning_scene", read_scene),
        planning_peers[0].create_service(ApplyPlanningScene, "/apply_planning_scene", apply_scene),
        planning_peers[0].create_service(GetStateValidity, "/check_state_validity", scene_validity),
    ])
    if mode == "planning_duplicate_servers":
        services.append(planning_peers[1].create_service(GetMotionPlan, "/plan_kinematic_path", reject_plan))

if renewal_mode or consumed_mode:
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from control_msgs.action import FollowJointTrajectory

    class CapturingServer(ActionServer):
        async def _execute_goal_request(self, request):
            global rejected_uuid
            rejected_uuid = bytes(request[1].goal_id.uuid).hex()
            await super()._execute_goal_request(request)

    def reject_during_renewal(_goal):
        global action_calls, renewal_at
        action_calls += 1
        if full_scene_mode:
            assert scene_object is None, "障碍存在时不能发送碰撞FJT"
        if action_calls == 1 and renewal_mode:
            # run10: Hold后20ms预约续租，3ms后目标到达时槽仍被占用。
            time.sleep(0.020)
            renewal_at = time.monotonic()
            time.sleep(0.003)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def stop_after_retry(goal_handle):
        global permit_consumed, simulated_positions, permit_consumed_at, cancel_running, cancel_future, cancel_handle
        if full_scene_mode and action_calls == 17:
            from rclpy.task import Future
            cancel_running = True
            simulated_positions[0] += 0.001
            cancel_handle = goal_handle
            cancel_future = Future(executor=planning_executor)
            return await cancel_future
        if consumed_mode and (scene_mode or action_calls == 1):
            simulated_positions = list(goal_handle.request.trajectory.points[-1].positions)
            permit_consumed = True
            permit_consumed_at = time.monotonic()
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = 0
            return result
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = -1
        return result

    def receipt(_request, response):
        response.success = mode != "renewal_no_receipt"
        response.message = "RENEWAL_PENDING:" + ("00" * 16 if mode == "renewal_wrong_uuid" else rejected_uuid)
        return response

    def install_permit(_request, response):
        global permit_consumed, permit_requests, permit_requested_at
        permit_requests += 1
        permit_requested_at = time.monotonic()
        if mode == "consumed_permit":
            assert permit_requests == 1, "同一等待轮不能重复D5"

        response.success = mode != "consumed_permit_denied"
        response.message = "Operation queued; poll bootstrap_status for final result" if response.success else "Operation unavailable or another management request is pending"
        if response.success and mode != "consumed_permit":
            permit_consumed = False
        return response

    def accept_cancel(_handle):
        global cancel_requested, cancel_receipts
        cancel_requested = True
        cancel_receipts += 1
        return CancelResponse.ACCEPT

    services.append(node.create_service(Trigger, "/device_action_gate/set_bench_permit", install_permit))
    services.append(node.create_service(Trigger, "/device_action_gate/last_goal_rejection", receipt))
    action_server = CapturingServer(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory",
                                   execute_callback=stop_after_retry, goal_callback=reject_during_renewal, cancel_callback=accept_cancel)

timer = node.create_timer(0.02, publish)
root = pathlib.Path(tempfile.mkdtemp(prefix="baseline-software-"))
params = {
    "joint_names": ["Joint" + str(axis) for axis in range(1, 7)],
    "lower_rad": [-1.0] * 6,
    "upper_rad": [1.0] * 6,
    "rest_rad": [0.0] * 6,
    "velocity_limits_rad_s": [0.03] * 6,
    "acceleration_limits_rad_s2": [0.1] * 6,
    "report_path": str(root / "report.txt"),
    "goal_timeline_path": str(root / "timeline.csv"),
    "raw_evidence_path": str(root / "raw.bin"),
}
if scene_mode:
    params["cylinder_pose"] = [0.3, 0.1, 0.4, 0.0, 0.0, 0.0, 1.0]
    params["cylinder_dimensions"] = [0.2, 0.05]
if mode == "noowner":
    params["total_timeout_s"] = 841
if full_scene_mode or external_mode:
    (root / "timeline.csv").write_text("SOFTWARE_ONLY gate timeline fixture\n")
    (root / "raw.bin").write_bytes(b"SOFTWARE_ONLY gate raw evidence fixture\n")
(root / "params.yaml").write_text(yaml.safe_dump({"manual_ros_baseline": {"ros__parameters": params}}))
log = open(root / "node.log", "w")
process = subprocess.Popen(
    [sys.argv[1], "--ros-args", "--params-file", str(root / "params.yaml")], stdout=log, stderr=log
)


def call(endpoint):
    client = node.create_client(Trigger, endpoint)
    deadline = time.monotonic() + 5
    while not client.service_is_ready() and time.monotonic() < deadline:
        spin_once()
    assert client.service_is_ready(), (endpoint, root)
    future = client.call_async(Trigger.Request())
    while not future.done() and time.monotonic() < deadline:
        spin_once()
    assert future.done(), (endpoint, root)
    response = future.result()
    node.destroy_client(client)
    return response


try:
    response = call("/device_action_gate/baseline_status")
    assert not response.success and "ReadOnly" in response.message, response
    assert calls["calibrate"] == calls["release"] == 0
    if mode == "owner":
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            spin_once()
        assert not call("/device_action_gate/auto_calibrate_and_test").success
        assert calls == {"calibrate": 0, "release": 0}, calls
        assert "ReadOnly" in call("/device_action_gate/baseline_status").message
        print("PASS existing owner rejected without takeover/release; report:", root)
        sys.exit(0)
    if external_mode:
        assert call("/device_action_gate/prepare_manual_control").success
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            spin_once()
            response = call("/device_action_gate/manual_control_status")
            if "MANUAL_CONTROL_READY" in response.message:
                break
        assert "MANUAL_CONTROL_READY" in response.message, (response, root)
        assert calls == {"calibrate": 1, "release": 0}
        assert call("/device_action_gate/disable_and_release").message == "RELEASE_QUEUED"
        deadline = time.monotonic() + (24 if mode == "external_release_no_authority" else 5)
        while time.monotonic() < deadline:
            spin_once()
            response = call("/device_action_gate/manual_control_status")
            assert calls["release"] == 1, ("Node repeated external D10", calls, root)
            if "MANUAL_CONTROL_RELEASED" in response.message or "MANUAL_CONTROL_FAILED" in response.message:
                break
        if mode == "external_release_no_authority":
            assert "MANUAL_CONTROL_FAILED" in response.message and "cleanup=UNCONFIRMED" in response.message, (response, root)
        else:
            assert response.success and "MANUAL_CONTROL_RELEASED" in response.message, (response, root)
            assert "RELEASE_CONFIRMED" in (root / "report.txt").read_text()
        assert action_calls == 0 and calls == {"calibrate": 1, "release": 1}
        print("PASS", mode, "external D10 is only polled; no extra management/Goal; report:", root)
        sys.exit(0)
    assert call("/device_action_gate/auto_calibrate_and_test").success
    assert not call("/device_action_gate/auto_calibrate_and_test").success
    if mode == "calibration_motion":
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            spin_once()
            response = call("/device_action_gate/baseline_status")
            assert "BASELINE_Calibrating" in response.message, (response, root)
            assert calls["release"] == 0, calls
        assert calls["calibrate"] == 1
        process.send_signal(signal.SIGTERM)
        deadline = time.monotonic() + 5
        while process.poll() is None and time.monotonic() < deadline:
            spin_once()
        assert process.poll() == 0, root
        assert "BASELINE_FAILED: 节点关闭 cleanup=RELEASED_IDLE" in (root / "report.txt").read_text()
        print("PASS initial stationary admission only; moving qualification continues beyond5s; report:", root)
        sys.exit(0)
    if mode in ("status_cadence", "status_qualification_cadence"):
        deadline = time.monotonic() + (4.0 if mode == "status_qualification_cadence" else 3.0)
        while time.monotonic() < deadline:
            spin_once()
            response = call("/device_action_gate/baseline_status")
            assert "BASELINE_Calibrating" in response.message, (response, root)
            assert calls["release"] == 0, calls
        assert status_period_index >= (7 if mode == "status_qualification_cadence" else 10)
        assert calls["calibrate"] == 1
        process.send_signal(signal.SIGTERM)
        deadline = time.monotonic() + 5
        while process.poll() is None and time.monotonic() < deadline:
            spin_once()
        assert process.poll() == 0, root
        assert "BASELINE_FAILED: 节点关闭 cleanup=RELEASED_IDLE" in (root / "report.txt").read_text()
        print("PASS", mode, "independent Status cadence / 20ms State / axis CAN age11ms; fresh release confirmed; report:", root)
        sys.exit(0)
    if mode == "shutdown":
        deadline = time.monotonic() + 5
        while calls["calibrate"] < 1 and time.monotonic() < deadline:
            spin_once()
        assert calls["calibrate"] == 1
        process.send_signal(signal.SIGTERM)
        deadline = time.monotonic() + 5
        while process.poll() is None and time.monotonic() < deadline:
            spin_once()
        assert process.poll() == 0, root
        report = (root / "report.txt").read_text()
        assert "BASELINE_FAILED: 节点关闭 cleanup=RELEASED_IDLE" in report, report
        assert calls == {"calibrate": 1, "release": 1}, calls
        print("PASS SIGTERM finally cleanup confirmed; report:", root)
        sys.exit(0)
    deadline = time.monotonic() + (25 if (fault_release_mode or mode in ("unconfirmed", "wrongref", "release_retry", "release_unavailable", "release_pending_evidence")) else (18 if consumed_mode else 8))
    while time.monotonic() < deadline:
        spin_once()
        response = call("/device_action_gate/baseline_status")
        if "BASELINE_FAILED:" in response.message or "BASELINE_PASSED" in response.message:
            break
    if mode in ("scene_restore_plan_failure", "scene_clear_plan_failure", "scene_blocked_plan_success"):
        assert not response.success and "BASELINE_FAILED:" in response.message, (response, root)
        assert action_calls == 16 and cancel_receipts == 0 and scene_object is None
        assert "cleanup=RELEASED_IDLE" in response.message, response
        assert calls == {"calibrate": 1, "release": 1}
        assert "MoveIt error=" in response.message, response
        if mode == "scene_restore_plan_failure":
            assert len(obstacle_probe_requests) == 3 and len(set(obstacle_probe_requests)) == 1
        elif mode == "scene_clear_plan_failure":
            assert len(obstacle_probe_requests) == 1 and scene_validity_calls == 0
        else:
            assert "规划成功" in response.message and obstacle_rejections == 1
        print("PASS", mode, "A/B/A proof cannot skip clear success or accept blocked success; report:", root)
        sys.exit(0)
    if mode == "scene_unrelated_plan_error":
        assert not response.success and "圆柱规划失败原因并非碰撞路径拒绝" in response.message, response
        assert "MoveIt error=-15" in response.message, response
        assert action_calls == 16 and scene_validity_calls == 2 and obstacle_rejections == 1
        assert cancel_receipts == 0 and scene_object is None
        assert "cleanup=RELEASED_IDLE" in response.message, response
        assert calls == {"calibrate": 1, "release": 1}
        print("PASS", mode, "unrelated error rejected despite same-ID collision; report:", root)
        sys.exit(0)
    if mode == "scene_cancel_evidence_failure":
        assert not response.success and "未确认运行中的目标取消终态" in response.message, response
        assert "cleanup=RELEASED_IDLE" in response.message, response
        assert cancel_receipts == 1 and action_calls == 17 and cancel_future.done()
        report = (root / "report.txt").read_text()
        assert "result_code=6 error_code=-4 error_string=EVIDENCE_INCOMPLETE" in report, (report, root)
        assert "cancel_requested=1 goal_running=1" in report, (report, root)
        print("PASS cancel public result retains evidence failure and cleanup; report:", root)
        sys.exit(0)
    if full_scene_mode:
        assert response.success and "BASELINE_PASSED" in response.message, (response, root)
        assert action_calls == 17 and scene_validity_calls == 2 and obstacle_rejections == 1, (action_calls, scene_validity_calls, obstacle_rejections, root)
        assert cancel_receipts == 1 and cancel_future.done() and scene_object is None
        assert len(obstacle_probe_requests) == 3 and len(set(obstacle_probe_requests)) == 1, root
        assert permit_requests == 0 and calls == {"calibrate": 1, "release": 1}
        report = (root / "report.txt").read_text()
        for event in ("CYLINDER_START_CLEAR", "CYLINDER_COLLISION_CONFIRMED", "OBSTACLE_PLAN_REJECTED", "CYLINDER_CLEAR_PLAN_BEFORE", "CYLINDER_CLEAR_PLAN_AFTER", "CYLINDER_ABA_CONFIRMED", "CANCEL_TERMINAL", "CANCEL_STOP_CONFIRMED", "RELEASE_CONFIRMED"):
            assert event in report, (event, root)
        print("PASS", mode, "nominal obstacle + 17 real public actions/cancel + BASELINE_PASSED; software-only evidence fixtures:", root)
        sys.exit(0)
    assert "BASELINE_FAILED:" in response.message, (response, root)
    if mode == "noowner":
        assert "total_timeout_s必须为1..840" in response.message, response
        assert "cleanup=NOT_ACQUIRED" in response.message, response
        assert calls == {"calibrate": 0, "release": 0}, calls
        print("PASS invalid configuration never acquires owner or sends D10; report:", root)
        sys.exit(0)
    if scene_mode:
        assert action_calls == 16, (action_calls, response, root)
        assert "cleanup=RELEASED_IDLE" in response.message, response
        if mode in ("scene_normalized", "scene_equivalent_local"):
            assert scene_validity_calls == 1, (scene_validity_calls, response, root)
            assert "新增圆柱遮挡当前REST起点" in response.message, response
            assert "SCENE_ROUNDTRIP_CONFIRMED" in (root / "report.txt").read_text()
        else:
            assert scene_validity_calls == 0, (scene_validity_calls, response, root)
            assert "圆柱" in response.message, response
        print("PASS", mode, "16 public Goals then strict scene roundtrip; report:", root)
        sys.exit(0)
    if consumed_mode:
        assert action_calls == (2 if mode == "consumed_permit" else 1), (action_calls, response, root)
        if mode == "consumed_permit":
            assert permit_requests == 1 and permit_refresh_count == 1, (permit_requests, permit_refresh_count, root)
            assert "ARRIVAL_CONFIRMED" in (root / "report.txt").read_text()
            assert "NEXT_MOTION_READY" in (root / "report.txt").read_text()
            assert "公开FJT非成功终态" in response.message, response
        assert "cleanup=RELEASED_IDLE" in response.message, response
        print("PASS", mode, "consumed permit arrival separated from next ready; report:", root)
        sys.exit(0)
    if renewal_mode:
        assert action_calls == (2 if mode == "renewal_retry" else 1), (action_calls, response, root)
        assert "cleanup=RELEASED_IDLE" in response.message, response
        if mode == "renewal_retry":
            assert "公开FJT非成功终态" in response.message, response
        if mode == "renewal_pending":
            assert time.monotonic() - renewal_at >= 1.8, (response, root)
        print("PASS", mode, "UUID-bound single retry; report:", root)
        sys.exit(0)
    if planning_mode:
        if mode in ("planning_duplicate_nodes", "planning_private_scene"):
            assert plan_calls == 1, (response, plan_calls, root)
            assert "REST往返无合法完整路径" in response.message, response
        else:
            assert plan_calls == 0, (response, plan_calls, root)
            assert "规划/场景服务缺失或存在重复实例" in response.message, response
        assert "cleanup=RELEASED_IDLE" in response.message, response
        assert calls == {"calibrate": 1, "release": 1}, calls
        print("PASS", mode, "actual server count preserved; report:", root)
        sys.exit(0)
    expected = "FAULT_INJECTED_CALIBRATION_DENIED" if calibration_denied else "源反馈停止进展"
    assert expected in response.message, response
    if mode == "status_stale":
        # Status单独1秒预算；结束仍须恢复后的fresh双源释放确认。
        assert time.monotonic() - calibration_at >= 0.95, (response, root)
    if mode in ("state_stale_status_live", "can_stale"):
        assert time.monotonic() - calibration_at < 0.8, (response, root)
    cleanup = ("cleanup=RELEASED_FAULT" if mode=="released_fault" else "cleanup=UNCONFIRMED") if fault_release_mode else ("cleanup=UNCONFIRMED" if mode in ("unconfirmed", "wrongref", "release_unavailable") else "cleanup=RELEASED_IDLE")
    assert cleanup in response.message, response
    if mode == "release_retry":
        assert calls == {"calibrate": 1, "release": 3}, calls
    elif mode == "release_unavailable":
        assert calls["calibrate"] == 1 and 2 <= calls["release"] <= 105, calls
    else:
        assert calls == {"calibrate": 1, "release": 1}, calls
    assert not call("/device_action_gate/auto_calibrate_and_test").success
    print("PASS", mode, response.message, "report:", root)
finally:
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
    if action_server is not None:
        action_server.destroy()
    if planning_executor is not None:
        planning_executor.shutdown()
    for peer in planning_peers:
        peer.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
    log.close()
