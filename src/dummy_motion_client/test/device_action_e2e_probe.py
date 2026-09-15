#!/usr/bin/env python3
"""Component-only C05 peer, real ros2_control hardware/JTC, and public FJT probe."""
import json
import os
import pathlib
import socket
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ConfigureController, LoadController, SetHardwareComponentState, SwitchController
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


def wait_future(node, future, timeout=5):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), "ROS response timeout"
    return future.result()


def call(node, service_type, name, request):
    client = node.create_client(service_type, name)
    assert client.wait_for_service(timeout_sec=10), f"service missing: {name}"
    return wait_future(node, client.call_async(request))


def urdf(endpoint):
    return f'''<robot name="component_dummy"><link name="base_link"/><link name="link1"/>
<joint name="Joint1" type="revolute"><parent link="base_link"/><child link="link1"/>
<axis xyz="0 0 1"/><limit lower="-0.5" upper="0.5" velocity="0.25" effort="1"/></joint>
<ros2_control name="dummy" type="system"><hardware>
<plugin>dummy_ros_control/GuardedZenbeddedSystem</plugin>
<param name="endpoint">{endpoint}</param><param name="device_id">01000000000000000000000000000000</param>
<param name="client_instance_id">02000000000000000000000000000000</param>
<param name="profile_id">2</param><param name="configured_mask">1</param><param name="max_state_age_us">1000000</param>
</hardware><joint name="Joint1"><command_interface name="position"/>
<state_interface name="position"><param name="initial_value">0</param></state_interface>
<state_interface name="velocity"><param name="initial_value">0</param></state_interface></joint></ros2_control></robot>'''


def main():
    gate, peer, bundle = (pathlib.Path(value).resolve() for value in sys.argv[1:4])
    unsupported = len(sys.argv) > 4 and sys.argv[4] == "unsupported"
    with socket.socket() as reserve:
        reserve.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{reserve.getsockname()[1]}"
    env = dict(os.environ, ROS_DOMAIN_ID=str(100 + os.getpid() % 100))
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    processes = []
    with tempfile.TemporaryDirectory(prefix="device-action-e2e-") as temporary:
        directory = pathlib.Path(temporary)
        params = {
            "controller_manager": {"ros__parameters": {"update_rate": 100,
                "hardware_components_initial_state": {"inactive": ["dummy"]},
                "arm_controller": {"type": "dummy_ros_control/GuardedJointTrajectoryController"}}},
            "arm_controller": {"ros__parameters": {"joints": ["Joint1"],
                "command_interfaces": ["position"], "state_interfaces": ["position", "velocity"],
                "allow_partial_joints_goal": False, "allow_nonzero_velocity_at_trajectory_end": False}},
            "device_action_gate": {"ros__parameters": {"endpoint": endpoint,
                "device_id": "01000000000000000000000000000000", "client_instance_id": "02000000000000000000000000000000",
                "profile_id": 2, "configured_mask": 1, "contract_bundle": str(bundle), "profile_name": "sil_j1"}},
            "robot_state_publisher": {"ros__parameters": {"robot_description": urdf(endpoint)}},
        }
        configuration = directory / "params.yaml"
        configuration.write_text(yaml.safe_dump(params))
        logs = {}

        def launch(name, command):
            log = open(directory / f"{name}.log", "a+")
            logs[name] = log
            process = subprocess.Popen(command, env=env, stdout=log, stderr=log)
            processes.append(process)
            return process

        try:
            launch("peer", [str(peer), "--endpoint", endpoint, "--device-id", "01000000000000000000000000000000",
                            "--client-id", "02000000000000000000000000000000", "--profile-id", "2", "--configured-mask", "1",
                            "--contract-bundle", str(bundle), "--profile-name", "sil_j1",
                            "--motion-supported", "false" if unsupported else "true"])
            time.sleep(0.4)
            launch("gate", [str(gate), "--ros-args", "--params-file", str(configuration)])
            launch("description", ["/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher", "--ros-args", "--params-file", str(configuration)])
            rclpy.init()
            node = rclpy.create_node("device_action_e2e_probe")
            service = "/controller_manager/set_hardware_component_state"
            client = node.create_client(SetHardwareComponentState, service)
            assert client.wait_for_service(timeout_sec=10), "manager did not start"
            time.sleep(1)
            request = SetHardwareComponentState.Request(name="dummy")
            request.target_state.id = 3
            assert wait_future(node, client.call_async(request)).ok, "hardware activation failed"
            assert call(node, LoadController, "/controller_manager/load_controller", LoadController.Request(name="arm_controller")).ok
            assert call(node, ConfigureController, "/controller_manager/configure_controller", ConfigureController.Request(name="arm_controller")).ok
            switch = SwitchController.Request(activate_controllers=["arm_controller"], strictness=2)
            switch.timeout.sec = 5
            assert call(node, SwitchController, "/controller_manager/switch_controller", switch).ok
            action = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
            assert action.wait_for_server(timeout_sec=5)
            time.sleep(0.3)
            logs["peer"].seek(0)
            assert '"event":"manage"' not in logs["peer"].read(), "startup implicitly issued Enable"
            enable_response = call(node, Trigger, "/device_action_gate/enable", Trigger.Request())
            if unsupported:
                assert not enable_response.success, "motion-unsupported device accepted Enable"
                logs["peer"].seek(0)
                assert '"event":"manage"' not in logs["peer"].read(), "mutation sent to unsupported peer"
                print(json.dumps({"evidence_scope": "component_only", "motion_unsupported_rejected": True, "mutation_count": 0}))
                node.destroy_node()
                rclpy.shutdown()
                return
            assert enable_response.success, "explicit enable unavailable"
            time.sleep(0.5)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ["Joint1"]
            point = JointTrajectoryPoint(positions=[0.05])
            point.time_from_start.sec = 1
            goal.trajectory.points = [point]
            feedback = []
            handle = wait_future(node, action.send_goal_async(goal, feedback_callback=lambda value: feedback.append(value.feedback)))
            assert handle.accepted, "public goal rejected after Enable"
            result = wait_future(node, handle.get_result_async(), 10)
            assert result.status == 4 and result.result.error_code == 0, f"motion did not succeed: {result}"
            assert feedback, "public FJT did not publish feedback"
            assert all(item.joint_names == ["Joint1"] and len(item.actual.positions) == 1 and
                       len(item.desired.positions) == 1 for item in feedback), "feedback axis shape"
            assert any(abs(item.actual.positions[0] - 0.05) > 0.005 for item in feedback), "actual feedback mirrored goal"
            elapsed = [item.actual.time_from_start.sec + item.actual.time_from_start.nanosec * 1e-9 for item in feedback]
            assert elapsed == sorted(elapsed) and 0 <= elapsed[0] < 0.1 and elapsed[-1] < 2, "feedback clock"

            def send(target, duration=1, tolerance=None):
                motion = FollowJointTrajectory.Goal()
                motion.trajectory.joint_names = ["Joint1"]
                sample = JointTrajectoryPoint(positions=[target])
                sample.time_from_start.sec = duration
                motion.trajectory.points = [sample]
                if tolerance is not None:
                    motion.path_tolerance = [JointTolerance(name="Joint1", position=tolerance)]
                stream = []
                accepted = wait_future(node, action.send_goal_async(motion, feedback_callback=lambda value: stream.append(value.feedback)))
                return accepted, stream

            second, second_feedback = send(0.08)
            assert second.accepted, "second goal was not admitted after confirmed HOLD"
            second_result = wait_future(node, second.get_result_async(), 10)
            assert second_result.status == 4, f"second goal failed: {second_result}"
            assert second_feedback and abs(second_feedback[0].actual.positions[0] - 0.05) < 0.002, "second goal jumped to home"
            canceled, cancel_feedback = send(0.15, 3)
            assert canceled.accepted
            overlap, _ = send(0.1)
            assert not overlap.accepted, "overlapping public goal accepted"
            until_cancel = time.monotonic() + 0.2
            while time.monotonic() < until_cancel:
                rclpy.spin_once(node, timeout_sec=0.01)
            cancel_response = wait_future(node, canceled.cancel_goal_async())
            assert cancel_response.goals_canceling, "cancel request not accepted"
            canceled_result = wait_future(node, canceled.get_result_async(), 10)
            assert canceled_result.status == 5, f"cancel did not cross confirmed HOLD barrier: {canceled_result}"
            aborted, abort_feedback = send(0.12, tolerance=1e-7)
            assert aborted.accepted, "cancel barrier did not reopen admission"
            aborted_result = wait_future(node, aborted.get_result_async(), 10)
            assert aborted_result.status == 6 and "PATH_TOLERANCE" in aborted_result.result.error_string, f"abort path not confirmed: {aborted_result}"
            logs["peer"].seek(0)
            events = [json.loads(line) for line in logs["peer"].read().splitlines() if line.startswith('{"event"')]
            operations = [event["operation"] for event in events if event["event"] == "manage"]
            assert operations == [5, 6, 13, 6, 13, 6, 7, 6, 7], operations
            commands = [event for event in events if event["event"] == "command"]
            sequences = [event["producer_seq"] for event in commands]
            assert sequences == sorted(set(sequences)), "producer generations repeated or regressed"
            ticket_count = len({event["freshness_ticket"] for event in commands})
            assert ticket_count > 20, "peer did not rotate freshness tickets during motion"
            frozen = []
            for index, event in enumerate(events):
                if event["event"] != "manage" or event["operation"] != 13:
                    continue
                prior = [value for value in events[:index] if value["event"] == "command" and value["motion_epoch"] == event["motion_epoch"]][-1]
                later = [value for value in events[index + 1:] if value["event"] == "command"]
                assert not any(value["motion_epoch"] == event["motion_epoch"] for value in later), "command emitted after terminal freeze"
                assert later and abs(later[0]["position"][0] - prior["position"][0]) < 1e-9, "completion hold jumped"
                frozen.append({"motion_epoch": prior["motion_epoch"], "producer_seq": prior["producer_seq"], "position": prior["position"][0]})
            results = [result, second_result, canceled_result, aborted_result]
            print(json.dumps({"evidence_scope": "component_only", "public_fjt_succeeded": result.status == 4,
                              "success_count": sum(item.status == 4 for item in results),
                              "cancel_count": sum(item.status == 5 for item in results),
                              "abort_count": sum(item.status == 6 for item in results),
                              "feedback_count": sum(map(len, [feedback, second_feedback, cancel_feedback, abort_feedback])),
                              "terminal_freezes": frozen, "operations": operations,
                              "distinct_freshness_tickets": ticket_count,
                              "real_hardware_plugin": True, "real_guarded_jtc": True}))
            node.destroy_node()
            rclpy.shutdown()
        except BaseException:
            for name, log in logs.items():
                log.seek(0)
                print(f"{name}:\n{log.read()[-18000:]}", file=sys.stderr)
            raise
        finally:
            for process in reversed(processes):
                process.terminate()
            for process in reversed(processes):
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for log in logs.values(): log.close()


if __name__ == "__main__":
    main()
