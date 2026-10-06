#!/usr/bin/env python3
"""Exercise the actual public ROS action server before any device mutation."""
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger


def main():
    executable = pathlib.Path(sys.argv[1])
    assert executable.is_file(), "production device_action_gate executable is missing"
    env = dict(os.environ, ROS_DOMAIN_ID=str(100 + os.getpid() % 100))
    os.environ.update(ROS_DOMAIN_ID=env["ROS_DOMAIN_ID"])
    options = [
        str(executable), "--ros-args", "-p", "endpoint:=tcp/127.0.0.1:29997",
        "-p", 'device_id:="01000000000000000000000000000000"',
        "-p", 'client_instance_id:="02000000000000000000000000000000"',
        "-p", "profile_id:=2", "-p", "configured_mask:=1",
        "-p", "session_id:=11", "-p", 'permit_sha256:="01' + "00" * 31 + '"',
    ]
    missing = subprocess.run(options, env=env, capture_output=True, text=True, timeout=3)
    assert missing.returncode != 0 and "bundle" in missing.stderr, "missing profile bundle must fail startup"
    bundle = pathlib.Path(__file__).resolve().parents[3] / "contracts" / "dummy_robot_v2"
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(options + ["-p", f"contract_bundle:={bundle}", "-p", "profile_name:=sil_j1"],
                                   env=env, stdout=log, stderr=log)
        try:
            rclpy.init()
            node = rclpy.create_node("public_gate_attacker")
            client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
            if not client.wait_for_server(timeout_sec=10):
                log.seek(0)
                raise AssertionError("public gate did not start: " + log.read())
            enable = node.create_client(Trigger, "/device_action_gate/enable")
            assert enable.wait_for_service(timeout_sec=2), "explicit enable service missing"
            response = enable.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, response, timeout_sec=2)
            assert response.done() and not response.result().success, "enable without device authority was accepted"
            renewal = node.create_client(Trigger, "/device_action_gate/renew_session_lease")
            assert renewal.wait_for_service(timeout_sec=2), "explicit lease renewal service missing"
            response = renewal.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, response, timeout_sec=2)
            assert response.done() and not response.result().success, "renewal without device authority was accepted"
            rejection = node.create_client(Trigger, "/device_action_gate/last_goal_rejection")
            assert rejection.wait_for_service(timeout_sec=2), "goal rejection receipt service missing"
            response = rejection.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, response, timeout_sec=2)
            assert response.done() and not response.result().success, "receipt exists before goal rejection"
            variants = ("nan", "infinity", "unknown_joint", "empty", "negative_time", "terminal_velocity", "out_of_range")
            for variant in variants:
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = ["Joint1"]
                point = JointTrajectoryPoint(positions=[0.1])
                point.time_from_start.sec = 1
                goal.trajectory.points = [point]
                if variant == "nan": point.positions = [math.nan]
                if variant == "infinity": point.positions = [math.inf]
                if variant == "unknown_joint": goal.trajectory.joint_names = ["forged"]
                if variant == "empty": goal.trajectory.points = []
                if variant == "negative_time": point.time_from_start.sec = -1
                if variant == "terminal_velocity": point.velocities = [0.2]
                if variant == "out_of_range": point.positions = [100]
                future = client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                assert future.done() and not future.result().accepted, variant
            response = rejection.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, response, timeout_sec=2)
            assert response.done() and not response.result().success, "invalid goals produced a renewal receipt"
            time.sleep(0.1)
            log.seek(0)
            output = log.read()
            assert output.count("DEVICE_GOAL_REJECTED_VALIDATION") == len(variants), output
            print(json.dumps({"evidence_scope": "component_only", "public_action_validation": True,
                              "rejected_variants": list(variants)}))
            node.destroy_node()
            rclpy.shutdown()
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=5) == 0, "gate did not exit cleanly after SIGINT"
        finally:
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    main()
