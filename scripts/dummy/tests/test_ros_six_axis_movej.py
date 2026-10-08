"""六轴 ROS 目标与规划结果的本地边界。"""

import importlib.util
import json
from pathlib import Path
import sys
import types
from types import SimpleNamespace

import pytest


MODULE = Path(__file__).resolve().parents[1] / "ros_six_axis_movej.py"
SPEC = importlib.util.spec_from_file_location("ros_six_axis_movej", MODULE)
movej = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(movej)


def test_target_requires_six_distinct_bounded_axes():
    assert movej.validate_target([0.01, -0.02, 1.59, -0.01, 0.02, -0.03])
    with pytest.raises(ValueError):
        movej.validate_target([0.01])
    with pytest.raises(ValueError):
        movej.validate_target([0, 0, 0, 0, 0, float("nan")])
    with pytest.raises(ValueError):
        movej.validate_target([0, 0, 0, 0, 0, 13])


def test_integer_targets_are_normalized_for_ros_float_fields():
    target = movej.validate_target([0, 0, 0, 0, 0, 0], [(-0.6, 0.6)] * 6)
    assert target == (0.0,) * 6
    assert all(type(value) is float for value in target)


def test_integer_json_target_serializes_as_ros_joint_constraint(tmp_path):
    pytest.importorskip("moveit_msgs.msg")
    from moveit_msgs.msg import JointConstraint
    from rclpy.serialization import serialize_message

    path = tmp_path / "targets.json"
    path.write_text("[[0, 0, 0, 0, 0, 0]]")
    target = movej.load_targets_json(path, [(-0.6, 0.6)] * 6)[0]
    assert serialize_message(JointConstraint(joint_name="Joint1", position=target[0]))


def test_targets_json_is_bounded_and_validated_before_ros(tmp_path):
    path = tmp_path / "targets.json"
    first = [0, 0, 1.59, 0, 0, 0]
    second = [0.01, -0.01, 1.6, 0, 0, 0]
    path.write_text(json.dumps([first, second]))
    assert movej.load_targets_json(path) == (tuple(first), tuple(second))
    for bad in ([], [first] * 51, [first, None], [first, [0] * 5],
                [first, [0, 0, 1.59, 0, 0, 13]],
                {"targets": [first]}, [first, [0, 0, 1.59, 0, 0, True]]):
        path.write_text(json.dumps(bad))
        with pytest.raises(ValueError):
            movej.load_targets_json(path)
    path.write_text(" " * 65537)
    with pytest.raises(ValueError, match="size"):
        movej.load_targets_json(path)


def test_multi_goal_sequence_refreshes_permit_and_releases_only_once(monkeypatch):
    adapter = FakeLifecycleAdapter()
    events = []
    adapter.renew_motion_permit = lambda: events.append("permit")
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    targets = ((0, 0, 1.59, 0, 0, 0), (0.01, 0, 1.6, 0, 0, 0))

    def perform(target, index):
        events.append(("goal", index))
        return {"goal_id": f"{index + 1:032x}", "status": "SUCCEEDED",
                "target_rad": list(target), "feedback": [{"actual_rad": list(target)}]}

    report = movej.run_guarded_sequence(adapter, ["hardware"], targets, perform)
    assert events == [("goal", 0), "permit", ("goal", 1), "release"]
    assert [goal["goal_id"] for goal in report["goals"]] == [f"{index:032x}" for index in (1, 2)]
    assert report["scope"] == "ROS_ACTION_SESSION_ONLY"
    assert report["status"] == "SUCCEEDED"
    assert report["stop_confirmation"] == "CONFIRMED_RELEASED_IDLE"


def test_multi_goal_failure_stops_and_retains_completed_goal(monkeypatch):
    adapter = FakeLifecycleAdapter()
    events = []
    adapter.renew_motion_permit = lambda: events.append("permit")
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))

    def perform(target, index):
        events.append(("goal", index))
        if index == 1:
            raise RuntimeError("second goal aborted")
        return {"goal_id": "1" * 32, "status": "SUCCEEDED"}

    with pytest.raises(movej.ConfirmedStopError, match="second goal aborted") as error:
        movej.run_guarded_sequence(adapter, ["hardware"], ((0,) * 6,) * 2, perform)
    assert events == [("goal", 0), "permit", ("goal", 1), "release"]
    assert error.value.failed_goal_index == 1
    assert error.value.completed_goals == [{"goal_id": "1" * 32,
                                            "status": "SUCCEEDED", "index": 0}]


def test_second_goal_permit_timeout_releases_before_sending_goal(monkeypatch):
    adapter = FakeLifecycleAdapter()
    events = []
    adapter.renew_motion_permit = lambda: (_ for _ in ()).throw(
        RuntimeError("new motion permit deadline expired"))
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))

    def perform(target, index):
        events.append(("goal", index))
        return {"goal_id": "1" * 32, "status": "SUCCEEDED"}

    with pytest.raises(movej.ConfirmedStopError) as error:
        movej.run_guarded_sequence(adapter, [], ((0,) * 6,) * 2, perform, 5.0)
    assert events == [("goal", 0), "release"]
    assert error.value.failed_goal_index == 1
    assert len(error.value.completed_goals) == 1


def test_failed_accepted_goal_uuid_is_kept_in_negative_session_report(monkeypatch):
    adapter = FakeLifecycleAdapter()
    adapter.renew_motion_permit = lambda: None
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned: ([], []))
    failed = {"goal_id": "2" * 32, "trajectory_sha256": "a" * 64,
              "status": "FAILED", "reason": "action aborted"}

    def perform(target, index):
        if index:
            raise movej.GoalAttemptError("action aborted", failed)
        return {"goal_id": "1" * 32, "status": "SUCCEEDED"}

    with pytest.raises(movej.ConfirmedStopError) as error:
        movej.run_guarded_sequence(adapter, [], ((0,) * 6,) * 2, perform)
    assert error.value.failed_goal_report == {**failed, "index": 1}
    assert len(error.value.completed_goals) == 1


@pytest.mark.parametrize("fail_at", [1, 2])
def test_multi_goal_unconfirmed_stop_never_reports_session_success(monkeypatch, fail_at):
    adapter = FakeLifecycleAdapter()
    adapter.renew_motion_permit = lambda: None
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned: (
        _ for _ in ()).throw(movej.UnconfirmedStopError(
            "UNCONFIRMED_STOP", ["dummy_ref_real_6dof"], ["arm_controller"])))

    def perform(target, index):
        if index == fail_at:
            raise RuntimeError("goal failed")
        return {"goal_id": f"{index + 1:032x}", "status": "SUCCEEDED"}

    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.run_guarded_sequence(adapter, [], ((0,) * 6,) * 2, perform)
    assert len(error.value.completed_goals) == (1 if fail_at == 1 else 2)
    assert error.value.failed_goal_index == fail_at
    assert error.value.still_active_components == ["dummy_ref_real_6dof"]
    assert error.value.unresolved_components == ["arm_controller"]


@pytest.mark.parametrize("state,expected_active,expected_unknown", [
    ("active", ["dummy_ref_real_6dof"], []),
    ("unknown", [], ["dummy_ref_real_6dof"]),
])
def test_preactivated_stack_must_be_inactive_after_final_release(
        state, expected_active, expected_unknown):
    releases = []
    adapter = SimpleNamespace(
        settle_pending=lambda deadline: True,
        confirmed_stop=lambda: releases.append("release") or True,
        hardware_state=lambda name, deadline: state,
        controller_state=lambda name, deadline: "inactive")
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.run_guarded_sequence(adapter, [], ((0, 0, 1.59, 0, 0, 0),),
                                   lambda target, index: {
                                       "goal_id": "1" * 32, "status": "SUCCEEDED"})
    assert releases == ["release"]
    assert error.value.completed_goals[0]["goal_id"] == "1" * 32
    assert error.value.still_active_components == expected_active
    assert error.value.unresolved_components == expected_unknown


def test_duplicate_goal_uuid_stops_before_next_goal(monkeypatch):
    adapter = FakeLifecycleAdapter()
    adapter.renew_motion_permit = lambda: None
    events = []
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    with pytest.raises(movej.ConfirmedStopError, match="duplicate") as error:
        movej.run_guarded_sequence(adapter, [], ((0,) * 6,) * 2,
                                   lambda target, index:
                                   {"goal_id": "1" * 32, "status": "SUCCEEDED"})
    assert events == ["release"]
    assert len(error.value.completed_goals) == 1


def test_fifty_goal_session_has_one_release_and_unique_goal_reports(monkeypatch):
    adapter = FakeLifecycleAdapter()
    events = []
    adapter.renew_motion_permit = lambda: events.append("permit")
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    report = movej.run_guarded_sequence(adapter, [], ((0,) * 6,) * 50,
        lambda target, index: {"goal_id": f"{index + 1:032x}", "status": "SUCCEEDED"})
    assert report["requested_goals"] == report["completed_goals"] == 50
    assert len({goal["goal_id"] for goal in report["goals"]}) == 50
    assert events.count("permit") == 49
    assert events.count("release") == 1


def test_multi_goal_interruption_still_requests_guarded_release(monkeypatch):
    class Interrupted(BaseException):
        pass

    adapter = FakeLifecycleAdapter()
    events = []
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    with pytest.raises(movej.ConfirmedStopError, match="interrupted"):
        movej.run_guarded_sequence(adapter, [], ((0,) * 6,),
                                   lambda target, index: (_ for _ in ()).throw(
                                       Interrupted("interrupted")))
    assert events == ["release"]


def test_capture_many_uses_one_ros_owner_and_one_final_release(monkeypatch):
    events = []
    node = SimpleNamespace(create_client=lambda kind, name: events.append("planner") or
                           SimpleNamespace(wait_for_service=lambda timeout_sec: True),
                           destroy_node=lambda: events.append("destroy_node"))
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: events.append("init")
    rclpy.create_node = lambda name: node
    rclpy.shutdown = lambda: events.append("shutdown")
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(GetMotionPlan=object))

    class ActionClient:
        def __init__(self, node, kind, name):
            events.append("action_client")

        def wait_for_server(self, timeout_sec):
            return True

    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(ActionClient=ActionClient))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(FollowJointTrajectory=object))
    adapter = FakeLifecycleAdapter()
    adapter.deadline = movej.time.monotonic() + 5
    adapter.renew_motion_permit = lambda: events.append("permit")
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: events.append("bind"))
    monkeypatch.setattr(movej, "activate_controller_stack", lambda adapter, owned:
                        events.append("activate") or owned.append("hardware"))
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    monkeypatch.setattr(movej, "execute_goal", lambda node, planner, client, target, limits,
                        deadline, admission_adapter:
                        events.append(("goal", target)) or
                        {"goal_id": f"{len([item for item in events if isinstance(item, tuple)]):032x}",
                         "status": "SUCCEEDED", "target_rad": list(target)})
    targets = ((0, 0, 1.59, 0, 0, 0), (0.01, 0, 1.6, 0, 0, 0))
    report = movej.capture_many(targets, 5.0, movej.LIMITS,
                                activation_sha256="a" * 64, device_id="b" * 32,
                                candidate_manifest=Path("/tmp/bundle/manifest.json"),
                                activate_controller=True)
    assert events == ["init", "bind", "activate", "planner", "action_client",
                      ("goal", targets[0]), "permit", ("goal", targets[1]),
                      "release", "destroy_node", "shutdown"]
    assert report["completed_goals"] == 2


@pytest.mark.parametrize("overflow", [False, True])
def test_execute_goal_retains_uuid_on_terminal_failure_and_bounds_feedback(monkeypatch, overflow):
    target = (0, 0, 1.59, 0, 0, 0)
    trajectory = digest_vector()
    trajectory.joint_names = list(movej.JOINTS)
    trajectory.points[0].positions = list(target)
    plan = SimpleNamespace(motion_plan_response=SimpleNamespace(
        error_code=SimpleNamespace(val=1), trajectory=SimpleNamespace(joint_trajectory=trajectory)))
    handle = SimpleNamespace(accepted=True, goal_id=SimpleNamespace(uuid=bytes.fromhex("2" * 32)))
    monkeypatch.setitem(sys.modules, "action_msgs.msg", SimpleNamespace(
        GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=4)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(
        FollowJointTrajectory=SimpleNamespace(Goal=lambda: SimpleNamespace(trajectory=None))))
    monkeypatch.setitem(sys.modules, "moveit_msgs.msg", SimpleNamespace(
        Constraints=lambda **kwargs: SimpleNamespace(**kwargs),
        JointConstraint=lambda **kwargs: SimpleNamespace(**kwargs),
        MotionPlanRequest=lambda: SimpleNamespace(start_state=SimpleNamespace(is_diff=False))))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(
        GetMotionPlan=SimpleNamespace(Request=lambda: SimpleNamespace())))
    monkeypatch.setitem(sys.modules, "rclpy.serialization", SimpleNamespace(
        serialize_message=lambda value: b"trajectory"))
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout:
                        plan if future == "plan" else handle)
    monkeypatch.setattr(movej, "await_accepted_goal", lambda node, goal, timeout:
                        SimpleNamespace(status=4 if overflow else 6,
                                        result=SimpleNamespace(
                                            error_code=0 if overflow else -4,
                                            error_string="Joint3 path velocity tolerance")))
    sample = SimpleNamespace(feedback=SimpleNamespace(joint_names=list(movej.JOINTS),
        actual=SimpleNamespace(positions=target), desired=SimpleNamespace(positions=target)))

    def send_goal(goal, feedback_callback):
        for _ in range(movej.MAX_FEEDBACK_SAMPLES + 1 if overflow else 1):
            feedback_callback(sample)
        return "goal"

    client = SimpleNamespace(send_goal_async=send_goal)
    planner = SimpleNamespace(call_async=lambda request: "plan")
    with pytest.raises(movej.GoalAttemptError) as error:
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5)
    assert error.value.goal_report["goal_id"] == "2" * 32
    assert error.value.goal_report["trajectory_sha256"] == (
        "76604aa34f36c7ba7d9fcbf6ea2621f712b44e5dd1934975c3c07547ae19c022")
    assert error.value.goal_report["status"] == "FAILED"
    assert error.value.goal_report["result_error_string"] == "Joint3 path velocity tolerance"
    assert len(error.value.goal_report["feedback"]) <= movej.MAX_FEEDBACK_SAMPLES


def admission_case(monkeypatch, statuses, accepted_on_attempt, receipts=None):
    target = (0, 0, 1.59, 0, 0, 0)
    trajectory = digest_vector()
    trajectory.joint_names = list(movej.JOINTS)
    trajectory.points[0].positions = list(target)
    plan = SimpleNamespace(motion_plan_response=SimpleNamespace(
        error_code=SimpleNamespace(val=1), trajectory=SimpleNamespace(joint_trajectory=trajectory)))
    monkeypatch.setitem(sys.modules, "action_msgs.msg", SimpleNamespace(
        GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=4)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(
        FollowJointTrajectory=SimpleNamespace(Goal=lambda: SimpleNamespace(trajectory=None))))
    monkeypatch.setitem(sys.modules, "moveit_msgs.msg", SimpleNamespace(
        Constraints=lambda **kwargs: SimpleNamespace(**kwargs),
        JointConstraint=lambda **kwargs: SimpleNamespace(**kwargs),
        MotionPlanRequest=lambda: SimpleNamespace(start_state=SimpleNamespace(is_diff=False))))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(
        GetMotionPlan=SimpleNamespace(Request=lambda: SimpleNamespace())))
    monkeypatch.setitem(sys.modules, "rclpy.serialization", SimpleNamespace(
        serialize_message=lambda value: b"planned-trajectory"))
    events = []
    sent_goals = []
    status_iter = iter(statuses)
    receipt_iter = iter(receipts if receipts is not None else ())
    adapter = SimpleNamespace(Trigger=SimpleNamespace(Request=lambda: object()))

    def call(name, kind, request, deadline=None):
        if name == "/device_action_gate/last_goal_rejection":
            events.append(("receipt", len(sent_goals)))
            return next(receipt_iter, SimpleNamespace(success=False, message=""))
        assert name == "/device_action_gate/bootstrap_status"
        status = next(status_iter)
        events.append(("status", status))
        return SimpleNamespace(success=status == "ENABLED_HOLD", message=status)

    adapter.call = call

    def send_goal(goal, feedback_callback):
        sent_goals.append(goal)
        events.append(("send", len(sent_goals)))
        if len(sent_goals) == accepted_on_attempt:
            feedback_callback(SimpleNamespace(
                goal_id=SimpleNamespace(uuid=bytes([len(sent_goals)]) * 16),
                feedback=SimpleNamespace(
                joint_names=list(movej.JOINTS),
                actual=SimpleNamespace(positions=target),
                desired=SimpleNamespace(positions=target))))
        return len(sent_goals)

    def fake_wait(node, future, timeout):
        if future == "plan":
            return plan
        return SimpleNamespace(accepted=future == accepted_on_attempt,
                               goal_id=SimpleNamespace(uuid=bytes([future]) * 16))

    monkeypatch.setattr(movej, "wait", fake_wait)
    monkeypatch.setattr(movej, "await_accepted_goal", lambda node, handle, timeout:
                        events.append("result") or SimpleNamespace(
                            status=4, result=SimpleNamespace(
                                error_code=0, error_string="")))
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: events.append("sleep"))
    planner = SimpleNamespace(call_async=lambda request: "plan")
    client = SimpleNamespace(send_goal_async=send_goal)
    return target, trajectory, planner, client, adapter, events, sent_goals


def test_pending_admission_resends_same_plan_and_reports_only_accepted_uuid(monkeypatch):
    target, trajectory, planner, client, adapter, events, goals = admission_case(
        monkeypatch, ("LEASE_RENEWAL_PENDING", "BOOTSTRAP_PENDING", "ENABLED_HOLD"), 2,
        (SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "01" * 16),))
    report = movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                                movej.time.monotonic() + 5, adapter)
    assert len(goals) == 2 and goals[0] is goals[1]
    assert goals[0].trajectory is trajectory
    assert report["goal_id"] == "02" * 16
    assert report["trajectory_sha256"] == movej.trajectory_digest(trajectory)
    assert report["result_error_string"] == ""
    assert len(report["feedback"]) == 1
    assert events == [("send", 1), ("receipt", 1), ("status", "LEASE_RENEWAL_PENDING"),
                      "sleep", ("status", "BOOTSTRAP_PENDING"), "sleep",
                      ("status", "ENABLED_HOLD"), ("send", 2), "result"]


def test_matching_receipt_retries_when_status_is_already_ready(monkeypatch):
    target, trajectory, planner, client, adapter, events, goals = admission_case(
        monkeypatch, ("ENABLED_HOLD",), 2,
        (SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "01" * 16),))
    report = movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                                movej.time.monotonic() + 5, adapter)
    assert len(goals) == 2 and goals[0] is goals[1]
    assert goals[0].trajectory is trajectory
    assert report["goal_id"] == "02" * 16
    assert events == [("send", 1), ("receipt", 1), ("status", "ENABLED_HOLD"),
                      ("send", 2), "result"]


def test_cancel_execute_goal_tags_feedback_with_action_uuid(monkeypatch):
    target, _, planner, client, adapter, _, _ = admission_case(monkeypatch, (), 1)
    observed = []

    def await_cancel(node, handle, feedback, delta, deadline, accepted_index):
        observed.extend(feedback)
        assert accepted_index == 1
        return (SimpleNamespace(status=5, result=SimpleNamespace(
                    error_code=0, error_string="canceled")),
                {"goal_id": "01" * 16, "acknowledged": True})

    monkeypatch.setattr(movej, "await_mid_motion_cancel", await_cancel)
    report = movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                                movej.time.monotonic() + 5, adapter,
                                cancel_mid_motion=True)
    assert observed[0]["goal_id"] == "01" * 16
    assert report["action_status"] == 5


@pytest.mark.parametrize("reply", [
    SimpleNamespace(success=False, message="RENEWAL_PENDING:" + "01" * 16),
    SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "03" * 16),
    SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "01" * 15),
    SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "01" * 16 + ";FAULT"),
    SimpleNamespace(success=True, message="renewal_pending:" + "01" * 16),
    SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "AA" * 16),
])
def test_unknown_or_malformed_receipt_never_resends(monkeypatch, reply):
    target, _, planner, client, adapter, events, goals = admission_case(
        monkeypatch, ("ENABLED_HOLD",), 2, (reply,))
    with pytest.raises(movej.GoalAttemptError):
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter)
    assert len(goals) == 1
    assert events == [("send", 1), ("receipt", 1)]


def test_unavailable_receipt_never_resends_even_while_pending(monkeypatch):
    target, _, planner, client, adapter, events, goals = admission_case(
        monkeypatch, ("LEASE_RENEWAL_PENDING",), 2)
    with pytest.raises(movej.GoalAttemptError):
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter)
    assert len(goals) == 1
    assert events == [("send", 1), ("receipt", 1)]


def test_receipt_service_error_never_resends(monkeypatch):
    target, _, planner, client, adapter, events, goals = admission_case(
        monkeypatch, ("LEASE_RENEWAL_PENDING",), 2)
    original_call = adapter.call

    def call(name, kind, request, deadline=None):
        if name == "/device_action_gate/last_goal_rejection":
            raise TimeoutError("receipt service timed out")
        return original_call(name, kind, request, deadline)

    adapter.call = call
    with pytest.raises(movej.GoalAttemptError):
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter)
    assert len(goals) == 1
    assert events == [("send", 1)]


def test_invalid_plan_never_reaches_admission_receipt_or_action(monkeypatch):
    target, trajectory, planner, client, adapter, events, goals = admission_case(
        monkeypatch, ("ENABLED_HOLD",), 2,
        (SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "01" * 16),))
    trajectory.points[0].positions[0] = 0.5
    with pytest.raises(ValueError, match="terminal differs"):
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter)
    assert goals == []
    assert events == []


@pytest.mark.parametrize("status", ["FAULT", "AUTHORITY_CHANGED",
                                    "RELEASE_PENDING", "EVIDENCE_INCOMPLETE;MISSING",
                                    "OTHER"])
def test_rejection_without_explicit_admission_pending_never_resends(monkeypatch, status):
    target, _, planner, client, adapter, events, goals = admission_case(
        monkeypatch, (status,), 2,
        (SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "01" * 16),))
    with pytest.raises(movej.GoalAttemptError) as error:
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter)
    assert len(goals) == 1
    assert "result" not in events
    assert "goal_id" not in error.value.goal_report
    assert movej.rejection_diagnostic_from_error(error.value)["trajectory_sha256"]


def test_pending_admission_deadline_stops_without_resending(monkeypatch):
    target, _, planner, client, adapter, events, goals = admission_case(
        monkeypatch, ("LEASE_RENEWAL_PENDING",), 2,
        (SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "01" * 16),))
    clock = [0.0]
    monkeypatch.setattr(movej.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    with pytest.raises(movej.GoalAttemptError, match="deadline"):
        movej.execute_goal(object(), planner, client, target, movej.LIMITS, 0.04, adapter)
    assert len(goals) == 1
    assert "result" not in events
    assert clock[0] <= 0.04


def test_pending_admission_fault_stops_without_resending(monkeypatch):
    target, _, planner, client, adapter, events, goals = admission_case(
        monkeypatch, ("LEASE_RENEWAL_PENDING", "FAULT"), 2,
        (SimpleNamespace(success=True, message="RENEWAL_PENDING:" + "01" * 16),))
    with pytest.raises(movej.GoalAttemptError):
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter)
    assert len(goals) == 1
    assert "result" not in events


def test_repeated_pending_admission_has_attempt_cap(monkeypatch):
    statuses = ("BOOTSTRAP_PENDING", "ENABLED_HOLD") * movej.MAX_ADMISSION_RETRIES
    receipts = tuple(SimpleNamespace(success=True,
                     message="RENEWAL_PENDING:" + f"{attempt:02x}" * 16)
                     for attempt in range(1, movej.MAX_ADMISSION_RETRIES + 1))
    target, _, planner, client, adapter, events, goals = admission_case(
        monkeypatch, statuses, movej.MAX_ADMISSION_RETRIES + 2, receipts)
    with pytest.raises(movej.GoalAttemptError):
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter)
    assert len(goals) == movej.MAX_ADMISSION_RETRIES + 1
    assert "result" not in events


def test_acceptance_timeout_reports_known_target_and_hash_without_uuid(monkeypatch):
    target = (0, 0, 1.59, 0, 0, 0)
    trajectory = digest_vector()
    trajectory.joint_names = list(movej.JOINTS)
    trajectory.points[0].positions = list(target)
    plan = SimpleNamespace(motion_plan_response=SimpleNamespace(
        error_code=SimpleNamespace(val=1), trajectory=SimpleNamespace(joint_trajectory=trajectory)))
    monkeypatch.setitem(sys.modules, "action_msgs.msg", SimpleNamespace(GoalStatus=object))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(
        FollowJointTrajectory=SimpleNamespace(Goal=lambda: SimpleNamespace(trajectory=None))))
    monkeypatch.setitem(sys.modules, "moveit_msgs.msg", SimpleNamespace(
        Constraints=lambda **kwargs: SimpleNamespace(**kwargs),
        JointConstraint=lambda **kwargs: SimpleNamespace(**kwargs),
        MotionPlanRequest=lambda: SimpleNamespace(start_state=SimpleNamespace(is_diff=False))))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(
        GetMotionPlan=SimpleNamespace(Request=lambda: SimpleNamespace())))
    monkeypatch.setitem(sys.modules, "rclpy.serialization", SimpleNamespace(
        serialize_message=lambda value: b"planned-trajectory"))

    def wait_for_acceptance(node, future, timeout):
        if future == "plan":
            return plan
        raise RuntimeError("acceptance timed out")

    monkeypatch.setattr(movej, "wait", wait_for_acceptance)
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned: ([], []))
    planner = SimpleNamespace(call_async=lambda request: "plan")
    client = SimpleNamespace(send_goal_async=lambda goal, feedback_callback: "sent")
    with pytest.raises(movej.ConfirmedStopError) as error:
        movej.run_guarded_sequence(object(), [], (target,), lambda target, index:
            movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                               movej.time.monotonic() + 5))
    failed = error.value.failed_goal_report
    assert error.value.failed_goal_index == 0
    assert failed["index"] == 0
    assert failed["target_rad"] == list(target)
    assert failed["trajectory_sha256"] == (
        "76604aa34f36c7ba7d9fcbf6ea2621f712b44e5dd1934975c3c07547ae19c022")
    assert failed["status"] == "FAILED"
    assert "goal_id" not in failed


def digest_vector():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=42, nanosec=123), frame_id="base"),
        joint_names=["J1", "J2"],
        points=[SimpleNamespace(positions=[1.0, -2.0], velocities=[0.5, -0.5],
                                accelerations=[], effort=[0.0, 3.0],
                                time_from_start=SimpleNamespace(sec=2, nanosec=500))])


GOLDEN_TRAJECTORY_SHA256 = "ca083d6d4341d3de12155159dc0755593f6569ca76add9015c227b070c3e271c"


def test_trajectory_digest_matches_cross_language_golden_vector():
    assert movej.trajectory_digest(digest_vector()) == GOLDEN_TRAJECTORY_SHA256


@pytest.mark.parametrize("field", [
    "stamp_sec", "stamp_nanosec", "frame_id", "joint_names", "points",
    "positions", "velocities", "accelerations", "effort", "duration_sec",
    "duration_nanosec",
])
def test_trajectory_digest_binds_every_semantic_field(field):
    trajectory = digest_vector()
    point = trajectory.points[0]
    changes = {
        "stamp_sec": lambda: setattr(trajectory.header.stamp, "sec", 43),
        "stamp_nanosec": lambda: setattr(trajectory.header.stamp, "nanosec", 124),
        "frame_id": lambda: setattr(trajectory.header, "frame_id", "tool"),
        "joint_names": lambda: trajectory.joint_names.append("J3"),
        "points": lambda: trajectory.points.append(digest_vector().points[0]),
        "positions": lambda: point.positions.__setitem__(0, 1.25),
        "velocities": lambda: point.velocities.__setitem__(0, 0.75),
        "accelerations": lambda: point.accelerations.append(0.1),
        "effort": lambda: point.effort.__setitem__(0, 0.1),
        "duration_sec": lambda: setattr(point.time_from_start, "sec", 3),
        "duration_nanosec": lambda: setattr(point.time_from_start, "nanosec", 501),
    }
    changes[field]()
    assert movej.trajectory_digest(trajectory) != GOLDEN_TRAJECTORY_SHA256


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_trajectory_digest_rejects_nonfinite_point_values(bad):
    trajectory = digest_vector()
    trajectory.points[0].effort[0] = bad
    with pytest.raises(ValueError, match="finite"):
        movej.trajectory_digest(trajectory)


def test_trajectory_digest_normalizes_negative_zero():
    trajectory = digest_vector()
    trajectory.points[0].effort[0] = -0.0
    assert movej.trajectory_digest(trajectory) == GOLDEN_TRAJECTORY_SHA256


def test_trajectory_digest_rejects_invalid_nanoseconds():
    trajectory = digest_vector()
    trajectory.header.stamp.nanosec = 1_000_000_000
    with pytest.raises(ValueError, match="nanosec"):
        movej.trajectory_digest(trajectory)


def test_plan_rejects_missing_or_reordered_axis():
    correct = [f"Joint{axis}" for axis in range(1, 7)]
    trajectory = SimpleNamespace(joint_names=correct, points=[SimpleNamespace(positions=[0, 0, 1.59, 0, 0, 0])])
    movej.validate_plan(trajectory)
    trajectory.joint_names = list(reversed(correct))
    with pytest.raises(ValueError):
        movej.validate_plan(trajectory)
    trajectory.joint_names = correct
    trajectory.points[0].positions = [0]
    with pytest.raises(ValueError):
        movej.validate_plan(trajectory)
    trajectory.points[0].positions = [0, 0, 1.59, 0, 0, 13]
    with pytest.raises(ValueError):
        movej.validate_plan(trajectory)


def test_plan_terminal_must_match_requested_target():
    trajectory = SimpleNamespace(joint_names=[f"Joint{axis}" for axis in range(1, 7)],
                                 points=[SimpleNamespace(positions=[0, 0, 1.59, 0, 0, 0])])
    with pytest.raises(ValueError):
        movej.validate_plan(trajectory, (0.02, 0, 1.59, 0, 0, 0))


def test_bench_limits_follow_verified_candidate_and_accept_joint3_zero(tmp_path):
    fixture_module = Path(__file__).with_name("test_convert_application_ros_limits.py")
    spec = importlib.util.spec_from_file_location("converter_fixture", fixture_module)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest, limits_json, _ = module.source(tmp_path)
    limits = movej.load_bench_limits(limits_json, manifest)
    assert limits[2] == (-0.6, 0.6)
    target = movej.validate_target([0, 0, 0, 0, 0, 0], limits)
    trajectory = SimpleNamespace(joint_names=list(movej.JOINTS),
                                 points=[SimpleNamespace(positions=target)])
    movej.validate_plan(trajectory, target, limits)
    with pytest.raises(ValueError, match="limits"):
        movej.validate_target([0, 0, 1.59, 0, 0, 0], limits)
    with pytest.raises(ValueError, match="limits"):
        movej.validate_plan(SimpleNamespace(
            joint_names=list(movej.JOINTS),
            points=[SimpleNamespace(positions=[0, 0, 1.59, 0, 0, 0])]), limits=limits)


def test_bench_cli_requires_explicit_verified_limits(tmp_path, monkeypatch):
    monkeypatch.setattr("sys.argv", ["ros_six_axis_movej.py", "--reference-frame",
                                    "BENCH_SESSION", "--target", "0", "0", "0", "0",
                                    "0", "0", "--report", str(tmp_path / "report.json")])
    monkeypatch.setattr(movej, "capture", lambda *args: pytest.fail("ROS capture called"))
    with pytest.raises(SystemExit) as error:
        movej.main()
    assert error.value.code == 2
    assert not (tmp_path / "report.json").exists()


def test_bench_cli_passes_verified_bounds_to_capture(tmp_path, monkeypatch):
    fixture_module = Path(__file__).with_name("test_convert_application_ros_limits.py")
    spec = importlib.util.spec_from_file_location("converter_fixture", fixture_module)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest, limits_json, _ = module.source(tmp_path)
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "BENCH_SESSION",
        "--candidate-manifest", str(manifest), "--ros-limits-json", str(limits_json),
        "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
        "--target", "0", "0", "0", "0", "0", "0",
        "--report", str(tmp_path / "report.json"),
    ])
    seen = []
    def fake_capture(target, timeout, limits, **kwargs):
        seen.append((target, limits, kwargs))
        return {"status": "SUCCEEDED"}
    monkeypatch.setattr(movej, "capture", fake_capture)
    assert movej.main() == 0
    assert seen == [((0.0,) * 6, ((-0.6, 0.6),) * 6,
                     {"guarded_session": True, "activate_controller": False,
                      "activation_sha256": "a" * 64,
                      "device_id": "b" * 32, "candidate_manifest": manifest})]


def test_multi_cli_uses_bounded_targets_and_session_capture(tmp_path, monkeypatch):
    fixture_module = Path(__file__).with_name("test_convert_application_ros_limits.py")
    spec = importlib.util.spec_from_file_location("converter_fixture", fixture_module)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest, limits_json, _ = module.source(tmp_path)
    targets = tmp_path / "targets.json"
    targets.write_text(json.dumps([[0] * 6, [0.01] * 6]))
    report_path = tmp_path / "report.json"
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "BENCH_SESSION",
        "--candidate-manifest", str(manifest), "--ros-limits-json", str(limits_json),
        "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
        "--targets-json", str(targets), "--report", str(report_path),
    ])
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs: pytest.fail("single capture called"))
    seen = []
    monkeypatch.setattr(movej, "capture_many", lambda *args, **kwargs: (
        seen.append((args, kwargs)) or {"scope": "ROS_ACTION_SESSION_ONLY", "status": "SUCCEEDED",
                                         "completed_goals": 2, "goals": []}))
    assert movej.main() == 0
    assert seen == [((((0.0,) * 6, (0.01,) * 6), 45.0, ((-0.6, 0.6),) * 6),
                     {"activate_controller": False, "activation_sha256": "a" * 64,
                      "device_id": "b" * 32, "candidate_manifest": manifest})]


def test_multi_cli_rejects_unguarded_or_conflicting_mode_before_ros(tmp_path, monkeypatch):
    targets = tmp_path / "targets.json"
    targets.write_text(json.dumps([[0, 0, 1.59, 0, 0, 0]]))
    monkeypatch.setattr(movej, "capture_many", lambda *args, **kwargs: pytest.fail("ROS called"))
    for options in (
            ["--reference-frame", "ABSOLUTE_MODEL", "--targets-json", str(targets)],
            ["--reference-frame", "BENCH_SESSION", "--target", "0", "0", "1.59",
             "0", "0", "0", "--targets-json", str(targets)]):
        monkeypatch.setattr("sys.argv", ["ros_six_axis_movej.py", *options,
                                        "--report", str(tmp_path / "report.json")])
        with pytest.raises(SystemExit) as error:
            movej.main()
        assert error.value.code == 2


def test_multi_cli_failure_report_keeps_completed_goals_and_stop_state(tmp_path, monkeypatch):
    fixture_module = Path(__file__).with_name("test_convert_application_ros_limits.py")
    spec = importlib.util.spec_from_file_location("converter_fixture", fixture_module)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest, limits_json, _ = module.source(tmp_path)
    targets = tmp_path / "targets.json"
    targets.write_text(json.dumps([[0] * 6, [0.01] * 6]))
    report_path = tmp_path / "report.json"
    monkeypatch.setattr("sys.argv", ["ros_six_axis_movej.py", "--reference-frame",
        "BENCH_SESSION", "--candidate-manifest", str(manifest),
        "--ros-limits-json", str(limits_json), "--activation-sha256", "a" * 64,
        "--device-id", "b" * 32, "--targets-json", str(targets),
        "--report", str(report_path)])
    first = {"index": 0, "goal_id": "1" * 32, "status": "SUCCEEDED"}
    failed = {"index": 1, "target_rad": [0.01] * 6,
              "trajectory_sha256": "c" * 64, "status": "FAILED"}
    monkeypatch.setattr(movej, "capture_many", lambda *args, **kwargs: (
        _ for _ in ()).throw(movej.ConfirmedStopError(
            "second goal rejected", completed_goals=[first], failed_goal_index=1,
            failed_goal_report=failed)))
    assert movej.main() == 1
    report = json.loads(report_path.read_text())
    assert report["scope"] == "ROS_ACTION_SESSION_ONLY"
    assert report["status"] == "FAIL"
    assert report["stop_confirmation"] == "CONFIRMED_RELEASED_IDLE"
    assert report["goals"] == [first, failed]
    assert report["failed_goal_index"] == 1


def test_bench_cli_requires_identity_pins_without_activation(tmp_path, monkeypatch):
    fixture_module = Path(__file__).with_name("test_convert_application_ros_limits.py")
    spec = importlib.util.spec_from_file_location("converter_fixture", fixture_module)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest, limits_json, _ = module.source(tmp_path)
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "BENCH_SESSION",
        "--candidate-manifest", str(manifest), "--ros-limits-json", str(limits_json),
        "--target", "0", "0", "0", "0", "0", "0",
        "--report", str(tmp_path / "report.json"),
    ])
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs: pytest.fail("capture called"))
    with pytest.raises(SystemExit) as error:
        movej.main()
    assert error.value.code == 2


def test_activation_cli_requires_bench_session(tmp_path, monkeypatch):
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "ABSOLUTE_MODEL",
        "--activate-controller", "--target", "0", "0", "1.59", "0", "0", "0",
        "--report", str(tmp_path / "report.json"),
    ])
    monkeypatch.setattr(movej, "capture", lambda *args: pytest.fail("ROS capture called"))
    with pytest.raises(SystemExit) as error:
        movej.main()
    assert error.value.code == 2


def test_bench_cli_forwards_explicit_activation(tmp_path, monkeypatch):
    fixture_module = Path(__file__).with_name("test_convert_application_ros_limits.py")
    spec = importlib.util.spec_from_file_location("converter_fixture", fixture_module)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest, limits_json, _ = module.source(tmp_path)
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "BENCH_SESSION",
        "--candidate-manifest", str(manifest), "--ros-limits-json", str(limits_json),
        "--activate-controller", "--activation-sha256", "a" * 64,
        "--device-id", "b" * 32, "--target", "0", "0", "0", "0", "0", "0",
        "--report", str(tmp_path / "report.json"),
    ])
    calls = []
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs: (
        calls.append((args, kwargs)) or {"status": "SUCCEEDED"}))
    assert movej.main() == 0
    assert calls[0][1] == {"guarded_session": True, "activate_controller": True,
                           "activation_sha256": "a" * 64,
                           "device_id": "b" * 32, "candidate_manifest": manifest}


def test_ros_adapter_uses_strict_single_controller_switch():
    class Request:
        STRICT = 2

        def __init__(self):
            self.activate_controllers = []
            self.strictness = 0
            self.timeout = SimpleNamespace(sec=0)

    adapter = object.__new__(movej.RosControlAdapter)
    adapter.SwitchController = SimpleNamespace(Request=Request)
    adapter.deadline = movej.time.monotonic() + 45.0
    calls = []
    adapter.call = lambda name, service_type, request, deadline=None, transition=False: (
        calls.append((name, request)) or SimpleNamespace(ok=True))
    adapter.activate_controller("arm_controller")
    assert calls[0][0] == "/controller_manager/switch_controller"
    assert calls[0][1].activate_controllers == ["arm_controller"]
    assert calls[0][1].strictness == 2
    assert calls[0][1].timeout.sec == 5


def test_ros_adapter_rejects_failed_hardware_service():
    class Request:
        def __init__(self, name):
            self.name = name
            self.target_state = SimpleNamespace(id=0, label="")

    adapter = object.__new__(movej.RosControlAdapter)
    adapter.SetHardwareComponentState = SimpleNamespace(Request=Request)
    adapter.State = SimpleNamespace(PRIMARY_STATE_ACTIVE=3)
    calls = []
    adapter.call = lambda name, service_type, request, deadline=None, transition=False: (
        calls.append((name, request)) or SimpleNamespace(ok=False))
    with pytest.raises(RuntimeError, match="hardware active transition rejected"):
        adapter.activate_hardware("dummy_ref_real_6dof")
    assert calls[0][1].target_state.id == 3
    assert calls[0][1].target_state.label == "active"


@pytest.mark.parametrize("success,message,expected", [
    (True, "ENABLED_HOLD", True),
    (True, "OPERATION_COMPLETE", False),
    (False, "ENABLED_HOLD", False),
])
def test_ros_adapter_requires_exact_confirmed_hold(success, message, expected):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    calls = []
    adapter.call = lambda name, service_type, request: (
        calls.append(name) or SimpleNamespace(success=success, message=message))
    assert adapter.confirmed_hold() is expected
    assert calls == ["/device_action_gate/bootstrap_status"]


def test_multi_goal_permit_waits_for_lease_then_exact_enabled_hold(monkeypatch):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.deadline = movej.time.monotonic() + 5
    events = []
    statuses = iter(("LEASE_RENEWAL_PENDING", "LEASE_RENEWED",
                     "BOOTSTRAP_PENDING", "ENABLED_HOLD"))

    def call(name, service_type, request):
        events.append(name)
        if name.endswith("set_bench_permit"):
            return SimpleNamespace(success=events.count(name) == 2,
                                   message="Operation queued")
        message = next(statuses)
        return SimpleNamespace(success=message in ("LEASE_RENEWED", "ENABLED_HOLD"),
                               message=message)

    adapter.call = call
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: None)
    adapter.renew_motion_permit()
    assert events.count("/device_action_gate/set_bench_permit") == 2
    assert events.count("/device_action_gate/bootstrap_status") == 4


def test_multi_goal_permit_rejects_fault_without_claiming_hold(monkeypatch):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.deadline = movej.time.monotonic() + 5
    adapter.call = lambda name, service_type, request: (
        SimpleNamespace(success=True, message="Operation queued")
        if name.endswith("set_bench_permit") else
        SimpleNamespace(success=False, message="LEASE_RENEWAL_UNCONFIRMED"))
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: None)
    with pytest.raises(RuntimeError, match="LEASE_RENEWAL_UNCONFIRMED"):
        adapter.renew_motion_permit()


def test_multi_goal_permit_retries_post_complete_window_before_new_goal(monkeypatch):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.deadline = movej.time.monotonic() + 5
    replies = iter((
        ("set_bench_permit", False, "Operation unavailable or another management request is pending"),
        ("bootstrap_status", False, "BOOTSTRAP_PENDING"),
        ("set_bench_permit", False, "Operation unavailable or another management request is pending"),
        ("bootstrap_status", True, "OPERATION_COMPLETE"),
        ("set_bench_permit", False, "Operation unavailable or another management request is pending"),
        ("bootstrap_status", True, "ENABLED_HOLD"),
        ("set_bench_permit", True, "Operation queued; poll bootstrap_status for final result"),
        ("bootstrap_status", False, "BOOTSTRAP_PENDING"),
        ("bootstrap_status", True, "ENABLED_HOLD")))
    calls = []

    def call(name, service_type, request):
        expected, success, message = next(replies)
        assert name.endswith(expected)
        calls.append(expected)
        return SimpleNamespace(success=success, message=message)

    adapter.call = call
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: None)
    adapter.renew_motion_permit()
    assert calls.count("set_bench_permit") == 4
    assert calls[-2:] == ["bootstrap_status", "bootstrap_status"]


@pytest.mark.parametrize("status", ["FAULT", "AUTHORITY_CHANGED",
                                    "EVIDENCE_INCOMPLETE;RELEASE_UNCONFIRMED"])
def test_multi_goal_permit_rejects_nontransient_status_without_retry(status, monkeypatch):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.deadline = movej.time.monotonic() + 5
    calls = []

    def call(name, service_type, request):
        calls.append(name)
        return (SimpleNamespace(success=False, message="Operation unavailable or another management request is pending")
                if name.endswith("set_bench_permit") else
                SimpleNamespace(success=False, message=status))

    adapter.call = call
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: None)
    with pytest.raises(RuntimeError, match="new motion permit unavailable"):
        adapter.renew_motion_permit()
    assert len(calls) == 2


def test_multi_goal_permit_never_reuses_hold_after_unavailable_until_deadline(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(movej.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.deadline = 0.11
    calls = []

    def call(name, service_type, request):
        calls.append(name)
        return (SimpleNamespace(success=False, message="Operation unavailable or another management request is pending")
                if name.endswith("set_bench_permit") else
                SimpleNamespace(success=True, message="ENABLED_HOLD"))

    adapter.call = call
    with pytest.raises(RuntimeError, match="new motion permit deadline") as error:
        adapter.renew_motion_permit()
    assert "set_permit=UNAVAILABLE" in str(error.value)
    assert "bootstrap=ENABLED_HOLD" in str(error.value)
    assert calls.count("/device_action_gate/set_bench_permit") >= 2
    assert clock[0] <= adapter.deadline


def test_multi_goal_permit_error_never_echoes_untrusted_service_text():
    secret = "credential=" + "x" * 500
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.deadline = movej.time.monotonic() + 5
    adapter.call = lambda name, service_type, request: (
        SimpleNamespace(success=False, message=secret)
        if name.endswith("set_bench_permit") else
        SimpleNamespace(success=False, message=secret))
    with pytest.raises(RuntimeError) as error:
        adapter.renew_motion_permit()
    assert "set_permit=UNAVAILABLE" in str(error.value)
    assert "bootstrap=OTHER" in str(error.value)
    assert secret not in str(error.value)
    assert len(str(error.value)) < 180


def test_live_gate_binding_rejects_identity_mismatch_before_activation(tmp_path):
    adapter = FakeLifecycleAdapter()
    adapter.gate_parameters = lambda: {
        "activation_sha256": "a" * 64, "device_id": "b" * 32,
        "contract_bundle": str(tmp_path / "bundle"),
        "profile_name": "f405_6dof_bench", "guarded_controller": "arm_controller"}
    with pytest.raises(RuntimeError, match="identity"):
        movej.verify_live_gate(adapter, "c" * 64, "b" * 32,
                               tmp_path / "bundle" / "manifest.json")
    assert adapter.events == []


def test_live_gate_binding_accepts_exact_pinned_identity(tmp_path):
    adapter = FakeLifecycleAdapter()
    adapter.gate_parameters = lambda: {
        "activation_sha256": "a" * 64, "device_id": "b" * 32,
        "contract_bundle": str(tmp_path / "bundle"),
        "profile_name": "f405_6dof_bench", "guarded_controller": "arm_controller"}
    movej.verify_live_gate(adapter, "a" * 64, "b" * 32,
                           tmp_path / "bundle" / "manifest.json")


def test_remaining_time_uses_single_deadline(monkeypatch):
    monkeypatch.setattr(movej.time, "monotonic", lambda: 10.0)
    assert movej.remaining_time(12.0) == 2.0
    with pytest.raises(RuntimeError, match="deadline"):
        movej.remaining_time(9.0)


def test_failure_cleanup_stops_then_deactivates_owned_components_in_reverse():
    adapter = FakeLifecycleAdapter()
    states = {"hardware": "active", "arm_controller": "active",
              "joint_state_broadcaster": "active"}
    adapter.confirmed_stop = lambda: adapter.events.append("stop") or True
    adapter.controller_state = lambda name, deadline=None: adapter.events.append(
        ("controller_state", name)) or states[name]
    adapter.hardware_state = lambda name, deadline=None: adapter.events.append(
        ("hardware_state", name)) or states["hardware"]
    def deactivate_controller(name):
        adapter.events.append(("deactivate", name))
        states[name] = "inactive"
    def deactivate_hardware(name):
        adapter.events.append(("deactivate_hardware", name))
        states["hardware"] = "inactive"
    adapter.deactivate_controller = deactivate_controller
    adapter.deactivate_hardware = deactivate_hardware
    movej.recover_after_failure(adapter, ["hardware", "joint_state_broadcaster", "arm_controller"])
    assert adapter.events[0] == "stop"
    assert [event for event in adapter.events if event[0] in
            ("deactivate", "deactivate_hardware")] == [
                ("deactivate", "arm_controller"),
                ("deactivate", "joint_state_broadcaster"),
                ("deactivate_hardware", "dummy_ref_real_6dof")]


def test_failed_stop_does_not_deactivate_and_reports_unresolved_components():
    adapter = FakeLifecycleAdapter()
    adapter.confirmed_stop = lambda: False
    adapter.deactivate_hardware = lambda name: pytest.fail("deactivated without confirmed release")
    with pytest.raises(movej.UnconfirmedStopError, match="UNCONFIRMED_STOP") as error:
        movej.recover_after_failure(adapter, ["hardware"])
    assert "dummy_ref_real_6dof" in error.value.unresolved_components


@pytest.mark.parametrize("settled,stopped,expected", [
    (False, True, ("settle_pending=PENDING", "confirmed_stop=CONFIRMED")),
    (True, False, ("settle_pending=SETTLED", "confirmed_stop=NOT_CONFIRMED")),
])
def test_cleanup_failure_identifies_each_confirmation_result(settled, stopped, expected):
    adapter = SimpleNamespace(
        settle_pending=lambda deadline: settled,
        confirmed_stop=lambda: stopped,
        hardware_state=lambda name, deadline: "inactive",
        controller_state=lambda name, deadline: "inactive")
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.recover_after_failure(adapter, [])
    assert all(fragment in str(error.value) for fragment in expected)


def test_cleanup_failure_identifies_settle_exception_without_echoing_it():
    secret = "credential=" + "x" * 500
    adapter = SimpleNamespace(
        settle_pending=lambda deadline: (_ for _ in ()).throw(RuntimeError(secret)),
        confirmed_stop=lambda: True,
        hardware_state=lambda name, deadline: "inactive",
        controller_state=lambda name, deadline: "inactive")
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.recover_after_failure(adapter, [])
    assert "settle_pending=ERROR" in str(error.value)
    assert "confirmed_stop=CONFIRMED" in str(error.value)
    assert secret not in str(error.value)


def test_cleanup_failure_identifies_release_service_call_failure():
    secret = "credential=" + "x" * 500
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.settle_pending = lambda deadline: True
    adapter.call = lambda name, service_type, request, deadline: (
        _ for _ in ()).throw(RuntimeError(secret))
    adapter.hardware_state = lambda name, deadline: "inactive"
    adapter.controller_state = lambda name, deadline: "inactive"
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.recover_after_failure(adapter, [])
    assert "confirmed_stop=ERROR" in str(error.value)
    assert "release_reply=NOT_OBSERVED" in str(error.value)
    assert "bootstrap_reply=NOT_OBSERVED" in str(error.value)
    assert "stop_phase=RELEASE_CALL_FAILED" in str(error.value)
    assert secret not in str(error.value)


def test_cleanup_failure_categorizes_release_reply_without_echoing_it():
    secret = "credential=" + "x" * 500
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.settle_pending = lambda deadline: True
    adapter.call = lambda name, service_type, request, deadline: SimpleNamespace(
        success=False, message=secret)
    adapter.hardware_state = lambda name, deadline: "inactive"
    adapter.controller_state = lambda name, deadline: "inactive"
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.recover_after_failure(adapter, [])
    assert "confirmed_stop=NOT_CONFIRMED" in str(error.value)
    assert "release_reply=OTHER" in str(error.value)
    assert "bootstrap_reply=NOT_OBSERVED" in str(error.value)
    assert secret not in str(error.value)
    assert len(str(error.value)) < 300


def test_cleanup_failure_keeps_last_bootstrap_reply_category(monkeypatch):
    secret = "credential=" + "x" * 500
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.settle_pending = lambda deadline: True
    adapter.call = lambda name, service_type, request, deadline: (
        SimpleNamespace(success=False, message="RELEASE_UNAVAILABLE")
        if name.endswith("disable_and_release") else
        SimpleNamespace(success=False, message="LEASE_RENEWAL_PENDING"))
    adapter.hardware_state = lambda name, deadline: "inactive"
    adapter.controller_state = lambda name, deadline: "inactive"
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: (
        _ for _ in ()).throw(RuntimeError(secret)))
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.recover_after_failure(adapter, [])
    assert "confirmed_stop=ERROR" in str(error.value)
    assert "release_reply=RELEASE_UNAVAILABLE" in str(error.value)
    assert "bootstrap_reply=LEASE_RENEWAL_PENDING" in str(error.value)
    assert secret not in str(error.value)
    assert len(str(error.value)) < 300


def test_release_unavailable_retries_after_management_complete_until_released(monkeypatch):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.stop_deadline = movej.time.monotonic() + 5
    replies = iter((
        ("disable_and_release", False, "RELEASE_UNAVAILABLE"),
        ("bootstrap_status", True, "OPERATION_COMPLETE"),
        ("disable_and_release", False, "RELEASE_QUEUED"),
        ("bootstrap_status", False, "RELEASE_PENDING"),
        ("bootstrap_status", True, "RELEASED_IDLE")))
    calls = []

    def call(name, service_type, request, deadline):
        expected, success, message = next(replies)
        assert name.endswith(expected)
        assert deadline == adapter.stop_deadline
        calls.append(expected)
        return SimpleNamespace(success=success, message=message)

    adapter.call = call
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: None)
    assert adapter.confirmed_stop() is True
    assert calls == ["disable_and_release", "bootstrap_status", "disable_and_release",
                     "bootstrap_status", "bootstrap_status"]
    assert adapter.stop_release_reply == "RELEASE_QUEUED"
    assert adapter.stop_bootstrap_reply == "RELEASED_IDLE"


def test_cancel_release_unavailable_then_global_idle_is_not_stop_proof(monkeypatch):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.stop_deadline = movej.time.monotonic() + 5
    replies = iter((
        ("disable_and_release", False, "RELEASE_UNAVAILABLE"),
        ("bootstrap_status", False, "RELEASE_PENDING"),
        ("bootstrap_status", True, "RELEASED_IDLE")))
    calls = []

    def call(name, service_type, request, deadline):
        expected, success, message = next(replies)
        assert name.endswith(expected)
        calls.append(expected)
        return SimpleNamespace(success=success, message=message)

    adapter.call = call
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: None)
    assert adapter.confirmed_stop() is True
    assert calls.count("disable_and_release") == 1
    monkeypatch.setattr(movej, "complete_success", lambda adapter, owned, report:
                        {**report, "stop_confirmation": "CONFIRMED_RELEASED_IDLE"})
    report = movej.complete_cancel_observation(adapter, [], {
        "status": "CANCELED", "goal_id": "12" * 16, "action_status": 5})
    assert report["status"] == "UNCONFIRMED_STOP"
    assert report["action_observation"] == "CANCELED"
    assert report["stop_confirmation"] == "UNCONFIRMED_STOP"
    assert report["release_observation"] == {
        "request_reply": "RELEASE_UNAVAILABLE", "bootstrap_reply": "RELEASED_IDLE"}
    assert report["safety_verdict"] == "NOT_QUALIFIED"


@pytest.mark.parametrize("release_reply", ["RELEASE_QUEUED", "RELEASED_IDLE"])
def test_release_observation_gap_recovers_without_second_release(monkeypatch, release_reply):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.stop_deadline = movej.time.monotonic() + 5
    replies = iter((
        ("disable_and_release", False, release_reply),
        ("bootstrap_status", False, "RELEASE_OBSERVATION_UNAVAILABLE"),
        ("bootstrap_status", True, "RELEASED_IDLE")))
    calls = []

    def call(name, service_type, request, deadline):
        expected, success, message = next(replies)
        assert name.endswith(expected)
        assert deadline == adapter.stop_deadline
        calls.append(expected)
        return SimpleNamespace(success=success, message=message)

    adapter.call = call
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: None)
    assert adapter.confirmed_stop() is True
    assert calls == ["disable_and_release", "bootstrap_status", "bootstrap_status"]
    assert adapter.stop_release_reply == release_reply
    assert adapter.stop_bootstrap_reply == "RELEASED_IDLE"


def test_release_observation_gap_until_deadline_stays_unconfirmed(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(movej.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.stop_deadline = 0.11
    calls = []

    def call(name, service_type, request, deadline):
        assert deadline == adapter.stop_deadline
        calls.append(name)
        return (SimpleNamespace(success=False, message="RELEASE_QUEUED")
                if name.endswith("disable_and_release") else
                SimpleNamespace(success=False, message="RELEASE_OBSERVATION_UNAVAILABLE"))

    adapter.call = call
    with pytest.raises(RuntimeError, match="deadline"):
        adapter.confirmed_stop()
    assert sum(name.endswith("disable_and_release") for name in calls) == 1
    assert sum(name.endswith("bootstrap_status") for name in calls) >= 2
    assert adapter.stop_bootstrap_reply == "RELEASE_OBSERVATION_UNAVAILABLE"
    assert clock[0] <= adapter.stop_deadline


@pytest.mark.parametrize("status", ["EVIDENCE_INCOMPLETE;RELEASE_UNCONFIRMED",
                                    "RELEASE_UNCONFIRMED", "FAULT", "UNKNOWN"])
def test_release_queued_rejects_terminal_or_unknown_bootstrap_reply(status):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.stop_deadline = movej.time.monotonic() + 5
    calls = []

    def call(name, service_type, request, deadline):
        calls.append(name)
        return (SimpleNamespace(success=False, message="RELEASE_QUEUED")
                if name.endswith("disable_and_release") else
                SimpleNamespace(success=False, message=status))

    adapter.call = call
    assert adapter.confirmed_stop() is False
    assert len(calls) == 2
    assert adapter.stop_bootstrap_reply == movej.stop_reply_category(status)


@pytest.mark.parametrize("status", ["FAULT", "AUTHORITY_CHANGED",
                                    "EVIDENCE_INCOMPLETE;RELEASE_UNCONFIRMED"])
def test_release_unavailable_rejects_fault_or_lost_identity(status):
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.settle_pending = lambda deadline: True
    adapter.hardware_state = lambda name, deadline: "inactive"
    adapter.controller_state = lambda name, deadline: "inactive"
    calls = []

    def call(name, service_type, request, deadline):
        calls.append(name)
        return (SimpleNamespace(success=False, message="RELEASE_UNAVAILABLE")
                if name.endswith("disable_and_release") else
                SimpleNamespace(success=False, message=status))

    adapter.call = call
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.recover_after_failure(adapter, [])
    assert len(calls) == 2
    assert "confirmed_stop=NOT_CONFIRMED" in str(error.value)
    assert "release_reply=RELEASE_UNAVAILABLE" in str(error.value)


def test_permanently_busy_release_stays_within_original_stop_deadline(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(movej.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(movej.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    adapter.stop_deadline = 0.11
    calls = []

    def call(name, service_type, request, deadline):
        assert deadline == 0.11
        calls.append(name)
        return (SimpleNamespace(success=False, message="RELEASE_UNAVAILABLE")
                if name.endswith("disable_and_release") else
                SimpleNamespace(success=True, message="ENABLED_HOLD"))

    adapter.call = call
    with pytest.raises(RuntimeError, match="deadline"):
        adapter.confirmed_stop()
    assert len(calls) <= 8
    assert clock[0] <= adapter.stop_deadline


def test_unsettled_activation_prevents_deactivation_even_after_release():
    adapter = FakeLifecycleAdapter()
    adapter.settle_pending = lambda deadline: False
    adapter.confirmed_stop = lambda: True
    adapter.deactivate_hardware = lambda name: pytest.fail("deactivated with pending activation")
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.recover_after_failure(adapter, ["hardware"])
    assert "dummy_ref_real_6dof" in error.value.unresolved_components


def test_preexisting_active_component_is_reported_without_deactivation():
    adapter = FakeLifecycleAdapter()
    adapter.confirmed_stop = lambda: True
    adapter.controller_state = lambda name, deadline=None: (
        "active" if name == "arm_controller" else "inactive")
    adapter.deactivate_controller = lambda name: pytest.fail("deactivated preexisting controller")
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.recover_after_failure(adapter, [])
    assert error.value.still_active_components == ["arm_controller"]
    assert error.value.unresolved_components == []


def test_successful_one_shot_releases_before_reporting_success():
    adapter = FakeLifecycleAdapter()
    adapter.activate_hardware("dummy_ref_real_6dof")
    adapter.activate_controller("joint_state_broadcaster")
    adapter.activate_controller("arm_controller")
    initial_events = len(adapter.events)
    adapter.confirmed_stop = lambda: adapter.events.append("release") or True
    def deactivate_controller(name):
        adapter.events.append(("deactivate", name))
        adapter.controllers[name] = "inactive"
    def deactivate_hardware(name):
        adapter.events.append(("deactivate_hardware", name))
        adapter.hardware = "inactive"
    adapter.deactivate_controller = deactivate_controller
    adapter.deactivate_hardware = deactivate_hardware
    report = movej.complete_success(adapter, ["hardware", "joint_state_broadcaster",
                                              "arm_controller"], {"status": "SUCCEEDED"})
    assert report["stop_confirmation"] == "CONFIRMED_RELEASED_IDLE"
    assert adapter.events[initial_events] == "release"
    assert [event for event in adapter.events[initial_events:] if isinstance(event, tuple) and
            event[0] in ("deactivate", "deactivate_hardware")] == [
                ("deactivate", "arm_controller"),
                ("deactivate", "joint_state_broadcaster"),
                ("deactivate_hardware", "dummy_ref_real_6dof")]


def test_one_shot_cannot_report_success_without_confirmed_release():
    adapter = FakeLifecycleAdapter()
    adapter.confirmed_stop = lambda: False
    with pytest.raises(movej.UnconfirmedStopError):
        movej.complete_success(adapter, [], {"status": "SUCCEEDED"})


@pytest.mark.parametrize("activate_controller", [True, False])
@pytest.mark.parametrize("accepted", [True, False])
def test_successful_capture_releases_before_return(monkeypatch, activate_controller, accepted):
    target = (0.0, 0.0, 1.59, 0.0, 0.0, 0.0)
    trajectory = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""),
        joint_names=list(movej.JOINTS), points=[
        SimpleNamespace(positions=list(target), velocities=[0.1] * 6,
                        accelerations=[0.2] * 6, effort=[],
                        time_from_start=SimpleNamespace(sec=1, nanosec=250000000)),
        SimpleNamespace(positions=list(target), velocities=[0.0] * 6,
                        accelerations=[-0.1] * 6, effort=[],
                        time_from_start=SimpleNamespace(sec=2, nanosec=750000000)),
    ])
    plan_response = SimpleNamespace(motion_plan_response=SimpleNamespace(
        error_code=SimpleNamespace(val=1), trajectory=SimpleNamespace(joint_trajectory=trajectory)))
    handle = SimpleNamespace(accepted=accepted, goal_id=SimpleNamespace(uuid=bytes(16)))
    planner = SimpleNamespace(wait_for_service=lambda timeout_sec: True,
                              call_async=lambda request: "plan")
    node = SimpleNamespace(create_client=lambda kind, name: planner,
                           destroy_node=lambda: None)
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.create_node = lambda name: node
    rclpy.shutdown = lambda: None

    class MotionPlanRequest:
        def __init__(self):
            self.start_state = SimpleNamespace(is_diff=False)

    class ActionClient:
        def __init__(self, node, kind, name):
            pass

        def wait_for_server(self, timeout_sec):
            return True

        def send_goal_async(self, goal, feedback_callback):
            feedback_callback(SimpleNamespace(feedback=SimpleNamespace(
                joint_names=list(movej.JOINTS),
                actual=SimpleNamespace(positions=target),
                desired=SimpleNamespace(positions=target))))
            return "goal"

    packages = {
        "rclpy": rclpy,
        "rclpy.serialization": SimpleNamespace(serialize_message=lambda value: b"planned-trajectory"),
        "rclpy.action": SimpleNamespace(ActionClient=ActionClient),
        "action_msgs.msg": SimpleNamespace(GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=4)),
        "control_msgs.action": SimpleNamespace(FollowJointTrajectory=SimpleNamespace(
            Goal=lambda: SimpleNamespace(trajectory=None))),
        "moveit_msgs.msg": SimpleNamespace(
            Constraints=lambda **kwargs: SimpleNamespace(**kwargs),
            JointConstraint=lambda **kwargs: SimpleNamespace(**kwargs),
            MotionPlanRequest=MotionPlanRequest),
        "moveit_msgs.srv": SimpleNamespace(GetMotionPlan=SimpleNamespace(Request=lambda: SimpleNamespace())),
    }
    for name, package in packages.items():
        monkeypatch.setitem(sys.modules, name, package)
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: (
        plan_response if future == "plan" else handle))
    monkeypatch.setattr(movej, "await_accepted_goal", lambda node, handle, timeout: (
        SimpleNamespace(status=4, result=SimpleNamespace(error_code=0, error_string=""))))
    adapter = FakeLifecycleAdapter()
    if not activate_controller:
        adapter.activate_hardware("dummy_ref_real_6dof")
        adapter.activate_controller("joint_state_broadcaster")
        adapter.activate_controller("arm_controller")
    adapter.confirmed_stop = lambda: adapter.events.append("release") or True
    def deactivate_controller(name):
        adapter.events.append(("deactivate", name))
        adapter.controllers[name] = "inactive"
    def deactivate_hardware(name):
        adapter.events.append(("deactivate_hardware", name))
        adapter.hardware = "inactive"
    adapter.deactivate_controller = deactivate_controller
    adapter.deactivate_hardware = deactivate_hardware
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: adapter.events.append("identity"))
    try:
        report = movej.capture(target, 5.0, movej.LIMITS,
                               guarded_session=True, activate_controller=activate_controller,
                               activation_sha256="a" * 64, device_id="b" * 32,
                               candidate_manifest=Path("/tmp/bundle/manifest.json"))
    except (movej.ConfirmedStopError, movej.UnconfirmedStopError) as error:
        assert not accepted or not activate_controller
        if not activate_controller:
            assert isinstance(error, movej.UnconfirmedStopError)
            assert error.still_active_components == list(map(movej.component_name,
                                                             movej.ALL_COMPONENTS))
            assert not any(event[0] in ("deactivate", "deactivate_hardware") for event in
                           adapter.events if isinstance(event, tuple))
        else:
            assert isinstance(error, movej.ConfirmedStopError)
        if not accepted:
            diagnostic = movej.rejection_diagnostic_from_error(error)
            assert diagnostic["trajectory_sha256"] == movej.trajectory_digest(trajectory)
            assert diagnostic["joint_names"] == list(movej.JOINTS)
            assert diagnostic["points"] == [
                {"positions": list(target), "velocities": [0.1] * 6,
                 "accelerations": [0.2] * 6,
                 "time_from_start": {"sec": 1, "nanosec": 250000000}},
                {"positions": list(target), "velocities": [0.0] * 6,
                 "accelerations": [-0.1] * 6,
                 "time_from_start": {"sec": 2, "nanosec": 750000000}},
            ]
            assert diagnostic["time_deltas_ns"] == [1250000000, 1500000000]
            assert diagnostic["axis_maxima"][0]["abs_velocity_rad_s"] == 0.1
        assert "release" in adapter.events
        return
    assert accepted and activate_controller
    assert report["status"] == "SUCCEEDED"
    assert report["trajectory_sha256"] == movej.trajectory_digest(trajectory)
    assert report["action_started_steady_us"] > 0
    assert report["stop_confirmation"] == "CONFIRMED_RELEASED_IDLE"
    assert "identity" in adapter.events
    assert "release" in adapter.events
    assert adapter.events.index("release") < adapter.events.index(
        ("deactivate", "arm_controller"))
    assert report["still_active_components"] == []


def test_timed_out_transition_retains_pending_future_and_client(monkeypatch):
    future = FakeFuture(done=False)
    events = []
    client = SimpleNamespace(wait_for_service=lambda timeout_sec: True,
                             call_async=lambda request: future)
    node = SimpleNamespace(create_client=lambda kind, name: client,
                           destroy_client=lambda value: events.append("destroyed"))
    adapter = object.__new__(movej.RosControlAdapter)
    adapter.node = node
    adapter.deadline = movej.time.monotonic() + 5
    adapter.pending_transitions = []
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: (
        _ for _ in ()).throw(RuntimeError("timed out")))
    with pytest.raises(RuntimeError, match="timed out"):
        adapter.call("/controller_manager/switch_controller", object, object(), transition=True)
    assert len(adapter.pending_transitions) == 1
    assert events == []


def test_cancel_and_terminal_result_share_one_cleanup_deadline(monkeypatch):
    result_future = FakeFuture(SimpleNamespace(status=5), done=False)
    cancel_future = FakeFuture(SimpleNamespace(goals_canceling=[object()]))
    handle = FakeHandle(result_future, cancel_future)
    times = iter((10.0, 12.0, 14.0))
    monkeypatch.setattr(movej.time, "monotonic", lambda: next(times))
    timeouts = []

    def fake_wait(node, future, timeout):
        timeouts.append(timeout)
        if future is result_future and len(timeouts) == 1:
            raise RuntimeError("timed out")
        return future.result()

    monkeypatch.setattr(movej, "wait", fake_wait)
    with pytest.raises(RuntimeError, match="terminal action status"):
        movej.await_accepted_goal(object(), handle, 45.0)
    assert timeouts == [45.0, 3.0, 1.0]


def test_planning_failure_requests_stop_and_rolls_back_before_return(monkeypatch):
    events = []
    node = SimpleNamespace(
        create_client=lambda kind, name: SimpleNamespace(
            wait_for_service=lambda timeout_sec: events.append("planner_wait") or False),
        destroy_node=lambda: events.append("node_destroyed"))
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: events.append("ros_initialized")
    rclpy.create_node = lambda name: node
    rclpy.shutdown = lambda: events.append("ros_shutdown")
    packages = {
        "rclpy": rclpy,
        "rclpy.action": types.SimpleNamespace(ActionClient=object),
        "action_msgs.msg": types.SimpleNamespace(GoalStatus=object),
        "control_msgs.action": types.SimpleNamespace(FollowJointTrajectory=object),
        "moveit_msgs.msg": types.SimpleNamespace(
            Constraints=object, JointConstraint=object, MotionPlanRequest=object),
        "moveit_msgs.srv": types.SimpleNamespace(GetMotionPlan=object),
    }
    for name, package in packages.items():
        monkeypatch.setitem(sys.modules, name, package)
    adapter = FakeLifecycleAdapter()
    adapter.confirmed_stop = lambda: events.append("stop") or True
    def deactivate_controller(name):
        events.append(("deactivate", name))
        adapter.controllers[name] = "inactive"
    def deactivate_hardware(name):
        events.append(("deactivate_hardware", name))
        adapter.hardware = "inactive"
    adapter.deactivate_controller = deactivate_controller
    adapter.deactivate_hardware = deactivate_hardware
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: None)
    with pytest.raises(movej.ConfirmedStopError, match="planning service unavailable"):
        movej.capture((0.0,) * 6, 5.0, movej.LIMITS, activate_controller=True,
                      activation_sha256="a" * 64, device_id="b" * 32,
                      candidate_manifest=Path("/tmp/bundle/manifest.json"))
    assert events == ["ros_initialized", "planner_wait", "stop",
                      ("deactivate", "arm_controller"),
                      ("deactivate", "joint_state_broadcaster"),
                      ("deactivate_hardware", "dummy_ref_real_6dof"),
                      "node_destroyed", "ros_shutdown"]


def test_failure_report_separates_confirmed_stop_from_capture_failure(tmp_path, monkeypatch):
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "ABSOLUTE_MODEL",
        "--target", "0", "0", "1.59", "0", "0", "0",
        "--report", str(tmp_path / "report.json"),
    ])
    monkeypatch.setattr(movej, "capture", lambda *args: (
        _ for _ in ()).throw(movej.ConfirmedStopError("planning failed")))
    assert movej.main() == 1
    import json
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["status"] == "FAIL"
    assert report["stop_confirmation"] == "CONFIRMED_RELEASED_IDLE"


@pytest.mark.parametrize("confirmed", [True, False])
def test_rejected_plan_is_written_into_failed_report_without_claiming_success(
        tmp_path, monkeypatch, capsys, confirmed):
    import json
    diagnostic = {"trajectory_sha256": "a" * 64, "joint_names": list(movej.JOINTS),
                  "points": [{"positions": [0, 0, 1.59, 0, 0, 0],
                              "velocities": [], "accelerations": [],
                              "time_from_start": {"sec": 1, "nanosec": 0}}]}
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "ABSOLUTE_MODEL",
        "--target", "0", "0", "1.59", "0", "0", "0",
        "--report", str(tmp_path / "report.json"),
    ])

    def rejected_capture(*args):
        error = (movej.ConfirmedStopError("guarded action rejected MoveIt plan")
                 if confirmed else movej.UnconfirmedStopError(
                     "UNCONFIRMED_STOP: guarded action rejected MoveIt plan"))
        raise error from movej.RejectedPlanError(diagnostic)

    monkeypatch.setattr(movej, "capture", rejected_capture)
    assert movej.main() == 1
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["status"] == ("FAIL" if confirmed else "UNCONFIRMED_STOP")
    assert report["stop_confirmation"] == (
        "CONFIRMED_RELEASED_IDLE" if confirmed else "UNCONFIRMED_STOP")
    assert report["rejected_plan"] == diagnostic
    assert "action_status" not in report
    printed = json.loads(capsys.readouterr().out)
    assert printed["rejected_plan"] == {
        "report_path": str(tmp_path / "report.json"),
        "trajectory_sha256": "a" * 64, "point_count": 1}
    assert "points" not in printed["rejected_plan"]


def test_rejected_plan_diagnostic_keeps_missing_derivatives_unknown():
    trajectory = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""),
        joint_names=list(movej.JOINTS), points=[
        SimpleNamespace(positions=[0, 0, 1.59, 0, 0, 0], velocities=[],
                        accelerations=[], effort=[],
                        time_from_start=SimpleNamespace(sec=0, nanosec=0))])
    diagnostic = movej.rejection_diagnostic(trajectory, b"serialized")
    assert diagnostic["time_deltas_ns"] == [0]
    assert diagnostic["axis_maxima"][2] == {
        "joint_name": "Joint3", "abs_position_rad": 1.59,
        "abs_velocity_rad_s": None, "abs_acceleration_rad_s2": None}


def test_rejection_diagnostic_refuses_oversized_plan():
    point = SimpleNamespace(positions=[0, 0, 1.59, 0, 0, 0], velocities=[],
                            accelerations=[], time_from_start=SimpleNamespace(sec=0, nanosec=0))
    trajectory = SimpleNamespace(joint_names=list(movej.JOINTS),
                                 points=[point] * (movej.MAX_DIAGNOSTIC_POINTS + 1))
    with pytest.raises(RuntimeError, match="point count"):
        movej.rejection_diagnostic(trajectory, b"serialized")
    trajectory.points = [point]
    with pytest.raises(RuntimeError, match="byte count"):
        movej.rejection_diagnostic(trajectory, b"x" * (movej.MAX_SERIALIZED_TRAJECTORY_BYTES + 1))


class FakeLifecycleAdapter:
    def __init__(self, hold="ENABLED_HOLD", hardware="active", controllers=None):
        self.hold = hold
        self.hardware = hardware
        self.controllers = controllers or {
            "joint_state_broadcaster": "active", "arm_controller": "active"}
        self.events = []

    def settle_pending(self, deadline):
        return True

    def confirmed_hold(self):
        self.events.append("hold")
        return self.hold == "ENABLED_HOLD"

    def activate_hardware(self, name):
        self.events.append(("hardware", name))

    def hardware_state(self, name, deadline=None):
        self.events.append(("hardware_state", name))
        return self.hardware if ("hardware", name) in self.events else "inactive"

    def activate_controller(self, name):
        self.events.append(("controller", name))

    def controller_state(self, name, deadline=None):
        self.events.append(("controller_state", name))
        return self.controllers.get(name) if ("controller", name) in self.events else "inactive"


def test_activation_orders_and_checks_each_transition():
    adapter = FakeLifecycleAdapter()
    movej.activate_controller_stack(adapter)
    assert adapter.events == [
        "hold", ("hardware_state", "dummy_ref_real_6dof"),
        ("hardware", "dummy_ref_real_6dof"),
        ("hardware_state", "dummy_ref_real_6dof"),
        ("controller_state", "joint_state_broadcaster"),
        ("controller", "joint_state_broadcaster"),
        ("controller_state", "joint_state_broadcaster"),
        ("controller_state", "arm_controller"),
        ("controller", "arm_controller"), ("controller_state", "arm_controller"),
    ]


@pytest.mark.parametrize("adapter,events", [
    (FakeLifecycleAdapter(hold="BOOTSTRAP_PENDING"), ["hold"]),
    (FakeLifecycleAdapter(hardware="inactive"), [
        "hold", ("hardware_state", "dummy_ref_real_6dof"),
        ("hardware", "dummy_ref_real_6dof"),
        ("hardware_state", "dummy_ref_real_6dof")]),
    (FakeLifecycleAdapter(controllers={"joint_state_broadcaster": "inactive"}), [
        "hold", ("hardware_state", "dummy_ref_real_6dof"),
        ("hardware", "dummy_ref_real_6dof"),
        ("hardware_state", "dummy_ref_real_6dof"),
        ("controller_state", "joint_state_broadcaster"),
        ("controller", "joint_state_broadcaster"),
        ("controller_state", "joint_state_broadcaster")]),
])
def test_activation_fails_closed_before_next_transition(adapter, events):
    with pytest.raises(RuntimeError):
        movej.activate_controller_stack(adapter)
    assert adapter.events == events


class FakeFuture:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def exception(self):
        return None

    def result(self):
        return self._result


class FakeHandle:
    def __init__(self, result_future, cancel_future):
        self.result_future = result_future
        self.cancel_future = cancel_future
        self.cancel_count = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        return self.cancel_future


def goal_info(goal_uuid):
    return SimpleNamespace(goal_id=SimpleNamespace(uuid=goal_uuid))


def test_result_timeout_cancels_and_waits_for_terminal_result(monkeypatch):
    result = SimpleNamespace(status=5)
    result_future = FakeFuture(result, done=False)
    cancel_future = FakeFuture(SimpleNamespace(goals_canceling=[object()]))
    handle = FakeHandle(result_future, cancel_future)
    calls = []

    def fake_wait(node, future, timeout):
        calls.append((future, timeout))
        if future is result_future and len(calls) == 1:
            raise RuntimeError("ROS request timed out or failed")
        return future.result()

    monkeypatch.setattr(movej, "wait", fake_wait)
    with pytest.raises(RuntimeError, match="terminal action status 5"):
        movej.await_accepted_goal(object(), handle, 45.0)
    assert handle.cancel_count == 1
    assert [future for future, _ in calls] == [result_future, cancel_future, result_future]
    assert all(timeout <= 5.0 for _, timeout in calls[1:])


@pytest.mark.parametrize("reply_kind,accepted", [
    ("matching", True), ("foreign", False), ("malformed", False),
])
def test_result_timeout_cancel_detail_requires_matching_goal_info(
        monkeypatch, reply_kind, accepted):
    goal_uuid = bytes.fromhex("12" * 16)
    cancel_goal = (goal_info(goal_uuid) if reply_kind == "matching" else
                   goal_info(bytes.fromhex("34" * 16)) if reply_kind == "foreign" else
                   object())
    result_future = FakeFuture(SimpleNamespace(status=5), done=False)
    cancel_future = FakeFuture(SimpleNamespace(goals_canceling=[cancel_goal]))
    handle = FakeHandle(result_future, cancel_future)
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    calls = 0

    def fake_wait(node, future, timeout):
        nonlocal calls
        calls += 1
        if future is result_future and calls == 1:
            raise RuntimeError("ROS result timed out")
        return future.result()

    monkeypatch.setattr(movej, "wait", fake_wait)
    with pytest.raises(RuntimeError, match="terminal action status 5") as raised:
        movej.await_accepted_goal(object(), handle, 45.0)
    assert ("cancel accepted" in str(raised.value)) is accepted


def test_result_timeout_without_terminal_result_is_unconfirmed(monkeypatch):
    result_future = FakeFuture(done=False)
    cancel_future = FakeFuture(SimpleNamespace(goals_canceling=[object()]))
    handle = FakeHandle(result_future, cancel_future)

    def fake_wait(node, future, timeout):
        if future is result_future:
            raise RuntimeError("ROS request timed out or failed")
        return future.result()

    monkeypatch.setattr(movej, "wait", fake_wait)
    with pytest.raises(movej.UnconfirmedStopError, match="UNCONFIRMED_STOP"):
        movej.await_accepted_goal(object(), handle, 45.0)
    assert handle.cancel_count == 1


def test_result_timeout_without_terminal_status_is_unconfirmed(monkeypatch):
    result_future = FakeFuture(SimpleNamespace(status=3), done=False)
    cancel_future = FakeFuture(SimpleNamespace(goals_canceling=[object()]))
    handle = FakeHandle(result_future, cancel_future)
    attempts = 0

    def fake_wait(node, future, timeout):
        nonlocal attempts
        if future is result_future:
            attempts += 1
            if attempts == 1:
                raise RuntimeError("ROS request timed out or failed")
        return future.result()

    monkeypatch.setattr(movej, "wait", fake_wait)
    with pytest.raises(movej.UnconfirmedStopError, match="UNCONFIRMED_STOP"):
        movej.await_accepted_goal(object(), handle, 45.0)


def test_completed_result_does_not_cancel(monkeypatch):
    result = SimpleNamespace(status=4)
    handle = FakeHandle(FakeFuture(result), FakeFuture())
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    assert movej.await_accepted_goal(object(), handle, 45.0) is result
    assert handle.cancel_count == 0


def test_mid_motion_cancel_requires_matching_ack_and_terminal_result(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    result_future = FakeFuture(SimpleNamespace(status=5, result=SimpleNamespace(
        error_code=0, error_string="canceled")))
    cancel_future = FakeFuture(SimpleNamespace(goals_canceling=[
        goal_info(goal_uuid)]))
    handle = FakeHandle(result_future, cancel_future)
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"goal_id": goal_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": goal_uuid.hex(), "actual_rad": [0.002, 0, 0, 0, 0, 0]}]
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())

    result, cancel = movej.await_mid_motion_cancel(
        object(), handle, feedback, 0.001, movej.time.monotonic() + 5)

    assert handle.cancel_count == 1
    assert result.status == 5
    assert cancel["goal_id"] == goal_uuid.hex()
    assert cancel["acknowledged"] is True


def test_mid_motion_cancel_ack_uses_goal_info_goal_id(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    result_future = FakeFuture(SimpleNamespace(status=5, result=SimpleNamespace(
        error_code=0, error_string="canceled")))
    cancel_future = FakeFuture(SimpleNamespace(goals_canceling=[
        goal_info(goal_uuid)]))
    handle = FakeHandle(result_future, cancel_future)
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"goal_id": goal_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": goal_uuid.hex(), "actual_rad": [0.002, 0, 0, 0, 0, 0]}]
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())

    result, cancel = movej.await_mid_motion_cancel(
        object(), handle, feedback, 0.001, movej.time.monotonic() + 5)

    assert result.status == 5
    assert cancel["acknowledged"] is True


def test_preempt_probe_submits_b_before_canceling_a(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    handle = FakeHandle(FakeFuture(SimpleNamespace(status=5, result=SimpleNamespace(
        error_code=0, error_string="canceled"))),
        FakeFuture(SimpleNamespace(goals_canceling=[goal_info(goal_uuid)])))
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"goal_id": goal_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": goal_uuid.hex(), "actual_rad": [0.002, 0, 0, 0, 0, 0]}]
    events = []
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    original_cancel = handle.cancel_goal_async

    def cancel_a():
        events.append("cancel_a")
        return original_cancel()

    handle.cancel_goal_async = cancel_a
    result, cancel = movej.await_mid_motion_cancel(
        object(), handle, feedback, 0.001, movej.time.monotonic() + 5,
        overlap_probe=lambda: events.append("submit_b") or
                      {"goal_id": "34" * 16, "status": "REJECTED"})
    assert events == ["submit_b", "cancel_a"]
    assert result.status == 5
    assert cancel["overlap_probe"] == {"goal_id": "34" * 16, "status": "REJECTED"}


def test_preempt_probe_error_still_cancels_a(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    handle = FakeHandle(FakeFuture(SimpleNamespace(status=5, result=SimpleNamespace(
        error_code=0, error_string="canceled"))),
        FakeFuture(SimpleNamespace(goals_canceling=[goal_info(goal_uuid)])))
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"goal_id": goal_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": goal_uuid.hex(), "actual_rad": [0.002, 0, 0, 0, 0, 0]}]
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    def fail_overlap():
        raise RuntimeError("B admission unavailable")
    _, cancel = movej.await_mid_motion_cancel(
        object(), handle, feedback, 0.001, movej.time.monotonic() + 5,
        overlap_probe=fail_overlap)
    assert handle.cancel_count == 1
    assert cancel["overlap_probe"]["status"] == "ADMISSION_UNCONFIRMED"


def test_preempt_result_future_creation_error_cancels_a(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    handle = FakeHandle(FakeFuture(), FakeFuture(SimpleNamespace(goals_canceling=[
        goal_info(goal_uuid)])))
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    handle.get_result_async = lambda: (_ for _ in ()).throw(RuntimeError("result future unavailable"))
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    with pytest.raises(movej.CancelAttemptError) as raised:
        movej.await_mid_motion_cancel(object(), handle, [], 0.001,
                                      movej.time.monotonic() + 5)
    assert handle.cancel_count == 1
    assert raised.value.cancel["goal_id"] == goal_uuid.hex()


def test_preempt_overlap_exception_preserves_b_uuid_while_canceling_a(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    b_uuid = "34" * 16
    handle = FakeHandle(FakeFuture(SimpleNamespace(status=5, result=SimpleNamespace(
        error_code=0, error_string="canceled"))),
        FakeFuture(SimpleNamespace(goals_canceling=[goal_info(goal_uuid)])))
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"goal_id": goal_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": goal_uuid.hex(), "actual_rad": [0.002, 0, 0, 0, 0, 0]}]
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    _, cancel = movej.await_mid_motion_cancel(
        object(), handle, feedback, 0.001, movej.time.monotonic() + 5,
        overlap_probe=lambda: (_ for _ in ()).throw(RuntimeError("B reply lost")),
        overlap_goal_id=b_uuid)
    assert handle.cancel_count == 1
    assert cancel["overlap_probe"]["goal_id"] == b_uuid
    assert cancel["overlap_probe"]["status"] == "ADMISSION_UNCONFIRMED"


def test_preempt_overlap_interrupt_preserves_b_uuid_and_cancels_a(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    b_uuid = "34" * 16
    handle = FakeHandle(FakeFuture(SimpleNamespace(status=5, result=SimpleNamespace(
        error_code=0, error_string="canceled"))),
        FakeFuture(SimpleNamespace(goals_canceling=[goal_info(goal_uuid)])))
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"goal_id": goal_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": goal_uuid.hex(), "actual_rad": [0.002, 0, 0, 0, 0, 0]}]
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    with pytest.raises(movej.CancelAttemptError) as raised:
        movej.await_mid_motion_cancel(
            object(), handle, feedback, 0.001, movej.time.monotonic() + 5,
            overlap_probe=lambda: (_ for _ in ()).throw(KeyboardInterrupt("stop probe")),
            overlap_goal_id=b_uuid)
    assert handle.cancel_count == 1
    assert raised.value.cancel["overlap_probe"]["goal_id"] == b_uuid
    assert raised.value.cancel["overlap_probe"]["status"] == "ADMISSION_UNCONFIRMED"


def test_preempt_signal_with_known_b_handle_cleans_b_then_a(monkeypatch):
    a_uuid = bytes.fromhex("12" * 16)
    b_uuid = "34" * 16
    a = FakeHandle(FakeFuture(SimpleNamespace(status=5, result=SimpleNamespace(
        error_code=0, error_string="canceled"))),
        FakeFuture(SimpleNamespace(goals_canceling=[goal_info(a_uuid)])))
    a.goal_id = SimpleNamespace(uuid=a_uuid)
    b = FakeHandle(FakeFuture(), FakeFuture())
    b.accepted = True
    b.goal_id = SimpleNamespace(uuid=bytes.fromhex(b_uuid))
    feedback = [{"goal_id": a_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": a_uuid.hex(), "actual_rad": [0.002, 0, 0, 0, 0, 0]}]
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    monkeypatch.setattr(movej, "ros_goal_uuid", lambda goal_id: goal_id)
    def cleanup(node, handle, stop_deadline=None):
        handle.cancel_count += 1
        if handle is b and handle.cancel_count == 1:
            raise KeyboardInterrupt("interrupted while cleaning B")
        return {"goal_id": bytes(handle.goal_id.uuid).hex(), "cancel_requested": True}
    monkeypatch.setattr(movej, "cancel_accepted_on_interrupt", cleanup)
    client = SimpleNamespace(send_goal_async=lambda goal, goal_uuid: FakeFuture(b))
    with pytest.raises(movej.CancelAttemptError) as raised:
        movej.await_mid_motion_cancel(
            object(), a, feedback, 0.001, movej.time.monotonic() + 5,
            overlap_probe=lambda: movej.probe_overlap_goal(
                object(), client, object(), movej.time.monotonic() + 5, b_uuid),
            overlap_goal_id=b_uuid)
    assert b.cancel_count == 2
    assert a.cancel_count == 1
    assert raised.value.cancel["overlap_probe"]["status"] == "ADMISSION_UNCONFIRMED"


@pytest.mark.parametrize("accepted,wrong_identity,expected", [
    (False, False, "REJECTED"), (True, False, "ACCEPTED_UNEXPECTED"),
    (True, True, "ADMISSION_IDENTITY_MISMATCH"),
])
def test_overlap_probe_records_explicit_b_admission_and_cleans_unexpected_acceptance(
        monkeypatch, accepted, wrong_identity, expected):
    goal_id = "34" * 16
    sent = []
    handle = SimpleNamespace(accepted=accepted, goal_id=SimpleNamespace(
        uuid=bytes.fromhex("44" * 16 if wrong_identity else goal_id)))
    client = SimpleNamespace(send_goal_async=lambda goal, goal_uuid:
                             sent.append((goal, goal_uuid)) or FakeFuture(handle))
    uuid_type = lambda uuid: SimpleNamespace(uuid=uuid)
    monkeypatch.setitem(sys.modules, "unique_identifier_msgs",
                        SimpleNamespace(msg=SimpleNamespace(UUID=uuid_type)))
    monkeypatch.setitem(sys.modules, "unique_identifier_msgs.msg",
                        SimpleNamespace(UUID=uuid_type))
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    cleanup = []
    monkeypatch.setattr(movej, "cancel_accepted_on_interrupt", lambda *args, **kwargs:
                        cleanup.append(kwargs) or {"cancel_requested": True,
                                                 "action_result_confirmed": True})
    probe_deadline = movej.time.monotonic() + 5
    report = movej.probe_overlap_goal(object(), client, object(),
                                      probe_deadline, goal_id)
    assert report["goal_id"] == goal_id and report["status"] == expected
    assert sent[0][1].uuid == list(bytes.fromhex(goal_id))
    assert len(cleanup) == int(accepted)
    assert all(item["stop_deadline"] == probe_deadline for item in cleanup)


def test_preempt_probe_sends_only_a_and_b_and_preserves_unsent_c_uuid(monkeypatch):
    target, trajectory, planner, client, adapter, _, _ = admission_case(
        monkeypatch, (), 1)
    ids = {"A": "11" * 16, "B": "22" * 16, "C": "33" * 16}
    monkeypatch.setitem(sys.modules, "unique_identifier_msgs.msg",
                        SimpleNamespace(UUID=lambda uuid: SimpleNamespace(uuid=uuid)))
    plan = SimpleNamespace(motion_plan_response=SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        trajectory=SimpleNamespace(joint_trajectory=trajectory)))
    sent = []

    def send(goal, feedback_callback=None, goal_uuid=None):
        sent.append(bytes(goal_uuid.uuid).hex())
        if feedback_callback is not None:
            feedback_callback(SimpleNamespace(
                goal_id=SimpleNamespace(uuid=bytes.fromhex(ids["A"])),
                feedback=SimpleNamespace(joint_names=list(movej.JOINTS),
                    actual=SimpleNamespace(positions=list(target)),
                    desired=SimpleNamespace(positions=list(target)))))
        return len(sent)

    client.send_goal_async = send
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout:
        plan if future == "plan" else SimpleNamespace(
            accepted=future == 1,
            goal_id=SimpleNamespace(uuid=bytes.fromhex(ids["A" if future == 1 else "B"]))))

    probe_deadlines = []
    original_probe = movej.probe_overlap_goal
    monkeypatch.setattr(movej, "probe_overlap_goal", lambda node, client, goal, probe_deadline,
                        goal_id: probe_deadlines.append(probe_deadline) or
                        original_probe(node, client, goal, probe_deadline, goal_id))

    def cancel_a(node, handle, feedback, delta, deadline, accepted_index,
                 overlap_probe, overlap_goal_id):
        assert overlap_goal_id == ids["B"]
        return (SimpleNamespace(status=5, result=SimpleNamespace(
                    error_code=0, error_string="canceled")),
                {"goal_id": ids["A"], "acknowledged": True,
                 "overlap_probe": overlap_probe()})

    monkeypatch.setattr(movej, "await_mid_motion_cancel", cancel_a)
    deadline = movej.time.monotonic() + 20
    report = movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                                deadline, adapter,
                                preempt_probe=True, preempt_goal_ids=ids)
    assert sent == [ids["A"], ids["B"]]
    assert len(probe_deadlines) == 1
    assert probe_deadlines[0] <= movej.time.monotonic() + movej.PREEMPT_B_PROBE_TIMEOUT
    assert probe_deadlines[0] <= deadline - movej.STOP_CONFIRM_TIMEOUT
    assert report["preempt_goals"]["B"]["status"] == "REJECTED"
    assert report["preempt_goals"]["C"] == {
        "goal_id": ids["C"], "status": "NOT_SUBMITTED",
        "phase": "WAITING_FOR_CANCEL_RAW_AND_NEW_PERMIT"}


def test_preempt_then_goal_serializes_joiner_schema_after_permit(monkeypatch):
    ids = {"A": "11" * 16, "B": "22" * 16, "C": "33" * 16}
    events = []
    adapter = FakeLifecycleAdapter()
    adapter.deadline = movej.time.monotonic() + 30
    adapter.confirmed_hold = lambda: pytest.fail("Hold checked before op5")
    adapter.Trigger = SimpleNamespace(Request=lambda: object())
    permit_replies = iter((False, True))

    def call(name, kind, request):
        if name.endswith("set_bench_permit"):
            queued = next(permit_replies)
            events.append("op5_queued" if queued else "op5_unavailable")
            return SimpleNamespace(success=queued)
        assert name.endswith("bootstrap_status")
        status = ("OPERATION_COMPLETE" if "op5_queued" not in events
                  else "ENABLED_HOLD")
        events.append(status)
        return SimpleNamespace(success=status == "ENABLED_HOLD", message=status)

    adapter.call = call
    adapter.renew_motion_permit = types.MethodType(
        movej.RosControlAdapter.renew_motion_permit, adapter)
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: None)
    monkeypatch.setattr(movej, "verify_active_stack", lambda adapter: None)
    b = {"goal_id": ids["B"], "status": "REJECTED",
         "phase": "A_MOTION_FEEDBACK_OVERLAP",
         "second_begin_raw_join": "REQUIRED_SEPARATELY"}
    a = {"goal_id": ids["A"], "status": "CANCELED", "action_status": 5,
         "cancel": {"goal_id": ids["A"], "requested": True,
                    "acknowledged": True, "overlap_probe": b},
         "result_error_code": 0, "preempt_goals": {
             "A": {"goal_id": ids["A"], "status": "CANCELED"},
             "B": b,
             "C": {"goal_id": ids["C"], "status": "NOT_SUBMITTED"}}}

    def execute(*args, **kwargs):
        events.append("A" if kwargs.get("preempt_probe") else "C")
        if kwargs.get("preempt_probe"):
            return a
        assert kwargs["goal_uuid_hex"] == ids["C"]
        assert 0 < kwargs["submission_deadline"] - movej.time.monotonic() <= 4
        assert args[5] - kwargs["submission_deadline"] > 20
        return {"goal_id": ids["C"], "status": "SUCCEEDED",
                "action_status": 4, "result_error_code": 0}

    monkeypatch.setattr(movej, "execute_goal", execute)
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(
        init=lambda: None, create_node=lambda name: SimpleNamespace(
            create_client=lambda *args: SimpleNamespace(wait_for_service=lambda **kw: True),
            destroy_node=lambda: None), shutdown=lambda: None))
    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(
        ActionClient=lambda *args: SimpleNamespace(wait_for_server=lambda **kw: True)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(FollowJointTrajectory=object))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(GetMotionPlan=object))
    report = movej.capture((0, 0, 1.59, 0, 0, 0), 30, movej.LIMITS,
                           guarded_session=True, preempt_then_goal=True,
                           post_cancel_target=(0.01, 0, 1.6, 0, 0, 0),
                           preempt_goal_ids=ids)
    assert events == ["A", "op5_unavailable", "OPERATION_COMPLETE",
                      "op5_queued", "ENABLED_HOLD", "C", "release"]
    assert report["scope"] == "ROS_PREEMPT_RECOVERY"
    assert report["status"] == "SUCCEEDED"
    assert report["stop_confirmation"] == "CONFIRMED_RELEASED_IDLE"
    assert report["requested_goals"] == report["completed_goals"] == 2
    assert [(goal["index"], goal["status"]) for goal in report["goals"]] == [
        (0, "CANCELED"), (1, "SUCCEEDED")]
    assert report["preempt_goals"]["C"]["status"] == "SUCCEEDED"
    assert report["second_begin_raw_join"] == "REQUIRED_SEPARATELY"
    assert report["qualification"] == "NOT_QUALIFIED"
    serialized = json.loads(json.dumps(report))
    assert serialized["scope"] == "ROS_PREEMPT_RECOVERY"
    assert serialized["goals"][0]["cancel"]["overlap_probe"] == \
        serialized["preempt_goals"]["B"]
    assert serialized["preempt_goals"]["B"]["phase"] == \
        "A_MOTION_FEEDBACK_OVERLAP"
    assert serialized["preempt_goals"]["B"]["second_begin_raw_join"] == \
        "REQUIRED_SEPARATELY"
    assert [(goal["goal_id"], goal["status"]) for goal in serialized["goals"]] == [
        (ids["A"], "CANCELED"), (ids["C"], "SUCCEEDED")]


@pytest.mark.parametrize("a_status,b_status,permit_ok", [
    ("FAILED", "REJECTED", True), ("CANCELED", "ADMISSION_UNCONFIRMED", True),
    ("CANCELED", "REJECTED", False)])
def test_preempt_then_goal_fails_closed_before_c(monkeypatch, a_status, b_status, permit_ok):
    ids = {"A": "11" * 16, "B": "22" * 16, "C": "33" * 16}
    events = []
    adapter = FakeLifecycleAdapter()
    adapter.confirmed_hold = lambda: pytest.fail("Hold checked before op5")

    def renew():
        events.append("permit")
        if not permit_ok:
            raise RuntimeError("new motion permit unconfirmed")

    adapter.renew_motion_permit = renew
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: None)
    monkeypatch.setattr(movej, "verify_active_stack", lambda adapter: None)
    monkeypatch.setattr(movej, "execute_goal", lambda *args, **kwargs:
        events.append("A" if kwargs.get("preempt_probe") else "C") or {
            "goal_id": ids["A"], "status": a_status, "action_status": 5,
            "result_error_code": 0, "preempt_goals": {
                "A": {"goal_id": ids["A"], "status": a_status},
                "B": {"goal_id": ids["B"], "status": b_status},
                "C": {"goal_id": ids["C"], "status": "NOT_SUBMITTED"}}})
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(
        init=lambda: None, create_node=lambda name: SimpleNamespace(
            create_client=lambda *args: SimpleNamespace(wait_for_service=lambda **kw: True),
            destroy_node=lambda: None), shutdown=lambda: None))
    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(
        ActionClient=lambda *args: SimpleNamespace(wait_for_server=lambda **kw: True)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(FollowJointTrajectory=object))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(GetMotionPlan=object))
    with pytest.raises(movej.UnconfirmedStopError):
        movej.capture((0, 0, 1.59, 0, 0, 0), 30, movej.LIMITS,
                      guarded_session=True, preempt_then_goal=True,
                      post_cancel_target=(0.01, 0, 1.6, 0, 0, 0),
                      preempt_goal_ids=ids)
    assert "C" not in events
    assert events.count("release") == 1


def test_post_cancel_permit_failure_releases_once_and_deactivates_owned_stack(monkeypatch):
    ids = {"A": "11" * 16, "B": "22" * 16, "C": "33" * 16}
    events = []
    states = {"hardware": "active", "joint_state_broadcaster": "active",
              "arm_controller": "active"}
    adapter = SimpleNamespace(
        settle_pending=lambda deadline: True,
        confirmed_stop=lambda: events.append("release") or True,
        hardware_state=lambda name, deadline=None: states["hardware"],
        controller_state=lambda name, deadline=None: states[name],
        deactivate_hardware=lambda name: states.update(hardware="inactive"),
        deactivate_controller=lambda name: states.update({name: "inactive"}),
        renew_motion_permit=lambda: (_ for _ in ()).throw(
            RuntimeError("new motion permit unconfirmed")))
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: None)
    monkeypatch.setattr(movej, "activate_controller_stack", lambda adapter, owned:
                        owned.extend(("hardware", *movej.CONTROLLERS)))
    a = {"goal_id": ids["A"], "status": "CANCELED", "action_status": 5,
         "result_error_code": 0, "preempt_goals": {
             "A": {"goal_id": ids["A"], "status": "CANCELED"},
             "B": {"goal_id": ids["B"], "status": "REJECTED"},
             "C": {"goal_id": ids["C"], "status": "NOT_SUBMITTED"}}}
    monkeypatch.setattr(movej, "execute_goal", lambda *args, **kwargs:
                        events.append("A" if kwargs.get("preempt_probe") else "C") or a)
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(
        init=lambda: None, create_node=lambda name: SimpleNamespace(
            create_client=lambda *args: SimpleNamespace(wait_for_service=lambda **kw: True),
            destroy_node=lambda: None), shutdown=lambda: None))
    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(
        ActionClient=lambda *args: SimpleNamespace(wait_for_server=lambda **kw: True)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(FollowJointTrajectory=object))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(GetMotionPlan=object))
    with pytest.raises(movej.UnconfirmedStopError, match="new motion permit unconfirmed"):
        movej.capture((0, 0, 1.59, 0, 0, 0), 30, movej.LIMITS,
                      guarded_session=True, activate_controller=True,
                      preempt_then_goal=True,
                      post_cancel_target=(0.01, 0, 1.6, 0, 0, 0),
                      preempt_goal_ids=ids)
    assert events == ["A", "release"]
    assert set(states.values()) == {"inactive"}


def test_preempt_then_goal_cli_passes_validated_second_target_and_reserves_uuids(
        tmp_path, monkeypatch):
    ids = iter(("11" * 16, "22" * 16, "33" * 16))
    monkeypatch.setattr(movej.uuid, "uuid4", lambda: SimpleNamespace(hex=next(ids)))
    monkeypatch.setattr(movej, "load_bench_limits", lambda *args: movej.LIMITS)
    seen = []
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs:
                        seen.append(kwargs) or {"scope": "ROS_PREEMPT_RECOVERY",
                                                 "status": "SUCCEEDED"})
    report_path = tmp_path / "recovery.json"
    monkeypatch.setattr(sys, "argv", ["ros_six_axis_movej.py", "--reference-frame",
        "BENCH_SESSION", "--candidate-manifest", str(tmp_path / "manifest.json"),
        "--ros-limits-json", str(tmp_path / "limits.json"),
        "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
        "--target", "0", "0", "1.59", "0", "0", "0",
        "--preempt-then-goal", "--post-cancel-target", "0.01", "0", "1.6", "0",
        "0", "0", "--report", str(report_path)])
    assert movej.main() == 0
    assert seen[0]["preempt_then_goal"] is True
    assert seen[0]["post_cancel_target"] == (0.01, 0, 1.6, 0, 0, 0)
    assert seen[0]["preempt_goal_ids"] == dict(zip("ABC", ("11" * 16, "22" * 16,
                                                        "33" * 16)))


@pytest.mark.parametrize("extra", [
    [], ["--post-cancel-target", "0", "0", "1.6", "0", "0", "0",
         "--preempt-probe"],
    ["--post-cancel-target", "0", "0", "1.6", "0", "0", "0",
     "--timeout", "121"],
    ["--post-cancel-target", "0", "0", "1.6", "0", "0", "20"]])
def test_preempt_then_goal_cli_rejects_missing_or_conflicting_target(
        tmp_path, monkeypatch, extra):
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs: pytest.fail("ROS called"))
    monkeypatch.setattr(movej, "load_bench_limits", lambda *args: movej.LIMITS)
    monkeypatch.setattr(sys, "argv", ["ros_six_axis_movej.py", "--reference-frame",
        "BENCH_SESSION", "--candidate-manifest", str(tmp_path / "manifest.json"),
        "--ros-limits-json", str(tmp_path / "limits.json"),
        "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
        "--target", "0", "0", "1.59", "0", "0", "0",
        "--preempt-then-goal", "--report", str(tmp_path / "report.json"), *extra])
    if extra and extra[-1] == "20":
        assert movej.main() == 1
    else:
        with pytest.raises(SystemExit) as error:
            movej.main()
        assert error.value.code == 2


def test_reserved_c_uuid_is_sent_and_result_future_error_requests_cancel(monkeypatch):
    target, _, planner, client, adapter, _, _ = admission_case(monkeypatch, (), 1)
    reserved = "33" * 16
    monkeypatch.setitem(sys.modules, "unique_identifier_msgs.msg",
                        SimpleNamespace(UUID=lambda uuid: SimpleNamespace(uuid=uuid)))
    sent = []
    handle = SimpleNamespace(accepted=True,
                             goal_id=SimpleNamespace(uuid=bytes.fromhex(reserved)))
    client.send_goal_async = lambda goal, feedback_callback=None, goal_uuid=None: (
        sent.append(bytes(goal_uuid.uuid).hex()) or "accepted")
    original_wait = movej.wait
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout:
                        handle if future == "accepted" else original_wait(node, future, timeout))
    monkeypatch.setattr(movej, "await_accepted_goal", lambda *args:
                        (_ for _ in ()).throw(AttributeError("result future unavailable")))
    canceled = []
    monkeypatch.setattr(movej, "cancel_accepted_on_interrupt", lambda *args, **kwargs:
                        canceled.append(kwargs) or {"cancel_requested": True})
    with pytest.raises(movej.GoalAttemptError) as error:
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter,
                           goal_uuid_hex=reserved)
    assert sent == [reserved]
    assert len(canceled) == 1
    assert error.value.goal_report["goal_id"] == reserved


def test_reserved_c_admission_budget_does_not_shorten_result_wait(monkeypatch):
    target, _, planner, client, adapter, _, _ = admission_case(monkeypatch, (), 1)
    reserved = "33" * 16
    monkeypatch.setitem(sys.modules, "unique_identifier_msgs.msg",
                        SimpleNamespace(UUID=lambda uuid: SimpleNamespace(uuid=uuid)))
    handle = SimpleNamespace(accepted=True,
                             goal_id=SimpleNamespace(uuid=bytes.fromhex(reserved)))

    def send(goal, feedback_callback=None, goal_uuid=None):
        assert bytes(goal_uuid.uuid).hex() == reserved
        feedback_callback(SimpleNamespace(
            goal_id=handle.goal_id, feedback=SimpleNamespace(
                joint_names=list(movej.JOINTS),
                actual=SimpleNamespace(positions=list(target)),
                desired=SimpleNamespace(positions=list(target)))))
        return "accepted"

    client.send_goal_async = send
    original_wait = movej.wait
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout:
                        handle if future == "accepted" else original_wait(node, future, timeout))
    result_waits = []
    monkeypatch.setattr(movej, "await_accepted_goal", lambda node, handle, timeout:
                        result_waits.append(timeout) or SimpleNamespace(
                            status=4, result=SimpleNamespace(error_code=0, error_string="")))
    now = movej.time.monotonic()
    report = movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                                now + 20, adapter, goal_uuid_hex=reserved,
                                submission_deadline=now + 4)
    assert report["status"] == "SUCCEEDED"
    assert result_waits[0] > 15


def test_preempt_then_goal_c_failure_retains_a_b_c_and_releases(monkeypatch):
    ids = {"A": "11" * 16, "B": "22" * 16, "C": "33" * 16}
    events = []
    adapter = FakeLifecycleAdapter()
    adapter.confirmed_hold = lambda: True
    adapter.renew_motion_permit = lambda: events.append("permit")
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: None)
    monkeypatch.setattr(movej, "verify_active_stack", lambda adapter: None)
    a = {"goal_id": ids["A"], "status": "CANCELED", "action_status": 5,
         "result_error_code": 0, "preempt_goals": {
             "A": {"goal_id": ids["A"], "status": "CANCELED"},
             "B": {"goal_id": ids["B"], "status": "REJECTED"},
             "C": {"goal_id": ids["C"], "status": "NOT_SUBMITTED"}}}

    def execute(*args, **kwargs):
        if kwargs.get("preempt_probe"):
            return a
        raise movej.GoalAttemptError("C result unavailable", {
            "goal_id": ids["C"], "status": "FAILED", "reason": "C result unavailable"})

    monkeypatch.setattr(movej, "execute_goal", execute)
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(
        init=lambda: None, create_node=lambda name: SimpleNamespace(
            create_client=lambda *args: SimpleNamespace(wait_for_service=lambda **kw: True),
            destroy_node=lambda: None), shutdown=lambda: None))
    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(
        ActionClient=lambda *args: SimpleNamespace(wait_for_server=lambda **kw: True)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(FollowJointTrajectory=object))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(GetMotionPlan=object))
    with pytest.raises(movej.UnconfirmedStopError) as error:
        movej.capture((0, 0, 1.59, 0, 0, 0), 30, movej.LIMITS,
                      guarded_session=True, preempt_then_goal=True,
                      post_cancel_target=(0.01, 0, 1.6, 0, 0, 0),
                      preempt_goal_ids=ids)
    failed = error.value.failed_goal_report
    assert events == ["permit", "release"]
    assert failed["preempt_goals"]["A"]["status"] == "CANCELED"
    assert failed["preempt_goals"]["B"]["status"] == "REJECTED"
    assert failed["preempt_goals"]["C"]["goal_id"] == ids["C"]
    assert [goal["index"] for goal in failed["goals"]] == [0, 1]


def test_preempt_b_budget_expiry_skips_b_and_still_cancels_a(monkeypatch):
    target, _, planner, client, adapter, _, _ = admission_case(monkeypatch, (), 1)
    ids = {"A": "01" * 16, "B": "22" * 16, "C": "33" * 16}
    monkeypatch.setitem(sys.modules, "unique_identifier_msgs.msg",
                        SimpleNamespace(UUID=lambda uuid: SimpleNamespace(uuid=uuid)))
    sent = []
    original_send = client.send_goal_async
    client.send_goal_async = lambda goal, feedback_callback=None, goal_uuid=None: \
        sent.append(goal_uuid) or original_send(goal, feedback_callback)
    monkeypatch.setattr(movej, "await_mid_motion_cancel",
                        lambda node, handle, feedback, delta, deadline, accepted_index,
                        overlap_probe, overlap_goal_id: (
                            SimpleNamespace(status=5, result=SimpleNamespace(
                                error_code=0, error_string="canceled")),
                            {"goal_id": ids["A"], "acknowledged": True,
                             "overlap_probe": overlap_probe()}))
    report = movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                                movej.time.monotonic() + movej.STOP_CONFIRM_TIMEOUT / 2,
                                adapter, preempt_probe=True, preempt_goal_ids=ids)
    assert len(sent) == 1
    assert report["preempt_goals"]["B"]["goal_id"] == ids["B"]
    assert report["preempt_goals"]["B"]["status"] == "NOT_SUBMITTED"
    assert report["preempt_goals"]["B"]["phase"] == "BUDGET_EXHAUSTED"


def test_preempt_probe_cli_is_bounded_and_never_reports_success(tmp_path, monkeypatch):
    fixture_module = Path(__file__).with_name("test_convert_application_ros_limits.py")
    spec = importlib.util.spec_from_file_location("converter_fixture", fixture_module)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    manifest, limits_json, _ = module.source(tmp_path)
    report_path = tmp_path / "preempt.json"
    argv = ["ros_six_axis_movej.py", "--reference-frame", "BENCH_SESSION",
            "--candidate-manifest", str(manifest), "--ros-limits-json", str(limits_json),
            "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
            "--target", "0", "0", "0", "0", "0", "0",
            "--preempt-probe", "--report", str(report_path)]
    seen = []
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs:
        seen.append(kwargs) or {"scope": "ROS_PREEMPT_PROBE", "status": "SUCCEEDED",
            "qualification": "NOT_QUALIFIED", "goals": {
                "A": {"goal_id": "1" * 32}, "B": {"goal_id": "2" * 32},
                "C": {"goal_id": "3" * 32, "phase": "NOT_SUBMITTED"}}})
    monkeypatch.setattr("sys.argv", argv)
    assert movej.main() == 1
    assert seen[0]["preempt_probe"] is True
    report = json.loads(report_path.read_text())
    assert report["status"] == "UNCONFIRMED_STOP"
    assert set(report["goals"]) == {"A", "B", "C"}


def test_preempt_setup_interrupt_preserves_reserved_uuids_in_unqualified_report(
        tmp_path, monkeypatch):
    report_path = tmp_path / "preempt-interrupted.json"
    ids = iter(("11" * 16, "22" * 16, "33" * 16))
    monkeypatch.setattr(movej.uuid, "uuid4", lambda: SimpleNamespace(hex=next(ids)))
    monkeypatch.setattr(movej, "load_bench_limits", lambda *args: movej.LIMITS)
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs:
                        (_ for _ in ()).throw(KeyboardInterrupt("before adapter binding")))
    monkeypatch.setattr(sys, "argv", ["ros_six_axis_movej.py", "--reference-frame",
        "BENCH_SESSION", "--candidate-manifest", str(tmp_path / "manifest.json"),
        "--ros-limits-json", str(tmp_path / "limits.json"),
        "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
        "--target", "0", "0", "1.59", "0", "0", "0", "--preempt-probe",
        "--report", str(report_path)])
    try:
        code = movej.main()
    except KeyboardInterrupt:
        pytest.fail("preempt setup interrupt escaped without a report")
    assert code == 1
    report = json.loads(report_path.read_text())
    assert report["status"] == "UNCONFIRMED_STOP"
    assert report["qualification"] == "NOT_QUALIFIED"
    assert report["goals"] == {name: {"goal_id": goal_id, "status": "NOT_SUBMITTED",
                                      "phase": "SETUP_UNCONFIRMED"}
                               for name, goal_id in zip("ABC", ("11" * 16, "22" * 16,
                                                         "33" * 16))}


@pytest.mark.parametrize("extra", [
    ["--cancel-mid-motion"], ["--timeout", "121"],
])
def test_preempt_probe_rejects_conflicting_or_unbounded_mode(tmp_path, monkeypatch, extra):
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs: pytest.fail("ROS called"))
    monkeypatch.setattr("sys.argv", ["ros_six_axis_movej.py", "--reference-frame",
        "BENCH_SESSION", "--target", "0", "0", "1.59", "0", "0", "0",
        "--preempt-probe", "--report", str(tmp_path / "report.json"), *extra])
    with pytest.raises(SystemExit) as error:
        movej.main()
    assert error.value.code == 2


def test_mid_motion_cancel_ignores_foreign_goal_and_nonfinite_feedback(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    handle = FakeHandle(FakeFuture(SimpleNamespace(status=4)), FakeFuture())
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"goal_id": goal_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": "34" * 16, "actual_rad": [0.1, 0, 0, 0, 0, 0]},
                {"goal_id": goal_uuid.hex(), "actual_rad": [float("nan"), 0, 0, 0, 0, 0]}]
    with pytest.raises(movej.CancelAttemptError, match="before observed"):
        movej.await_mid_motion_cancel(
            object(), handle, feedback, 0.001, movej.time.monotonic() + 5)
    assert handle.cancel_count == 0


@pytest.mark.parametrize("feedback,ack,status", [
    ([{"actual_rad": [0.0] * 6}], True, 5),
    ([{"actual_rad": [0.0] * 6}, {"actual_rad": [0.002, 0, 0, 0, 0, 0]}], False, 5),
    ([{"actual_rad": [0.0] * 6}, {"actual_rad": [0.002, 0, 0, 0, 0, 0]}], True, 4),
])
def test_mid_motion_cancel_rejects_missing_motion_ack_or_canceled_result(
        monkeypatch, feedback, ack, status):
    goal_uuid = bytes.fromhex("12" * 16)
    result_future = FakeFuture(SimpleNamespace(status=status, result=SimpleNamespace(
        error_code=0, error_string="")))
    cancel_future = FakeFuture(SimpleNamespace(goals_canceling=[
        goal_info(goal_uuid if ack else bytes(16))]))
    handle = FakeHandle(result_future, cancel_future)
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())

    with pytest.raises(RuntimeError):
        movej.await_mid_motion_cancel(
            object(), handle, feedback, 0.001, movej.time.monotonic() + 5)


def test_mid_motion_cancel_ignores_feedback_before_acceptance(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    handle = FakeHandle(FakeFuture(SimpleNamespace(status=4)), FakeFuture())
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"actual_rad": [0.0] * 6},
                {"actual_rad": [0.002, 0, 0, 0, 0, 0]}]
    with pytest.raises(movej.CancelAttemptError, match="before observed"):
        movej.await_mid_motion_cancel(
            object(), handle, feedback, 0.001, movej.time.monotonic() + 5,
            accepted_feedback_index=2)
    assert handle.cancel_count == 0


def test_mid_motion_cancel_waits_for_post_acceptance_feedback(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    handle = FakeHandle(FakeFuture(SimpleNamespace(status=5, result=SimpleNamespace(
        error_code=0, error_string="")), done=False),
        FakeFuture(SimpleNamespace(goals_canceling=[goal_info(goal_uuid)])))
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    feedback = [{"goal_id": goal_uuid.hex(), "actual_rad": [0.0] * 6},
                {"goal_id": goal_uuid.hex(), "actual_rad": [0.1, 0, 0, 0, 0, 0]}]
    observed = []

    def spin_once(node, timeout_sec):
        observed.append(handle.cancel_count)
        feedback.append({"goal_id": goal_uuid.hex(),
                         "actual_rad": [0.1 + 0.002 * len(observed), 0, 0, 0, 0, 0]})

    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(spin_once=spin_once))
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    result, cancel = movej.await_mid_motion_cancel(
        object(), handle, feedback, 0.001, movej.time.monotonic() + 5,
        accepted_feedback_index=2)
    assert result.status == 5
    assert observed == [0, 0]
    assert cancel["motion_feedback_start_index"] == 2
    assert cancel["motion_feedback_trigger_index"] == 3


def test_cancel_mode_requires_bounded_guarded_single_goal(tmp_path, monkeypatch):
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "ABSOLUTE_MODEL",
        "--target", "0", "0", "1.59", "0", "0", "0",
        "--cancel-mid-motion", "--report", str(tmp_path / "report.json")])
    with pytest.raises(SystemExit, match="2"):
        movej.main()
    assert not (tmp_path / "report.json").exists()


def test_cancel_mode_cli_routes_to_guarded_action_and_keeps_qualification_separate(
        tmp_path, monkeypatch):
    captured = {}
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "BENCH_SESSION",
        "--target", "0", "0", "1.59", "0", "0", "0",
        "--candidate-manifest", str(tmp_path / "manifest.json"),
        "--ros-limits-json", str(tmp_path / "limits.json"),
        "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
        "--cancel-mid-motion", "--report", str(tmp_path / "report.json")])
    monkeypatch.setattr(movej, "load_bench_limits", lambda *args: movej.LIMITS)

    def capture(*args, **kwargs):
        captured.update(kwargs)
        return {"status": "UNCONFIRMED_STOP", "goal_id": "12" * 16,
                "action_status": 5, "action_observation": "CANCELED",
                "stop_confirmation": "UNCONFIRMED_STOP",
                "raw_stop_release_join": "REQUIRED_SEPARATELY",
                "safety_verdict": "NOT_QUALIFIED"}

    monkeypatch.setattr(movej, "capture", capture)
    assert movej.main() == 1
    assert captured["cancel_mid_motion"] is True
    assert captured["guarded_session"] is True
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["safety_verdict"] == "NOT_QUALIFIED"


def test_cancel_cli_never_returns_zero_for_action_only_canceled(tmp_path, monkeypatch):
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "BENCH_SESSION",
        "--target", "0", "0", "1.59", "0", "0", "0",
        "--candidate-manifest", str(tmp_path / "manifest.json"),
        "--ros-limits-json", str(tmp_path / "limits.json"),
        "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
        "--cancel-mid-motion", "--report", str(tmp_path / "report.json")])
    monkeypatch.setattr(movej, "load_bench_limits", lambda *args: movej.LIMITS)
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs:
                        {"status": "CANCELED", "action_status": 5})
    assert movej.main() == 1


def test_accepted_interrupt_requests_bounded_cancel_and_retains_result(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    result = SimpleNamespace(status=5, result=SimpleNamespace(error_code=0,
                                                              error_string="interrupted"))
    handle = FakeHandle(FakeFuture(result), FakeFuture(SimpleNamespace(goals_canceling=[
        goal_info(goal_uuid)])))
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())
    report = movej.cancel_accepted_on_interrupt(object(), handle)
    assert handle.cancel_count == 1
    assert report["goal_id"] == goal_uuid.hex()
    assert report["action_status"] == 5
    assert report["result_error_string"] == "interrupted"
    assert report["cancel_acknowledged"] is True


def test_interrupt_cancel_ack_uses_goal_info_goal_id(monkeypatch):
    goal_uuid = bytes.fromhex("12" * 16)
    result = SimpleNamespace(status=5, result=SimpleNamespace(error_code=0,
                                                              error_string="interrupted"))
    handle = FakeHandle(FakeFuture(result), FakeFuture(SimpleNamespace(goals_canceling=[
        goal_info(goal_uuid)])))
    handle.goal_id = SimpleNamespace(uuid=goal_uuid)
    monkeypatch.setattr(movej, "wait", lambda node, future, timeout: future.result())

    report = movej.cancel_accepted_on_interrupt(object(), handle)

    assert report["cancel_acknowledged"] is True


@pytest.mark.parametrize("interrupt", [KeyboardInterrupt, SystemExit])
def test_accepted_single_goal_interrupt_preserves_cancel_result(monkeypatch, interrupt):
    target, _, planner, client, adapter, _, _ = admission_case(
        monkeypatch, (), 1)
    observed = []
    monkeypatch.setattr(movej, "await_accepted_goal", lambda node, handle, timeout:
                        (_ for _ in ()).throw(interrupt("user interrupt")))
    monkeypatch.setattr(movej, "cancel_accepted_on_interrupt", lambda node, handle:
                        observed.append(bytes(handle.goal_id.uuid).hex()) or {
                            "goal_id": bytes(handle.goal_id.uuid).hex(),
                            "cancel_requested": True, "cancel_acknowledged": True,
                            "action_status": 5, "result_error_code": 0,
                            "result_error_string": "interrupted"})
    with pytest.raises(movej.GoalAttemptError) as raised:
        movej.execute_goal(object(), planner, client, target, movej.LIMITS,
                           movej.time.monotonic() + 5, adapter)
    assert observed == ["01" * 16]
    assert raised.value.goal_report["goal_id"] == "01" * 16
    assert raised.value.goal_report["action_status"] == 5


def test_cancel_capture_keeps_goal_when_gate_only_observes_global_idle(monkeypatch):
    events = []
    node = SimpleNamespace(create_client=lambda kind, name:
                           SimpleNamespace(wait_for_service=lambda timeout_sec: True),
                           destroy_node=lambda: events.append("destroy_node"))
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: events.append("init")
    rclpy.create_node = lambda name: node
    rclpy.shutdown = lambda: events.append("shutdown")
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(ActionClient=lambda *args:
                            SimpleNamespace(wait_for_server=lambda timeout_sec: True)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(
        FollowJointTrajectory=object))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(GetMotionPlan=object))
    adapter = FakeLifecycleAdapter()
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: None)
    monkeypatch.setattr(movej, "verify_active_stack", lambda adapter: None)
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("release") or ([], []))
    failed = {"goal_id": "12" * 16, "action_status": 5,
              "result_error_code": 0, "result_error_string": "canceled"}
    monkeypatch.setattr(movej, "execute_goal", lambda *args, **kwargs:
                        (_ for _ in ()).throw(movej.GoalAttemptError("interrupted", failed)))
    with pytest.raises(movej.UnconfirmedStopError) as raised:
        movej.capture((0, 0, 1.59, 0, 0, 0), 5, movej.LIMITS,
                      guarded_session=True, cancel_mid_motion=True)
    assert events == ["init", "release", "destroy_node", "shutdown"]
    assert raised.value.failed_goal_report["goal_id"] == "12" * 16
    assert raised.value.failed_goal_report["stop_confirmation"] == "UNCONFIRMED_STOP"


def test_preempt_release_interrupt_keeps_submitted_a_b_and_does_not_retry(
        tmp_path, monkeypatch):
    events = []
    node = SimpleNamespace(create_client=lambda kind, name:
                           SimpleNamespace(wait_for_service=lambda timeout_sec: True),
                           destroy_node=lambda: events.append("destroy_node"))
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: events.append("init")
    rclpy.create_node = lambda name: node
    rclpy.shutdown = lambda: events.append("shutdown")
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(ActionClient=lambda *args:
                            SimpleNamespace(wait_for_server=lambda timeout_sec: True)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(
        FollowJointTrajectory=object))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(GetMotionPlan=object))
    adapter = FakeLifecycleAdapter()
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: None)
    monkeypatch.setattr(movej, "verify_active_stack", lambda adapter: None)
    monkeypatch.setattr(movej, "recover_after_failure", lambda adapter, owned:
                        events.append("unexpected_second_release") or ([], []))
    goals = {"A": {"goal_id": "11" * 16, "status": "CANCELED"},
             "B": {"goal_id": "22" * 16, "status": "REJECTED"},
             "C": {"goal_id": "33" * 16, "status": "NOT_SUBMITTED"}}
    monkeypatch.setattr(movej, "execute_goal", lambda *args, **kwargs:
                        {"goal_id": "11" * 16, "status": "CANCELED",
                         "preempt_goals": goals})
    monkeypatch.setattr(movej, "complete_cancel_observation", lambda *args:
                        (_ for _ in ()).throw(KeyboardInterrupt("release interrupted")))
    try:
        with pytest.raises(movej.UnconfirmedStopError) as raised:
            movej.capture((0, 0, 1.59, 0, 0, 0), 10, movej.LIMITS,
                          guarded_session=True, preempt_probe=True,
                          preempt_goal_ids={key: value["goal_id"] for key, value in goals.items()})
    except KeyboardInterrupt:
        pytest.fail("release interrupt escaped without preserving A/B history")
    assert raised.value.failed_goal_report["preempt_goals"] == goals
    assert raised.value.failed_goal_report["stop_confirmation"] == "UNCONFIRMED_STOP"
    assert events == ["init", "destroy_node", "shutdown"]
    report_path = tmp_path / "interrupted-release.json"
    monkeypatch.setattr(movej, "load_bench_limits", lambda *args: movej.LIMITS)
    monkeypatch.setattr(movej, "capture", lambda *args, **kwargs:
                        (_ for _ in ()).throw(raised.value))
    monkeypatch.setattr(sys, "argv", ["ros_six_axis_movej.py", "--reference-frame",
        "BENCH_SESSION", "--candidate-manifest", str(tmp_path / "manifest.json"),
        "--ros-limits-json", str(tmp_path / "limits.json"),
        "--activation-sha256", "a" * 64, "--device-id", "b" * 32,
        "--target", "0", "0", "1.59", "0", "0", "0", "--preempt-probe",
        "--report", str(report_path)])
    assert movej.main() == 1
    persisted = json.loads(report_path.read_text())
    assert persisted["goals"] == goals
    assert persisted["status"] == "UNCONFIRMED_STOP"
    assert persisted["qualification"] == "NOT_QUALIFIED"


@pytest.mark.parametrize("release_fails", (False, True))
@pytest.mark.parametrize("teardown_step", ("destroy", "shutdown"))
def test_preempt_teardown_failure_preserves_real_goal_history_and_prior_error(
        monkeypatch, release_fails, teardown_step):
    events = []
    node = SimpleNamespace(create_client=lambda kind, name:
                           SimpleNamespace(wait_for_service=lambda timeout_sec: True),
                           destroy_node=lambda: (_ for _ in ()).throw(
                               RuntimeError("node teardown failed")) if teardown_step == "destroy"
                               else events.append("destroy_node"))
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: events.append("init")
    rclpy.create_node = lambda name: node
    rclpy.shutdown = lambda: (_ for _ in ()).throw(
        RuntimeError("shutdown failed")) if teardown_step == "shutdown" else events.append("shutdown")
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(ActionClient=lambda *args:
                            SimpleNamespace(wait_for_server=lambda timeout_sec: True)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(
        FollowJointTrajectory=object))
    monkeypatch.setitem(sys.modules, "moveit_msgs.srv", SimpleNamespace(GetMotionPlan=object))
    adapter = FakeLifecycleAdapter()
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: adapter)
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: None)
    monkeypatch.setattr(movej, "verify_active_stack", lambda adapter: None)
    goals = {"A": {"goal_id": "11" * 16, "status": "CANCELED"},
             "B": {"goal_id": "22" * 16, "status": "REJECTED"},
             "C": {"goal_id": "33" * 16, "status": "NOT_SUBMITTED"}}
    report = {"goal_id": goals["A"]["goal_id"], "status": "CANCELED",
              "preempt_goals": goals}
    monkeypatch.setattr(movej, "execute_goal", lambda *args, **kwargs: report)
    if release_fails:
        def release(*args):
            raise movej.UnconfirmedStopError("original release unconfirmed",
                failed_goal_report={**report, "stop_confirmation": "UNCONFIRMED_STOP"})
    else:
        release = lambda *args: {**report, "stop_confirmation": "UNCONFIRMED_STOP"}
    monkeypatch.setattr(movej, "complete_cancel_observation", release)
    try:
        with pytest.raises(movej.UnconfirmedStopError) as raised:
            movej.capture((0, 0, 1.59, 0, 0, 0), 10, movej.LIMITS,
                          guarded_session=True, preempt_probe=True,
                          preempt_goal_ids={name: item["goal_id"] for name, item in goals.items()})
    except RuntimeError:
        pytest.fail("teardown replaced preempt goal history")
    assert raised.value.failed_goal_report["preempt_goals"] == goals
    if release_fails:
        assert "original release unconfirmed" in str(raised.value)
    assert events == (["init", "shutdown"] if teardown_step == "destroy" else
                      ["init", "destroy_node"])


def test_direct_cancel_capture_rejects_unguarded_or_unbounded_before_ros():
    target = (0, 0, 1.59, 0, 0, 0)
    with pytest.raises(ValueError, match="guarded"):
        movej.capture(target, 5.0, movej.LIMITS, cancel_mid_motion=True)
    with pytest.raises(ValueError, match="bounded"):
        movej.capture(target, 121.0, movej.LIMITS, guarded_session=True,
                      cancel_mid_motion=True)


def test_cancel_failure_report_keeps_goal_and_action_evidence(tmp_path, monkeypatch):
    failed = {"goal_id": "12" * 16, "action_status": 5,
              "result_error_code": 0, "result_error_string": "canceled",
              "cancel": {"goal_id": "12" * 16, "acknowledged": True},
              "device_stop_evidence": "REQUIRED_SEPARATELY",
              "device_release_evidence": "REQUIRED_SEPARATELY"}
    monkeypatch.setattr("sys.argv", [
        "ros_six_axis_movej.py", "--reference-frame", "ABSOLUTE_MODEL",
        "--target", "0", "0", "1.59", "0", "0", "0",
        "--report", str(tmp_path / "report.json")])
    monkeypatch.setattr(movej, "capture", lambda *args: (_ for _ in ()).throw(
        movej.ConfirmedStopError("cancel result recorded", failed_goal_report=failed)))
    assert movej.main() == 1
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["goal"] == failed
    assert report["stop_confirmation"] == "CONFIRMED_RELEASED_IDLE"
    assert report["status"] != "SUCCEEDED"
