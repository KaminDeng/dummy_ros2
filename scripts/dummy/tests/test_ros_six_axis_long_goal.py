"""六轴长目标的离线边界，绝不启动 ROS 或硬件。"""
##
# @file test_ros_six_axis_long_goal.py
# @brief 600秒单目标路径、时间锚与结果门的纯离线回归。
#
# build_path只生成数值轨迹；_send_goal/capture通过假ROS对象和monotonic_ns跳时推进，未真实等待600s或发送目标。当前候选/独立long policy从已有文件读，不等新HIL或实机许可。位置rad、动态rad/s与rad/s²，轨迹时间ns、报告µs。

import importlib.util
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "ros_six_axis_long_goal.py"
SPEC = importlib.util.spec_from_file_location("ros_six_axis_long_goal", SCRIPT)
long_goal = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(long_goal)
ROOT = Path(__file__).resolve().parents[3]
CANDIDATE = ROOT / "contracts/dummy_robot_v2_application_hold_no_load_candidate"


##
# @brief 返回六轴±0.6rad、速度0.15rad/s、加速度1rad/s²的合成边界。
# @return 六份(lower,upper,vmax,amax)元组，单位rad/rad/s/rad/s²。
def limits():
    return tuple((-0.6, 0.6, 0.15, 1.0) for _ in range(6))


##
# @brief 动态加载邻近converter核validate_bound可调用，不执行转换/ROS。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_neighboring_converter_loads_from_real_client_entry():
    module = long_goal._load_neighbor("convert_application_ros_limits")
    assert callable(module.validate_bound)


##
# @brief 人工时间accepted后600.1s成功可成报告，599.9s拒绝；actual字样是报告时间角色，不是真时长实验。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_success_report_binds_actual_600_second_action_and_joiner_fields():
    path = long_goal.build_path((0.0,) * 6, limits())
    result = SimpleNamespace(status=4, result=SimpleNamespace(error_code=0))
    start_ns = 123_000_000_000
    accepted_ns = start_ns + 300_000_000
    report = long_goal.success_report(path, "ab" * 32, "cd" * 32,
                                      "ef" * 16, result, start_ns, accepted_ns,
                                      accepted_ns + 600_100_000_000, [{"elapsed_us": 1}])
    assert report["action_started_steady_us"] == start_ns // 1000
    assert report["action_accepted_steady_us"] == accepted_ns // 1000
    assert report["action_elapsed_us"] == 600_100_000
    assert report["target_rad"] == list(path.points[-1].positions)
    assert report["status"] == "SUCCEEDED"
    with pytest.raises(RuntimeError, match="before 600s"):
        long_goal.success_report(path, "ab" * 32, "cd" * 32,
                                 "ef" * 16, result, start_ns, accepted_ns,
                                 accepted_ns + 599_900_000_000, [])


##
# @brief 起发后5s才接纳，不能把admission等待计入所需600s运行跨度。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_delayed_goal_admission_does_not_count_toward_600_seconds():
    path = long_goal.build_path((0.0,) * 6, limits())
    result = SimpleNamespace(status=4, result=SimpleNamespace(error_code=0))
    started_ns = 123_000_000_000
    accepted_ns = started_ns + 5_000_000_000
    with pytest.raises(RuntimeError, match="before 600s"):
        long_goal.success_report(path, "ab" * 32, "cd" * 32,
                                 "ef" * 16, result, started_ns, accepted_ns,
                                 started_ns + 600_100_000_000, [])


##
# @brief 数值路径301点/2s间隔、端点静止/回起点、六轴均变化、总行程≤10rad及有限动态界，不执行路径。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_600_second_goal_has_bounded_knots_and_stationary_end():
    path = long_goal.build_path((0.0,) * 6, limits())
    assert path.joint_names == tuple(f"Joint{axis}" for axis in range(1, 7))
    assert len(path.points) == 301
    assert [point.time_ns for point in path.points] == list(range(0, 600_000_000_001, 2_000_000_000))
    assert path.points[0].positions == path.points[-1].positions == (0.0,) * 6
    assert path.points[0].velocities == path.points[-1].velocities == (0.0,) * 6
    assert path.points[0].accelerations == path.points[-1].accelerations == (0.0,) * 6
    assert all(any(abs(point.positions[axis]) > 0 for point in path.points)
               for axis in range(6))
    assert sum(sum(abs(current.positions[axis] - previous.positions[axis])
                   for previous, current in zip(path.points, path.points[1:]))
               for axis in range(6)) <= 10.0
    assert all(all(math.isfinite(value) for value in
                   (*point.positions, *point.velocities, *point.accelerations))
               for point in path.points)
    assert all(max(abs(point.velocities[axis]) for point in path.points) < 0.15
               and max(abs(point.accelerations[axis]) for point in path.points) < 1.0
               for axis in range(6))


##
# @brief 长度5、非有限/bool或0.7rad越BENCH界均构造前ValueError。
# @param start 长目标起点六轴rad，参数化非法形状/值。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("start", [
    (0.0,) * 5, (0.0,) * 5 + (float("nan"),),
    (0.0,) * 5 + (float("inf"),), (0.0,) * 5 + (True,),
    (0.0,) * 5 + (0.7,),
])
def test_bad_start_is_rejected(start):
    with pytest.raises(ValueError):
        long_goal.build_path(start, limits())


##
# @brief 轴数不足、速度能力过低或幅度0.04导致预算问题均拒，不能只按duration生成任意轨迹。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_bad_limits_and_travel_overflow_rejected():
    with pytest.raises(ValueError):
        long_goal.build_path((0.0,) * 6, limits()[:5])
    with pytest.raises(ValueError):
        long_goal.build_path((0.0,) * 6, tuple((-0.6, 0.6, 0.001, 1.0) for _ in range(6)))
    with pytest.raises(ValueError):
        long_goal.build_path((0.0,) * 6, limits(), amplitude_rad=0.04)


##
# @brief 读取既有普通10s policy并要求long timing拒绝，不改冻结候选。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_old_ten_second_candidate_cannot_pass_long_policy():
    import json
    policy = json.loads((CANDIDATE / "policy/application_hold_no_load.json").read_text())
    assert policy["timing"]["max_goal_duration_us"] == 10_000_000
    with pytest.raises(ValueError, match="long-goal timing"):
        long_goal.validate_long_policy(policy)


##
# @brief 独立dpsdk long policy可缺而skip；存在时规范policy SHA匹配，普通旧policy摘要不能代替。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_normal_bound_candidate_rejected_against_explicit_long_source():
    source = (Path(__file__).resolve().parents[4] / "dpsdk/configs" /
              "f405_6dof_application_long_goal_no_load_candidate.json")
    if not source.is_file():
        pytest.skip("separate dpsdk long policy source unavailable")
    policy = json.loads(source.read_text())
    canonical = json.dumps({key: value for key, value in policy.items() if key != "note"},
                           sort_keys=True, separators=(",", ":")).encode()
    expected = hashlib.sha256(canonical).hexdigest()
    assert long_goal.validate_long_policy_binding(policy, expected) == expected
    old_policy = json.loads((CANDIDATE / "policy/application_hold_no_load.json").read_text())
    old_canonical = json.dumps({key: value for key, value in old_policy.items()
                                if key != "note"}, sort_keys=True, separators=(",", ":")).encode()
    with pytest.raises(ValueError):
        long_goal.validate_long_policy_binding(policy, hashlib.sha256(old_canonical).hexdigest())


##
# @brief 合成BENCH/no-load/630s目标/650s permit/660s trial/30s session门，短目标/permit或10ms节奏拒绝。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_long_policy_requires_exact_hashed_timing_and_no_load():
    policy = {"reference_kind": "BENCH_SESSION", "no_load_bench": True,
              "gravity_loaded": False, "active_axes": list(long_goal.JOINTS),
              "limits": {"max_trial_travel_rad": 10.0},
              "timing": {"control_period_us": 20_000,
                         "max_goal_duration_us": 630_000_000,
                         "trial_max_duration_us": 660_000_000,
                         "motion_permit_us": 650_000_000,
                         "session_lease_us": 30_000_000}}
    long_goal.validate_long_policy(policy)
    for field, invalid in (("max_goal_duration_us", 10_000_000),
                           ("motion_permit_us", 20_000_000),
                           ("control_period_us", 10_000)):
        changed = {**policy, "timing": {**policy["timing"], field: invalid}}
        with pytest.raises(ValueError, match="long-goal timing"):
            long_goal.validate_long_policy(changed)


##
# @brief 人工样本六轴顺序、≤年龄、安静速度及有限位置都须满足，不仅看数据数量。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_joint_state_requires_fresh_stationary_six_axes():
    sample = {"names": long_goal.JOINTS, "positions": (0.0,) * 6,
              "velocities": (0.0,) * 6, "age_s": 0.1}
    assert long_goal.validate_start_sample(sample, limits()) == (0.0,) * 6
    for changed in ({"names": tuple(reversed(long_goal.JOINTS))},
                    {"age_s": 1.1}, {"velocities": (0.02,) * 6},
                    {"positions": (float("nan"),) + (0.0,) * 5}):
        with pytest.raises(ValueError):
            long_goal.validate_start_sample({**sample, **changed}, limits())


##
# @brief 构造假Action client、ns时钟与plan/result桥，admission/running各独立跳时，feedback按开关人工回调。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param admission_ns fake发送到接受时钟增加ns，默认5s。
# @param running_ns fake接受到结果时钟增加ns，默认600.1s。
# @param status 人工Action状态码或bootstrap分类文本，按对应测试接口解释。
# @param error_code 模拟Action result错误码，0为成功。
# @param feedback_valid true完整六轴反馈，false仅五轴名。
# @param feedback_sent 是否同步注入反馈回调。
# @return (movej模块替身依赖已注入, fake Action client)。
def goal_entry(monkeypatch, *, admission_ns=5_000_000_000,
               running_ns=600_100_000_000, status=4, error_code=0,
               feedback_valid=True, feedback_sent=True):
    clock = [123_000_000_000]
    goal_id = bytes.fromhex("ef" * 16)
    movej = long_goal._load_neighbor("ros_six_axis_movej")
    movej.remaining_time = lambda deadline: 1.0
    movej.wait = lambda node, future, timeout: future

    ##
    # @par 内部回调 await_goal
    # 假等待把clock增加running_ns并返回指定status/error_code，不实际等待或轮询设备。
    # 参数 node：节点占位NS/object，相关ROS边界已替换。
    # 参数 handle：已接受/被测试的FakeHandle或NS，非真实设备owner。
    # 参数 timeout：本次允许等待秒数，替身不真实等待。
    # 返回：指定status/error_code的NS结果。
    def await_goal(node, handle, timeout):
        clock[0] += running_ns
        return SimpleNamespace(status=status, result=SimpleNamespace(error_code=error_code))

    movej.await_accepted_goal = await_goal
    movej.trajectory_digest = lambda trajectory: "ab" * 32
    monkeypatch.setattr(long_goal.time, "monotonic_ns", lambda: clock[0])
    monkeypatch.setattr(long_goal, "_ros_trajectory", lambda path: object())
    monkeypatch.setitem(sys.modules, "action_msgs.msg", SimpleNamespace(
        GoalStatus=SimpleNamespace(STATUS_SUCCEEDED=4)))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(
        FollowJointTrajectory=SimpleNamespace(Goal=SimpleNamespace)))
    monkeypatch.setitem(sys.modules, "rclpy.serialization", SimpleNamespace(
        serialize_message=lambda trajectory: b"trajectory"))

    ##
    # @par 内部回调 send_goal
    # 假发送先加admission_ns，按开关发六轴/五轴反馈，再返回固定accepted UUID句柄。
    # 参数 goal：Action Goal占位，携轨迹供断言，未真正发送。
    # 参数 feedback_callback：调用方反馈回调，替身在本地同步调用。
    # 返回：accepted=True且固定ef UUID的NS handle。
    def send_goal(goal, feedback_callback):
        clock[0] += admission_ns
        if feedback_sent:
            joints = long_goal.JOINTS if feedback_valid else long_goal.JOINTS[:5]
            sample = SimpleNamespace(joint_names=joints,
                                     actual=SimpleNamespace(positions=(0.0,) * 6),
                                     desired=SimpleNamespace(positions=(0.0,) * 6))
            feedback_callback(SimpleNamespace(feedback=sample))
        return SimpleNamespace(accepted=True, goal_id=SimpleNamespace(uuid=goal_id))

    client = SimpleNamespace(send_goal_async=send_goal)
    return movej, client


##
# @brief 真实_send_goal方法配假client，核5s准入不计运行600.1s；短599.9s失败保UUID。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_real_goal_entry_excludes_admission_delay_from_elapsed_time(monkeypatch):
    movej, client = goal_entry(monkeypatch)
    path = long_goal.build_path((0.0,) * 6, limits())
    report = long_goal._send_goal(None, client, None, path, 1.0, "cd" * 32, movej)
    assert report["action_elapsed_us"] == 600_100_000
    assert report["action_accepted_steady_us"] - report["action_started_steady_us"] == 5_000_000
    assert report["goal_id"] == "ef" * 16

    movej, client = goal_entry(monkeypatch, running_ns=599_900_000_000)
    with pytest.raises(movej.GoalAttemptError, match="before 600s") as error:
        long_goal._send_goal(None, client, None, path, 1.0, "cd" * 32, movej)
    assert error.value.goal_report["status"] == "FAILED"
    assert error.value.goal_report["goal_id"] == "ef" * 16


##
# @brief ABORTED/错误码/坏反馈/无反馈均GoalAttemptError，保UUID且device_qualification未声称。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param case 参数化故障选项字典/路径场景名，按本用例说明。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("case", [
    {"status": 6}, {"error_code": 1},
    {"feedback_valid": False}, {"feedback_sent": False},
])
def test_real_goal_entry_fails_closed_on_result_or_feedback(monkeypatch, case):
    movej, client = goal_entry(monkeypatch, **case)
    path = long_goal.build_path((0.0,) * 6, limits())
    with pytest.raises(movej.GoalAttemptError, match="feedback invalid") as error:
        long_goal._send_goal(None, client, None, path, 1.0, "cd" * 32, movej)
    assert error.value.goal_report["status"] == "FAILED"
    assert error.value.goal_report["device_qualification"] == "NOT_CLAIMED"
    assert error.value.goal_report["goal_id"] == "ef" * 16


##
# @brief fresh start失败时仍Release/destroy/shutdown，是否确认决定ConfirmedStopError或UnconfirmedStop，保原原因。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param release_confirmed 清理释放确认开关，决定异常类型。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("release_confirmed", [True, False])
def test_capture_failure_requires_confirmed_release(monkeypatch, release_confirmed):
    events = []
    movej = long_goal._load_neighbor("ros_six_axis_movej")
    node = SimpleNamespace(destroy_node=lambda: events.append("destroy"))
    rclpy = SimpleNamespace(init=lambda: events.append("init"),
                            create_node=lambda name: node,
                            shutdown=lambda: events.append("shutdown"))
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.action", SimpleNamespace(ActionClient=lambda *args: None))
    monkeypatch.setitem(sys.modules, "control_msgs.action", SimpleNamespace(
        FollowJointTrajectory=object))
    monkeypatch.setattr(long_goal, "_load_neighbor", lambda name: movej)
    monkeypatch.setattr(movej, "RosControlAdapter", lambda node, deadline: object())
    monkeypatch.setattr(movej, "verify_live_gate", lambda *args: events.append("bind"))
    monkeypatch.setattr(movej, "verify_active_stack", lambda adapter: events.append("stack"))
    monkeypatch.setattr(long_goal, "_fresh_start", lambda *args: (_ for _ in ()).throw(
        RuntimeError("joint state unavailable")))

    ##
    # @par 内部回调 release
    # 记录release；开关false抛UnconfirmedStop，true返回空活跃/未知列表，未调用服务。
    # 参数 adapter：生命周期/服务替身，具体事件和状态由用例注入。
    # 参数 owned：本次启动的组件清单，仅其有撤活责任。
    # 返回：人工清理结果或抛UnconfirmedStopError，不发Release。
    def release(adapter, owned):
        events.append("release")
        if not release_confirmed:
            raise movej.UnconfirmedStopError("UNCONFIRMED_STOP: release not observed")
        return [], []

    monkeypatch.setattr(movej, "recover_after_failure", release)
    expected = movej.ConfirmedStopError if release_confirmed else movej.UnconfirmedStopError
    with pytest.raises(expected, match="joint state unavailable"):
        long_goal.capture(limits(), "manifest", "ab" * 32, "ef" * 16, "cd" * 32)
    assert events == ["init", "bind", "stack", "release", "destroy", "shutdown"]


##
# @brief 即bound对象伪装650s permit，真实旧policy仍在long timing门拒绝，不能靠上层声明绕过。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_old_ten_second_bundle_rejected_by_real_candidate_loader(monkeypatch):
    old_policy = CANDIDATE / "policy/application_hold_no_load.json"
    converter = long_goal._load_neighbor("convert_application_ros_limits")
    monkeypatch.setattr(converter, "validate_bound", lambda limits_path, manifest_path:
                        SimpleNamespace(limits={"policy_sha256": "ab" * 32},
                                        gate_lease=SimpleNamespace(
                                            permit_remaining_us=650_000_000,
                                            requested_session_lease_us=30_000_000)))
    monkeypatch.setattr(long_goal, "_load_neighbor", lambda name: converter)
    with pytest.raises(ValueError, match="long-goal timing"):
        long_goal.load_bound_candidate("limits.json", CANDIDATE / "manifest.json", old_policy)
