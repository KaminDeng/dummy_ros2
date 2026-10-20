"""P9 long-goal qualification stays tied to strict raw ROS evidence."""
##
# @file test_validate_p9_long_goal.py
# @brief P9单目标600秒证据validator的合成时序/覆盖与strict join回归。
#
# joined/action/witness均人工字典，假raw_hex00/11不是合法设备页链；这些直接调用只测P9额外门，完整raw重放由verify_paths用例另测。600s跨度/30窗口及30001个State是数值数据，不真实运行600s。正例始终NOT_QUALIFIED、轨迹形状NOT_VERIFIED，不升级HIL/机械资格。

import copy
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("validate_p9_long_goal", ROOT / "validate_p9_long_goal.py")
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)

POLICY = "aa" * 32
FIRMWARE = "bb" * 32


##
# @brief 构造600s目标timeline、六轴组声明和四文件摘要占位，Release只有两个假hex页，不能独立算strict raw成功。
# @return 人工joined字典，raw页为占位而非完整wire证明。
def joined():
    feedback_rows = [{"event": "feedback", "steady_us": 1000 + second * 1_000_000}
                     for second in range(1, 600)]
    return {
        "source": "ros_moveit_jtc_usb_ref_can", "profile_id": 201,
        "configured_mask": 63, "identity": {"firmware_hash": FIRMWARE},
        "provenance": {"raw_sha256": "cc" * 32,
                       "timeline_sha256": "dd" * 32,
                       "action_sha256": "ee" * 32,
                       "context_sha256": "ff" * 32},
        "goals": [{"action_status": "SUCCEEDED", "terminal": "MOTION_COMPLETED_HOLDING",
                   "hold_confirmed": True, "run_id": 4,
                   "path_tolerance_rad": [0.003] * 6,
                   "groups": [{"axis_mask": 63, "sent_mask": 63, "committed": True}],
                   "timeline": [{"event": "accepted", "steady_us": 1000},
                                {"event": "before_begin", "steady_us": 1100},
                                {"event": "after_begin", "steady_us": 1200},
                                *feedback_rows,
                                {"event": "terminal_success", "steady_us": 600_001_200},
                                {"event": "pre_release_hold", "steady_us": 600_002_000},
                                {"event": "post_release_status", "steady_us": 600_003_000}]}],
        "pre_release_hold": {"confirmed": True, "run_id": 4},
        "release": {"terminal": "RELEASED", "parent_run_id": 4,
                    "evidence_pages": [{"raw_hex": "00"}, {"raw_hex": "11"}]},
    }


##
# @brief 生成599个1s间隔三角期望/actual反馈及600s/301点/行程3.6rad自声明报告，不发Action。
# @return 人工单目标Action报告。
def action():
    return {"scope": "ROS_ACTION_ONLY", "status": "SUCCEEDED",
            "policy_sha256": POLICY, "duration_s": 600,
            "planned_points": 301, "total_travel_rad": 3.6,
            "action_started_steady_us": 900,
            "action_accepted_steady_us": 1000, "action_elapsed_us": 600_000_000,
            "feedback": [{"elapsed_us": second * 1_000_000 + 100,
                          "desired_rad": [0.01 * min(second % 20, 20 - second % 20) / 10] * 6,
                          "actual_rad": [0.01 * min(second % 20, 20 - second % 20) / 10] * 6}
                         for second in range(1, 600)]}


##
# @brief 给P9额外门提供人工20ms最大间隙及30个20s窗各轴0.01rad范围，不是直接raw解析。
# @return 人工六轴600s窗口范围见证字典。
def witness():
    return {"start_us": 1200, "end_us": 600_001_200,
            "max_gap_us": 20_000,
            "window_ranges_rad": [[0.01] * 6 for _ in range(30)]}


##
# @brief 默认合成joined/action并传固定policy/firmware、raw终时与witness调用validator；空dict会因or回退，负例须按实际输入解读。
# @param evidence 合成joined长目标字典或None默认夹具。
# @param report 合成Action长目标报告或None默认夹具。
# @return P9额外validator结果；缺strict文件重放时不单独证明完整采集。
def check(evidence=None, report=None):
    return validator.validate_long_goal(evidence or joined(), report or action(),
                                        policy_sha256=POLICY, firmware_sha256=FIRMWARE,
                                        raw_terminal_us=600_001_200,
                                        raw_motion=witness())


##
# @brief 精确600s活跃/原raw跨度只得到generic long motion positive，资格未通过且轨迹形状未验证。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_accepts_exact_600_second_gate_span_only_as_unqualified_evidence():
    result = check()
    assert result["status"] == "ROS_SINGLE_GOAL_600S_MOTION_EVIDENCE_POSITIVE"
    assert result["evidence_scope"] == "GENERIC_SIX_AXIS_LONG_MOTION"
    assert result["trajectory_shape"] == "NOT_VERIFIED"
    assert result["qualification_scope"] == "NOT_QUALIFIED"
    assert result["accepted_to_terminal_us"] == 600_000_200
    assert result["active_to_terminal_us"] == 600_000_000
    assert result["active_to_raw_terminal_us"] == 600_000_000


##
# @brief 即Action夸duration/elapsed，Gate活跃跨度短201µs仍拒，不能由报告自声明补足。
# @param change 参数化变异回调，对当前独立字典修改。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("change", [
    lambda e, a: a.update(duration_s=9999, action_elapsed_us=999999999),
    lambda e, a: e["goals"][0]["timeline"][-3].update(steady_us=600_000_999),
])
def test_action_claim_cannot_hide_short_gate_span(change):
    evidence, report = joined(), action()
    evidence["goals"][0]["timeline"][-3]["steady_us"] = 600_000_999
    change(evidence, report)
    with pytest.raises(ValueError, match="600000000"):
        check(evidence, report)


##
# @brief provenance四输入摘要缺任一即拒，未在本直接调用中重读文件。
# @param field 被变异的轨迹语义字段。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("field", ["raw_sha256", "timeline_sha256", "action_sha256",
                                     "context_sha256"])
def test_missing_capture_digest_fails_closed(field):
    evidence = joined()
    del evidence["provenance"][field]
    with pytest.raises(ValueError, match=field):
        check(evidence)


##
# @brief 固件/policy错、scope会话化、多goal、无Hold/Release/groups等与单目标合同不符拒绝。
# @param mutation 冻结字节/inode/模式或probe健康字段变异值。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("mutation", [
    lambda e, a: e["identity"].update(firmware_hash="12" * 32),
    lambda e, a: a.update(policy_sha256="12" * 32),
    lambda e, a: a.update(scope="ROS_ACTION_SESSION_ONLY"),
    lambda e, a: e["goals"].append(copy.deepcopy(e["goals"][0])),
    lambda e, a: e["goals"][0].update(hold_confirmed=False),
    lambda e, a: e["release"].update(terminal="UNKNOWN"),
    lambda e, a: e["goals"][0].update(groups=[]),
])
def test_rejects_identity_scope_or_missing_chain(mutation):
    evidence, report = joined(), action()
    mutation(evidence, report)
    with pytest.raises(ValueError):
        check(evidence, report)


##
# @brief raw终时缺失或相对活跃起点不足600s都拒绝，不只看ROS终态。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_raw_terminal_duration_is_required():
    with pytest.raises(ValueError, match="raw motion terminal"):
        validator.validate_long_goal(joined(), action(), policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE, raw_motion=witness())
    with pytest.raises(ValueError, match="raw motion terminal"):
        validator.validate_long_goal(joined(), action(), policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE,
                                     raw_terminal_us=600_000_999, raw_motion=witness())


##
# @brief 确认活动CANCELED_AND_RELEASED终态也支持，但不放宽raw运动终时门。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_confirmed_active_release_terminal_is_supported():
    evidence = joined()
    evidence["release"]["terminal"] = "CANCELED_AND_RELEASED"
    assert check(evidence=evidence)["qualification_scope"] == "NOT_QUALIFIED"
    report = action()
    report["action_accepted_steady_us"] = 1
    with pytest.raises(ValueError, match="raw motion terminal"):
        validator.validate_long_goal(joined(), report, policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE,
                                     raw_terminal_us=600_000_999, raw_motion=witness())


##
# @brief 时长/点数/feedback密度、静止/单轴窗无运动、大间隙/Gate稀疏均拒绝；仅字段自声明不够。
# @param mutation 冻结字节/inode/模式或probe健康字段变异值。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize("mutation", [
    lambda e, a, w: a.update(duration_s=10),
    lambda e, a, w: a.update(planned_points=2),
    lambda e, a, w: a.update(action_elapsed_us=10_000_000),
    lambda e, a, w: a["feedback"].__setitem__(slice(10, -10), []),
    lambda e, a, w: [sample.update(desired_rad=[0.0] * 6,
                                   actual_rad=[0.0] * 6) for sample in a["feedback"]],
    lambda e, a, w: [sample.update(actual_rad=[0.0] * 6) for sample in a["feedback"]],
    lambda e, a, w: a["feedback"][200].update(actual_rad=[0.1] * 6),
    lambda e, a, w: w["window_ranges_rad"][15].__setitem__(2, 0.0),
    lambda e, a, w: w.update(max_gap_us=10_000_000),
    lambda e, a, w: e["goals"][0]["timeline"].__setitem__(slice(10, -10), []),
])
def test_rejects_delayed_completion_or_static_and_sparse_motion(mutation):
    evidence, report, raw_motion = joined(), action(), witness()
    mutation(evidence, report, raw_motion)
    with pytest.raises(ValueError):
        validator.validate_long_goal(evidence, report, policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE,
                                     raw_terminal_us=600_001_200,
                                     raw_motion=raw_motion)


##
# @brief accepted到terminal虽长，after_begin改晚使active不足600s仍拒绝。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_short_active_span_fails_even_when_acceptance_span_is_long():
    evidence = joined()
    evidence["goals"][0]["timeline"][2]["steady_us"] = 2000
    with pytest.raises(ValueError, match="active motion"):
        check(evidence=evidence)


##
# @brief 每20s只一个反馈脉冲不能算持续运动，即使witness每窗范围自报0.01仍拒绝。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_one_pulse_per_window_is_not_continuous_motion():
    evidence, report, raw_motion = joined(), action(), witness()
    for second, sample in enumerate(report["feedback"], 1):
        pulse = 0.01 if second % 20 == 10 else 0.0
        sample.update(desired_rad=[pulse] * 6, actual_rad=[pulse] * 6)
    raw_motion["window_ranges_rad"] = [[0.01] * 6 for _ in range(30)]
    with pytest.raises(ValueError, match="sparse or static"):
        validator.validate_long_goal(evidence, report, policy_sha256=POLICY,
                                     firmware_sha256=FIRMWARE,
                                     raw_terminal_us=600_001_200,
                                     raw_motion=raw_motion)


##
# @brief actual保留0.001rad量化可在误差范围内通过，不要求浮点逐字相等。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_quantized_actual_motion_is_accepted():
    report = action()
    for sample in report["feedback"]:
        sample["actual_rad"] = [round(value, 3) for value in sample["desired_rad"]]
    assert check(report=report)["qualification_scope"] == "NOT_QUALIFIED"


##
# @brief 人工30001 State每20ms核30窗运动范围；一窗全静止会得到零范围，混旧epoch必须拒unlinked。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_structured_raw_state_witness_covers_all_six_axes_for_600_seconds():
    records = [{"kind": 2, "boot": 2, "session": 3, "when": 1100,
                "fields": {"position_valid_mask": 63, "configured_mask": 63,
                           "profile_id": 201, "motion_epoch": 1,
                           "position_rad": [0.0] * 6}}]
    for tick in range(30_001):
        when = 1200 + tick * 20_000
        phase = (tick % 1000) / 1000
        position = 0.01 * (1 - abs(2 * phase - 1))
        records.append({"kind": 2, "boot": 2, "session": 3, "when": when,
                        "fields": {"position_valid_mask": 63, "configured_mask": 63,
                                   "profile_id": 201, "motion_epoch": 2,
                                   "position_rad": [position] * 6}})
    goal = {"boot_epoch": 2, "session_id": 3, "motion_epoch": 2}
    observed = validator._raw_motion(records, goal, 1200)
    assert observed["max_gap_us"] == 20_000
    assert len(observed["window_ranges_rad"]) == 30
    assert all(min(ranges) > 0.004 for ranges in observed["window_ranges_rad"])
    for item in records:
        if 1200 + 15 * 20_000_000 <= item["when"] < 1200 + 16 * 20_000_000:
            item["fields"] = {**item["fields"], "position_rad": [0.0] * 6}
    observed = validator._raw_motion(records, goal, 1200)
    assert observed["window_ranges_rad"][15] == [0.0] * 6
    records[100]["fields"] = {**records[100]["fields"], "motion_epoch": 1}
    with pytest.raises(ValueError, match="unlinked six-axis raw motion"):
        validator._raw_motion(records, goal, 1200)


##
# @brief 每1000采样只一个0.01rad脉冲，稳健窗口范围全归零，不能用极端值当连续运动。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_raw_one_sample_pulses_fail_robust_window_motion():
    records = []
    for tick in range(30_001):
        position = 0.01 if tick % 1000 == 500 else 0.0
        records.append({"kind": 2, "boot": 2, "session": 3,
                        "when": 1200 + tick * 20_000,
                        "fields": {"position_valid_mask": 63, "configured_mask": 63,
                                   "profile_id": 201, "motion_epoch": 2,
                                   "position_rad": [position] * 6}})
    goal = {"boot_epoch": 2, "session_id": 3, "motion_epoch": 2}
    observed = validator._raw_motion(records, goal, 1200)
    assert observed["window_ranges_rad"] == [[0.0] * 6 for _ in range(30)]


##
# @brief 复用已有join合成夹具实际离线重放，短时先因motion observations拒；再伪provenance须strict SHA重放拒，real命名不表示真实采集。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_cli_paths_replay_strict_join_and_reject_short_real_fixture(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "existing_join_tests", ROOT / "tests/test_join_real_6dof_evidence.py")
    fixtures = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixtures)
    timeline, raw, action_path, context, _ = fixtures.inputs.__wrapped__(tmp_path)
    report = json.loads(action_path.read_text())
    report.update(policy_sha256=POLICY, action_accepted_steady_us=1000)
    action_path.write_text(json.dumps(report))
    fixtures.repin(timeline, raw, action_path, context)
    evidence = fixtures.joiner.join_evidence(timeline, raw, action_path, context)
    joined_path = tmp_path / "joined.json"
    joined_path.write_text(json.dumps(evidence))
    with pytest.raises(ValueError, match="motion observations"):
        validator.verify_paths(joined=joined_path, timeline=timeline, raw=raw,
                               action=action_path, context=context,
                               policy_sha256=POLICY, firmware_sha256="11" * 32)
    evidence["provenance"]["raw_sha256"] = "00" * 32
    joined_path.write_text(json.dumps(evidence))
    with pytest.raises(ValueError, match="strict SHA-pinned raw replay"):
        validator.verify_paths(joined=joined_path, timeline=timeline, raw=raw,
                               action=action_path, context=context,
                               policy_sha256=POLICY, firmware_sha256="11" * 32)
