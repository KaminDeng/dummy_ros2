#!/usr/bin/env python3
##
# @file validate_p9_long_goal.py
# @brief 离线复核单个六轴ROS长目标的已拼接原始证据。
#
# 先由邻近strict joiner从raw/timeline/action/context重建并比对，再核600s持续运动、六轴反馈幅度和间隔。
# 正结论仅GENERIC_SIX_AXIS_LONG_MOTION，轨迹形状NOT_VERIFIED、qualification NOT_QUALIFIED、硬件资格NOT_ASSERTED；不能因P9证据通过签发许可或声称特定轨迹形状完成。
"""Offline P9 single-goal duration check over strict joined ROS/raw evidence."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import re


##
# @brief 最少600,000,000µs；按20s分30窗，raw最大间隙250ms，ROS/Gate最大间隙2s。
# 每窗desired/raw六轴四分位跨度至少0.002rad、actual至少0.001rad；有限样本门槛不是连续运动学/最坏延迟证明。
MIN_DURATION_US = 600_000_000
WINDOW_US = 20_000_000
MAX_RAW_GAP_US = 250_000
MAX_FEEDBACK_GAP_US = 2_000_000
MIN_AXIS_RANGE_RAD = 0.002
MIN_ACTUAL_RANGE_RAD = 0.001
SHA256 = re.compile(r"[0-9a-f]{64}\Z")


##
# @brief 从同目录显式加载strict evidence joiner模块。
# @return 已执行模块对象，提供_snapshot/join_evidence/_raw_records；导入错误传播。
# 动态exec_module会执行邻近模块顶层代码，因此调用者依赖本地源码版本；不启动运动或RPC。
def _joiner():
    path = Path(__file__).with_name("join_real_6dof_evidence.py")
    spec = importlib.util.spec_from_file_location("join_real_6dof_evidence", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


##
# @brief 验证SHA文本形状。
# @param value 待核字符串。
# @param name 报错字段标签。
# @return 64字符小写hex原值；本函数允许全零且不计算源文件摘要。
def _hash(value, name):
    if not isinstance(value, str) or not SHA256.fullmatch(value):
        raise ValueError(f"missing or invalid {name}")
    return value


##
# @brief 要求一个完整有限六轴位置列表。
# @param sample 含字段的映射。
# @param name 取值字段名，如actual_rad/desired_rad/position_rad。
# @return 原六元素list，单位rad；bool、非数值、缺失或NaN/Infinity拒绝。
def _six_positions(sample, name):
    values = sample.get(name) if isinstance(sample, dict) else None
    if (not isinstance(values, list) or len(values) != 6 or
            any(type(value) not in (int, float) or not math.isfinite(value)
                for value in values)):
        raise ValueError(f"invalid six-axis {name}")
    return values


##
# @brief 把前600s分30个半开20s窗并计算每轴稳健跨度。
# @param samples (Host关联时刻µs,六轴rad)序列。
# @param start_us 活动起点Host steady µs。
# @return 30×6跨度列表rad；任一窗少于10样本抛ValueError。
# 跨度取排序后的约上四分位减下四分位，减少单个尖峰伪造持续运动；不是最大−最小，也不验证窗内连续路径形状。
def _window_ranges(samples, start_us):
    ranges = []
    for index in range(MIN_DURATION_US // WINDOW_US):
        start = start_us + index * WINDOW_US
        end = start + WINDOW_US
        positions = [values for when, values in samples if start <= when < end]
        if len(positions) < 10:
            raise ValueError("missing motion observations in 20s window")
        axis_ranges = []
        for axis in range(6):
            ordered = sorted(row[axis] for row in positions)
            quartile = len(ordered) // 4
            axis_ranges.append(ordered[-quartile - 1] - ordered[quartile])
        ranges.append(axis_ranges)
    return ranges


##
# @brief 提取目标活动前600s内同boot/session的raw State运动样本。
# @param records strict joiner解码的原始记录，预期已有顺序/身份检查。
# @param goal 关联目标的boot/session/motion身份。
# @param active_start after_begin的Host steady µs。
# @return start/end/max_gap/window_ranges摘要；缺样本或六轴/profile/motion不符拒绝。
# 只看kind2，要求mask63/profile201；把起止边界也纳入max_gap。函数没有自行排序records，顺序保证来自前置joiner。
def _raw_motion(records, goal, active_start):
    deadline = active_start + MIN_DURATION_US
    samples = []
    for item in records:
        if (item["kind"] != 2 or item["boot"] != goal["boot_epoch"] or
                item["session"] != goal["session_id"] or
                not active_start <= item["when"] <= deadline):
            continue
        fields = item["fields"]
        if (fields["position_valid_mask"] != 63 or fields["configured_mask"] != 63 or
                fields["profile_id"] != 201 or
                fields["motion_epoch"] != goal["motion_epoch"]):
            raise ValueError("unlinked six-axis raw motion observation")
        samples.append((item["when"], _six_positions(fields, "position_rad")))
    if not samples:
        raise ValueError("missing raw motion observations")
    times = [active_start, *(when for when, _ in samples), deadline]
    return {"start_us": active_start, "end_us": deadline,
            "max_gap_us": max(right - left for left, right in zip(times, times[1:])),
            "window_ranges_rad": _window_ranges(samples, active_start)}


##
# @brief 在严格join之后核P9时长、关联Hold/Release及持续反馈。
# @param evidence 已严格拼接的证据映射，单goal、profile201/mask63。
# @param action ROS_ACTION_ONLY且SUCCEEDED的action报告。
# @param policy_sha256 调用者期望策略SHA，与action字段比较。
# @param firmware_sha256 期望固件SHA，与joined identity比较。
# @param raw_terminal_us 唯一raw motion final6的Host记录时间µs，不能省略。
# @param raw_motion 前置_raw_motion摘要，覆盖after_begin后完整600s。
# @return 正证据范围报告，仍NOT_QUALIFIED/NOT_ASSERTED且trajectory_shape=NOT_VERIFIED；失败ValueError。
# 此底层入口信任前置join已核原始链/来源；直接传手写evidence并不等价于verify_paths的严格文件重放。
# 分别要求Gate accepted→terminal、after_begin→terminal、after_begin→raw terminal均至少600s，raw terminal不得晚于Gate terminal。
def validate_long_goal(evidence, action, *, policy_sha256, firmware_sha256,
                       raw_terminal_us=None, raw_motion=None):
    """Check P9-specific facts after the strict joiner has validated the capture."""
    _hash(policy_sha256, "expected policy SHA-256")
    _hash(firmware_sha256, "expected firmware SHA-256")
    if not isinstance(evidence, dict) or not isinstance(action, dict):
        raise ValueError("structured joined evidence and action report required")
    if (evidence.get("source") != "ros_moveit_jtc_usb_ref_can" or
            evidence.get("profile_id") != 201 or evidence.get("configured_mask") != 63 or
            action.get("scope") != "ROS_ACTION_ONLY" or
            action.get("status") != "SUCCEEDED"):
        raise ValueError("unsupported P9 single six-axis ROS action scope")
    provenance = evidence.get("provenance")
    if not isinstance(provenance, dict):
        raise ValueError("missing capture provenance")
    for name in ("raw_sha256", "timeline_sha256", "action_sha256", "context_sha256"):
        _hash(provenance.get(name), name)
    identity = evidence.get("identity")
    if not isinstance(identity, dict) or _hash(identity.get("firmware_hash"), "firmware_hash") != firmware_sha256:
        raise ValueError("firmware identity mismatch")
    if _hash(action.get("policy_sha256"), "policy_sha256") != policy_sha256:
        raise ValueError("long-goal policy identity mismatch")
    goals = evidence.get("goals")
    if not isinstance(goals, list) or len(goals) != 1:
        raise ValueError("exactly one joined ROS goal required")
    goal = goals[0]
    rows = goal.get("timeline") if isinstance(goal, dict) else None
    if (goal.get("action_status") != "SUCCEEDED" or
            goal.get("terminal") != "MOTION_COMPLETED_HOLDING" or
            goal.get("hold_confirmed") is not True or
            not isinstance(rows, list) or len(rows) < 7 or
            [row.get("event") for row in rows[:3]] !=
            ["accepted", "before_begin", "after_begin"] or
            [row.get("event") for row in rows[-3:]] !=
            ["terminal_success", "pre_release_hold", "post_release_status"]):
        raise ValueError("missing joined motion terminal or Hold chain")
    ##
    # @brief motion groups必须全六轴SENT且committed；pre_release_hold关联同run，独立release以parent_run关联并至少两页。
    # MotionCompletedHolding不等于release，二者证据分别核，不能用最终已清owner补出先前运动完成。
    groups = goal.get("groups")
    if (not isinstance(groups, list) or not groups or
            any(group.get("axis_mask") != 63 or group.get("sent_mask") != 63 or
                group.get("committed") is not True for group in groups)):
        raise ValueError("missing committed six-axis motion evidence")
    hold, release = evidence.get("pre_release_hold"), evidence.get("release")
    if (not isinstance(hold, dict) or hold.get("confirmed") is not True or
            hold.get("run_id") != goal.get("run_id") or
            not isinstance(release, dict) or
            release.get("terminal") not in ("RELEASED", "CANCELED_AND_RELEASED") or
            release.get("parent_run_id") != goal.get("run_id") or
            not isinstance(release.get("evidence_pages"), list) or
            len(release["evidence_pages"]) < 2):
        raise ValueError("missing confirmed Hold/Release evidence chain")
    accepted = rows[0].get("steady_us")
    active_start = rows[2].get("steady_us")
    terminal = rows[-3].get("steady_us")
    action_accepted = action.get("action_accepted_steady_us")
    if (any(type(value) is not int or value <= 0 for value in
            (accepted, active_start, terminal, action_accepted)) or
            action_accepted > accepted or active_start < accepted or terminal < active_start):
        raise ValueError("invalid action/Gate accepted-to-terminal clock binding")
    duration = terminal - accepted
    if duration < MIN_DURATION_US:
        raise ValueError("Gate accepted-to-terminal below 600000000 us")
    if terminal - active_start < MIN_DURATION_US:
        raise ValueError("Gate active motion below 600000000 us from after_begin")
    if (type(raw_terminal_us) is not int or raw_terminal_us > terminal or
            raw_terminal_us - active_start < MIN_DURATION_US):
        raise ValueError("raw motion terminal below 600000000 us from after_begin")
    if (action.get("duration_s") != 600 or type(action.get("planned_points")) is not int or
            action["planned_points"] != 301 or
            type(action.get("action_elapsed_us")) is not int or
            action["action_elapsed_us"] < MIN_DURATION_US or
            type(action.get("total_travel_rad")) not in (int, float) or
            not math.isfinite(action["total_travel_rad"]) or
            action["total_travel_rad"] < 3.0):
        raise ValueError("unbound P9 600s/301-point six-axis action shape")
    feedback = action.get("feedback")
    started = action.get("action_started_steady_us")
    if not isinstance(feedback, list) or type(started) is not int or started <= 0:
        raise ValueError("missing long-goal ROS feedback")
    tolerance = _six_positions(goal, "path_tolerance_rad")
    if any(value <= 0 for value in tolerance):
        raise ValueError("invalid bounded six-axis path tolerance")
    ##
    # @brief 每轴追踪容差取目标path_tolerance与0.005rad较小者，要求actual/desired每样本差不超界。
    # desired与actual分别统计20s窗运动跨度，避免把静止实际值或只变化的目标当完整长运动证据。
    tolerance = [min(value, 0.005) for value in tolerance]
    ros_samples = []
    actual_samples = []
    for item in feedback:
        elapsed = item.get("elapsed_us") if isinstance(item, dict) else None
        if type(elapsed) is not int or elapsed < 0:
            raise ValueError("invalid ROS feedback clock")
        when = started + elapsed
        if active_start <= when <= terminal:
            actual = _six_positions(item, "actual_rad")
            desired = _six_positions(item, "desired_rad")
            if any(abs(a - d) > limit for a, d, limit in
                   zip(actual, desired, tolerance)):
                raise ValueError("ROS actual feedback diverges from commanded path")
            ros_samples.append((when, desired))
            actual_samples.append((when, actual))
    ros_times = [active_start, *(when for when, _ in ros_samples), terminal]
    if (len(ros_samples) < 300 or
            any(right <= left or right - left > MAX_FEEDBACK_GAP_US
                for left, right in zip(ros_times, ros_times[1:])) or
            any(axis_range < MIN_AXIS_RANGE_RAD for ranges in
                _window_ranges(ros_samples, active_start) for axis_range in ranges) or
            any(axis_range < MIN_ACTUAL_RANGE_RAD for ranges in
                _window_ranges(actual_samples, active_start) for axis_range in ranges)):
        raise ValueError("sparse or static six-axis ROS feedback over 600s")
    ##
    # @brief Gate反馈时间与ROS反馈分别核严格前进及最大2s间隙；raw另核250ms和30窗六轴跨度。
    # 这些是已有记录的离线稠密度约束，不把接收时间当设备Source采样时刻。
    gate_times = [row.get("steady_us") for row in rows if row.get("event") == "feedback" and
                  row.get("steady_us", -1) >= active_start]
    if (any(type(when) is not int for when in gate_times) or
            any(right <= left or right - left > MAX_FEEDBACK_GAP_US
                for left, right in zip([active_start, *gate_times, terminal],
                                       [*gate_times, terminal]))):
        raise ValueError("sparse Gate feedback over 600s")
    if (not isinstance(raw_motion, dict) or raw_motion.get("start_us") != active_start or
            raw_motion.get("end_us") != active_start + MIN_DURATION_US or
            type(raw_motion.get("max_gap_us")) is not int or
            raw_motion["max_gap_us"] > MAX_RAW_GAP_US or
            not isinstance(raw_motion.get("window_ranges_rad"), list) or
            len(raw_motion["window_ranges_rad"]) != MIN_DURATION_US // WINDOW_US or
            any(not isinstance(ranges, list) or len(ranges) != 6 or
                any(type(value) not in (int, float) or not math.isfinite(value) or
                    value < MIN_AXIS_RANGE_RAD for value in ranges)
                for ranges in raw_motion["window_ranges_rad"])):
        raise ValueError("missing continuous six-axis raw motion over 600s")
    return {"status": "ROS_SINGLE_GOAL_600S_MOTION_EVIDENCE_POSITIVE",
            "evidence_scope": "GENERIC_SIX_AXIS_LONG_MOTION",
            "trajectory_shape": "NOT_VERIFIED",
            "qualification_scope": "NOT_QUALIFIED", "hardware_qualification": "NOT_ASSERTED",
            "accepted_to_terminal_us": duration,
            "active_to_terminal_us": terminal - active_start,
            "active_to_raw_terminal_us": raw_terminal_us - active_start,
            "raw_terminal_us": raw_terminal_us,
            "policy_sha256": policy_sha256, "firmware_sha256": firmware_sha256,
            "raw_sha256": provenance["raw_sha256"]}


##
# @brief 从五份输入文件重放strict join，再计算P9专属事实。
# @param joined 待核joined JSON路径。
# @param timeline Gate timeline路径。
# @param raw 原始采集路径。
# @param action ROS action结果路径。
# @param context 捕获上下文路径。
# @param policy_sha256 期望策略SHA。
# @param firmware_sha256 期望固件SHA。
# @return validate_long_goal的范围受限报告；不匹配、非唯一终页或文件/解析错误失败。
# joined对象必须与joiner重新生成结果完全相等；随后重新快照action/raw并核与provenance摘要一致，避免只信joined自报字段。
# 唯一终页筛选kind1/run/category4/final6；其它强身份/链关系由strict joiner负责。
def verify_paths(*, joined, timeline, raw, action, context, policy_sha256,
                 firmware_sha256):
    joiner = _joiner()
    joined_bytes, _ = joiner._snapshot(Path(joined))
    evidence = json.loads(joined_bytes)
    rebuilt = joiner.join_evidence(timeline, raw, action, context)
    if evidence != rebuilt:
        raise ValueError("joined evidence differs from strict SHA-pinned raw replay")
    action_bytes, action_hash = joiner._snapshot(Path(action))
    if action_hash != evidence["provenance"]["action_sha256"]:
        raise ValueError("action SHA-256 mismatch")
    raw_bytes, raw_hash = joiner._snapshot(Path(raw))
    if raw_hash != evidence["provenance"]["raw_sha256"]:
        raise ValueError("raw SHA-256 mismatch")
    records = joiner._raw_records(raw_bytes)
    goal = evidence["goals"][0]
    terminals = [item["when"] for item in records if item["kind"] == 1 and
                 item["run"] == goal["run_id"] and
                 item["fields"]["category"] == 4 and
                 item["fields"]["final_state"] == 6]
    if len(terminals) != 1:
        raise ValueError("missing unique raw motion terminal")
    raw_motion = _raw_motion(records, goal, goal["timeline"][2]["steady_us"])
    return validate_long_goal(evidence, json.loads(action_bytes),
                              policy_sha256=policy_sha256,
                              firmware_sha256=firmware_sha256,
                              raw_terminal_us=terminals[0], raw_motion=raw_motion)


##
# @brief 对既有证据文件运行离线P9校验并保存报告。
# @return 正证据0；捕获输入错误1并输出NOT_QUALIFIED；报告写入异常仍可传播。
# 无服务客户端/设备操作；--policy-sha256与--firmware-sha256是外部期望输入，脚本不从当前硬件读取它们。
def main():
    parser = argparse.ArgumentParser(description="P9 单次六轴 ROS 600s 原始证据门禁")
    for name in ("joined", "timeline", "raw", "action", "context", "report"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--policy-sha256", required=True)
    parser.add_argument("--firmware-sha256", required=True)
    args = parser.parse_args()
    try:
        result = verify_paths(joined=args.joined, timeline=args.timeline, raw=args.raw,
                              action=args.action, context=args.context,
                              policy_sha256=args.policy_sha256,
                              firmware_sha256=args.firmware_sha256)
        code = 0
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
        result = {"status": "NOT_QUALIFIED", "qualification_scope": "NOT_QUALIFIED",
                  "hardware_qualification": "NOT_ASSERTED", "reason": str(error)}
        code = 1
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
