#!/usr/bin/env python3
##
# @file build_real_6dof_context.py
# @brief 从同一次Gate/raw/action捕获生成显式预算与SHA绑定上下文。
#
# 全程离线读取，不连接ROS/设备；输出新context是验证输入关系，不签名授权。
# producer_lag为Host接收−设备Source产生的偏移跨度max−min，消去固定时钟偏移，不是绝对单向通信延迟或跨设备同步保证。
"""从同一次 ROS 原始采集生成 join_real_6dof_evidence 所需的上下文。"""

import argparse
from bisect import bisect_right
import csv
import io
import hashlib
import json
import math
import os
import struct
from pathlib import Path
import tempfile

try:
    from . import join_real_6dof_evidence as joiner
except ImportError:
    import join_real_6dof_evidence as joiner


PRODUCER_LAG_METHOD = "state_receive_minus_produced_offset_span"
WINDOW_LAG_METHOD = PRODUCER_LAG_METHOD + "_motion_window"


##
# @brief 核取消过渡可计入观测覆盖的完整原页链与精确回执。
# @param records 已解码DPRAW记录。
# @param goal CSV目标行。
# @param request 唯一Cancel原请求记录。
# @param receipt 关联COMMITTED Cancel回执。
# @param stop_page 当前run新stop epoch的CAN页。
# @param client 唯一client hex身份。
# @return None；静态hash不唯一、链/全六轴SENT/干净取消终态不符拒绝。
# State没有run字段且事务号可滞后；只能用本run完整链补观测覆盖，不能把旧State operation号单独当Stop确认。
def _cancel_coverage_chain(records, goal, request, receipt, stop_page, client):
    """完整取消链只授权观测覆盖，State旧事务号不作为Stop确认。"""
    boot, session, run = int(goal["boot"]), int(goal["session"]), int(goal["run_id"])
    wanted, result = request["fields"], receipt["fields"]
    if (wanted["configured_mask"] != 63 or wanted["expected_profile_id"] <= 0 or
            wanted["expected_reference_epoch"] != int(goal["reference_epoch"]) or
            result["reference_epoch"] != wanted["expected_reference_epoch"] or
            result["profile_id"] != wanted["expected_profile_id"] or
            result["management_epoch"] != wanted["expected_management_epoch"] + 1 or
            result["first_failure_reason"] != 0):
        raise ValueError("unlinked cancel coverage receipt")
    digest_fields = ("contract_sha256", "model_sha256", "profile_sha256", "firmware_sha256")
    identities = {tuple(item["fields"][name] for name in digest_fields)
                  for item in records if item["kind"] == 3 and item["boot"] == boot and
                  item["fields"]["device_id"] == wanted["device_id"] and
                  item["fields"]["profile_id"] == wanted["expected_profile_id"] and
                  item["fields"]["configured_mask"] == wanted["configured_mask"]}
    if len(identities) != 1:
        raise ValueError("ambiguous cancel coverage static identity")
    hashes = tuple(bytes.fromhex(value) for value in next(iter(identities)))
    if any(len(value) != 32 or not any(value) for value in hashes):
        raise ValueError("invalid cancel coverage static identity")
    seed = struct.pack("<QQ", run, boot) + bytes.fromhex(client) + b"".join(hashes)
    previous = hashlib.sha256(seed.ljust(192, b"\0")).digest()
    pages = joiner._pages(records, run, boot, session, 1, 0)
    previous_count = 0
    for index, page in enumerate(pages):
        raw, fields = page["raw"], page["fields"]
        terminal = index == len(pages) - 1
        if (len(raw) != 372 or raw[371] != 0 or raw[307:339] != previous or
                raw[339:371] != hashlib.sha256(previous + raw[:307] + raw[371:372]).digest() or
                fields["evidence_seq"] != index or fields["group_count"] < previous_count or
                fields["group_count"] > len(pages) - 1 or fields["final"] != 1 or
                fields["first_failure_source"] != 0 or fields["first_failure_axis_mask"] != 0):
            raise ValueError("unverified cancel coverage chain")
        if not terminal and (fields["category"] not in (1, 2) or fields["group_id"] == 0 or
                fields["group_count"] < index + 1 or
                (fields["expected_mask"], fields["submitted_mask"], fields["sent_mask"],
                 fields["failed_mask"]) != (63, 63, 63, 0)):
            raise ValueError("unverified cancel coverage CAN SENT")
        previous, previous_count = raw[339:371], fields["group_count"]
    terminal = pages[-1]["fields"]
    if (stop_page not in pages or terminal["final_state"] != 7 or terminal["terminal_reason"] != 0 or
            terminal["motion_epoch"] != int(goal["motion_epoch"]) or
            terminal["motion_complete"] or terminal["release_committed"] or
            terminal["group_id"] != 0 or terminal["group_count"] != len(pages) - 1 or
            any(terminal[name] != 0 for name in
                ("expected_mask", "submitted_mask", "sent_mask", "failed_mask"))):
        raise ValueError("unlinked cancel coverage terminal")


##
# @brief 从首accepted到最终释放Idle限定同一次运动观测窗口。
# @param timeline_data CSV原字节，含Gate事件。
# @param records 同次raw解码记录。
# @param client 非零唯一管理client hex。
# @param state_max_age Host覆盖间隔/末尾Idle年龄上限µs。
# @return (start_us,end_us,窗口全部State,release_run)；缺唯一release、错身份/epoch/覆盖失败拒绝。
# 分别核Disable请求/回执/终页/新鲜Idle；取消段还核原Cancel请求、精确回执及完整原链。
# 双时钟不直接比较绝对值，覆盖检查用Host when；Source produced用于随后offset跨度。只允许受证据支持的同session释放过渡，不以位移猜Run。
def _motion_window(timeline_data, records, client, state_max_age):
    try:
        reader = csv.DictReader(io.StringIO(timeline_data.decode("utf-8")))
        rows = [row for row in reader if row.get("event") != "COMPLETE"]
        goals = [row for row in rows if row.get("event") == "accepted"]
        if not goals:
            raise ValueError("missing ROS accepted goal")
        final = rows[-1]
        if final.get("event") != "post_release_status":
            raise ValueError("missing final ROS Release observation")
        start = int(goals[0]["steady_us"])
        end = int(final["steady_us"])
        boot, session = int(goals[0]["boot"]), int(goals[0]["session"])
        motion_runs = [int(row["run_id"]) for row in goals]
        release_run = int(final["run_id"])
        request_id = int(final["release_request_id"])
        operation_id = int(final["release_operation_id"])
        goal_times = [int(row["steady_us"]) for row in goals]
        goal_phases = []
        for row in rows:
            if row["event"] == "accepted":
                goal_phases.append(set())
            if int(row["session"]) == session:
                goal_phases[-1].add((int(row["reference_epoch"]),
                                     int(row["motion_epoch"])))
        phase_markers = [(int(row["steady_us"]), int(row["motion_epoch"]))
                         for row in rows if row["event"] in
                         ("accepted", "after_begin", "terminal_success",
                          "terminal_canceled", "pre_release_hold",
                          "post_release_status")]
        release_phase = (int(final["reference_epoch"]), int(final["motion_epoch"]))
        releasing_phase = (int(goals[-1]["reference_epoch"]), release_phase[1])
    except (UnicodeDecodeError, TypeError, KeyError, IndexError, ValueError) as error:
        if isinstance(error, ValueError) and ("accepted" in str(error) or
                                              "Release" in str(error)):
            raise
        raise ValueError("invalid ROS motion window timeline") from error
    if (start <= 0 or end <= start or boot <= 0 or session <= 0 or
            release_run <= max(motion_runs) or request_id <= 0 or operation_id <= 0 or
            any(int(row["boot"]) != boot or int(row["session"]) != session or
                int(row["run_id"]) <= 0 for row in goals)):
        raise ValueError("ROS motion window identity or ordering mismatch")
    if any(later[0] < earlier[0] or later[1] < earlier[1] for earlier, later in
           zip(phase_markers, phase_markers[1:])):
        raise ValueError("ROS motion window timeline phase regression")
    requests = [item for item in records if item["kind"] == 4 and
                item["fields"]["operation"] == 10 and
                item["fields"]["request_id"] == request_id and
                item["fields"]["client_instance_id"] == client and
                (item["boot"], item["session"]) == (boot, session)]
    terminals = [item for item in records if item["kind"] == 1 and
                 item["fields"]["category"] == 4 and
                 item["fields"]["release_committed"] == 1 and
                 item["fields"]["final_state"] in (2, 8) and
                 (item["boot"], item["session"], item["run"]) ==
                 (boot, session, release_run)]
    queries = [item for item in records if item["kind"] == 4 and
               item["fields"]["operation"] == 12 and
               item["fields"]["queried_operation_id"] == request_id and
               item["fields"]["client_instance_id"] == client and
               (item["boot"], item["session"]) == (boot, session) and
               requests and item["when"] >= requests[0]["when"]]
    responses = [item for item in records if item["kind"] == 5 and
                 (item["fields"]["request_id"] == request_id or
                  any(query["fields"]["request_id"] == item["fields"]["request_id"] and
                      query["when"] <= item["when"] for query in queries)) and
                 item["fields"]["operation_id"] == operation_id and
                 item["fields"]["client_instance_id"] == client and
                 item["fields"]["transaction_phase"] == 3 and
                 item["fields"]["result_code"] == 0 and
                 item["fields"]["release_committed"] == 1 and
                 (item["boot"], item["session"], item["run"]) ==
                 (boot, 0, release_run)]
    if (len(requests) != 1 or len(terminals) != 1 or len(responses) != 1 or
            not start < requests[0]["when"] <= terminals[0]["when"] <= end or
            not requests[0]["when"] <= responses[0]["when"] <= end):
        raise ValueError("missing or unlinked raw Release proof")
    device_id = requests[0]["fields"]["device_id"]
    if device_id == "00" * 16:
        raise ValueError("motion window device identity missing")
    cancel_bounds = {}
    cancel_proofs = {}
    for goal_index, goal in enumerate(goals):
        goal_rows = rows[rows.index(goal):rows.index(goals[goal_index + 1])
                         if goal_index + 1 < len(goals) else len(rows)]
        canceled = [row for row in goal_rows if row["event"] == "terminal_canceled"]
        if not canceled:
            continue
        if len(canceled) != 1:
            raise ValueError("ambiguous ROS canceled goal phase")
        terminal = canceled[0]
        begins = [row for row in goal_rows if row["event"] == "after_begin"]
        identity_fields = ("goal_uuid", "trajectory_sha256", "boot", "session",
                           "reference_epoch", "run_id", "axis_mask")
        if (len(begins) != 1 or
                any(begins[0][field] != goal[field] for field in identity_fields) or
                not int(goal["steady_us"]) <= int(begins[0]["steady_us"]) <
                    int(terminal["steady_us"])):
            raise ValueError("missing or unlinked canceled goal after_begin")
        begin_time = int(begins[0]["steady_us"])
        old_epoch = int(begins[0]["motion_epoch"])
        if old_epoch < int(goal["motion_epoch"]):
            raise ValueError("canceled goal after_begin epoch regression")
        stop_epoch = int(terminal["motion_epoch"])
        request_id = int(terminal["cancel_request_id"])
        operation_id = int(terminal["cancel_operation_id"])
        terminal_time = int(terminal["steady_us"])
        cancel_requests = [item for item in records if item["kind"] == 4 and
                           item["fields"]["operation"] == 9 and
                           item["fields"]["request_id"] == request_id and
                           item["fields"]["client_instance_id"] == client and
                           item["fields"]["device_id"] == device_id and
                           item["fields"]["expected_motion_epoch"] == old_epoch and
                           (item["boot"], item["session"]) == (boot, session) and
                           begin_time <= item["when"] <= terminal_time]
        stop_pages = [item for item in records if item["kind"] == 1 and
                      (item["boot"], item["session"], item["run"]) ==
                      (boot, session, int(goal["run_id"])) and
                      (item["fields"]["attempt_id"], item["fields"]["attempt_kind"],
                       item["fields"]["category"], item["fields"]["motion_epoch"]) ==
                      (int(goal["run_id"]), 1, 2, stop_epoch) and
                      cancel_requests and
                      cancel_requests[0]["when"] <= item["when"] <= terminal_time]
        query_binding = ("expected_boot_epoch", "expected_session_id",
                         "expected_reference_epoch", "expected_motion_epoch",
                         "expected_management_epoch", "expected_profile_id",
                         "configured_mask")
        cancel_queries = [item for item in records if item["kind"] == 4 and
                          item["fields"]["operation"] == 12 and
                          item["fields"]["queried_operation_id"] == request_id and
                          item["fields"]["client_instance_id"] == client and
                          item["fields"]["device_id"] == device_id and
                          (item["boot"], item["session"]) == (boot, session) and
                          cancel_requests and
                          all(item["fields"][name] ==
                              cancel_requests[0]["fields"][name]
                              for name in query_binding) and
                          cancel_requests[0]["when"] <= item["when"] <= terminal_time]
        receipts = [item for item in records if item["kind"] == 5 and
                    (item["fields"]["request_id"] == request_id or
                     any(query["fields"]["request_id"] ==
                         item["fields"]["request_id"] and
                         query["when"] <= item["when"] for query in cancel_queries)) and
                    item["fields"]["operation_id"] == operation_id and
                    item["fields"]["client_instance_id"] == client and
                    item["fields"]["device_id"] == device_id and
                    item["fields"]["transaction_phase"] == 3 and
                    item["fields"]["result_code"] == 0 and
                    item["fields"]["receipt"] == 1 and
                    item["fields"]["motion_complete"] == 0 and
                    item["fields"]["release_committed"] == 0 and
                    item["fields"]["motion_epoch"] == stop_epoch and
                    (item["boot"], item["session"], item["run"]) ==
                    (boot, session, int(goal["run_id"])) and
                    goal_times[goal_index] <= item["when"] <= terminal_time]
        if (stop_epoch != old_epoch + 1 or request_id <= 0 or operation_id <= 0 or
                len(cancel_requests) != 1 or not stop_pages or len(receipts) != 1 or
                not cancel_requests[0]["when"] <= receipts[0]["when"]):
            raise ValueError("missing or unlinked raw cancel Stop CAN page")
        cancel_bounds[goal_index] = (cancel_requests[0]["when"],
                                     stop_pages[0]["when"], old_epoch, stop_epoch,
                                     begin_time, int(goal["motion_epoch"]))
        cancel_proofs[goal_index] = (goal, cancel_requests[0], receipts[0], stop_pages[0])
    idle_statuses = [item for item in records if item["kind"] == 3 and
                     (item["boot"], item["session"], item["fields"]["run_id"]) ==
                     (boot, 0, release_run) and
                     item["fields"]["device_id"] == device_id and
                     item["fields"]["core_state"] == 1 and
                     item["fields"]["release_committed"] == 1 and
                     item["fields"]["operation_id"] == responses[0]["fields"]["operation_id"] and
                     item["fields"]["final_state"] == terminals[0]["fields"]["final_state"] and
                     item["fields"]["permit_remaining_us"] == 0 and
                     (item["fields"]["reference_epoch"], item["fields"]["motion_epoch"]) == release_phase and
                     requests[0]["when"] <= item["when"] <= end and
                     end - item["when"] <= state_max_age]
    idle_states = [item for item in records if item["kind"] == 2 and
                   (item["boot"], item["session"]) == (boot, 0) and
                   item["fields"]["device_id"] == device_id and
                   (item["fields"]["reference_epoch"],
                    item["fields"]["motion_epoch"]) == release_phase and
                   item["fields"]["core_state"] == 1 and
                   requests[0]["when"] <= item["when"] <= end and
                   end - item["when"] <= state_max_age]
    if not idle_statuses or not idle_states:
        raise ValueError("missing fresh raw Release Idle observation")
    window_states = [item for item in records if item["kind"] == 2 and
                     start <= item["when"] <= end]
    if len(window_states) < 2:
        raise ValueError("insufficient motion window State samples")
    if any(item["boot"] != boot or item["session"] not in (session, 0) or
           item["fields"]["device_id"] != device_id or
           (item["session"] == 0 and
            ((item["fields"]["reference_epoch"],
              item["fields"]["motion_epoch"]) != release_phase or
             item["when"] < requests[0]["when"] or
             item["fields"]["core_state"] != 1)) or
           (item["session"] == session and
            ((item["fields"]["reference_epoch"],
              item["fields"]["motion_epoch"]) not in
             goal_phases[bisect_right(goal_times, item["when"]) - 1] and
             not (item["when"] >= requests[0]["when"] and
                  bisect_right(goal_times, item["when"]) == len(goals) and
                  (item["fields"]["reference_epoch"],
                   item["fields"]["motion_epoch"]) == releasing_phase) or
             item["when"] > terminals[0]["when"]))
           for item in window_states):
        raise ValueError("motion window State identity mismatch")
    coverage_states = []
    verified_cancel_coverage = set()
    for item in window_states:
        if item["session"] != session:
            coverage_states.append(item)
            continue
        goal_index = bisect_right(goal_times, item["when"]) - 1
        bound = cancel_bounds.get(goal_index)
        if bound is None:
            coverage_states.append(item)
            continue
        request_time, stop_time, old_epoch, stop_epoch, begin_time, accepted_epoch = bound
        epoch = item["fields"]["motion_epoch"]
        # 上游已核对唯一 Disable 的请求、回执、终链及 Idle。取消完成后
        # 同 session 的 Disabling 属于该释放事务，不再属于停止 epoch。
        if (goal_index == len(goals) - 1 and item["when"] >= requests[0]["when"] and
                item["fields"]["core_state"] in (7, 8) and
                item["fields"]["management_epoch"] == responses[0]["fields"]["management_epoch"] and
                item["fields"]["configured_mask"] == 63 and
                item["fields"]["profile_id"] == requests[0]["fields"]["expected_profile_id"] and
                (item["fields"]["reference_epoch"], epoch) == releasing_phase):
            coverage_states.append(item)
            continue
        if ((item["when"] < begin_time and epoch not in (accepted_epoch, old_epoch)) or
                (begin_time <= item["when"] < request_time and epoch != old_epoch) or
                (item["when"] > stop_time and epoch != stop_epoch)):
            raise ValueError("motion window State identity phase mismatch")
        if not request_time <= item["when"] <= stop_time:
            coverage_states.append(item)
        else:
            goal, request, receipt, stop_page = cancel_proofs[goal_index]
            fields = item["fields"]
            # State无run字段且事务号可旧；本run完整链和精确回执提供关联。
            eligible = (epoch == stop_epoch and
                        fields["reference_epoch"] == request["fields"]["expected_reference_epoch"] and
                        fields["profile_id"] == receipt["fields"]["profile_id"] and
                        fields["configured_mask"] == 63 and
                        fields["management_epoch"] == receipt["fields"]["management_epoch"] and
                        fields["core_state"] in (7, 4) and
                        (fields["core_state"] != 4 or fields["hold_ready"] == 1) and
                        all(fields[name] == 63 for name in
                            ("position_valid_mask", "velocity_valid_mask", "raw_position_valid_mask")))
            if eligible:
                if goal_index not in verified_cancel_coverage:
                    _cancel_coverage_chain(records, goal, request, receipt, stop_page, client)
                    verified_cancel_coverage.add(goal_index)
                coverage_states.append(item)
    marker_index = 0
    phase_floor = phase_markers[0][1]
    previous_epoch = phase_floor
    for item in window_states:
        while (marker_index + 1 < len(phase_markers) and
               phase_markers[marker_index + 1][0] <= item["when"]):
            marker_index += 1
            phase_floor = phase_markers[marker_index][1]
        epoch = item["fields"]["motion_epoch"]
        if epoch < phase_floor or epoch < previous_epoch:
            raise ValueError("motion window State identity phase regression")
        previous_epoch = epoch
    observations = [start, *(item["when"] for item in coverage_states), end]
    if any(later - earlier > state_max_age for earlier, later in
           zip(observations, observations[1:])):
        raise ValueError("insufficient motion window State coverage")
    return start, end, window_states, release_run


##
# @brief 校验六个有限正预算/容差值。
# @param values list/tuple六项，排除bool。
# @param name 诊断字段名。
# @return 六项list；不符ValueError。
def _six_positive(values, name):
    if (not isinstance(values, (list, tuple)) or len(values) != 6 or
            any(type(value) not in (int, float) or not math.isfinite(value) or
                value <= 0 for value in values)):
        raise ValueError(f"invalid {name}: six positive finite values required")
    return list(values)


##
# @brief 要求预算为精确正整数。
# @param value 待核值。
# @param name 错误字段名。
# @return 原int，bool不接受；未独立检查uint64上界。
def _positive_int(value, name):
    if type(value) is not int or value <= 0:
        raise ValueError(f"invalid {name}: positive integer required")
    return value


##
# @brief 校验源快照与预算后以独占link发布新context。
# @param timeline_path CSV捕获路径。
# @param raw_path DPRAW01路径。
# @param action_path Action JSON路径。
# @param output_path 全新context路径，父目录必须已存在，不能是任一输入。
# @param path_tolerance_rad 六轴有限正路径容差rad。
# @param goal_tolerance_rad 六轴有限正目标容差rad。
# @param producer_lag_budget_us offset跨度预算µs，window_v2固定250000。
# @param feedback_age_budget_us 六轴设备反馈年龄预算µs。
# @param ros_feedback_lag_budget_us ROS/raw时间邻近预算µs。
# @param gate_state_max_age_us Gate关联State最大Host年龄µs，不得超过joiner的1s上限。
# @param producer_lag_scope full_stream_v1全流跨度，motion_window_v2只对严格关联活动窗计算。
# @return 写出并返回context dict；输入/预算不符或目标已存在失败。
# 输入用joiner._snapshot一次读取并绑定SHA；raw需同一非零管理client，Source产生时刻正且不回退（允许相等）。
# 临时文件置同目录、flush/fsync文件后os.link到最终名，已存在不会覆盖；finally删临时名。未fsync父目录，不应扩大为断电持久事务保证。
def build_context(timeline_path, raw_path, action_path, output_path, *,
                  path_tolerance_rad, goal_tolerance_rad,
                  producer_lag_budget_us, feedback_age_budget_us,
                  ros_feedback_lag_budget_us, gate_state_max_age_us,
                  producer_lag_scope="full_stream_v1"):
    """校验采集和显式限值，再独占、原子地写入 context.json。"""
    paths = [Path(path) for path in (timeline_path, raw_path, action_path)]
    output = Path(output_path)
    if output.resolve() in {path.resolve() for path in paths}:
        raise ValueError("output must differ from every capture input")
    path_tolerance_rad = _six_positive(path_tolerance_rad, "path tolerance")
    goal_tolerance_rad = _six_positive(goal_tolerance_rad, "goal tolerance")
    producer_lag_budget_us = _positive_int(producer_lag_budget_us, "producer lag budget")
    feedback_age_budget_us = _positive_int(feedback_age_budget_us, "feedback age budget")
    ros_feedback_lag_budget_us = _positive_int(ros_feedback_lag_budget_us,
                                               "ROS feedback lag budget")
    gate_state_max_age_us = _positive_int(gate_state_max_age_us,
                                          "Gate state maximum age")
    if gate_state_max_age_us > joiner.STATUS_MAX_AGE_US:
        raise ValueError("Gate state maximum age exceeds joiner limit")
    if producer_lag_scope not in ("full_stream_v1", "motion_window_v2"):
        raise ValueError("invalid producer lag scope")
    if producer_lag_scope == "motion_window_v2" and producer_lag_budget_us != 250000:
        raise ValueError("motion window producer lag budget must be 250000us")

    snapshots = [joiner._snapshot(path) for path in paths]
    raw_records = joiner._raw_records(snapshots[1][0])
    states = [item for item in raw_records if item["kind"] == 2]
    if not states:
        raise ValueError("empty DPRAW01 State stream")
    management = [item for item in raw_records if item["kind"] in (4, 5)]
    requests = [item for item in management if item["kind"] == 4]
    if not requests:
        raise ValueError("missing DPRAW01 management request client identity")
    clients = {item["fields"]["client_instance_id"] for item in management}
    if len(clients) != 1 or "00" * 16 in clients:
        raise ValueError("mismatched or zero DPRAW01 management client identity")
    offsets = []
    previous_produced = None
    for item in states:
        produced = item["fields"]["produced_at_us"]
        if produced <= 0 or (previous_produced is not None and
                             produced < previous_produced):
            raise ValueError("nonmonotonic DPRAW01 State produced time")
        previous_produced = produced
        offsets.append(item["when"] - produced)
    ##
    # @brief 全流offset跨度保留environment_lag；v2另算活动窗跨度并在context保留两者及window run身份。
    # 正窗口门槛通过不意味着全采集环境总跨度也低于预算。
    environment_lag = max(offsets) - min(offsets)
    try:
        action = json.loads(snapshots[2][0])
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("invalid action JSON") from error
    if not isinstance(action, dict):
        raise ValueError("invalid action JSON object")

    window = None
    producer_lag = environment_lag
    if producer_lag_scope == "motion_window_v2":
        start, end, window_states, release_run = _motion_window(
            snapshots[0][0], raw_records, next(iter(clients)), gate_state_max_age_us)
        window_offsets = [item["when"] - item["fields"]["produced_at_us"]
                          for item in window_states]
        producer_lag = max(window_offsets) - min(window_offsets)
        window = {"start_steady_us": start, "end_steady_us": end,
                  "state_samples": len(window_states), "release_run_id": release_run}
    if producer_lag > producer_lag_budget_us:
        raise ValueError("observed State producer lag exceeds producer lag budget")

    context = {
        "timeline_sha256": snapshots[0][1], "raw_sha256": snapshots[1][1],
        "action_sha256": snapshots[2][1], "client_id": next(iter(clients)),
        "path_tolerance_rad": path_tolerance_rad,
        "goal_tolerance_rad": goal_tolerance_rad,
        "producer_lag_us": producer_lag,
        "producer_lag_method": (WINDOW_LAG_METHOD if window is not None else
                                PRODUCER_LAG_METHOD),
        "producer_lag_budget_us": producer_lag_budget_us,
        "feedback_age_budget_us": feedback_age_budget_us,
        "ros_feedback_lag_budget_us": ros_feedback_lag_budget_us,
        "gate_state_max_age_us": gate_state_max_age_us,
    }
    if window is not None:
        context = {**context, "producer_lag_gate_version": 2,
                   "producer_lag_environment_us": environment_lag,
                   "producer_lag_window": window}
    encoded = (json.dumps(context, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode()
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=output.parent, prefix=f".{output.name}.",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return context


##
# @brief 解析逗号分隔浮点文本。
# @param value 命令行文本。
# @return float列表；此层不核恰六项/正值/有限，交build_context验证。
def _six_argument(value):
    try:
        return [float(part) for part in value.split(",")]
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected six comma-separated numbers") from error


##
# @brief 要求显式捕获路径和预算后生成离线context。
# @param argv 可选argv列表，None使用进程参数。
# @return 成功0；OSError/ValueError转parser.error，打印路径和观察跨度。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("timeline", "raw", "action", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    for name in ("path-tolerance-rad", "goal-tolerance-rad"):
        parser.add_argument(f"--{name}", type=_six_argument, required=True)
    for name in ("producer-lag-budget-us", "feedback-age-budget-us",
                 "ros-feedback-lag-budget-us", "gate-state-max-age-us"):
        parser.add_argument(f"--{name}", type=int, required=True)
    parser.add_argument("--producer-lag-scope", default="full_stream_v1",
                        choices=("full_stream_v1", "motion_window_v2"))
    args = parser.parse_args(argv)
    try:
        result = build_context(
            args.timeline, args.raw, args.action, args.output,
            path_tolerance_rad=args.path_tolerance_rad,
            goal_tolerance_rad=args.goal_tolerance_rad,
            producer_lag_budget_us=args.producer_lag_budget_us,
            feedback_age_budget_us=args.feedback_age_budget_us,
            ros_feedback_lag_budget_us=args.ros_feedback_lag_budget_us,
            gate_state_max_age_us=args.gate_state_max_age_us,
            producer_lag_scope=args.producer_lag_scope)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(args.output.resolve()),
                      "producer_lag_us": result["producer_lag_us"],
                      "producer_lag_method": result["producer_lag_method"]},
                     sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
