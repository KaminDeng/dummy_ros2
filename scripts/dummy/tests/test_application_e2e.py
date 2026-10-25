"""P8 六轴 ROS 原始证据的最小拒绝门禁。"""
##
# @file test_application_e2e.py
# @brief 六轴应用证据validator的合成wire、SHA页链、管理与timeline拒绝矩阵。
#
# 全部raw/manage/pages/时间和反馈均在内存构造；source文本ros_moveit_jtc_usb_ref_can只是fixture标签，不证明发生过ROS/USB/CAN或物理动作。位置/误差rad，年龄/Host接收时间µs；Source字段和Host字段不直接等同。SHA重算用于隔离语义负例，不构成设备签名。

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
from time import perf_counter

import pytest


MODULE = Path(__file__).resolve().parents[1] / "application_e2e.py"
SPEC = importlib.util.spec_from_file_location("application_e2e", MODULE)
application_e2e = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(application_e2e)
LAYOUT = json.loads((Path(__file__).resolve().parents[3] /
    "contracts/dummy_robot_v2/codec/wire_layout.json").read_text())


##
# @brief 按当前wire_layout字段offset小端打包合成管理请求/响应，未指定字段留零；schema固定2.4，外层记录元数据与payload分开。
# @param kind DPRAW记录种类：4请求、5响应；其它值也走响应布局，调用方只用4/5。
# @param when 本条Host接收/记录单调时间µs，不是Source生产时间。
# @param operation 请求操作号，8 Complete、9 Cancel、10 DisableAndRelease、12 Query。
# @param request_id 原始64位关联ID，外层request_seq默认取其数值。
# @param run 设备attempt/RUN ID，和ROS goal UUID不同。
# @param session 本条owner session ID，0表示已释放。
# @param phase 响应transaction_phase，默认3 Committed。
# @param result 响应result_code，默认0成功。
# @param operation_id 响应事务强ID，默认1，区别于request_id。
# @param client 16字节client ID的32字符hex。
# @param queried_operation_id Query指向的原请求键；默认0。
# @param motion_epoch 响应当前motion epoch；默认0。
# @param expected_motion_epoch 请求锚定motion epoch；默认0。
# @param request_seq 显式uint32发送序号；None使用request_id，不裁剪溢出。
# @param motion_complete 显式终态位；None按响应非零session自动设1。
# @param release_committed 显式释放位；None按响应session0自动设1。
# @return 含kind/when/外层身份/seq/raw_hex的合成管理record字典。
def manage(kind, when, *, operation=0, request_id=1, run=0, session=3,
           phase=3, result=0, operation_id=1, client="11" * 16,
           queried_operation_id=0, motion_epoch=0, expected_motion_epoch=0,
           request_seq=None,
           motion_complete=None, release_committed=None):
    name = "DummyManageV2Request" if kind == 4 else "DummyManageV2Response"
    raw = bytearray(LAYOUT["payloads"][name]["encoded_size"])
    values = {"schema_major": 2, "schema_minor": 4, "client_instance_id": client,
              "device_id": "22" * 16, "request_id": request_id,
              "request_seq": request_id if request_seq is None else request_seq,
              "operation": operation,
              "queried_operation_id": queried_operation_id,
              "expected_boot_epoch": 2, "expected_session_id": 3,
              "expected_motion_epoch": expected_motion_epoch,
              "expected_profile_id": 201, "transaction_phase": phase,
              "result_code": result, "operation_id": operation_id,
              "receipt": 1,
              "motion_complete": int(kind == 5 and session != 0)
                                 if motion_complete is None else motion_complete,
              "release_committed": int(kind == 5 and session == 0)
                                   if release_committed is None else release_committed,
              "boot_epoch": 2, "session_id": session, "profile_id": 201,
              "run_id": run, "motion_epoch": motion_epoch}
    codes = {"uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q",
             "float64": "d", "bool": "B"}
    for field in LAYOUT["payloads"][name]["fields"]:
        value = values.get(field["name"])
        if value is None:
            continue
        if field["count"] > 1:
            value = list(bytes.fromhex(value) if isinstance(value, str) else value)
        else:
            value = [value]
        struct.pack_into("<" + codes[field["type"]] * field["count"], raw,
                         field["offset"], *value)
    return {"kind": kind, "when": when, "boot": 2,
            "session": 3 if kind == 4 else session,
            "run": 0 if kind == 4 else run,
            "seq": (request_id if request_seq is None else request_seq)
            if kind == 4 else 0, "raw_hex": raw.hex()}


##
# @brief 生成372字节组页及末页：192字节零尾seed包含run/boot/client/四摘要，每页链按previous+前307字节+末字节重算。
# @param identity 合成client/device及合同/模型/profile/固件摘要字典。
# @param run 设备attempt/RUN ID，和ROS goal UUID不同。
# @param session 本条owner session ID，0表示已释放。
# @param boot 外层boot epoch，默认场景为2。
# @param epoch 默认组及终页birth motion epoch。
# @param kind attempt_kind：默认1普通RUN，3专用Release，2用于安全owner负例。
# @param parent 父RUN ID，专用Release需链接前一RUN。
# @param final_state 终页状态码：6完成Hold、7取消Hold、8释放、2活动取消释放。
# @param group_id 单组默认ID，group_ids显式列表优先。
# @param group_ids 组ID有序列表；None只用group_id。
# @param group_epochs 逐组motion epoch列表，和birth/终页epoch分离。
# @param terminal_epoch 末页的birth motion epoch，不等于最后一组阶段epoch。
# @param group_counts 逐组累积count；None自动1..N，终页总数N。
# @param attempt_id 显式attempt ID；None等于run。
# @param categories 每个组页的类别列表：1普通Hold/运动、2Safety、3Disable；顺序由用例决定。
# @param safety_failure_source 类别2安全组的failure_source，默认0干净。
# @return N个组页加1个终页的raw_hex/boot/session字典列表。
def pages(identity, *, run, session, boot=2, epoch=2, kind=1, parent=0,
          final_state=6, group_id=7, group_ids=None, group_epochs=None,
          terminal_epoch=None, group_counts=None, attempt_id=None,
          categories=None, safety_failure_source=0):
    group_ids = [group_id] if group_ids is None else group_ids
    group_epochs = [epoch] * len(group_ids) if group_epochs is None else group_epochs
    group_counts = list(range(1, len(group_ids) + 1)) if group_counts is None else group_counts
    categories = [1] * len(group_ids) if categories is None else categories
    seed = struct.pack("<QQ", run, boot) + bytes.fromhex(identity["client_id"])
    seed += b"".join(bytes.fromhex(identity[name]) for name in
                     ("contract_hash", "model_hash", "profile_hash", "firmware_hash"))
    previous = hashlib.sha256(seed.ljust(192, b"\0")).digest()
    result = []
    for seq in range(len(group_ids) + 1):
        terminal = seq == len(group_ids)
        raw = bytearray(372)
        struct.pack_into("<HHQIQII", raw, 0, 2, 4, run, seq,
                         0 if terminal else group_ids[seq], 0 if terminal else seq + 1,
                         (epoch if terminal_epoch is None else terminal_epoch) if terminal
                         else group_epochs[seq])
        raw[32:38] = bytes((4, 0, 0, 0, 0, 1)) if terminal else bytes(
            (categories[seq], 63, 63, 63, 0, 1))
        if not terminal and categories[seq] == 2:
            struct.pack_into("<H", raw, 38, safety_failure_source)
        struct.pack_into("<IQB QBBB H", raw, 281,
                         len(group_ids) if terminal else group_counts[seq],
                         run if attempt_id is None else attempt_id, kind, parent,
                         int(terminal and final_state == 6),
                         int(terminal and final_state in (2, 8)),
                         final_state if terminal else 0, 0)
        raw[307:339] = previous
        digest = hashlib.sha256(previous + raw[:307] + raw[371:372]).digest()
        raw[339:371] = digest
        result.append({"raw_hex": raw.hex(), "boot_epoch": boot, "session_id": session})
        previous = digest
    return result


##
# @brief 构造一次成功RUN4、session3、Complete及Release子RUN5的最小完整字典，timeline/管理回执/页链/反馈全人工生成。
# @return 合成完整证据字典，不是设备报告。
@pytest.fixture
def evidence():
    identity = {name: "11" * width for name, width in
                (("client_id", 16), ("contract_hash", 32), ("model_hash", 32),
                 ("profile_hash", 32), ("firmware_hash", 32))}
    identity["device_id"] = "22" * 16
    result = {
        "source": "ros_moveit_jtc_usb_ref_can",
        "profile_id": 201,
        "configured_mask": 63,
        "identity": identity,
        "environment": {"producer_lag_us": 1000, "producer_lag_budget_us": 10000},
        "goals": [{
            "goal_id": "a" * 32, "action_status": "SUCCEEDED", "result_error_code": 0,
            "device_id": identity["device_id"],
            "run_id": 4, "session_id": 3, "boot_epoch": 2, "motion_epoch": 2,
            "target_rad": [0.01, -0.02, 1.59, -0.01, 0.02, -0.03],
            "feedback_rad": [0.01, -0.02, 1.59, -0.01, 0.02, -0.03],
            "path_error_max_rad": [0.001] * 6,
            "path_tolerance_rad": [0.01] * 6,
            "goal_tolerance_rad": [0.003] * 6,
            "feedback_age_us": [1000] * 6, "feedback_age_budget_us": 10000,
            "timeline": [
                {"event": event, "goal_uuid": "a" * 32, "boot_epoch": 2,
                 "session_id": 0 if event == "post_release_status" else 3,
                 "run_id": 5 if event == "post_release_status" else 4,
                 "motion_epoch": 3 if event == "post_release_status" else 2,
                 "steady_us": 1000 * (index + 1), "axis_mask": 63,
                 "release_request_id": 2 if event == "post_release_status" else 0,
                 "release_operation_id": 2 if event == "post_release_status" else 0,
                 "status_core_state": 1 if event == "post_release_status" else 4 if event == "pre_release_hold" else 3,
                 "state_core_state": 1 if event == "post_release_status" else 4 if event == "pre_release_hold" else 3,
                 **({"feedback_age_us": [1000] * 6,
                     "feedback_seq": [1] * 6} if event == "feedback" else {})}
                for index, event in enumerate(
                    ("accepted", "before_begin", "after_begin", "feedback",
                     "terminal_success", "pre_release_hold", "post_release_status"))],
            "evidence_pages": pages(identity, run=4, session=3),
            "groups": [{"group_id": 7, "run_id": 4, "session_id": 3,
                        "motion_epoch": 2, "axis_mask": 63, "sent_mask": 63,
                        "committed": True}],
            "terminal": "MOTION_COMPLETED_HOLDING", "hold_confirmed": True,
        }],
        "pre_release_hold": {"run_id": 4, "session_id": 3, "boot_epoch": 2,
                             "motion_epoch": 2, "confirmed": True},
        "management_records": [manage(4, 4800, operation=8, request_id=1),
                               manage(5, 5000, request_id=1, run=4),
                               manage(4, 6200, operation=10, request_id=2),
                               manage(5, 6700, request_id=2, run=5, session=0,
                                      operation_id=2, motion_epoch=3)],
            "release": {"run_id": 5, "parent_run_id": 4, "session_id": 3,
                    "boot_epoch": 2, "terminal": "RELEASED", "confirmed": True,
                    "request_id": 2, "operation_id": 2,
                    "evidence_pages": pages(identity, run=5, session=3, kind=3,
                                            parent=4, final_state=8, group_id=8,
                                            group_epochs=[3], terminal_epoch=2,
                                            categories=[3])},
    }
    return result


##
# @brief 合成完整成功+释放证据应被validator接受，不称现场执行通过。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_six_axis_evidence_passes(evidence):
    application_e2e.validate_evidence(evidence)


##
# @brief goal换device后不得借另一设备的公共identity与页链。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_other_device_goal_cannot_share_application_evidence(evidence):
    evidence["goals"][0]["device_id"] = "33" * 16
    with pytest.raises(ValueError, match="device"):
        application_e2e.validate_evidence(evidence)


##
# @brief 请求/响应raw同时改device也不能混入原应用证据，核管理身份绑定。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_other_device_management_receipt_cannot_share_application_evidence(evidence):
    for item in evidence["management_records"][:2]:
        raw = bytearray(bytes.fromhex(item["raw_hex"]))
        name = "DummyManageV2Request" if item["kind"] == 4 else "DummyManageV2Response"
        offset = next(field["offset"] for field in
            LAYOUT["payloads"][name]["fields"] if field["name"] == "device_id")
        raw[offset:offset + 16] = bytes.fromhex("33" * 16)
        item["raw_hex"] = raw.hex()
    with pytest.raises(ValueError, match="management record identity"):
        application_e2e.validate_evidence(evidence)


##
# @brief 删管理记录后即使JSON和raw末态声称释放也不能正向通过。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_release_without_management_receipts_is_not_positive(evidence):
    del evidence["management_records"]
    with pytest.raises(ValueError, match="management|receipt"):
        application_e2e.validate_evidence(evidence)


##
# @brief 将Release改为活动Hold刷新后Disable：组epoch2→3、类别Hold→Disable、终页仍birth2及Committed响应epoch3。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return 原evidence对象，已原地改成活动Hold刷新释放场景。
def active_release(evidence):
    evidence["release"].update(terminal="CANCELED_AND_RELEASED",
        evidence_pages=pages(evidence["identity"], run=5, session=3,
                             final_state=2, group_ids=[8, 9],
                             group_epochs=[2, 3], terminal_epoch=2,
                             categories=[1, 3]))
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    return evidence


##
# @brief 合成活动Hold→Disable释放与同请求强回执可接受，不仅靠terminal字符串。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_hold_refresh_release_requires_matching_committed_receipt(evidence):
    active_release(evidence)
    application_e2e.validate_evidence(evidence)


##
# @brief 30个Hold刷新组后一个Disable仍可形成有序释放，不能只支持固定两组。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_accepts_long_hold_phase_before_disable(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(
        evidence["identity"], run=5, session=3, final_state=2,
        group_ids=list(range(8, 39)),
        group_epochs=[2] * 30 + [3], terminal_epoch=2,
        categories=[1] * 30 + [3])
    application_e2e.validate_evidence(evidence)


##
# @brief Disable仍用旧epoch2即使timeline/回执也改2仍拒绝，必须新释放阶段。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_disable_sent_without_new_epoch(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, final_state=2, group_ids=[8, 9],
        group_epochs=[2, 2], terminal_epoch=2, categories=[1, 3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 2
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=2)
    with pytest.raises(ValueError, match="Hold then Disable motion epoch phase"):
        application_e2e.validate_evidence(evidence)


##
# @brief 释放回执不能覆盖timeline双流仍Hold4的事实，未到UnreferencedIdle不得称释放。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_receipt_cannot_override_observed_hold(evidence):
    active_release(evidence)
    status = evidence["goals"][0]["timeline"][-1]
    status["status_core_state"] = 4
    status["state_core_state"] = 4
    with pytest.raises(ValueError, match="unconfirmed timeline HOLD or release"):
        application_e2e.validate_evidence(evidence)


##
# @brief 组从2到3但终页motion保birth2是合法角色区分，不要求所有页epoch相同。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_hold_disable_epochs_advance_then_terminal_uses_birth(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9],
        group_epochs=[2, 3], terminal_epoch=2, categories=[1, 3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    application_e2e.validate_evidence(evidence)


##
# @brief Hold2→Safety3→Disable3允许，终页保birth2；这是输入序列语义而非实际减速试验。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_accepts_bounded_safety_deceleration_before_disable(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9, 10],
        group_epochs=[2, 3, 3], terminal_epoch=2, categories=[1, 2, 3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    application_e2e.validate_evidence(evidence)


##
# @brief Safety/Disable乱序、缺Disable或旧epoch Safety等参数组合均拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param categories 每个组页的类别列表：1普通Hold/运动、2Safety、3Disable；顺序由用例决定。
# @param epochs 逐组motion epoch列表，用来核阶段顺序。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("categories,epochs", [
    ([1, 3, 2], [2, 3, 3]),
    ([2, 1, 3], [3, 2, 3]),
    ([1, 2, 2], [2, 3, 3]),
    ([1, 2, 3], [2, 2, 3]),
])
def test_active_release_rejects_wrong_safety_stop_phase(evidence, categories, epochs):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9, 10],
        group_epochs=epochs, terminal_epoch=2, categories=categories)
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    with pytest.raises(ValueError, match="Hold then Disable|phase"):
        application_e2e.validate_evidence(evidence)


##
# @brief Safety组带failure_source17后即使链自洽也不能当干净正常释放。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_fault_source_on_safety_deceleration(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9, 10],
        group_epochs=[2, 3, 3], terminal_epoch=2, categories=[1, 2, 3],
        safety_failure_source=17)
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    with pytest.raises(ValueError, match="Hold then Disable"):
        application_e2e.validate_evidence(evidence)


##
# @brief 长Hold流最终Disable后又回Hold，状态阶段回放必须拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_hold_after_disable_in_long_stream(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2,
        group_ids=list(range(8, 18)), group_epochs=[2] * 8 + [3, 3],
        terminal_epoch=2, categories=[1] * 8 + [3, 1])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = 3
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=3)
    with pytest.raises(ValueError, match="Hold then Disable"):
        application_e2e.validate_evidence(evidence)


##
# @brief 组跳代/回退或终页非birth epoch逐项拒绝，即使末timeline/回执跟着改也不够。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param group_epochs 逐组motion epoch列表，和birth/终页epoch分离。
# @param terminal_epoch 末页的birth motion epoch，不等于最后一组阶段epoch。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("group_epochs,terminal_epoch", [
    ([2, 4], 2), ([3, 2], 2), ([2, 3], 3),
])
def test_active_release_rejects_skipped_regressing_or_wrong_terminal_epoch(
        evidence, group_epochs, terminal_epoch):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, final_state=2, group_ids=[8, 9],
        group_epochs=group_epochs, terminal_epoch=terminal_epoch,
        categories=[1, 3])
    evidence["goals"][0]["timeline"][-1]["motion_epoch"] = group_epochs[-1]
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=group_epochs[-1])
    with pytest.raises(ValueError, match="phase|evidence"):
        application_e2e.validate_evidence(evidence)


##
# @brief Query指原Release request2、回复新Query RID3但op2且强终态时可关联原事务。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_accepts_query_result_for_same_request(evidence):
    active_release(evidence)
    evidence["management_records"][-1:] = [
        manage(4, 6600, operation=12, request_id=3, queried_operation_id=2),
        manage(5, 6700, request_id=3, run=5, session=0, operation_id=2,
               motion_epoch=3)]
    application_e2e.validate_evidence(evidence)


##
# @brief 用大64位RID和固定request_seq构造两次字节相同Query及单个后到强结果，不重发原Release变更。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return 原Query字典，evidence也已原地换成两次Query+唯一结果。
def retrying_release_query(evidence):
    active_release(evidence)
    query = manage(4, 6500, operation=12, request_id=15082018622492830540,
                   request_seq=2093819891, queried_operation_id=2)
    evidence["management_records"][-1:] = [
        query, {**query, "when": 6600},
        manage(5, 6700, request_id=15082018622492830540,
               run=5, session=0, operation_id=2,
               motion_epoch=3)]
    return query


##
# @brief 相同Query重试在唯一结果前接受，并核两raw字节均与原Query相同。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_accepts_identical_query_retry_before_one_result(evidence):
    query = retrying_release_query(evidence)
    application_e2e.validate_evidence(evidence)
    assert evidence["management_records"][-3]["raw_hex"] == query["raw_hex"]
    assert evidence["management_records"][-2]["raw_hex"] == query["raw_hex"]


##
# @brief 同RID重试改末字节拒绝重复身份，不能借重试改变请求体。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_mutated_query_retry(evidence):
    retrying_release_query(evidence)
    retry = evidence["management_records"][-2]
    raw = bytearray.fromhex(retry["raw_hex"])
    raw[-1] ^= 1
    retry["raw_hex"] = raw.hex()
    with pytest.raises(ValueError, match="duplicate management request identity"):
        application_e2e.validate_evidence(evidence)


##
# @brief 唯一结果后再回放同Query视歧义而拒绝，不当新的等待周期。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_query_replay_after_response(evidence):
    retrying_release_query(evidence)
    records = evidence["management_records"]
    records[-2], records[-1] = {**records[-1], "when": 6600}, {**records[-2], "when": 6700}
    with pytest.raises(ValueError, match="duplicate management request identity"):
        application_e2e.validate_evidence(evidence)


##
# @brief 第二Query时间退到原6500µs之前即拒绝管理记录倒序。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_backdated_query_retry(evidence):
    retrying_release_query(evidence)
    evidence["management_records"][-2]["when"] = 6499
    with pytest.raises(ValueError, match="regressing management record time"):
        application_e2e.validate_evidence(evidence)


##
# @brief 继续插入三次重试超出允许重复次数，不能无限借同RID续等。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_query_beyond_three_retries(evidence):
    retrying_release_query(evidence)
    records = evidence["management_records"]
    records[-1:-1] = [{**records[-2], "when": when} for when in (6620, 6640, 6660)]
    with pytest.raises(ValueError, match="duplicate management request identity"):
        application_e2e.validate_evidence(evidence)


##
# @brief 原Release变更重复发送不享受只读Query重试例外。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_repeated_release_mutation(evidence):
    active_release(evidence)
    original = evidence["management_records"][-2]
    evidence["management_records"].insert(-1, {**original, "when": 6500})
    with pytest.raises(ValueError, match="duplicate management request identity"):
        application_e2e.validate_evidence(evidence)


##
# @brief 重试只允许一个强结果，同一结果再出现拒绝歧义。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_retried_query_rejects_duplicate_committed_result(evidence):
    retrying_release_query(evidence)
    response = evidence["management_records"][-1]
    evidence["management_records"].append({**response, "when": 6800})
    with pytest.raises(ValueError, match="ambiguous committed management receipt"):
        application_e2e.validate_evidence(evidence)


##
# @brief Complete/Release即使phase Committed，receipt0/3/4仍不能作确认。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param operation 8选Complete响应、10选Release响应。
# @param receipt Manage receipt码，1/2为当前正例确认码，其他按负例注入。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("operation,receipt", [
    (8, 0), (8, 3), (8, 4), (10, 0), (10, 3), (10, 4)])
def test_committed_management_requires_confirmed_receipt(evidence, operation, receipt):
    index = 1 if operation == 8 else 3
    response = evidence["management_records"][index]
    raw = bytearray.fromhex(response["raw_hex"])
    offset = next(field["offset"] for field in
                  LAYOUT["payloads"]["DummyManageV2Response"]["fields"]
                  if field["name"] == "receipt")
    raw[offset] = receipt
    response["raw_hex"] = raw.hex()
    with pytest.raises(ValueError, match="receipt|identity"):
        application_e2e.validate_evidence(evidence)


##
# @brief 强Release结果receipt1或2均可接受，独立于Query RID。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param receipt Manage receipt码，1/2为当前正例确认码，其他按负例注入。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("receipt", [1, 2])
def test_committed_management_accepts_confirmed_receipt_codes(evidence, receipt):
    response = evidence["management_records"][-1]
    raw = bytearray.fromhex(response["raw_hex"])
    offset = next(field["offset"] for field in
                  LAYOUT["payloads"]["DummyManageV2Response"]["fields"]
                  if field["name"] == "receipt")
    raw[offset] = receipt
    response["raw_hex"] = raw.hex()
    application_e2e.validate_evidence(evidence)


##
# @brief 破坏Query的device/boot/session/ref/motion/management/profile/mask任一原锚字段均拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @param value 该字段的参数化错误值。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field,value", [
    ("device_id", "33" * 16),
    ("expected_boot_epoch", 9),
    ("expected_session_id", 9),
    ("expected_reference_epoch", 9),
    ("expected_motion_epoch", 9),
    ("expected_management_epoch", 9),
    ("expected_profile_id", 9),
    ("configured_mask", 63),
])
def test_release_query_must_preserve_original_request_identity(evidence, field, value):
    active_release(evidence)
    query = manage(4, 6600, operation=12, request_id=3, queried_operation_id=2)
    raw = bytearray.fromhex(query["raw_hex"])
    layout = next(item for item in LAYOUT["payloads"]["DummyManageV2Request"]["fields"]
                  if item["name"] == field)
    if layout["count"] > 1:
        raw[layout["offset"]:layout["offset"] + layout["count"]] = bytes.fromhex(value)
    else:
        struct.pack_into("<" + {"uint8": "B", "uint16": "H", "uint32": "I",
                                "uint64": "Q"}[layout["type"]], raw, layout["offset"], value)
    query["raw_hex"] = raw.hex()
    if field == "expected_boot_epoch":
        query["boot"] = value
    elif field == "expected_session_id":
        query["session"] = value
    evidence["management_records"][-1:] = [
        query, manage(5, 6700, request_id=3, run=5, session=0, operation_id=2)]
    with pytest.raises(ValueError, match="receipt|identity"):
        application_e2e.validate_evidence(evidence)


##
# @brief Query指Complete request1而非Release2不能借其响应确认释放。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_query_for_another_request(evidence):
    active_release(evidence)
    evidence["management_records"][-1:] = [
        manage(4, 6600, operation=12, request_id=3, queried_operation_id=1),
        manage(5, 6700, request_id=3, run=5, session=0, operation_id=2)]
    with pytest.raises(ValueError, match="receipt"):
        application_e2e.validate_evidence(evidence)


##
# @brief 附加相同强Release回执也属于歧义，不因内容相同就去重成成功。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_duplicate_committed_receipt(evidence):
    active_release(evidence)
    evidence["management_records"].append(copy.deepcopy(evidence["management_records"][-1]))
    with pytest.raises(ValueError, match="ambiguous"):
        application_e2e.validate_evidence(evidence)


##
# @brief attempt_kind2安全所有者不能冒充正常活动Hold释放依据。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_active_release_rejects_safety_owned_attempt(evidence):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, kind=2, final_state=2,
        group_ids=[8, 9], categories=[2, 3])
    with pytest.raises(ValueError, match="Hold then Disable|evidence"):
        application_e2e.validate_evidence(evidence)


##
# @brief Release JSON的request/op键改99与raw不符必须拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field", ["request_id", "operation_id"])
def test_active_release_timeline_key_must_match_raw_receipt(evidence, field):
    active_release(evidence)
    evidence["release"][field] = 99
    with pytest.raises(ValueError, match="request key"):
        application_e2e.validate_evidence(evidence)


##
# @brief 删/换run/Prepared或旧RID回执逐项ValueError，不能靠外层terminal补齐。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param change 修改记录/goal的参数化回调，在当前独立fixture上执行。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("change", [
    lambda records: records.pop(),
    lambda records: records[-1].update(raw_hex=manage(5, 6700, request_id=2,
        run=4, session=0, operation_id=2)["raw_hex"]),
    lambda records: records[-1].update(raw_hex=manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, phase=2)["raw_hex"]),
    lambda records: records[-1].update(raw_hex=manage(5, 6700, request_id=1,
        run=5, session=0, operation_id=2)["raw_hex"]),
])
def test_active_release_rejects_missing_cross_run_pending_or_old_receipt(evidence, change):
    active_release(evidence)
    change(evidence["management_records"])
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 只有Hold、只有Disable或反序类别都不能证明活动Hold后的Disable发送。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param categories 每个组页的类别列表：1普通Hold/运动、2Safety、3Disable；顺序由用例决定。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("categories", [[1, 1], [3, 3], [3, 1]])
def test_active_release_requires_hold_then_disable_sent(evidence, categories):
    active_release(evidence)
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, final_state=2, group_ids=[8, 9], categories=categories)
    with pytest.raises(ValueError, match="Hold then Disable"):
        application_e2e.validate_evidence(evidence)


##
# @brief 构造同RUN birth Hold1→目标2→Complete Hold3，子Release Disable4且末页保birth3，timeline角色同步更新。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return 原evidence对象，阶段和raw链已同步重建。
def mixed_epoch_success(evidence):
    goal = evidence["goals"][0]
    for row in goal["timeline"][:2]:
        row["motion_epoch"] = 1
    goal["timeline"][-3]["motion_epoch"] = 3
    goal["timeline"][-2]["motion_epoch"] = 3
    goal["timeline"][-1]["motion_epoch"] = 4
    goal["groups"] = [{**goal["groups"][0], "group_id": group_id,
                       "motion_epoch": epoch} for group_id, epoch in
                      ((6, 1), (7, 2), (8, 3))]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[6, 7, 8], group_epochs=[1, 2, 3], terminal_epoch=1)
    evidence["pre_release_hold"]["motion_epoch"] = 3
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=3, kind=3, parent=4, final_state=8,
        group_id=9, group_epochs=[4], terminal_epoch=3, categories=[3])
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=4)
    return evidence


##
# @brief 混合epoch完整页链可保留前置/完成后Hold，但目标归运动epoch2。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_full_chain_keeps_pre_begin_hold_and_complete_hold_outside_goal(evidence):
    mixed_epoch_success(evidence)
    application_e2e.validate_evidence(evidence)


##
# @brief 删birth Hold1而从目标epoch2起链，不能借重算SHA伪装完整RUN起点。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_mixed_success_requires_birth_epoch_raw_prefix(evidence):
    mixed_epoch_success(evidence)
    goal = evidence["goals"][0]
    goal["groups"] = [{**goal["groups"][0], "group_id": group_id,
                       "motion_epoch": epoch} for group_id, epoch in ((7, 2), (8, 3))]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8], group_epochs=[2, 3], terminal_epoch=2)
    with pytest.raises(ValueError, match="phase|evidence"):
        application_e2e.validate_evidence(evidence)


##
# @brief 目标后仍epoch2且无Complete新Hold阶段，不能完成成功证据。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_mixed_success_requires_complete_epoch_after_goal(evidence):
    mixed_epoch_success(evidence)
    goal = evidence["goals"][0]
    goal["timeline"][-3]["motion_epoch"] = 2
    goal["timeline"][-2]["motion_epoch"] = 2
    goal["groups"][-1]["motion_epoch"] = 2
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[6, 7, 8], group_epochs=[1, 2, 2], terminal_epoch=1)
    evidence["pre_release_hold"]["motion_epoch"] = 2
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=2, kind=3, parent=4, final_state=8, group_id=9)
    with pytest.raises(ValueError, match="epoch"):
        application_e2e.validate_evidence(evidence)


##
# @brief 页组epoch阶段重放/跳步或终页birth错误，分别匹配phase/evidence拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param epochs 逐组motion epoch列表，用来核阶段顺序。
# @param terminal_epoch 末页的birth motion epoch，不等于最后一组阶段epoch。
# @param match 期望ValueError消息的正则片段。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("epochs,terminal_epoch,match", [
    ([1, 2, 1, 3], 1, "phase"),
    ([1, 3, 2], 1, "phase"),
    ([1, 1, 3], 1, "phase"),
    ([1, 2, 3], 2, "evidence"),
])
def test_mixed_epoch_chain_rejects_phase_replay_or_wrong_birth_epoch(
        evidence, epochs, terminal_epoch, match):
    mixed_epoch_success(evidence)
    goal = evidence["goals"][0]
    goal["groups"] = [{**goal["groups"][0], "group_id": 6 + index,
                       "motion_epoch": epoch} for index, epoch in enumerate(epochs)]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[6 + index for index in range(len(epochs))],
        group_epochs=epochs, terminal_epoch=terminal_epoch)
    with pytest.raises(ValueError, match=match):
        application_e2e.validate_evidence(evidence)


##
# @brief 两目标不同UUID/RUN共享session，只最后释放子RUN6；第二接纳早于前一Hold则拒跨目标重叠。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_two_goals_share_session_and_release_only_after_last(evidence):
    first = evidence["goals"][0]
    second = copy.deepcopy(first)
    first["timeline"].pop()
    second.update(goal_id="b" * 32, run_id=5,
                  evidence_pages=pages(evidence["identity"], run=5, session=3, group_id=8))
    second["groups"][0].update(group_id=8, run_id=5)
    for row in second["timeline"]:
        row["goal_uuid"] = second["goal_id"]
        row["steady_us"] += 10000
        if row["event"] not in ("accepted", "before_begin"):
            row["run_id"] = 6 if row["event"] == "post_release_status" else 5
        if row["event"] == "post_release_status":
            row.update(release_request_id=4, release_operation_id=4)
    evidence["goals"].append(second)
    evidence["pre_release_hold"]["run_id"] = 5
    evidence["release"].update(run_id=6, parent_run_id=5,
        request_id=4, operation_id=4,
        evidence_pages=pages(evidence["identity"], run=6, session=3,
                             kind=3, parent=5, final_state=8, group_id=9,
                             group_epochs=[3], terminal_epoch=2, categories=[3]))
    evidence["management_records"] = evidence["management_records"][:2] + [
        manage(4, 14800, operation=8, request_id=3),
        manage(5, 15000, request_id=3, run=5, operation_id=3),
        manage(4, 16200, operation=10, request_id=4),
        manage(5, 16700, request_id=4, run=6, session=0, operation_id=4,
               motion_epoch=3)]
    application_e2e.validate_evidence(evidence)
    overlap = copy.deepcopy(evidence)
    overlap["goals"][1]["timeline"][0]["steady_us"] = 5999
    with pytest.raises(ValueError, match="cross-goal"):
        application_e2e.validate_evidence(overlap)


##
# @brief 两组累积count1/2与末页总数2一致可接受，不能要求每页都等终总数。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_multi_group_evidence_count_grows_to_terminal_total(evidence):
    goal = evidence["goals"][0]
    goal["groups"].append({**goal["groups"][0], "group_id": 8})
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
                                    group_ids=[7, 8], group_counts=[1, 2])
    application_e2e.validate_evidence(evidence)


##
# @brief 实际计时3000/30000合成组的validator，要求十倍规模耗时比<25；是本机粗尺度回归，不是严格复杂度/WCET证明。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_large_page_chain_validation_scales_linearly(evidence):
    identity = evidence["identity"]
    durations = []
    for count in (3000, 30000):
        attempt = {"evidence_pages": pages(identity, run=4, session=3,
                                             group_ids=list(range(1, count + 1)))}
        start = perf_counter()
        application_e2e._validate_pages(
            attempt, identity, run=4, session=3, boot=2, epoch=2,
            kind=1, parent=0, final_state=6,
            group_ids=list(range(1, count + 1)))
        durations.append(perf_counter() - start)
    assert durations[1] < durations[0] * 25, durations


##
# @brief 64组中重用首ID或反序，即使页链重算有效仍按组关联/顺序拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param mutation 参数化变异函数或分支名，逐例只制造指定错误。
# @param match 期望ValueError消息的正则片段。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("mutation,match", [
    (lambda ids: [*ids[:-1], ids[0]], "six-axis CAN"),
    (lambda ids: list(reversed(ids)), "CAN groups"),
])
def test_page_chain_rejects_duplicate_or_reordered_groups(evidence, mutation, match):
    identity = evidence["identity"]
    expected_ids = list(range(1, 65))
    attempt = {"evidence_pages": pages(identity, run=4, session=3,
                                         group_ids=mutation(expected_ids))}
    with pytest.raises(ValueError, match=match):
        application_e2e._validate_pages(
            attempt, identity, run=4, session=3, boot=2, epoch=2,
            kind=1, parent=0, final_state=6, group_ids=expected_ids)


##
# @brief 改倒数第二组raw字节不重签，前63组正确也不能掩盖末端链破坏。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_page_chain_rejects_invalid_digest_after_many_valid_groups(evidence):
    identity = evidence["identity"]
    attempt = {"evidence_pages": pages(identity, run=4, session=3,
                                         group_ids=list(range(1, 65)))}
    raw = bytearray.fromhex(attempt["evidence_pages"][-2]["raw_hex"])
    raw[40] ^= 1
    attempt["evidence_pages"][-2]["raw_hex"] = raw.hex()
    with pytest.raises(ValueError, match="invalid evidence chain"):
        application_e2e._validate_pages(
            attempt, identity, run=4, session=3, boot=2, epoch=2,
            kind=1, parent=0, final_state=6, group_ids=list(range(1, 65)))


##
# @brief 直接核首previous等于192字节seed零补尾SHA，不省略未填尾字节。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_wire_chain_seed_includes_zero_initialized_tail(evidence):
    identity = evidence["identity"]
    seed = struct.pack("<QQ", 4, 2) + bytes.fromhex(identity["client_id"])
    seed += b"".join(bytes.fromhex(identity[name]) for name in
                     ("contract_hash", "model_hash", "profile_hash", "firmware_hash"))
    raw = bytes.fromhex(evidence["goals"][0]["evidence_pages"][0]["raw_hex"])
    assert raw[307:339] == hashlib.sha256(seed.ljust(192, b"\0")).digest()


##
# @brief goal缺UUID/RUN/session/motion/groups任一桥接字段则拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field", ["goal_id", "run_id", "session_id", "motion_epoch", "groups"])
def test_missing_action_to_device_link_rejected(evidence, field):
    del evidence["goals"][0][field]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 复制目标导致同ROS UUID重复，即使内容相同也拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_duplicate_goal_uuid_rejected(evidence):
    evidence["goals"].append(copy.deepcopy(evidence["goals"][0]))
    with pytest.raises(ValueError, match="duplicate ROS goal id"):
        application_e2e.validate_evidence(evidence)


##
# @brief 不同UUID仍不能共用同device RUN，避免一个设备执行冒充多个目标。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_duplicate_device_run_across_distinct_goals_rejected(evidence):
    first = evidence["goals"][0]
    first["timeline"].pop()
    second = copy.deepcopy(evidence["goals"][0])
    second["goal_id"] = "b" * 32
    for row in second["timeline"]:
        row["goal_uuid"] = second["goal_id"]
    second["timeline"].append({**evidence["goals"][0]["timeline"][-1],
                               "event": "post_release_status", "session_id": 0,
                               "steady_us": 7000, "status_core_state": 1,
                               "state_core_state": 1, "goal_uuid": second["goal_id"]})
    evidence["goals"].append(second)
    with pytest.raises(ValueError, match="duplicate device run"):
        application_e2e.validate_evidence(evidence)


##
# @brief run变list时应ValueError而非内部不可hash异常泄漏。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_malformed_unhashable_run_is_validation_error(evidence):
    evidence["goals"][0]["run_id"] = [4]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 组run/session/motion、mask63、sent全轴及committed各自必须匹配。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @param value 该字段的参数化错误值。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field,value", [
    ("run_id", 5), ("session_id", 4), ("motion_epoch", 3),
    ("axis_mask", 1), ("sent_mask", 62), ("committed", False),
])
def test_group_must_bind_all_six_axes_to_goal(evidence, field, value):
    evidence["goals"][0]["groups"][0][field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 空groups不能靠已有raw/Action成功字符串通过。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_empty_can_groups_rejected(evidence):
    evidence["goals"][0]["groups"] = []
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 缺raw页或六轴feedback_age都不提供正向成功证据。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field", ["evidence_pages", "feedback_age_us"])
def test_missing_raw_goal_evidence_rejected(evidence, field):
    del evidence["goals"][0][field]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 删首/末页、重复页、篡改字节或外层session错都拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param mutation 参数化变异函数或分支名，逐例只制造指定错误。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("mutation", [
    lambda pages: pages.pop(0),
    lambda pages: pages.pop(),
    lambda pages: pages.append(copy.deepcopy(pages[-1])),
    lambda pages: pages[0].update(raw_hex=pages[0]["raw_hex"][:-2] + "01"),
    lambda pages: pages[0].update(session_id=9),
])
def test_raw_chain_gap_duplicate_tamper_or_wrong_session_rejected(evidence, mutation):
    mutation(evidence["goals"][0]["evidence_pages"])
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 把首组count改2并重签完整链，仍因末总数语义不一致拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_raw_group_count_must_match_terminal_even_with_valid_chain(evidence):
    records = evidence["goals"][0]["evidence_pages"]
    first = bytearray.fromhex(records[0]["raw_hex"])
    struct.pack_into("<I", first, 281, 2)
    first[339:371] = hashlib.sha256(first[307:339] + first[:307] + first[371:372]).digest()
    records[0]["raw_hex"] = first.hex()
    terminal = bytearray.fromhex(records[1]["raw_hex"])
    terminal[307:339] = first[339:371]
    terminal[339:371] = hashlib.sha256(
        terminal[307:339] + terminal[:307] + terminal[371:372]).digest()
    records[1]["raw_hex"] = terminal.hex()
    with pytest.raises(ValueError, match="group count"):
        application_e2e.validate_evidence(evidence)


##
# @brief 页attempt_id99但run4，重签也不能修复事务身份。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_raw_attempt_id_must_match_run_even_with_valid_chain(evidence):
    goal = evidence["goals"][0]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
                                    attempt_id=99)
    with pytest.raises(ValueError, match="attempt"):
        application_e2e.validate_evidence(evidence)


##
# @brief raw sent改62并重签，JSON仍63不能覆盖缺轴事实。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_partial_raw_can_sent_mask_rejected_even_when_json_claims_full(evidence):
    records = evidence["goals"][0]["evidence_pages"]
    first = bytearray.fromhex(records[0]["raw_hex"])
    first[35] = 62
    first[339:371] = hashlib.sha256(first[307:339] + first[:307] + first[371:372]).digest()
    records[0]["raw_hex"] = first.hex()
    terminal = bytearray.fromhex(records[1]["raw_hex"])
    terminal[307:339] = first[339:371]
    terminal[339:371] = hashlib.sha256(
        terminal[307:339] + terminal[:307] + terminal[371:372]).digest()
    records[1]["raw_hex"] = terminal.hex()
    with pytest.raises(ValueError, match="six-axis CAN"):
        application_e2e.validate_evidence(evidence)


##
# @brief 任一轴年龄超10000µs、只有五轴或预算0均拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @param value 该字段的参数化错误值。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field,value", [
    ("feedback_age_us", [1000, 1000, 1000, 1000, 1000, 10001]),
    ("feedback_age_us", [1000] * 5),
    ("feedback_age_budget_us", 0),
])
def test_stale_or_incomplete_six_axis_feedback_rejected(evidence, field, value):
    evidence["goals"][0][field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 缺顶层已确认Hold链接，不接受直接释放成功。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_missing_pre_release_hold_rejected(evidence):
    del evidence["pre_release_hold"]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief confirmed=false、parent错、session错或UNCONFIRMED_STOP终态均拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @param value 该字段的参数化错误值。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field,value", [
    ("confirmed", False), ("parent_run_id", 2), ("session_id", 9),
    ("terminal", "UNCONFIRMED_STOP"),
])
def test_unconfirmed_or_unlinked_release_rejected(evidence, field, value):
    evidence["release"][field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief Release末页缺失不能只靠管理回执或JSON confirmed通过。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_missing_release_terminal_rejected(evidence):
    evidence["release"]["evidence_pages"].pop()
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 反馈行UUID/RUN/session/boot/motion/mask任一漂移必须拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @param value 该字段的参数化错误值。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field,value", [
    ("goal_uuid", "b" * 32), ("run_id", 9), ("session_id", 9),
    ("boot_epoch", 9), ("motion_epoch", 9), ("axis_mask", 1),
])
def test_timeline_event_must_bind_to_same_goal_and_run(evidence, field, value):
    evidence["goals"][0]["timeline"][3][field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 分别删feedback/terminal_success/pre_release_hold，七步因果链不能缺环。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param event 要删除/编码的timeline事件名。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("event", ["feedback", "terminal_success", "pre_release_hold"])
def test_timeline_requires_feedback_terminal_and_hold(evidence, event):
    evidence["goals"][0]["timeline"] = [row for row in
        evidence["goals"][0]["timeline"] if row["event"] != event]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief timeline中第六轴反馈年龄超预算也拒绝，不仅检查goal摘要值。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_timeline_stale_feedback_rejected(evidence):
    evidence["goals"][0]["timeline"][3]["feedback_age_us"][5] = 10001
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 在明确lag预算内，Host收到的反馈可比after_begin事件早50µs，不强制错误的跨回调顺序。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_same_cycle_feedback_receive_time_can_precede_after_begin(evidence):
    evidence["environment"].update(producer_lag_us=50, producer_lag_budget_us=100)
    evidence["goals"][0]["timeline"][3]["steady_us"] = 2950
    application_e2e.validate_evidence(evidence)


##
# @brief v2窗口方法/范围/释放run齐备时保留环境峰值，预算必须符合窗口批准值250000µs，改500000拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_window_lag_version_requires_explicit_method_and_budget(evidence):
    environment = evidence["environment"]
    environment.update(producer_lag_gate_version=2,
                       producer_lag_method="state_receive_minus_produced_offset_span_motion_window",
                       producer_lag_environment_us=300000,
                       producer_lag_window={"start_steady_us": 1000,
                                            "end_steady_us": 7000,
                                            "state_samples": 3,
                                            "release_run_id": evidence["release"]["run_id"]},
                       producer_lag_budget_us=250000)
    application_e2e.validate_evidence(evidence)
    environment["producer_lag_budget_us"] = 500000
    with pytest.raises(ValueError, match="window producer lag"):
        application_e2e.validate_evidence(evidence)


##
# @brief accepted/before_begin和terminal/Hold同一µs允许，因果顺序不要求严格时间递增。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_causal_rows_may_share_microsecond_timestamp(evidence):
    rows = evidence["goals"][0]["timeline"]
    rows[1]["steady_us"] = rows[0]["steady_us"]
    rows[5]["steady_us"] = rows[4]["steady_us"]
    application_e2e.validate_evidence(evidence)


##
# @brief 反馈早于after_begin超允许offset span时拒绝，不任意挪动事件时间。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_feedback_receive_time_outside_lag_budget_rejected(evidence):
    evidence["environment"].update(producer_lag_us=50, producer_lag_budget_us=100)
    evidence["goals"][0]["timeline"][3]["steady_us"] = 2800
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 即使lag预算很大，反馈Host时间仍不能早于目标接纳。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_feedback_receive_time_cannot_predate_goal_acceptance(evidence):
    evidence["environment"].update(producer_lag_us=50, producer_lag_budget_us=3000)
    evidence["goals"][0]["timeline"][3]["steady_us"] = 900
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 新增反馈行接收时间3900晚插在3950后，拒倒序。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_feedback_receive_times_cannot_run_backward(evidence):
    rows = evidence["goals"][0]["timeline"]
    later = copy.deepcopy(rows[3])
    later["steady_us"] = 3900
    rows[3]["steady_us"] = 3950
    rows.insert(4, later)
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 反馈seq1→2及接收4000→4500µs可接受，不局限单反馈行。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_multiple_fresh_feedback_rows_accepted(evidence):
    rows = evidence["goals"][0]["timeline"]
    intermediate = copy.deepcopy(rows[3])
    intermediate["steady_us"] = 4500
    intermediate["feedback_seq"] = [2] * 6
    rows.insert(4, intermediate)
    application_e2e.validate_evidence(evidence)


##
# @brief 新增中间反馈某轴过期仍拒绝，不能只看首末反馈。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_stale_intermediate_feedback_rejected(evidence):
    rows = evidence["goals"][0]["timeline"]
    intermediate = copy.deepcopy(rows[3])
    intermediate["steady_us"] = 4500
    intermediate["feedback_age_us"][0] = 10001
    rows.insert(4, intermediate)
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief post_release仍session3不能确认释放，须清零。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_post_release_status_must_clear_session(evidence):
    evidence["goals"][0]["timeline"][-1]["session_id"] = 3
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief post_release run99不能借子Release RUN5证据。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_post_release_status_must_bind_release_child_run(evidence):
    evidence["goals"][0]["timeline"][-1]["run_id"] = 99
    with pytest.raises(ValueError, match="release.*run"):
        application_e2e.validate_evidence(evidence)


##
# @brief State仍core3时不因Status Idle或强回执就宣称双流释放。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_post_release_status_must_confirm_both_idle(evidence):
    evidence["goals"][0]["timeline"][-1]["state_core_state"] = 3
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 释放前Status是Idle而非Hold，双流Holding链接不成立。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_pre_release_hold_must_confirm_both_hold(evidence):
    evidence["goals"][0]["timeline"][-2]["status_core_state"] = 1
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief source=native_cli不满足本ROS应用证据来源合同，不把Native结果冒充ROS链。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_native_only_claim_rejected(evidence):
    evidence["source"] = "native_cli"
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 末误差/路径误差过大、缺轴/错epoch或Hold未确认任一不因Action SUCCEEDED放行。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param change 修改记录/goal的参数化回调，在当前独立fixture上执行。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("change", [
    lambda goal: goal.update(feedback_rad=[0.1] * 6),
    lambda goal: goal.update(path_error_max_rad=[0.1] * 6),
    lambda goal: goal["groups"][0].update(sent_mask=1),
    lambda goal: goal["groups"][0].update(motion_epoch=1),
    lambda goal: goal.update(hold_confirmed=False),
])
def test_false_success_rejected(evidence, change):
    change(evidence["goals"][0])
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 此构造先缺cancel Gate强键而拒绝；后续旧释放epoch/stop=false负例仍仅断言ValueError，不独立证明首失败字段。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_cancel_with_old_epoch_or_unconfirmed_stop_rejected(evidence):
    goal = copy.deepcopy(evidence["goals"][0])
    evidence["goals"][0]["timeline"].pop()
    goal.update(goal_id="b" * 32, run_id=5, action_status="CANCELED",
                terminal="CANCELED_HOLDING", stop_confirmed=True, stop_epoch=3)
    goal["groups"][0].update(group_id=8, run_id=5, motion_epoch=3)
    goal["evidence_pages"] = pages(evidence["identity"], run=5, session=3,
                                    epoch=2, group_epochs=[3], terminal_epoch=2,
                                    final_state=7, group_id=8)
    for row in goal["timeline"]:
        row["goal_uuid"] = goal["goal_id"]
        row["steady_us"] += 10000
        if row["event"] not in ("accepted", "before_begin"):
            row["run_id"] = 5
    goal["timeline"][4]["event"] = "terminal_canceled"
    goal["timeline"][4]["motion_epoch"] = 3
    goal["timeline"][5]["motion_epoch"] = 3
    evidence["goals"].append(goal)
    evidence["pre_release_hold"].update(run_id=5, motion_epoch=3)
    evidence["release"].update(run_id=6, parent_run_id=5,
        evidence_pages=pages(evidence["identity"], run=6, session=3,
                             epoch=3, kind=3, parent=5, final_state=8, group_id=9))
    goal["timeline"][-1]["run_id"] = 6
    with pytest.raises(ValueError, match="cancel Gate key"):
        application_e2e.validate_evidence(evidence)
    stale_release = copy.deepcopy(evidence)
    stale_release["release"]["evidence_pages"] = pages(evidence["identity"],
        run=6, session=3, epoch=2, kind=3, parent=5, final_state=8, group_id=9)
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(stale_release)
    for changed in (dict(stop_confirmed=False), dict(stop_epoch=2)):
        invalid = copy.deepcopy(evidence)
        invalid["goals"][1].update(changed)
        with pytest.raises(ValueError):
            application_e2e.validate_evidence(invalid)


##
# @brief 虽名称称allows，当前首先期望缺cancel Gate键拒绝；再核stop后回旧运动组拒绝，不能将本例报成正向取消覆盖。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_cancel_allows_motion_group_then_new_epoch_stop_group(evidence):
    goal = copy.deepcopy(evidence["goals"][0])
    evidence["goals"][0]["timeline"].pop()
    goal.update(goal_id="b" * 32, run_id=5, action_status="CANCELED",
                terminal="CANCELED_HOLDING", stop_confirmed=True, stop_epoch=3)
    goal["groups"] = [
        {**goal["groups"][0], "group_id": 8, "run_id": 5, "motion_epoch": 2},
        {**goal["groups"][0], "group_id": 9, "run_id": 5, "motion_epoch": 3},
    ]
    goal["evidence_pages"] = pages(evidence["identity"], run=5, session=3,
        group_ids=[8, 9], group_epochs=[2, 3], terminal_epoch=2, final_state=7)
    for row in goal["timeline"]:
        row["goal_uuid"] = goal["goal_id"]
        row["steady_us"] += 10000
        if row["event"] not in ("accepted", "before_begin"):
            row["run_id"] = 5
    goal["timeline"][4]["event"] = "terminal_canceled"
    goal["timeline"][4]["motion_epoch"] = 3
    goal["timeline"][5]["motion_epoch"] = 3
    evidence["goals"].append(goal)
    evidence["pre_release_hold"].update(run_id=5, motion_epoch=3)
    evidence["release"].update(run_id=6, parent_run_id=5,
        evidence_pages=pages(evidence["identity"], run=6, session=3,
                             epoch=3, kind=3, parent=5, final_state=8, group_id=10))
    goal["timeline"][-1]["run_id"] = 6
    with pytest.raises(ValueError, match="cancel Gate key"):
        application_e2e.validate_evidence(evidence)
    invalid = copy.deepcopy(evidence)
    invalid["goals"][1]["groups"].append({**goal["groups"][0], "group_id": 10})
    invalid["goals"][1]["evidence_pages"] = pages(evidence["identity"], run=5,
        session=3, group_ids=[8, 9, 10], group_epochs=[2, 3, 2],
        terminal_epoch=2, final_state=7)
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(invalid)


##
# @brief 当前取消报告缺cancel Gate键，要求该明确错误；不因stop_confirmed字符串接受，不能据此单独定位raw缺失门。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_report_without_stop_raw_is_not_positive_motion_evidence(evidence):
    goal = evidence["goals"][0]
    goal.update(action_status="CANCELED", terminal="CANCELED_HOLDING",
                stop_confirmed=True, stop_epoch=3)
    goal["groups"][0]["motion_epoch"] = 3
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
                                    group_epochs=[3], terminal_epoch=2, final_state=7)
    goal["timeline"][4]["event"] = "terminal_canceled"
    goal["timeline"][4]["motion_epoch"] = 3
    goal["timeline"][5]["motion_epoch"] = 3
    evidence["pre_release_hold"]["motion_epoch"] = 3
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=3, kind=3, parent=4, final_state=8, group_id=8)
    with pytest.raises(ValueError, match="cancel Gate key"):
        application_e2e.validate_evidence(evidence)


##
# @brief 构造Cancel9原请求epoch2、强结果epoch3/同RUN4、安全组、Gate取消键1/1及后续显式Release子RUN5/epoch4。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return 原evidence对象，Cancel强键/安全页/释放链接已补齐。
def canceled_chain(evidence):
    goal = evidence["goals"][0]
    goal.update(action_status="CANCELED", terminal="CANCELED_HOLDING",
                stop_confirmed=True, stop_epoch=3,
                cancel_request_id=1, cancel_operation_id=1)
    goal["groups"] = [
        {**goal["groups"][0], "motion_epoch": 2},
        {**goal["groups"][0], "group_id": 8, "motion_epoch": 3},
    ]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8], group_epochs=[2, 3], terminal_epoch=2,
        categories=[1, 2], final_state=7)
    goal["timeline"][4].update(event="terminal_canceled", motion_epoch=3)
    for row in goal["timeline"]:
        row.update(cancel_request_id=0, cancel_operation_id=0)
    goal["timeline"][4].update(cancel_request_id=1, cancel_operation_id=1)
    goal["timeline"][5]["motion_epoch"] = 3
    evidence["pre_release_hold"]["motion_epoch"] = 3
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=3, kind=3, parent=4, final_state=8,
        group_id=9, group_epochs=[4], terminal_epoch=3, categories=[3])
    goal["timeline"][-1]["motion_epoch"] = 4
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=4)
    evidence["management_records"][0] = manage(4, 4800, operation=9,
        request_id=1, expected_motion_epoch=2)
    evidence["management_records"][1] = manage(5, 5000, request_id=1, run=4,
        motion_epoch=3, motion_complete=0)
    return evidence


##
# @brief 完整合成取消链必须具新epoch安全组和独立Release后才能通过。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_chain_requires_new_epoch_stop_and_explicit_release(evidence):
    application_e2e.validate_evidence(canceled_chain(evidence))


##
# @brief 取消路径也受v2 lag250000µs门，300000µs不能因停止被放宽。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_window_gate_rejects_over_budget_lag(evidence):
    canceled_chain(evidence)
    evidence["environment"].update(
        producer_lag_us=300000, producer_lag_budget_us=250000,
        producer_lag_gate_version=2,
        producer_lag_method="state_receive_minus_produced_offset_span_motion_window",
        producer_lag_environment_us=300000,
        producer_lag_window={"start_steady_us": 1000,
                             "end_steady_us": 7000,
                             "state_samples": 4,
                             "release_run_id": evidence["release"]["run_id"]})
    with pytest.raises(ValueError, match="producer lag budget"):
        application_e2e.validate_evidence(evidence)


##
# @brief Gate terminal4900先于Cancel强结果5000不成立，不能事后补强回执。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_chain_rejects_commit_after_gate_terminal(evidence):
    canceled_chain(evidence)
    evidence["goals"][0]["timeline"][4]["steady_us"] = 4900
    with pytest.raises(ValueError, match="cancel committed receipt follows Gate terminal"):
        application_e2e.validate_evidence(evidence)


##
# @brief accepted旧epoch1与取消链原锚不符须拒绝。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_chain_rejects_accepted_epoch_mismatch(evidence):
    canceled_chain(evidence)
    evidence["goals"][0]["timeline"][0]["motion_epoch"] = 1
    with pytest.raises(ValueError, match="accepted motion epoch"):
        application_e2e.validate_evidence(evidence)


##
# @brief 把Stop整体改到epoch4并同步回执/Release仍拒绝，取消只能进下一epoch3。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_chain_rejects_skipped_stop_epoch_even_with_matching_receipts(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    goal["stop_epoch"] = 4
    goal["groups"][1]["motion_epoch"] = 4
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8], group_epochs=[2, 4], terminal_epoch=2,
        categories=[1, 2], final_state=7)
    goal["timeline"][4]["motion_epoch"] = 4
    goal["timeline"][5]["motion_epoch"] = 4
    goal["timeline"][-1]["motion_epoch"] = 5
    evidence["pre_release_hold"]["motion_epoch"] = 4
    evidence["release"]["evidence_pages"] = pages(evidence["identity"],
        run=5, session=3, epoch=4, kind=3, parent=4, final_state=8,
        group_id=9, group_epochs=[5], terminal_epoch=4, categories=[3])
    evidence["management_records"][1] = manage(5, 5000, request_id=1,
        run=4, motion_epoch=4, motion_complete=0)
    evidence["management_records"][-1] = manage(5, 6700, request_id=2,
        run=5, session=0, operation_id=2, motion_epoch=5)
    with pytest.raises(ValueError, match="cancel stop epoch must advance once"):
        application_e2e.validate_evidence(evidence)


##
# @brief Cancel请求expected_motion1而目标运行2，不能借后续强结果洗掉原锚错误。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_chain_rejects_cancel_request_bound_to_old_motion_epoch(evidence):
    canceled_chain(evidence)
    evidence["management_records"][0] = manage(4, 4800, operation=9,
        request_id=1, expected_motion_epoch=1)
    with pytest.raises(ValueError, match="cancel request motion epoch"):
        application_e2e.validate_evidence(evidence)


##
# @brief Cancel请求后出现新Stop epoch反馈、seq递增且早于Action结果是合法时序。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_goal_allows_monotonic_stop_epoch_feedback_before_action_result(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    goal["timeline"].insert(4, {**goal["timeline"][3],
        "steady_us": 4900, "motion_epoch": 3, "feedback_seq": [2] * 6})
    application_e2e.validate_evidence(evidence)


##
# @brief 反馈已进入Stop3后再回运动2，即使seq继续增也拒绝旧阶段复活。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_goal_rejects_motion_epoch_after_stop_feedback(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    goal["timeline"].insert(4, {**goal["timeline"][3],
        "steady_us": 4700, "motion_epoch": 3, "feedback_seq": [2] * 6})
    goal["timeline"].insert(5, {**goal["timeline"][3],
        "steady_us": 4900, "motion_epoch": 2, "feedback_seq": [3] * 6})
    with pytest.raises(ValueError, match="old motion epoch resumed"):
        application_e2e.validate_evidence(evidence)


##
# @brief Stop epoch反馈早于Cancel请求，不可用未来管理命令解释已出现的阶段。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_goal_rejects_stop_feedback_before_cancel_request(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    goal["timeline"].insert(4, {**goal["timeline"][3],
        "steady_us": 4700, "motion_epoch": 3, "feedback_seq": [2] * 6})
    with pytest.raises(ValueError, match="stop feedback precedes cancel request"):
        application_e2e.validate_evidence(evidence)


##
# @brief 在完整取消链加birth Hold1、Run2、Safety3、固定Hold3，终页仍birth1。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return 原evidence对象，含前后普通Hold及必须的Safety组。
def canceled_chain_with_hold_refresh(evidence):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    original = goal["groups"][0]
    goal["groups"] = [{**original, "group_id": group_id, "motion_epoch": epoch}
        for group_id, epoch in ((7, 1), (8, 2), (9, 3), (10, 3))]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8, 9, 10], group_epochs=[1, 2, 3, 3],
        terminal_epoch=1, categories=[1, 1, 2, 1], final_state=7)
    goal["timeline"][0]["motion_epoch"] = 1
    goal["timeline"][1]["motion_epoch"] = 1
    return evidence


##
# @brief 合成前置及Stop后固定Hold刷新允许，但角色/原请求仍须闭合。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_chain_accepts_prebegin_and_post_stop_fixed_hold_refresh(evidence):
    application_e2e.validate_evidence(canceled_chain_with_hold_refresh(evidence))


##
# @brief 把Safety类别全改普通Hold即使epoch相同仍拒绝，不能拿刷新代替安全停组。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_chain_still_requires_safety_group_before_hold_refresh(evidence):
    canceled_chain_with_hold_refresh(evidence)
    goal = evidence["goals"][0]
    goal["evidence_pages"] = pages(evidence["identity"], run=4, session=3,
        group_ids=[7, 8, 9, 10], group_epochs=[1, 2, 3, 3],
        terminal_epoch=1, categories=[1, 1, 1, 1], final_state=7)
    with pytest.raises(ValueError, match="Hold SENT"):
        application_e2e.validate_evidence(evidence)


##
# @brief Complete/Release响应同时置motion_complete与release_committed为1须拒绝语义冲突。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param operation 8 Complete或10 Release，分别改相应强回执。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("operation", (8, 10))
def test_committed_receipt_rejects_conflicting_terminal_flags(evidence, operation):
    index = 1 if operation == 8 else -1
    original = evidence["management_records"][index]
    evidence["management_records"][index] = manage(5, original["when"],
        request_id=1 if operation == 8 else 2,
        operation_id=1 if operation == 8 else 2,
        run=4 if operation == 8 else 5,
        session=3 if operation == 8 else 0,
        motion_complete=1, release_committed=1)
    with pytest.raises(ValueError, match="management receipt"):
        application_e2e.validate_evidence(evidence)


##
# @brief 16类取消/释放缺失或错身份负例逐一拒绝，包含Gate键、Pending、缺轴/末页、无Disable和跳RUN。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param mutation 参数化变异函数或分支名，逐例只制造指定错误。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("mutation", [
    "pending_cancel", "wrong_cancel_run", "old_cancel_key", "old_cancel_operation",
    "missing_stop_axis",
    "missing_cancel_terminal", "missing_release_receipt", "missing_release_page",
    "release_without_disable", "active_cancel_release", "skipped_release_run",
    "cancel_motion_complete", "cancel_release_committed",
    "old_timeline_cancel_key", "old_timeline_cancel_operation", "missing_timeline_cancel_key",
])
def test_canceled_chain_rejects_incomplete_raw_proof(evidence, mutation):
    canceled_chain(evidence)
    goal = evidence["goals"][0]
    if mutation == "pending_cancel":
        evidence["management_records"][1] = manage(5, 5000, request_id=1,
            run=4, phase=2, motion_epoch=3)
    elif mutation == "wrong_cancel_run":
        evidence["management_records"][1] = manage(5, 5000, request_id=1,
            run=6, motion_epoch=3)
    elif mutation == "cancel_motion_complete":
        evidence["management_records"][1] = manage(5, 5000, request_id=1,
            run=4, motion_epoch=3, motion_complete=1)
    elif mutation == "cancel_release_committed":
        evidence["management_records"][1] = manage(5, 5000, request_id=1,
            run=4, motion_epoch=3, motion_complete=0, release_committed=1)
    elif mutation == "old_cancel_key":
        goal["cancel_request_id"] = 99
    elif mutation == "old_cancel_operation":
        goal["cancel_operation_id"] = 99
    elif mutation == "old_timeline_cancel_key":
        goal["timeline"][4]["cancel_request_id"] = 99
    elif mutation == "old_timeline_cancel_operation":
        goal["timeline"][4]["cancel_operation_id"] = 99
    elif mutation == "missing_timeline_cancel_key":
        del goal["timeline"][4]["cancel_request_id"]
    elif mutation == "missing_stop_axis":
        raw = bytearray.fromhex(goal["evidence_pages"][1]["raw_hex"])
        raw[34] = 1
        goal["evidence_pages"][1]["raw_hex"] = raw.hex()
    elif mutation == "missing_cancel_terminal":
        goal["evidence_pages"].pop()
    elif mutation == "missing_release_receipt":
        evidence["management_records"].pop()
    elif mutation == "release_without_disable":
        evidence["release"]["evidence_pages"] = pages(evidence["identity"],
            run=5, session=3, epoch=3, kind=3, parent=4, final_state=8,
            group_id=9, categories=[1])
    elif mutation == "active_cancel_release":
        evidence["release"].update(terminal="CANCELED_AND_RELEASED",
            evidence_pages=pages(evidence["identity"], run=5, session=3,
                epoch=3, kind=1, parent=0, final_state=2, group_ids=[9, 10],
                categories=[1, 3]))
    elif mutation == "skipped_release_run":
        evidence["release"]["run_id"] = 6
        goal["timeline"][-1]["run_id"] = 6
        evidence["release"]["evidence_pages"] = pages(evidence["identity"],
            run=6, session=3, epoch=3, kind=3, parent=4, final_state=8,
            group_id=9, categories=[3])
        evidence["management_records"][-1] = manage(5, 6700, request_id=2,
            run=6, session=0, operation_id=2)
    else:
        evidence["release"]["evidence_pages"].pop()
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 超lag而标fault_domain=DEVICE不接受，环境执行问题不能冒充设备故障归因。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_environment_lag_cannot_be_called_device_fault(evidence):
    evidence["environment"].update(producer_lag_us=20000)
    evidence["goals"][0].update(action_status="ABORTED", result_error_code=-4,
                                terminal="PRODUCER_LAG", fault_domain="DEVICE")
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


##
# @brief 明确ENVIRONMENT的producer lag中止仍不能作为设备成功证据。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_environment_only_fault_cannot_be_reported_as_device_success(evidence):
    evidence["environment"]["producer_lag_us"] = 20000
    evidence["goals"][0].update(action_status="ABORTED", result_error_code=-4,
                                terminal="PRODUCER_LAG", fault_domain="ENVIRONMENT")
    with pytest.raises(ValueError, match="environment-only"):
        application_e2e.validate_evidence(evidence)


##
# @brief 环境中止且缺timeline应ValueError，不泄漏内部异常或忽略释放链。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_environment_abort_without_release_timeline_fails_as_validation_error(evidence):
    evidence["environment"]["producer_lag_us"] = 20000
    goal = evidence["goals"][0]
    goal.update(action_status="ABORTED", result_error_code=-4,
                terminal="PRODUCER_LAG", fault_domain="ENVIRONMENT")
    del goal["timeline"]
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)




##
# @brief environment非dict、goal=None或group=None形状异常均须ValueError。
# @param evidence evidence fixture返回的可变合成证据字典；每例独立，不来自设备采集。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @param value 该字段的参数化错误值。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field,value", [("environment", []), ("goals", [None]),
                                         ("goals", [{"goal_id": "a" * 32, "groups": [None]}])])
def test_malformed_evidence_fails_as_validation_error(evidence, field, value):
    evidence[field] = value
    with pytest.raises(ValueError):
        application_e2e.validate_evidence(evidence)


def test_current_selected_schema_five_keeps_full_evidence_validation(evidence):
    """当前页由公共encode构造，独立SHA链及管理准入仍执行完整正向门。"""
    from selected_protocol import current_layout, decode, evidence_parts

    def values_for(layout, fields):
        result = {}
        for name, value in fields.items():
            field = layout.field(name)
            result[name] = (bytes.fromhex(value) if field.array and field.type == 'uint8'
                            else bool(value) if field.type == 'bool' else value)
        result['schema_minor'] = layout.constant('SCHEMA_MINOR')
        return result

    for record in evidence['management_records']:
        name = 'DummyManageV2Request' if record['kind'] == 4 else 'DummyManageV2Response'
        layout = current_layout(name)
        values = values_for(layout, decode(name, bytes.fromhex(record['raw_hex'])))
        record['raw_hex'] = layout.encode(values).hex()
    layout = current_layout('DummyAttemptEvidenceV2')
    for attempt in [*evidence['goals'], evidence['release']]:
        seed = struct.pack('<QQ', attempt['run_id'], attempt['boot_epoch']) + bytes.fromhex(evidence['identity']['client_id'])
        seed += b''.join(bytes.fromhex(evidence['identity'][name]) for name in
                         ('contract_hash', 'model_hash', 'profile_hash', 'firmware_hash'))
        previous = hashlib.sha256(seed.ljust(192, b'\0')).digest()
        for page in attempt['evidence_pages']:
            values = values_for(layout, decode('DummyAttemptEvidenceV2', bytes.fromhex(page['raw_hex'])))
            values['previous_chain_sha256'] = previous
            raw = layout.encode(values)
            parts = evidence_parts(raw)
            digest = hashlib.sha256(previous + parts['prefix'] + parts['lost']).digest()
            values['chain_sha256'] = digest
            page['raw_hex'] = layout.encode(values).hex()
            previous = digest
    assert application_e2e.validate_evidence(evidence) is None
