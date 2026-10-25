"""离线生成真实六轴 ROS 证据上下文的契约测试。"""
##
# @file test_build_real_6dof_context.py
# @brief 合成DPRAW01/CSV/Action文件的context构建、时钟窗口和取消过渡绑定回归。
#
# 所有记录均用wire/record在临时文件人工生成，未采集ROS/USB/设备。Host接收when与Source produced_at_us均为µs但不同域；lag取receive−produced偏移的跨度，不是绝对时钟同步或单向延迟。基础夹具仅测context，不等完整应用证据；旧window用1s State容差，dense专用严格250ms。

import csv
import hashlib
import json
from pathlib import Path
import struct

import pytest

from scripts.dummy import build_real_6dof_context as builder
from scripts.dummy import join_real_6dof_evidence as joiner


##
# @brief 按joiner.LAYOUT打包给定消息小端字节，schema默认当前2.5，其它未给字段留零；不调用设备codec发送。
# @param name LAYOUT payload消息类型名。
# @param values 字段名到值字典，数组hex先转bytes，其它按字段类型小端写入。
# @return 固定encoded_size的原wire bytes。
def wire(name, **values):
    layout = joiner.LAYOUT["payloads"][name]
    raw = bytearray(layout["encoded_size"])
    types = {"uint8": "B", "uint16": "H", "uint32": "I", "uint64": "Q",
             "float64": "d", "bool": "B"}
    for field in layout["fields"]:
        value = {"schema_major": 2, "schema_minor": 5, **values}.get(field["name"])
        if value is None:
            continue
        if field["count"] > 1:
            value = bytes.fromhex(value) if isinstance(value, str) else value
        else:
            value = [value]
        struct.pack_into("<" + types[field["type"]] * field["count"], raw,
                         field["offset"], *value)
    return bytes(raw)


##
# @brief 把kind/长度/Host接收µs/外层boot/session/run/seq与payload拼成一条DPRAW01记录，不加文件magic。
# @param kind DPRAW种类：1 Evidence、2 State、3 Status、4 Manage请求、5响应。
# @param when 本条Host接收/记录单调时间µs，不是Source生产时间。
# @param boot 外层boot epoch，默认场景为2。
# @param session 本条owner session ID，0表示已释放。
# @param run 设备attempt/RUN ID，和ROS goal UUID不同。
# @param seq DPRAW外层序号；含义随kind为request_seq/evidence_seq等。
# @param payload 待拼接的原wire bytes，不自动校验其内层身份。
# @return header+payload bytes，可拼入DPRAW01文件。
def record(kind, when, boot, session, run, seq, payload):
    return joiner.RECORD_HEADER.pack(kind, len(payload), when, boot, session,
                                     run, seq) + payload


##
# @brief 临时写最小COMPLETE CSV、ROS_ACTION_ONLY JSON及两State/管理记录，Source时间刻意与Host不同域。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return 临时四路径元组，输出尚未创建。
@pytest.fixture
def capture(tmp_path):
    timeline = tmp_path / "timeline.csv"
    timeline.write_text("event,steady_us\nCOMPLETE,1\n")
    action = tmp_path / "action.json"
    action.write_text('{"scope":"ROS_ACTION_ONLY"}\n')
    client = "11" * 16
    request = wire("DummyManageV2Request", client_instance_id=client,
                   expected_boot_epoch=2, expected_session_id=3, request_seq=1)
    response = wire("DummyManageV2Response", client_instance_id=client,
                    boot_epoch=2, session_id=3)
    states = [wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                   produced_at_us=produced) for produced in (1090, 1180)]
    raw = tmp_path / "raw.bin"
    raw.write_bytes(b"DPRAW01\0" +
                    record(4, 95, 2, 3, 0, 1, request) +
                    record(2, 100, 2, 3, 0, 0, states[0]) +
                    record(5, 150, 2, 3, 0, 0, response) +
                    record(2, 200, 2, 3, 0, 0, states[1]))
    return timeline, raw, action, tmp_path / "context.json"


##
# @brief 对基础capture调用生产build_context，固定路径/目标容差及显式lag/age预算，不做ROS运行。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return 生产builder返回并写入的context字典。
def build(capture):
    timeline, raw, action, output = capture
    return builder.build_context(
        timeline, raw, action, output,
        path_tolerance_rad=[0.01] * 6, goal_tolerance_rad=[0.003] * 6,
        producer_lag_budget_us=500000, feedback_age_budget_us=10000,
        ros_feedback_lag_budget_us=2000, gate_state_max_age_us=500000)


##
# @brief 核client、观测offset跨度10µs、预算与三输入原SHA，以及输出JSON等于返回context。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_builds_context_from_parsed_records_and_pins_inputs(capture):
    context = build(capture)
    timeline, raw, action, output = capture
    assert context["client_id"] == "11" * 16
    assert context["producer_lag_us"] == 10
    assert context["producer_lag_method"] == "state_receive_minus_produced_offset_span"
    assert context["producer_lag_budget_us"] == 500000
    assert context["path_tolerance_rad"] == [0.01] * 6
    for name, path in (("timeline", timeline), ("raw", raw), ("action", action)):
        assert context[f"{name}_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert json.loads(output.read_text()) == context


##
# @brief 只替换raw首次client为另一ID导致管理两向不一致，拒绝且无输出。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_rejects_mismatched_client_without_output(capture):
    raw = capture[1]
    source = raw.read_bytes()
    raw.write_bytes(source.replace(bytes.fromhex("11" * 16),
                                   bytes.fromhex("22" * 16), 1))
    with pytest.raises(ValueError, match="client"):
        build(capture)
    assert not capture[3].exists()


##
# @brief 删所有kind2 State后不能计算context，必须拒绝且无输出。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_rejects_empty_state_stream(capture):
    raw = capture[1]
    records = joiner._raw_records(raw.read_bytes())
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], item["raw"])
        for item in records if item["kind"] != 2))
    with pytest.raises(ValueError, match="State"):
        build(capture)
    assert not capture[3].exists()


##
# @brief raw记录Host接收时间倒序拒绝，不按文件顺序默许逆因果。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_rejects_nonmonotonic_receive_times(capture):
    raw = capture[1]
    records = joiner._raw_records(raw.read_bytes())
    reordered = [records[0], records[2], records[1], records[3]]
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], item["raw"]) for item in reordered))
    with pytest.raises(ValueError, match="nonmonotonic"):
        build(capture)
    assert not capture[3].exists()


##
# @brief 后一State produced从1180退1080拒绝Source时钟回退，即使Host仍增。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_rejects_nonmonotonic_state_production_times(capture):
    raw = capture[1]
    records = joiner._raw_records(raw.read_bytes())
    earlier = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                   produced_at_us=1080)
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], earlier if index == 3 else item["raw"])
        for index, item in enumerate(records)))
    with pytest.raises(ValueError, match="nonmonotonic DPRAW01 State"):
        build(capture)
    assert not capture[3].exists()


##
# @brief 观测跨度10µs而预算9µs必须拒绝且不落context。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_rejects_budget_below_observed_lag(capture):
    timeline, raw, action, output = capture
    with pytest.raises(ValueError, match="producer lag budget"):
        builder.build_context(
            timeline, raw, action, output,
            path_tolerance_rad=[0.01] * 6, goal_tolerance_rad=[0.003] * 6,
            producer_lag_budget_us=9, feedback_age_budget_us=10000,
            ros_feedback_lag_budget_us=2000, gate_state_max_age_us=500000)
    assert not output.exists()


##
# @brief 偏移10000/5000/9000µs跨度5000，首点高值不能被滑动基线漏掉；4999预算拒绝。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_first_high_offset_still_counts_full_observed_span(capture):
    raw = capture[1]
    records = joiner._raw_records(raw.read_bytes())
    states = [(20000, 10000), (30000, 25000), (40000, 31000)]
    raw.write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], item["raw"])
        for item in records if item["kind"] in (4, 5)) + b"".join(
        record(2, when, 2, 3, 0, 0, wire("DummyJointStateV2",
            boot_epoch=2, session_id=3, produced_at_us=produced))
        for when, produced in states))
    context = build(capture)
    assert context["producer_lag_us"] == 5000
    assert context["producer_lag_method"] == "state_receive_minus_produced_offset_span"
    timeline, raw, action, output = capture
    with pytest.raises(ValueError, match="producer lag budget"):
        builder.build_context(
            timeline, raw, action, output.with_name("tight-context.json"),
            path_tolerance_rad=[0.01] * 6, goal_tolerance_rad=[0.003] * 6,
            producer_lag_budget_us=4999, feedback_age_budget_us=10000,
            ros_feedback_lag_budget_us=2000, gate_state_max_age_us=500000)


##
# @brief 既有context文件排他创建失败，原JSON字节保持。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_existing_output_is_preserved(capture):
    output = capture[3]
    output.write_text('{"existing":true}\n')
    with pytest.raises(FileExistsError):
        build(capture)
    assert output.read_text() == '{"existing":true}\n'


##
# @brief 直接调用builder.main解析CLI列表，核stdout/输出预算；仅写离线context，不启动进程或设备。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @param capsys pytest stdout/stderr捕获fixture。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_cli_writes_explicit_budgets(capture, capsys):
    timeline, raw, action, output = capture
    assert builder.main([
        "--timeline", str(timeline), "--raw", str(raw),
        "--action", str(action), "--output", str(output),
        "--path-tolerance-rad", "0.01,0.01,0.01,0.01,0.01,0.01",
        "--goal-tolerance-rad", "0.003,0.003,0.003,0.003,0.003,0.003",
        "--producer-lag-budget-us", "500000",
        "--feedback-age-budget-us", "10000",
        "--ros-feedback-lag-budget-us", "2000",
        "--gate-state-max-age-us", "500000",
    ]) == 0
    cli_result = json.loads(capsys.readouterr().out)
    assert cli_result["producer_lag_us"] == 10
    assert cli_result["producer_lag_method"] == "state_receive_minus_produced_offset_span"
    assert json.loads(output.read_text())["gate_state_max_age_us"] == 500000


##
# @brief 构造运动前/中/释放后七个State、目标timeline、Release请求/强结果/末页/双Idle，开关逐项注入身份与缺失。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param offsets 七State的receive−produced偏移扰动µs，决定窗口内/全环境跨度。
# @param release 是否写Release请求/强结果/终页。
# @param idle 是否写释放Idle Status。
# @param other_boot 在第四State注入boot3。
# @param goal 是否写接纳到释放timeline；false用于缺目标负例。
# @param queried_reply true使用Query RID关联Release原请求，否则直接响应。
# @param other_device 在第四State注入device33。
# @param other_epoch 在第四State注入motion99。
# @param second_goal 构造两目标、第二motion2与最终Release3。
# @param early_second_epoch 第二motion提前出现在前一目标期间。
# @return 临时(timeline, raw, action, output)四路径元组。
def window_capture(tmp_path, offsets=(400000, 0, 100, 200, 300, 400, 400000),
                   *, release=True, idle=True, other_boot=False, goal=True,
                   queried_reply=False, other_device=False, other_epoch=False,
                   second_goal=False, early_second_epoch=False):
    """Small raw/timeline pair with State samples before, during, and after motion."""
    timeline = tmp_path / "timeline.csv"
    header = ("event,goal_uuid,trajectory_sha256,steady_us,boot,session,"
              "reference_epoch,motion_epoch,run_id,axis_mask,status_core_state,"
              "state_core_state,release_request_id,release_operation_id")
    ##
    # @par 内部回调 row
    # 构造14列CSV行，session0自动reference0，其它默认ref1；位置无测量，只有身份/时间角色。
    # 参数 event：要删除/编码的timeline事件名。
    # 参数 when：本条Host接收/记录单调时间µs，不是Source生产时间。
    # 参数 session：本条owner session ID，0表示已释放。
    # 参数 run：设备attempt/RUN ID，和ROS goal UUID不同。
    # 参数 request：CSV释放request_id，默认0。
    # 参数 operation：CSV释放operation_id，默认0。
    # 参数 motion：CSV motion epoch，默认1。
    # 返回：一行CSV文本，不含行尾换行。
    def row(event, when, session=3, run=7, request=0, operation=0, motion=1):
        reference = 0 if session == 0 else 1
        return f"{event},{'a' * 32},{'b' * 64},{when},2,{session},{reference},{motion},{run},63,4,4,{request},{operation}"
    first_run = 6 if second_goal else 7
    rows = [row("accepted", 1000000, run=first_run),
            row("before_begin", 1000000, run=first_run),
            row("after_begin", 1100000, run=first_run),
            row("feedback", 1200000, run=first_run),
            row("terminal_success", 2500000, run=first_run),
            row("pre_release_hold", 2600000, run=first_run)] if goal else []
    if second_goal and goal:
        rows.extend((row("accepted", 2600000, motion=2),
                     row("before_begin", 2600000, motion=2),
                     row("after_begin", 2650000, motion=2),
                     row("feedback", 2700000, motion=2),
                     row("terminal_success", 3000000, motion=2),
                     row("pre_release_hold", 3100000, motion=2)))
    if goal:
        rows.append(row("post_release_status", 4000000, 0, 8, 9, 10,
                        motion=3 if second_goal else 1))
    timeline.write_text("\n".join([header, *rows,
                                   f"COMPLETE,{2 if second_goal else 1}", ""]))
    action = tmp_path / "action.json"
    action.write_text('{"scope":"ROS_ACTION_ONLY"}\n')
    client = "11" * 16
    request = wire("DummyManageV2Request", client_instance_id=client,
                   device_id="22" * 16,
                   expected_boot_epoch=2, expected_session_id=3,
                   request_seq=1)
    release_request = wire("DummyManageV2Request", client_instance_id=client,
                           device_id="22" * 16,
                           expected_boot_epoch=2, expected_session_id=3,
                           request_seq=2, request_id=9, operation=10)
    response = wire("DummyManageV2Response", client_instance_id=client,
                    device_id="22" * 16,
                    boot_epoch=2, session_id=0,
                    request_id=11 if queried_reply else 9, operation_id=10,
                    run_id=8, transaction_phase=3, result_code=0,
                    release_committed=1)
    query_request = wire("DummyManageV2Request", client_instance_id=client,
                         expected_boot_epoch=2, expected_session_id=3,
                         request_seq=3, request_id=11, operation=12,
                         queried_operation_id=9)
    terminal = wire("DummyAttemptEvidenceV2", run_id=8, attempt_id=8,
                    category=4, final=1, final_state=8, release_committed=1)
    status = wire("DummyDeviceStatusV2", boot_epoch=2, session_id=0,
                  device_id="22" * 16, reference_epoch=0,
                  motion_epoch=3 if second_goal else 1,
                  run_id=8, core_state=1, release_committed=1, operation_id=10,
                  final_state=8, attempt_kind=3)
    events = [(4, 500000, 2, 3, 0, 1, request)]
    for index, (when, offset) in enumerate(zip((700000, 1300000, 1800000,
                                                 2700000, 3300000, 3900000,
                                                 4500000), offsets)):
        session = 0 if when >= 3800000 else 3
        state = wire("DummyJointStateV2", boot_epoch=3 if other_boot and index == 3 else 2,
                     session_id=session, produced_at_us=when + 100000 - offset,
                     device_id=("33" if other_device and index == 3 else "22") * 16,
                     reference_epoch=0 if session == 0 else 1,
                     motion_epoch=(99 if other_epoch and index == 3 else
                                   3 if second_goal and session == 0 else
                                   2 if early_second_epoch and index in (2, 3) else
                                   2 if second_goal and index == 4 else 1),
                     core_state=1 if session == 0 else 4)
        events.append((2, when, 3 if other_boot and index == 3 else 2,
                       session, 0, 0, state))
    if release:
        events.extend([(4, 3150000 if second_goal else 2900000,
                        2, 3, 0, 2, release_request),
                       (1, 3400000, 2, 3, 8, 0, terminal),
                       (5, 3500000, 2, 0, 8, 0, response)])
        if queried_reply:
            events.append((4, 3450000, 2, 3, 0, 3, query_request))
    if idle:
        events.append((3, 3800000, 2, 0, 8, 0, status))
    raw = tmp_path / "raw.bin"
    raw.write_bytes(b"DPRAW01\0" + b"".join(
        record(*event) for event in sorted(events, key=lambda event: event[1])))
    return timeline, raw, action, tmp_path / "context.json"


##
# @brief 以motion_window_v2和producer预算250ms构建；此旧夹具Gate State容差显式1s，不冒充250ms覆盖。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return 以旧1s State容差构建的v2窗口context字典。
def build_window(capture):
    return builder.build_context(
        *capture, path_tolerance_rad=[0.01] * 6,
        goal_tolerance_rad=[0.003] * 6, producer_lag_budget_us=250000,
        feedback_age_budget_us=10000, ros_feedback_lag_budget_us=2000,
        gate_state_max_age_us=1000000, producer_lag_scope="motion_window_v2")


##
# @brief 把成功window改取消epoch1→2、Cancel请求/强结果和安全页；可注入过早/歧义/旧State及缺Stop页。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param early_stop_epoch 提前在Cancel前出现Stop2 State。
# @param stop_page 是否写绑定安全Stop页，默认true。
# @param ambiguous_stop_epoch 请求后页前插新epoch State以核可等待歧义。
# @param ambiguous_only 删确定旧State，只留不足覆盖的歧义样本。
# @param stale_after_page 安全页后再插旧motion1 State。
# @return 原capture四路径，timeline/raw已原地改写。
def canceled_window_capture(tmp_path, *, early_stop_epoch=False, stop_page=True,
                            ambiguous_stop_epoch=False, ambiguous_only=False,
                            stale_after_page=False):
    capture = window_capture(tmp_path)
    timeline, raw, _, _ = capture
    text = timeline.read_text()
    text = text.replace("release_request_id,release_operation_id",
                        "release_request_id,release_operation_id,cancel_request_id,cancel_operation_id")
    rows = text.splitlines()
    for index in range(1, len(rows) - 1):
        if rows[index].startswith("terminal_success,"):
            rows[index] = rows[index].replace("terminal_success,", "terminal_canceled,", 1)
            columns = rows[index].split(",")
            columns[7] = "2"
            rows[index] = ",".join(columns) + ",7,7"
        elif rows[index].startswith("pre_release_hold,"):
            columns = rows[index].split(",")
            columns[7] = "2"
            rows[index] = ",".join(columns) + ",0,0"
        elif rows[index].startswith("post_release_status,"):
            columns = rows[index].split(",")
            columns[7] = "3"
            rows[index] = ",".join(columns) + ",0,0"
        else:
            rows[index] += ",0,0"
    timeline.write_text("\n".join(rows) + "\n")
    records = joiner._raw_records(raw.read_bytes())
    rewritten = []
    for item in records:
        payload = item["raw"]
        if item["kind"] == 2:
            fields = item["fields"]
            stop_epoch = item["when"] >= 2100000 or (
                early_stop_epoch and item["when"] == 1800000)
            if item["session"] == 3 and stop_epoch:
                payload = wire("DummyJointStateV2", **{
                    **fields, "motion_epoch": 2})
            elif item["session"] == 0:
                payload = wire("DummyJointStateV2", **{
                    **fields, "motion_epoch": 3})
        elif item["kind"] == 3:
            payload = wire("DummyDeviceStatusV2", **{
                **item["fields"], "motion_epoch": 3})
        rewritten.append(record(item["kind"], item["when"], item["boot"],
                                item["session"], item["run"], item["seq"], payload))
    cancel = wire("DummyManageV2Request", client_instance_id="11" * 16,
                  device_id="22" * 16, expected_boot_epoch=2,
                  expected_session_id=3, expected_motion_epoch=1,
                  request_seq=7, request_id=7, operation=9)
    rewritten.append(record(4, 2000000, 2, 3, 0, 7, cancel))
    receipt = wire("DummyManageV2Response", client_instance_id="11" * 16,
                   device_id="22" * 16, request_id=7, operation_id=7,
                   boot_epoch=2, session_id=3, run_id=7, motion_epoch=2,
                   receipt=1, transaction_phase=3, result_code=0)
    rewritten.append(record(5, 2200000, 2, 3, 7, 0, receipt))
    if stop_page:
        stop = wire("DummyAttemptEvidenceV2", run_id=7, attempt_id=7,
                    attempt_kind=1, category=2, motion_epoch=2,
                    evidence_seq=1)
        rewritten.append(record(1, 2100000, 2, 3, 7, 1, stop))
    if ambiguous_stop_epoch or ambiguous_only:
        state = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                     device_id="22" * 16, reference_epoch=1,
                     motion_epoch=2, core_state=4, produced_at_us=2149900)
        rewritten.append(record(2, 2050000, 2, 3, 0, 0, state))
    if stale_after_page:
        state = wire("DummyJointStateV2", boot_epoch=2, session_id=3,
                     device_id="22" * 16, reference_epoch=1,
                     motion_epoch=1, core_state=4, produced_at_us=2449900)
        rewritten.append(record(2, 2350000, 2, 3, 0, 0, state))
    if ambiguous_only:
        rewritten = [item for item in rewritten if not (
            joiner.RECORD_HEADER.unpack_from(item)[0] == 2 and
            joiner.RECORD_HEADER.unpack_from(item)[2] == 1800000)]
    raw.write_bytes(b"DPRAW01\0" + b"".join(sorted(
        rewritten, key=lambda raw_record: joiner.RECORD_HEADER.unpack_from(raw_record)[2])))
    return capture


##
# @brief Stop页后新epoch State可入窗口覆盖，人工lag跨度400µs。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_window_accepts_stop_epoch_after_bound_stop_page(tmp_path):
    context = build_window(canceled_window_capture(tmp_path))
    assert context["producer_lag_us"] == 400


##
# @brief 请求后页前Stop State可作为有界歧义观测，仍需完整强证据并保v2方法。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_window_allows_stop_state_between_request_and_page(tmp_path):
    context = build_window(canceled_window_capture(
        tmp_path, ambiguous_stop_epoch=True))
    assert context["producer_lag_gate_version"] == 2


##
# @brief Stop epoch提前到Cancel请求前的State，身份阶段不成立且不写输出。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_window_rejects_stop_epoch_before_bound_stop_page(tmp_path):
    capture = canceled_window_capture(tmp_path, early_stop_epoch=True)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)
    assert not capture[3].exists()


##
# @brief 只有歧义State不能补足确定覆盖，必须报State coverage。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_window_cannot_fill_coverage_with_ambiguous_state(tmp_path):
    capture = canceled_window_capture(tmp_path, ambiguous_only=True)
    with pytest.raises(ValueError, match="State coverage"):
        build_window(capture)
    assert not capture[3].exists()


##
# @brief 已证Stop后旧epoch State再到拒绝，不能重新纳入运动阶段。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_window_rejects_old_epoch_after_stop_page(tmp_path):
    capture = canceled_window_capture(tmp_path, stale_after_page=True)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)
    assert not capture[3].exists()


##
# @brief 缺绑定Stop CAN页不能单靠新State/回执解释安全阶段。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_canceled_window_requires_bound_stop_page(tmp_path):
    capture = canceled_window_capture(tmp_path, stop_page=False)
    with pytest.raises(ValueError, match="Stop CAN page"):
        build_window(capture)
    assert not capture[3].exists()


##
# @brief 窗口内lag400µs、全环境跨度400000µs分别保留，不把起停外峰值当本运动lag。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_window_gate_excludes_pre_goal_and_post_release_peaks(tmp_path):
    context = build_window(window_capture(tmp_path))
    assert context["producer_lag_us"] == 400
    assert context["producer_lag_environment_us"] == 400000
    assert context["producer_lag_gate_version"] == 2
    assert context["producer_lag_method"] == "state_receive_minus_produced_offset_span_motion_window"


##
# @brief Release经Query关联原请求的强结果也可确定窗口终端，不只支持直回执。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_window_gate_accepts_query_linked_release_receipt(tmp_path):
    assert build_window(window_capture(tmp_path, queried_reply=True))["producer_lag_us"] == 400


##
# @brief 直接/Query强结果可先于异步末页到达，最终两者齐备仍可构context。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param queried_reply true使用Query RID关联Release原请求，否则直接响应。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("queried_reply", [False, True])
def test_window_gate_accepts_release_receipt_before_terminal_page(tmp_path, queried_reply):
    capture = window_capture(tmp_path, queried_reply=queried_reply)
    records = joiner._raw_records(capture[1].read_bytes())
    reordered = [record(
        item["kind"], 3550000 if item["kind"] == 1 else
        3500000 if item["kind"] == 5 and item["fields"].get("release_committed") else
        item["when"], item["boot"], item["session"], item["run"], item["seq"], item["raw"])
        for item in records]
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(sorted(
        reordered, key=lambda raw_record: joiner.RECORD_HEADER.unpack_from(raw_record)[2])))
    assert build_window(capture)["producer_lag_us"] == 400


##
# @brief 强Release响应接收2800000早于原请求2900000，不可作为释放证明。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_window_gate_rejects_release_receipt_before_request(tmp_path):
    capture = window_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    reordered = [record(
        item["kind"], 2800000 if item["kind"] == 5 and
        item["fields"].get("release_committed") else item["when"],
        item["boot"], item["session"], item["run"], item["seq"], item["raw"])
        for item in records]
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(sorted(
        reordered, key=lambda raw_record: joiner.RECORD_HEADER.unpack_from(raw_record)[2])))
    with pytest.raises(ValueError, match="Release proof"):
        build_window(capture)


##
# @brief raw终页release_committed0，即使管理响应成功也不能闭合Release proof。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_window_gate_rejects_uncommitted_release_terminal(tmp_path):
    capture = window_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"],
        wire("DummyAttemptEvidenceV2", run_id=8, attempt_id=8,
             category=4, final=1, final_state=8, release_committed=0)
        if item["kind"] == 1 else item["raw"]) for item in records))
    with pytest.raises(ValueError, match="Release proof"):
        build_window(capture)


##
# @brief 仅保窗口两端State导致中间间隙过大，不能以全环境有样本代替连续覆盖。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_window_gate_rejects_sparse_state_coverage(tmp_path):
    capture = window_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(record(
        item["kind"], item["when"], item["boot"], item["session"],
        item["run"], item["seq"], item["raw"]) for item in records
        if item["kind"] != 2 or item["when"] in (700000, 1300000,
                                                   3900000, 4500000)))
    with pytest.raises(ValueError, match="State coverage"):
        build_window(capture)


##
# @brief 窗口中混另一device或motion99的State拒绝，不能拿它补年龄覆盖。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param options 传给window_capture的单一或组合故障开关字典。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("options", [{"other_device": True}, {"other_epoch": True}])
def test_window_gate_rejects_cross_identity_coverage_sample(tmp_path, options):
    capture = window_capture(tmp_path, **options)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)


##
# @brief 第二目标接纳后仍旧motion State不得填新目标覆盖。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_window_gate_rejects_prior_goal_phase_after_next_goal_acceptance(tmp_path):
    capture = window_capture(tmp_path, second_goal=True)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)


##
# @brief 第二目标motion提前出现也拒绝，不能只检查最终目标身份集合。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_window_gate_rejects_next_goal_phase_before_acceptance(tmp_path):
    capture = window_capture(tmp_path, second_goal=True, early_second_epoch=True)
    with pytest.raises(ValueError, match="State identity"):
        build_window(capture)


##
# @brief 运动/中间Hold/Stop三个位置各注入300ms偏移峰，窗口lag仍须超预算拒绝。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param offsets 七State的receive−produced偏移扰动µs，决定窗口内/全环境跨度。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("offsets", [
    (0, 0, 300000, 100, 200, 300, 0),
    (0, 0, 100, 300000, 200, 300, 0),
    (0, 0, 100, 200, 300000, 400, 0),
])
def test_window_gate_rejects_peak_during_motion_hold_or_stop(tmp_path, offsets):
    with pytest.raises(ValueError, match="producer lag budget"):
        build_window(window_capture(tmp_path, offsets))


##
# @brief 缺Release、缺Idle、跨boot或无accepted目标分别拒绝且不产生context。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param options 传给window_capture的单一或组合故障开关字典。
# @param match 期望ValueError消息的正则片段。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("options,match", [
    ({"release": False}, "Release"),
    ({"idle": False}, "Idle"),
    ({"other_boot": True}, "identity"),
    ({"goal": False}, "accepted"),
])
def test_window_gate_requires_raw_proof_and_identity(tmp_path, options, match):
    capture = window_capture(tmp_path, **options)
    with pytest.raises(ValueError, match=match):
        build_window(capture)
    assert not capture[3].exists()


##
# @brief 将角色改为accepted103→after_begin104→Cancel安全105→Release106，逐wire字段同步，不改变原输入生产逻辑。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return 四路径夹具，全部角色epoch已映射至103..106。
def begun_cancel_capture(tmp_path):
    """复现真实accepted103→after_begin104→cancel SafetyHold105角色。"""
    capture = canceled_window_capture(tmp_path)
    timeline, raw, _, _ = capture
    rows = list(csv.DictReader(timeline.read_text().splitlines()[:-1]))
    for row in rows:
        epoch = int(row["motion_epoch"])
        row["motion_epoch"] = str(103 if row["event"] in ("accepted", "before_begin")
                                  else 104 if epoch == 1 else 105 if epoch == 2 else 106)
    with timeline.open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows); stream.write("COMPLETE,1\n")
    rewritten = []
    for item in joiner._raw_records(raw.read_bytes()):
        fields = dict(item["fields"])
        for field in ("motion_epoch", "expected_motion_epoch"):
            if field in fields:
                fields[field] = {0: 0, 1: 104, 2: 105, 3: 106}[fields[field]]
        name = {1: "DummyAttemptEvidenceV2", 2: "DummyJointStateV2",
                3: "DummyDeviceStatusV2", 4: "DummyManageV2Request",
                5: "DummyManageV2Response"}[item["kind"]]
        rewritten.append(record(item["kind"], item["when"], item["boot"],
                                item["session"], item["run"], item["seq"], wire(name, **fields)))
    raw.write_bytes(b"DPRAW01\0" + b"".join(rewritten))
    return capture


##
# @brief Cancel绑定真正运行104而非接纳103，完整链可接受且lag400µs。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_cancel_window_binds_active_after_begin_epoch_not_accepted_epoch(tmp_path):
    assert build_window(begun_cancel_capture(tmp_path))["producer_lag_us"] == 400


##
# @brief after_begin缺失/重复/跨身份/太晚/轨迹UUID错或Cancel仍锚103逐项拒绝。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param mutation 参数化变异函数或分支名，逐例只制造指定错误。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("mutation", ["missing", "duplicate", "boot", "session", "run",
                                      "reference", "uuid", "trajectory", "late", "old_request"])
def test_cancel_window_rejects_ambiguous_or_unbound_after_begin(tmp_path, mutation):
    capture = begun_cancel_capture(tmp_path)
    timeline, raw, _, _ = capture
    lines = timeline.read_text().splitlines()
    index = next(i for i, line in enumerate(lines) if line.startswith("after_begin,"))
    fields = lines[index].split(",")
    if mutation == "missing": lines.pop(index)
    elif mutation == "duplicate": lines.insert(index, lines[index])
    elif mutation == "late": fields[3] = "2600000"; lines[index] = ",".join(fields)
    elif mutation != "old_request":
        column = {"boot": 4, "session": 5, "run": 8, "reference": 6,
                  "uuid": 1, "trajectory": 2}[mutation]
        fields[column] = "999"; lines[index] = ",".join(fields)
    if mutation == "old_request":
        rewritten = []
        for item in joiner._raw_records(raw.read_bytes()):
            payload = item["raw"]
            if item["kind"] == 4 and item["fields"]["operation"] == 9:
                payload = wire("DummyManageV2Request", **{
                    **item["fields"], "expected_motion_epoch": 103})
            rewritten.append(record(item["kind"], item["when"], item["boot"],
                                    item["session"], item["run"], item["seq"], payload))
        raw.write_bytes(b"DPRAW01\0" + b"".join(rewritten))
    timeline.write_text("\n".join(lines) + "\n")
    with pytest.raises(ValueError): build_window(capture)


##
# @brief Cancel结果的RID/op/epoch任一偏移，不能作为绑定Stop页证明。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param field 本负例要删/改的字段名，取值见参数化表。
# @param value 该字段的参数化错误值。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("field,value", [("operation_id", 6), ("motion_epoch", 104),
                                         ("request_id", 6)])
def test_cancel_window_keeps_exact_terminal_receipt_binding(tmp_path, field, value):
    capture = begun_cancel_capture(tmp_path)
    raw = capture[1]
    rewritten = []
    for item in joiner._raw_records(raw.read_bytes()):
        payload = item["raw"]
        if item["kind"] == 5 and item["fields"]["operation_id"] == 7:
            payload = wire("DummyManageV2Response", **{**item["fields"], field: value})
        rewritten.append(record(item["kind"], item["when"], item["boot"],
                                item["session"], item["run"], item["seq"], payload))
    raw.write_bytes(b"DPRAW01\0" + b"".join(rewritten))
    with pytest.raises(ValueError, match="Stop CAN page"):
        build_window(capture)


##
# @brief 读取DPRAW01，逐record调用transform后按Host接收时间排序并覆写本临时raw；不校验transform正确性。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @param transform 逐解析record返回完整重新打包bytes的回调。
# @return None；只重写临时capture raw文件。
def _rewrite_records(capture, transform):
    rewritten = []
    for item in joiner._raw_records(capture[1].read_bytes()):
        rewritten.append(transform(item))
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(sorted(
        rewritten, key=lambda value: joiner.RECORD_HEADER.unpack_from(value)[2])))


##
# @brief 同事务强结果已确认时Idle双流可先于异步末页，后到页补齐后接受。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_release_idle_channels_can_arrive_before_terminal_evidence(tmp_path):
    capture = window_capture(tmp_path)
    # 已确认本次op10，Idle Status/State先到，终页异步稍后到达。
    _rewrite_records(capture, lambda item: record(item["kind"],
        3950000 if item["kind"] == 1 and item["fields"].get("release_committed") else item["when"],
        item["boot"], item["session"], item["run"], item["seq"], item["raw"]))
    assert build_window(capture)["producer_lag_us"] == 400


##
# @brief Host接收活跃State可先于after_begin回调记录；绑定仍完整时不强求错误回调顺序。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_cancel_active_state_can_arrive_before_after_begin_timeline(tmp_path):
    capture = begun_cancel_capture(tmp_path)
    lines = capture[0].read_text().splitlines()
    index = next(i for i, line in enumerate(lines) if line.startswith("after_begin,"))
    fields = lines[index].split(","); fields[3] = "1400000"; lines[index] = ",".join(fields)
    capture[0].write_text("\n".join(lines) + "\n")
    assert build_window(capture)["producer_lag_us"] == 400


##
# @brief 提前Idle例外仍拒绝请求前到达、错op/run/boot/session/epoch、残permit、Pending或缺末页。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param mutation 参数化变异函数或分支名，逐例只制造指定错误。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("mutation", ["before_request", "operation", "run", "boot", "session",
                                      "phase", "permit", "receipt_pending", "no_terminal"])
def test_release_early_idle_still_requires_this_request_and_clean_identity(tmp_path, mutation):
    capture = window_capture(tmp_path)
    ##
    # @par 内部回调 transform
    # 按mutation只改Status/回执字段或接收时间，保持其余raw记录，制造提前Idle的精确负例。
    # 参数 item：当前解析DPRAW记录字典，含fields/raw及外层元数据。
    # 返回：重新编码的完整DPRAW记录bytes，不改生产实现。
    def transform(item):
        when = item["when"]
        payload = item["raw"]
        boot, session = item["boot"], item["session"]
        if item["kind"] == 3:
            fields = dict(item["fields"])
            if mutation == "before_request": when = 2800000
            elif mutation == "operation": fields["operation_id"] = 99
            elif mutation == "run": fields["run_id"] = 9
            elif mutation == "boot": fields["boot_epoch"] = 9; boot = 9
            elif mutation == "session": fields["session_id"] = 3; session = 3
            elif mutation == "phase": fields["motion_epoch"] = 9
            elif mutation == "permit": fields["permit_remaining_us"] = 1
            payload = wire("DummyDeviceStatusV2", **fields)
        if item["kind"] == 5 and mutation == "receipt_pending":
            payload = wire("DummyManageV2Response", **{**item["fields"], "transaction_phase": 2})
        return record(item["kind"], when, boot, session, item["run"], item["seq"], payload)
    _rewrite_records(capture, transform)
    if mutation == "no_terminal":
        raw = capture[1]
        raw.write_bytes(b"DPRAW01\0" + b"".join(record(x["kind"], x["when"], x["boot"],
            x["session"], x["run"], x["seq"], x["raw"]) for x in joiner._raw_records(raw.read_bytes())
            if x["kind"] != 1))
    with pytest.raises(ValueError, match="Release"):
        build_window(capture)


##
# @brief 生产Gate 250ms State预算与v2运动窗，专用于dense20ms夹具，避免沿用旧1s容差。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @return 使用250ms State门的context字典。
def build_strict_window(capture):
    """真实Gate的250ms预算；不沿用老window夹具的1s采样容差。"""
    return builder.build_context(
        *capture, path_tolerance_rad=[0.01] * 6,
        goal_tolerance_rad=[0.003] * 6, producer_lag_budget_us=250000,
        feedback_age_budget_us=10000, ros_feedback_lag_budget_us=2000,
        gate_state_max_age_us=250000, producer_lag_scope="motion_window_v2")


##
# @brief 生成20ms State、Stop页晚300ms、完整SHA页链与精确Cancel Query op508；Stop期间State仍可报旧op501但epoch/management已新。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return 密集State/完整取消链的四路径夹具。
def dense_cancel_capture(tmp_path):
    """20ms真wire角色：103接纳→104运行→105安全停止；Stop页晚300ms。

    Stop期间State仍报告旧事务op501，身份与管理代次已切到105/143。
    完整本run页链和精确Cancel Query回执提供安全相位证明。
    """
    capture = begun_cancel_capture(tmp_path)
    timeline, raw, _, _ = capture
    lines = timeline.read_text().splitlines()
    for index, line in enumerate(lines):
        if line.startswith("terminal_canceled,"):
            fields = line.split(","); fields[-1] = "508"; lines[index] = ",".join(fields)
    timeline.write_text("\n".join(lines) + "\n")
    hashes = {"contract_sha256": "41" * 32, "model_sha256": "42" * 32,
              "profile_sha256": "43" * 32, "firmware_sha256": "44" * 32}
    events = []
    cancel_fields = None
    for item in joiner._raw_records(raw.read_bytes()):
        if item["kind"] == 2 and 1000000 <= item["when"] <= 4000000:
            continue
        if item["kind"] == 1 and item["run"] == 7:
            continue
        if item["kind"] == 5 and item["fields"]["request_id"] == 7:
            continue
        payload = item["raw"]
        if item["kind"] == 4 and item["fields"]["operation"] == 9:
            cancel_fields = {**item["fields"], "expected_reference_epoch": 1,
                             "expected_motion_epoch": 104, "expected_management_epoch": 142,
                             "expected_profile_id": 201, "configured_mask": 63}
            payload = wire("DummyManageV2Request", **cancel_fields)
        elif item["kind"] == 3:
            # Only the later Release Idle Status supplies the immutable digest tuple.
            # Fast canceled goals need not publish any same-run HOLD Status.
            payload = wire("DummyDeviceStatusV2", **{
                **item["fields"], **hashes, "profile_id": 201, "configured_mask": 63})
        events.append(record(item["kind"], item["when"], item["boot"],
                             item["session"], item["run"], item["seq"], payload))
    seed = struct.pack("<QQ", 7, 2) + bytes.fromhex("11" * 16)
    seed += b"".join(bytes.fromhex(hashes[name]) for name in
                     ("contract_sha256", "model_sha256", "profile_sha256", "firmware_sha256"))
    chain = hashlib.sha256(seed.ljust(192, b"\0")).digest()
    stop_fields = dict(run_id=7, attempt_id=7, attempt_kind=1, parent_run_id=0,
                       evidence_seq=0, category=2, motion_epoch=105, group_id=1,
                       group_count=1, producer_seq=1, expected_mask=63,
                       submitted_mask=63, sent_mask=63, failed_mask=0, final=1,
                       queued_at_us=[2000100] * 6, taken_at_us=[2000200] * 6,
                       submitted_at_us=[2000300] * 6, sent_at_us=[2000400] * 6)
    for when, fields in ((2300000, stop_fields), (2350000, dict(
            run_id=7, attempt_id=7, attempt_kind=1, parent_run_id=0,
            evidence_seq=1, category=4, motion_epoch=103, group_count=1,
            final=1, final_state=7, motion_complete=0, release_committed=0))):
        fields = {**fields, "previous_chain_sha256": chain.hex()}
        payload = wire("DummyAttemptEvidenceV2", **fields)
        chain = hashlib.sha256(chain + payload[:307] + payload[371:372]).digest()
        payload = wire("DummyAttemptEvidenceV2", **fields, chain_sha256=chain.hex())
        events.append(record(1, when, 2, 3, 7, fields["evidence_seq"], payload))
    assert cancel_fields is not None
    query = wire("DummyManageV2Request", **{**cancel_fields, "operation": 12,
                 "request_id": 8, "request_seq": 8, "queried_operation_id": 7})
    events.append(record(4, 2360000, 2, 3, 0, 8, query))
    receipt = wire("DummyManageV2Response", client_instance_id="11" * 16,
                   device_id="22" * 16, request_id=8, operation_id=508,
                   boot_epoch=2, session_id=3, reference_epoch=1, motion_epoch=105,
                   management_epoch=143, profile_id=201, run_id=7, receipt=1,
                   transaction_phase=3, result_code=0)
    events.append(record(5, 2400000, 2, 3, 7, 0, receipt))
    for index, when in enumerate(range(1000000, 4000001, 20000)):
        session = 0 if when >= 3400000 else 3
        motion = (106 if session == 0 else 103 if when < 1100000 else
                  104 if when < 2000000 else 105)
        stopping = 2000000 <= when <= 2300000
        core = 1 if session == 0 else 7 if stopping else 5 if motion == 104 else 4
        state = wire("DummyJointStateV2", boot_epoch=2, session_id=session,
                     device_id="22" * 16, reference_epoch=0 if session == 0 else 1,
                     motion_epoch=motion, management_epoch=143 if motion >= 105 else 142,
                     profile_id=201, configured_mask=63, core_state=core,
                     operation_id=501 if stopping or motion < 105 else 508,
                     transaction_phase=3, hold_ready=int(core == 4),
                     position_valid_mask=63, velocity_valid_mask=63,
                     raw_position_valid_mask=63, can_rx_age_us=[1000] * 6,
                     state_seq=index + 1, produced_at_us=when + 100000 - (index % 5) * 100)
        events.append(record(2, when, 2, session, 0, index + 1, state))
    raw.write_bytes(b"DPRAW01\0" + b"".join(sorted(
        events, key=lambda value: joiner.RECORD_HEADER.unpack_from(value)[2])))
    return capture


##
# @brief 核16个Stop State每20ms且op501，强raw/回执补足角色后可在250ms门内覆盖。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_dense_cancel_stopping_states_with_old_operation_fill_250ms_coverage(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    stopping = [x for x in records if x["kind"] == 2 and
                2000000 <= x["when"] <= 2300000]
    assert len(stopping) == 16
    assert all(x["fields"]["operation_id"] == 501 for x in stopping)
    assert max(b["when"] - a["when"] for a, b in zip(stopping, stopping[1:])) == 20000
    assert build_strict_window(capture)["producer_lag_us"] == 400


##
# @brief 只改Stop接收段State字段并重编码外层boot/session，不替代页链/强结果。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @param changes 要覆写的字段字典，作用范围由helper明确限制。
# @return None；原地变异临时State记录。
def _alter_dense_transition(capture, changes):
    ##
    # @par 内部回调 transform
    # 对2000000..2300000µs的State应用changes，其它种类原字段重编码，外层boot/session跟内层变化。
    # 参数 item：当前解析DPRAW记录字典，含fields/raw及外层元数据。
    # 返回：重新编码的完整DPRAW记录bytes，不改生产实现。
    def transform(item):
        fields = dict(item["fields"])
        if item["kind"] == 2 and 2000000 <= item["when"] <= 2300000:
            fields.update(changes)
        name = {1: "DummyAttemptEvidenceV2", 2: "DummyJointStateV2",
                3: "DummyDeviceStatusV2", 4: "DummyManageV2Request",
                5: "DummyManageV2Response"}[item["kind"]]
        return record(item["kind"], item["when"],
                      fields.get("boot_epoch", item["boot"]),
                      fields.get("session_id", item["session"]),
                      item["run"], item["seq"], wire(name, **fields))
    _rewrite_records(capture, transform)


##
# @brief 改本RUN指定类别后顺序重签所有页链，保证语义负例不会仅因旧SHA先拒绝。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @param category 要变异的Evidence类别，2安全组或4终页。
# @param changes 要覆写的字段字典，作用范围由helper明确限制。
# @return None；原地变异并重签本RUN页链。
def _alter_dense_pages(capture, category, changes):
    """重新签夹具页链，保证SENT/终态负例不靠错误SHA偶然拒绝。"""
    records = joiner._raw_records(capture[1].read_bytes())
    first = next(x for x in records if x["kind"] == 1 and x["run"] == 7)
    previous = bytes.fromhex(first["fields"]["previous_chain_sha256"])
    rewritten = []
    for item in records:
        payload = item["raw"]
        if item["kind"] == 1 and item["run"] == 7:
            fields = dict(item["fields"])
            if fields["category"] == category:
                fields.update(changes)
            fields["previous_chain_sha256"] = previous.hex()
            payload = wire("DummyAttemptEvidenceV2", **fields)
            previous = hashlib.sha256(previous + payload[:307] + payload[371:372]).digest()
            fields["chain_sha256"] = previous.hex()
            payload = wire("DummyAttemptEvidenceV2", **fields)
        rewritten.append(record(item["kind"], item["when"], item["boot"],
                                item["session"], item["run"], item["seq"], payload))
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(rewritten))


##
# @brief 同绑定安全过渡可为hold_ready的Hold4且仍旧op501，不强制只接Stopping7。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_dense_cancel_ready_hold_state_with_old_operation_fills_coverage(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    _alter_dense_transition(capture, {"core_state": 4, "hold_ready": 1})
    assert build_strict_window(capture)["producer_lag_us"] == 400


##
# @brief 十四种State身份/phase/mask/有效性/时钟破坏不得填严格覆盖，输出不存在。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param changes 要覆写的字段字典，作用范围由helper明确限制。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("changes", [
    {"boot_epoch": 3}, {"session_id": 4}, {"device_id": "33" * 16},
    {"reference_epoch": 2}, {"motion_epoch": 104}, {"management_epoch": 142},
    {"profile_id": 202}, {"configured_mask": 31}, {"core_state": 5},
    {"position_valid_mask": 0}, {"velocity_valid_mask": 31},
    {"raw_position_valid_mask": 0}, {"core_state": 4, "hold_ready": 0},
    {"produced_at_us": 0},
])
def test_dense_cancel_invalid_transition_state_cannot_fill_coverage(tmp_path, changes):
    capture = dense_cancel_capture(tmp_path)
    _alter_dense_transition(capture, changes)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


##
# @brief 九种Query结果身份/epoch/结果错误不允许旧op State的过渡例外。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param changes 要覆写的字段字典，作用范围由helper明确限制。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("changes", [
    {"request_id": 99}, {"operation_id": 507}, {"reference_epoch": 2},
    {"motion_epoch": 104}, {"management_epoch": 142}, {"profile_id": 202},
    {"client_instance_id": "33" * 16}, {"run_id": 9},
    {"transaction_phase": 4, "result_code": 10},
])
def test_dense_cancel_wrong_receipt_cannot_authorize_transition(tmp_path, changes):
    capture = dense_cancel_capture(tmp_path)
    ##
    # @par 内部回调 transform
    # 只改RID8的Cancel Query回执并使外层run跟随字段，其它记录保持，隔离强结果门。
    # 参数 item：当前解析DPRAW记录字典，含fields/raw及外层元数据。
    # 返回：重新编码的完整DPRAW记录bytes，不改生产实现。
    def transform(item):
        if item["kind"] == 5 and item["fields"]["request_id"] == 8:
            fields = {**item["fields"], **changes}
            return record(5, item["when"], item["boot"], item["session"],
                          fields["run_id"], 0, wire("DummyManageV2Response", **fields))
        return record(item["kind"], item["when"], item["boot"],
                      item["session"], item["run"], item["seq"], item["raw"])
    _rewrite_records(capture, transform)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


##
# @brief SENT/提交缺轴、failed/lost、错终态/birth/failure_source均在重签有效SHA后仍拒绝。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param category 要变异的Evidence类别，2安全组或4终页。
# @param changes 要覆写的字段字典，作用范围由helper明确限制。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("category,changes", [
    (2, {"sent_mask": 31}), (2, {"failed_mask": 1}),
    (2, {"submitted_mask": 31}), (2, {"evidence_lost": 1}),
    (4, {"final_state": 6, "motion_complete": 1}),
    (4, {"motion_epoch": 104}), (4, {"first_failure_source": 1}),
])
def test_dense_cancel_invalid_full_chain_cannot_authorize_transition(tmp_path, category, changes):
    capture = dense_cancel_capture(tmp_path)
    _alter_dense_pages(capture, category, changes)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


##
# @brief 只翻Stop首链摘要一字节，身份字段未改也不得提供过渡授权。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_dense_cancel_corrupt_sha_cannot_authorize_transition(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    ##
    # @par 内部回调 transform
    # 定位RUN7首Evidence页seq0翻chain[0]，不重签；其它记录原样。
    # 参数 item：当前解析DPRAW记录字典，含fields/raw及外层元数据。
    # 返回：重新编码的完整DPRAW记录bytes，不改生产实现。
    def transform(item):
        payload = item["raw"]
        if item["kind"] == 1 and item["run"] == 7 and item["seq"] == 0:
            payload = payload[:339] + bytes([payload[339] ^ 1]) + payload[340:]
        return record(item["kind"], item["when"], item["boot"],
                      item["session"], item["run"], item["seq"], payload)
    _rewrite_records(capture, transform)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


##
# @brief 删RUN7所有Evidence页后，State/Query结果仍不能独立放行过渡。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_dense_cancel_missing_full_proof_cannot_authorize_transition(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(record(
        x["kind"], x["when"], x["boot"], x["session"], x["run"], x["seq"], x["raw"])
        for x in records if not (x["kind"] == 1 and x["run"] == 7)))
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


##
# @brief 后到Release Idle Status给两种contract摘要，静态四摘要不唯一则不能推导原RUN页链seed。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_dense_cancel_ambiguous_static_identity_quad_cannot_authorize_transition(tmp_path):
    capture = dense_cancel_capture(tmp_path)
    records = joiner._raw_records(capture[1].read_bytes())
    status = next(x for x in records if x["kind"] == 3)
    fields = {**status["fields"], "contract_sha256": "55" * 32}
    # Both Status observations are later unrelated Release Idle observations.
    extra = record(3, 3850000, 2, 0, 8, 0, wire("DummyDeviceStatusV2", **fields))
    capture[1].write_bytes(b"DPRAW01\0" + b"".join(sorted(
        [record(x["kind"], x["when"], x["boot"], x["session"], x["run"], x["seq"], x["raw"])
         for x in records] + [extra], key=lambda x: joiner.RECORD_HEADER.unpack_from(x)[2])))
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


##
# @brief 协调改原Cancel/Query、回执、Status或过渡State的mask/profile/ref，测试自洽伪造仍须绑定目标。
# @param capture (timeline路径, raw路径, action路径, 输出context路径)临时夹具。
# @param mask 要替换的configured_mask；None保持。
# @param profile 要替换的profile ID；None保持。
# @param reference 要替换的reference epoch；None保持。
# @return None；原地改写所选身份声明。
def _alter_dense_cancel_binding(capture, *, mask=None, profile=None, reference=None):
    ##
    # @par 内部回调 transform
    # 按mask/profile/reference分别选定请求/结果/Status/过渡State字段重编码，不对不相关消息乱改。
    # 参数 item：当前解析DPRAW记录字典，含fields/raw及外层元数据。
    # 返回：重新编码的完整DPRAW记录bytes，不改生产实现。
    def transform(item):
        fields = dict(item["fields"])
        wanted = item["kind"] == 4 and (fields["operation"] == 9 or
                 (fields["operation"] == 12 and fields["queried_operation_id"] == 7))
        receipt = item["kind"] == 5 and fields["request_id"] == 8
        status = item["kind"] == 3
        transition = item["kind"] == 2 and 2000000 <= item["when"] <= 2300000
        if mask is not None and (wanted or status):
            fields["configured_mask"] = mask
        if profile is not None and (wanted or receipt or status or transition):
            fields["expected_profile_id" if wanted else "profile_id"] = profile
        if reference is not None and (wanted or receipt):
            fields["expected_reference_epoch" if wanted else "reference_epoch"] = reference
        name = {1: "DummyAttemptEvidenceV2", 2: "DummyJointStateV2",
                3: "DummyDeviceStatusV2", 4: "DummyManageV2Request",
                5: "DummyManageV2Response"}[item["kind"]]
        return record(item["kind"], item["when"], item["boot"], item["session"],
                      item["run"], item["seq"], wire(name, **fields))
    _rewrite_records(capture, transform)


##
# @brief 整体mask1/profile0/ref2虽可内部相互一致，仍不符合六轴目标授权而拒绝。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param changes 要覆写的字段字典，作用范围由helper明确限制。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize("changes", [{"mask": 1}, {"profile": 0}, {"reference": 2}])
def test_dense_cancel_wanted_authority_must_bind_goal(tmp_path, changes):
    capture = dense_cancel_capture(tmp_path)
    _alter_dense_cancel_binding(capture, **changes)
    with pytest.raises(ValueError):
        build_strict_window(capture)
    assert not capture[3].exists()


##
# @brief 在dense取消后合成当前Disable105→106、management143→144和Query确认，开关注入错身份/缺typed请求/无Idle等。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param mutation 参数化变异函数或分支名，逐例只制造指定错误。
# @param release_state Release过渡core，参数化7与实际Disabling8。
# @return 当前Release绑定/变异的四路径夹具。
def _cancel_disabling_after_verified_release_request(tmp_path,mutation=None,release_state=7):
 c=dense_cancel_capture(tmp_path)
 ##
 # @par 内部回调 transform
 # 将当前Release请求及随后State/强结果/Idle换为一致106/144，并按mutation局部破坏；负例不修改产品validator。
 # 参数 row：当前解析DPRAW记录，按本helper mutation注入释放阶段变异。
 # 返回：重新编码的完整DPRAW记录bytes，不改生产实现。
 def transform(row):
  fields=dict(row['fields']);boot=row['boot'];session=row['session'];when=row['when']
  if row['kind']==4 and fields['operation']==10:
   fields.update(expected_reference_epoch=1,expected_motion_epoch=105,expected_management_epoch=143,expected_profile_id=201,configured_mask=63)
  if row['kind']==2 and 2900000<=when<3400000:
   fields.update(motion_epoch=106,management_epoch=144,core_state=release_state,hold_ready=0)
   if mutation=='epoch':fields['motion_epoch']=107
   elif mutation=='management':fields['management_epoch']=145
   elif mutation=='boot':fields['boot_epoch']=boot=3
   elif mutation=='session':fields['session_id']=session=4
   elif mutation=='ref':fields['reference_epoch']=2
   elif mutation=='before':when=2899999
   elif mutation=='stateRun':fields['core_state']=5
   elif mutation=='stateHold':fields['core_state']=4;fields['hold_ready']=1
   elif mutation=='mask':fields['configured_mask']=31
   elif mutation=='profile':fields['profile_id']=202
  if row['kind']==5 and fields['request_id']==9:
   fields['management_epoch']=144
   fields['request_id']=11
   if mutation=='receipt':fields['operation_id']=999
  if row['session']==0 and row['kind'] in (2,3):fields['management_epoch']=144
  if mutation=='noIdle'and row['session']==0 and row['kind']in(2,3):fields['core_state']=4
  if mutation=='sent31'and row['kind']==1 and fields.get('category')==2:fields['sent_mask']=31
  if mutation=='chain'and row['kind']==1 and fields.get('category')==2:fields['chain_sha256']='ff'*32
  name={1:'DummyAttemptEvidenceV2',2:'DummyJointStateV2',3:'DummyDeviceStatusV2',4:'DummyManageV2Request',5:'DummyManageV2Response'}[row['kind']]
  return record(row['kind'],when,boot,session,row['run'],row['seq'],wire(name,**fields))
 _rewrite_records(c,transform)
 records=joiner._raw_records(c[1].read_bytes());root=next(r['fields']for r in records if r['kind']==4 and r['fields']['operation']==10)
 query=wire('DummyManageV2Request',**{**root,'operation':12,'request_id':11,'request_seq':11,'queried_operation_id':9})
 rows=[record(r['kind'],r['when'],r['boot'],r['session'],r['run'],r['seq'],r['raw'])for r in records if not(mutation=='noTypedDisable'and r['kind']==4 and r['fields']['operation']==10)]
 rows.append(record(4,3305000,2,3,0,11,query));c[1].write_bytes(b'DPRAW01\0'+b''.join(sorted(rows,key=lambda b:joiner.RECORD_HEADER.unpack_from(b)[2])))
 return c

##
# @brief 当前绑定Disable请求后的过渡State可填窗口，核返回WINDOW_LAG_METHOD。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_cancel_Disabling_after_current_bound_Disable_fills_window(tmp_path):
 c=_cancel_disabling_after_verified_release_request(tmp_path);result=build_strict_window(c)
 assert result['producer_lag_method']==builder.WINDOW_LAG_METHOD
##
# @brief 十二种原锚/停止链/phase/掩码错值使context拒绝且不落盘。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param bad 需注入的错误分支名，见参数化表。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize('bad',['before','epoch','boot','session','ref','receipt','chain','sent31','stateRun','stateHold','mask','profile'])
def test_cancel_Disabling_without_bound_release_or_full_stop_proof_is_rejected(tmp_path,bad):
 c=_cancel_disabling_after_verified_release_request(tmp_path,bad)
 with pytest.raises(ValueError):build_strict_window(c)
 assert not c[3].exists()


##
# @brief 参数化State7和实际Disabling8都必须按本次Release的epoch/op/management绑定后才接受。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param release_state Release过渡core，参数化7与实际Disabling8。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize('release_state',[7,8])
def test_cancel_release_State7_and_actual_Disabling8_bound_receipt_management(tmp_path,release_state):
 c=_cancel_disabling_after_verified_release_request(tmp_path,release_state=release_state)
 result=build_strict_window(c)
 assert result['producer_lag_method']==builder.WINDOW_LAG_METHOD
##
# @brief 两种状态各覆盖十五类缺失/错绑定，尤其缺typed Disable或最终双Idle都不能借过渡放行。
# @param tmp_path pytest临时目录Path；raw/context/YAML等夹具只在此处生成。
# @param bad 需注入的错误分支名，见参数化表。
# @param release_state Release过渡core，参数化7与实际Disabling8。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
@pytest.mark.parametrize('bad',['before','epoch','boot','session','ref','management','receipt','chain','sent31','stateRun','stateHold','mask','profile','noTypedDisable','noIdle'])
@pytest.mark.parametrize('release_state',[7,8])
def test_cancel_release_State7_8_must_match_full_Current_Disable_proof(tmp_path,bad,release_state):
 c=_cancel_disabling_after_verified_release_request(tmp_path,bad,release_state=release_state)
 with pytest.raises(ValueError):build_strict_window(c)
 assert not c[3].exists()
