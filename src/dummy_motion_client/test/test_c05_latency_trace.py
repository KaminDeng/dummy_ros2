"""C05 离线时序证据的边界测试。"""
##
# @file test_c05_latency_trace.py
# @brief 离线有界日志/时钟/阶段关联与非资格结论测试。
#
# 标记与bridge字典均人工；只有write_report产生临时文件，不运行peer/ROS。完整600/500µs间隔正例仍strict_50ms_pass=false，不能将缺wire序号的桥chunk当完整链。

import importlib.util
import json
from pathlib import Path

import pytest


MODULE = Path(__file__).with_name("c05_latency_trace.py")
SPEC = importlib.util.spec_from_file_location("c05_latency_trace", MODULE)
trace = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(trace)


##
# @brief 构造stage/sequence/at_us/clock人工标记，不采样系统时钟。
# @param stage 人工时序阶段名。
# @param sequence State或producer序号，须按所在方向解释。
# @param at_us 同声明域下时间µs，本marker不实际读取时钟。
# @param clock 时钟域声明文本，默认steady_monotonic_us。
# @return 人工时序事件dict。
def marker(stage, sequence, at_us, clock="steady_monotonic_us"):
    return {"stage": stage, "sequence": sequence, "at_us": at_us, "clock": clock}


##
# @brief 缺host/relay等阶段须INCOMPLETE并明确missing，不补造时间。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_missing_sequence_stages_cannot_be_reported_complete():
    report = trace.summarize([
        marker("peer_state_publish", 7, 1000),
        marker("peer_command_receive", 11, 5000),
    ], {}, max_events=8)

    assert report["status"] == "INCOMPLETE"
    assert report["state"][0]["sequence"] == 7
    assert "host_state_callback" in report["state"][0]["missing"]
    assert "relay_state_ingress" in report["state"][0]["missing"]
    assert "host_command_publish" in report["command"][0]["missing"]
    assert report["strict_50ms_pass"] is False


##
# @brief 混wall与steady声明拒绝，不直接相减不同域。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_mixed_clock_domain_is_rejected():
    with pytest.raises(ValueError, match="clock domain"):
        trace.summarize([
            marker("peer_state_publish", 7, 1000),
            marker("host_state_callback", 7, 2000, "wall_unix_us"),
        ], {}, max_events=8)


##
# @brief 4标记上限3必须拒绝，不默默截断成可用报告。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_event_overflow_is_rejected_without_truncation():
    with pytest.raises(ValueError, match="overflow"):
        trace.summarize([marker("peer_state_publish", index, index * 1000)
                         for index in range(1, 5)], {}, max_events=3)


##
# @brief 17条gap超过16限制也拒绝，即使无wire事件。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_bridge_history_overflow_is_rejected():
    with pytest.raises(ValueError, match="overflow"):
        trace.summarize([], {"clock": "steady_monotonic_us", "peer_to_host": {
            "long_gaps": [{"gap_us": 130000}] * 17}})


##
# @brief 桥chunk自身seq保留但wire关联unavailable，缺relay序号仍显式missing。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_native_bridge_chunks_are_bounded_but_not_assigned_sequence_ids():
    bridge = {"clock": "steady_monotonic_us", "peer_to_host": {"chunks": 4, "recent_chunks": [
        {"seq": 3, "ingress_at_us": 1300, "egress_at_us": 1400, "bytes": 32},
        {"seq": 4, "ingress_at_us": 2200, "egress_at_us": 2300, "bytes": 64},
    ], "max_gap_us": 900, "long_gaps": [{"gap_us": 900}]},
        "host_to_peer": {"chunks": 0, "recent_chunks": []}}
    report = trace.summarize([marker("peer_state_publish", 7, 1000)], bridge,
                             max_events=8)

    assert report["bridge"]["peer_to_host"]["recent_chunks"][0]["seq"] == 3
    assert report["bridge"]["peer_to_host"]["sequence_correlation"] == "unavailable"
    assert report["bridge"]["peer_to_host"]["max_gap_us"] == 900
    assert "relay_state_ingress" in report["state"][0]["missing"]


##
# @brief 桥wall声明与事件steady不符时拒绝。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_mixed_bridge_clock_is_rejected():
    with pytest.raises(ValueError, match="clock domain"):
        trace.summarize([marker("peer_state_publish", 7, 1000)],
                        {"clock": "wall_unix_us"})


##
# @brief 人工完整State600/Command500µs只MEASURED_COMPONENT_ONLY，严格50ms仍false。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_complete_chain_reports_measured_intervals_without_policy_pass():
    events = [marker(stage, 7, at_us) for stage, at_us in (
        ("peer_state_publish", 1000), ("relay_state_ingress", 1200),
        ("relay_state_egress", 1300), ("host_state_callback", 1600))]
    events += [marker(stage, 11, at_us) for stage, at_us in (
        ("host_command_publish", 2000), ("relay_command_ingress", 2100),
        ("relay_command_egress", 2200), ("peer_command_receive", 2500))]

    report = trace.summarize(events, {}, max_events=8)

    assert report["status"] == "MEASURED_COMPONENT_ONLY"
    assert report["state"][0]["end_to_end_us"] == 600
    assert report["command"][0]["end_to_end_us"] == 500
    assert report["strict_50ms_pass"] is False


##
# @brief 核最大130000µs对应seq8→9，不只给一个无上下文数值。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_peer_cadence_names_sequences_around_largest_gap():
    report = trace.summarize([
        marker("peer_state_publish", 7, 1000),
        marker("peer_state_publish", 8, 51000),
        marker("peer_state_publish", 9, 181000),
    ], {}, max_events=8)

    assert report["peer_state_max_gap"] == {"before_seq": 8, "after_seq": 9,
                                            "gap_us": 130000}


##
# @brief ready日志忽略，state/command提取精确，超过1上限拒绝。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_peer_log_extraction_keeps_only_bounded_sequence_markers():
    lines = [json.dumps({"event": "state_trace", "state_seq": 7,
                         "published_at_us": 1000, "clock": "steady_monotonic_us"}),
             json.dumps({"event": "command", "producer_seq": 11,
                         "received_at_us": 2500, "clock": "steady_monotonic_us"}),
             '{"event":"ready"}']

    assert trace.peer_events(lines, max_events=2) == [
        marker("peer_state_publish", 7, 1000),
        marker("peer_command_receive", 11, 2500),
    ]
    with pytest.raises(ValueError, match="overflow"):
        trace.peer_events(lines, max_events=1)


##
# @brief 只提接收时间1800与seq7，不能误用Source produced1000为Host回调。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_heartbeat_stop_log_exposes_last_host_callback_counter():
    lines = ["device transport state heartbeat stop: gap_us=130000 callbacks=3 "
             "last_callback_seq=7 last_callback_produced_us=1000 "
             "last_callback_received_us=1800 last_accepted_seq=7"]

    assert trace.host_callback_events(lines) == [marker("host_state_callback", 7, 1800)]
    assert trace.host_callback_events(["unrelated log line"]) == []


##
# @brief seq0不可作为有用回调计数，ValueError。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_host_callback_counter_rejects_unusable_values():
    with pytest.raises(ValueError, match="host callback counter"):
        trace.host_callback_events(["last_callback_seq=0 last_callback_received_us=1800"])


##
# @brief 首次文件内容等报告，第二次同名必须FileExistsError。
# @param tmp_path pytest临时目录Path。
# @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
def test_report_file_is_created_without_overwriting_prior_evidence(tmp_path):
    path = tmp_path / "c05-trace.json"
    report = trace.summarize([marker("peer_state_publish", 7, 1000)], {})

    trace.write_report(path, report)

    assert json.loads(path.read_text()) == report
    with pytest.raises(FileExistsError):
        trace.write_report(path, report)
