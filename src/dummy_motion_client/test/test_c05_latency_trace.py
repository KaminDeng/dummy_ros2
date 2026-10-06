"""C05 离线时序证据的边界测试。"""

import importlib.util
import json
from pathlib import Path

import pytest


MODULE = Path(__file__).with_name("c05_latency_trace.py")
SPEC = importlib.util.spec_from_file_location("c05_latency_trace", MODULE)
trace = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(trace)


def marker(stage, sequence, at_us, clock="steady_monotonic_us"):
    return {"stage": stage, "sequence": sequence, "at_us": at_us, "clock": clock}


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


def test_mixed_clock_domain_is_rejected():
    with pytest.raises(ValueError, match="clock domain"):
        trace.summarize([
            marker("peer_state_publish", 7, 1000),
            marker("host_state_callback", 7, 2000, "wall_unix_us"),
        ], {}, max_events=8)


def test_event_overflow_is_rejected_without_truncation():
    with pytest.raises(ValueError, match="overflow"):
        trace.summarize([marker("peer_state_publish", index, index * 1000)
                         for index in range(1, 5)], {}, max_events=3)


def test_bridge_history_overflow_is_rejected():
    with pytest.raises(ValueError, match="overflow"):
        trace.summarize([], {"clock": "steady_monotonic_us", "peer_to_host": {
            "long_gaps": [{"gap_us": 130000}] * 17}})


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


def test_mixed_bridge_clock_is_rejected():
    with pytest.raises(ValueError, match="clock domain"):
        trace.summarize([marker("peer_state_publish", 7, 1000)],
                        {"clock": "wall_unix_us"})


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


def test_peer_cadence_names_sequences_around_largest_gap():
    report = trace.summarize([
        marker("peer_state_publish", 7, 1000),
        marker("peer_state_publish", 8, 51000),
        marker("peer_state_publish", 9, 181000),
    ], {}, max_events=8)

    assert report["peer_state_max_gap"] == {"before_seq": 8, "after_seq": 9,
                                            "gap_us": 130000}


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


def test_heartbeat_stop_log_exposes_last_host_callback_counter():
    lines = ["device transport state heartbeat stop: gap_us=130000 callbacks=3 "
             "last_callback_seq=7 last_callback_produced_us=1000 "
             "last_callback_received_us=1800 last_accepted_seq=7"]

    assert trace.host_callback_events(lines) == [marker("host_state_callback", 7, 1800)]
    assert trace.host_callback_events(["unrelated log line"]) == []


def test_host_callback_counter_rejects_unusable_values():
    with pytest.raises(ValueError, match="host callback counter"):
        trace.host_callback_events(["last_callback_seq=0 last_callback_received_us=1800"])


def test_report_file_is_created_without_overwriting_prior_evidence(tmp_path):
    path = tmp_path / "c05-trace.json"
    report = trace.summarize([marker("peer_state_publish", 7, 1000)], {})

    trace.write_report(path, report)

    assert json.loads(path.read_text()) == report
    with pytest.raises(FileExistsError):
        trace.write_report(path, report)
