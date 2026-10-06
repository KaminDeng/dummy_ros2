"""C05 离线单调时钟时序诊断，不产生合格判定。"""

from collections import defaultdict
import json
import re
from pathlib import Path


CLOCK = "steady_monotonic_us"
STATE_STAGES = ("peer_state_publish", "relay_state_ingress",
                "relay_state_egress", "host_state_callback")
COMMAND_STAGES = ("host_command_publish", "relay_command_ingress",
                  "relay_command_egress", "peer_command_receive")
MAX_EVENTS = 4096
MAX_CHUNKS = 16


def peer_events(lines, max_events=MAX_EVENTS):
    events = []
    for line in lines:
        if not line.startswith("{"):
            continue
        value = json.loads(line)
        if value.get("event") == "state_trace":
            events.append({"stage": "peer_state_publish", "sequence": value["state_seq"],
                           "at_us": value["published_at_us"], "clock": value["clock"]})
        elif value.get("event") == "command" and "received_at_us" in value:
            events.append({"stage": "peer_command_receive",
                           "sequence": value["producer_seq"],
                           "at_us": value["received_at_us"], "clock": value["clock"]})
        if len(events) > max_events:
            raise ValueError("peer trace event overflow")
    return events


def host_callback_events(lines, max_events=MAX_EVENTS):
    observed = {}
    for line in lines:
        sequence = re.search(r"\blast_callback_seq=(\d+)\b", line)
        received = re.search(r"\blast_callback_received_us=(\d+)\b", line)
        if sequence is None and received is None:
            continue
        if sequence is None or received is None:
            raise ValueError("incomplete host callback counter")
        seq, at_us = int(sequence.group(1)), int(received.group(1))
        if seq <= 0 or at_us <= 0 or (seq in observed and observed[seq] != at_us):
            raise ValueError("invalid host callback counter")
        observed[seq] = at_us
        if len(observed) > max_events:
            raise ValueError("host callback counter overflow")
    return [{"stage": "host_state_callback", "sequence": seq, "at_us": at_us,
             "clock": CLOCK} for seq, at_us in sorted(observed.items())]


def _rows(events, stages):
    grouped = defaultdict(dict)
    for event in events:
        if event["stage"] in stages:
            sequence = event["sequence"]
            if sequence in grouped and event["stage"] in grouped[sequence]:
                raise ValueError("duplicate sequence stage")
            grouped[sequence][event["stage"]] = event["at_us"]
    rows = []
    for sequence, times in sorted(grouped.items()):
        missing = [stage for stage in stages if stage not in times]
        observed = [times[stage] for stage in stages if stage in times]
        if observed != sorted(observed):
            raise ValueError("sequence stage clock order")
        row = {"sequence": sequence, "timestamps_us": times, "missing": missing}
        if not missing:
            row["end_to_end_us"] = times[stages[-1]] - times[stages[0]]
        rows.append(row)
    return rows


def _bridge_rows(bridge):
    if bridge and bridge.get("clock") != CLOCK:
        raise ValueError("mixed or missing bridge clock domain")
    result = {}
    for direction in ("peer_to_host", "host_to_peer"):
        value = bridge.get(direction, {})
        chunks = value.get("recent_chunks", [])
        gaps = value.get("long_gaps", [])
        if len(chunks) > MAX_CHUNKS or len(gaps) > MAX_CHUNKS:
            raise ValueError("bridge history overflow")
        for chunk in chunks:
            ingress = chunk.get("ingress_at_us")
            egress = chunk.get("egress_at_us")
            if not isinstance(ingress, int) or not isinstance(egress, int) or ingress <= 0 or egress < ingress:
                raise ValueError("bridge chunk clock order")
        result[direction] = {
            "chunks": value.get("chunks", 0),
            "recent_chunks": chunks,
            "long_gaps": gaps,
            "max_gap_us": value.get("max_gap_us", 0),
            "max_write_us": value.get("max_write_us", 0),
            "sequence_correlation": "unavailable",
        }
    return result


def _peer_state_max_gap(rows):
    largest = None
    for before, after in zip(rows, rows[1:]):
        gap = after["timestamps_us"]["peer_state_publish"] - before["timestamps_us"]["peer_state_publish"]
        if gap < 0:
            raise ValueError("peer state clock order")
        if largest is None or gap > largest["gap_us"]:
            largest = {"before_seq": before["sequence"], "after_seq": after["sequence"],
                       "gap_us": gap}
    return largest


def summarize(events, bridge, max_events=MAX_EVENTS):
    if not isinstance(max_events, int) or not 0 < max_events <= MAX_EVENTS:
        raise ValueError("invalid event bound")
    if len(events) > max_events:
        raise ValueError("trace event overflow")
    stages = STATE_STAGES + COMMAND_STAGES
    for event in events:
        if event.get("clock") != CLOCK:
            raise ValueError("mixed or missing clock domain")
        if event.get("stage") not in stages:
            raise ValueError("unknown trace stage")
        if not isinstance(event.get("sequence"), int) or event["sequence"] <= 0:
            raise ValueError("invalid sequence")
        if not isinstance(event.get("at_us"), int) or event["at_us"] <= 0:
            raise ValueError("invalid timestamp")
    state = _rows(events, STATE_STAGES)
    command = _rows(events, COMMAND_STAGES)
    complete = bool(state and command) and all(not row["missing"] for row in state + command)
    return {
        "evidence_scope": "component_only",
        "clock": CLOCK,
        "status": "MEASURED_COMPONENT_ONLY" if complete else "INCOMPLETE",
        "strict_50ms_pass": False,
        "state": state,
        "peer_state_max_gap": _peer_state_max_gap(state),
        "command": command,
        "bridge": _bridge_rows(bridge),
    }


def write_report(path, report):
    with Path(path).open("x", encoding="utf-8") as output:
        json.dump(report, output, sort_keys=True)
        output.write("\n")
