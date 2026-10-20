"""C05 离线单调时钟时序诊断，不产生合格判定。"""
##
# @file c05_latency_trace.py
# @brief 有界C05日志标记与PTY块的离线时序汇总。
#
# 只读/整理已采日志，clock必须声明steady_monotonic_us，声明不独立证明跨机器时钟已同步。State/Command按各自序号关联，native桥chunk无法映射wire序号；阶段齐全也仅MEASURED_COMPONENT_ONLY，strict_50ms_pass始终false，不自动作策略/硬件合格结论。

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


##
# @brief 从首字符为左花括号的JSON日志提state_trace/command接收标记，其余事件忽略；超过事件上限抛错不截断。
# @param lines 日志文本行序列；本函数只解析，不采样运行进程。
# @param max_events 允许标记数上限，默认4096；超过拒绝不截断。
# @return 有界State发布/Command接收标记列表。
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


##
# @brief 从heartbeat诊断提seq/received配对，拒缺一、非正或同seq时间冲突；同值重复去重后按seq输出。
# @param lines 日志文本行序列；本函数只解析，不采样运行进程。
# @param max_events 允许标记数上限，默认4096；超过拒绝不截断。
# @return 按seq去重排序的Host接收标记。
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


##
# @brief 按sequence归组指定阶段，重复stage或已见时间倒序拒绝；缺阶段显式列missing，只有完整行才算端到端差。
# @param events 带stage/sequence/at_us/clock的已采或合成标记列表。
# @param stages 指定方向的有序阶段名元组。
# @return 每sequence时间字典、missing及可选端到端µs的行列表。
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


##
# @brief 桥必须同声明clock、每方向最近chunk/长gap至多16，核入出µs顺序；保统计但sequence_correlation固定unavailable。
# @param bridge 原生PTY桥诊断字典，空表示无桥证据。
# @return 双方向块统计，wire序号关联均unavailable。
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


##
# @brief 按已归序行计算peer发布最大间隙并保两侧seq；不足两点返回None，不推断丢帧原因。
# @param rows 已按sequence排序、含timestamps_us的State行；计算peer gap要求相应peer时间存在。
# @return 最大gap及两侧seq字典，少于两行None；缺peer字段可抛KeyError。
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


##
# @brief 核最多4096事件、阶段/序号/时间/clock，汇总双流与桥；全部阶段齐也不赋strict50ms通过。
# @param events 带stage/sequence/at_us/clock的已采或合成标记列表。
# @param bridge 原生PTY桥诊断字典，空表示无桥证据。
# @param max_events 允许标记数上限，默认4096；超过拒绝不截断。
# @return component_only时序报告；strict_50ms_pass始终False。
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


##
# @brief 排他x模式创建UTF8 JSON并末尾换行，既有证据文件拒绝覆盖。
# @param path 输入/输出文件Path，具体读写及排他要求见函数职责。
# @param report 要写出的JSON可编码报告字典。
# @return None；只排他写报告文件。
def write_report(path, report):
    with Path(path).open("x", encoding="utf-8") as output:
        json.dump(report, output, sort_keys=True)
        output.write("\n")
