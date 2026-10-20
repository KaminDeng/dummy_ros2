#!/usr/bin/env python3
##
# @file verify.py
# @brief 按R00至R08夹具调度适配器并生成JSON/JUnit/文本证据。
#
# 默认hardware=none仍会调用对应run_case适配器，可能运行其软件工具/进程；plan-only/manual只生成NOT_RUN，不自动探测或移动硬件。
# runner校验已有trace文件摘要，但sut_source_sha256/input_digest只核文本长度，不独立重算源/刺激；字段存在和PASS不是机械资格证明。
"""Strict suite runner for the Dummy ROS project."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from uuid import uuid4
import xml.etree.ElementTree as element_tree


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
TEST_ROOT = WORKSPACE_ROOT / "scripts" / "dummy" / "tests"
sys.path.insert(0, str(TEST_ROOT))

from dummy_test_support.task_adapters import run_case  # noqa: E402


##
# @brief 聚合退出码：PASS0、未知suite/case2、BLOCKED3、未执行/跳过4，FAIL1优先。
# SUITES指定fixture文件；modules/timeout元数据并未在本runner循环直接读取，不可据此声称这里逐项强制其超时。
EXIT_PASS = 0
EXIT_UNKNOWN_SUITE = 2
EXIT_BLOCKED = 3
EXIT_NOT_RUN = 4
SUITES = {
    "R00": {
        "fixture": "R00.json",
    },
    "R01": {
        "fixture": "R01.json",
    },
    "R02": {
        "fixture": "R02.json",
    },
    "R03": {
        "fixture": "R03.json",
    },
    "R04": {
        "fixture": "R04.json",
    },
    "R05": {
        "fixture": "R05.json",
    },
    "R06": {
        "fixture": "R06.json",
        "modules": ("scripts/dummy/tests/test_r06_acceptance.py",),
    },
    "R07": {
        "fixture": "R07.json",
        "modules": ("scripts/dummy/tests/test_r07_acceptance.py",),
        "timeout": 180,
    },
    "R08": {
        "fixture": "R08.json",
        "modules": ("scripts/dummy/tests/test_r08_acceptance.py",),
    },
}


##
# @brief 把每case状态转成JUnit XML。
# @param report 含suite/cases的聚合映射。
# @param path 写入文件，不在此创建父目录。
# @return None；I/O或缺字段异常传播。
# FAIL计failure；BLOCKED/NOT_RUN/SKIP计skipped。JUnit跳过仅表示未满足运行覆盖，不能被解释为通过或批准。
def write_junit(report: dict[str, object], path: Path) -> None:
    cases = report["cases"]
    suite = element_tree.Element(
        "testsuite",
        name=str(report["suite"]),
        tests=str(len(cases)),
        failures=str(sum(result["status"] == "FAIL" for result in cases)),
        skipped=str(sum(result["status"] in {"BLOCKED", "NOT_RUN", "SKIP"} for result in cases)),
    )
    for result in cases:
        case = element_tree.SubElement(suite, "testcase", name=result["case_id"])
        if result["status"] == "FAIL":
            element_tree.SubElement(case, "failure", message=result["reason"] or result["status"])
        elif result["status"] in {"BLOCKED", "NOT_RUN", "SKIP"}:
            element_tree.SubElement(case, "skipped", message=result["reason"] or result["status"])
    path.write_text(element_tree.tostring(suite, encoding="unicode") + "\n")


##
# @brief 写逐case状态、trace SHA及非空原因的简洁日志。
# @param report 聚合报告。
# @param path 目标文本文件。
# @return None；不复查trace内容，写异常传播。
def write_log(report: dict[str, object], path: Path) -> None:
    lines = [f"suite={report['suite']}", f"hardware={report['hardware']}"]
    for result in report["cases"]:
        lines.append(
            f"case={result['case_id']} status={result['status']} trace={result['trace_sha256']}"
        )
        if result["reason"]:
            lines.append(f"reason={result['reason']}")
    path.write_text("\n".join(lines) + "\n")


##
# @brief 把一次runner/适配器诊断写成唯一trace并计算原字节SHA。
# @param case_id 用于文件名前缀的夹具ID。
# @param trace 待写文本，会附末尾换行。
# @return (绝对路径字符串,实际trace文件SHA)。
# 目录由DUMMY_EVIDENCE_DIR覆盖，缺省build_dummy/reports/evidence；每次uuid新文件，不覆盖同名历史trace。不对任意外部case_id做路径净化。
def _write_trace(case_id: str, trace: str) -> tuple[str, str]:
    directory = Path(os.environ.get("DUMMY_EVIDENCE_DIR", WORKSPACE_ROOT / "build_dummy/reports/evidence"))
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{case_id.lower()}-{uuid4().hex}.trace"
    path.write_text(trace + "\n")
    return str(path.resolve()), hashlib.sha256(path.read_bytes()).hexdigest()


##
# @brief 把runner发现的无效适配器结果转换成有trace的FAIL。
# @param case_id 应执行的case标识。
# @param reason 结构/异常原因。
# @return schema1 FAIL证据映射，executed=true指runner确实执行了验证/记录。
# sut_source_sha256/input_digest均由case_id字节hash占位，不能称真实被测源码或完整刺激摘要。
def _invalid_result(case_id: str, reason: str) -> dict[str, object]:
    trace_path, trace_sha256 = _write_trace(case_id, reason)
    return {
        "schema": 1, "case_id": case_id, "sut_kind": "document_utility",
        "executed": True,
        "status": "FAIL",
        "reason": reason,
        "sut_source_sha256": hashlib.sha256(case_id.encode()).hexdigest(),
        "input_digest": hashlib.sha256(case_id.encode()).hexdigest(),
        "trace_path": trace_path, "trace_sha256": trace_sha256,
        "observations": {"runner_validation_error": reason},
    }


##
# @brief 为plan-only/manual显式记录未执行。
# @param case_id 夹具case ID。
# @param hardware plan-only或manual标记。
# @return executed=false/NOT_RUN且hardware_accessed=false的报告，并写诊断trace。
# manual表示待人工执行，不会自动启动probe；source摘要仍为case_id占位，input_digest才是本地stimulus规范JSON SHA。
def _hardware_not_run_result(case_id: str, hardware: str) -> dict[str, object]:
    reason = f"--hardware {hardware} does not execute probes automatically"
    stimulus = {"case_id": case_id, "hardware": hardware, "test_only": True}
    trace_path, trace_sha256 = _write_trace(case_id, json.dumps({"reason": reason, "stimulus": stimulus}))
    return {
        "schema": 1, "case_id": case_id,
        "sut_kind": "manual_hardware" if hardware == "manual" else "document_utility",
        "executed": False, "status": "NOT_RUN", "reason": reason,
        "sut_source_sha256": hashlib.sha256(case_id.encode()).hexdigest(),
        "input_digest": hashlib.sha256(
            json.dumps(stimulus, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest(),
        "trace_path": trace_path, "trace_sha256": trace_sha256,
        "observations": {"hardware_accessed": False, "hardware_mode": hardware},
    }


##
# @brief 核适配器结果结构及实际trace文件摘要。
# @param case_id 请求的case标识，必须精确匹配。
# @param result 适配器对象，应为含必需字段的dict。
# @return 原合法结果或新建FAIL诊断结果。
# 核schema/sut_kind/executed/status/reason/observations类型，三个摘要仅检查字符串长度64，未核hex形状或重算sut/input。
# trace_path必须为存在绝对文件且文件SHA匹配；检查和后续读取非原子，不是外部签名认证。
def _validate_case_result(case_id: str, result: object) -> dict[str, object]:
    if not isinstance(result, dict):
        return _invalid_result(case_id, "adapter did not return an evidence object")
    required = {
        "schema", "case_id", "sut_kind", "executed", "status", "reason",
        "sut_source_sha256", "input_digest", "trace_path", "trace_sha256", "observations",
    }
    missing = sorted(required.difference(result))
    if missing:
        return _invalid_result(case_id, f"adapter evidence missing fields: {', '.join(missing)}")
    if result["case_id"] != case_id:
        return _invalid_result(case_id, "adapter evidence case_id does not match requested fixture case")
    if result["schema"] != 1 or not isinstance(result["sut_kind"], str) or not result["sut_kind"]:
        return _invalid_result(case_id, "adapter evidence schema/sut_kind is invalid")
    if not isinstance(result["executed"], bool):
        return _invalid_result(case_id, "adapter evidence executed must be boolean")
    if result["status"] not in {"PASS", "FAIL", "BLOCKED", "NOT_RUN", "SKIP"}:
        return _invalid_result(case_id, f"adapter evidence status is invalid: {result['status']!r}")
    if not isinstance(result["reason"], str) or not isinstance(result["observations"], dict):
        return _invalid_result(case_id, "adapter evidence reason/observations has invalid type")
    for field in ("sut_source_sha256", "input_digest", "trace_sha256"):
        value = result[field]
        if not isinstance(value, str) or len(value) != 64:
            return _invalid_result(case_id, f"adapter evidence {field} is not a SHA-256 digest")
    trace_path = Path(result["trace_path"])
    if not trace_path.is_absolute() or not trace_path.is_file():
        return _invalid_result(case_id, "adapter evidence trace_path is not an existing absolute file")
    if hashlib.sha256(trace_path.read_bytes()).hexdigest() != result["trace_sha256"]:
        return _invalid_result(case_id, "adapter evidence trace_sha256 does not match trace_path")
    return result


##
# @brief 按优先级聚合已有case结果。
# @param results 校验后的case映射列表。
# @return (状态字符串,退出码)：FAIL优先，其次BLOCKED、NOT_RUN；SKIP或未executed降为NOT_RUN；全部真实PASS才0。
# 空列表不是空成功，返回NOT_RUN/4。
def _suite_status(results: list[dict[str, object]]) -> tuple[str, int]:
    if not results:
        return "NOT_RUN", EXIT_NOT_RUN
    if any(result["status"] == "FAIL" for result in results):
        return "FAIL", 1
    if any(result["status"] == "BLOCKED" for result in results):
        return "BLOCKED", EXIT_BLOCKED
    if any(result["status"] == "NOT_RUN" for result in results):
        return "NOT_RUN", EXIT_NOT_RUN
    if any(not result["executed"] or result["status"] == "SKIP" for result in results):
        return "NOT_RUN", EXIT_NOT_RUN
    if all(result["status"] == "PASS" and result["executed"] for result in results):
        return "PASS", EXIT_PASS
    return "NOT_RUN", EXIT_NOT_RUN


##
# @brief 读取注册fixture并逐case生成/验证证据。
# @param suite SUITES中已存在的键；本函数直接索引，不自行处理未知键。
# @param hardware none会调用适配器，plan-only/manual生成未执行记录。
# @param selected_case 可选精确case ID，必须位于fixture cases中。
# @return (聚合报告,退出码)；未知case为UNKNOWN_CASE/2，文件/fixture解析异常可传播。
# 适配器异常转换FAIL仍保留报告；只有字符串case ID才交run_case。上下文test_only=true是测试声明，不等同对适配器所有副作用的强制沙箱。
def run_registered_suite(
    suite: str, hardware: str, selected_case: str | None = None,
) -> tuple[dict[str, object], int]:
    definition = SUITES[suite]
    fixture = json.loads((TEST_ROOT / "fixtures" / definition["fixture"]).read_text())
    cases = fixture.get("cases")
    if not isinstance(cases, list):
        results = [_invalid_result("<fixture>", "fixture cases must be a list")]
    else:
        if selected_case is not None:
            if selected_case not in cases:
                return {
                    "suite": suite, "hardware": hardware, "status": "UNKNOWN_CASE",
                    "executed_test_count": 0, "cases": [],
                }, EXIT_UNKNOWN_SUITE
            cases = [selected_case]
        results = []
        for case_id in cases:
            if not isinstance(case_id, str):
                results.append(_invalid_result("<fixture>", "fixture case identifiers must be strings"))
                continue
            if hardware in {"plan-only", "manual"}:
                result = _hardware_not_run_result(case_id, hardware)
            else:
                try:
                    result = run_case(suite, case_id, {
                        "case_id": case_id, "hardware": hardware, "test_only": True,
                    })
                except Exception as error:  # Keep a report even when an adapter fails unexpectedly.
                    result = _invalid_result(case_id, f"adapter raised {type(error).__name__}: {error}")
            results.append(_validate_case_result(case_id, result))
    status, exit_code = _suite_status(results)
    return {
        "schema": 1, "suite": suite, "hardware": hardware, "status": status,
        "executed_test_count": sum(bool(result["executed"]) for result in results),
        "cases": results,
    }, exit_code


##
# @brief 检查suite后写JSON、JUnit和log三种报告。
# @return 聚合退出码；未知suite直接2且不写报告，未捕获的读写/解析异常可传播。
# 三个产物顺序写入而非原子多文件事务；只执行被选套件/case，不据fixture存在宣称全部回归已运行。
def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", required=True)
    parser.add_argument("--case")
    parser.add_argument("--hardware", default="none", choices=("none", "plan-only", "manual"))
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.suite not in SUITES:
        return EXIT_UNKNOWN_SUITE
    report, exit_code = run_registered_suite(
        arguments.suite, arguments.hardware, arguments.case,
    )
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    arguments.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    write_junit(report, arguments.report.with_suffix(".junit.xml"))
    write_log(report, arguments.report.with_suffix(".log"))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
