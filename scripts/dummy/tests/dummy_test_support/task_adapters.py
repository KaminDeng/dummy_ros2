"""Dispatch test cases to their suite-specific adapters."""
##
# @file task_adapters.py
# @brief 验收suite分派、真实子进程运行和trace文件装配的公共入口。
#
# hardware=none/test_only是严格输入约定，不是OS沙箱：所选suite仍可编译C++、起ROS/peer或发仿真Action。source摘要绑定当前指定文件，trace摘要绑定生成文件字节；二者不能证明运行二进制来源、覆盖充分或实体许可。

from __future__ import annotations

from hashlib import sha256
import json
import os
import re
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any, Callable
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[4]
EvidenceValidator = Callable[[dict[str, Any]], bool | tuple[bool, str]]


##
# @brief 依source_paths顺序拼相对名与原字节/缺失标记，返回摘要和缺失列表，不执行源文件。
# @param source_paths 相对ROS根的有序源码路径元组，用于当前文件摘要/缺失检查。
# @return (源文件拼接SHA-256, 缺失相对路径列表)。
def _source_evidence(source_paths: tuple[str, ...]) -> tuple[str, list[str]]:
    digest = sha256()
    missing: list[str] = []
    for relative in source_paths:
        path = ROOT / relative
        digest.update(relative.encode())
        if path.is_file():
            digest.update(path.read_bytes())
        else:
            digest.update(b"<missing>")
            missing.append(relative)
    return digest.hexdigest(), missing


##
# @brief 缺源统一FAIL/executed=False，trace记录case/target/缺失名，不把无法绑定源码当NOT_RUN成功。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param target 不能执行的目标标识，写trace以区别case。
# @param source_hash 调用方已算的当前源码摘要hex，不是运行二进制SHA。
# @param missing 缺失的源码相对路径列表。
# @return FAIL且executed=False的诊断结果。
def _missing_source_result(
    case_id: str, target: str, source_hash: str, missing: list[str],
) -> dict[str, Any]:
    output = f"case={case_id};target={target};missing_sources={','.join(missing)}"
    return {
        "case_id": case_id, "executed": False, "status": "FAIL", "reason": output,
        "sut_source_sha256": source_hash, "trace_sha256": sha256(output.encode()).hexdigest(),
        "_trace_text": output,
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{target}",
            "missing_source_paths": missing, "subtest_output_sha256": sha256(output.encode()).hexdigest(),
        },
    }


##
# @brief 验证stimulus并检查源文件；缺源优先FAIL，否则NOT_RUN，executed=False并说明缺case探针。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @param source_paths 相对ROS根的有序源码路径元组，用于当前文件摘要/缺失检查。
# @param reason 人类可读失败/未运行原因。
# @param sut_kind 证据类型标签，默认值不提升实际覆盖层级。
# @return NOT_RUN或源缺失FAIL结果。
def not_run_case(
    case_id: str, stimulus: dict[str, Any], source_paths: tuple[str, ...], reason: str,
    *, sut_kind: str = "document_utility",
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, "not-run", source_hash, missing)
    trace = f"case={case_id};status=NOT_RUN;reason={reason}"
    return {
        "case_id": case_id, "executed": False, "status": "NOT_RUN", "reason": reason,
        "sut_source_sha256": source_hash, "trace_sha256": sha256(trace.encode()).hexdigest(),
        "sut_kind": sut_kind, "_trace_text": trace,
        "observations": {"case_id": case_id, "case_specific_probe_available": False},
    }


##
# @brief 可先查询唯一公开Action再跑指定unittest≤90s；PASS仅依退出0且Ran计数>0，本helper未单独排除全skip。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @param target unittest可加载的点分模块/类/方法名。
# @param source_paths 相对ROS根的有序源码路径元组，用于当前文件摘要/缺失检查。
# @param runtime_action 可选需唯一server的Action名；None不做该预检。
# @param sut_kind 证据类型标签，默认值不提升实际覆盖层级。
# @param python_executable 可选解释器路径，None使用当前sys.executable。
# @return 单目标测试证据；PASS依据退出0且Ran>0，不单独过滤skip。
def run_unittest_case(
    case_id: str,
    stimulus: dict[str, Any],
    target: str,
    source_paths: tuple[str, ...],
    runtime_action: str | None = None,
    sut_kind: str = "document_utility",
    python_executable: str | None = None,
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, target, source_hash, missing)
    if runtime_action is not None:
        try:
            availability = subprocess.run(
                ["ros2", "action", "info", runtime_action, "-t"], cwd=ROOT,
                capture_output=True, text=True, check=False, timeout=10,
            )
            available = availability.returncode == 0 and "Action servers: 1" in availability.stdout
            availability_trace = availability.stdout + availability.stderr
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            available = False
            availability_trace = str(error)
        if not available:
            reason = f"required live action server unavailable: {runtime_action}"
            trace = f"case={case_id};{reason};{availability_trace}"
            return {
                "case_id": case_id, "executed": False, "status": "NOT_RUN", "reason": reason,
                "sut_source_sha256": source_hash, "trace_sha256": sha256(trace.encode()).hexdigest(),
                "sut_kind": sut_kind, "_trace_text": trace,
                "observations": {
                    "case_id": case_id, "case_target": f"{case_id}:{target}",
                    "runtime_action": runtime_action, "runtime_available": False,
                    "subtest_output_sha256": sha256(availability_trace.encode()).hexdigest(),
                },
            }
    try:
        completed = subprocess.run(
            [python_executable or sys.executable, "-m", "unittest", target, "-v"], cwd=ROOT,
            capture_output=True, text=True, check=False, timeout=90,
        )
        output = completed.stdout + completed.stderr
    except subprocess.TimeoutExpired as error:
        output = (error.stdout or "") + (error.stderr or "")
        completed = subprocess.CompletedProcess(error.cmd, 124, output, "")
    match = re.search(r"Ran (\d+) tests?", output)
    count = int(match.group(1)) if match else 0
    ##
    # @par 测试计数边界
    # 只解析Ran并检查退出0；unittest全skip若仍Ran>0也可能通过此通用helper，不能套用R00专用分类器的全skip拒绝保证。
    passed = completed.returncode == 0 and count > 0
    trace = f"case={case_id};target={target};returncode={completed.returncode}\n{output}"
    return {
        "case_id": case_id, "executed": count > 0, "status": "PASS" if passed else "FAIL",
        "reason": "" if passed else output[-4000:], "sut_source_sha256": source_hash,
        "trace_sha256": sha256(trace.encode()).hexdigest(),
        "sut_kind": sut_kind, "_trace_text": trace,
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{target}",
            "return_code": completed.returncode, "executed_test_count": count,
            "subtest_output_sha256": sha256(output.encode()).hexdigest(), "requirement_observed": passed,
        },
    }


##
# @brief 可先查Action，再以90s期限跑probe写临时JSON，退出3为NOT_RUN、0且validator通过才PASS，其它失败保FAIL。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @param probe_path ROS根下probe脚本相对路径，调用会实际运行它。
# @param source_paths 相对ROS根的有序源码路径元组，用于当前文件摘要/缺失检查。
# @param validator 对probe JSON返回bool或(bool, reason)的case校验函数。
# @param sut_kind 证据类型标签，默认值不提升实际覆盖层级。
# @param runtime_action 可选需唯一server的Action名；None不做该预检。
# @return probe执行/不可用/validator结论与trace结果。
def run_action_probe_case(
    case_id: str,
    stimulus: dict[str, Any],
    probe_path: str,
    source_paths: tuple[str, ...],
    *,
    validator: EvidenceValidator,
    sut_kind: str = "ros_launch",
    runtime_action: str | None = None,
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, probe_path, source_hash, missing)
    if runtime_action is not None:
        try:
            availability = subprocess.run(
                ["ros2", "action", "info", runtime_action, "-t"], cwd=ROOT,
                capture_output=True, text=True, check=False, timeout=10,
            )
            available = availability.returncode == 0 and "Action servers: 1" in availability.stdout
            availability_trace = availability.stdout + availability.stderr
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            available = False
            availability_trace = str(error)
        if not available:
            reason = f"required live action server unavailable: {runtime_action}"
            trace = f"case={case_id};{reason};{availability_trace}"
            return {
                "case_id": case_id, "executed": False, "status": "NOT_RUN", "reason": reason,
                "sut_source_sha256": source_hash, "trace_sha256": sha256(trace.encode()).hexdigest(),
                "sut_kind": sut_kind, "_trace_text": trace,
                "observations": {
                    "case_id": case_id, "case_target": f"{case_id}:{probe_path}",
                    "runtime_action": runtime_action, "runtime_available": False,
                    "subtest_output_sha256": sha256(availability_trace.encode()).hexdigest(),
                },
            }
    with tempfile.TemporaryDirectory(prefix=f"{case_id.lower()}_probe_") as temporary:
        report = Path(temporary) / "observations.json"
        try:
            completed = subprocess.run(
                [sys.executable, str(ROOT / probe_path), "--report", str(report)], cwd=ROOT,
                capture_output=True, text=True, check=False, timeout=90,
            )
            output = completed.stdout + completed.stderr
        except subprocess.TimeoutExpired as error:
            output = (error.stdout or "") + (error.stderr or "")
            completed = subprocess.CompletedProcess(error.cmd, 124, output, "")
        try:
            observed = json.loads(report.read_text()) if report.is_file() else {}
        except json.JSONDecodeError as error:
            observed = {}
            output += f"\ninvalid probe JSON: {error}"
    trace = f"case={case_id};probe={probe_path};returncode={completed.returncode}\n{output}"
    if completed.returncode == 3:
        status, executed, reason = "NOT_RUN", False, output[-4000:] or "required live runtime unavailable"
    elif completed.returncode == 0 and isinstance(observed, dict):
        validation = validator(observed)
        valid, validation_reason = (
            validation if isinstance(validation, tuple) else (validation, "case validator rejected probe observations")
        )
        status, executed, reason = (
            ("PASS", True, "") if valid else ("FAIL", True, validation_reason)
        )
    else:
        status, executed, reason = "FAIL", True, output[-4000:] or "probe produced no observations"
    return {
        "case_id": case_id, "executed": executed, "status": status, "reason": reason,
        "sut_source_sha256": source_hash, "trace_sha256": sha256(trace.encode()).hexdigest(),
        "sut_kind": sut_kind, "_trace_text": trace,
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{probe_path}",
            "probe_return_code": completed.returncode,
            "subtest_output_sha256": sha256(output.encode()).hexdigest(),
            "probe_observations": observed,
        },
    }


##
# @brief 临时以g++17/Wall/Wextra/Werror/pthread编译单源再运行，各90s；fatal error编译失败归BLOCKED，其余失败FAIL。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @param source_path 需临时编译的单个C++源码相对路径。
# @param source_paths 相对ROS根的有序源码路径元组，用于当前文件摘要/缺失检查。
# @param observation_name 将编译/执行成功布尔写入observations时的字段名。
# @return 编译/执行状态与指定观测布尔的core_binary结果。
def run_cpp_probe_case(
    case_id: str,
    stimulus: dict[str, Any],
    source_path: str,
    source_paths: tuple[str, ...],
    observation_name: str,
) -> dict[str, Any]:
    expected = {"case_id": case_id, "hardware": "none", "test_only": True}
    if stimulus != expected:
        raise ValueError("adapter stimulus is invalid")
    source_hash, missing = _source_evidence(source_paths)
    if missing:
        return _missing_source_result(case_id, source_path, source_hash, missing)
    with tempfile.TemporaryDirectory(prefix=f"{case_id.lower()}_cpp_") as temporary:
        binary = Path(temporary) / "probe"
        compile_command = [
            "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
            "-I", str(ROOT / "src/dummy_ros_control/include"), str(ROOT / source_path), "-o", str(binary),
        ]
        try:
            compiled = subprocess.run(
                compile_command, cwd=ROOT, capture_output=True, text=True, check=False, timeout=90,
            )
            executed = (
                subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False, timeout=90)
                if compiled.returncode == 0 else None
            )
            output = compiled.stdout + compiled.stderr
            if executed is not None:
                output += executed.stdout + executed.stderr
        except (FileNotFoundError, subprocess.TimeoutExpired) as error:
            compiled = subprocess.CompletedProcess(compile_command, 124, "", str(error))
            executed = None
            output = str(error)
    passed = compiled.returncode == 0 and executed is not None and executed.returncode == 0
    blocked = compiled.returncode != 0 and "fatal error:" in output
    trace = f"case={case_id};compile={' '.join(compile_command)}\n{output}"
    return {
        "case_id": case_id, "executed": executed is not None,
        "status": "PASS" if passed else ("BLOCKED" if blocked else "FAIL"),
        "reason": "" if passed else output[-4000:], "sut_source_sha256": source_hash,
        "trace_sha256": sha256(trace.encode()).hexdigest(), "sut_kind": "core_binary", "_trace_text": trace,
        "observations": {
            "case_id": case_id, "case_target": f"{case_id}:{source_path}",
            "compile_return_code": compiled.returncode,
            "probe_return_code": None if executed is None else executed.returncode,
            observation_name: passed,
        },
    }


from dummy_test_support.adapters.r00 import run_r00_case
from dummy_test_support.adapters.r01 import run_r01_case
from dummy_test_support.adapters.r02 import run_r02_case
from dummy_test_support.adapters.r03 import run_r03_case
from dummy_test_support.adapters.r04 import run_r04_case
from dummy_test_support.adapters.r05 import run_r05_case
from dummy_test_support.adapters.r06 import run_r06_case
from dummy_test_support.adapters.r07 import run_r07_case
from dummy_test_support.adapters.r08 import run_r08_case


##
# @brief 按ensure_ascii/sort_keys/紧凑JSON编码stimulus再SHA，不是外部签名或授权令牌。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 规范紧凑JSON字节的SHA-256。
def _input_digest(stimulus: dict[str, Any]) -> str:
    encoded = json.dumps(stimulus, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode()
    return sha256(encoded).hexdigest()


##
# @brief 浅拷贝结果、补schema/sut_kind/input_digest，写独立UUID trace并以实际文件含末尾换行字节重算SHA。
# @param result adapter原结果字典，函数浅拷贝后补trace文件信息。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 带schema/input_digest/trace绝对路径及实际文件SHA的新浅拷贝。
def _materialize_evidence(result: dict[str, Any], stimulus: dict[str, Any]) -> dict[str, Any]:
    evidence = dict(result)
    evidence["schema"] = 1
    evidence["sut_kind"] = evidence.get("sut_kind", "document_utility")
    evidence["input_digest"] = _input_digest(stimulus)
    ##
    # @par trace物化
    # 原_trace_text优先；缺它才序列化结果字典。末尾新添换行后重算实际trace文件SHA，替换adapter先前的trace摘要。
    trace_text = str(evidence.pop("_trace_text", ""))
    if not trace_text:
        trace_text = json.dumps(evidence, ensure_ascii=True, sort_keys=True, default=str)
    ##
    # @par 写入副作用
    # DUMMY_EVIDENCE_DIR可显式覆盖输出根；默认写工作树build_dummy/reports/evidence，UUID避免普通同case名冲突，但不是外部防篡改存储。
    trace_dir = Path(os.environ.get("DUMMY_EVIDENCE_DIR", ROOT / "build_dummy/reports/evidence"))
    trace_dir.mkdir(parents=True, exist_ok=True)
    trace_path = trace_dir / f"{evidence['case_id'].lower()}-{uuid4().hex}.trace"
    trace_path.write_text(trace_text + "\n")
    evidence["trace_path"] = str(trace_path.resolve())
    evidence["trace_sha256"] = sha256(trace_path.read_bytes()).hexdigest()
    return evidence


##
# @brief 按R00–R08选择adapter，未知suite拒绝；执行完成后必物化trace文件，默认写build_dummy/reports/evidence。
# @param suite R00–R08套件名。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @param stimulus 输入字典；公共runner要求恰为case_id、hardware=none、test_only=True，不是硬件沙箱。
# @return 所选adapter结果经trace物化后的字典。
def run_case(suite: str, case_id: str, stimulus: dict[str, Any]) -> dict[str, Any]:
    if suite == "R00":
        result = run_r00_case(case_id, stimulus)
    elif suite == "R01":
        result = run_r01_case(case_id, stimulus)
    elif suite == "R02":
        result = run_r02_case(case_id, stimulus)
    elif suite == "R03":
        result = run_r03_case(case_id, stimulus)
    elif suite == "R04":
        result = run_r04_case(case_id, stimulus)
    elif suite == "R05":
        result = run_r05_case(case_id, stimulus)
    elif suite == "R06":
        result = run_r06_case(case_id, stimulus)
    elif suite == "R07":
        result = run_r07_case(case_id, stimulus)
    elif suite == "R08":
        result = run_r08_case(case_id, stimulus)
    else:
        raise ValueError(f"unsupported suite: {suite}")
    return _materialize_evidence(result, stimulus)
