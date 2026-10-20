"""Verify strict non-R00 suite registration and exit behavior."""
##
# @file test_verify_suites.py
# @brief 验收总调度注册、退出码、trace及JUnit的混合本地测试。
#
# 部分mock run_case只测报告装配；run_suite/unittest/action probe方法实际起子进程，所选suite可能编译组件或访问已有ROS图，不能统称纯mock。success_probe只写observed=true JSON，不生成现场测量；固定suite计数/运行时缺失假设属于旧oracle，注释不修断言。

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as element_tree


ROOT = Path(__file__).resolve().parents[3]
VERIFY = ROOT / "scripts/dummy/verify.py"
sys.path.insert(0, str(VERIFY.parent))
import verify  # noqa: E402


##
# @brief 总runner注册/报告/真实子套件调用混合的unittest集合。
class VerifySuitesTest(unittest.TestCase):
    ##
    # @brief 用/usr/bin/python3实际运行verify指定suite/可选case，45s期限并读临时报告，不在此隔离所有子suite副作用。
    # @param suite verify套件名，实际子进程可能运行相应adapter。
    # @param case 单case标识或None运行整套。
    # @param hardware none实际运行；其它模式由verify分派约束，不是OS沙箱。
    # @return (实际verify退出码, JSON报告)。
    def run_suite(
        self, suite: str, *, case: str | None = None, hardware: str = "none",
    ) -> tuple[int, dict]:
        with tempfile.TemporaryDirectory(prefix="verify_suite_") as temporary:
            report = Path(temporary) / "report.json"
            completed = subprocess.run(
                [
                    "/usr/bin/python3", str(VERIFY), "--suite", suite,
                    "--hardware", hardware, "--report", str(report),
                    *( ("--case", case) if case is not None else () ),
                ],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=45,
            )
            self.assertTrue(report.is_file(), completed.stdout + completed.stderr)
            return completed.returncode, json.loads(report.read_text())

    ##
    # @brief 每suite取fixture首case真实运行，要求不是用法错误2且报告只含该case，不要求每case均PASS。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_every_registered_suite_supports_a_registered_single_case(self):
        for suite, definition in verify.SUITES.items():
            fixture = json.loads((verify.TEST_ROOT / "fixtures" / definition["fixture"]).read_text())
            case_id = fixture["cases"][0]
            code, report = self.run_suite(suite, case=case_id)
            self.assertNotEqual(2, code, suite)
            self.assertEqual([case_id], [result["case_id"] for result in report["cases"]])

    ##
    # @brief mock dispatch下plan-only/manual均退出4/NOT_RUN且不调用run_case，不等于none模式同样无副作用。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_plan_only_and_manual_do_not_dispatch_a_case_probe(self):
        with mock.patch.object(verify, "run_case") as dispatch:
            for hardware in ("plan-only", "manual"):
                report, code = verify.run_registered_suite("R01", hardware, "GEN07")
                self.assertEqual(4, code)
                self.assertEqual("NOT_RUN", report["status"])
                self.assertFalse(report["cases"][0]["executed"])
        dispatch.assert_not_called()

    ##
    # @brief 实际AUD01两次写不同UUID trace，核schema/input digest及文件原SHA；来源仍文档utility。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_runner_materializes_required_case_evidence_and_real_unique_traces(self):
        with tempfile.TemporaryDirectory(prefix="verify_trace_") as temporary:
            evidence_dir = Path(temporary) / "traces"
            with mock.patch.dict("os.environ", {"DUMMY_EVIDENCE_DIR": str(evidence_dir)}):
                first, _ = verify.run_registered_suite("R00", "none", "AUD01")
                second, _ = verify.run_registered_suite("R00", "none", "AUD01")
            for result in (first["cases"][0], second["cases"][0]):
                self.assertEqual(1, result["schema"])
                self.assertEqual("document_utility", result["sut_kind"])
                self.assertEqual(64, len(result["input_digest"]))
                trace = Path(result["trace_path"])
                self.assertTrue(trace.is_file())
                self.assertEqual(result["trace_sha256"], __import__("hashlib").sha256(trace.read_bytes()).hexdigest())
            self.assertNotEqual(first["cases"][0]["trace_path"], second["cases"][0]["trace_path"])

    ##
    # @brief 真实运行只输出observed的success fixture，而validator要求required，必须FAIL，退出0不能自动变PASS。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_action_probe_requires_its_case_validator(self):
        from dummy_test_support.task_adapters import run_action_probe_case

        result = run_action_probe_case(
            "LIVE",
            {"case_id": "LIVE", "hardware": "none", "test_only": True},
            "scripts/dummy/tests/fixtures/success_probe.py",
            ("scripts/dummy/tests/fixtures/success_probe.py",),
            validator=lambda observed: observed.get("required") is True,
        )

        self.assertEqual("FAIL", result["status"])
        self.assertIn("validator", result["reason"])

    ##
    # @brief 实际R01全suite，固定期待PASS/7条；依赖当前模型/旧oracle，不证明实体运动。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_r01_runs_registered_model_suite(self):
        code, report = self.run_suite("R01")
        self.assertEqual(0, code)
        self.assertEqual("PASS", report["status"])
        self.assertEqual(7, report["executed_test_count"])

    ##
    # @brief 实际R03全suite，固定期待PASS/7条，可能编译/运行RPC组件，不是纯注册检查。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_r03_runs_registered_wire_rpc_suite(self):
        code, report = self.run_suite("R03")
        self.assertEqual(0, code)
        self.assertEqual("PASS", report["status"])
        self.assertEqual(7, report["executed_test_count"])

    ##
    # @brief 实际R05全suite但oracle假定runtime缺失，期待4/NOT_RUN；有运行图时不是通用预期。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_r05_without_runtime_is_not_run(self):
        code, report = self.run_suite("R05")
        self.assertEqual(4, code)
        self.assertEqual("NOT_RUN", report["status"])

    ##
    # @brief MOD06实际离线模型目标应PASS，ROS11假定无Gazebo时NOT_RUN，后者依赖环境。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_r05_executes_model_contract_and_requires_live_controller(self):
        from dummy_test_support.adapters import r05

        model = r05.run_r05_case("MOD06", {
            "case_id": "MOD06", "hardware": "none", "test_only": True,
        })
        self.assertEqual("PASS", model["status"])
        self.assertTrue(model["executed"])
        controller = r05.run_r05_case("ROS11", {
            "case_id": "ROS11", "hardware": "none", "test_only": True,
        })
        self.assertEqual("NOT_RUN", controller["status"])
        self.assertFalse(controller["executed"])

    ##
    # @brief 实际R04全suite固定PASS/10条，运行的是组件/插件用例，不给实机实时结论。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_r04_runs_nonblocking_plugin_suite(self):
        code, report = self.run_suite("R04")
        self.assertEqual(0, code)
        self.assertEqual("PASS", report["status"])
        self.assertEqual(10, report["executed_test_count"])

    ##
    # @brief 自建RX fixture与两份mock证据，核调度参数及原证据保留，不执行真实FIRST/SECOND目标。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_registered_runner_preserves_each_case_adapter_evidence(self):
        with tempfile.TemporaryDirectory(prefix="verify_cases_") as temporary:
            fixture_root = Path(temporary)
            ##
            # @par 内部回调 evidence
            # 为FIRST/SECOND写真实临时trace及SHA，但source/input摘要和观察内容都是人工值。
            # 参数 case_id：待注册/物化的case标识。
            # 参数 source_digest：人工64hex源码摘要占位，不是本回调重算。
            # 参数 input_digest：人工64hex输入摘要占位。
            # 参数 observation：写入临时trace的人工文本。
            # 返回：人工证据dict，trace文件真实存在但其它摘要为占位。
            def evidence(case_id: str, source_digest: str, input_digest: str, observation: str) -> dict:
                trace_path = fixture_root / f"{case_id}.trace"
                trace_path.write_text(observation + "\n")
                return {
                    "schema": 1, "case_id": case_id, "sut_kind": "document_utility",
                    "executed": True, "status": "PASS", "reason": "",
                    "sut_source_sha256": source_digest, "input_digest": input_digest,
                    "trace_path": str(trace_path.resolve()),
                    "trace_sha256": __import__("hashlib").sha256(trace_path.read_bytes()).hexdigest(),
                    "observations": {"probe": observation},
                }

            first = evidence("FIRST", "a" * 64, "b" * 64, "first")
            second = evidence("SECOND", "c" * 64, "d" * 64, "second")
            (fixture_root / "fixtures").mkdir()
            (fixture_root / "fixtures" / "RX.json").write_text(
                json.dumps({"suite": "RX", "cases": ["FIRST", "SECOND"]})
            )
            definition = {"fixture": "RX.json", "modules": ("scripts/dummy/tests/test_r01_contract.py",)}
            with (
                mock.patch.object(verify, "TEST_ROOT", fixture_root),
                mock.patch.object(verify, "SUITES", {"RX": definition}),
                mock.patch.object(verify, "run_case", side_effect=[first, second]) as run_case,
            ):
                report, code = verify.run_registered_suite("RX", "none")

        self.assertEqual(0, code)
        self.assertEqual([first, second], report["cases"])
        self.assertEqual(
            [
                mock.call("RX", "FIRST", {"case_id": "FIRST", "hardware": "none", "test_only": True}),
                mock.call("RX", "SECOND", {"case_id": "SECOND", "hardware": "none", "test_only": True}),
            ],
            run_case.call_args_list,
        )

    ##
    # @brief 仅核SUITES注册R02与fixture名，不运行R02。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_r02_is_a_registered_fixture_suite(self):
        self.assertIn("R02", verify.SUITES)
        self.assertEqual("R02.json", verify.SUITES["R02"]["fixture"])

    ##
    # @brief mock单case NOT_RUN证据归总NOT_RUN/退出4，保原case不伪PASS。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_runtime_not_run_case_returns_exit_four(self):
        with tempfile.TemporaryDirectory(prefix="verify_not_run_") as temporary:
            fixture_root = Path(temporary)
            trace_path = fixture_root / "ros01.trace"
            trace_path.write_text("required live runtime unavailable\n")
            result = {
                "schema": 1, "case_id": "ROS01", "sut_kind": "ros_launch",
                "executed": False, "status": "NOT_RUN", "reason": "required live runtime unavailable",
                "sut_source_sha256": "a" * 64, "input_digest": "b" * 64,
                "trace_path": str(trace_path.resolve()),
                "trace_sha256": __import__("hashlib").sha256(trace_path.read_bytes()).hexdigest(),
                "observations": {"runtime_available": False},
            }
            (fixture_root / "fixtures").mkdir()
            (fixture_root / "fixtures" / "RX.json").write_text(
                json.dumps({"suite": "RX", "cases": ["ROS01"]})
            )
            with (
                mock.patch.object(verify, "TEST_ROOT", fixture_root),
                mock.patch.object(
                    verify,
                    "SUITES",
                    {"RX": {"fixture": "RX.json", "modules": ("scripts/dummy/tests/test_r01_contract.py",)}},
                ),
                mock.patch.object(verify, "run_case", return_value=result),
            ):
                report, code = verify.run_registered_suite("RX", "none")

        self.assertEqual(4, code)
        self.assertEqual("NOT_RUN", report["status"])
        self.assertEqual([result], report["cases"])

    ##
    # @brief 空cases fixture应退出4/NOT_RUN，不因没有失败就PASS。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_empty_and_unexecuted_fixtures_do_not_pass(self):
        with tempfile.TemporaryDirectory(prefix="verify_empty_") as temporary:
            fixture_root = Path(temporary)
            (fixture_root / "fixtures").mkdir()
            (fixture_root / "fixtures" / "EMPTY.json").write_text(
                json.dumps({"suite": "EMPTY", "cases": []})
            )
            with (
                mock.patch.object(verify, "TEST_ROOT", fixture_root),
                mock.patch.object(
                    verify,
                    "SUITES",
                    {"EMPTY": {"fixture": "EMPTY.json", "modules": ("scripts/dummy/tests/test_r01_contract.py",)}},
                ),
            ):
                report, code = verify.run_registered_suite("EMPTY", "none")

        self.assertEqual(4, code)
        self.assertEqual("NOT_RUN", report["status"])

    ##
    # @brief mock所有SKIP且未执行归NOT_RUN/4，区别公共unittest helper只看Ran的局部规则。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_all_skipped_cases_do_not_pass(self):
        with tempfile.TemporaryDirectory(prefix="verify_skipped_") as temporary:
            fixture_root = Path(temporary)
            trace_path = fixture_root / "skipped.trace"
            trace_path.write_text("probe intentionally skipped\n")
            skipped = {
                "schema": 1, "case_id": "SKIPPED", "sut_kind": "document_utility",
                "executed": False, "status": "SKIP", "reason": "probe intentionally skipped",
                "sut_source_sha256": "a" * 64, "input_digest": "b" * 64,
                "trace_path": str(trace_path.resolve()),
                "trace_sha256": __import__("hashlib").sha256(trace_path.read_bytes()).hexdigest(),
                "observations": {"probe_started": False},
            }
            (fixture_root / "fixtures").mkdir()
            (fixture_root / "fixtures" / "RX.json").write_text(
                json.dumps({"suite": "RX", "cases": ["SKIPPED"]})
            )
            with (
                mock.patch.object(verify, "TEST_ROOT", fixture_root),
                mock.patch.object(
                    verify,
                    "SUITES",
                    {"RX": {"fixture": "RX.json", "modules": ("scripts/dummy/tests/test_r01_contract.py",)}},
                ),
                mock.patch.object(verify, "run_case", return_value=skipped),
            ):
                report, code = verify.run_registered_suite("RX", "none")

        self.assertEqual(4, code)
        self.assertEqual("NOT_RUN", report["status"])

    ##
    # @brief 真实写临时JUnit，FAIL为failure，NOT_RUN/SKIP为skipped，不混成失败或通过。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_junit_distinguishes_failure_from_runtime_skip(self):
        report = {
            "suite": "RX",
            "hardware": "none",
            "cases": [
                {"case_id": "FAILED", "status": "FAIL", "reason": "probe failed"},
                {"case_id": "OFFLINE", "status": "NOT_RUN", "reason": "runtime missing"},
                {"case_id": "SKIPPED", "status": "SKIP", "reason": "excluded"},
            ],
        }
        with tempfile.TemporaryDirectory(prefix="verify_junit_") as temporary:
            path = Path(temporary) / "report.junit.xml"
            verify.write_junit(report, path)
            cases = {case.attrib["name"]: case for case in element_tree.parse(path).getroot()}

        self.assertIsNotNone(cases["FAILED"].find("failure"))
        self.assertIsNone(cases["FAILED"].find("skipped"))
        self.assertIsNotNone(cases["OFFLINE"].find("skipped"))
        self.assertIsNotNone(cases["SKIPPED"].find("skipped"))

    ##
    # @brief BLOCKED序列化到skipped计数和节点，机器报告不误为PASS。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_junit_serializes_blocked_as_skipped(self):
        report = {
            "suite": "RX",
            "cases": [
                {"case_id": "BLOCKED", "status": "BLOCKED", "reason": "missing dependency"},
            ],
        }
        with tempfile.TemporaryDirectory(prefix="verify_blocked_junit_") as temporary:
            path = Path(temporary) / "report.junit.xml"
            verify.write_junit(report, path)
            root = element_tree.parse(path).getroot()
        self.assertEqual("1", root.attrib["skipped"])
        self.assertIsNotNone(root.find(".//skipped"))

    ##
    # @brief mock R08结果经公共trace物化返回，未知suite ValueError，不运行REL01发布复核。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_task_adapter_dispatches_r08_and_rejects_unknown_suite(self):
        from dummy_test_support import task_adapters

        evidence = {
            "case_id": "REL01", "executed": True, "status": "PASS", "reason": "",
            "sut_source_sha256": "a" * 64, "observations": {},
        }
        stimulus = {"case_id": "REL01", "hardware": "none", "test_only": True}
        with mock.patch.object(task_adapters, "run_r08_case", return_value=evidence):
            result = task_adapters.run_case("R08", "REL01", stimulus)
        self.assertEqual("REL01", result["case_id"])
        with self.assertRaises(ValueError):
            task_adapters.run_case("UNKNOWN", "X", {
                "case_id": "X", "hardware": "none", "test_only": True,
            })

    ##
    # @brief 实际verify R00，旧oracle假定ROS runtime缺失、3执行/4case及退出4，未在此执行构建。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_r00_uses_the_common_report_status_and_count_schema(self):
        with tempfile.TemporaryDirectory(prefix="verify_r00_") as temporary:
            report_path = Path(temporary) / "report.json"
            completed = subprocess.run(
                [
                    "/usr/bin/python3", str(VERIFY), "--suite", "R00", "--hardware", "none",
                    "--report", str(report_path),
                ],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=45,
            )
            self.assertTrue(report_path.is_file(), completed.stdout + completed.stderr)
            report = json.loads(report_path.read_text())

        self.assertEqual(4, completed.returncode)
        self.assertEqual("NOT_RUN", report["status"])
        self.assertEqual(3, report["executed_test_count"])
        self.assertEqual(4, len(report["cases"]))

    ##
    # @brief mock MOD05 unittest核未传runtime_action且ROS01不在离线TARGETS，不真实查服务。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_r02_keeps_offline_cases_out_of_live_action_gate(self):
        from dummy_test_support.adapters import r02

        result = {
            "case_id": "MOD05", "executed": True, "status": "PASS", "reason": "",
            "sut_source_sha256": "a" * 64, "trace_sha256": "b" * 64, "observations": {},
        }
        with mock.patch.object(r02, "run_unittest_case", return_value=result) as run_unittest:
            r02.run_r02_case("MOD05", {"case_id": "MOD05", "hardware": "none", "test_only": True})

        self.assertNotIn("runtime_action", run_unittest.call_args.kwargs)
        self.assertNotIn("ROS01", r02.TARGETS)

    ##
    # @brief 公共helper发现不存在source立即FAIL，不启动目标unittest。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_missing_adapter_source_returns_failed_evidence(self):
        from dummy_test_support.task_adapters import run_unittest_case

        result = run_unittest_case(
            "MISSING",
            {"case_id": "MISSING", "hardware": "none", "test_only": True},
            "scripts.dummy.tests.test_r01_contract.R01ContractTest.test_model_bundle_is_complete_and_hash_verified",
            ("does/not/exist.py",),
        )

        self.assertEqual("FAIL", result["status"])
        self.assertTrue(result["observations"]["missing_source_paths"])

    ##
    # @brief 实际跑模型bundle目标并核输出摘要长度，依赖目标原oracle，不是摘要值正确性完整重算。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_unittest_evidence_includes_actual_subtest_output_digest(self):
        from dummy_test_support.task_adapters import run_unittest_case

        result = run_unittest_case(
            "OUTPUT",
            {"case_id": "OUTPUT", "hardware": "none", "test_only": True},
            "scripts.dummy.tests.test_r01_contract.R01ContractTest.test_model_bundle_is_complete_and_hash_verified",
            ("scripts/dummy/tests/test_r01_contract.py",),
        )

        self.assertEqual("PASS", result["status"])
        self.assertEqual(64, len(result["observations"]["subtest_output_sha256"]))

    ##
    # @brief 真实起success_probe写JSON后validator核observed=True；real JSON仅指文件实际读写，不是live设备观测。
    # @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
    def test_action_probe_successfully_reads_real_json_observations(self):
        from dummy_test_support.task_adapters import run_action_probe_case

        result = run_action_probe_case(
            "LIVE",
            {"case_id": "LIVE", "hardware": "none", "test_only": True},
            "scripts/dummy/tests/fixtures/success_probe.py",
            ("scripts/dummy/tests/fixtures/success_probe.py",),
            validator=lambda observed: observed.get("observed") is True,
        )

        self.assertEqual("PASS", result["status"])
        self.assertTrue(result["executed"])
        self.assertEqual({"observed": True}, result["observations"]["probe_observations"])


if __name__ == "__main__":
    unittest.main()
