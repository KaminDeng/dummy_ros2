"""Verify strict non-R00 suite registration and exit behavior."""

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


class VerifySuitesTest(unittest.TestCase):
    def run_suite(self, suite: str) -> tuple[int, dict]:
        with tempfile.TemporaryDirectory(prefix="verify_suite_") as temporary:
            report = Path(temporary) / "report.json"
            completed = subprocess.run(
                ["/usr/bin/python3", str(VERIFY), "--suite", suite, "--hardware", "none", "--report", str(report)],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=45,
            )
            self.assertTrue(report.is_file(), completed.stdout + completed.stderr)
            return completed.returncode, json.loads(report.read_text())

    def test_r01_runs_registered_model_suite(self):
        code, report = self.run_suite("R01")
        self.assertEqual(0, code)
        self.assertEqual("PASS", report["status"])
        self.assertEqual(7, report["executed_test_count"])

    def test_r03_runs_registered_wire_rpc_suite(self):
        code, report = self.run_suite("R03")
        self.assertEqual(0, code)
        self.assertEqual("PASS", report["status"])
        self.assertEqual(7, report["executed_test_count"])

    def test_r05_without_runtime_is_not_run(self):
        code, report = self.run_suite("R05")
        self.assertEqual(3, code)
        self.assertEqual("NOT_RUN", report["status"])

    def test_r05_keeps_model_and_controller_contract_cases_offline(self):
        from dummy_test_support.adapters import r05

        evidence = {
            "case_id": "case", "executed": True, "status": "PASS", "reason": "",
            "sut_source_sha256": "a" * 64, "trace_sha256": "b" * 64,
            "observations": {},
        }
        with mock.patch.object(r05, "run_unittest_case", return_value=evidence) as execute:
            for case_id in ("MOD06", "ROS11"):
                r05.run_r05_case(case_id, {
                    "case_id": case_id, "hardware": "none", "test_only": True,
                })
                self.assertIsNone(execute.call_args.kwargs["runtime_action"])

    def test_r04_runs_nonblocking_plugin_suite(self):
        code, report = self.run_suite("R04")
        self.assertEqual(0, code)
        self.assertEqual("PASS", report["status"])
        self.assertEqual(10, report["executed_test_count"])

    def test_registered_runner_preserves_each_case_adapter_evidence(self):
        first = {
            "case_id": "FIRST", "executed": True, "status": "PASS", "reason": "",
            "sut_source_sha256": "a" * 64, "trace_sha256": "b" * 64,
            "observations": {"probe": "first"},
        }
        second = {
            "case_id": "SECOND", "executed": True, "status": "PASS", "reason": "",
            "sut_source_sha256": "c" * 64, "trace_sha256": "d" * 64,
            "observations": {"probe": "second"},
        }
        with tempfile.TemporaryDirectory(prefix="verify_cases_") as temporary:
            fixture_root = Path(temporary)
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

    def test_r02_is_a_registered_fixture_suite(self):
        self.assertIn("R02", verify.SUITES)
        self.assertEqual("R02.json", verify.SUITES["R02"]["fixture"])

    def test_runtime_not_run_case_returns_exit_three(self):
        result = {
            "case_id": "ROS01", "executed": False, "status": "NOT_RUN",
            "reason": "required live runtime unavailable", "sut_source_sha256": "a" * 64,
            "trace_sha256": "b" * 64, "observations": {"runtime_available": False},
        }
        with tempfile.TemporaryDirectory(prefix="verify_not_run_") as temporary:
            fixture_root = Path(temporary)
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

        self.assertEqual(3, code)
        self.assertEqual("NOT_RUN", report["status"])
        self.assertEqual([result], report["cases"])

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
        self.assertEqual("UNEXECUTED", report["status"])

    def test_all_skipped_cases_do_not_pass(self):
        skipped = {
            "case_id": "SKIPPED", "executed": False, "status": "SKIP",
            "reason": "probe intentionally skipped", "sut_source_sha256": "a" * 64,
            "trace_sha256": "b" * 64, "observations": {"probe_started": False},
        }
        with tempfile.TemporaryDirectory(prefix="verify_skipped_") as temporary:
            fixture_root = Path(temporary)
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
        self.assertEqual("UNEXECUTED", report["status"])

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

        self.assertEqual(3, completed.returncode)
        self.assertEqual("NOT_RUN", report["status"])
        self.assertEqual(3, report["executed_test_count"])
        self.assertEqual(4, len(report["cases"]))

    def test_r02_keeps_offline_cases_out_of_live_action_gate(self):
        from dummy_test_support.adapters import r02

        result = {
            "case_id": "MOD05", "executed": True, "status": "PASS", "reason": "",
            "sut_source_sha256": "a" * 64, "trace_sha256": "b" * 64, "observations": {},
        }
        with mock.patch.object(r02, "run_unittest_case", return_value=result) as run_unittest:
            r02.run_r02_case("MOD05", {"case_id": "MOD05", "hardware": "none", "test_only": True})

        self.assertNotIn("runtime_action", run_unittest.call_args.kwargs)
        self.assertNotIn(".launch.", r02.TARGETS["ROS01"])

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

    def test_action_probe_successfully_reads_real_json_observations(self):
        from dummy_test_support.task_adapters import run_action_probe_case

        result = run_action_probe_case(
            "LIVE",
            {"case_id": "LIVE", "hardware": "none", "test_only": True},
            "scripts/dummy/tests/fixtures/success_probe.py",
            ("scripts/dummy/tests/fixtures/success_probe.py",),
        )

        self.assertEqual("PASS", result["status"])
        self.assertTrue(result["executed"])
        self.assertEqual({"observed": True}, result["observations"]["probe_observations"])


if __name__ == "__main__":
    unittest.main()
