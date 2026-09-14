"""Verify strict non-R00 suite registration and exit behavior."""

import json
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
VERIFY = ROOT / "scripts/dummy/verify.py"


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
        self.assertGreaterEqual(report["executed_test_count"], 8)

    def test_r03_runs_registered_wire_rpc_suite(self):
        code, report = self.run_suite("R03")
        self.assertEqual(0, code)
        self.assertEqual("PASS", report["status"])
        self.assertEqual(5, report["executed_test_count"])

    def test_r05_without_runtime_is_not_run(self):
        code, report = self.run_suite("R05")
        self.assertEqual(3, code)
        self.assertEqual("NOT_RUN", report["status"])

    def test_r04_runs_nonblocking_plugin_suite(self):
        code, report = self.run_suite("R04")
        self.assertEqual(0, code)
        self.assertEqual("PASS", report["status"])
        self.assertEqual(4, report["executed_test_count"])


if __name__ == "__main__":
    unittest.main()
