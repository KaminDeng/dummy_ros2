#!/usr/bin/env python3
"""Acceptance checks for the R00 environment and safety runner."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
VERIFY = WORKSPACE_ROOT / "scripts" / "dummy" / "verify.py"
BUILD = WORKSPACE_ROOT / "scripts" / "dummy" / "build.sh"
ENVIRONMENT_REPORT = WORKSPACE_ROOT / "scripts" / "dummy" / "environment_report.py"
BRINGUP_PACKAGE = WORKSPACE_ROOT / "src" / "dummy_bringup" / "package.xml"


class R00AcceptanceTest(unittest.TestCase):
    def test_build_script_passes_log_base_as_a_colcon_global_option(self):
        build_source = BUILD.read_text()

        self.assertIn(
            "colcon --log-base log_dummy build --build-base build_dummy",
            build_source,
        )
        self.assertIn("-DPython3_EXECUTABLE=/usr/bin/python3", build_source)

    def test_build_script_preserves_explicit_dpsdk_root(self):
        build_source = BUILD.read_text()
        self.assertNotIn("../dpsdk", build_source)
        self.assertIn('DPSDK_ROOT="${DPSDK_ROOT:-', build_source)
        self.assertIn("export DPSDK_ROOT", build_source)

    def test_environment_report_uses_explicit_source_roots(self):
        with tempfile.TemporaryDirectory(prefix="r00_roots_") as temporary:
            root = Path(temporary)
            dpsdk = root / "dpsdk"
            dummy = root / "dummy"
            (dpsdk / "tools/ros2").mkdir(parents=True)
            (dpsdk / "tools/ros2/zenbedded.repos").write_text("repositories: {}\n")
            (dummy / "Robot/instances").mkdir(parents=True)
            (dummy / "Robot/instances/dummy_robot.cpp").write_text("// sentinel\n")
            output = root / "environment.json"
            environment = dict(os.environ, DPSDK_ROOT=str(dpsdk), DUMMY_ROOT=str(dummy))
            completed = subprocess.run(
                ["/usr/bin/python3", str(ENVIRONMENT_REPORT), "--output", str(output)],
                cwd=WORKSPACE_ROOT, env=environment, capture_output=True, text=True, check=False,
            )
            self.assertEqual(0, completed.returncode, completed.stderr)
            report = json.loads(output.read_text())
            self.assertEqual(str(dpsdk), report["dpsdk_root"])
            self.assertEqual(str(dummy), report["dummy_root"])
            self.assertTrue(report["roots"]["dpsdk_sentinel"])
            self.assertTrue(report["roots"]["dummy_sentinel"])

    def test_bringup_package_declares_ament_cmake_build_type(self):
        package_xml = BRINGUP_PACKAGE.read_text()

        self.assertIn("<build_type>ament_cmake</build_type>", package_xml)

    def run_case(self, case_id: str | None) -> tuple[int, dict[str, object]]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            report_path = Path(temporary_directory) / "report.json"
            command = [
                "python3",
                str(VERIFY),
                "--suite",
                "R00",
                "--hardware",
                "none",
                "--report",
                str(report_path),
            ]
            if case_id is not None:
                command.extend(("--case", case_id))
            completed = subprocess.run(
                command,
                cwd=WORKSPACE_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertTrue(
                report_path.exists(),
                f"runner did not write report: {completed.stderr}",
            )
            self.assertTrue(report_path.with_suffix(".junit.xml").exists())
            self.assertTrue(report_path.with_suffix(".log").exists())
            return completed.returncode, json.loads(report_path.read_text())

    def test_aud01_rejects_a_dpsdk_subdirectory_as_the_root(self):
        return_code, report = self.run_case("AUD01")

        self.assertEqual(return_code, 0)
        result = report["cases"][0]
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["observations"]["invalid_dpsdk_root_rejected"])
        self.assertTrue(result["observations"]["source_hashes_unchanged"])

    def test_aud02_reports_failed_empty_and_all_skipped_test_runs_as_non_pass(self):
        return_code, report = self.run_case("AUD02")

        self.assertEqual(return_code, 0)
        result = report["cases"][0]
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["observations"]["failed_test_exit"], 1)
        self.assertEqual(result["observations"]["empty_test_exit"], 4)
        self.assertEqual(result["observations"]["all_skipped_test_exit"], 4)

    def test_aud03_denies_all_hardware_probes_before_opening_a_device(self):
        return_code, report = self.run_case("AUD03")

        self.assertEqual(return_code, 0)
        result = report["cases"][0]
        self.assertEqual(result["status"], "PASS")
        observations = result["observations"]
        self.assertEqual(observations["real_serial_open_count"], 0)
        self.assertEqual(observations["socketcan_open_count"], 0)
        self.assertEqual(observations["jlink_spawn_count"], 0)
        self.assertTrue(observations["denial_observed"])
        self.assertEqual(
            observations["denied_probes"],
            ["ttyACM0", "socketcan", "jlink"],
        )

    def test_ros01_requires_r02_runtime_evidence_after_a_clean_build(self):
        return_code, report = self.run_case(None)

        self.assertEqual(return_code, 3)
        result = next(case for case in report["cases"] if case["case_id"] == "ROS01")
        self.assertEqual(result["status"], "NOT_RUN")
        self.assertTrue(result["observations"]["r02_runtime_required"])


if __name__ == "__main__":
    unittest.main()
