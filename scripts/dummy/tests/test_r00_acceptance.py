#!/usr/bin/env python3
"""Acceptance checks for the R00 environment and safety runner."""
##
# @file test_r00_acceptance.py
# @brief R00构建脚本静态约束及真实环境/verify子进程验收。
#
# 部分用例只检查文本，环境报告和run_case会实际运行Python/verify、写临时报告及默认trace；ROS01全套case可查询Gazebo。AUD03零硬件计数来自自有guard，不是内核级硬件拦截证明。

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


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R00AcceptanceTest(unittest.TestCase):
    ##
    # @brief 静态核colcon --log-base位置及/usr/bin/python3配置，不执行构建。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_build_script_passes_log_base_as_a_colcon_global_option(self):
        build_source = BUILD.read_text()

        self.assertIn(
            "colcon --log-base log_dummy build --build-base build_dummy",
            build_source,
        )
        self.assertIn("-DPython3_EXECUTABLE=/usr/bin/python3", build_source)

    ##
    # @brief 静态禁个人绝对路径并核DPSDK_ROOT默认/导出语法，不实际source构建脚本。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_build_script_preserves_explicit_dpsdk_root(self):
        build_source = BUILD.read_text()
        self.assertNotIn("../dpsdk", build_source)
        self.assertIn('DPSDK_ROOT="${DPSDK_ROOT:-', build_source)
        self.assertIn("export DPSDK_ROOT", build_source)

    ##
    # @brief 建最小sentinel目录并运行environment_report，核显式根与版本字段；不把sentinel当完整SDK可构建。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
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
            for name in ("ros_distro", "moveit", "joint_trajectory_controller", "rmw", "gazebo"):
                self.assertIn(name, report["versions"])
                self.assertTrue(report["versions"][name])

    ##
    # @brief 文本核package.xml build_type，不执行ament。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_bringup_package_declares_ament_cmake_build_type(self):
        package_xml = BRINGUP_PACKAGE.read_text()

        self.assertIn("<build_type>ament_cmake</build_type>", package_xml)

    ##
    # @brief 临时运行verify R00/hardware none，可选case；要求JSON/JUnit/log三产物存在后返回状态与报告。
    # @param case_id 单个R00 case或None运行整套，后者可能访问ROS图。
    # @return (verify子进程退出码, 解析的JSON报告)。
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

    ##
    # @brief 实际verify AUD01要求合法根/子目录拒绝和所列源码摘要不变。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_aud01_rejects_a_dpsdk_subdirectory_as_the_root(self):
        return_code, report = self.run_case("AUD01")

        self.assertEqual(return_code, 0)
        result = report["cases"][0]
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["observations"]["invalid_dpsdk_root_rejected"])
        self.assertTrue(result["observations"]["source_hashes_unchanged"])

    ##
    # @brief 实际AUD02内部运行三个临时unittest，核失败1、空4、全skip4分类。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_aud02_reports_failed_empty_and_all_skipped_test_runs_as_non_pass(self):
        return_code, report = self.run_case("AUD02")

        self.assertEqual(return_code, 0)
        result = report["cases"][0]
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["observations"]["failed_test_exit"], 1)
        self.assertEqual(result["observations"]["empty_test_exit"], 4)
        self.assertEqual(result["observations"]["all_skipped_test_exit"], 4)

    ##
    # @brief 实际verify调用内存guard，断言三个拒绝名称和opened元组派生零计数；无真实设备打开尝试。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
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

    ##
    # @brief 本oracle明确假定运行时缺失，期望全R00退出3且ROS01 NOT_RUN；存在Gazebo时并非通用预期。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_ros01_requires_r02_runtime_evidence_after_a_clean_build(self):
        return_code, report = self.run_case(None)

        self.assertEqual(return_code, 3)
        result = next(case for case in report["cases"] if case["case_id"] == "ROS01")
        self.assertEqual(result["status"], "NOT_RUN")
        self.assertFalse(result["executed"])
        self.assertTrue(result["reason"])


if __name__ == "__main__":
    unittest.main()
