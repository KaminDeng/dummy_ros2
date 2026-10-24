#!/usr/bin/env python3
"""Build and execute the ROS-owned R03 codec and goal-gate probes."""
##
# @file test_r03_acceptance.py
# @brief 当前codec编译执行、跨进程RPC及插件加载验收，保留R03套件入口。
#
# 这些方法不是纯静态：compile_and_run调用g++和组件；RPC方法运行r03_rpc_probe及析构probe，插件方法运行pluginlib loader。真实dp_comm指进程/传输实现，不是实体USB/电机验证。
import os

from pathlib import Path
import subprocess
import tempfile
import json
import unittest


ROOT = Path(__file__).resolve().parents[3]
DPSDK = Path(os.environ.get("DPSDK_ROOT", str(Path(__file__).resolve().parents[3].parent / "dpsdk_consolidated")))
C03_INCLUDE = ROOT / "contracts/dummy_robot_v2/codec"


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R03AcceptanceTest(unittest.TestCase):
    ##
    # @brief 临时g++17严格警告编译所给单源，含ROS控制/合同/SDK include，再要求二进制退出0；未设子进程timeout。
    # @param source 实际C++源码Path，调用会编译并运行二进制。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def compile_and_run(self, source: Path) -> None:
        self.assertTrue(source.is_file(), f"missing R03 SUT: {source}")
        with tempfile.TemporaryDirectory(prefix="dummy_r03_") as temporary:
            binary = Path(temporary) / source.stem
            completed = subprocess.run(
                [
                    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "src/dummy_ros_control/include"),
                    "-I", str(C03_INCLUDE),
                    "-I", str(DPSDK / "libraries/dp_comm/include"),
                    str(source), "-o", str(binary),
                ],
                cwd=ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(0, completed.returncode, completed.stderr)
            executed = subprocess.run([str(binary)], check=False, capture_output=True, text=True)
            self.assertEqual(0, executed.returncode, executed.stderr)

    ##
    # @brief 实际编译运行wire_probe_tests，覆盖范围由该C++断言决定，不能按case名额外扩展。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_gen01_gen03_and_rpc_payloads_use_actual_generated_codecs(self):
        self.compile_and_run(ROOT / "src/dummy_ros_control/test/wire_probe_tests.cpp")

    ##
    # @brief 复用wire_probe_tests实际运行，不是独立新RPC会话实验。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_rpc12_preserves_caller_owner_and_session_snapshot(self):
        self.compile_and_run(ROOT / "src/dummy_ros_control/test/wire_probe_tests.cpp")

    ##
    # @brief 实际编译运行goal_gate_tests核组件目标登记/摘要门，不发机械目标。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_mov14_unregistered_goal_and_wrong_digest_are_rejected(self):
        self.compile_and_run(ROOT / "src/dummy_ros_control/test/goal_gate_tests.cpp")

    ##
    # @brief 运行35s RPC orchestrator、核Queued非Accepted/大ID/三句柄用例，再执行已建析构probe。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_rpc01_and_rpc08_use_real_dp_comm_transport_and_handles(self):
        probe = ROOT / "scripts/dummy/r03_rpc_probe.py"
        self.assertTrue(probe.is_file(), "R03 RPC orchestrator is missing")
        with tempfile.TemporaryDirectory(prefix="dummy_r03_rpc_") as temporary:
            report = Path(temporary) / "report.json"
            completed = subprocess.run(
                ["/usr/bin/python3", str(probe), "--report", str(report)],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=35,
            )
            self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)
            observed = json.loads(report.read_text())
            self.assertTrue(observed["interprocess_service_passed"])
            self.assertTrue(observed["queued_not_accepted"])
            self.assertTrue(observed["large_uint64_preserved"])
            self.assertEqual(3, observed["request_handle_tests_passed"])
        destructor_probe = ROOT / "build_dummy/dummy_ros_control/request_handle_destructor_tests"
        self.assertTrue(destructor_probe.is_file(), "RequestHandle destructor probe was not built")
        completed = subprocess.run(
            [str(destructor_probe)], cwd=ROOT, capture_output=True, text=True, check=False
        )
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)

    ##
    # @brief 先核源码关闭基类输入与导出文本，再以显式ament/LD路径运行已建plugin loader；文本不等于运行期全图无旁路。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_guarded_jtc_plugin_closes_base_action_and_topic_inputs(self):
        header = ROOT / "src/dummy_ros_control/include/dummy_ros_control/guarded_joint_trajectory_controller.hpp"
        source = ROOT / "src/dummy_ros_control/src/guarded_joint_trajectory_controller.cpp"
        plugins = ROOT / "src/dummy_ros_control/dummy_ros_control_plugins.xml"
        self.assertTrue(header.is_file())
        self.assertTrue(source.is_file())
        self.assertTrue(plugins.is_file())
        implementation = source.read_text()
        self.assertIn("joint_command_subscriber_.reset()", implementation)
        self.assertIn("action_server_.reset()", implementation)
        self.assertIn("add_new_trajectory_msg", implementation)
        self.assertIn("GuardedJointTrajectoryController", plugins.read_text())
        loader = ROOT / "build_dummy/dummy_ros_control/guarded_plugin_load_tests"
        self.assertTrue(loader.is_file(), "pluginlib load probe was not built")
        environment = dict(__import__("os").environ)
        prefix = ROOT / "install_dummy/dummy_ros_control"
        environment["AMENT_PREFIX_PATH"] = f"{prefix}:/opt/ros/jazzy"
        environment["LD_LIBRARY_PATH"] = f"{prefix}/lib:/opt/ros/jazzy/lib"
        completed = subprocess.run(
            [str(loader)], cwd=ROOT, env=environment,
            capture_output=True, text=True, check=False
        )
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)


if __name__ == "__main__":
    unittest.main()
