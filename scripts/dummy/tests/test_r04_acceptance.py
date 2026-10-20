"""Compile and execute R04 nonblocking producer/state acceptance tests."""
##
# @file test_r04_acceptance.py
# @brief R04生产者/状态组件编译运行及插件接口文本回归。
#
# compile_run真实g++/执行，其他方法也运行build_dummy既有测试；不启动实体硬件。只检退出0与指定文本，不提供完整实时调度/WCET或机械运动证据。

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R04AcceptanceTest(unittest.TestCase):
    ##
    # @brief 严格g++17/pthread编译单个控制测试源到临时目录并运行；两个子进程均未设timeout。
    # @param source_name src/dummy_ros_control/test下待编译C++文件名。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def compile_run(self, source_name: str) -> None:
        source = ROOT / "src/dummy_ros_control/test" / source_name
        self.assertTrue(source.is_file(), f"missing R04 test SUT: {source}")
        with tempfile.TemporaryDirectory(prefix="r04_") as temporary:
            binary = Path(temporary) / source.stem
            completed = subprocess.run(
                ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
                 "-I", str(ROOT / "src/dummy_ros_control/include"), str(source), "-o", str(binary)],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            self.assertEqual(0, completed.returncode, completed.stderr)
            executed = subprocess.run([str(binary)], capture_output=True, text=True, check=False)
            self.assertEqual(0, executed.returncode, executed.stdout + executed.stderr)

    ##
    # @brief 编译producer_tests后再运行已建command_channel_tests，核两组件退出0。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_fre04_fre05_and_fre06_generation_identity(self):
        self.compile_run("producer_tests.cpp")
        binary = ROOT / "build_dummy/dummy_ros_control/command_channel_tests"
        self.assertTrue(binary.is_file(), "shared command channel test was not built")
        completed = subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False)
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)

    ##
    # @brief 编译运行stale_state_tests，共享同一C++测试覆盖这些映射case。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_rpc09_fre09_fre10_and_ros07_state_validation(self):
        self.compile_run("stale_state_tests.cpp")

    ##
    # @brief 运行已建nonblocking_tests和dp_comm_worker_tests；具体时限/背压断言由其实现给出。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_ros09_bounded_publisher_backpressure(self):
        for name in ("nonblocking_tests", "dp_comm_worker_tests"):
            binary = ROOT / "build_dummy/dummy_ros_control" / name
            self.assertTrue(binary.is_file(), f"{name} was not built")
            completed = subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False)
            self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)

    ##
    # @brief 源码/插件文本检查System接口与禁词/write边界，再跑wire_state_ingest；禁词检查可能受注释文本影响，原oracle保持。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_guarded_system_is_exported_without_implicit_authorization(self):
        header = ROOT / "src/dummy_ros_control/include/dummy_ros_control/guarded_zenbedded_system.hpp"
        source = ROOT / "src/dummy_ros_control/src/guarded_zenbedded_system.cpp"
        plugins = ROOT / "src/dummy_ros_control/dummy_ros_control_plugins.xml"
        self.assertTrue(header.is_file())
        self.assertTrue(source.is_file())
        implementation = source.read_text()
        self.assertIn("hardware_interface::SystemInterface", header.read_text())
        self.assertIn("GuardedZenbeddedSystem", plugins.read_text())
        for forbidden in ("Grant", "Capture", "EnableHold", "async_request", "spin_some"):
            self.assertNotIn(forbidden, implementation)
        controller = (
            ROOT / "src/dummy_ros_control/src/guarded_joint_trajectory_controller.cpp"
        ).read_text()
        self.assertIn("JointTrajectoryController::update", controller)
        self.assertIn("update_sink_", controller)
        write_body = implementation.split("GuardedZenbeddedSystem::write", 1)[1].split(
            "GuardedZenbeddedSystem::ingest_state", 1
        )[0]
        self.assertNotIn("record_update", write_body)
        binary = ROOT / "build_dummy/dummy_ros_control/wire_state_ingest_tests"
        self.assertTrue(binary.is_file(), "wire-state ingest test was not built")
        completed = subprocess.run([str(binary)], cwd=ROOT, capture_output=True, text=True, check=False)
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)


if __name__ == "__main__":
    unittest.main()
