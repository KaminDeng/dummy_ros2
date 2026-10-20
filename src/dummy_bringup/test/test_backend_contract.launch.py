"""Launch-level contract checks for the two device SIL entries and real J1 gate."""
##
# @file test_backend_contract.launch.py
# @brief SIL/real J1入口配置与缺参阻断的launch级合同测试。
#
# LaunchDescription只起一个常驻shell供launch_testing就绪；测试正文还会运行ros2 launch --show-args，以及缺device_id的real_j1入口。源码字符串检查不能证明桥已连接设备或电机已获授权；缺参用例期望在启动实际桥前失败。

from pathlib import Path
import subprocess
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import yaml


##
# @brief 装配常驻shell与ReadyToTest，保持launch测试上下文存活。
# @return LaunchDescription，包含实际ExecuteProcess shell及ReadyToTest动作；不在此函数内启动。
def generate_test_description():
    return LaunchDescription([
        ExecuteProcess(cmd=["/bin/sh", "-c", "while :; do sleep 1; done"]),
        launch_testing.actions.ReadyToTest(),
    ])


##
# @brief 配置文件/源码与CLI边界的unittest集合，不将文本检查当设备运行。
class BackendContractLaunchTest(unittest.TestCase):
    ##
    # @brief 读backends.yaml核三项自动授权关闭、J1轴集合及SIL入口peer/Action字符串，未执行这些SIL入口。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
    def test_backend_profiles_are_explicit_and_motion_is_not_automatic(self):
        package_root = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((package_root / "config/backends.yaml").read_text())
        self.assertEqual(False, config["security"]["auto_grant"])
        self.assertEqual(False, config["security"]["auto_capture_reference"])
        self.assertEqual(False, config["security"]["auto_enable"])
        self.assertEqual(["Joint1"], config["backends"]["sil_j1"]["active_axes"])
        self.assertEqual(["Joint1"], config["backends"]["real_j1"]["active_axes"])
        for name in ("sil_6dof", "sil_j1"):
            entry = config["backends"][name]
            for key in ("device_id", "serial", "profile", "endpoint"):
                self.assertTrue(entry[key])
            source = (package_root / "launch" / f"{name}.launch.py").read_text()
            self.assertIn("dummy_ref_d08_device_peer", source)
            self.assertIn("device_action_gate", source)
            self.assertNotIn("ros_device_executable", source)
            self.assertNotIn("/enable", source)

    ##
    # @brief 静态核real_j1含只读执行门与BLOCKED标记，不含Grant/Capture/Enable文本；不证明运行期权限。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
    def test_real_j1_has_an_executable_read_only_gate(self):
        source = (Path(__file__).resolve().parents[1] / "launch/real_j1.launch.py").read_text()
        self.assertIn("read_only_device_executable", source)
        self.assertIn("ExecuteProcess", source)
        self.assertIn("BLOCKED real_j1", source)
        self.assertNotIn("Grant", source)
        self.assertNotIn("Capture", source)
        self.assertNotIn("Enable", source)

    ##
    # @brief 分别以10s子进程期限运行三入口--show-args，并核源码profile/mask和real只读身份参数；没有断言桥实际运动。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
    def test_launch_expands_identity_and_mode_into_bridge_command(self):
        package_root = Path(__file__).resolve().parents[1]
        for name in ("sil_6dof", "sil_j1", "real_j1"):
            launch_file = package_root / "launch" / f"{name}.launch.py"
            completed = subprocess.run(
                ["ros2", "launch", str(launch_file), "--show-args"],
                capture_output=True, text=True, check=False, timeout=10,
            )
            output = completed.stdout + completed.stderr
            self.assertEqual(0, completed.returncode, output)
            self.assertIn("endpoint", output)
            source = launch_file.read_text()
            if name == "real_j1":
                for value in ("device_id", "serial", "profile", "--read-only"):
                    self.assertIn(value, source)
            else:
                profile_id, mask = (("1", "63") if name == "sil_6dof" else ("2", "1"))
                self.assertIn(f'"--profile-id", "{profile_id}"', source)
                self.assertIn(f'"--configured-mask", "{mask}"', source)

    ##
    # @brief 缺device_id实际调用real_j1 launch，要求非零退出并点明必填参数，避免把未配置入口静默运行。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
    def test_missing_bridge_arguments_fail_closed(self):
        package_root = Path(__file__).resolve().parents[1]
        cases = (
            ("real_j1", (), "device_id"),
        )
        for name, arguments, required in cases:
            completed = subprocess.run(
                ["ros2", "launch", str(package_root / "launch" / f"{name}.launch.py"), *arguments],
                capture_output=True, text=True, check=False, timeout=10,
            )
            output = completed.stdout + completed.stderr
            self.assertNotEqual(0, completed.returncode, output)
            self.assertIn(f"required argument '{required}'", output, output)
