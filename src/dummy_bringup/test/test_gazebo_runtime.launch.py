"""Launch-test the canonical Gazebo action path."""
##
# @file test_gazebo_runtime.launch.py
# @brief Gazebo公开Action执行/取消/恢复的实际launch集成测试。
#
# 生成器包含gazebo_6dof.launch.py，运行时会创建ROS/Gazebo链路；正文另启动action_gate_probe.py并给60s期限。通过的是仿真反馈与公开Action合同，不是USB或实体机械臂资格。

from pathlib import Path
import json
import subprocess
import tempfile
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing


##
# @brief 按安装share路径包含完整Gazebo六轴launch并发布测试就绪标志。
# @return 包含实际Gazebo启动描述与ReadyToTest的LaunchDescription；ReadyToTest本身不保证所有服务已ready。
def generate_test_description():
    launch_file = Path(get_package_share_directory("dummy_bringup")) / "launch/gazebo_6dof.launch.py"
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
        launch_testing.actions.ReadyToTest(),
    ])


##
# @brief Gazebo运行期间调用真实Action probe并检查报告的测试集合。
class GazeboRuntime(unittest.TestCase):
    ##
    # @brief 运行probe写临时JSON，核首目标取消状态5、忙时第二目标拒绝、第三目标成功状态4、越界拒绝及J1变化超过0.02rad。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
    def test_public_action_moves_and_recovers_after_cancel(self):
        probe = Path(get_package_prefix("dummy_bringup")) / "lib/dummy_bringup/action_gate_probe.py"
        with tempfile.TemporaryDirectory(prefix="r02_launch_") as temporary:
            report = Path(temporary) / "action.json"
            result = subprocess.run(
                ["/usr/bin/python3", str(probe), "--report", str(report)],
                capture_output=True, text=True, check=False, timeout=60,
            )
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertTrue(report.is_file())
            observations = json.loads(report.read_text())
            self.assertTrue(observations["first_goal_accepted"])
            self.assertFalse(observations["second_goal_accepted"])
            self.assertEqual(5, observations["first_goal_terminal_status"])
            self.assertTrue(observations["third_goal_accepted"])
            self.assertEqual(4, observations["third_goal_terminal_status"])
            self.assertFalse(observations["out_of_bounds_goal_accepted"])
            self.assertGreater(observations["joint_state_sample_count"], 2)
            self.assertGreater(observations["joint1_position_change_rad"], 0.02)
