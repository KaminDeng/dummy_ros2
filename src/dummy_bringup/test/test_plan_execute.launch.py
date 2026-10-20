"""Launch-test MoveIt planning and synchronized scene behavior."""
##
# @file test_plan_execute.launch.py
# @brief Gazebo环境中实际MoveIt规划与场景一致性的launch测试。
#
# 启动描述包含gazebo_6dof.launch.py，正文运行planning_probe.py并给90s期限。探针只规划/查询，明确要求trajectory_executed=false；不能把文件名plan_execute当作已执行轨迹或实体动作。

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
# @brief 从安装share包含Gazebo六轴launch与ReadyToTest；不在此函数内等待所有服务就绪。
# @return 包含实际Gazebo launch动作的LaunchDescription。
def generate_test_description():
    launch_file = Path(get_package_share_directory("dummy_bringup")) / "launch/gazebo_6dof.launch.py"
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
        launch_testing.actions.ReadyToTest(),
    ])


##
# @brief Gazebo场景中调用真实规划probe、要求不执行轨迹的测试集合。
class PlanExecuteRuntime(unittest.TestCase):
    ##
    # @brief 实际探针核有效关节计划、不可达笛卡尔fraction<1、障碍状态/目标拒绝、canonical场景及revision匹配，并明确未执行轨迹。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
    def test_planning_probe_observes_collision_and_partial_cartesian_path(self):
        probe = Path(get_package_prefix("dummy_bringup")) / "lib/dummy_bringup/planning_probe.py"
        with tempfile.TemporaryDirectory(prefix="r05_launch_") as temporary:
            report = Path(temporary) / "planning.json"
            result = subprocess.run(
                ["/usr/bin/python3", str(probe), "--report", str(report)],
                capture_output=True, text=True, check=False, timeout=90,
            )
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertTrue(report.is_file())
            observations = json.loads(report.read_text())
            self.assertEqual(1, observations["valid_joint_plan_error_code"])
            self.assertGreater(observations["valid_joint_trajectory_points"], 1)
            self.assertLess(observations["unreachable_cartesian_fraction"], 1.0)
            self.assertTrue(observations["barrier_collision_state_found"])
            self.assertNotEqual(1, observations["barrier_collision_goal_error_code"])
            self.assertTrue(observations["planning_scene_matches_canonical"])
            self.assertTrue(observations["world_scene_revision_matches"])
            self.assertFalse(observations["trajectory_executed"])
