"""Runtime acceptance for the actual MoveIt planning services."""
##
# @file test_r05_acceptance.py
# @brief 真实公开Action/MoveIt探针与纯目标范围函数的混合验收。
#
# Action方法会向已有公开server发目标/取消；规划方法会查询/规划但显式不执行轨迹；范围方法仅调用纯函数。脚本不在此启动Gazebo或设备，所连接运行图的后端需外部明确，不能直接称实体机械臂已验证。

import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
PROBE = ROOT / "src/dummy_bringup/scripts/planning_probe.py"
ACTION_PROBE = ROOT / "src/dummy_bringup/scripts/action_gate_probe.py"


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R05PlanningAcceptance(unittest.TestCase):
    ##
    # @brief 以45s期限运行Action probe，核首目标取消5、忙拒绝、第三目标成功4及越界拒绝。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_public_action_gate_rejects_busy_then_allows_goal_after_cancel(self):
        self.assertTrue(ACTION_PROBE.is_file(), "R05 action gate probe is missing")
        with tempfile.TemporaryDirectory(prefix="r05_action_") as temporary:
            report = Path(temporary) / "observations.json"
            result = subprocess.run(
                ["/usr/bin/python3", str(ACTION_PROBE), "--report", str(report)],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=45,
            )
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            observed = json.loads(report.read_text())
            self.assertTrue(observed["first_goal_accepted"])
            self.assertFalse(observed["second_goal_accepted"])
            self.assertEqual(5, observed["first_goal_terminal_status"])
            self.assertTrue(observed["third_goal_accepted"])
            self.assertEqual(4, observed["third_goal_terminal_status"])
            self.assertFalse(observed["out_of_bounds_goal_accepted"])

    ##
    # @brief 仅调用validate_joint_target核J6 3π允许/5π拒绝、J3零和NaN拒绝，不发送服务。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_contract_bounds_preserve_joint6_multiturn_and_reject_joint3_zero(self):
        self.assertTrue(PROBE.is_file(), "R05 MoveIt service probe is missing")
        sys.path.insert(0, str(PROBE.parent))
        from planning_probe import validate_joint_target

        self.assertTrue(validate_joint_target((0, 0, math.pi / 2, 0, 0, 3 * math.pi)))
        self.assertFalse(validate_joint_target((0, 0, math.pi / 2, 0, 0, 5 * math.pi)))
        self.assertFalse(validate_joint_target((0, 0, 0, 0, 0, 0)))
        self.assertFalse(validate_joint_target((math.nan, 0, math.pi / 2, 0, 0, 0)))

    ##
    # @brief 实际45s规划probe核有效计划/IK、越界不发服务、部分笛卡尔和障碍拒绝，且trajectory_executed=false。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_real_moveit_plans_and_rejects_invalid_paths(self):
        self.assertTrue(PROBE.is_file(), "R05 MoveIt service probe is missing")
        with tempfile.TemporaryDirectory(prefix="r05_planning_") as temporary:
            report = Path(temporary) / "observations.json"
            result = subprocess.run(
                ["/usr/bin/python3", str(PROBE), "--report", str(report)],
                cwd=ROOT, capture_output=True, text=True, check=False, timeout=45,
            )
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertTrue(report.is_file(), "probe produced no runtime evidence")
            observed = json.loads(report.read_text())
            self.assertGreater(observed["valid_joint_trajectory_points"], 1)
            self.assertEqual(1, observed["valid_joint_plan_error_code"])
            self.assertTrue(observed["out_of_bounds_rejected_by_guard"])
            self.assertFalse(observed["out_of_bounds_service_sent"])
            self.assertLess(observed["unreachable_cartesian_fraction"], 1.0)
            self.assertFalse(observed["trajectory_executed"])
            self.assertTrue(observed["barrier_collision_state_found"])
            self.assertNotEqual(1, observed["barrier_collision_goal_error_code"])
            self.assertEqual(1, observed["pose_ik_error_code"])
            self.assertTrue(observed["pose_ik_solution_within_contract_limits"])


if __name__ == "__main__":
    unittest.main()
