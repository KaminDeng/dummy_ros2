"""使用真实子进程退出状态验证软件夹具的失败传播与刺激。"""
##
# @file safe_prime_fixture_integrity_tests.py
# @brief 软件safe-prime夹具的实际退出传播及负例刺激完整性。
#
# 导入时从sys.argv移除被测二进制路径，测试实际执行该程序，不接真实USB。退出码归一、未知selector拒绝及generation/transferred/rejected_delta日志证明刺激到达组件，不等于实体安全停止或新SDK资格。
import subprocess
import sys
import unittest

BINARY = sys.argv.pop(1)


##
# @brief 真实组件子进程退出码与刺激检查集合。
class FixtureIntegrityTests(unittest.TestCase):
    ##
    # @brief 运行传入BINARY和参数，合并stdout/stderr、10s期限，不复制生产判定。
    # @param arguments 传给被测组件的额外argv，不经shell执行。
    # @return 真实CompletedProcess，失败码由外层断言解释。
    def invoke(self, *arguments):
        return subprocess.run([BINARY, *arguments], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, timeout=10)

    ##
    # @brief fixture_result输入0/1/256/512/−1，只有0子进程成功，其它归1且保原失败值文本。
    # @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
    def test_real_process_exit_preserves_failure(self):
        for result in (0, 1, 256, 512, -1):
            with self.subTest(result=result):
                child = self.invoke("fixture_result", str(result))
                self.assertEqual(child.returncode, 0 if result == 0 else 1, child.stdout)
                if result:
                    self.assertIn(f"FIXTURE_FAILURE result={result}", child.stdout)

    ##
    # @brief 四个未知/错拼selector须非零和UNKNOWN_SOURCE_FIXTURE_MODE，不静默落默认正例。
    # @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
    def test_unknown_safe_prime_selectors_are_rejected(self):
        for mode in ("cancel_coordination_typo", "safe_prime_typo",
                     "safe_prime_guard_typo", "safe_prime_guard_wrong_query_op"):
            with self.subTest(mode=mode):
                child = self.invoke("six_axis", mode)
                self.assertNotEqual(child.returncode, 0, child.stdout)
                self.assertIn("UNKNOWN_SOURCE_FIXTURE_MODE", child.stdout)

    ##
    # @brief 七个guard负例要求非零generation/实际transfer/reject增量和COMMAND_PRIME_DIAG，避免无刺激就通过。
    # @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
    def test_negative_guards_receive_real_intent(self):
        for mode in ("wrong_boot", "wrong_session", "wrong_ref", "old_motion",
                     "arbitrary_target", "foreign_run", "wrong_status_phase"):
            with self.subTest(mode=mode):
                child = self.invoke("six_axis", "safe_prime_guard_" + mode)
                self.assertEqual(child.returncode, 0, child.stdout)
                self.assertRegex(child.stdout,
                                 r"SAFE_PRIME_GUARD_STIMULUS generation=[1-9][0-9]* transferred=1 rejected_delta=[1-9][0-9]*")
                self.assertIn("COMMAND_PRIME_DIAG reject", child.stdout)


if __name__ == "__main__":
    unittest.main()
