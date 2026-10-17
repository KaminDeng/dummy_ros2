"""使用真实子进程退出状态验证软件夹具的失败传播与刺激。"""
import subprocess
import sys
import unittest

BINARY = sys.argv.pop(1)


class FixtureIntegrityTests(unittest.TestCase):
    def invoke(self, *arguments):
        return subprocess.run([BINARY, *arguments], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, timeout=10)

    def test_real_process_exit_preserves_failure(self):
        for result in (0, 1, 256, 512, -1):
            with self.subTest(result=result):
                child = self.invoke("fixture_result", str(result))
                self.assertEqual(child.returncode, 0 if result == 0 else 1, child.stdout)
                if result:
                    self.assertIn(f"FIXTURE_FAILURE result={result}", child.stdout)

    def test_unknown_safe_prime_selectors_are_rejected(self):
        for mode in ("cancel_coordination_typo", "safe_prime_typo",
                     "safe_prime_guard_typo", "safe_prime_guard_wrong_query_op"):
            with self.subTest(mode=mode):
                child = self.invoke("six_axis", mode)
                self.assertNotEqual(child.returncode, 0, child.stdout)
                self.assertIn("UNKNOWN_SOURCE_FIXTURE_MODE", child.stdout)

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
