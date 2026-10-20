"""R08 release evidence acceptance."""
##
# @file test_r08_acceptance.py
# @brief REL01静态发布证据布尔结论的适配器验收。
#
# 运行R08会读取已有报告/锁/Git/mtime并写trace，不重新运行设备或R06/R07。要求返回字段为真不能扩大为实机功能/机械资格或可复现构建证明。

from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R08Acceptance(unittest.TestCase):
    ##
    # @brief 调用REL01并核executed/摘要与九项发布结论，未直接断言status字段，但requirement_observed必须true。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_rel01(self):
        observed = run_case("R08", "REL01", {
            "case_id": "REL01", "hardware": "none", "test_only": True,
        })
        self.assertIs(observed["executed"], True)
        self.assertTrue(observed["sut_source_sha256"])
        self.assertTrue(observed["trace_sha256"])
        values = observed["observations"]
        self.assertTrue(values["source_lock_present"])
        self.assertFalse(values["runtime_not_run_as_pass"])
        self.assertTrue(values["ros_gazebo_record_separate"])
        self.assertTrue(values["wire_lock_matches_bundle"])
        self.assertTrue(values["dpsdk_commit_matches"])
        self.assertTrue(values["dpsdk_contract_clean"])
        self.assertTrue(values["dpsdk_peer_current"])
        self.assertTrue(values["runtime_reports_current"])
        self.assertTrue(values["requirement_observed"])


if __name__ == "__main__":
    unittest.main()
