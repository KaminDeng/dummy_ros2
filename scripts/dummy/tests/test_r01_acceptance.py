"""R01 adapter evidence is tied to the real model and FK probes."""
##
# @file test_r01_acceptance.py
# @brief R01适配器路由与证据状态的验收断言。
#
# evidence会实际通过task_adapters启动模型/FK目标并写trace；输入来自现有模型/离线样本，不产生新机械测量。最后用例保留旧MOD04/06 NOT_RUN期望，但当前r01 TARGETS已有对应探针；该不一致不在注释中改oracle。

from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R01Acceptance(unittest.TestCase):
    ##
    # @brief 调用R01真实adapter并物化trace，传严格hardware none/test_only stimulus。
    # @param case_id 当前case标识；支持集合由本suite映射/分支决定。
    # @return R01结果及已物化trace字段。
    def evidence(self, case_id: str) -> dict:
        return run_case("R01", case_id, {
            "case_id": case_id, "hardware": "none", "test_only": True,
        })

    ##
    # @brief 要求PASS且case_target指J1 bench anchor roundtrip方法，不据此断言现场已标定。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_mod01_uses_the_joint1_anchor_round_trip_probe(self):
        result = self.evidence("MOD01")
        self.assertEqual("PASS", result["status"], result["reason"])
        self.assertIn("joint1_bench_mapping", result["observations"]["case_target"])

    ##
    # @brief 要求PASS且目标指固定/随机KDL离线FK对照。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_mod02_uses_reference_chain_fk_samples(self):
        result = self.evidence("MOD02")
        self.assertEqual("PASS", result["status"], result["reason"])
        self.assertIn("fixed_and_random_evidence_match_kdl", result["observations"]["case_target"])

    ##
    # @brief 要求PASS且目标为旧J3数学行与实际CAD FK差异用例。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_mod03_uses_the_actual_legacy_chain_divergence_probe(self):
        result = self.evidence("MOD03")
        self.assertEqual("PASS", result["status"], result["reason"])
        self.assertIn("literal_legacy_axis3_row_diverges", result["observations"]["case_target"])

    ##
    # @brief 保留原NOT_RUN/executed=False断言；当前adapter已映射这两目标，不能把本旧期望写成其现行行为保证。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_mod04_and_mod06_are_not_claimed_without_their_specific_probe(self):
        for case_id in ("MOD04", "MOD06"):
            result = self.evidence(case_id)
            self.assertEqual("NOT_RUN", result["status"])
            self.assertFalse(result["executed"])


if __name__ == "__main__":
    unittest.main()
