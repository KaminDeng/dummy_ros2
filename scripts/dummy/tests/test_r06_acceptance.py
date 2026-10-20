"""R06 device action lifecycle acceptance against ROS and the shared DPSDK core."""
##
# @file test_r06_acceptance.py
# @brief 按既有R06 fixture动态生成真实adapter调用的验收方法。
#
# fixture是期望值/用例名，读取它不等于设备验证；每生成方法仍调用run_case，可能真实运行组件/ROS/POSIX peer。非ROS06固定NOT_RUN的旧oracle与当前adapter可在真实DPSDK peer E2E成功后PASS的分支有条件冲突，保留原断言。

import json
from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


ROOT = Path(__file__).resolve().parents[3]
FIXTURE = json.loads((ROOT / "scripts/dummy/tests/fixtures/R06.json").read_text())
CASES = FIXTURE["cases"]


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R06Acceptance(unittest.TestCase):
    pass


##
# @brief 闭包捕获case_id并返回unittest方法，顶层setattr为fixture每case注册独立名字，不在注册时执行adapter。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @return 闭包test函数，尚未执行其adapter。
def _make_case(case_id: str):
    ##
    # @par 内部回调 test
    # 实际调用R06 adapter，核摘要/身份；非ROS06固定要求NOT_RUN后检查组件，ROS06可NOT_RUN或PASS并核反馈数量/位移。
    # 返回：None；通过assert/unittest断言报告结果，失败抛异常。
    def test(self):
        observed = run_case("R06", case_id, {
            "case_id": case_id,
            "hardware": "none",
            "test_only": True,
        })
        self.assertTrue(observed["sut_source_sha256"])
        self.assertTrue(observed["trace_sha256"])
        values = observed["observations"]
        self.assertEqual(case_id, values["case_id"])
        self.assertNotIn("case_passed", values)
        if case_id != "ROS06":
            self.assertIs(observed["executed"], False)
            self.assertEqual("NOT_RUN", observed["status"])
            self.assertIs(values["requirement_observed"], False)
            self.assertIn("no cross-process", values["evidence_scope"])
            self.assertTrue(observed["reason"])
            component = observed["component_evidence"]
            self.assertIs(component["executed"], True)
            values = observed["component_observations"]
        elif not observed["executed"]:
            self.assertEqual("NOT_RUN", observed["status"])
            self.assertIs(values["requirement_observed"], False)
            self.assertTrue(observed["reason"])
            return
        else:
            self.assertIs(observed["executed"], True)
            self.assertEqual("PASS", observed["status"])
            self.assertTrue(values["requirement_observed"])
        for key, expected in FIXTURE["expected_observations"][case_id].items():
            self.assertIn(key, values, f"{case_id} must report {key}")
            self.assertEqual(expected, values[key], f"{case_id}: {key}")
        if case_id == "ROS06":
            self.assertGreater(values["joint_state_sample_count"], 2)
            self.assertGreater(values["joint1_position_change_rad"], 0.02)

    return test


for _case_id in CASES:
    setattr(R06Acceptance, f"test_{_case_id.lower()}", _make_case(_case_id))


if __name__ == "__main__":
    unittest.main()
