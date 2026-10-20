"""R07 backend and SIL fault acceptance."""
##
# @file test_r07_acceptance.py
# @brief 按R07既有fixture注册后端/SIL验收方法。
#
# fixture只提供case名；真正运行方法会调用adapter启动CTest、上游/本机peer或Action。real_hardware_accessed=False是返回范围字段，不是独立硬件审计；第一条断言已强制PASS，下面else分支不能作为本oracle接受NOT_RUN的证据。

import json
from pathlib import Path
import sys
import unittest

TEST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_ROOT))
from dummy_test_support.task_adapters import run_case


ROOT = Path(__file__).resolve().parents[3]
CASES = json.loads((ROOT / "scripts/dummy/tests/fixtures/R07.json").read_text())["cases"]


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R07Acceptance(unittest.TestCase):
    pass


##
# @brief 按case_id生成并注册unittest闭包，不在导入时运行ROS/peer。
# @param case_id 当前case标识；支持集合由本suite映射/分支决定。
# @return 闭包test函数，尚未执行其adapter。
def _make_case(case_id: str):
    ##
    # @par 内部回调 test
    # 先强制PASS，再核摘要及每case要求字段、上游trace、J1接口/steady/无自动管理声明；条件else因前置PASS断言不会成为成功路径。
    # 返回：None；通过assert/unittest断言报告结果，失败抛异常。
    def test(self):
        observed = run_case("R07", case_id, {
            "case_id": case_id, "hardware": "none", "test_only": True,
        })
        self.assertEqual("PASS", observed["status"])
        self.assertIsInstance(observed["reason"], str)
        self.assertTrue(observed["sut_source_sha256"])
        self.assertTrue(observed["trace_sha256"])
        values = observed["observations"]
        self.assertEqual(case_id, values["case_id"])
        self.assertFalse(values.get("real_hardware_accessed", False))
        if observed["status"] == "PASS":
            self.assertIs(observed["executed"], True)
            self.assertTrue(values["requirement_observed"])
        else:
            self.assertIs(observed["executed"], False)
            self.assertTrue(observed["reason"])
            self.assertTrue(values["blocked"])

        required = {
            "B02": ("posix_peer_started", "ros_device_chain_started"),
            "AUTH04": ("auto_management_calls", "posix_peer_started"),
            "FRE11": ("steady_clock_source", "simulation_clock_isolated"),
            "SEN05": ("robot_core", "virtual_can", "configured_mask"),
            "SEN06": ("robot_core", "virtual_can", "configured_mask"),
            "ROS07": ("stale_read_result",),
            "ROS08": ("management_while_inactive",),
            "ROS10": ("controller_state_interfaces", "controller_command_interfaces"),
        }[case_id]
        for field in required:
            self.assertIn(field, values)
        if case_id in {"B02", "AUTH04", "SEN05", "SEN06"}:
            self.assertIn(values["upstream_subtest_status"], {"PASS", "FAIL"})
            self.assertTrue(values["upstream_trace_sha256"])
        if case_id == "AUTH04":
            self.assertEqual(0, values["auto_management_calls"])
        if case_id == "FRE11" and observed["status"] == "PASS":
            self.assertEqual("steady", values["steady_clock_source"])
            self.assertIs(values["simulation_clock_isolated"], True)
        if case_id == "ROS07" and observed["status"] == "PASS":
            self.assertEqual("ERROR_WHEN_ACTIVE", values["stale_read_result"])
            self.assertEqual("OK", values["inactive_poll_result"])
        if case_id == "ROS10" and observed["status"] == "PASS":
            self.assertEqual(["Joint1/position", "Joint1/velocity"], values["controller_state_interfaces"])
            self.assertEqual(["Joint1/position"], values["controller_command_interfaces"])

    return test


for _case_id in CASES:
    setattr(R07Acceptance, f"test_{_case_id.lower()}", _make_case(_case_id))


if __name__ == "__main__":
    unittest.main()
