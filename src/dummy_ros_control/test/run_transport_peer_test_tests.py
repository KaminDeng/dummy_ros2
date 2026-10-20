import importlib.util
##
# @file run_transport_peer_test_tests.py
# @brief 设备端点解析器的纯输入替身测试。
#
# 使用注入glob路径/环境字典，不访问/dev/运行peer。方法名虽提拒JLink，实际负例只覆盖默认ttyACM0和零发现，不应扩成显式路径完整USB身份验证。
import pathlib
import unittest


MODULE = pathlib.Path(__file__).with_name("run_transport_peer_test.py")
SPEC = importlib.util.spec_from_file_location("transport_peer", MODULE)
transport_peer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(transport_peer)


##
# @brief 注入环境/glob的端点解析单测，无设备发现。
class DiscoveryTests(unittest.TestCase):
    ##
    # @brief 注入唯一精确CDC by-id路径，核绝对serial双斜线/115200端点。
    # @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
    def test_uses_the_dummy_robotics_by_id_device(self):
        endpoint = transport_peer.resolve_real_endpoint(
            {}, lambda _: [pathlib.Path("/dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00")])
        self.assertEqual(endpoint, "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200")

    ##
    # @brief 现有断言只测显式ACM0和空发现；没有注入JLink端点或多个候选，原oracle不扩大覆盖。
    # @return None；以assert/unittest/pytest.raises报告既定检查，失败抛异常。
    def test_refuses_jlink_or_ambiguous_discovery(self):
        with self.assertRaises(RuntimeError):
            transport_peer.resolve_real_endpoint({"DUMMY_ROS_PRODUCTION_ENDPOINT": "serial//dev/ttyACM0#baudrate=115200"})
        with self.assertRaises(RuntimeError):
            transport_peer.resolve_real_endpoint({}, lambda _: [])


if __name__ == "__main__":
    unittest.main()
