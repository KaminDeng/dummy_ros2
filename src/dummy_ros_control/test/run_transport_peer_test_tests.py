import importlib.util
import pathlib
import unittest


MODULE = pathlib.Path(__file__).with_name("run_transport_peer_test.py")
SPEC = importlib.util.spec_from_file_location("transport_peer", MODULE)
transport_peer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(transport_peer)


class DiscoveryTests(unittest.TestCase):
    def test_uses_the_dummy_robotics_by_id_device(self):
        endpoint = transport_peer.resolve_real_endpoint(
            {}, lambda _: [pathlib.Path("/dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00")])
        self.assertEqual(endpoint, "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200")

    def test_refuses_jlink_or_ambiguous_discovery(self):
        with self.assertRaises(RuntimeError):
            transport_peer.resolve_real_endpoint({"DUMMY_ROS_PRODUCTION_ENDPOINT": "serial//dev/ttyACM0#baudrate=115200"})
        with self.assertRaises(RuntimeError):
            transport_peer.resolve_real_endpoint({}, lambda _: [])


if __name__ == "__main__":
    unittest.main()
