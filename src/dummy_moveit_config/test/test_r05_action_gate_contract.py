"""Configuration boundary checks for the native Gazebo action gate."""

from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[3]


class R05ActionGateContract(unittest.TestCase):
    def test_public_action_uses_gate_and_jtc_is_internal(self):
        source = ROOT / "src/dummy_motion_client/src/native_action_gate.cpp"
        self.assertTrue(source.is_file(), "native action gate is missing")
        controllers = yaml.safe_load(
            (ROOT / "src/dummy_moveit_config/config/ros2_controllers.yaml").read_text()
        )
        manager = controllers["controller_manager"]["ros__parameters"]
        self.assertIn("_internal_arm_controller", manager)
        self.assertNotIn("dummy_arm_controller", manager)
        moveit = yaml.safe_load(
            (ROOT / "src/dummy_moveit_config/config/moveit_controllers.yaml").read_text()
        )
        self.assertEqual(["arm_controller"], moveit["moveit_simple_controller_manager"]["controller_names"])
        self.assertIn('"/arm_controller/follow_joint_trajectory"', source.read_text())
        self.assertIn('"/_internal_arm_controller/follow_joint_trajectory"', source.read_text())
        self.assertIn("auto node = std::make_shared<dummy_motion_client::NativeActionGate>()", source.read_text())
        self.assertNotIn(".detach()", source.read_text())
        internal = controllers["_internal_arm_controller"]["ros__parameters"]
        for name in ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"):
            self.assertEqual(0.5, internal["constraints"][name]["max_deceleration_on_cancel"])


if __name__ == "__main__":
    unittest.main()
