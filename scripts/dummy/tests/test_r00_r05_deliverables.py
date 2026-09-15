"""Required R00-R05 source and test registration checks."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]


class R00R05Deliverables(unittest.TestCase):
    def test_r01_and_r02_acceptance_entrypoints_are_delivered(self):
        for name in ("test_r01_acceptance.py", "test_r02_acceptance.py"):
            path = ROOT / "scripts/dummy/tests" / name
            self.assertTrue(path.is_file(), str(path))

    def test_reference_chain_xacro_is_delivered(self):
        path = ROOT / "src/dummy-ros2_description/urdf/dummy_reference_chain.xacro"
        self.assertTrue(path.is_file(), str(path))
        text = path.read_text()
        self.assertIn("<xacro:macro", text)
        self.assertIn("Joint6", text)

    def test_gazebo_runtime_launch_test_is_registered(self):
        path = ROOT / "src/dummy_bringup/test/test_gazebo_runtime.launch.py"
        self.assertTrue(path.is_file(), str(path))
        cmake = (ROOT / "src/dummy_bringup/CMakeLists.txt").read_text()
        self.assertIn("test_gazebo_runtime.launch.py", cmake)
        self.assertIn("add_launch_test", cmake)

    def test_moveit_plan_execute_launch_test_is_registered(self):
        path = ROOT / "src/dummy_bringup/test/test_plan_execute.launch.py"
        self.assertTrue(path.is_file(), str(path))
        cmake = (ROOT / "src/dummy_bringup/CMakeLists.txt").read_text()
        self.assertIn("test_plan_execute.launch.py", cmake)


if __name__ == "__main__":
    unittest.main()
