#!/usr/bin/env python3
"""Static prerequisites for the R02 Gazebo runtime test."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as element_tree


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
MOVEIT = WORKSPACE_ROOT / "src/dummy_moveit_config"
BRINGUP = WORKSPACE_ROOT / "src/dummy_bringup"


class R02GazeboContractTest(unittest.TestCase):
    def test_gazebo_nodes_use_sim_time_and_do_not_start_device_transport(self):
        source = (MOVEIT / "launch/gz_sim.launch.py").read_text()
        self.assertGreaterEqual(source.count('"use_sim_time": True'), 3)
        for forbidden in ("ttyACM", "SocketCAN", "JLink", "dpsdk", "zenoh"):
            self.assertNotIn(forbidden, source)

    def test_combined_launch_exposes_gui_and_rviz_controls(self):
        source = (MOVEIT / "launch/gz_moveit.launch.py").read_text()
        self.assertIn('DeclareLaunchArgument("gui"', source)
        self.assertIn('DeclareLaunchArgument("rviz"', source)
        self.assertIn('LaunchConfiguration("gui")', source)
        self.assertIn('IfCondition(LaunchConfiguration("rviz"))', source)

    def test_bringup_installs_canonical_gazebo_entrypoint(self):
        launch = BRINGUP / "launch/gazebo_6dof.launch.py"
        self.assertTrue(launch.is_file())
        self.assertIn("install(DIRECTORY launch", (BRINGUP / "CMakeLists.txt").read_text())

    def test_collision_geometry_uses_gazebo_supported_positive_boxes(self):
        root = element_tree.parse(
            WORKSPACE_ROOT / "src/dummy-ros2_description/urdf/dummy-ros2.xacro"
        ).getroot()
        links = root.findall("link")
        self.assertEqual(7, len(links))
        for link in links:
            self.assertIsNotNone(link.find("visual/geometry/mesh"), link.attrib["name"])
            collision = link.find("collision/geometry")
            self.assertIsNone(collision.find("mesh"), link.attrib["name"])
            size = [float(value) for value in collision.find("box").attrib["size"].split()]
            self.assertEqual(3, len(size))
            self.assertTrue(all(value > 0 for value in size), link.attrib["name"])


if __name__ == "__main__":
    unittest.main()
