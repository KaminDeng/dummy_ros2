#!/usr/bin/env python3
"""Ensure Gazebo and MoveIt consume one versioned scene definition."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as element_tree

import yaml


ROOT = Path(__file__).resolve().parents[3]
SCENE = ROOT / "src/dummy_bringup/config/scene.yaml"
WORLD = ROOT / "src/dummy_moveit_config/worlds/empty.sdf"


class R05SceneContractTest(unittest.TestCase):
    def test_scene_object_matches_gazebo_world(self):
        self.assertTrue(SCENE.is_file(), "canonical scene.yaml is missing")
        scene = yaml.safe_load(SCENE.read_text())
        self.assertEqual("dummy_scene_v1", scene["scene_revision"])
        self.assertEqual("world", scene["frame_id"])
        self.assertEqual(1, len(scene["objects"]))
        expected = scene["objects"][0]

        world = element_tree.parse(WORLD).getroot()
        model = world.find(f".//model[@name='{expected['id']}']")
        self.assertIsNotNone(model)
        size = [float(value) for value in model.find("link/collision/geometry/box/size").text.split()]
        pose = [float(value) for value in model.find("pose").text.split()]
        self.assertEqual(expected["size"], size)
        self.assertEqual(expected["pose_xyz_rpy"], pose)

    def test_gazebo_bringup_starts_scene_publisher_using_canonical_yaml(self):
        publisher = ROOT / "src/dummy_bringup/scripts/scene_publisher.py"
        launch = (ROOT / "src/dummy_bringup/launch/gazebo_6dof.launch.py").read_text()
        self.assertTrue(publisher.is_file())
        self.assertTrue(publisher.read_text().startswith("#!/usr/bin/python3\n"))
        self.assertIn('executable="scene_publisher.py"', launch)
        self.assertIn('"scene_file"', launch)
        self.assertNotIn("planning_barrier", publisher.read_text())

    def test_scene_loader_builds_typed_planning_scene_world(self):
        sys.path.insert(0, str(ROOT / "src/dummy_bringup/scripts"))
        from scene_publisher import load_scene
        from rclpy.serialization import serialize_message
        from moveit_msgs.srv import ApplyPlanningScene

        planning_scene = load_scene(SCENE)
        self.assertEqual("planning_barrier", planning_scene.world.collision_objects[0].id)
        request = ApplyPlanningScene.Request(scene=planning_scene)
        self.assertGreater(len(serialize_message(request)), 0)


if __name__ == "__main__":
    unittest.main()
