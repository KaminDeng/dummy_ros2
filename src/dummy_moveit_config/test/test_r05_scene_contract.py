#!/usr/bin/env python3
##
# @file test_r05_scene_contract.py
# @brief 验证Gazebo world与MoveIt场景使用同一版本化YAML来源。
# @details 加载类型化消息并离线序列化，检查生成SDF/重复对象ID拒绝；不调用场景服务。

"""Ensure Gazebo and MoveIt consume one versioned scene definition."""

from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as element_tree

import yaml


ROOT = Path(__file__).resolve().parents[3]
SCENE = ROOT / "src/dummy_bringup/config/scene.yaml"
WORLD = ROOT / "src/dummy_moveit_config/worlds/empty.sdf"


##
# @brief canonical场景的SDF/PlanningScene双消费者回归。
class R05SceneContractTest(unittest.TestCase):
    ##
    # @brief 核canonical scene版本/world坐标/单个对象，按对象ID匹配SDF并逐项比较尺寸与xyz/rpy。
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

    ##
    # @brief 核publisher文件/shebang、launch入口与scene_file参数，拒绝publisher硬编码planning_barrier名称。
    def test_gazebo_bringup_starts_scene_publisher_using_canonical_yaml(self):
        publisher = ROOT / "src/dummy_bringup/scripts/scene_publisher.py"
        launch = (ROOT / "src/dummy_bringup/launch/gazebo_6dof.launch.py").read_text()
        self.assertTrue(publisher.is_file())
        self.assertTrue(publisher.read_text().startswith("#!/usr/bin/python3\n"))
        self.assertIn('executable="scene_publisher.py"', launch)
        self.assertIn('"scene_file"', launch)
        self.assertNotIn("planning_barrier", publisher.read_text())

    ##
    # @brief 调用真实load_scene构造PlanningScene，再包装ApplyPlanningScene请求离线序列化；不发送ROS服务。
    def test_scene_loader_builds_typed_planning_scene_world(self):
        sys.path.insert(0, str(ROOT / "src/dummy_bringup/scripts"))
        from scene_publisher import load_scene
        from rclpy.serialization import serialize_message
        from moveit_msgs.srv import ApplyPlanningScene

        planning_scene = load_scene(SCENE)
        self.assertEqual("planning_barrier", planning_scene.world.collision_objects[0].id)
        request = ApplyPlanningScene.Request(scene=planning_scene)
        self.assertGreater(len(serialize_message(request)), 0)

    ##
    # @brief 比较render_world生成XML与受控SDF的规范解析后序列化，并核构建脚本调用生成器。
    def test_world_is_generated_from_the_canonical_scene(self):
        sys.path.insert(0, str(ROOT / "src/dummy_bringup/scripts"))
        from generate_world import render_world

        generated = element_tree.fromstring(render_world(SCENE))
        committed = element_tree.parse(WORLD).getroot()
        self.assertEqual(
            element_tree.tostring(generated, encoding="unicode"),
            element_tree.tostring(committed, encoding="unicode"),
        )
        self.assertIn("generate_world.py", (ROOT / "scripts/dummy/build.sh").read_text())

    ##
    # @brief 在临时YAML重复objects条目，真实world生成器和场景加载器都必须抛ValueError。
    def test_both_scene_consumers_reject_duplicate_identifiers(self):
        sys.path.insert(0, str(ROOT / "src/dummy_bringup/scripts"))
        from generate_world import render_world
        from scene_publisher import load_scene

        invalid = SCENE.read_text().replace("objects:\n", "objects:\n" + SCENE.read_text().split("objects:\n", 1)[1])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "duplicate.yaml"
            path.write_text(invalid)
            with self.assertRaises(ValueError):
                render_world(path)
            with self.assertRaises(ValueError):
                load_scene(path)


if __name__ == "__main__":
    unittest.main()
