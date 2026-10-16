#!/usr/bin/env python3
"""Verify ROS joint configuration against the C02 model contract."""

from __future__ import annotations

import math
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as element_tree

import yaml
import xacro


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
CONTRACT = WORKSPACE_ROOT / "contracts/dummy_robot_v2"
DESCRIPTION = WORKSPACE_ROOT / "src/dummy-ros2_description"
MOVEIT = WORKSPACE_ROOT / "src/dummy_moveit_config"


class ModelAlignmentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = yaml.safe_load((CONTRACT / "model/model.yaml").read_text())
        cls.joints = {joint["name"]: joint for joint in cls.model["joints"]}
        cls.urdf = element_tree.fromstring(
            xacro.process_file(str(DESCRIPTION / "urdf/dummy-ros2.xacro")).toxml()
        )

    def test_urdf_limits_match_contract_without_wrapping_joint6(self):
        urdf_joints = {
            joint.attrib["name"]: joint
            for joint in self.urdf.findall("joint")
            if joint.attrib["name"].startswith("Joint")
        }
        self.assertEqual(set(self.joints), set(urdf_joints))
        for name, contract_joint in self.joints.items():
            limit = urdf_joints[name].find("limit")
            lower, upper = contract_joint["model_limits_rad"]
            self.assertAlmostEqual(lower, float(limit.attrib["lower"]), places=12, msg=name)
            self.assertAlmostEqual(upper, float(limit.attrib["upper"]), places=12, msg=name)
        self.assertAlmostEqual(-4.0 * math.pi, float(urdf_joints["Joint6"].find("limit").attrib["lower"]), places=12)
        self.assertAlmostEqual(4.0 * math.pi, float(urdf_joints["Joint6"].find("limit").attrib["upper"]), places=12)

    def test_joint6_preserves_three_pi_and_rejects_five_pi_without_wrapping(self):
        lower, upper = self.joints["Joint6"]["model_limits_rad"]
        self.assertLessEqual(lower, 3.0 * math.pi)
        self.assertGreaterEqual(upper, 3.0 * math.pi)
        self.assertGreater(5.0 * math.pi, upper)
        self.assertNotEqual(3.0 * math.pi, math.remainder(3.0 * math.pi, 2.0 * math.pi))

    def test_mesh_bounds_match_transformed_collision_boxes(self):
        def vertices(path: Path):
            data = path.read_bytes()
            triangle_count = struct.unpack_from("<I", data, 80)[0]
            self.assertEqual(84 + triangle_count * 50, len(data), path.name)
            for triangle in range(triangle_count):
                offset = 84 + triangle * 50 + 12
                for vertex in range(3):
                    yield struct.unpack_from("<fff", data, offset + vertex * 12)

        links = [link for link in self.urdf.findall("link") if link.find("visual/geometry/mesh") is not None]
        self.assertEqual(7, len(links))
        for link in links:
            visual = link.find("visual")
            collision = link.find("collision")
            mesh_name = Path(visual.find("geometry/mesh").attrib["filename"]).name
            points = list(vertices(DESCRIPTION / "meshes" / mesh_name))
            raw_size = [(max(point[axis] for point in points) - min(point[axis] for point in points)) * 0.001
                        for axis in range(3)]
            collision_size = [float(value) for value in collision.find("geometry/box").attrib["size"].split()]
            self.assertEqual(3, len(collision_size))
            self.assertTrue(all(value > 0.0 for value in collision_size))
            self.assertAlmostEqual(sorted(raw_size)[0], sorted(collision_size)[0], places=5, msg=link.attrib["name"])
            self.assertAlmostEqual(sorted(raw_size)[1], sorted(collision_size)[1], places=5, msg=link.attrib["name"])
            self.assertAlmostEqual(sorted(raw_size)[2], sorted(collision_size)[2], places=5, msg=link.attrib["name"])
            self.assertIsNotNone(visual.find("origin"), link.attrib["name"])
            self.assertIsNotNone(collision.find("origin"), link.attrib["name"])
        report = " ".join((WORKSPACE_ROOT / "docs/dummy/model_alignment.md").read_text().split())
        self.assertIn("does not establish real collision clearance", report)

    def test_r01_tests_are_registered_with_ament(self):
        cmake = (MOVEIT / "CMakeLists.txt").read_text()
        self.assertIn("test_model_alignment.py", cmake)
        self.assertIn("test_reference_chain_fk.py", cmake)

    def test_initial_positions_are_complete_and_within_contract_limits(self):
        positions = yaml.safe_load((MOVEIT / "config/initial_positions.yaml").read_text())["initial_positions"]
        self.assertEqual(set(self.joints), set(positions))
        for name, position in positions.items():
            lower, upper = self.joints[name]["model_limits_rad"]
            self.assertGreaterEqual(position, lower, name)
            self.assertLessEqual(position, upper, name)

    def test_joint1_bench_mapping_applies_anchor_once_and_round_trips(self):
        joint = self.joints["Joint1"]
        anchor = 2.0
        position = 0.1
        motor_rev = anchor + joint["direction"] * joint["gear_ratio"] * position / (2.0 * math.pi)
        restored = joint["direction"] * 2.0 * math.pi * (motor_rev - anchor) / joint["gear_ratio"]
        self.assertAlmostEqual(2.0 - 5.0 / (2.0 * math.pi), motor_rev, places=12)
        self.assertAlmostEqual(position, restored, places=12)

    def test_all_zero_pose_is_rejected_by_joint3_contract_range(self):
        lower, _ = self.joints["Joint3"]["model_limits_rad"]
        self.assertGreater(lower, 0.0)

    def test_all_srdf_group_states_are_complete_and_within_limits(self):
        srdf = element_tree.parse(MOVEIT / "config/dummy-ros2.srdf").getroot()
        states = srdf.findall("group_state")
        self.assertGreaterEqual(len(states), 2)
        for state in states:
            positions = {joint.attrib["name"]: float(joint.attrib["value"]) for joint in state.findall("joint")}
            self.assertEqual(set(self.joints), set(positions), state.attrib["name"])
            for name, position in positions.items():
                lower, upper = self.joints[name]["model_limits_rad"]
                self.assertGreaterEqual(position, lower, f"{state.attrib['name']}:{name}")
                self.assertLessEqual(position, upper, f"{state.attrib['name']}:{name}")

    def test_alignment_report_does_not_claim_physical_collision_accuracy(self):
        report = " ".join(
            (WORKSPACE_ROOT / "docs/dummy/model_alignment.md").read_text().split()
        )
        self.assertIn("simulation-only approximations", report)
        self.assertIn("does not establish real collision clearance", report)


if __name__ == "__main__":
    unittest.main()
