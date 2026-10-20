#!/usr/bin/env python3
"""Verify ROS joint configuration against the C02 model contract."""

from __future__ import annotations

import math
import hashlib
import itertools
import json
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as element_tree

import yaml
import xacro
import numpy as np


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

    def test_visual_and_collision_geometry_align_with_cad_bounds(self):
        def vertices(path: Path):
            self.assertTrue(path.is_file(), str(path))
            if path.suffix == '.obj':
                return np.array([tuple(map(float, line.split()[1:4]))
                                 for line in path.read_text().splitlines() if line.startswith('v ')])
            self.assertEqual('.stl', path.suffix)
            data = path.read_bytes()
            triangle_count = struct.unpack_from("<I", data, 80)[0]
            self.assertEqual(84 + triangle_count * 50, len(data), path.name)
            triangles = np.frombuffer(data, dtype=np.dtype([
                ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]),
                offset=84, count=triangle_count)
            return triangles['vertices'].reshape(-1, 3).astype(float)

        def bounds(element, cad_bounds=None):
            origin = element.find('origin')
            self.assertIsNotNone(origin)
            xyz, rpy = [np.array(list(map(float, origin.get(key, '0 0 0').split())))
                        for key in ('xyz', 'rpy')]
            self.assertEqual((3,), xyz.shape)
            self.assertEqual((3,), rpy.shape)
            self.assertTrue(np.isfinite(xyz).all() and np.isfinite(rpy).all())
            r, p, y = rpy
            rotation = np.array([
                [math.cos(y)*math.cos(p), math.cos(y)*math.sin(p)*math.sin(r)-math.sin(y)*math.cos(r), math.cos(y)*math.sin(p)*math.cos(r)+math.sin(y)*math.sin(r)],
                [math.sin(y)*math.cos(p), math.sin(y)*math.sin(p)*math.sin(r)+math.cos(y)*math.cos(r), math.sin(y)*math.sin(p)*math.cos(r)-math.cos(y)*math.sin(r)],
                [-math.sin(p), math.cos(p)*math.sin(r), math.cos(p)*math.cos(r)]])
            geometry = element.find('geometry')
            mesh, box, cylinder = (geometry.find(kind) for kind in ('mesh', 'box', 'cylinder'))
            if mesh is not None:
                prefix = 'package://dummy-ros2_description/'
                self.assertTrue(mesh.get('filename').startswith(prefix))
                scale = np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
                np.testing.assert_allclose(scale, [.001]*3, atol=0, rtol=0)
                points = (vertices(DESCRIPTION / mesh.get('filename')[len(prefix):])
                          if cad_bounds is None else np.array(list(itertools.product(*zip(*cad_bounds)))))
                self.assertGreater(len(points), 3)
                self.assertTrue(np.isfinite(points).all())
                points = (points*scale) @ rotation.T + xyz
                return points.min(axis=0), points.max(axis=0)
            if box is not None:
                dimensions = np.array(list(map(float, box.get('size').split())))
                self.assertEqual((3,), dimensions.shape)
                self.assertTrue(np.isfinite(dimensions).all() and (dimensions > 0).all())
                extent = np.abs(rotation) @ (dimensions/2)
            else:
                self.assertIsNotNone(cylinder)
                radius, length = (float(cylinder.get(key)) for key in ('radius', 'length'))
                self.assertTrue(all(math.isfinite(value) and value > 0 for value in (radius, length)))
                extent = radius*np.linalg.norm(rotation[:, :2], axis=1) + length/2*np.abs(rotation[:, 2])
            return xyz-extent, xyz+extent

        links = [link for link in self.urdf.findall("link") if link.find("visual/geometry/mesh") is not None]
        self.assertEqual({'base_link', 'j6_reducer_housing', *[f'link{i}_1_1' for i in range(1, 7)]},
                         {link.get('name') for link in links})
        cad_groups = {item['link']: item for item in json.loads(
            (DESCRIPTION/'meshes/full_step_manifest.json').read_text())['groups'].values()}
        self.assertEqual({'base_link', *[f'link{i}_1_1' for i in range(1, 6)]}, set(cad_groups))
        visual_report = json.loads(
            (DESCRIPTION/'meshes/full_step_visual_optimization.json').read_text())
        visual_variant = visual_report.get('active_visual_variant', 'optimized')
        self.assertIn(visual_variant, ('original', 'optimized'))
        identity_key = 'original_sha256' if visual_variant == 'original' else 'sha256'
        visual_identities = {item['file']: item[identity_key] for item in visual_report['files']}
        for link in links:
            visual_min, visual_max = bounds(link.find('visual'))
            self.assertTrue((visual_max > visual_min).all(), link.get('name'))
            collisions = link.findall('collision')
            self.assertTrue(collisions, link.get('name'))
            if link.get('name') in cad_groups:
                # STEP曲面的原始显示离散与碰撞包络不是同一几何；碰撞覆盖以独立CAD
                # bounds为依据，而非把有离散误差的OBJ顶点当物理外轮廓。
                visual = link.find('visual')
                mesh_path = DESCRIPTION/'meshes'/Path(visual.find('geometry/mesh').get('filename')).name
                self.assertEqual(visual_identities[mesh_path.name], hashlib.sha256(mesh_path.read_bytes()).hexdigest())
                self.assertEqual(visual.find('origin').attrib, collisions[0].find('origin').attrib)
                visual_min, visual_max = bounds(visual, cad_groups[link.get('name')]['local_bounds_mm'])
            collision_bounds = [bounds(collision) for collision in collisions]
            collision_min = np.min([item[0] for item in collision_bounds], axis=0)
            collision_max = np.max([item[1] for item in collision_bounds], axis=0)
            # 逐轴比较连杆局部坐标，不能排序尺寸掩盖轴向/安装原点错误。
            # 1e-8m仅允许STL float32舍入；这里仍不证明任意内部凹面或实机间隙。
            self.assertTrue((collision_min <= visual_min + 1e-8).all(), link.get('name'))
            self.assertTrue((collision_max >= visual_max - 1e-8).all(), link.get('name'))
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
