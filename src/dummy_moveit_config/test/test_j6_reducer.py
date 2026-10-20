"""验证减速器安装后运动学、外壳归属和碰撞覆盖；不访问实机。"""
import math
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro


ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT / 'src/dummy-ros2_description'
HOUSING = 'j6_reducer_housing'


def transform(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0],
                   [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = xyz
    return result


def origin(element):
    node = element.find('origin')
    if node is None:
        return np.eye(4)
    return transform(*[tuple(map(float, node.get(key, '0 0 0').split()))
                       for key in ('xyz', 'rpy')])


def frames(root, positions):
    result = {'base_link': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in result:
                continue
            motion = np.eye(4)
            if joint.get('type') == 'revolute':
                axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
                angle = positions.get(joint.get('name'), 0.)
                cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                                  [-axis[1], axis[0], 0]])
                motion[:3, :3] = (np.eye(3) + math.sin(angle) * cross
                                     + (1 - math.cos(angle)) * cross @ cross)
            result[joint.find('child').get('link')] = result[parent] @ origin(joint) @ motion
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise AssertionError('模型中有断开的关节链')
    return result


class J6ReducerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.fromstring(xacro.process_file(
            str(DESCRIPTION / 'urdf/dummy-ros2.xacro')).toxml())

    def test_output_flange_extends_28mm_without_changing_six_axes(self):
        self.assertEqual(6, len(self.root.findall("joint[@type='revolute']")))
        for angles in ([0, 0, 90, 0, 0, 0], [20, -60, 164, 40, 70, 160],
                       [-50, 30, 80, -30, -90, -540]):
            f = frames(self.root, {f'Joint{i+1}': math.radians(q)
                                   for i, q in enumerate(angles)})
            # 以STEP末端支架安装平面为基准，输出沿局部+Y外移28mm。
            mount = f['link5_1_1'] @ transform((-.0168, .0625, 0.))
            np.testing.assert_allclose(f[HOUSING],mount,atol=1e-12)
            expected = mount @ np.array([0., .028, 0., 1.])
            np.testing.assert_allclose(f['link6_1_1'][:, 3], expected, atol=1e-12)
            self.assertAlmostEqual(.028, np.linalg.norm(
                f['link6_1_1'][:3, 3] - mount[:3, 3]), places=12)

    def test_housing_does_not_rotate_with_joint6(self):
        first = frames(self.root, {'Joint3': math.pi / 2, 'Joint6': 0.})
        second = frames(self.root, {'Joint3': math.pi / 2, 'Joint6': 1.2})
        self.assertIn(HOUSING, first, '缺少减速器固定壳体')
        np.testing.assert_allclose(first[HOUSING], second[HOUSING], atol=1e-12)
        self.assertFalse(np.allclose(first['link6_1_1'][:3, :3],
                                     second['link6_1_1'][:3, :3]))

    def test_collision_primitives_cover_every_stl_vertex(self):
        housing = self.root.find(f"link[@name='{HOUSING}']")
        self.assertIsNotNone(housing, '缺少减速器碰撞模型')
        visual = housing.find('visual')
        mesh = visual.find('geometry/mesh')
        scale = np.array(list(map(float, mesh.get('scale').split())))
        self.assertEqual([.001] * 3, list(scale))
        data = (DESCRIPTION / 'meshes' / Path(mesh.get('filename')).name).read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        self.assertEqual(84 + count * 50, len(data))
        collisions = housing.findall('collision')
        self.assertEqual(2, len(collisions))
        localizers = [(np.linalg.inv(origin(c)), c.find('geometry')) for c in collisions]
        for triangle in range(count):
            for vertex in range(3):
                point = struct.unpack_from('<3f', data, 84 + triangle * 50 + 12 + vertex * 12)
                point = origin(visual) @ np.r_[np.array(point) * scale, 1.]
                covered = False
                for inverse, geometry in localizers:
                    xyz = (inverse @ point)[:3]
                    box = geometry.find('box')
                    if box is not None:
                        size = np.array(list(map(float, box.get('size').split())))
                        covered |= bool(np.all(np.abs(xyz) <= size / 2 + 1e-8))
                    cylinder = geometry.find('cylinder')
                    if cylinder is not None:
                        covered |= (np.linalg.norm(xyz[:2]) <= float(cylinder.get('radius')) + 1e-8
                                    and abs(xyz[2]) <= float(cylinder.get('length')) / 2 + 1e-8)
                self.assertTrue(covered, f'STL 顶点未被碰撞体覆盖：{point}')

    def test_only_mounting_neighbors_exempt_housing_collision(self):
        srdf = ET.parse(ROOT / 'src/dummy_moveit_config/config/dummy-ros2.srdf').getroot()
        pairs = {frozenset((p.get('link1'), p.get('link2')))
                 for p in srdf.findall('disable_collisions')}
        for neighbor in ('link5_1_1', 'link6_1_1'):
            self.assertIn(frozenset((HOUSING, neighbor)), pairs)
        for other in ('base_link', 'link1_1_1', 'link2_1_1', 'link3_1_1', 'link4_1_1'):
            self.assertNotIn(frozenset((HOUSING, other)), pairs)
        # 末端位置改变后，不沿用旧模型对前臂的 Never 豁免。
        self.assertNotIn(frozenset(('link3_1_1', 'link6_1_1')), pairs)


if __name__ == '__main__':
    unittest.main()
