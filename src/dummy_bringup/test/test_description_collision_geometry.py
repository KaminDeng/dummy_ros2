"""碰撞几何保留原CAD网格与局部坐标，不覆盖关节零位约定。"""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class CollisionGeometryTests(unittest.TestCase):
    def setUp(self):
        self.description = Path(__file__).resolve().parents[2] / 'dummy-ros2_description'
        self.root = ET.parse(self.description / 'urdf/dummy-ros2.xacro').getroot()

    def test_seven_collisions_match_visual_mesh_and_origin(self):
        links = self.root.findall('link')
        self.assertEqual(len(links), 7)
        for link in links:
            with self.subTest(link=link.attrib['name']):
                visual = link.find('visual')
                collision = link.find('collision')
                self.assertIsNotNone(collision)
                self.assertEqual(collision.find('origin').attrib, visual.find('origin').attrib)
                geometry = collision.find('geometry')
                self.assertEqual([child.tag for child in geometry], ['mesh'])
                self.assertEqual(geometry.find('mesh').attrib, visual.find('geometry/mesh').attrib)
                mesh = geometry.find('mesh').attrib['filename']
                self.assertTrue(mesh.endswith('.stl'))
                self.assertTrue((self.description / 'meshes' / Path(mesh).name).is_file())

    def test_joint3_canonical_offset_and_axis_are_preserved(self):
        joint = self.root.find("joint[@name='Joint3']")
        self.assertEqual(joint.find('origin').attrib['xyz'], '0.03665 0.0 0.146')
        rpy = [float(x) for x in joint.find('origin').attrib['rpy'].split()]
        self.assertAlmostEqual(rpy[0], math.pi / 2)
        self.assertEqual(rpy[1:], [0., 0.])
        self.assertEqual([float(x) for x in joint.find('axis').attrib['xyz'].split()], [-1., 0., 0.])


if __name__ == '__main__':
    unittest.main()
