"""碰撞几何保留原CAD网格与局部坐标，不覆盖关节零位约定。"""
##
# @file test_description_collision_geometry.py
# @brief 直接解析CAD xacro的碰撞网格与J3局部变换回归。
#
# 读取未展开XML并比较七个link的visual/collision原属性；只核资源与几何声明，不做MoveIt碰撞查询、网格拓扑或运动验证。平移m、rpy rad、轴向量无量纲。
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


##
# @brief 以原XML为输入的七网格/J3局部变换断言集合。
class CollisionGeometryTests(unittest.TestCase):
    ##
    # @brief 每个用例定位描述包并解析原始dummy-ros2.xacro到self.root。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
    def setUp(self):
        self.description = Path(__file__).resolve().parents[2] / 'dummy-ros2_description'
        self.root = ET.parse(self.description / 'urdf/dummy-ros2.xacro').getroot()

    ##
    # @brief 逐link核唯一mesh几何、origin及mesh属性与visual一致且STL文件存在；存在不代表网格内容正确。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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

    ##
    # @brief 核J3平移(0.03665,0,0.146)m、rollπ/2rad及局部负X轴，保留当前CAD零位约定。
    # @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
    def test_joint3_canonical_offset_and_axis_are_preserved(self):
        joint = self.root.find("joint[@name='Joint3']")
        self.assertEqual(joint.find('origin').attrib['xyz'], '0.03665 0.0 0.146')
        rpy = [float(x) for x in joint.find('origin').attrib['rpy'].split()]
        self.assertAlmostEqual(rpy[0], math.pi / 2)
        self.assertEqual(rpy[1:], [0., 0.])
        self.assertEqual([float(x) for x in joint.find('axis').attrib['xyz'].split()], [-1., 0., 0.])


if __name__ == '__main__':
    unittest.main()
