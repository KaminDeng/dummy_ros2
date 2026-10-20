##
# @file test_full_step_model.py
# @brief 检查完整STEP视觉资源、本地材质和独立电机碰撞模型的发布约束。
# @details 展开生产Xacro后核URI/材质/固定关节与SRDF；不证明任意姿态均无碰撞。

"""完整STEP模型必须随ROS包发布，并保留颜色和电机独立碰撞检查。"""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import xacro


ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT/'src/dummy-ros2_description'


##
# @brief 完整STEP资源、材质和独立碰撞结构回归。
class FullStepModelTest(unittest.TestCase):
    ##
    # @brief 类级一次展开生产Xacro并缓存XML供各测试读取；需要合同的类同时读取当前合同关节表。
    @classmethod
    def setUpClass(cls):
        cls.model=ET.fromstring(xacro.process_file(str(DESCRIPTION/'urdf/dummy-ros2.xacro')).toxml())

    ##
    # @brief 逐底座/J1..J5检查完整STEP OBJ包URI、实际存在的本地MTL引用及未被统一URDF material覆盖。
    def test_complete_visual_meshes_have_local_materials(self):
        for link_name in ('base_link',*[f'link{i}_1_1' for i in range(1,6)]):
            visual=self.model.find(f"link[@name='{link_name}']/visual")
            mesh=visual.find('geometry/mesh')
            uri=mesh.get('filename')
            self.assertTrue(uri.startswith('package://dummy-ros2_description/meshes/full_step_'),uri)
            path=DESCRIPTION/'meshes'/uri.rsplit('/',1)[-1]
            self.assertEqual('.obj',path.suffix)
            references=re.findall(r'^mtllib (.+)$',path.read_text(),re.M)
            self.assertTrue(references)
            for reference in references:self.assertTrue((path.parent/reference).is_file())
            self.assertIsNone(visual.find('material'),'不能统一覆盖源材质')

    ##
    # @brief 核独立电机都有collision和fixed挂接，保留指定非邻近碰撞对且不存在Never原因的全局豁免。
    def test_independent_motor_collisions_are_not_globally_exempt(self):
        motors=[link for link in self.model.findall('link') if link.get('name').startswith('step_')]
        self.assertGreaterEqual(len(motors),5)
        for motor in motors:
            self.assertIsNotNone(motor.find('collision'))
            joint=self.model.find(f"joint/child[@link='{motor.get('name')}']/..")
            self.assertEqual('fixed',joint.get('type'))
        srdf=ET.parse(ROOT/'src/dummy_moveit_config/config/dummy-ros2.srdf').getroot()
        pairs={frozenset((p.get('link1'),p.get('link2'))) for p in srdf.findall('disable_collisions')}
        self.assertNotIn(frozenset(('link2_1_1','step_link5_motor_1')),pairs)
        self.assertFalse(any(p.get('reason')=='Never' for p in srdf.findall('disable_collisions')))

    ##
    # @brief 扫描所有mesh URI，拒绝Downloads/reports/tmp开发机路径泄漏；这里只检查这三类禁用片段。
    def test_no_mesh_depends_on_download_or_report_directory(self):
        for mesh in self.model.findall('.//mesh'):
            uri=mesh.get('filename')
            self.assertNotIn('/Downloads/',uri)
            self.assertNotIn('/reports/',uri)
            self.assertNotIn('/tmp/',uri)


if __name__=='__main__':unittest.main()
