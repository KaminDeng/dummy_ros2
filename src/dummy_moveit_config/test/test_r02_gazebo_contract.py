#!/usr/bin/env python3
##
# @file test_r02_gazebo_contract.py
# @brief 检查R02仿真入口与完整碰撞几何的静态前提。
# @details 不启动仿真：核use_sim_time、入口安装、可用形状与本地STL结构，不代表运行时跟踪资格。

"""Static prerequisites for the R02 Gazebo runtime test."""

from pathlib import Path
import math
import struct
import unittest
import xml.etree.ElementTree as element_tree

import numpy as np
import xacro


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
MOVEIT = WORKSPACE_ROOT / "src/dummy_moveit_config"
BRINGUP = WORKSPACE_ROOT / "src/dummy_bringup"


##
# @brief R02仿真入口和碰撞资源结构回归。
class R02GazeboContractTest(unittest.TestCase):
    ##
    # @brief 核launch至少三处use_sim_time为True，并拒绝串口/CAN/JLink/DPSDK/Zenoh禁用片段。
    def test_gazebo_nodes_use_sim_time_and_do_not_start_device_transport(self):
        source = (MOVEIT / "launch/gz_sim.launch.py").read_text()
        self.assertGreaterEqual(source.count('"use_sim_time": True'), 3)
        for forbidden in ("ttyACM", "SocketCAN", "JLink", "dpsdk", "zenoh"):
            self.assertNotIn(forbidden, source)

    ##
    # @brief 核组合launch声明并使用gui/rviz参数以及RViz条件启动表达式。
    def test_combined_launch_exposes_gui_and_rviz_controls(self):
        source = (MOVEIT / "launch/gz_moveit.launch.py").read_text()
        self.assertIn('DeclareLaunchArgument("gui"', source)
        self.assertIn('DeclareLaunchArgument("rviz"', source)
        self.assertIn('LaunchConfiguration("gui")', source)
        self.assertIn('IfCondition(LaunchConfiguration("rviz"))', source)

    ##
    # @brief 核标准Gazebo入口存在，且bringup CMake文本包含launch目录安装规则。
    def test_bringup_installs_canonical_gazebo_entrypoint(self):
        launch = BRINGUP / "launch/gazebo_6dof.launch.py"
        self.assertTrue(launch.is_file())
        self.assertIn("install(DIRECTORY launch", (BRINGUP / "CMakeLists.txt").read_text())

    ##
    # @brief 核完整link集合/视觉归属、fixed附属体、有限origin和正尺寸；STL核本地URI、mm缩放、长度、有限非退化顶点。
    def test_complete_collision_geometry_uses_supported_positive_shapes(self):
        description = WORKSPACE_ROOT / 'src/dummy-ros2_description'
        root = element_tree.fromstring(xacro.process_file(
            str(description / 'urdf/dummy-ros2.xacro')).toxml())
        links = root.findall("link")
        motors = {'step_link2_motor_1', 'step_link2_motor_2', 'step_link3_motor_1',
                  'step_link4_motor_1', 'step_link5_motor_1'}
        guards = {'clearance_base_link', 'clearance_link1_1_1', 'clearance_link2_1_1',
                  'clearance_step_link2_motor_1', 'clearance_step_link2_motor_2'}
        visuals = {'base_link', 'j6_reducer_housing', *[f'link{i}_1_1' for i in range(1, 7)]}
        self.assertEqual(visuals | motors | guards, {link.get('name') for link in links})
        for link in links:
            name = link.get('name')
            if name in visuals:
                self.assertIsNotNone(link.find('visual/geometry/mesh'), name)
            else:
                self.assertIsNone(link.find('visual'), name)
                attachment = root.find(f"joint/child[@link='{name}']/..")
                self.assertIsNotNone(attachment, name)
                self.assertEqual('fixed', attachment.get('type'), name)
            collisions = link.findall('collision')
            self.assertTrue(collisions, name)
            for collision in collisions:
                origin = collision.find('origin')
                self.assertIsNotNone(origin, name)
                for key in ('xyz', 'rpy'):
                    values = list(map(float, origin.get(key).split()))
                    self.assertEqual(3, len(values), name)
                    self.assertTrue(all(math.isfinite(value) for value in values), name)
                geometry = collision.find('geometry')
                self.assertEqual(1, len(geometry), name)
                shape = geometry[0]
                self.assertIn(shape.tag, ('box', 'cylinder', 'mesh'), name)
                if shape.tag == 'mesh':
                    prefix = 'package://dummy-ros2_description/'
                    self.assertTrue(shape.get('filename').startswith(prefix), name)
                    path = description / shape.get('filename')[len(prefix):]
                    self.assertTrue(path.is_file(), str(path))
                    self.assertEqual('.stl', path.suffix)
                    scale = list(map(float, shape.get('scale', '1 1 1').split()))
                    self.assertEqual([.001]*3, scale, name)
                    data = path.read_bytes()
                    count = struct.unpack_from('<I', data, 80)[0]
                    self.assertGreater(count, 3, name)
                    self.assertEqual(84+50*count, len(data), name)
                    records = np.frombuffer(data, offset=84, count=count, dtype=np.dtype([
                        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
                    points = records['vertices'].reshape(-1, 3)
                    self.assertTrue(np.isfinite(points).all(), name)
                    self.assertTrue((points.max(axis=0) > points.min(axis=0)).all(), name)
                else:
                    dimensions = (list(map(float, shape.get('size').split())) if shape.tag == 'box'
                                  else [float(shape.get(key)) for key in ('radius', 'length')])
                    self.assertEqual(3 if shape.tag == 'box' else 2, len(dimensions), name)
                    self.assertTrue(all(math.isfinite(value) and value > 0 for value in dimensions), name)


if __name__ == "__main__":
    unittest.main()
