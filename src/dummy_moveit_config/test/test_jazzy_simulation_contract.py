#!/usr/bin/env python3
##
# @file test_jazzy_simulation_contract.py
# @brief 静态检查Jazzy Gazebo接口、加速度限值与world固定基座。
# @details 多数断言搜索源码片段，只证明配置结构/文本契约，不证明Gazebo已经启动或控制正常。

"""Regression checks for the Jazzy simulator and planning configuration."""

from pathlib import Path
import unittest

import yaml


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
MOVEIT_CONFIG = WORKSPACE_SRC / "dummy_moveit_config"
DESCRIPTION = WORKSPACE_SRC / "dummy-ros2_description"


##
# @brief Jazzy仿真配置静态前提。
class JazzySimulationContractTest(unittest.TestCase):
    ##
    # @brief 核六关节均声明加速度限制且max_acceleration大于0；不在此证明轨迹满足限值。
    def test_joint_limits_define_positive_acceleration_for_every_arm_joint(self):
        limits = yaml.safe_load(
            (MOVEIT_CONFIG / "config" / "joint_limits.yaml").read_text()
        )["joint_limits"]

        for joint in ("Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"):
            self.assertTrue(limits[joint]["has_acceleration_limits"])
            self.assertGreater(limits[joint]["max_acceleration"], 0.0)

    ##
    # @brief 合并相关Python/Xacro/XML源码拒绝旧gazebo_ros字串，并核gz_sim launch含ros_gz_sim/gz_ros2_control。
    def test_simulation_launch_uses_jazzy_gazebo_sim_interfaces(self):
        gazebo_sources = "\n".join(
            source.read_text()
            for directory in (
                DESCRIPTION / "urdf",
                MOVEIT_CONFIG / "config",
                MOVEIT_CONFIG / "launch",
            )
            for source in directory.rglob("*")
            if source.suffix in {".py", ".xacro", ".xml"}
        )
        launch_source = (MOVEIT_CONFIG / "launch" / "gz_sim.launch.py").read_text()

        self.assertNotIn("gazebo_ros", gazebo_sources)
        self.assertIn("ros_gz_sim", launch_source)
        self.assertIn("gz_ros2_control", launch_source)

    ##
    # @brief 查仿真Xacro的world link与fixed world_joint声明；是文本结构回归，不运行物理引擎。
    def test_gazebo_sim_model_anchors_the_base_link_to_world(self):
        simulation_xacro = (
            MOVEIT_CONFIG / "config" / "dummy-ros2.gz_sim.xacro"
        ).read_text()

        self.assertIn('<link name="world"', simulation_xacro)
        self.assertIn('<joint name="world_joint" type="fixed"', simulation_xacro)


if __name__ == "__main__":
    unittest.main()
