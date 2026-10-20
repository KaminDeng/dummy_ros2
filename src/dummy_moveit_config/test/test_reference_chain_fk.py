#!/usr/bin/env python3
##
# @file test_reference_chain_fk.py
# @brief 将独立参考链URDF的KDL正解与旧FK固定/随机证据比较。
# @details 参考链用于数学一致性，不是完整视觉装配URDF；固定例角度为deg、随机例为rad，输出位置m。

"""Compare the ROS reference-chain URDF with the C02 legacy FK evidence."""

from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as element_tree

import PyKDL


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
REFERENCE_CHAIN = WORKSPACE_ROOT / "src/dummy-ros2_description/urdf/dummy_reference_chain.urdf"
FIXED_CASES = WORKSPACE_ROOT / "contracts/dummy_robot_v1/fixtures/fk_cases.json"
RANDOM_CASES = WORKSPACE_ROOT / "contracts/dummy_robot_v1/evidence/fk_random_cases.json"


##
# @brief 将三个空白分隔数字转为浮点元组；类型标注不自动校验输入元素数。
# @param text 空白分隔的数值文本。
# @return 浮点元组。
def vector(text: str) -> tuple[float, float, float]:
    return tuple(float(value) for value in text.split())


##
# @brief 用origin的xyz/rpy构造KDL Frame，缺属性默认零，单位m/rad。
# @param origin 参考链origin XML元素。
# @return KDL刚体Frame。
def frame(origin: element_tree.Element) -> PyKDL.Frame:
    xyz = vector(origin.attrib.get("xyz", "0 0 0"))
    rpy = vector(origin.attrib.get("rpy", "0 0 0"))
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


##
# @brief 按参考URDF中joint顺序构造KDL Chain：revolute用RotAxis，其余按Fixed，保留每段origin。
# @param path 参考链URDF路径，默认受控数学参考链。
# @return 按XML顺序装配的PyKDL.Chain；不做通用树拓扑排序。
def load_chain(path: Path = REFERENCE_CHAIN) -> PyKDL.Chain:
    root = element_tree.parse(path).getroot()
    chain = PyKDL.Chain()
    for joint in root.findall("joint"):
        origin = frame(joint.find("origin"))
        if joint.attrib["type"] == "revolute":
            axis = vector(joint.find("axis").attrib["xyz"])
            chain.addSegment(
                PyKDL.Segment(
                    joint.attrib["name"],
                    PyKDL.Joint(PyKDL.Vector(), PyKDL.Vector(*axis), PyKDL.Joint.RotAxis),
                    origin,
                )
            )
        else:
            chain.addSegment(PyKDL.Segment(joint.attrib["name"], PyKDL.Joint(PyKDL.Joint.Fixed), origin))
    return chain


##
# @brief 把关节rad写入六维JntArray并运行递归KDL FK，非零状态抛RuntimeError。
# @param chain 由参考URDF构建的六关节KDL链。
# @param joints 关节rad列表，调用方提供六元素。
# @return 位置三元素列表与行优先旋转矩阵九元素列表。
def solve(chain: PyKDL.Chain, joints: list[float]) -> tuple[list[float], list[float]]:
    values = PyKDL.JntArray(6)
    for index, value in enumerate(joints):
        values[index] = value
    result = PyKDL.Frame()
    status = PyKDL.ChainFkSolverPos_recursive(chain).JntToCart(values, result)
    if status != 0:
        raise RuntimeError(f"KDL FK failed: {status}")
    position = [result.p[index] for index in range(3)]
    rotation = [result.M[row, column] for row in range(3) for column in range(3)]
    return position, rotation


##
# @brief 参考链KDL与独立旧FK证据的正负例。
class ReferenceChainFkTest(unittest.TestCase):
    ##
    # @brief 临时将第3轴X偏置错误换成Z偏置，核其零位位置偏差大于50µm，拒绝照抄旧DH行。
    def test_literal_legacy_axis3_row_diverges_from_actual_fk(self):
        source = REFERENCE_CHAIN.read_text().replace(
            '<origin xyz="-0.052 0 0" rpy="1.5707963267948966 0 0"/>',
            '<origin xyz="0 0 0.052" rpy="1.5707963267948966 0 0"/>',
        )
        with tempfile.TemporaryDirectory() as temporary:
            wrong_path = Path(temporary) / "wrong_axis3.urdf"
            wrong_path.write_text(source)
            position, _ = solve(load_chain(wrong_path), [0.0] * 6)
        expected = json.loads(FIXED_CASES.read_text())["cases"][0]["position"]
        self.assertGreater(max(abs(a - b) for a, b in zip(position, expected)), 0.00005)

    ##
    # @brief 合并固定deg与随机rad证据，至少132例，逐例位置误差小于50µm、旋转矩阵最大元素差小于0.001。
    def test_fixed_and_random_evidence_match_kdl(self):
        self.assertTrue(REFERENCE_CHAIN.is_file(), "reference-chain URDF is missing")
        chain = load_chain()
        self.assertEqual(6, chain.getNrOfJoints())
        fixed = json.loads(FIXED_CASES.read_text())["cases"]
        random_cases = json.loads(RANDOM_CASES.read_text())["cases"]
        samples = [
            ([math.radians(value) for value in case["joints"]], case["position"], case["rotation"])
            for case in fixed
        ]
        samples.extend(
            (case["joints_rad"], case["legacy_position_m"], case["legacy_rotation"])
            for case in random_cases
        )
        self.assertGreaterEqual(len(samples), 132)
        for joints, expected_position, expected_rotation in samples:
            position, rotation = solve(chain, joints)
            self.assertLess(max(abs(a - b) for a, b in zip(position, expected_position)), 0.00005)
            self.assertLess(max(abs(a - b) for a, b in zip(rotation, expected_rotation)), 0.001)


if __name__ == "__main__":
    unittest.main()
