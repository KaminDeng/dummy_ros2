#!/usr/bin/env python3
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
FIXED_CASES = WORKSPACE_ROOT / "contracts/dummy_robot_v2/fixtures/fk_cases.json"
RANDOM_CASES = WORKSPACE_ROOT / "contracts/dummy_robot_v2/evidence/fk_random_cases.json"


def vector(text: str) -> tuple[float, float, float]:
    return tuple(float(value) for value in text.split())


def frame(origin: element_tree.Element) -> PyKDL.Frame:
    xyz = vector(origin.attrib.get("xyz", "0 0 0"))
    rpy = vector(origin.attrib.get("rpy", "0 0 0"))
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


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


class ReferenceChainFkTest(unittest.TestCase):
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
