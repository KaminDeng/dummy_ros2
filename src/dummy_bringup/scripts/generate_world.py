#!/usr/bin/python3
"""Generate Gazebo static geometry from the same scene used by MoveIt."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as element_tree

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
WORLD_TEMPLATE = WORKSPACE_ROOT / "src/dummy_moveit_config/worlds/world_template.sdf"


def render_world(scene_path: Path) -> str:
    scene = yaml.safe_load(scene_path.read_text())
    if scene["frame_id"] != "world":
        raise ValueError("Gazebo scene objects must use the world frame")
    root = element_tree.parse(WORLD_TEMPLATE).getroot()
    world = root.find("world")
    identifiers: set[str] = set()
    for item in scene["objects"]:
        identifier = item["id"]
        size, pose = item["size"], item["pose_xyz_rpy"]
        if identifier in identifiers or item["shape"] != "box":
            raise ValueError(f"duplicate or unsupported scene object: {identifier}")
        if len(size) != 3 or len(pose) != 6 or any(not math.isfinite(value) for value in (*size, *pose)):
            raise ValueError(f"invalid dimensions or pose: {identifier}")
        if any(value <= 0 for value in size):
            raise ValueError(f"non-positive dimensions: {identifier}")
        identifiers.add(identifier)
        model = element_tree.SubElement(world, "model", name=identifier)
        element_tree.SubElement(model, "static").text = "true"
        element_tree.SubElement(model, "pose").text = " ".join(map(str, pose))
        link = element_tree.SubElement(model, "link", name=f"{identifier}_link")
        for kind in ("collision", "visual"):
            geometry = element_tree.SubElement(element_tree.SubElement(link, kind, name=f"{identifier}_{kind}"), "geometry")
            element_tree.SubElement(element_tree.SubElement(geometry, "box"), "size").text = " ".join(map(str, size))
    element_tree.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + element_tree.tostring(root, encoding="unicode")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output.write_text(render_world(arguments.scene))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
