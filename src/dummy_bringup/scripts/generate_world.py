#!/usr/bin/python3
##
# @file generate_world.py
# @brief 将公共 YAML 场景转为 Gazebo SDF 静态箱体。
# 与 MoveIt 场景共享对象配置；这里只在本机解析 YAML/XML 并生成字符串或文件，不启动 Gazebo、不发布场景。几何尺寸和位置采用米，RPY 采用弧度。
"""Generate Gazebo static geometry from the same scene used by MoveIt."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as element_tree

import yaml


##
# 模板由当前脚本向上三级定位源码根目录；本模块没有 ament 安装路径回退。
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
WORLD_TEMPLATE = WORKSPACE_ROOT / "src/dummy_moveit_config/worlds/world_template.sdf"


##
# @brief 在世界模板上添加唯一且合法的静态箱体。
# 要求每对象 size 为三个有限正数、pose_xyz_rpy 为六个有限数、shape 为 box 且 ID 唯一。collision 与 visual 使用相同箱体尺寸。模板路径按源码树固定定位。
# @exception ValueError 坐标系、对象类型、ID 或数值尺寸不符合要求。
# @param scene_path 公共 scene.yaml 的 Path，frame_id 必须为 world。
# @return str：带 XML 声明的完整 SDF 世界文本。
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


##
# @brief 解析场景及输出路径并写入生成的 SDF。
# 要求 --scene 与 --output；不会创建输出父目录，现有输出文件会被 write_text 覆盖。
# @return int：写入成功返回 0；读取、解析与写入错误继续抛给调用者。
def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output.write_text(render_world(arguments.scene))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
