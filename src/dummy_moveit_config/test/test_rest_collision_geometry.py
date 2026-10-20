##
# @file test_rest_collision_geometry.py
# @brief 回归REST让位空间和J1凹支架分区外包络的完整覆盖。
# @details 使用FCL与最终STL几何，拒绝用SRDF豁免掩盖误碰撞；这些离线局部对检查不替代完整MoveIt路径回归。

"""REST 必须保留进出空间；旋转底座的保守几何不能填实支架凹部。"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import fcl
import numpy as np
import trimesh
import xacro

from test_j6_reducer import frames, origin

ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT / 'src/dummy-ros2_description'


##
# @brief 把指定link各collision转为FCL BVH或box，结合关节链和collision origin得到base坐标物体。
# @param model 展开的生产URDF XML根。
# @param joint_positions Joint名→rad映射。
# @param name 需要建立FCL碰撞体的link名。
# @return FCL CollisionObject列表；这里只实现mesh/box，不是通用URDF形状解释器。
def collision_objects(model, joint_positions, name):
    transforms = frames(model, joint_positions)
    result = []
    for collision in model.find(f"link[@name='{name}']").findall('collision'):
        geometry = collision.find('geometry')
        mesh = geometry.find('mesh')
        if mesh is not None:
            source = DESCRIPTION / 'meshes' / mesh.get('filename').rsplit('/', 1)[-1]
            data = trimesh.load_mesh(source, process=False)
            vertices = data.vertices * np.fromstring(mesh.get('scale'), sep=' ')
            shape = fcl.BVHModel()
            shape.beginModel(len(vertices), len(data.faces))
            shape.addSubModel(vertices, np.asarray(data.faces, dtype=np.int32))
            shape.endModel()
        else:
            shape = fcl.Box(*map(float, geometry.find('box').get('size').split()))
        transform = transforms[name] @ origin(collision)
        result.append(fcl.CollisionObject(shape, fcl.Transform(transform[:3, :3], transform[:3, 3])))
    return result


##
# @brief 保留J1/J2电机碰撞对，固定其余REST轴并从REST-J2再减0.5°扫到Home-J2，共122点检查该对无碰撞。
def test_rest_j2_mount_clearance_without_collision_exemption():
    """旧整件凸包在 REST 下与 J2 电机误碰撞；修复必须改几何而非允许碰撞矩阵。"""
    model = ET.fromstring(xacro.process_file(str(DESCRIPTION/'urdf/dummy-ros2.xacro')).toxml())
    cfg = json.loads((ROOT/'src/dummy_bringup/config/manual_joint_panel.json').read_text())
    srdf = ET.parse(ROOT/'src/dummy_moveit_config/config/dummy-ros2.srdf').getroot()
    allowed = {frozenset((p.get('link1'), p.get('link2'))) for p in srdf.findall('disable_collisions')}
    pair = ('link1_1_1', 'step_link2_motor_2')
    assert frozenset(pair) not in allowed
    # 同一对碰撞几何在 REST 及其到 Home 的每 0.5 度 J2 扫描中均须无接触。
    for j2 in np.linspace(cfg['rest_deg'][1]-.5, 0., 122):
        positions = dict(zip((j['name'] for j in cfg['joints']), map(math.radians, cfg['rest_deg'])))
        positions['Joint2'] = math.radians(j2)
        first, second = (collision_objects(model, positions, name) for name in pair)
        for a in first:
            for b in second:
                assert not fcl.collide(a, b, fcl.CollisionRequest(), fcl.CollisionResult()), j2


##
# @brief 核四资源SHA，按Z=24mm两侧裁剪源三角面，最终凸面至少外扩0.3mm包含；12紧固件仍被14包络覆盖。
def test_refined_envelopes_cover_every_clipped_source_triangle_and_fastener():
    """校核最终文件的真实凸面，不能只信生成清单的数值或缩小后无碰撞。"""
    import hashlib
    from scipy.spatial import ConvexHull
    folder = DESCRIPTION/'meshes'
    report = json.loads((folder/'rest-collision-refinement.json').read_text())
    for name, key in [('full_step_link1-bracket-source.stl', 'source_bracket_sha256'),
                      ('full_step_link1-fasteners-source.stl', 'source_fasteners_sha256'),
                      ('full_step_link1-collision-local.stl', 'output_sha256'),
                      ('full_step_link1-collision-local-clearance-5mm.stl', 'guard_sha256')]:
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest() == report[key]
    bracket = trimesh.load_mesh(folder/'full_step_link1-bracket-source.stl')
    envelopes = trimesh.load_mesh(folder/'full_step_link1-collision-local.stl').split(only_watertight=False)
    # ConvexHull面方程的法向量为单位长度，内部满足n·x+d<=0；源STL单位mm，
    # 因而<=-0.3直接表达至少0.3mm面内余量。每侧必须由同一个凸包包含全部裁剪顶点。
    planes = [ConvexHull(part.vertices).equations for part in envelopes]
    for sign in (-1., 1.):
        clipped = trimesh.intersections.slice_mesh_plane(bracket, [0., 0., sign], [0., 0., 24.], cap=False)
        # 裁剪会插入所有三角形与平面的交点。凸体包含其全部顶点即包含整个三角形，
        # 不仅验证原网格顶点，避免跨切面三角形中间留下未覆盖的缺口。
        assert any(np.max(clipped.vertices @ p[:,:3].T + p[:,3]) <= -.3 for p in planes)
    fasteners = trimesh.load_mesh(folder/'full_step_link1-fasteners-source.stl').split(only_watertight=False)
    assert len(fasteners) == 12 and len(envelopes) == 14
    for part in fasteners:
        assert any(np.max(part.vertices @ p[:,:3].T + p[:,3]) <= 1e-4 for p in planes)
