#!/usr/bin/env python3
##
# @file refine_j1_collision.py
# @brief 原位重建J1分件保守碰撞体并更新其5mm保护体。
#
# 与只写候选目录的视觉工具不同，build会覆盖meshes目录内两个STL与三份清单。
# 分解只保留REST安装让位的几何表达，所有包络/源SHA证明局限于所给网格；不认证真实机械间隙，不调用ROS/电机。
"""分解 J1 凹支架的保守碰撞包络，保留 REST 安装让位空间；不连接 ROS/电机。"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_clearance_guards import expand_parts


##
# @brief 计算网格/清单原字节SHA。
# @param path 输入文件Path。
# @return 小写SHA-256；文件异常传播。
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


##
# @brief 构造带padding的固定方向凸包并核float32后最小源余量。
# @param points 三维输入顶点数组，单位mm，须足以建立三维ConvexHull。
# @param padding_mm 支撑平面外移量mm，build使用0.301。
# @return (Trimesh,最小余量mm)；小于固定0.3mm拒绝，几何退化由scipy异常报告。
# 这里只核输入点到最终凸面半空间的包含余量；不做装配干涉或实机标定验证。
def outer_envelope(points, padding_mm):
    """以固定支撑方向生成外包络，并在 float32 序列化精度下验证包含全部输入点。"""
    hull = ConvexHull(points)
    normals = trimesh.creation.icosphere(subdivisions=2).vertices
    support = (points @ normals.T).max(axis=0) + padding_mm
    corners = HalfspaceIntersection(np.c_[normals, -support], points[hull.vertices].mean(axis=0)).intersections
    outer = ConvexHull(corners)
    faces = outer.simplices.copy()
    triangles = corners[faces]
    flip = np.einsum('ij,ij->i', np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0]),
                     triangles.mean(axis=1)-corners.mean(axis=0)) < 0
    faces[flip] = faces[flip][:, ::-1]
    rounded = corners.astype(np.float32).astype(np.float64)
    planes = ConvexHull(rounded).equations
    margin = float((-planes[:,3] - (points @ planes[:,:3].T).max(axis=0)).min())
    if margin < .3:
        raise ValueError(f'碰撞包络不足 0.3mm：{margin}')
    return trimesh.Trimesh(vertices=rounded, faces=faces, process=False), margin


##
# @brief 切分支架、合并紧固件并原位更新目标及清单。
# @param meshes 含两份source STL、clearance-guards.json及full_step_manifest.json的目录。
# @return 本次refinement报告dict；缺源/清单行、几何或写入失败传播，可能留下部分更新。
# z=24mm用两个反向平面几何裁剪，cap=false再各自外包凸包；不是仅丢弃跨面三角形。
# 紧固件不重建，沿原网格合并；再对最终碰撞体expand_parts(5.01mm)，更新guard行及link1三角数。
# source_step_sha256是写死的来源声明，不在此读取STEP重算；source_bracket/fasteners和输出SHA才来自当前实际文件。
def build(meshes):
    """仅重建旋转底座及其 5mm 保护体；紧固件沿用原包络，其余连杆完全不变。"""
    bracket_path = meshes/'full_step_link1-bracket-source.stl'
    fasteners_path = meshes/'full_step_link1-fasteners-source.stl'
    bracket = trimesh.load_mesh(bracket_path, process=True)
    # z=24mm 位于底部圆盘/加强结构与侧向安装板的过渡处。
    # 对每个穿过切面的三角形做几何裁剪，而非丢掉跨面的三角形或只按顶点归组。
    cut_mm = 24.
    envelopes, margins = [], []
    for sign in (-1., 1.):
        clipped = trimesh.intersections.slice_mesh_plane(
            bracket, [0., 0., sign], [0., 0., cut_mm], cap=False)
        envelope, margin = outer_envelope(clipped.vertices, .301)
        envelopes.append(envelope)
        margins.append(margin)
    fasteners = trimesh.load_mesh(fasteners_path, process=False)
    target = meshes/'full_step_link1-collision-local.stl'
    trimesh.util.concatenate([fasteners, *envelopes]).export(target)
    guard, minimum, parts = expand_parts(target, 5.01)
    guard_path = meshes/'full_step_link1-collision-local-clearance-5mm.stl'
    guard.export(guard_path)
    clearance_path = meshes/'clearance-guards.json'
    clearance = json.loads(clearance_path.read_text())
    row = next(row for row in clearance['meshes'] if row['source'] == target.name)
    row.update(parts=parts, minimum_expansion_after_stl_roundtrip_mm=minimum,
               source_sha256=digest(target), output_sha256=digest(guard_path), triangles=len(guard.faces))
    clearance_path.write_text(json.dumps(clearance, ensure_ascii=False, indent=2)+'\n')
    report = dict(source_step_sha256='7099178daac19d822df14ca0495921032a858feabcb8ffb8678c8b06c68b5682',
                  source_part='J1/Dummy-0001', units='mm', cut_axis='z', cut_mm=cut_mm,
                  padding_mm=.301, minimum_source_margin_mm=margins,
                  source_bracket_sha256=digest(bracket_path), source_fasteners_sha256=digest(fasteners_path),
                  output_sha256=digest(target), guard_sha256=digest(guard_path),
                  guard_minimum_expansion_mm=minimum, triangles=len(trimesh.load_mesh(target).faces))
    (meshes/'rest-collision-refinement.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    model_manifest_path = meshes/'full_step_manifest.json'
    model_manifest = json.loads(model_manifest_path.read_text())
    model_manifest['groups']['link1']['collision_triangles'] = report['triangles']
    model_manifest['collision_refinement_manifest'] = 'rest-collision-refinement.json'
    model_manifest['unresolved'] = 'No physical motion qualification; REST planning geometry resolved by rest-collision-refinement.json'
    model_manifest_path.write_text(json.dumps(model_manifest, ensure_ascii=False, indent=2)+'\n')
    return report


##
# @brief 显式选择mesh目录后调用原位重建。
# @return 无显式返回值；成功打印报告，错误传播；不会自动备份或回滚已写资源。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--meshes', type=Path, required=True, help='描述包 meshes 目录，原位更新明确的两个碰撞网格及清单')
    args = parser.parse_args()
    print(json.dumps(build(args.meshes), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
