#!/usr/bin/env python3
"""离线优化 STEP 彩色 OBJ；只输出视觉资源，不连接 ROS 或改动碰撞几何。

依赖 numpy、trimesh、vtk。输入为毫米，输出目录必须与源目录分离。
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
import trimesh
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy


def read_obj(path):
    vertices, groups, name = [], {}, None
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append([float(v) for v in fields[1:4]])
        elif fields[0] == 'usemtl':
            name = fields[1]
            groups.setdefault(name, [])
        elif fields[0] == 'f':
            if name is None or len(fields) != 4:
                raise ValueError('输入必须为每面具有材质组的三角 OBJ')
            groups[name].append([int(v.split('/')[0]) - 1 for v in fields[1:]])
    return np.array(vertices), groups


def polydata(mesh):
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(mesh.vertices, deep=True))
    cells = vtk.vtkCellArray()
    offsets = np.arange(len(mesh.faces) + 1, dtype=np.int64) * 3
    cells.SetData(numpy_to_vtkIdTypeArray(offsets, deep=True),
                  numpy_to_vtkIdTypeArray(mesh.faces.astype(np.int64).ravel(), deep=True))
    result = vtk.vtkPolyData()
    result.SetPoints(points)
    result.SetPolys(cells)
    return result


def sample_error(source, target):
    distance = vtk.vtkImplicitPolyDataDistance()
    distance.SetInput(polydata(target))
    # 全部顶点与全部三角形中心；采样不等于连续 Hausdorff 上界。
    points = np.vstack((source.vertices, source.triangles_center))
    return max(abs(distance.EvaluateFunction(p)) for p in points)


def simplify(source, ratio):
    decimator = vtk.vtkDecimatePro()
    decimator.SetInputData(polydata(source))
    decimator.SetTargetReduction(1 - ratio)
    decimator.SetErrorIsAbsolute(1)
    decimator.SetAbsoluteError(.025)
    decimator.PreserveTopologyOn()
    decimator.BoundaryVertexDeletionOff()
    decimator.SetFeatureAngle(40)
    decimator.Update()
    result = decimator.GetOutput()
    faces = vtk_to_numpy(result.GetPolys().GetConnectivityArray()).reshape((-1, 3))
    return trimesh.Trimesh(vertices=vtk_to_numpy(result.GetPoints().GetData()), faces=faces, process=False)


def optimize_file(path, output, max_error):
    vertices, groups = read_obj(path)
    lines, offset, stats = ['mtllib ' + path.with_suffix('.mtl').name], 0, []
    for material, faces in groups.items():
        source = trimesh.Trimesh(vertices=vertices, faces=np.array(faces), process=True)
        source.remove_unreferenced_vertices()
        final, measured_error, ratio = source, 0., .12
        if len(source.faces) > 500:
            while ratio < .95:
                candidate = simplify(source, ratio)
                measured = max(sample_error(source, candidate), sample_error(candidate, source))
                if measured <= max_error:
                    final, measured_error = candidate, measured
                    break
                ratio *= 1.8
        lines.append('usemtl ' + material)
        lines.extend('v ' + ' '.join(f'{v:.9g}' for v in row) for row in final.vertices)
        lines.extend('f ' + ' '.join(str(int(v) + 1 + offset) for v in row) for row in final.faces)
        offset += len(final.vertices)
        stats.append(dict(material=material, original_faces=len(source.faces), faces=len(final.faces),
                          max_bidirectional_sample_error_mm=measured_error))
    destination = output / path.name
    destination.write_text('\n'.join(lines) + '\n')
    shutil.copy2(path.with_suffix('.mtl'), output / path.with_suffix('.mtl').name)
    return dict(file=path.name, original_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                sha256=hashlib.sha256(destination.read_bytes()).hexdigest(), groups=stats)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='含 full_step_*-colors.obj/mtl 的源目录')
    parser.add_argument('output', type=Path, help='独立候选输出目录，不能是源目录')
    parser.add_argument('--max-error-mm', type=float, default=.15)
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve() or not 0 < args.max_error_mm <= 1:
        parser.error('输出必须与源目录分离，采样误差阈值必须在 (0,1] mm 内')
    paths = sorted(args.source.glob('full_step_*-colors.obj'))
    if not paths:
        parser.error('没有找到彩色视觉 OBJ')
    args.output.mkdir(parents=True, exist_ok=True)
    stats = [optimize_file(path, args.output, args.max_error_mm) for path in paths]
    (args.output / 'visual-mesh-verification.json').write_text(json.dumps(stats, indent=2) + '\n')
    print('视觉三角面：', sum(g['original_faces'] for s in stats for g in s['groups']),
          '->', sum(g['faces'] for s in stats for g in s['groups']))


if __name__ == '__main__':
    main()
