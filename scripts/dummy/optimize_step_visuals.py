#!/usr/bin/env python3
##
# @file optimize_step_visuals.py
# @brief 按材质组离线简化毫米OBJ视觉网格。
#
# 写候选OBJ、复制MTL并生成每组面数/采样误差和原始文件SHA报告；不修改URDF或碰撞体。
# 输出只禁止与源目录resolve完全相同，未禁止其成为源目录子目录；误差是有限双向采样，不是连续Hausdorff上界或机械间隙证明。
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


##
# @brief 读取三角OBJ顶点和材质面组。
# @param path 文本OBJ，顶点坐标按mm，面必须在usemtl后且恰3个顶点。
# @return (numpy顶点数组,材质名到零基面索引列表的dict)；形状不支持ValueError。
# 只处理v/usemtl/f；忽略法线/纹理等行，面引用只取斜杠前整数并减1。不是完整OBJ语法实现，不额外核任意负索引或越界引用。
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


##
# @brief 把三角Trimesh转换为VTK多边形数据。
# @param mesh 顶点/三角面数组对象，调用者保证形状。
# @return vtkPolyData；numpy_to_vtk使用deep拷贝，避免返回对象依赖临时numpy存储。
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


##
# @brief 测源所有顶点及三角中心到目标面的最大绝对距离。
# @param source 采样源网格，单位mm。
# @param target VTK隐式距离的目标网格。
# @return 有限采样最大距离mm；只单向，双向误差由调用者两次取max。
# 不检查连续三角面上所有点，也不提供Hausdorff严格上界；空采样可能使max报错。
def sample_error(source, target):
    distance = vtk.vtkImplicitPolyDataDistance()
    distance.SetInput(polydata(target))
    # 全部顶点与全部三角形中心；采样不等于连续 Hausdorff 上界。
    points = np.vstack((source.vertices, source.triangles_center))
    return max(abs(distance.EvaluateFunction(p)) for p in points)


##
# @brief 用VTK DecimatePro构造一个保拓扑候选。
# @param source 源三角网格mm。
# @param ratio 请求保留比例，实际SetTargetReduction为1−ratio；此函数不独立检查范围。
# @return process=False的Trimesh候选；不在此判定其是否满足调用者误差阈值。
# 绝对误差参数0.025mm、特征角40度、保边界顶点；这些是简化器输入，仍须sample_error独立核对。
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


##
# @brief 逐材质尝试简化并写单个OBJ/MTL候选。
# @param path 输入OBJ，要求同目录有同stem的MTL。
# @param output 已存在候选输出目录。
# @param max_error 双向采样误差上限mm。
# @return 文件名、源/输出原字节SHA及各材质统计dict；文件/几何错误传播。
# 面数>500从保留比例0.12尝试，每次乘1.8直到小于0.95；首个采样合格候选保留，否则使用原网格、误差记0。
# 重新串接每组顶点并累加offset生成1基面索引，保材质标签；只复制MTL，未额外复制其可能引用的纹理文件。
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


##
# @brief 校验输出及误差阈值，批处理full_step彩色OBJ。
# @return 无显式返回值；缺文件/相同目录/阈值不在(0,1]mm由parser.error拒绝，其它异常传播。
# 默认阈值0.15mm，写visual-mesh-verification.json并打印总面数；没有修改碰撞模型/运动学或实机资格。
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
