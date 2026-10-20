#!/usr/bin/env python3
##
# @file build_clearance_guards.py
# @brief 离线生成近端碰撞体的5mm外包络及URDF/SRDF片段。
#
# 读取展开URDF与毫米网格，写输出STL、源/产物原始SHA和两个XML片段；不自动改源URDF/SRDF，也不接触ROS/设备。
# 近端保护体只与DISTAL列出的远端对象保留碰撞检查，其余组合通过生成的SRDF片段禁用；保护范围不是全机器人任意两物体5mm间隙保证。
"""为近端逐零件碰撞体生成5mm外包络。依赖numpy/scipy/trimesh；不连接实机。

只产生网格、清单及URDF/SRDF片段，模型的视觉和运动学资源不变。
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
import trimesh

##
# @brief PROXIMAL决定哪些源碰撞体外扩；DISTAL决定这些guard仍与哪些远端link检查。
# 名称表是显式设计范围，新增link不会自动获得同等保护语义。
PROXIMAL=('base_link','link1_1_1','link2_1_1','step_link2_motor_1','step_link2_motor_2')
DISTAL=('link4_1_1','link5_1_1','step_link4_motor_1','step_link5_motor_1','j6_reducer_housing','link6_1_1')


##
# @brief 对每个连通网格件构造固定方向凸外包络。
# @param source 输入网格Path，坐标按mm解释。
# @param distance_mm 各支撑平面向外平移量mm，build当前传5.01。
# @return (合并Trimesh,最小扩张mm,分件数)；空网格或实际float32包络不足固定5mm抛ValueError。
# 先按连通件凸包而非整件跨空隙合并；162个icosphere方向求支撑，半空间交点再建凸面。
# 最终STL精度用float32顶点重建面平面，并核所有输入顶点到各面的最小余量；不是只相信请求的distance_mm。
def expand_parts(source, distance_mm):
    mesh=trimesh.load_mesh(source,process=True)
    results=[];minimum=float('inf')
    directions=trimesh.creation.icosphere(subdivisions=2).vertices
    for part in mesh.split(only_watertight=False):
        points=np.asarray(part.vertices)
        hull=ConvexHull(points)
        # 固定162个支撑方向形成外包络，避免浮点法线噪声把面数扩张数倍。
        normals=directions/np.linalg.norm(directions,axis=1)[:,None]
        support=(points@normals.T).max(axis=0)+distance_mm
        corners=HalfspaceIntersection(np.c_[normals,-support],points[hull.vertices].mean(axis=0)).intersections
        outer=ConvexHull(corners)
        faces=outer.simplices.copy()
        triangles=corners[faces]
        ##
        # @brief 用面法线与面中心到总体中心的方向点积检测朝内面，翻转索引保持外向法线。
        inward=np.einsum('ij,ij->i',np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),
                          triangles.mean(axis=1)-corners.mean(axis=0))<0
        faces[inward]=faces[inward][:,::-1]
        # STL使用float32；先按实际序列化精度重新检查每一个凸面的支撑距离。
        rounded=corners.astype(np.float32).astype(np.float64)
        planes=ConvexHull(rounded).equations
        clearance=(-planes[:,3]-(points@planes[:,:3].T).max(axis=0)).min()
        minimum=min(minimum,float(clearance))
        results.append(trimesh.Trimesh(vertices=rounded,faces=faces,process=False))
    if not results or minimum<5.0:raise ValueError(f'保护体外扩不足5mm：{source} / {minimum}')
    return trimesh.util.concatenate(results),minimum,len(results)


##
# @brief 生成五个近端guard与受限碰撞对清单。
# @param model_path 当前已展开URDF文件；每个指定link读取首个collision。
# @param mesh_dir 输入STL所在目录，按URI末尾文件名拼接。
# @param output 输出目录，创建并覆盖同名产物；本函数不检查与输入目录隔离。
# @return 生成manifest映射；缺碰撞体/不支持形状、包络不足或I/O失败抛异常。
# 网格保留原collision origin/scale；盒体size以m解释，三维总长各增加0.01002m，即每侧5.01mm。
# 新guard以fixed零变换挂在原parent，SRDF禁用guard与自身/非DISTAL对象；片段必须由调用方显式集成才生效。
def build(model_path,mesh_dir,output):
    model=ET.parse(model_path).getroot()
    output.mkdir(parents=True,exist_ok=True)
    manifest=dict(minimum_clearance_mm=5.,generation_offset_mm=5.01,
                  proximal=list(PROXIMAL),distal=list(DISTAL),meshes=[])
    fragments=[]
    for parent in PROXIMAL:
        collision=model.find(f"link[@name='{parent}']/collision")
        if collision is None:raise ValueError('缺少原碰撞体：'+parent)
        node=ET.fromstring(ET.tostring(collision))
        mesh=node.find('geometry/mesh')
        if mesh is not None:
            name=mesh.get('filename').rsplit('/',1)[-1]
            source=mesh_dir/name
            result,minimum,parts=expand_parts(source,5.01)
            filename=source.stem+'-clearance-5mm.stl';target=output/filename
            result.export(target)
            manifest['meshes'].append(dict(source=name,output=filename,parts=parts,
                minimum_expansion_after_stl_roundtrip_mm=minimum,
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                output_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),triangles=len(result.faces)))
            mesh.set('filename','package://dummy-ros2_description/meshes/'+filename)
        else:
            box=node.find('geometry/box')
            if box is None:raise ValueError('仅支持现有网格和电机盒体：'+parent)
            box.set('size',' '.join(f'{float(v)+.01002:.16g}' for v in box.get('size').split()))
        name='clearance_'+parent
        link=ET.Element('link',name=name);link.append(node);fragments.append(link)
        joint=ET.Element('joint',name=name+'_fixed',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz='0 0 0',rpy='0 0 0');fragments.append(joint)
    (output/'clearance-guards.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (output/'clearance-guards.urdf.fragment').write_text('\n'.join(ET.tostring(n,encoding='unicode') for n in fragments)+'\n')
    all_links={n.get('name') for n in model.findall('link')}|{'clearance_'+p for p in PROXIMAL}
    pairs=set()
    for parent in PROXIMAL:
        guard='clearance_'+parent
        for other in all_links-{guard}-set(DISTAL):pairs.add(tuple(sorted((guard,other))))
    (output/'clearance-guards.srdf.fragment').write_text('\n'.join(
        ET.tostring(ET.Element('disable_collisions',link1=a,link2=b,reason='ClearanceGuardScope'),encoding='unicode')
        for a,b in sorted(pairs))+'\n')
    return manifest


##
# @brief 解析三个显式目录/文件参数并生成候选产物。
# @return 无显式返回值；成功打印manifest，参数/生成异常由argparse或调用栈报告。
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True,help='已展开的当前URDF')
    parser.add_argument('--meshes',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build(args.model,args.meshes,args.output),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
