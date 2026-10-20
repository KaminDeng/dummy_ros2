#!/usr/bin/env python3
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

PROXIMAL=('base_link','link1_1_1','link2_1_1','step_link2_motor_1','step_link2_motor_2')
DISTAL=('link4_1_1','link5_1_1','step_link4_motor_1','step_link5_motor_1','j6_reducer_housing','link6_1_1')


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


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True,help='已展开的当前URDF')
    parser.add_argument('--meshes',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build(args.model,args.meshes,args.output),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
