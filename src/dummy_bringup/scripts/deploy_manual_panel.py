#!/usr/bin/env python3
"""离线部署明确选定的Native/固件/身份；原子切换固定current入口，不搜索构建目录。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid

from manual_panel_config import deployment_root, installed_description, load_panel_config
from render_manual_panel_config import render_config


def resources():
    source = Path(__file__).resolve().parents[3]
    if (source/'contracts/dummy_robot_v2/codec/wire_layout.json').is_file():
        return source, Path(__file__).resolve().parents[1]/'config/manual_joint_panel.json'
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('dummy_bringup'))
    return share/'panel_runtime', share/'config/manual_joint_panel.json'


def deploy(native, firmware, identity, description):
    native=Path(native).resolve(strict=True);firmware=Path(firmware).resolve(strict=True)
    if not os.access(native,os.X_OK):
        raise ValueError('Native不是可执行文件')
    ids=json.loads(Path(identity).read_text())
    source,template=resources()
    root=deployment_root();releases=root/'releases';releases.mkdir(parents=True,exist_ok=True)
    release=Path(tempfile.mkdtemp(prefix='release-',dir=releases))
    try:
        shutil.copy2(native,release/'dummy_joint_cli')
        shutil.copy2(firmware,release/'firmware.bin')
        for relative in ('scripts/dummy/join_real_6dof_evidence.py',
                         'contracts/dummy_robot_v2/codec/wire_layout.json',
                         'contracts/dummy_robot_v2_legacy_2_4/codec/wire_layout.json'):
            target=release/'ros'/relative;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source/relative,target)
        binding={key:ids[key] for key in ('contract_sha256','model_sha256','profile_sha256')}
        for role,filename in (('native','dummy_joint_cli'),('firmware','firmware.bin')):
            binding[role+'_path']=str(release/filename)
            binding[role+'_sha256']=hashlib.sha256((release/filename).read_bytes()).hexdigest()
        binding['decoder_directory']=str(release/'ros/scripts/dummy')
        config=render_config(json.loads(template.read_text()),binding,Path(description))
        config['deployment_manifest']='deployment.json'
        config_path=release/'panel-config.json';config_path.write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
        files={str(p.relative_to(release)):hashlib.sha256(p.read_bytes()).hexdigest()
               for p in release.rglob('*') if p.is_file()}
        manifest=dict(format='dummy_manual_panel_deployment_v1',identity=binding,files=files)
        (release/'deployment.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
        load_panel_config(config_path,'real')
        link=root/('.current-'+uuid.uuid4().hex)
        link.symlink_to(release.relative_to(root),target_is_directory=True)
        os.replace(link,root/'current')
        return root/'current/panel-config.json'
    except Exception:
        shutil.rmtree(release)
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('native','firmware','identity'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--description-xacro',type=Path)
    args=parser.parse_args()
    try:
        output=deploy(args.native,args.firmware,args.identity,args.description_xacro or installed_description())
    except (OSError,ValueError,KeyError) as error:
        parser.exit(2,'部署失败：'+str(error)+'\n')
    print('已部署：'+str(output))
    print('启动：ros2 launch dummy_bringup manual_joint_panel.launch.py backend:=real')


if __name__=='__main__':
    main()
