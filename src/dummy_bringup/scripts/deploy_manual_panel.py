#!/usr/bin/env python3
##
# @file deploy_manual_panel.py
# @brief 离线封装明确选定的面板运行产物并切换 current。
# 部署把 Native、固件、解码器与线协议布局放进单独 release；身份 JSON 提供契约、模型、profile 的 SHA-256。这里只复制和校验本机文件，不烧录固件、不访问 USB。
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


##
# @brief 区分源码树与安装树，定位部署所需资源。
# 源码模式由已知 wire_layout.json 判定；安装模式从 ament 包共享目录的 panel_runtime 取资源，不搜索最新构建。
# @return tuple[Path, Path]：运行资源根目录和面板 JSON 模板。
def resources():
    source = Path(__file__).resolve().parents[3]
    if (source/'contracts/dummy_robot_v2/codec/wire_layout.json').is_file():
        return source, Path(__file__).resolve().parents[1]/'config/manual_joint_panel.json'
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('dummy_bringup'))
    return share/'panel_runtime', share/'config/manual_joint_panel.json'


##
# @brief 生成独立 release 并原子更新固定配置入口。
# 先复制产物并计算实际文件哈希，再渲染绑定配置及逐文件清单；校验通过才用 os.replace 切换符号链接。失败删除本次 release 并重抛异常；旧 release 不在此回收。
# @note 身份 JSON 中的模型/契约/profile 哈希由提供方给出；离线部署不证明在线设备运行了这些身份。
# @param native 明确选定的 Native 可执行文件路径。
# @param firmware 明确选定的固件二进制路径；只打包，不烧写。
# @param identity 包含 contract_sha256、model_sha256、profile_sha256 的 JSON 文件路径。
# @param description 供渲染器严格解析的机器人 xacro 路径。
# @return Path：current/panel-config.json，指向通过离线配置校验的 release。
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
        ##
        # 清单记录当时 release 内文件的 SHA-256，包含面板配置；deployment.json 尚未写入，因此不会自我散列。
        files={str(p.relative_to(release)):hashlib.sha256(p.read_bytes()).hexdigest()
               for p in release.rglob('*') if p.is_file()}
        manifest=dict(format='dummy_manual_panel_deployment_v1',identity=binding,files=files)
        (release/'deployment.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
        load_panel_config(config_path,'real')
        ##
        # 先建立唯一临时链接，再原子替换 current；运行中的调用方应仍按自身会话身份核对设备，不能把链接切换视为控制授权。
        link=root/('.current-'+uuid.uuid4().hex)
        link.symlink_to(release.relative_to(root),target_is_directory=True)
        os.replace(link,root/'current')
        return root/'current/panel-config.json'
    except Exception:
        shutil.rmtree(release)
        raise


##
# @brief 解析部署命令行并输出固定启动方式。
# 未指定 description-xacro 时使用已安装描述。打印的真机 launch 仅是提示，本函数不执行该命令。
# @return 无；成功打印配置路径，预期输入错误通过 argparse 以退出码 2 终止。
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
