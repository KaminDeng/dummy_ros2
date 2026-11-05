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
    if (source/'scripts/dummy/selected_protocol.py').is_file():
        return source, Path(__file__).resolve().parents[1]/'config/manual_joint_panel.json'
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('dummy_bringup'))
    return share/'panel_runtime', share/'config/manual_joint_panel.json'


def selected_protocol_release(source):
    """只部署正式当前FBS发布；记录/网络能力分别由实际reader验证。"""
    prefix = os.environ.get('DPSDK_HOST_SDK_PREFIX')
    explicit = os.environ.get('DPSDK_PROTOCOL_RELEASE')
    local = source / 'scripts/dummy/protocol'
    if explicit:
        release = Path(explicit).resolve(strict=True)
    elif prefix:
        release = Path(prefix).resolve(strict=True) / 'lib/dp_protocol'
        if not (release / 'release_manifest.json').is_file():
            release = Path(prefix).resolve(strict=True) / 'lib/cmake/DpProtocol'
    else:
        release = local.resolve(strict=True)
    if prefix and explicit:
        selected = Path(prefix).resolve(strict=True)
        if selected not in release.parents:
            raise ValueError('协议发布与DPSDK_HOST_SDK_PREFIX混用')
    if (release / 'release_manifest.json').is_file():
        manifest = json.loads((release / 'release_manifest.json').read_text())
        if manifest.get('version') != 1 or not isinstance(manifest.get('files'), dict):
            raise ValueError('不支持的FlatBuffers发布清单')
        files = manifest['files']
        if 'python/dp_protocol_canonical_record_reader.py' not in files:
            raise ValueError('当前发布缺少正式canonical record reader')
        for relative, digest in files.items():
            name = Path(relative)
            target = release / name
            if (name.is_absolute() or '..' in name.parts or target.is_symlink()
                    or release not in target.resolve(strict=True).parents
                    or hashlib.sha256(target.read_bytes()).hexdigest() != digest):
                raise ValueError('协议发布载荷摘要或路径无效: ' + relative)
        validate_record_reader(release)
        return release, 'flatbuffers'
    raise ValueError('当前部署只支持正式FBS Root发布；历史raw-fixed须独立历史reader')


def validate_record_reader(release):
    """在部署前执行当前元数据/记录算法往返；presence false/0不能被清空。"""
    import importlib.util
    path = release / 'python/dp_protocol_canonical_record_reader.py'
    spec = importlib.util.spec_from_file_location('_deployment_canonical_reader', path)
    module = importlib.util.module_from_spec(spec)
    exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
    codec = module.SchemaCodec.from_directory(release)
    name = 'dp.protocol.dummy.JointState'
    record = codec.encode_record(name, {'state_seq': 0, 'hold_ready': False, 'joints': []})
    fields = codec.decode_record(name, record)
    if fields['state_seq'] != 0 or fields['hold_ready'] is not False or fields['joints'] != []:
        raise ValueError('当前canonical reader的presence往返失败')
    return {'canonical_record': 'DPC1', 'capture_container': 'DPRAW02',
            'native_state_journal': 'dummy.accepted-state.dpc1',
            'network_root_decoder': False}


##
# @brief 生成独立 release 并原子更新固定配置入口。
# 先复制产物并计算实际文件哈希，再渲染绑定配置及逐文件清单；校验通过才用 os.replace 切换符号链接。失败删除本次 release 并重抛异常；旧 release 不在此回收。
# @note 身份 JSON 中的模型/契约/profile 哈希由提供方给出；离线部署不证明在线设备运行了这些身份。
# @param native 明确选定的 Native 可执行文件路径。
# @param firmware 明确选定的固件二进制路径；只打包，不烧写。
# @param identity 包含 contract_sha256、model_sha256、profile_sha256 的 JSON 文件路径。
# @param description 供渲染器严格解析的机器人 xacro 路径。
# @param endpoint 可选的显式端点值；覆盖独立--endpoint参数后再固定配置摘要，不探测设备。
# @return Path：current/panel-config.json，指向通过离线配置校验的 release。
def deploy(native, firmware, identity, description, endpoint=None):
    if endpoint is not None and (not isinstance(endpoint,str) or not endpoint or
                                 endpoint.startswith('-') or
                                 any(c.isspace() or c == '\0' for c in endpoint)):
        raise ValueError('endpoint必须是非空端点值，不能使用选项格式或空白')
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
                         'scripts/dummy/selected_protocol.py',
                         'scripts/dummy/current_root_records.py',
                         'scripts/dummy/application_e2e.py',
                         'scripts/dummy/build_real_6dof_context.py'):
            target=release/'ros'/relative;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source/relative,target)
        bundled_protocol, codec = selected_protocol_release(source)
        shutil.copytree(bundled_protocol,release/'ros/scripts/dummy/protocol',
                        ignore=shutil.ignore_patterns('__pycache__','*.py[co]'))
        binding={key:ids[key] for key in ('contract_sha256','model_sha256','profile_sha256')}
        for role,filename in (('native','dummy_joint_cli'),('firmware','firmware.bin')):
            binding[role+'_path']=str(release/filename)
            binding[role+'_sha256']=hashlib.sha256((release/filename).read_bytes()).hexdigest()
        binding['decoder_directory']=str(release/'ros/scripts/dummy')
        config=render_config(json.loads(template.read_text()),binding,Path(description))
        if endpoint is not None:
            args=config['native_arguments']
            if args.count('--endpoint') != 1 or args.index('--endpoint')+1 >= len(args):
                raise ValueError('模板endpoint仅支持唯一的--endpoint独立参数和值')
            index=args.index('--endpoint')+1
            if args[index].startswith('--'):
                raise ValueError('模板endpoint缺少独立参数值')
            args[index]=endpoint
        config['deployment_manifest']='deployment.json'
        if codec == 'flatbuffers':
            # Native状态使用SDK语义观测；ROS使用同一正式SDK；历史raw-fixed证据不得误解Root。
            config['protocol_codec'] = codec
            config['evidence_decoder_supported'] = True
            config['decoder_capabilities'] = validate_record_reader(bundled_protocol)
            config['native_state_format'] = 'accepted-state-dpc1'
        config_path=release/'panel-config.json';config_path.write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
        ##
        # 清单记录当时 release 内文件的 SHA-256，包含面板配置；deployment.json 尚未写入，因此不会自我散列。
        files={str(p.relative_to(release)):hashlib.sha256(p.read_bytes()).hexdigest()
               for p in release.rglob('*') if p.is_file()}
        manifest=dict(format='dummy_manual_panel_deployment_v1',identity=binding,files=files,
                      protocol_codec=codec, evidence_decoder_supported=True,
                      decoder_capabilities=validate_record_reader(bundled_protocol))
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
    parser.add_argument('--endpoint',required=True,help='明确指定真机端点，不沿用模板示例；不连接或探测串口')
    args=parser.parse_args()
    try:
        output=deploy(args.native,args.firmware,args.identity,args.description_xacro or installed_description(),endpoint=args.endpoint)
    except (OSError,ValueError,KeyError) as error:
        parser.exit(2,'部署失败：'+str(error)+'\n')
    print('已部署：'+str(output))
    print('启动：ros2 launch dummy_bringup manual_joint_panel.launch.py backend:=real')


if __name__=='__main__':
    main()
