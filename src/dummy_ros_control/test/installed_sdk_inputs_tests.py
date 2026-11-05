#!/usr/bin/env python3
"""以真实安装前缀运行独立CMake消费；篡改仅发生在临时完整副本。"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

parser=argparse.ArgumentParser()
parser.add_argument('--prefix',type=Path,required=True)
parser.add_argument('--lock-file',type=Path,required=True)
args=parser.parse_args()
with tempfile.TemporaryDirectory(prefix='dummy-installed-sdk-') as tmp:
    root=Path(tmp)
    project=root/'project';project.mkdir()
    (project/'CMakeLists.txt').write_text(
        'cmake_minimum_required(VERSION 3.19)\nproject(installed_sdk_probe NONE)\n'
        f'include("{args.lock_file}")\n')
    def configure(prefix,case,expected=None):
        result=subprocess.run(['cmake','-S',str(project),'-B',str(root/('build-'+case)),
            '-DDPSDK_ROOT=/unavailable/dpsdk-source',f'-DDPSDK_HOST_SDK_PREFIX={prefix}'],capture_output=True,text=True)
        output=result.stdout+result.stderr
        if expected is None:
            assert result.returncode==0,output
        else:
            assert result.returncode!=0 and expected in output,(case,output)
    configure(args.prefix,'positive')
    cases=[
      ('codec','lib/cmake/DPSDKHostSDK/DPSDKHostSDKInputs.cmake','flatbuffers','raw-fixed','协议'),
      ('abi','lib/cmake/DPSDKHostSDK/DPSDKHostSDKInputs.cmake','RUNTIME_MESSAGE_BYTES 1536','RUNTIME_MESSAGE_BYTES 651','角色'),
      ('storage','lib/cmake/DPSDKHostSDK/DPSDKHostSDKRuntime.cmake','STORAGE_BYTES 19456','STORAGE_BYTES 18000','角色'),
      ('root_deployment','include/dummy_root_deployment.hpp',None,None,'Root部署生成头'),
      ('runtime','lib/libdummy_ref_ros_pic_runtime.a',None,None,'安装归档身份'),
      ('native_runtime','lib/libdummy_ref_native_runtime.a',None,None,'安装归档身份'),
      ('provider','lib/libdp_host_instance_identity.a',None,None,'安装归档身份'),
      ('typed_header','include/dp_comm/typed_runtime.hpp',None,None,'公共头身份'),
      ('host_slots','include/dp_protocol/host_default_binding_slots.hpp',None,None,'公共头身份'),
      ('profile','lib/cmake/DPSDKHostSDK/DPSDKHostSDKRuntime.cmake','PROFILE dummy','PROFILE mcu','角色'),
      ('log_shim','lib/libdummy_ref_ros_pic_log_shim.a',None,None,'安装归档身份'),
      ('header','include/dp_comm/dp_comm.hpp',None,None,'公共头身份'),
      ('policy','include/dummy_ref/wire_validation.hpp',None,None,'业务验证头身份'),
      ('protocol','lib/dp_protocol/include/dummy_joint_state_bounded.h',None,None,'协议'),
      ('selection','lib/dp_protocol/include/protocol_selection.h',None,None,'协议'),
    ]
    for case,path,old,new,expected in cases:
        prefix=root/case;shutil.copytree(args.prefix,prefix)
        changed=prefix/path
        if old is None: changed.write_bytes(changed.read_bytes()+b'\n/* tampered */\n')
        else: changed.write_text(changed.read_text().replace(old,new))
        configure(prefix,case,expected)
    missing=root/'missing';shutil.copytree(args.prefix,missing)
    shutil.rmtree(missing/'lib/dp_protocol')
    configure(missing,'missing','DpProtocol')
print('真实安装SDK无源码配置通过；错Codec/ABI、缺包与六类篡改全部被拒绝')
