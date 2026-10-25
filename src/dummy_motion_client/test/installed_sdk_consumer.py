#!/usr/bin/env python3
"""屏蔽两仓整个目录后，在独立安装空间配置、编译、运行并dlopen真实插件。"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

parser=argparse.ArgumentParser()
parser.add_argument('--sdk-prefix',type=Path,required=True)
parser.add_argument('--ros-prefix',type=Path,required=True)
parser.add_argument('--sdk-source',type=Path,required=True)
parser.add_argument('--ros-source',type=Path,required=True)
args=parser.parse_args()
with tempfile.TemporaryDirectory(prefix='dummy-sdk-consumer-') as temporary:
    root=Path(temporary)
    sdk=root/'sdk';ros=root/'ros'
    shutil.copytree(args.sdk_prefix,sdk)
    shutil.copytree(args.ros_prefix,ros,symlinks=False)
    project=root/'project';project.mkdir()
    shutil.copyfile(Path(__file__).with_suffix('.cpp'),project/'main.cpp')
    (project/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.19)
project(installed_ros_consumer LANGUAGES CXX)
find_package(dummy_motion_client CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE dummy_motion_client::dummy_dp_management)
''')
    environment=os.environ.copy()
    prefixes=[sdk,ros/'dummy_ros_control',ros/'dummy_motion_client',Path('/opt/ros/jazzy')]
    environment['CMAKE_PREFIX_PATH']=':'.join(map(str,prefixes))
    environment['AMENT_PREFIX_PATH']=':'.join(map(str,prefixes[1:]))
    environment['LD_LIBRARY_PATH']=str(ros/'dummy_ros_control/lib')+':/opt/ros/jazzy/lib'
    environment.pop('DPSDK_ROOT',None)
    wrapper=['bwrap','--ro-bind','/','/','--dev','/dev','--proc','/proc','--bind',str(root),str(root),
        '--tmpfs',str(args.sdk_source.resolve()),'--tmpfs',str(args.ros_source.resolve()),
        '--chdir',str(root),'--']
    def run(command):
        subprocess.run(wrapper+command,env=environment,check=True)
    # 生产包源码仍可见，但源码SDK及冻结contracts完全不可见；此分支只配置、不重复编译。
    configuration_wrapper=['bwrap','--ro-bind','/','/','--dev','/dev','--proc','/proc',
        '--bind',str(root),str(root),'--tmpfs',str(args.sdk_source.resolve()),
        '--tmpfs',str(args.ros_source.resolve()/'contracts'),'--chdir',str(root),'--']
    for package in ('dummy_ros_control','dummy_motion_client'):
        subprocess.run(configuration_wrapper+['cmake','-S',str(args.ros_source/'src'/package),
            '-B',str(root/('production-'+package)),'-DBUILD_TESTING=OFF',
            '-DDPSDK_ROOT=/unavailable/dpsdk-source',f'-DDPSDK_HOST_SDK_PREFIX={sdk}',
            '-DPython3_EXECUTABLE=/usr/bin/python3'],env=environment,check=True)
    run(['cmake','-S',str(project),'-B',str(root/'build'),'-G','Ninja',
        '-DPython3_EXECUTABLE=/usr/bin/python3'])
    run(['cmake','--build',str(root/'build')])
    run([str(root/'build/consumer')])
    library=ros/'dummy_ros_control/lib/libdummy_guarded_zenbedded_system.so'
    run(['/usr/bin/python3','-c','import ctypes,sys;ctypes.CDLL(sys.argv[1])',str(library)])
print('两仓源码目录已由私有mount屏蔽；独立安装头/管理库/PIC runtime编译运行及真实插件dlopen通过')
