##
# @file setup.py
# @brief ament_python描述资源包的setuptools安装配置；只装配/安装包，不启动ROS节点。

# Copyright 2026 Dummy ROS maintainers
# Licensed under the Apache License, Version 2.0

from glob import glob
import os

from setuptools import setup

##
# 包名用于ament索引、资源安装目录与Python包列表；版本/维护者等原发布元数据在setup参数中保留。
package_name = 'dummy-ros2_description'

##
# 声明构建/安装元数据，由colcon/setuptools调用；本文件不在import时连接设备或加载模型进程。
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    ##
    # 资源marker安装到ament包索引，其他资源安装到share目录；glob在执行安装配置时取当前文件，不代表运行时动态扫描。
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ##
        # 仅安装匹配*.launch.py的启动源文件，不将__pycache__作为launch入口。
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ##
        # URDF、mesh、RViz配置随包安装，使包内URI不依赖下载目录；不复制DPSDK固件或生成实机部署身份。
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='Apache-2.0',
    tests_require=['pytest'],
    ##
    # console_scripts列表为空；该资源包没有通过setuptools注册命令行节点，启动入口由launch文件提供。
    entry_points={
        'console_scripts': [
        ],
    },
)
