#!/usr/bin/env python3
##
# @file manual_panel_config.py
# @brief 解析面板配置、固定部署位置和部署随附的解码器。
# 主面板启动和切换后端时调用；真机读取明确部署的 current 配置，模拟模式可读取仓库模板。这里仅校验和加载本机文件，不自动挑选 SDK 构建目录，也不建立设备会话。
"""固定部署入口与模拟器模板解析；绝不搜索最新SDK构建目录。"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path


##
# @brief 确定当前用户的面板部署根目录。
# 目录定位本身不会创建目录；部署脚本在此创建 releases 和 current 符号链接。
# @return Path：XDG_DATA_HOME 下的 dummy_ros/manual_panel；未设置时使用 ~/.local/share。
def deployment_root():
    return Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share'))) / 'dummy_ros/manual_panel'


##
# @brief 定位 ament 索引中已安装的机器人 xacro。
# 只构造路径；包不存在时由 ament 查询抛出异常，模型是否存在由后续配置加载校验。
# @return Path：dummy-ros2_description 包共享目录下的模型路径。
def installed_description():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('dummy-ros2_description')) / 'urdf/dummy-ros2.xacro'


##
# @brief 按显式参数和后端选择配置入口。
# @exception ValueError 后端名称不受支持；即使给定配置路径，也先检查后端。
# @param config_argument 可空的命令行配置路径；非空时展开用户主目录。
# @param backend 连接目标，仅支持 real 或 simulator。
# @param template_path 模拟模式默认使用的模板路径。
# @return Path：显式配置、模拟模板或固定 current/panel-config.json。
def resolve_config_path(config_argument, backend, template_path):
    if backend not in ('real', 'simulator'):
        raise ValueError('未知连接目标')
    if config_argument:
        return Path(config_argument).expanduser()
    if backend == 'simulator':
        return Path(template_path)
    return deployment_root()/'current/panel-config.json'


##
# @brief 读取配置并验证当前后端所需的部署文件。
# 真机拒绝未部署模板；存在 deployment_manifest 时逐文件比对 SHA-256，再检查 Native 可执行权限和解码入口。清单并非数字签名，不能代替固件在线身份确认。
# @exception ValueError 配置格式、部署完整性、后端或产物入口不满足条件。
# @note 模型路径使用 strict=True 解析，缺失文件的文件系统异常会继续交给调用方。
# @param config_path JSON 配置路径，允许 ~。
# @param backend real 执行部署校验；simulator 去掉模板中的部署占位标志。
# @param description_xacro 可选的模型覆盖路径；空值时先使用配置值，再查询已安装模型。
# @return dict：载入的配置，并补充绝对 description_xacro 与内部 _config_path。
def load_panel_config(config_path, backend, description_xacro=''):
    path = Path(config_path).expanduser()
    try:
        config = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        raise ValueError('未找到有效面板部署配置；请先运行deploy_manual_panel.py部署本机产物') from error
    if backend == 'real':
        if config.get('deployment_required'):
            raise ValueError('真机模式需要先运行deploy_manual_panel.py部署')
        if config.get('deployment_manifest'):
            try:
                manifest = json.loads((path.parent/config['deployment_manifest']).read_text())
                for relative, expected in manifest['files'].items():
                    artifact = path.parent/relative
                    if hashlib.sha256(artifact.read_bytes()).hexdigest() != expected:
                        raise ValueError(relative)
            except (OSError, KeyError, ValueError, TypeError) as error:
                raise ValueError('部署文件校验失败，请重新部署：' + str(error)) from error
        native = Path(config.get('native_binary', ''))
        decoder = Path(config.get('decoder_directory', ''))/'join_real_6dof_evidence.py'
        if not native.is_file() or not os.access(native, os.X_OK) or not decoder.is_file():
            raise ValueError('真机部署缺少可执行Native或解码器，请重新部署')
    elif backend == 'simulator':
        config.pop('deployment_required', None)
    else:
        raise ValueError('未知连接目标')
    ##
    # 模型覆盖只影响描述路径；_config_path 供后端切换重新定位本次配置，不代表设备当前在线身份。
    description = description_xacro or config.get('description_xacro') or str(installed_description())
    config['description_xacro'] = str(Path(description).resolve(strict=True))
    config['_config_path'] = str(path.resolve())
    return config


##
# @brief 动态导入当前部署指定的状态解码函数。
# 导入会执行该 Python 模块；应传入已选定并验证的部署配置。模块或入口缺失的异常由上层处理，函数不回退到其他目录。
# @param config 已载入的配置，必须包含 decoder_directory。
# @return callable：部署内 join_real_6dof_evidence.py 的 _decode 函数。
def load_decoder(config):
    path = Path(config['decoder_directory'])/'join_real_6dof_evidence.py'
    spec = importlib.util.spec_from_file_location('dummy_manual_panel_decoder', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._decode
