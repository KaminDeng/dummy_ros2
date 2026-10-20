#!/usr/bin/env python3
"""固定部署入口与模拟器模板解析；绝不搜索最新SDK构建目录。"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path


def deployment_root():
    return Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share'))) / 'dummy_ros/manual_panel'


def installed_description():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('dummy-ros2_description')) / 'urdf/dummy-ros2.xacro'


def resolve_config_path(config_argument, backend, template_path):
    if backend not in ('real', 'simulator'):
        raise ValueError('未知连接目标')
    if config_argument:
        return Path(config_argument).expanduser()
    if backend == 'simulator':
        return Path(template_path)
    return deployment_root()/'current/panel-config.json'


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
    description = description_xacro or config.get('description_xacro') or str(installed_description())
    config['description_xacro'] = str(Path(description).resolve(strict=True))
    config['_config_path'] = str(path.resolve())
    return config


def load_decoder(config):
    path = Path(config['decoder_directory'])/'join_real_6dof_evidence.py'
    spec = importlib.util.spec_from_file_location('dummy_manual_panel_decoder', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._decode
