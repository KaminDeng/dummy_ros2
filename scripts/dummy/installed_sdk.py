"""显式安装SDK中的当前机器人合同；不搜索构建目录。"""
import os
from pathlib import Path


def robot_bundle():
    prefix = os.environ.get('DPSDK_HOST_SDK_PREFIX')
    if not prefix:
        raise ValueError('请设置DPSDK_HOST_SDK_PREFIX选择安装SDK')
    root = Path(prefix) / 'share/dummy_ref/robot_contract'
    if not root.is_absolute() or not (root / 'manifest.json').is_file():
        raise ValueError('当前机器人合同必须来自明确的完整安装目录')
    return root
