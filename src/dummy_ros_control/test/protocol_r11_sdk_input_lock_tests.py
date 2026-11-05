#!/usr/bin/env python3
"""用真实旧安装前缀验证生产输入锁拒绝R11前SDK；不修改前缀。"""
import argparse
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--legacy-prefix', required=True, type=Path)
parser.add_argument('--lock-file', required=True, type=Path)
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix='dummy-r11-sdk-red-') as directory:
    project = Path(directory)
    (project / 'CMakeLists.txt').write_text(
        'cmake_minimum_required(VERSION 3.19)\nproject(r11_legacy_sdk_rejection NONE)\n'
        f'include("{args.lock_file.resolve()}")\n')
    command = ['cmake', '-S', str(project), '-B', str(project / 'build'),
               f'-DDPSDK_HOST_SDK_PREFIX={args.legacy_prefix.resolve()}',
               '-DDPSDK_ROOT=/unavailable-source']
    result = subprocess.run(command, capture_output=True, text=True)
    print(result.stdout + result.stderr)
    assert result.returncode != 0, 'R11生产输入锁错误接纳旧raw-fixed/651/18000安装SDK'
print('R11生产输入锁已拒绝旧SDK')
