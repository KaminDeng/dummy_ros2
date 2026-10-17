#!/usr/bin/env python3
"""离线绑定面板配置与真实部署产物；不连接ROS或设备。"""
import argparse
import copy
import hashlib
import json
from pathlib import Path


def render_config(template: dict, deployment: dict, description_xacro: Path) -> dict:
    def required(key):
        if key not in deployment:
            raise ValueError('缺少部署字段: ' + key)
        return deployment[key]

    def identity(key):
        value = required(key)
        if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdefABCDEF' for c in value):
            raise ValueError('身份字段无效: ' + key)
        return value.lower()

    paths = {}
    for role in ('firmware', 'native'):
        try:
            path = Path(required(role + '_path')).resolve(strict=True)
        except (OSError, TypeError) as error:
            raise ValueError('产物路径无效: ' + role) from error
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != identity(role + '_sha256'):
            raise ValueError('产物身份不匹配: ' + role)
        paths[role] = path
    try:
        description = description_xacro.resolve(strict=True)
    except OSError as error:
        raise ValueError('模型描述不存在') from error
    if not description.is_file():
        raise ValueError('模型描述不是文件')
    try:
        decoder = Path(required('decoder_directory')).resolve(strict=True)
    except (OSError, TypeError) as error:
        raise ValueError('候选decoder目录不存在') from error
    if not decoder.is_dir() or not (decoder / 'join_real_6dof_evidence.py').is_file():
        raise ValueError('候选decoder目录缺少实际解码入口')
    result = copy.deepcopy(template)
    args = result.get('native_arguments')
    if not isinstance(args, list) or not all(isinstance(arg, str) for arg in args):
        raise ValueError('native_arguments必须是字符串数组')
    for flag, key in (('--firmware', 'firmware_sha256'), ('--contract', 'contract_sha256'),
                      ('--model', 'model_sha256'), ('--profile-sha256', 'profile_sha256')):
        if args.count(flag) != 1 or args.index(flag) + 1 >= len(args):
            raise ValueError('缺少或重复参数: ' + flag)
        args[args.index(flag) + 1] = identity(key)
    result['native_binary'] = str(paths['native'])
    result['description_xacro'] = str(description)
    result['decoder_directory'] = str(decoder)
    result.pop('deployment_required', None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('template', 'deployment', 'description-xacro', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    try:
        config = render_config(json.loads(args.template.read_text()),
                               json.loads(args.deployment.read_text()), args.description_xacro)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as output:
            output.write(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
    except (OSError, ValueError, TypeError) as error:
        parser.exit(2, '配置生成失败: ' + str(error) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
