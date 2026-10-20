#!/usr/bin/env python3
##
# @file render_manual_panel_config.py
# @brief 在离线环境校验产物身份并生成部署配置。
# 渲染器使用模板的深拷贝，保留调用方模板内容；将显式部署文件路径及固件/契约/模型/profile 身份填入 Native 参数。这里只读写本机文件，不连接 ROS、USB 或电机。
"""离线绑定面板配置与真实部署产物；不连接ROS或设备。"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path


##
# @brief 验证部署输入，返回可供真机加载的独立配置。
# Native 与固件文件逐个比对给定 SHA-256；模型描述仅检查文件存在，模型身份来自 deployment.model_sha256。参数旗标必须各出现恰好一次且后面有值。
# @exception ValueError 缺字段、哈希格式/内容、路径、参数或超时不满足要求。
# @param template 面板配置模板字典；native_arguments 必须是字符串列表。
# @param deployment 明确的产物路径、各身份 SHA-256 与 decoder_directory。
# @param description_xacro 已选模型描述的 Path，必须实际存在且为文件。
# @return dict：模板深拷贝，绑定绝对路径、身份参数和有限的超时秒数。
def render_config(template: dict, deployment: dict, description_xacro: Path) -> dict:
    ##
    # @par 内部辅助函数 render_config.required
    # 取一个必需部署字段。
    # 参数 key：deployment 中必须存在的键名。
    # 返回：字段原值；不存在时抛出 ValueError。
    def required(key):
        if key not in deployment:
            raise ValueError('缺少部署字段: ' + key)
        return deployment[key]

    ##
    # @par 内部辅助函数 render_config.identity
    # 读取并规范化一个 SHA-256 十六进制身份。
    # 这里只校验文本格式；文件内容身份另由外层产物校验负责。
    # 参数 key：必需身份字段名。
    # 返回：str：64 位小写十六进制字符串。
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
    # 当前公开trial预算660s，包含Native规划/收敛/完成与清理；不是延长设备许可。
    ##
    # timeout 单位为秒：范围 (0,660]；该宿主等待预算不延长设备许可，bool 虽是 Python int 子类也明确拒绝。
    timeout = template.get('moveit_segment_timeout_s', 660.)
    if (isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or
            not math.isfinite(timeout) or not 0. < timeout <= 660.):
        raise ValueError('moveit_segment_timeout_s必须为(0,660]内的有限秒数')
    result = copy.deepcopy(template)
    result['moveit_segment_timeout_s'] = float(timeout)
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


##
# @brief 从 JSON 文件读取模板和部署信息并独占创建输出。
# 输出使用 x 模式，已有文件不会被覆盖；会创建所需父目录。命令行参数由 argparse 获取。
# @return int：成功返回 0；输入或写文件错误由 argparse 以退出码 2 终止。
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
