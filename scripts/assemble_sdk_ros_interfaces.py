#!/usr/bin/env python3
"""只从明确SDK携带的中央IDL装配；不在ROS仓库维护协议源副本。"""
import argparse
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-prefix', required=True, type=Path)
    parser.add_argument('--protocol-tools', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    sdk = args.sdk_prefix.resolve(strict=True)
    source = sdk / 'share/dummy_ref/ros_interfaces/dpsdk_interfaces'
    release = sdk / 'lib/dp_protocol'
    tools = args.protocol_tools.resolve(strict=True)
    if not (tools / 'dp_protocol_core/ros_package.py').is_file():
        parser.error('protocol-tools必须指向中央公共装配器目录')
    sys.path.insert(0, str(tools))
    from dp_protocol_core.flatbuffers_verify_release import verify_release
    import hashlib
    import json
    import re
    manifest = verify_release(release)
    if manifest['selection'] != {'protocol_set': 'dummy_managed', 'profile': 'dummy', 'codec': 'flatbuffers'}:
        parser.error('ROS装配只消费正式R11 dummy Root SDK')
    # 工具与SDK发布副本也核同一来源，不能用另一工作区的同名装配器。
    for name in ['ros_package.py', 'flatbuffers_verify_release.py']:
        actual = tools / 'dp_protocol_core' / name
        recorded = release / 'source/tools/dp_protocol_core' / name
        if hashlib.sha256(actual.read_bytes()).digest() != hashlib.sha256(recorded.read_bytes()).digest():
            parser.error('protocol-tools与SDK发布来源不一致')
    inputs = (sdk / 'lib/cmake/DPSDKHostSDK/DPSDKHostSDKInputs.cmake').read_text()
    for path in sorted((sdk / 'share/dummy_ref/ros_interfaces').rglob('*')):
        if not path.is_file():
            continue
        name = path.relative_to(sdk / 'share/dummy_ref/ros_interfaces').as_posix()
        key = re.sub('[^a-zA-Z0-9_]', '_', name)
        match = re.search(r'set\(DPSDKHostSDK_ROS_IDL_' + re.escape(key) + r'_SHA256 ([0-9a-f]{64})\)', inputs)
        if not match or hashlib.sha256(path.read_bytes()).hexdigest() != match.group(1):
            parser.error('SDK领域IDL来源不一致：' + name)
    from dp_protocol_core.ros_package import assemble_ros_package
    assemble_ros_package([source], args.output, exports=[
        'msg/DummyDeviceStatusV2.msg', 'msg/DummyJointStateV2.msg', 'msg/DummyProtocolV2.msg'])


if __name__ == '__main__':
    main()
