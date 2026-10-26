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
    source = sdk / 'lib/cmake/DpProtocol/sources/schemas/ros/dpsdk_interfaces'
    tools = args.protocol_tools.resolve(strict=True)
    if not (tools / 'dp_protocol_core/ros_package.py').is_file():
        parser.error('protocol-tools必须指向中央公共装配器目录')
    sys.path.insert(0, str(tools))
    from dp_protocol_core.ros_package import assemble_ros_package
    assemble_ros_package([source], args.output, exports=[
        'msg/DummyDeviceStatusV2.msg', 'msg/DummyJointStateV2.msg', 'msg/DummyProtocolV2.msg'])


if __name__ == '__main__':
    main()
