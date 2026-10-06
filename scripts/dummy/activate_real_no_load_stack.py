#!/usr/bin/env python3
"""UNQUALIFIED NO_LOAD_DEV 栈的本地只读诊断入口。"""

import argparse


def main(argv=None, client_factory=None):
    parser = argparse.ArgumentParser(description="UNQUALIFIED NO_LOAD_DEV 只读诊断")
    parser.add_argument("--execute", action="store_true", help="已禁用的旧执行选项")
    args = parser.parse_args(argv)
    if args.execute:
        print("UNCONFIRMED: --execute 已禁用；ROS service 超时请求可能晚完成，"
              "且此入口不能绑定 NO_LOAD_DEV activation/device 身份。"
              "必须由 gate 自持身份与请求代次后实现受控激活")
        return 1
    print("DRY_RUN UNQUALIFIED NO_LOAD_DEV: 本地只读；未连接 ROS，未变更状态。"
          "硬件与控制器激活需由绑定设备身份和请求代次的 gate 控制")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
