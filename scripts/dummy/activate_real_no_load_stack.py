#!/usr/bin/env python3
##
# @file activate_real_no_load_stack.py
# @brief 已禁用执行能力的NO_LOAD_DEV本地诊断入口。
#
# 此文件不导入ROS、不建立客户端、不改控制器状态；名称activate不表示实际提供激活。旧--execute一律拒绝，不能用它绕过设备身份/请求代次绑定。
"""UNQUALIFIED NO_LOAD_DEV 栈的本地只读诊断入口。"""

import argparse


##
# @brief 解析旧执行标志并输出本地诊断。
# @param argv 可选参数列表，None使用进程命令行。
# @param client_factory 保留的注入接口参数；当前实现完全不调用它。
# @return 普通诊断0；--execute打印UNCONFIRMED并返回1。
# 无论调用方是否传入工厂，都不连接ROS；NO_LOAD_DEV和UNQUALIFIED是输出声明，不产生现场许可。
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
