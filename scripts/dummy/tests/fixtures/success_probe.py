#!/usr/bin/env python3
"""Minimal successful JSON probe fixture for adapter tests."""
##
# @file success_probe.py
# @brief 仅写固定observed=true的验收adapter测试脚本。
#
# 顶层即解析必填--report并写JSON；这是成功文件IO fixture，不是ROS Action/服务/设备探针。未定义main保护，导入也会解析进程参数/写文件；验收成功必须由调用方专属validator解释。

import argparse
import json
from pathlib import Path


parser = argparse.ArgumentParser()
parser.add_argument("--report", type=Path, required=True)
arguments = parser.parse_args()
arguments.report.write_text(json.dumps({"observed": True}) + "\n")
