#!/usr/bin/env python3
"""在无既有SDK符号的独立Python进程中加载插件，验证实际动态依赖闭合。"""
##
# @file independent_library_load.py
# @brief 独立Python进程的真实ctypes动态库加载入口。
#
# 顶层ctypes.CDLL直接加载argv[1]并触发库依赖/初始化；不启动ROS节点或连接设备，但不是纯文本检查。用于新进程未预置SDK符号时检动态依赖，成功加载不等于插件完整生命周期验证。
import ctypes
import sys

ctypes.CDLL(sys.argv[1])
