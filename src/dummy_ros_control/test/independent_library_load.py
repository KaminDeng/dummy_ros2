#!/usr/bin/env python3
"""在无既有SDK符号的独立Python进程中加载插件，验证实际动态依赖闭合。"""
import ctypes
import sys

ctypes.CDLL(sys.argv[1])
