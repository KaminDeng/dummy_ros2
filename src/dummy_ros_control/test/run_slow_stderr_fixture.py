#!/usr/bin/env python3
"""离线fixture的stderr满管道对照，不访问真实设备。"""
##
# @file run_slow_stderr_fixture.py
# @brief 真实子进程stderr满管道背压对照包装器。
#
# 入口对同command正常/故意堵塞stderr各运行一次，stdout日志环境设RCUTILS_LOGGING_USE_STDOUT=1。会起新进程组，20s超时只杀该组并收输出；命令由调用方指定，包装器本身不保证任意命令无硬件动作。
import fcntl
import json
import os
import signal
import subprocess
import sys
import time


##
# @brief 建管道；blocked先非阻塞灌满后恢复阻塞、保读fd不消费，再Popen对照；超时SIGKILL自有组，finally关两fd。
# @param command 要实际Popen的argv；包装器不替调用方审查硬件副作用。
# @param blocked true人为灌满stderr管道且不消费，false常规捕获。
# @return 含blocked/timeout/时长/输出及非超时returncode的结果dict。
def run(command, blocked):
    read_fd, write_fd = os.pipe()
    try:
        flags = fcntl.fcntl(write_fd, fcntl.F_GETFL)
        if blocked:
            fcntl.fcntl(write_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            while True:
                try:
                    os.write(write_fd, b'x' * 4096)
                except BlockingIOError:
                    break
            fcntl.fcntl(write_fd, fcntl.F_SETFL, flags)
        environment = dict(os.environ, RCUTILS_LOGGING_USE_STDOUT='1')
        started = time.monotonic()
        process = subprocess.Popen(command, stderr=write_fd if blocked else subprocess.PIPE,
                                   stdout=subprocess.PIPE, env=environment, start_new_session=True)
        try:
            output, error = process.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, error = process.communicate()
            return {'blocked_sink': blocked, 'timeout': True, 'seconds': time.monotonic()-started,
                    'stdout': output.decode(errors='replace')}
        return {'blocked_sink': blocked, 'timeout': False, 'returncode': process.returncode,
                'seconds': time.monotonic()-started, 'stdout': output.decode(errors='replace'),
                'stderr': (error or b'').decode(errors='replace')}
    finally:
        os.close(read_fd)
        os.close(write_fd)


if __name__ == '__main__':
    outcomes = [run(sys.argv[1:], False), run(sys.argv[1:], True)]
    print(json.dumps(outcomes, ensure_ascii=False, indent=2))
    sys.exit(0 if all(not x['timeout'] and x['returncode'] == 0 for x in outcomes) else 1)
