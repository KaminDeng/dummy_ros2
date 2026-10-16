#!/usr/bin/env python3
"""离线fixture的stderr满管道对照，不访问真实设备。"""
import fcntl
import json
import os
import signal
import subprocess
import sys
import time


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
