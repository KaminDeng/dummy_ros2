"""以只监听回环的官方router维持Host会话，验证独立peer的真实Boot重建。"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time


def main():
    """启动一次router和完整业务actor；收尾只回收本次创建的两个进程。"""
    router, actor = map(Path, sys.argv[1:3])
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0))
        port = reservation.getsockname()[1]
    endpoint = f'tcp/127.0.0.1:{port}'
    config = {'mode': 'router', 'listen': {'endpoints': [endpoint]},
              'scouting': {'multicast': {'enabled': False}, 'gossip': {'enabled': False}},
              'adminspace': {'enabled': False}, 'plugins': {},
              # 固定单次发送边界；默认多transport帧批处理的兼容RED独立留存。
              'transport': {'link': {'tx': {'queue': {'batching': {'enabled': False}}}}}}
    with tempfile.TemporaryDirectory(prefix='r11-tcp-boot-') as directory:
        path = Path(directory)
        (path/'router.json').write_text(json.dumps(config))
        with (path/'router.log').open('w+') as log:
            process = subprocess.Popen([str(router), '-c', str(path/'router.json'),
                                        '--adminspace-permissions', 'none'],
                                       stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 5
                while True:
                    if process.poll() is not None:
                        raise RuntimeError('本次router提前退出')
                    try:
                        with socket.create_connection(('127.0.0.1', port), timeout=0.1):
                            break
                    except OSError:
                        if time.monotonic() >= deadline:
                            raise RuntimeError('本次回环router未就绪')
                        time.sleep(0.01)
                environment = dict(os.environ, DUMMY_R11_TCP_ROUTER=endpoint)
                child = subprocess.Popen([str(actor), 'protocol_r11_tcp_boot'], env=environment,
                                         start_new_session=True, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True)
                try:
                    output, _ = child.communicate(timeout=55)
                    print(output, end='', flush=True)
                    # 旧actor曾把assertion256截成exit0；显式终态也必须存在，不能只看低八位。
                    if child.returncode == 0 and not any(
                            line == 'TRANSPORT_COMPONENT_RESULT=PASS assertion=0'
                            for line in output.splitlines()):
                        print('缺少完整通过终态，拒绝仅凭进程退出码判定成功', file=sys.stderr)
                        return 1
                    return 0 if child.returncode == 0 else 1
                finally:
                    # 超时时连同actor创建的peer回收，不遗留串口/网络owner。
                    try:
                        os.killpg(child.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()
            finally:
                process.send_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                log.seek(0)
                print('ROUTER_CONFIG=' + json.dumps(config), flush=True)
                print(log.read(), flush=True)


if __name__ == '__main__':
    sys.exit(main())
