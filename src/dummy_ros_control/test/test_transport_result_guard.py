"""测试进程结果合同：大断言编号不能因 POSIX 低八位截断而变成成功。"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


COMPILER = 'c++'
HERE = Path(__file__).resolve().parent


class TransportResultGuardTests(unittest.TestCase):
    """只验证实际测试入口/runner，不把合成 actor 当成通信业务资格。"""

    def test_actual_main_preserves_failure_for_large_and_negative_codes(self):
        """直接编译仓库 main wrapper；替换业务分发器以穷举退出边界。"""
        source = (HERE / 'transport_component_tests.cpp').read_text()
        wrapper = 'int main(' + source.rsplit('\nint main(', 1)[1]
        prefix = ('#include <cstdio>\n#include <cstdlib>\n'
                  'static int run_transport_test_cases(int, char** argv) {\n'
                  'return int(std::strtol(argv[1], nullptr, 10)); }\n')
        with tempfile.TemporaryDirectory(prefix='dummy-exit-guard-') as directory:
            path = Path(directory)
            (path / 'main.cpp').write_text(prefix + wrapper)
            subprocess.run([COMPILER, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            str(path / 'main.cpp'), '-o', str(path / 'guard')], check=True)
            for code in (0, 1, 136, 255, 256, 512, -256):
                with self.subTest(code=code):
                    result = subprocess.run([str(path / 'guard'), str(code)],
                                            capture_output=True, text=True, check=False)
                    self.assertEqual(result.returncode, 0 if code == 0 else 1)
                    self.assertIn(f'assertion={code}', result.stderr)

    def test_runner_requires_explicit_pass_and_zero_exit(self):
        """用回环监听桩及合成 actor 检查 runner；无需安装或启动 Zenoh。"""
        with tempfile.TemporaryDirectory(prefix='dummy-runner-guard-') as directory:
            path = Path(directory)
            router = path / 'router.py'
            router.write_text(
                '#!/usr/bin/env python3\nimport json,socket,sys,time\n'
                'with open(sys.argv[2]) as stream: config=json.load(stream)\n'
                'port=int(config["listen"]["endpoints"][0].rsplit(":",1)[1])\n'
                'with socket.socket() as server:\n'
                ' server.bind(("127.0.0.1",port));server.listen();time.sleep(15)\n')
            router.chmod(0o755)
            for text, code, expected in (
                    ('component_only assertion failed: 256', 0, 1),
                    ('TRANSPORT_COMPONENT_RESULT=PASS assertion=0', 0, 0),
                    ('TRANSPORT_COMPONENT_RESULT=PASS assertion=0', 1, 1)):
                with self.subTest(text=text, code=code):
                    actor = path / 'actor.py'
                    actor.write_text('#!/usr/bin/env python3\n' +
                                     f'print({text!r})\nraise SystemExit({code})\n')
                    actor.chmod(0o755)
                    result = subprocess.run(
                        [sys.executable, str(HERE / 'run_root_tcp_boot_fixture.py'),
                         str(router), str(actor)], capture_output=True, text=True, timeout=10,
                        check=False)
                    self.assertEqual(result.returncode, expected, result.stdout + result.stderr)


if __name__ == '__main__':
    # 仅直接执行时消费可选编译器参数，unittest导入本模块不改全局argv。
    if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
        COMPILER = sys.argv.pop(1)
    unittest.main()
