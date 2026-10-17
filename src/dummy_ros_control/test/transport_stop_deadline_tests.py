"""真实普通Stop/Query/Idle worker分支的精确截止边界回归（Debug软件角色）。

复用现有PTY component目标，GDB仅在真实fresh普通Idle到达后注入本次时钟采样。
不复制生产判定、不增加生产hook；各测试用独立进程组并回收自己的peer。
"""
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import unittest

BINARY = str(Path(sys.argv.pop(1)).resolve())
SOURCE = Path(sys.argv.pop(1)).resolve()


class StopDeadlineTests(unittest.TestCase):
    def check_boundary(self, delta):
        gdb = shutil.which("gdb")
        self.assertIsNotNone(gdb, "此Debug软件回归需要真实GDB，不能静默跳过")
        lines = SOURCE.read_text().splitlines()
        decision = next(i for i, line in enumerate(lines, 1)
                        if "if (stop_confirmation_pending.load" in line and
                        "fresh_released_idle" in lines[i])
        following = next(i for i, line in enumerate(lines, 1)
                         if "if (!gate_release.idle_confirmed && confirmed_gate_released_idle" in line)
        commands = f"""set pagination off
set confirm off
set breakpoint pending on
set follow-fork-mode parent
set $injected=0
break {SOURCE}:{decision} if fresh_released_idle && !stop_candidate_revalidated
commands
silent
set var stop_now=stop_deadline_us+{delta}
set $injected=1
printf "SUT_READY fresh=%d cached=%d now=%llu deadline=%llu\\n",fresh_released_idle,stop_candidate_revalidated,stop_now,stop_deadline_us
disable 1
continue
end
break {SOURCE}:{following}
commands
silent
if $injected
printf "SUT_RESULT confirmed=%llu unconfirmed=%llu\\n",stop_requests_confirmed._M_i,unconfirmed_stop_requests._M_i
quit
end
continue
end
run six_axis
"""
        with tempfile.TemporaryDirectory(prefix="stop-deadline-") as temporary:
            script = Path(temporary) / "boundary.gdb"
            script.write_text(commands)
            process = subprocess.Popen([gdb, "-batch", "-x", str(script), "--args", BINARY],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, start_new_session=True)
            try:
                output, _ = process.communicate(timeout=30)
            finally:
                # 只回收该回归新建的session，包括GDB分离的synthetic peer。
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        self.assertEqual(process.returncode, 0, output)
        ready = re.search(r"SUT_READY fresh=1 cached=0 now=(\d+) deadline=(\d+)", output)
        self.assertIsNotNone(ready, output)
        self.assertEqual(int(ready[1]) - int(ready[2]), delta, output)
        result = re.search(r"SUT_RESULT confirmed=(\d+) unconfirmed=(\d+)", output)
        self.assertIsNotNone(result, output)
        self.assertEqual((int(result[1]), int(result[2])),
                         (1, 0) if delta == 0 else (0, 1), output)

    def test_ordinary_idle_exact_deadline_is_confirmed(self):
        self.check_boundary(0)

    def test_ordinary_idle_after_deadline_is_unconfirmed(self):
        self.check_boundary(1)


if __name__ == "__main__":
    unittest.main()
