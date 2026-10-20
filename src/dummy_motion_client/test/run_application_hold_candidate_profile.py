#!/usr/bin/env python3
"""用当前契约生成隔离测试候选；临时镜像仅用于绑定单测。"""
##
# @file run_application_hold_candidate_profile.py
# @brief 为C++绑定测试生成隔离候选的可运行包装器。
#
# 会调用SDK生成模块并执行传入测试二进制；临时image字节明确UNIT_TEST_BINDING_ONLY_NOT_DEPLOYABLE，不构建/烧录固件。source/template为既有输入，生成自洽候选不等于实机审批。

from pathlib import Path
import subprocess
import os
import sys
import tempfile


##
# @brief argv四路径：测试程序、DPSDK根、模板根、source wire；临时生成候选，生成失败check=True抛错，否则返回C++测试退出码并清目录。
# @return C++测试真实退出码；生成失败异常传播。
def main() -> int:
    executable, dpsdk_root, template_root, source_wire = map(Path, sys.argv[1:])
    with tempfile.TemporaryDirectory(prefix="dummy-candidate-profile-test-") as temporary:
        root = Path(temporary)
        image = root / "unit-test-image.bin"
        image.write_bytes(b"UNIT_TEST_BINDING_ONLY_NOT_DEPLOYABLE_FIRMWARE\n")
        candidate = root / "candidate"
        subprocess.run(
            [sys.executable, "-m", "dummy_ref_tools.generate_no_load_dev_candidate",
             "--template-candidate-manifest", str(template_root / "manifest.json"),
             "--source-wire-manifest", str(source_wire / "manifest.json"),
             "--firmware", str(image), "--output-dir", str(candidate)],
            check=True,
            env=dict(os.environ, PYTHONPATH=str(dpsdk_root / "applications/dummy_ref_control/tools")),
        )
        return subprocess.run(
            [str(executable), str(candidate), str(source_wire)],
            check=False,
        ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
