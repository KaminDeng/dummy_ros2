#!/usr/bin/env python3
"""用当前契约生成隔离测试候选；临时镜像仅用于绑定单测。"""

from pathlib import Path
import subprocess
import os
import sys
import tempfile


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
