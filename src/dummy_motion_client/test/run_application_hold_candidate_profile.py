#!/usr/bin/env python3
"""用当前契约生成隔离测试候选；临时镜像仅用于绑定单测。"""

from pathlib import Path
import subprocess
import sys
import tempfile


def main() -> int:
    executable, dpsdk_root, template_root = map(Path, sys.argv[1:])
    with tempfile.TemporaryDirectory(prefix="dummy-candidate-profile-test-") as temporary:
        root = Path(temporary)
        image = root / "unit-test-image.bin"
        image.write_bytes(b"UNIT_TEST_BINDING_ONLY_NOT_DEPLOYABLE_FIRMWARE\n")
        candidate = root / "candidate"
        subprocess.run(
            [sys.executable, str(dpsdk_root / "tools/dummy/generate_no_load_dev_candidate.py"),
             "--template-candidate-manifest", str(template_root / "manifest.json"),
             "--source-wire-manifest", str(dpsdk_root / "contracts/dummy_robot_v2/manifest.json"),
             "--firmware", str(image), "--output-dir", str(candidate)],
            check=True,
        )
        return subprocess.run(
            [str(executable), str(candidate), str(dpsdk_root / "contracts/dummy_robot_v2")],
            check=False,
        ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
