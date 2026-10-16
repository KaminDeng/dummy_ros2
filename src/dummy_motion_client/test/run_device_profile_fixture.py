#!/usr/bin/env python3
"""构造哈希自洽但未受信任的独立契约，保留原 C++ 拒绝断言。"""

from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile


def main() -> int:
    executable, trusted_bundle = map(Path, sys.argv[1:])
    with tempfile.TemporaryDirectory(prefix="dummy-foreign-profile-test-") as temporary:
        foreign = Path(temporary) / "foreign"
        shutil.copytree(trusted_bundle, foreign)
        profile = foreign / "profiles/sil_j1.yaml"
        with profile.open("a") as output:
            output.write("\n# 独立未受信任契约测试副本\n")
        manifest_path = foreign / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        for payload in manifest["payloads"]:
            payload["sha256"] = hashlib.sha256((foreign / payload["path"]).read_bytes()).hexdigest()
        manifest["payload_sorted_sha256"] = hashlib.sha256(
            "\n".join(f"{item['path']}:{item['sha256']}" for item in manifest["payloads"]).encode()
        ).hexdigest()
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        return subprocess.run(
            [str(executable), str(trusted_bundle), str(foreign)], check=False,
        ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
