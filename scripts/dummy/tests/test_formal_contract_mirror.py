"""当前正式contract镜像必须逐字节匹配canonical导出；不产生设备运行资格。"""
import hashlib
import os
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MIRROR = ROOT / 'contracts/dummy_robot_v2'
CANONICAL = Path(os.environ['DPSDK_ROOT']) / 'applications/dummy_ref_control/contracts/dummy_robot_v2'


def test_manifest_and_every_payload_match_canonical_export():
    assert (MIRROR / 'manifest.json').read_bytes() == (CANONICAL / 'manifest.json').read_bytes()
    manifest = json.loads((MIRROR / 'manifest.json').read_text())
    paths, rows = [], []
    for payload in manifest['payloads']:
        relative = Path(payload['path'])
        assert not relative.is_absolute() and '..' not in relative.parts
        mirror, canonical = MIRROR / relative, CANONICAL / relative
        assert mirror.read_bytes() == canonical.read_bytes(), str(relative)
        actual = hashlib.sha256(mirror.read_bytes()).hexdigest()
        assert actual == payload['sha256'], str(relative)
        paths.append(str(relative))
        rows.append(f'{relative}:{actual}')
    assert paths == sorted(set(paths))
    assert hashlib.sha256('\n'.join(rows).encode()).hexdigest() == manifest['payload_sorted_sha256']
