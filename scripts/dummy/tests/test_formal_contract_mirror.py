"""当前安装SDK机器人包的来源与文件完整性；不产生运行资格。"""
import hashlib
import json
from pathlib import Path
from scripts.dummy.installed_sdk import robot_bundle


def test_manifest_and_every_payload_match_canonical_export():
    bundle = robot_bundle()
    manifest = json.loads((bundle / 'manifest.json').read_text())
    assert manifest['release_id'] == 'dummy-wire-v2.5'
    assert (manifest['schema_major'], manifest['schema_minor']) == (2, 5)
    paths, rows = [], []
    for payload in manifest['payloads']:
        relative = Path(payload['path'])
        assert not relative.is_absolute() and '..' not in relative.parts
        actual = hashlib.sha256((bundle / relative).read_bytes()).hexdigest()
        assert actual == payload['sha256'], str(relative)
        paths.append(str(relative))
        rows.append(f'{relative}:{actual}')
    assert paths == sorted(set(paths))
    assert hashlib.sha256('\n'.join(rows).encode()).hexdigest() == manifest['payload_sorted_sha256']
    release = manifest['protocol_release']
    assert release['selection'] == {'protocol_set': 'dummy', 'profile': 'profile_full', 'codec': 'raw-fixed'}
    assert hashlib.sha256((bundle / release['path']).read_bytes()).hexdigest() == release['sha256']
    assert manifest['contract_sha256'] == manifest['wire_contract']['sha256']
