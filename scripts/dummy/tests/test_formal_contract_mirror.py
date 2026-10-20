"""当前正式contract镜像必须逐字节匹配canonical导出；不产生设备运行资格。"""
##
# @file test_formal_contract_mirror.py
# @brief 正式ROS合同镜像与显式canonical导出的逐字节/摘要一致性。
#
# 模块导入时必须已有DPSDK_ROOT环境变量，canonical取其applications/dummy_ref_control/contracts路径。只核manifest列举payload，不运行SDK/codec/设备；摘要相同不赋予发布资格或动作许可。
import hashlib
import os
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MIRROR = ROOT / 'contracts/dummy_robot_v2'
CANONICAL = Path(os.environ['DPSDK_ROOT']) / 'applications/dummy_ref_control/contracts/dummy_robot_v2'


##
# @brief manifest及每payload逐字节同canonical，路径非绝对/无..、原SHA/排序去重及聚合hash全部一致。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
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
