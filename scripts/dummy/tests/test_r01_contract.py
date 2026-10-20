#!/usr/bin/env python3
"""Contract-consumption tests for the R01 model task."""
##
# @file test_r01_contract.py
# @brief 当前镜像bundle文件SHA、聚合摘要及兄弟SDK字节一致性检查。
#
# 只读原始manifest/payload和固定dpsdk_consolidated/contracts路径，不运行codec/设备。摘要一致证明文件完整/镜像一致，不等于模型真实校准、runtime activation或执行许可。

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
BUNDLE = WORKSPACE_ROOT / "contracts" / "dummy_robot_v2"
DPSDK_BUNDLE = WORKSPACE_ROOT.parent / "dpsdk_consolidated" / "contracts" / "dummy_robot_v2"


##
# @brief 本套件unittest验收集合；实际子进程与证据层级见文件职责，各方法断言原样保留。
class R01ContractTest(unittest.TestCase):
    ##
    # @brief 核固定release/schema、14个payload排序去重、各文件原SHA，以及path:sha换行清单聚合SHA。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_model_bundle_is_complete_and_hash_verified(self):
        manifest_path = BUNDLE / "manifest.json"
        self.assertTrue(manifest_path.is_file(), "wire-v2 bundle is missing")
        manifest = json.loads(manifest_path.read_text())

        self.assertEqual("dummy_wire_release_v2", manifest["format"])
        self.assertEqual("dummy-wire-v2", manifest["release_id"])
        self.assertEqual(2, manifest["schema_major"])
        payloads = manifest["payloads"]
        paths = [entry["path"] for entry in payloads]
        self.assertEqual(sorted(paths), paths)
        self.assertEqual(len(paths), len(set(paths)))

        required = {
            "codec/dp_dummy_robot_v2.hpp",
            "codec/dp_dummy_robot_v2_dp_comm.hpp",
            "codec/dp_dummy_robot_v2_static_requirements.h",
            "codec/wire_layout.json",
            "enums/dummy_enum_values_v2.json",
            "idl/DummyDeviceStatusV2.msg",
            "idl/DummyJointCommandV2.msg",
            "idl/DummyJointStateV2.msg",
            "idl/DummyManageV2.srv",
            "metadata/dummy_robot_v2.yaml",
            "model/model.yaml",
            "profiles/sil_6dof.yaml",
            "profiles/sil_j1.yaml",
            "profiles/f405_j1_bench.yaml",
        }
        self.assertEqual(required, set(paths))
        for entry in payloads:
            payload = BUNDLE / entry["path"]
            self.assertTrue(payload.is_file())
            self.assertEqual(entry["sha256"], hashlib.sha256(payload.read_bytes()).hexdigest())

        listing = "\n".join(f'{entry["path"]}:{entry["sha256"]}' for entry in payloads)
        self.assertEqual(
            manifest["payload_sorted_sha256"],
            hashlib.sha256(listing.encode()).hexdigest(),
        )

    ##
    # @brief 要求固定兄弟SDK镜像目录、manifest完整相等并逐payload字节一致；不自动从环境变量选择SDK。
    # @return None；通过assert/unittest断言报告结果，失败抛异常。
    def test_wire_v2_mirror_matches_current_dpsdk_release(self):
        self.assertTrue(DPSDK_BUNDLE.is_dir(), "sibling DPSDK contract bundle is missing")
        ros_manifest = json.loads((BUNDLE / "manifest.json").read_text())
        dpsdk_manifest = json.loads((DPSDK_BUNDLE / "manifest.json").read_text())
        self.assertEqual("dummy-wire-v2", ros_manifest["release_id"])
        self.assertEqual(dpsdk_manifest, ros_manifest)
        for entry in dpsdk_manifest["payloads"]:
            self.assertEqual(
                (DPSDK_BUNDLE / entry["path"]).read_bytes(),
                (BUNDLE / entry["path"]).read_bytes(),
                entry["path"],
            )


if __name__ == "__main__":
    unittest.main()
