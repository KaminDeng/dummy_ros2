#!/usr/bin/env python3
"""Contract-consumption tests for the R01 model task."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
BUNDLE = WORKSPACE_ROOT / "contracts" / "dummy_robot_v2"
DPSDK_BUNDLE = WORKSPACE_ROOT.parent / "dpsdk_consolidated" / "contracts" / "dummy_robot_v2"


class R01ContractTest(unittest.TestCase):
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
