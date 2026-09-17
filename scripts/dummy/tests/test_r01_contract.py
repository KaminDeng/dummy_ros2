#!/usr/bin/env python3
"""Contract-consumption tests for the R01 model task."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
BUNDLE = WORKSPACE_ROOT / "contracts" / "dummy_robot_v2"
DPSDK_BUNDLE = WORKSPACE_ROOT.parent / "dpsdk" / "contracts" / "dummy_robot_v2"


class R01ContractTest(unittest.TestCase):
    def test_model_bundle_is_complete_and_hash_verified(self):
        manifest_path = BUNDLE / "manifest.json"
        self.assertTrue(manifest_path.is_file(), "C02 model-v1 mirror is missing")
        manifest = json.loads(manifest_path.read_text())

        self.assertIn(
            (manifest["release_id"], manifest["stage"]),
            (("dummy-model-v1", "model"), ("dummy-wire-v2", "wire")),
        )
        payloads = manifest["payloads"]
        paths = [entry["path"] for entry in payloads]
        self.assertEqual(sorted(paths), paths)
        self.assertEqual(len(paths), len(set(paths)))

        required = {
            "model/model.yaml",
            "model/geometry_reference.yaml",
            "profiles/profile_schema.json",
            "profiles/gazebo_6dof.yaml",
            "profiles/sil_6dof.yaml",
            "profiles/sil_j1.yaml",
            "profiles/f405_j1_passive.yaml",
            "profiles/f405_j1_bench.yaml",
            "fixtures/can_golden.json",
            "fixtures/fk_cases.json",
        }
        self.assertTrue(required.issubset(paths))
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
