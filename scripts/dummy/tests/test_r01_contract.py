#!/usr/bin/env python3
"""Contract-consumption tests for the R01 model task."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
BUNDLE = WORKSPACE_ROOT / "contracts" / "dummy_robot_v1"


class R01ContractTest(unittest.TestCase):
    def test_model_bundle_is_complete_and_hash_verified(self):
        manifest_path = BUNDLE / "manifest.json"
        self.assertTrue(manifest_path.is_file(), "C02 model-v1 mirror is missing")
        manifest = json.loads(manifest_path.read_text())

        self.assertIn(
            (manifest["release_id"], manifest["stage"]),
            (("dummy-model-v1", "model"), ("dummy-wire-v1", "wire")),
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


if __name__ == "__main__":
    unittest.main()
