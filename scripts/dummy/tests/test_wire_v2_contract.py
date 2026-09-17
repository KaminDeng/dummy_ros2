import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BUNDLE = ROOT / "contracts/dummy_robot_v2"

class WireV2ContractTest(unittest.TestCase):
    def setUp(self):
        self.manifest = json.loads((BUNDLE / "manifest.json").read_text())

    def test_release_identity_is_wire_major_v2(self):
        self.assertEqual("dummy_wire_release_v2", self.manifest["format"])
        self.assertEqual("dummy-wire-v2", self.manifest["release_id"])
        self.assertEqual(2, self.manifest["schema_major"])
        self.assertEqual(0, self.manifest["schema_minor"])

    def test_payloads_are_closed_and_aggregate_hash_matches(self):
        rows, paths = [], []
        for payload in self.manifest["payloads"]:
            relative = payload["path"]
            path = BUNDLE / relative
            self.assertTrue(path.is_file(), relative)
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(payload["sha256"], digest, relative)
            rows.append(f"{relative}:{digest}")
            paths.append(relative)
        self.assertEqual(sorted(paths), paths)
        aggregate = hashlib.sha256("\n".join(rows).encode()).hexdigest()
        self.assertEqual(self.manifest["payload_sorted_sha256"], aggregate)
        self.assertEqual("914074d31391b455d90d8edc833bfebbe082d1ef1f411df21e5e9fe23f9b28f8", aggregate)

    def test_bundle_contains_no_v1_runtime_payload(self):
        for payload in self.manifest["payloads"]:
            path = BUNDLE / payload["path"]
            self.assertNotIn("v1", payload["path"].lower())
            if payload["path"].split("/", 1)[0] in {"codec", "idl", "metadata", "enums"}:
                self.assertNotIn("dummy_robot_v1", path.read_text(errors="ignore"))

if __name__ == "__main__":
    unittest.main()
