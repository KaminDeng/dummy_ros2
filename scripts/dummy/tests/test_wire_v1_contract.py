"""Independent consumer verification for the published C05 wire bundle."""

import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
BUNDLE = ROOT / "contracts/dummy_robot_v1"


class WireV1ContractTest(unittest.TestCase):
    def test_bundle_is_closed_complete_and_hash_verified(self):
        manifest = json.loads((BUNDLE / "manifest.json").read_text())
        self.assertEqual("dummy-wire-v1", manifest["release_id"])
        self.assertEqual("wire", manifest["stage"])
        self.assertEqual("2f9ebf1", manifest["r03_commit"])
        self.assertEqual(
            {"command": 125, "state": 356, "status": 297, "request": 233, "response": 618},
            manifest["abi_catalog"]["encoded_sizes"],
        )
        entries = manifest["payloads"]
        paths = [entry["path"] for entry in entries]
        self.assertEqual(sorted(paths), paths)
        self.assertEqual(len(paths), len(set(paths)))
        listed = {
            str(path.relative_to(BUNDLE))
            for path in BUNDLE.rglob("*")
            if path.is_file()
        } - {"manifest.json"}
        self.assertEqual(set(paths), listed)
        for entry in entries:
            self.assertEqual(
                entry["sha256"], hashlib.sha256((BUNDLE / entry["path"]).read_bytes()).hexdigest()
            )
        listing = "\n".join(f'{entry["path"]}:{entry["sha256"]}' for entry in entries)
        self.assertEqual(manifest["payload_sorted_sha256"], hashlib.sha256(listing.encode()).hexdigest())


if __name__ == "__main__":
    unittest.main()
