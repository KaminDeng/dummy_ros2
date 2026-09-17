"""Independent consumer verification for the published C05 wire bundle."""

import hashlib
import json
from pathlib import Path
import runpy
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[3]
BUNDLE = ROOT / "contracts/dummy_robot_v1"


class WireV1ContractTest(unittest.TestCase):
    def test_bundle_is_closed_complete_and_hash_verified(self):
        manifest = json.loads((BUNDLE / "manifest.json").read_text())
        self.assertEqual("dummy-wire-v2", manifest["release_id"])
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
            if path.is_file() and "c03_prototype" not in path.relative_to(BUNDLE).parts
            and "__pycache__" not in path.relative_to(BUNDLE).parts
        } - {"manifest.json"}
        self.assertEqual(set(paths), listed)
        for entry in entries:
            self.assertEqual(
                entry["sha256"], hashlib.sha256((BUNDLE / entry["path"]).read_bytes()).hexdigest()
            )
        listing = "\n".join(f'{entry["path"]}:{entry["sha256"]}' for entry in entries)
        self.assertEqual(manifest["payload_sorted_sha256"], hashlib.sha256(listing.encode()).hexdigest())

    def test_activation_device_id_is_wire_identity_not_sha256(self):
        path = BUNDLE / "policy/validate_activation.py"
        validator = runpy.run_path(str(path))
        self.assertTrue(validator["is_device_id"]("01" + "00" * 15))
        self.assertFalse(validator["is_device_id"]("01" + "00" * 31))

    def test_c03_prototype_fixture_matches_pinned_commit_and_is_not_release_payload(self):
        prefix = "contracts/dummy_robot_v1/c03_prototype/generated"
        listing = subprocess.run(
            ["git", "ls-tree", "-r", "--name-only", "2f9ebf1", prefix],
            cwd=ROOT, capture_output=True, text=True, check=True,
        ).stdout.splitlines()
        self.assertTrue(listing)
        manifest = json.loads((BUNDLE / "manifest.json").read_text())
        self.assertFalse(any(entry["path"].startswith("c03_prototype/") for entry in manifest["payloads"]))
        for relative in listing:
            expected = subprocess.run(
                ["git", "show", f"2f9ebf1:{relative}"], cwd=ROOT, capture_output=True, check=True,
            ).stdout
            self.assertEqual(expected, (ROOT / relative).read_bytes(), relative)


if __name__ == "__main__":
    unittest.main()
