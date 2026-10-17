"""Offline identity and deny-gate checks for the F405 no-load candidate."""

import hashlib
import json
from pathlib import Path

import yaml

from scripts.dummy.real_six_axis_preflight import inspect_bundle


ROOT = Path(__file__).resolve().parents[3]
DPSDK = ROOT.parent / "dpsdk_consolidated"
BUNDLE = ROOT / "contracts/dummy_robot_v2_application_hold_no_load_candidate"
POLICY = DPSDK / "configs/f405_6dof_application_hold_no_load_candidate.json"
IMAGE = DPSDK / "build_dummy/reports/f405-p7-app-48c8-20260930.bin"
SOURCE_CONTRACT = DPSDK / "applications/dummy_ref_control/contracts/dummy_robot_v2/manifest.json"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_candidate_payloads_and_frozen_d_identity():
    manifest = json.loads((BUNDLE / "manifest.json").read_text())
    payloads = manifest["payloads"]
    paths = [item["path"] for item in payloads]
    assert paths == sorted(set(paths))
    for item in payloads:
        assert digest(BUNDLE / item["path"]) == item["sha256"]
    listing = "\n".join(f'{item["path"]}:{item["sha256"]}' for item in payloads)
    assert hashlib.sha256(listing.encode()).hexdigest() == manifest["payload_sorted_sha256"]

    source = json.loads(POLICY.read_text())
    canonical = json.dumps({key: value for key, value in source.items() if key != "note"},
                           sort_keys=True, separators=(",", ":")).encode()
    assert manifest["compiled_policy_sha256"] == hashlib.sha256(canonical).hexdigest()
    assert manifest["firmware_image_sha256"] == digest(IMAGE)
    assert manifest["source_contract_sha256"] == json.loads(SOURCE_CONTRACT.read_text())["payload_sorted_sha256"]
    assert manifest["model_sha256"] == digest(DPSDK / "applications/dummy_ref_control/contracts/dummy_robot_v2/model/model.yaml")
    assert json.loads((BUNDLE / "policy/application_hold_no_load.json").read_text()) == source
    profile = yaml.safe_load((BUNDLE / "profiles/f405_6dof_bench.yaml").read_text())
    assert profile["active_axes"] == source["active_axes"]
    assert profile["reference_kind"] == source["reference_kind"]
    assert profile["deployment_stop_policy"] == source["deployment_stop_policy"]
    assert profile["safe_terminal_action"] == source["safe_terminal_action"]
    assert profile["no_load_bench"] == source["no_load_bench"]
    assert profile["gravity_loaded"] == source["gravity_loaded"]
    for key in profile["timing"]:
        assert profile["timing"][key] == source["timing"][key]


def test_candidate_stays_unqualified_and_preflight_denies_motion():
    manifest = json.loads((BUNDLE / "manifest.json").read_text())
    profile = yaml.safe_load((BUNDLE / "profiles/f405_6dof_bench.yaml").read_text())
    assert manifest["runtime_enabled"] is False
    assert "post_build_bindings" not in manifest
    assert profile["artifact_kind"] == "application_hold_no_load_candidate"
    assert profile["motion_permitted_on_real_hardware"] is False
    assert profile["profile_id"] == 201
    assert profile["configured_mask"] == 63
    result = inspect_bundle(BUNDLE)
    assert result["bundle_verified"] is True
    assert result["motion_eligible"] is False
    assert result["reason"] == "final_release_identity_not_integrated"
