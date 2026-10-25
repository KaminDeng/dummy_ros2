"""Offline identity and deny-gate checks for the F405 no-load candidate."""
##
# @file test_application_hold_candidate_bundle.py
# @brief 既有F405默认deny候选与SDK来源/固件产物的离线身份检查。
#
# 读取显式DPSDK_ROOT和候选原件，不生成/flash固件，不连接设备。候选policy/model自洽不等于当前source绑定；未绑定的固件和不同source身份必须保持未资格。

import hashlib
import json
import os
import re
from pathlib import Path

import yaml

from scripts.dummy.real_six_axis_preflight import inspect_bundle
from scripts.dummy.installed_sdk import robot_bundle


ROOT = Path(__file__).resolve().parents[3]
DPSDK = Path(os.environ.get("DPSDK_ROOT", str(ROOT.parent / "dpsdk")))
BUNDLE = ROOT / "contracts/dummy_robot_v2_application_hold_no_load_candidate"
POLICY = BUNDLE / "policy/application_hold_no_load.json"
SDK_POLICY = DPSDK / "applications/dummy_ref_control/config/f405_6dof_application_hold_no_load_candidate.json"
SOURCE_CONTRACT = robot_bundle() / "manifest.json"


##
# @brief 读既有文件原字节并求SHA，缺文件直接异常，不替造产物。
# @param path 目标文件Path，读取原字节；是否原始被替换见函数职责。
# @return 文件原字节SHA-256 hex。
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


##
# @brief 核候选payload/聚合、内含policy/model和策略语义，明确旧source及空固件绑定不转移资格。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
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
    assert manifest["firmware_image_sha256"] is None
    assert manifest["source_firmware_image"] is None
    assert re.fullmatch(r"[0-9a-f]{64}", manifest["source_contract_sha256"])
    assert manifest["source_contract_sha256"] != json.loads(SOURCE_CONTRACT.read_text())["payload_sorted_sha256"]
    assert manifest["draft_status"].startswith("UNQUALIFIED:")
    assert "new_matching_J6_firmware_and_Host" in manifest["qualification_gap"]
    assert manifest["model_sha256"] == digest(BUNDLE / "model/model.yaml")
    assert manifest["model_sha256"] == source["model_sha256"]
    assert manifest["model_sha256"] != digest(robot_bundle() / "model/model.yaml")
    sdk_policy = json.loads(SDK_POLICY.read_text())
    sdk_policy["model_sha256"] = manifest["model_sha256"]
    assert source == sdk_policy
    profile = yaml.safe_load((BUNDLE / "profiles/f405_6dof_bench.yaml").read_text())
    assert profile["active_axes"] == source["active_axes"]
    assert profile["reference_kind"] == source["reference_kind"]
    assert profile["deployment_stop_policy"] == source["deployment_stop_policy"]
    assert profile["safe_terminal_action"] == source["safe_terminal_action"]
    assert profile["no_load_bench"] == source["no_load_bench"]
    assert profile["gravity_loaded"] == source["gravity_loaded"]
    for key in profile["timing"]:
        assert profile["timing"][key] == source["timing"][key]


##
# @brief 核默认runtime=false/无post_build、profile实机许可false，inspect_bundle只得到verified而非motion_eligible，固定拒绝原因保持。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
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
