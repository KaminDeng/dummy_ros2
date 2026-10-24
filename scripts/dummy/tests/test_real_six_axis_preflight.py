"""真实六轴入口在最终发布身份缺席时必须拒动。"""
##
# @file test_real_six_axis_preflight.py
# @brief 默认deny候选和串口/USB属性纯数据门的离线回归。
#
# bundle由临时最小YAML/manifest构造，endpoint验证只查语法/约定；live USB用例传人工properties字典，不运行udev、读/dev或确认设备连接。verified表示payload完整，不能升级motion_eligible或实体许可。

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


MODULE = Path(__file__).resolve().parents[1] / "real_six_axis_preflight.py"
SPEC = importlib.util.spec_from_file_location("real_six_axis_preflight", MODULE)
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


##
# @brief 写六轴名字的最小model与profile201/mask63默认deny，按相对路径排序生成原文件SHA和聚合manifest。
# @param tmp_path pytest临时目录Path，所有合成文件/报告只写此处。
# @return 临时最小bundle根Path，不是完整发布/激活包。
@pytest.fixture
def bundle(tmp_path):
    payloads = {
        "model/model.yaml": {"joints": [{"name": f"Joint{i}"} for i in range(1, 7)]},
        "profiles/f405_6dof_bench.yaml": {
            "profile_name": "f405_6dof_bench", "profile_id": 201,
            "configured_mask": 63, "active_axes": [f"Joint{i}" for i in range(1, 7)],
            "artifact_kind": "design_seed_default_deny", "test_only": False,
            "motion_permitted_on_real_hardware": False,
        },
    }
    manifest_items = []
    for relative, content in sorted(payloads.items()):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(content))
        manifest_items.append({"path": relative, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    listing = "\n".join(f"{item['path']}:{item['sha256']}" for item in manifest_items)
    (tmp_path / "manifest.json").write_text(json.dumps({
        "format": "dummy_wire_release_v2", "release_id": "dummy-wire-v2.5",
        "schema_major": 2, "schema_minor": 5,
        "payloads": manifest_items,
        "payload_sorted_sha256": hashlib.sha256(listing.encode()).hexdigest(),
    }))
    return tmp_path


@pytest.mark.parametrize("identity", [
    {"format": "dummy_wire_release_v1", "schema_major": 1},
    {"release_id": "dummy-wire-v2.4", "schema_minor": 4},
    {"schema_minor": 6},
    {"schema_major": True},
    {"schema_minor": "5"},
    {"schema_minor": None},
    {"release_id": "dummy-wire-v2"},
])
def test_unsupported_manifest_identity_is_rejected(bundle, identity):
    path = bundle / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest.update(identity)
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="unsupported release manifest"):
        preflight.inspect_bundle(bundle)


def test_missing_schema_is_rejected(bundle):
    path = bundle / "manifest.json"
    manifest = json.loads(path.read_text())
    del manifest["schema_minor"]
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="unsupported release manifest"):
        preflight.inspect_bundle(bundle)


def test_tampered_aggregate_is_rejected(bundle):
    path = bundle / "manifest.json"
    manifest = json.loads(path.read_text())
    manifest["payload_sorted_sha256"] = "00" * 32
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="aggregate hash"):
        preflight.inspect_bundle(bundle)


##
# @brief 摘要正确的design_seed仍motion_eligible=false并给默认拒绝原因。
# @param bundle bundle fixture生成的最小默认deny根目录Path。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_design_seed_is_verified_but_denied(bundle):
    result = preflight.inspect_bundle(bundle)
    assert result["bundle_verified"] is True
    assert result["profile_sha256"]
    assert result["motion_eligible"] is False
    assert result["reason"] == "design_seed_default_deny"


##
# @brief profile附一行而不更新payload SHA必须拒绝，即使YAML仍可解析。
# @param bundle bundle fixture生成的最小默认deny根目录Path。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_tampered_profile_is_rejected(bundle):
    with (bundle / "profiles/f405_6dof_bench.yaml").open("a") as output:
        output.write("changed: true\n")
    with pytest.raises(ValueError, match="hash"):
        preflight.inspect_bundle(bundle)


##
# @brief profile改measured_candidate/许可true并重算全部hash仍因最终发布身份未集成拒动，不能自声明激活。
# @param bundle bundle fixture生成的最小默认deny根目录Path。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_candidate_still_needs_final_release_identity(bundle):
    profile = bundle / "profiles/f405_6dof_bench.yaml"
    content = yaml.safe_load(profile.read_text())
    content.update(artifact_kind="measured_candidate", motion_permitted_on_real_hardware=True)
    profile.write_text(yaml.safe_dump(content))
    manifest = json.loads((bundle / "manifest.json").read_text())
    manifest["payloads"][1]["sha256"] = hashlib.sha256(profile.read_bytes()).hexdigest()
    listing = "\n".join(f"{item['path']}:{item['sha256']}" for item in manifest["payloads"])
    manifest["payload_sorted_sha256"] = hashlib.sha256(listing.encode()).hexdigest()
    (bundle / "manifest.json").write_text(json.dumps(manifest))
    result = preflight.inspect_bundle(bundle)
    assert result["motion_eligible"] is False
    assert result["reason"] == "final_release_identity_not_integrated"


##
# @brief 显式ttyACM1/12或精确STM32 by-id语法接受；这里只查字符串，不证明对应设备存在。
# @param endpoint 待核串口端点字符串，不实际打开设备。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
@pytest.mark.parametrize("endpoint", [
    "serial//dev/ttyACM1#baudrate=115200",
    "serial//dev/ttyACM12#baudrate=115200",
    "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200",
])
def test_endpoint_accepts_explicit_stm32_serial(endpoint):
    assert preflight.validate_endpoint(endpoint) == endpoint


##
# @brief TCP、默认ACM0、遍历、JLink、错interface/baud等字符串拒绝，避免意外默认设备。
# @param endpoint 待核串口端点字符串，不实际打开设备。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
@pytest.mark.parametrize("endpoint", [
    "tcp/127.0.0.1:7447",
    "serial//dev/ttyACM0#baudrate=115200",
    "serial//dev/../tmp/fake",
    "serial//dev/serial/by-id/usb-SEGGER_J-Link_123-if00#baudrate=115200",
    "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if01#baudrate=115200",
    "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=9600",
    "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_../ttyACM0-if00#baudrate=115200",
])
def test_endpoint_must_be_explicit_non_default_serial(endpoint):
    with pytest.raises(ValueError):
        preflight.validate_endpoint(endpoint)


##
# @brief 人工VID1209/PID7461/interface00/serial/CDC/ACM属性正例，逐字段换JLink或非ACM反例拒绝；不采真实USB属性。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_live_usb_identity_requires_exact_stm32_cdc_properties():
    endpoint = ("serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_"
                "Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200")
    properties = {
        "ID_BUS": "usb", "ID_VENDOR_ID": "1209", "ID_MODEL_ID": "7461",
        "ID_USB_INTERFACE_NUM": "00", "ID_SERIAL_SHORT": "DEMO00000001",
        "ID_USB_DRIVER": "cdc_acm", "DEVNAME": "/dev/ttyACM1",
    }
    assert preflight.validate_live_usb_properties(endpoint, "DEMO00000001", properties)
    for key, wrong in (
        ("ID_VENDOR_ID", "1366"), ("ID_MODEL_ID", "0105"),
        ("ID_USB_INTERFACE_NUM", "01"), ("ID_SERIAL_SHORT", "other"),
        ("ID_USB_DRIVER", "jlink"), ("DEVNAME", "/dev/ttyUSB0"),
    ):
        with pytest.raises(ValueError):
            preflight.validate_live_usb_properties(endpoint, "DEMO00000001",
                                                   {**properties, key: wrong})
    with pytest.raises(ValueError):
        preflight.validate_live_usb_properties(
            "serial//dev/ttyACM1#baudrate=115200", "DEMO00000001", properties)


##
# @brief by-id端点转Path保持绝对/dev/serial/by-id位置，防止丢前导斜线变相对路径。
# @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
def test_by_id_path_remains_absolute():
    endpoint = ("serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_"
                "Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200")
    assert preflight.stm32_by_id_path(endpoint, "DEMO00000001") == Path(
        "/dev/serial/by-id/usb-Dummy_Robotics_STM32_"
        "Virtual_ComPort_in_FS_Mode_DEMO00000001-if00")
