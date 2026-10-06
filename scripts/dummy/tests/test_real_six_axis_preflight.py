"""真实六轴入口在最终发布身份缺席时必须拒动。"""

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
        "format": "dummy_wire_release_v2", "release_id": "dummy-wire-v2.4",
        "payloads": manifest_items,
        "payload_sorted_sha256": hashlib.sha256(listing.encode()).hexdigest(),
    }))
    return tmp_path


def test_design_seed_is_verified_but_denied(bundle):
    result = preflight.inspect_bundle(bundle)
    assert result["bundle_verified"] is True
    assert result["profile_sha256"]
    assert result["motion_eligible"] is False
    assert result["reason"] == "design_seed_default_deny"


def test_tampered_profile_is_rejected(bundle):
    with (bundle / "profiles/f405_6dof_bench.yaml").open("a") as output:
        output.write("changed: true\n")
    with pytest.raises(ValueError, match="hash"):
        preflight.inspect_bundle(bundle)


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


@pytest.mark.parametrize("endpoint", [
    "serial//dev/ttyACM1#baudrate=115200",
    "serial//dev/ttyACM12#baudrate=115200",
    "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200",
])
def test_endpoint_accepts_explicit_stm32_serial(endpoint):
    assert preflight.validate_endpoint(endpoint) == endpoint


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


def test_by_id_path_remains_absolute():
    endpoint = ("serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_"
                "Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200")
    assert preflight.stm32_by_id_path(endpoint, "DEMO00000001") == Path(
        "/dev/serial/by-id/usb-Dummy_Robotics_STM32_"
        "Virtual_ComPort_in_FS_Mode_DEMO00000001-if00")
