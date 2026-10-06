#!/usr/bin/env python3
"""只读检查真实六轴 bundle；当前没有最终许可时一律不签发运动资格。"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import stat
import subprocess

import yaml


JOINTS = [f"Joint{axis}" for axis in range(1, 7)]
PROFILE = "profiles/f405_6dof_bench.yaml"
MODEL = "model/model.yaml"
STM32_BY_ID = re.compile(
    r"serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_"
    r"Virtual_ComPort_in_FS_Mode_([A-Za-z0-9]+)-if00#baudrate=115200\Z"
)


def validate_endpoint(endpoint):
    pattern = (r"serial//dev/(?:ttyACM[1-9][0-9]*|"
               r"serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_[A-Za-z0-9]+-if00)"
               r"#baudrate=115200")
    if not re.fullmatch(pattern, endpoint):
        raise ValueError("explicit non-default Zenoh serial endpoint required")
    return endpoint


def stm32_by_id_path(endpoint, expected_serial):
    match = STM32_BY_ID.fullmatch(endpoint)
    if match is None or match.group(1) != expected_serial:
        raise ValueError("STM32 by-id endpoint and serial required")
    return Path(endpoint.split("#", 1)[0].removeprefix("serial/"))


def validate_live_usb_properties(endpoint, expected_serial, properties):
    stm32_by_id_path(endpoint, expected_serial)
    required = {
        "ID_BUS": "usb", "ID_VENDOR_ID": "1209", "ID_MODEL_ID": "7461",
        "ID_USB_INTERFACE_NUM": "00", "ID_SERIAL_SHORT": expected_serial,
        "ID_USB_DRIVER": "cdc_acm",
    }
    if any(properties.get(key) != value for key, value in required.items()):
        raise ValueError("STM32 USB identity mismatch")
    if not re.fullmatch(r"/dev/ttyACM[0-9]+", properties.get("DEVNAME", "")):
        raise ValueError("STM32 CDC tty identity mismatch")
    return True


def verify_live_stm32_usb(endpoint, expected_serial):
    by_id = stm32_by_id_path(endpoint, expected_serial)
    if not by_id.is_symlink():
        raise ValueError("STM32 by-id symlink missing")
    target = by_id.resolve(strict=True)
    if not stat.S_ISCHR(target.stat().st_mode):
        raise ValueError("STM32 by-id target is not a character device")
    result = subprocess.run(
        ["udevadm", "info", "--query=property", f"--name={by_id}"],
        check=True, capture_output=True, text=True, timeout=3,
    )
    properties = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator and key:
            if key in properties:
                raise ValueError("duplicate udev property")
            properties[key] = value
    validate_live_usb_properties(endpoint, expected_serial, properties)
    if properties["DEVNAME"] != str(target) or by_id.resolve(strict=True) != target:
        raise ValueError("STM32 by-id target changed during verification")
    return target


def inspect_bundle(root):
    root = Path(root).resolve(strict=True)
    manifest = json.loads((root / "manifest.json").read_text())
    if (manifest.get("format") != "dummy_wire_release_v2" or
            manifest.get("release_id") != "dummy-wire-v2.4"):
        raise ValueError("unsupported release manifest")
    payloads = manifest.get("payloads")
    if not isinstance(payloads, list) or not payloads:
        raise ValueError("empty manifest payloads")
    verified = {}
    for item in payloads:
        relative = item.get("path")
        if not isinstance(relative, str) or not relative or Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise ValueError("unsafe manifest path")
        path = (root / relative).resolve(strict=True)
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError("manifest payload outside bundle")
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != item.get("sha256") or relative in verified:
            raise ValueError("bundle payload hash mismatch")
        verified[relative] = actual
    if list(verified) != sorted(verified) or MODEL not in verified or PROFILE not in verified:
        raise ValueError("model/profile manifest coverage invalid")
    listing = "\n".join(f"{path}:{digest}" for path, digest in verified.items())
    contract_hash = hashlib.sha256(listing.encode()).hexdigest()
    if contract_hash != manifest.get("payload_sorted_sha256"):
        raise ValueError("bundle aggregate hash mismatch")
    profile = yaml.safe_load((root / PROFILE).read_text())
    model = yaml.safe_load((root / MODEL).read_text())
    if (profile.get("profile_name") != "f405_6dof_bench" or
            profile.get("profile_id") != 201 or profile.get("configured_mask") != 63 or
            profile.get("active_axes") != JOINTS or
            [joint.get("name") for joint in model.get("joints", [])] != JOINTS or
            profile.get("test_only") is not False):
        raise ValueError("real six-axis profile/model identity mismatch")
    reason = ("design_seed_default_deny" if profile.get("artifact_kind") == "design_seed_default_deny"
              else "final_release_identity_not_integrated")
    return {"bundle_verified": True, "profile_id": 201, "configured_mask": 63,
            "contract_sha256": contract_hash, "model_sha256": verified[MODEL],
            "profile_sha256": verified[PROFILE], "motion_eligible": False, "reason": reason}


def main():
    parser = argparse.ArgumentParser(description="离线检查真实六轴 bundle，不打开串口或运动授权")
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--device-id", required=True)
    parser.add_argument("--client-id", required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    try:
        validate_endpoint(args.endpoint)
        for name in ("device_id", "client_id"):
            value = getattr(args, name)
            if not re.fullmatch(r"[0-9a-f]{32}", value) or value == "0" * 32:
                raise ValueError(f"invalid {name}")
        if not re.fullmatch(r"[A-Za-z0-9_-]{4,64}", args.serial):
            raise ValueError("invalid board serial")
        report = inspect_bundle(args.bundle)
        report.update(endpoint=args.endpoint, device_id=args.device_id,
                      client_id=args.client_id, serial=args.serial)
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
        report = {"bundle_verified": False, "motion_eligible": False, "reason": str(error)}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
