#!/usr/bin/env python3
##
# @file real_six_axis_preflight.py
# @brief 核当前v2.5六轴bundle的离线自洽，始终拒绝运动资格。
#
# CLI只验证端点/ID文本与文件，不调用verify_live_stm32_usb、不打开串口；总是motion_eligible=false且退出1。
# 文件内另有显式live USB属性helper，会查询udev和设备节点，不能因为CLI离线就把所有函数都称纯内存。
# 只接受当前dummy-wire-v2.5/schema2.5，旧版或未知schema不能因相似名字自动通过；自洽manifest不等于外部可信签名。
"""只读检查真实六轴 bundle；当前没有最终许可时一律不签发运动资格。"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import stat
import subprocess

import yaml


##
# @brief 固定六轴顺序、模型/profile payload路径以及STM32 by-id端点正则。
# 设备短序列号须与调用者期望精确相同；正则只约束名称，不能单独证明当前USB设备身份。
JOINTS = [f"Joint{axis}" for axis in range(1, 7)]
PROFILE = "profiles/f405_6dof_bench.yaml"
MODEL = "model/model.yaml"
STM32_BY_ID = re.compile(
    r"serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_"
    r"Virtual_ComPort_in_FS_Mode_([A-Za-z0-9]+)-if00#baudrate=115200\Z"
)


##
# @brief 拒绝默认ttyACM0及非显式串行端点形状。
# @param endpoint 115200串行端点字符串，允许ttyACM正整数或指定STM32 by-id名称。
# @return 原文本；不符ValueError。
# ttyACM分支不能以0开头；这里只核字符串，不stat、不udev、不连接设备。
def validate_endpoint(endpoint):
    pattern = (r"serial//dev/(?:ttyACM[1-9][0-9]*|"
               r"serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_[A-Za-z0-9]+-if00)"
               r"#baudrate=115200")
    if not re.fullmatch(pattern, endpoint):
        raise ValueError("explicit non-default Zenoh serial endpoint required")
    return endpoint


##
# @brief 要求指定STM32 by-id端点并绑定短序列号。
# @param endpoint 必须匹配STM32_BY_ID的完整文本。
# @param expected_serial 期望短序列号，精确比较捕获字段。
# @return 去serial/及波特后缀后的/dev/serial/by-id Path；未检查存在。
def stm32_by_id_path(endpoint, expected_serial):
    match = STM32_BY_ID.fullmatch(endpoint)
    if match is None or match.group(1) != expected_serial:
        raise ValueError("STM32 by-id endpoint and serial required")
    return Path(endpoint.split("#", 1)[0].removeprefix("serial/"))


##
# @brief 核调用方提供的udev属性与指定USB身份。
# @param endpoint STM32 by-id端点。
# @param expected_serial 期望ID_SERIAL_SHORT。
# @param properties 已解析属性映射，需usb/1209/7461/interface00/cdc_acm及ttyACM DEVNAME。
# @return 全部字段相符True，否则ValueError；本函数本身不运行udevadm。
# 匹配描述符不是固件镜像SHA/会话许可认证。
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


##
# @brief 显式查询当前by-id节点与udev属性，核前后目标未改变。
# @param endpoint 仅支持指定STM32 by-id端点。
# @param expected_serial 期望设备短序列号。
# @return 解析后的字符设备Path；缺symlink、属性不符、目标变化、3s命令超时或I/O错误失败。
# 运行udevadm info但不open串口收发。检查与后续使用非原子，不能概括为抵御所有并发设备替换；main没有调用此helper。
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


##
# @brief 按清单逐文件重算SHA并核当前六轴角色。
# @param root 存在的bundle目录，含manifest/model/profile。
# @return bundle_verified=true的报告，但motion_eligible恒false；文件/形状/身份错误抛异常。
# 拒绝绝对/..路径、越根、重复payload和未排序清单，重算path:hash无尾换行的聚合SHA。
# 模型只核关节名称顺序，profile核名称/id201/mask63/活动轴/test_only；不做完整真实机械模型校准/策略验证，也不要求目录无额外文件。
def inspect_bundle(root):
    root = Path(root).resolve(strict=True)
    manifest = json.loads((root / "manifest.json").read_text())
    if (manifest.get("format") != "dummy_wire_release_v2" or
            manifest.get("release_id") != "dummy-wire-v2.5" or
            type(manifest.get("schema_major")) is not int or
            type(manifest.get("schema_minor")) is not int or
            manifest["schema_major"] != 2 or manifest["schema_minor"] != 5):
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


##
# @brief 离线整理端点、ID和bundle检查报告。
# @return 无论检查成功失败都返回1，表示没有最终运动资格；报告写失败可另抛异常。
# device/client必须为32个小写十六进制字符（128bit）、且非全零，serial形状允许下划线/短横；此入口不核其与by-id捕获序列号一致，不调用live USB helper。
# 成功reason按design_seed_default_deny或final_release_identity_not_integrated区分，不能把bundle_verified=true当允许launch/激活。
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
