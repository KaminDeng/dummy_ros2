import hashlib
##
# @file test_wire_v2_contract.py
# @brief 冻结wire-v2 manifest身份、payload摘要与无v1运行负载回归。
#
# 只读合同文件；版本由生成codec的schema常量独立核对，摘要与canonical导出比较，不依赖旧版聚合常数。文件一致不代表实机兼容。
import json
import os
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BUNDLE = ROOT / "contracts/dummy_robot_v2"
CANONICAL = Path(os.environ.get("DPSDK_ROOT", str(ROOT.parent / "dpsdk"))) / "applications/dummy_ref_control/contracts/dummy_robot_v2"

##
# @brief 冻结wire-v2文件身份/摘要的unittest集合，只读取合同，不执行设备。
class WireV2ContractTest(unittest.TestCase):
    ##
    # @brief 每例重新解析本地manifest到self.manifest，不验签或加载设备。
    # @return None；只读取manifest到本测试实例。
    def setUp(self):
        self.manifest = json.loads((BUNDLE / "manifest.json").read_text())

    ##
    # @brief 核manifest与生成codec的schema一致，release标签含同一minor。
    # @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
    def test_release_identity_is_wire_major_v2(self):
        self.assertEqual("dummy_wire_release_v2", self.manifest["format"])
        codec = (BUNDLE / "codec/dp_dummy_robot_v2.hpp").read_text()
        major = int(re.search(r"kSchemaMajor = (\d+)U", codec).group(1))
        minor = int(re.search(r"kSchemaMinor = (\d+)U", codec).group(1))
        self.assertEqual(2, major)
        self.assertEqual((major, minor), (self.manifest["schema_major"], self.manifest["schema_minor"]))
        self.assertEqual(f"dummy-wire-v{major}.{minor}", self.manifest["release_id"])

    ##
    # @brief 核payload文件集封闭、排序去重、实际SHA/聚合以及canonical manifest身份。
    # @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
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
        self.assertEqual(sorted(set(paths)), paths)
        # 发布清单和最终发布资格schema是包元数据，不属于manifest的运行payload集合。
        metadata = {"manifest.json", "evidence/release_manifest.schema.json"}
        self.assertEqual(set(paths), {str(path.relative_to(BUNDLE)) for path in BUNDLE.rglob("*")
                                     if path.is_file() and str(path.relative_to(BUNDLE)) not in metadata})
        aggregate = hashlib.sha256("\n".join(rows).encode()).hexdigest()
        self.assertEqual(self.manifest["payload_sorted_sha256"], aggregate)
        self.assertEqual((BUNDLE / "manifest.json").read_bytes(), (CANONICAL / "manifest.json").read_bytes())

    ##
    # @brief 路径不含v1，codec/idl/metadata/enums文本无dummy_robot_v1；未检查其它文件类别的全部内容。
    # @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
    def test_bundle_contains_no_v1_runtime_payload(self):
        for payload in self.manifest["payloads"]:
            path = BUNDLE / payload["path"]
            self.assertNotIn("v1", payload["path"].lower())
            if payload["path"].split("/", 1)[0] in {"codec", "idl", "metadata", "enums"}:
                self.assertNotIn("dummy_robot_v1", path.read_text(errors="ignore"))

if __name__ == "__main__":
    unittest.main()
