import hashlib
##
# @file test_wire_v2_contract.py
# @brief 冻结wire-v2 manifest身份、payload摘要与无v1运行负载回归。
#
# 只读合同文件；期望release dummy-wire-v2/schema2.0及固定聚合SHA属于原oracle，不自动适配其它阶段。检查只遍历manifest列项、不扫描额外文件，不能从方法名closed推断完整目录封闭或实机兼容。
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BUNDLE = ROOT / "contracts/dummy_robot_v2"

##
# @brief 冻结wire-v2文件身份/摘要的unittest集合，只读取合同，不执行设备。
class WireV2ContractTest(unittest.TestCase):
    ##
    # @brief 每例重新解析本地manifest到self.manifest，不验签或加载设备。
    # @return None；只读取manifest到本测试实例。
    def setUp(self):
        self.manifest = json.loads((BUNDLE / "manifest.json").read_text())

    ##
    # @brief 精确断言format/release/schema2.0冻结身份，不把主版本相同当所有布局兼容。
    # @return None；以assert/pytest.raises/unittest检查既定结论，不提供设备执行凭据。
    def test_release_identity_is_wire_major_v2(self):
        self.assertEqual("dummy_wire_release_v2", self.manifest["format"])
        self.assertEqual("dummy-wire-v2", self.manifest["release_id"])
        self.assertEqual(2, self.manifest["schema_major"])
        self.assertEqual(0, self.manifest["schema_minor"])

    ##
    # @brief 每列payload存在且SHA正确、路径排序和聚合等固定摘要；此例不独立检查重复项或目录额外文件。
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
        self.assertEqual(sorted(paths), paths)
        aggregate = hashlib.sha256("\n".join(rows).encode()).hexdigest()
        self.assertEqual(self.manifest["payload_sorted_sha256"], aggregate)
        self.assertEqual("914074d31391b455d90d8edc833bfebbe082d1ef1f411df21e5e9fe23f9b28f8", aggregate)

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
