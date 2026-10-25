#!/usr/bin/env python3
"""用当前契约生成隔离测试候选；临时镜像仅用于绑定单测。"""
##
# @file run_application_hold_candidate_profile.py
# @brief 为C++绑定测试生成隔离候选的可运行包装器。
#
# 会调用SDK生成模块并执行传入测试二进制；临时image字节明确UNIT_TEST_BINDING_ONLY_NOT_DEPLOYABLE，不构建/烧录固件。source/template为既有输入，生成自洽候选不等于实机审批。

from pathlib import Path
import subprocess
import hashlib
import json
import shutil
import os
import sys
import tempfile


##
# @brief argv四路径：测试程序、DPSDK根、模板根、source wire；临时生成候选，生成失败check=True抛错，否则返回C++测试退出码并清目录。
# @return C++测试真实退出码；生成失败异常传播。
def main() -> int:
    executable, dpsdk_root, template_root, source_wire = map(Path, sys.argv[1:])
    with tempfile.TemporaryDirectory(prefix="dummy-candidate-profile-test-") as temporary:
        root = Path(temporary)
        image = root / "unit-test-image.bin"
        image.write_bytes(b"UNIT_TEST_BINDING_ONLY_NOT_DEPLOYABLE_FIRMWARE\n")
        # 只在临时夹具显式绑定当前SDK模型和来源；原默认deny业务模板不被重签。
        template = root / "current-template"
        shutil.copytree(template_root, template)
        source = json.loads((source_wire / "manifest.json").read_text())
        manifest_path = template / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        shutil.copy2(source_wire / "model/model.yaml", template / "model/model.yaml")
        model_sha = hashlib.sha256((template / "model/model.yaml").read_bytes()).hexdigest()
        policy_path = template / "policy/application_hold_no_load.json"
        policy = json.loads(policy_path.read_text())
        policy['model_sha256'] = model_sha
        policy_path.write_text(json.dumps(policy, indent=2) + "\n")
        canonical = json.dumps({key: value for key, value in policy.items() if key != 'note'},
                               sort_keys=True, separators=(',', ':')).encode()
        manifest.update(source_contract_sha256=source['payload_sorted_sha256'],
                        model_sha256=model_sha, compiled_policy_sha256=hashlib.sha256(canonical).hexdigest())
        for item in manifest['payloads']:
            item['sha256'] = hashlib.sha256((template / item['path']).read_bytes()).hexdigest()
        manifest['payload_sorted_sha256'] = hashlib.sha256('\n'.join(
            f"{item['path']}:{item['sha256']}" for item in manifest['payloads']).encode()).hexdigest()
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        candidate = root / "candidate"
        subprocess.run(
            [sys.executable, "-m", "dummy_ref_tools.generate_no_load_dev_candidate",
             "--template-candidate-manifest", str(manifest_path),
             "--source-wire-manifest", str(source_wire / "manifest.json"),
             "--firmware", str(image), "--output-dir", str(candidate)],
            check=True,
            env=dict(os.environ, PYTHONPATH=str(dpsdk_root / "applications/dummy_ref_control/tools")),
        )
        limits = root / 'current-limits.json'
        subprocess.run([sys.executable, '-m', 'dummy_ref_tools.generate_application_ros_limits',
                        '--candidate-manifest', str(candidate / 'manifest.json'), '--output', str(limits)],
                       check=True, env=dict(os.environ, PYTHONPATH=str(dpsdk_root / 'applications/dummy_ref_control/tools')))
        converter = Path(__file__).resolve().parents[3] / 'scripts/dummy/convert_application_ros_limits.py'
        subprocess.run([sys.executable, str(converter), '--limits', str(limits),
                        '--candidate-manifest', str(candidate / 'manifest.json'),
                        '--output-dir', str(root / 'converted')], check=True)
        return subprocess.run(
            [str(executable), str(candidate), str(source_wire)],
            check=False,
        ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
