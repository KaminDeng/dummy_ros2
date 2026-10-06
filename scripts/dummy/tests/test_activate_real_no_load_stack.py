"""旧激活 helper 必须保持只读，且执行请求在 ROS 初始化前拒绝。"""

import importlib.util
from pathlib import Path
import subprocess
import sys


MODULE = Path(__file__).resolve().parents[1] / "activate_real_no_load_stack.py"
SPEC = importlib.util.spec_from_file_location("activate_real_no_load_stack", MODULE)
helper = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(helper)


def test_default_cli_is_read_only(capsys):
    assert helper.main([]) == 0
    output = capsys.readouterr().out
    assert "DRY_RUN" in output and "UNQUALIFIED" in output
    assert "未连接 ROS" in output


def test_execute_is_rejected_before_client_creation(capsys):
    created = []

    def client_factory():
        created.append(True)
        raise AssertionError("ROS 客户端不得创建")

    assert helper.main(["--execute"], client_factory=client_factory) != 0
    assert created == []
    output = capsys.readouterr().out
    assert "UNCONFIRMED" in output and "禁用" in output


def test_execute_process_fails_without_ros_or_device_access():
    result = subprocess.run([sys.executable, str(MODULE), "--execute"],
                            capture_output=True, text=True, timeout=5, check=False)
    assert result.returncode != 0
    assert "UNCONFIRMED" in result.stdout


def test_module_has_no_lifecycle_mutation_interface():
    assert not hasattr(helper, "activate")
    assert not hasattr(helper, "RosClient")


def test_read_only_diagnostic_is_not_installed_as_operator_entry():
    cmake = MODULE.parents[2] / "src/dummy_bringup/CMakeLists.txt"
    install_block = cmake.read_text().split("install(FILES", 1)[1].split(
        "DESTINATION share/${PROJECT_NAME}/scripts)", 1)[0]
    assert "activate_real_no_load_stack.py" not in install_block
