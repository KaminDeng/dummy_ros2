"""旧激活 helper 必须保持只读，且执行请求在 ROS 初始化前拒绝。"""
##
# @file test_activate_real_no_load_stack.py
# @brief 旧activation诊断入口的只读默认与执行前拒绝测试。
#
# 默认main只输出DRY_RUN/UNQUALIFIED；--execute用注入factory和独立进程核初始化前拒绝，不创建ROS客户端或设备连接。最后CMake文本检查只核未安装操作入口，不代表所有部署路径均不存在。

import importlib.util
from pathlib import Path
import subprocess
import sys


MODULE = Path(__file__).resolve().parents[1] / "activate_real_no_load_stack.py"
SPEC = importlib.util.spec_from_file_location("activate_real_no_load_stack", MODULE)
helper = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(helper)


##
# @brief 直接main([])返回0且声明DRY_RUN/UNQUALIFIED/未连接ROS，不把退出0当激活成功。
# @param capsys pytest stdout/stderr捕获fixture。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_default_cli_is_read_only(capsys):
    assert helper.main([]) == 0
    output = capsys.readouterr().out
    assert "DRY_RUN" in output and "UNQUALIFIED" in output
    assert "未连接 ROS" in output


##
# @brief 注入一旦被调就失败的client_factory，--execute应非零且从未调用它。
# @param capsys pytest stdout/stderr捕获fixture。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_execute_is_rejected_before_client_creation(capsys):
    created = []

    ##
    # @par 内部回调 client_factory
    # 记录调用并立即抛AssertionError，若执行路径触达客户端构造就使测试失败。
    # 返回：无正常返回；总抛AssertionError。
    def client_factory():
        created.append(True)
        raise AssertionError("ROS 客户端不得创建")

    assert helper.main(["--execute"], client_factory=client_factory) != 0
    assert created == []
    output = capsys.readouterr().out
    assert "UNCONFIRMED" in output and "禁用" in output


##
# @brief 以5s期限实际运行旧helper --execute进程，必须非零并打印UNCONFIRMED；不执行激活。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_execute_process_fails_without_ros_or_device_access():
    result = subprocess.run([sys.executable, str(MODULE), "--execute"],
                            capture_output=True, text=True, timeout=5, check=False)
    assert result.returncode != 0
    assert "UNCONFIRMED" in result.stdout


##
# @brief 核模块未导出activate/RosClient名字，属于接口形状检查。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_module_has_no_lifecycle_mutation_interface():
    assert not hasattr(helper, "activate")
    assert not hasattr(helper, "RosClient")


##
# @brief 解析现有CMake install(FILES文本块，要求旧helper不在该操作入口装配中。
# @return None；成功/拒绝用assert/pytest.raises断言，不产生设备执行凭据。
def test_read_only_diagnostic_is_not_installed_as_operator_entry():
    cmake = MODULE.parents[2] / "src/dummy_bringup/CMakeLists.txt"
    install_block = cmake.read_text().split("install(FILES", 1)[1].split(
        "DESTINATION share/${PROJECT_NAME}/scripts)", 1)[0]
    assert "activate_real_no_load_stack.py" not in install_block
