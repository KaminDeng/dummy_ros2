"""面向操作员解释错误，同时保留原始码用于复盘。"""
##
# @file test_manual_panel_messages.py
# @brief 操作员说明与原始错误码并存的纯字符串回归。
#
# 动态导入消息模块仅做文本映射，不读取实时Fault/MoveIt，也不发控制。包含已知事件、未知reason、MoveIt负错误和退出56归档失败。
import importlib.util
from pathlib import Path

source = Path(__file__).resolve().parents[1]/'scripts/manual_panel_messages.py'
spec = importlib.util.spec_from_file_location('manual_panel_messages', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


##
# @brief 已知MOVE_STARTED/0x1002要有中文解释和原码，未知0xffff仍保留且标未识别。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_known_native_event_and_fault_have_explanation_and_original_code():
    assert '开始' in module.explain_panel_log('MOVE_STARTED')
    assert 'MOVE_STARTED' in module.explain_panel_log('MOVE_STARTED')
    text = module.explain_panel_log('当前故障 reason=0x1002')
    assert '限位' in text and '0x1002' in text
    unknown = module.explain_panel_log('reason=0xffff')
    assert '0xffff' in unknown and '未识别' in unknown


##
# @brief 核−31逆解、−10碰撞、−6超时说明与原码，不据此断言实际错误原因。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_moveit_error_explains_operator_action():
    text = module.moveit_error(-31)
    assert '逆解' in text and '-31' in text
    assert '碰撞' in module.moveit_error(-10)
    assert '超时' in module.moveit_error(-6)


##
# @brief 退出code56文本同时保留数字及日志归档说明，避免只给不透明进程码。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_exit_archive_failure_has_human_explanation():
    text = module.explain_panel_log('通信进程已退出，code=56')
    assert '56' in text and '日志' in text
