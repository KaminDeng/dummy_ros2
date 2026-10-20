"""人工双限位GUI事件/原方法离线验证，不构造Native或USB。"""
##
# @file test_manual_panel_limit_reference.py
# @brief 双限位参考确认及受控Rest退让的AST面板逻辑回归。
#
# 复用去Qt构造的panel夹具，弹框/命令都是列表；没有Native、ROS或USB。名义止挡/工作Rest是deg，State位置rad。LIMITS确认不运动，与后续REST命令分开核；退让预览不改实际State。
import math
import time
from test_manual_panel_fault_readonly import panel


##
# @brief 将基础夹具设为manual_limits，确认姿势J2−73°/J3 180°，working范围与原Rest独立，记录确认文本。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param core 人工core_state，默认2 Idle。
# @return (面板逻辑对象, Qt替身scope)。
def limits_panel(tmp_path, core=2):
    p, scope = panel(tmp_path, core)
    p.cfg.update(calibration_mode='manual_limits', calibration_pose_deg=[0,-73,180,0,0,0],
                 home_deg=[0,0,90,0,0,0])
    p.cfg['joints'][1]['nominal_limits_deg'] = [-73,90]
    p.cfg['joints'][2]['nominal_limits_deg'] = [35,180]
    p.limits[1] = (-60,77); p.limits[2] = (48,167)
    p.last_state['position_rad'] = [0, math.radians(-73), math.pi, 0, 0, 0]
    p.pending_home = False; p.pending_home_generation = None; p.pending_authorization = None
    p.dialogs = []
    ##
    # @par 内部回调 confirm
    # 将title/text记入dialogs并固定Yes，不创建真实对话框。
    # 参数 parent：对话框父对象，替身忽略。
    # 参数 title：确认标题。
    # 参数 text：确认正文，记录供断言。
    # 参数 args：额外位置参数，替身忽略。
    # 返回：固定Yes数值1。
    def confirm(parent, title, text, *args): p.dialogs.append((title,text)); return 1
    scope['QtWidgets'].QMessageBox.question = confirm
    return p, scope


##
# @brief 首次参考确认只发LIMITS CONFIRM_MARGIN，说明双限位而非旧Home，原home_deg不变。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_limits_initial_confirm_uses_limits_command_not_old_home(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.start_zero()
    assert p.commands == ['LIMITS CONFIRM_MARGIN']
    assert '双限位' in p.dialogs[0][0] and '−73' not in p.dialogs[0][0]
    assert '不是旧Home' in p.dialogs[0][1] and '180' in p.dialogs[0][1]
    assert p.cfg['home_deg'] == [0,0,90,0,0,0]


##
# @brief 错确认姿势或用户No都不发LIMITS，防止静默用旧Home采参考。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_limits_wrong_configuration_and_declined_confirmation_send_nothing(tmp_path):
    p, scope = limits_panel(tmp_path)
    p.cfg['calibration_pose_deg'] = [0,0,90,0,0,0]
    p.start_zero(); assert p.commands == []
    p.cfg['calibration_pose_deg'] = [0,-73,180,0,0,0]
    scope['QtWidgets'].QMessageBox.question = lambda *args: 0
    p.start_zero(); assert p.commands == []


##
# @brief 旧HOME_CONFIRMED事件不能触发后续运动，匹配LIMITS_CONFIRMED才发REST CONFIRM_MARGIN。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_only_matching_limits_confirmation_event_starts_rest(tmp_path):
    p, _ = limits_panel(tmp_path); p.zero_busy = True
    p.worker.readAllStandardOutput = lambda: b'home_rest_event=HOME_CONFIRMED_NO_MOTION\n'
    p.stdout_ready(); assert p.commands == []
    p.worker.readAllStandardOutput = lambda: b'home_rest_event=LIMITS_CONFIRMED_NO_MOTION\n'
    p.stdout_ready(); assert p.commands == ['REST CONFIRM_MARGIN']


##
# @brief 已有Hold先RELEASE，等只读状态及匹配pending_home_generation后才重发LIMITS。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_repeat_limits_waits_for_release_and_new_generation(tmp_path):
    p, _ = limits_panel(tmp_path, core=4); p.wait_home = False
    p.start_zero(); assert p.commands == ['RELEASE'] and p.pending_home
    p.wait_home = True; p.busy = False; p.authorized = False
    p.last_state['core_state'] = 2; p.last_state['session_id'] = 0
    p.last_state_ns = time.monotonic_ns(); p.tick()
    assert p.commands == ['RELEASE']
    p.worker_generation = p.pending_home_generation; p.tick()
    assert p.commands == ['RELEASE','LIMITS CONFIRM_MARGIN']


##
# @brief 边界RESUME说明只退危险J2下/J3上侧、其他轴冻结且不重新采零；不生成普通recovery目标。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_boundary_resume_explicitly_explains_original_rest_without_recapture(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.authorized = False; p.start_resume()
    assert p.commands == ['RESUME']
    assert '仅危险的J2下侧/J3上侧移动' in p.dialogs[-1][1] and '其余轴冻结' in p.dialogs[-1][1]
    assert '不重新采零' in p.dialogs[-1][1] and '工作Rest' in p.dialogs[-1][1]
    assert p.recovery_target() is None


##
# @brief Rest退让保留J1实际位置，仅J2/J3变动；Fault关闭所有控制且三个恢复入口均不发命令。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_manual_rest_recovery_freezes_other_axes_and_fault_still_denies(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][0] = .2
    target = p.manual_limits_retreat_target()
    assert target[0] == math.degrees(.2) and target[1:3] == p.cfg['rest_deg'][1:3]
    p.tick(); assert p.resume.enabled and not p.recover.enabled
    p.last_state['core_state'] = 9; p.tick()
    assert not any(getattr(p,x).enabled for x in ('zero','resume','move','recover'))
    p.start_zero(); p.start_resume(); p.start_recovery(); assert p.commands == []


##
# @brief 其它轴已超工作范围时不能借专用J2/J3退让放行RESUME。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_non_j2_j3_insufficient_margin_cannot_use_manual_rest_resume(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.limits[0] = (-10,10); p.last_state['position_rad'][0] = math.radians(12)
    assert p.manual_limits_retreat_target() is None
    p.start_resume(); assert p.commands == []


##
# @brief 仅J2危险时J2到Rest−60°，已安全J3保留50°。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_safe_j3_is_frozen_when_only_j2_lower_side_is_dangerous(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][2] = math.radians(50)
    target = p.manual_limits_retreat_target()
    assert target[1] == -60 and math.isclose(target[2], 50)


##
# @brief 仅J3危险时J3到原Rest，安全J2保留−30°。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_safe_j2_is_frozen_when_only_j3_upper_side_is_dangerous(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][1] = math.radians(-30)
    target = p.manual_limits_retreat_target()
    assert math.isclose(target[1], -30) and target[2] == p.cfg['rest_deg'][2]


##
# @brief J2上侧或J3下侧危险不属于专用退让方向，返回None。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_opposite_j2_upper_and_j3_lower_sides_cannot_request_manual_retreat(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][1] = math.radians(89)
    assert p.manual_limits_retreat_target() is None
    p.last_state['position_rad'][1] = math.radians(-73)
    p.last_state['position_rad'][2] = math.radians(36)
    assert p.manual_limits_retreat_target() is None


##
# @brief −73.09/−73.14°和180.09°可落入连续带，−73.18°拒绝；计算目标从不覆盖实际State。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_rest_reentry_continuity_band_does_not_change_actual_state(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][1] = math.radians(-73.09)
    p.last_state['position_rad'][2] = math.radians(180.09)
    actual = list(p.last_state['position_rad'])
    assert p.manual_limits_retreat_target() is not None
    assert p.last_state['position_rad'] == actual
    p.last_state['position_rad'][1] = math.radians(-73.14)
    assert p.manual_limits_retreat_target() is not None
    p.last_state['position_rad'][1] = math.radians(-73.18)
    assert p.manual_limits_retreat_target() is None


##
# @brief 当前WAIT后LIMITS被CAPTURE_MODE_REJECTED时退只读并清待Home/授权，不自动重试命令。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_current_capture_mode_reject_returns_readonly_without_retry(tmp_path):
    p, _ = limits_panel(tmp_path); p.authorized = False
    p.worker.readAllStandardOutput = lambda: b'home_rest_event=WAIT_MANUAL_HOME boot=7b reference=1 state_seq=10 session=0\n'
    p.stdout_ready(); p.start_zero()
    assert p.commands == ['LIMITS CONFIRM_MARGIN'] and p.busy and p.zero_busy
    p.worker.readAllStandardOutput = lambda: b'home_rest_event=CAPTURE_MODE_REJECTED boot=7b reference=1 state_seq=10 session=0\n'
    p.stdout_ready()
    assert p.wait_home and not p.busy and not p.zero_busy and not p.authorized
    assert p.pending_authorization is None and not p.pending_home and p.pending_home_generation is None
    assert p.commands == ['LIMITS CONFIRM_MARGIN']


##
# @brief 无当前WAIT、旧generation或Fault下的拒绝事件不得重建可操作状态。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_capture_mode_reject_without_current_wait_or_in_fault_does_not_restore_controls(tmp_path):
    p, _ = limits_panel(tmp_path); p.zero_busy = p.busy = True; p.wait_home = False
    p.worker.readAllStandardOutput = lambda: b'home_rest_event=CAPTURE_MODE_REJECTED boot=7b reference=1 state_seq=10 session=0\n'
    p.stdout_ready(); assert not p.wait_home
    p.readonly_wait_generation = p.worker_generation - 1; p.readonly_wait_identity = (123,1,10)
    p.stdout_ready(); assert not p.wait_home
    p.readonly_wait_generation = p.worker_generation; p.last_state['core_state'] = 9
    p.stdout_ready(); p.tick()
    assert not p.wait_home and not p.authorized and p.pending_authorization is None
    assert not any(getattr(p,x).enabled for x in ('zero','resume','move','recover'))
    assert p.commands == []


##
# @brief 半度working边界下预览仍用原J2/J3 Rest，其余四轴冻结且State原样保留。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_half_degree_restore_preview_uses_original_rest_contract_for_both_axes(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.limits[1] = (-72.5, 89.5); p.limits[2] = (35.5, 179.5)
    p.last_state['position_rad'] = [.1, math.radians(-73), math.radians(179.5), .2, -.1, .3]
    actual = list(p.last_state['position_rad'])
    target = p.manual_limits_retreat_target()
    assert target[1:3] == p.cfg['rest_deg'][1:3]
    assert all(target[i] == math.degrees(actual[i]) for i in (0, 3, 4, 5))
    assert p.last_state['position_rad'] == actual and p.commands == []
    p.last_state['position_rad'][1] = math.radians(-72.5)
    p.last_state['position_rad'][2] = math.radians(180)
    assert p.manual_limits_retreat_target()[1:3] == p.cfg['rest_deg'][1:3]


##
# @brief 即使点已在普通范围，J2/J3靠止挡13°触发区仍按原Rest预览；这里只核面板目标，不执行Native退让。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_restore_preview_matches_native_thirteen_degree_trigger_inside_ordinary_range(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.limits[1] = (-72.5, 89.5); p.limits[2] = (35.5, 179.5)
    for j2, j3 in ((-72.9, 179.5), (-70, 175)):
        p.last_state['position_rad'] = [.1, math.radians(j2), math.radians(j3), .2, -.1, .3]
        actual = list(p.last_state['position_rad'])
        if j2 == -70: assert p.resume_position_issue() is None
        target = p.manual_limits_retreat_target()
        assert target[1:3] == p.cfg['rest_deg'][1:3]
        assert all(target[i] == math.degrees(actual[i]) for i in (0, 3, 4, 5))
        assert p.last_state['position_rad'] == actual and p.commands == []
