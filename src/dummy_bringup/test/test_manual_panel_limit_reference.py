"""人工双限位GUI事件/原方法离线验证，不构造Native或USB。"""
import math
import time
from test_manual_panel_fault_readonly import panel


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
    def confirm(parent, title, text, *args): p.dialogs.append((title,text)); return 1
    scope['QtWidgets'].QMessageBox.question = confirm
    return p, scope


def test_limits_initial_confirm_uses_limits_command_not_old_home(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.start_zero()
    assert p.commands == ['LIMITS CONFIRM_MARGIN']
    assert '双限位' in p.dialogs[0][0] and '−73' not in p.dialogs[0][0]
    assert '不是旧Home' in p.dialogs[0][1] and '180' in p.dialogs[0][1]
    assert p.cfg['home_deg'] == [0,0,90,0,0,0]


def test_limits_wrong_configuration_and_declined_confirmation_send_nothing(tmp_path):
    p, scope = limits_panel(tmp_path)
    p.cfg['calibration_pose_deg'] = [0,0,90,0,0,0]
    p.start_zero(); assert p.commands == []
    p.cfg['calibration_pose_deg'] = [0,-73,180,0,0,0]
    scope['QtWidgets'].QMessageBox.question = lambda *args: 0
    p.start_zero(); assert p.commands == []


def test_only_matching_limits_confirmation_event_starts_rest(tmp_path):
    p, _ = limits_panel(tmp_path); p.zero_busy = True
    p.worker.readAllStandardOutput = lambda: b'home_rest_event=HOME_CONFIRMED_NO_MOTION\n'
    p.stdout_ready(); assert p.commands == []
    p.worker.readAllStandardOutput = lambda: b'home_rest_event=LIMITS_CONFIRMED_NO_MOTION\n'
    p.stdout_ready(); assert p.commands == ['REST CONFIRM_MARGIN']


def test_repeat_limits_waits_for_release_and_new_generation(tmp_path):
    p, _ = limits_panel(tmp_path, core=4); p.wait_home = False
    p.start_zero(); assert p.commands == ['RELEASE'] and p.pending_home
    p.wait_home = True; p.busy = False; p.authorized = False
    p.last_state['core_state'] = 2; p.last_state['session_id'] = 0
    p.last_state_ns = time.monotonic_ns(); p.tick()
    assert p.commands == ['RELEASE']
    p.worker_generation = p.pending_home_generation; p.tick()
    assert p.commands == ['RELEASE','LIMITS CONFIRM_MARGIN']


def test_boundary_resume_explicitly_explains_original_rest_without_recapture(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.authorized = False; p.start_resume()
    assert p.commands == ['RESUME']
    assert '仅危险的J2下侧/J3上侧移动' in p.dialogs[-1][1] and '其余轴冻结' in p.dialogs[-1][1]
    assert '不重新采零' in p.dialogs[-1][1] and '工作Rest' in p.dialogs[-1][1]
    assert p.recovery_target() is None


def test_manual_rest_recovery_freezes_other_axes_and_fault_still_denies(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][0] = .2
    target = p.manual_limits_retreat_target()
    assert target[0] == math.degrees(.2) and target[1:3] == p.cfg['rest_deg'][1:3]
    p.tick(); assert p.resume.enabled and not p.recover.enabled
    p.last_state['core_state'] = 9; p.tick()
    assert not any(getattr(p,x).enabled for x in ('zero','resume','move','recover'))
    p.start_zero(); p.start_resume(); p.start_recovery(); assert p.commands == []


def test_non_j2_j3_insufficient_margin_cannot_use_manual_rest_resume(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.limits[0] = (-10,10); p.last_state['position_rad'][0] = math.radians(12)
    assert p.manual_limits_retreat_target() is None
    p.start_resume(); assert p.commands == []


def test_safe_j3_is_frozen_when_only_j2_lower_side_is_dangerous(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][2] = math.radians(50)
    target = p.manual_limits_retreat_target()
    assert target[1] == -60 and math.isclose(target[2], 50)


def test_safe_j2_is_frozen_when_only_j3_upper_side_is_dangerous(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][1] = math.radians(-30)
    target = p.manual_limits_retreat_target()
    assert math.isclose(target[1], -30) and target[2] == p.cfg['rest_deg'][2]


def test_opposite_j2_upper_and_j3_lower_sides_cannot_request_manual_retreat(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][1] = math.radians(89)
    assert p.manual_limits_retreat_target() is None
    p.last_state['position_rad'][1] = math.radians(-73)
    p.last_state['position_rad'][2] = math.radians(36)
    assert p.manual_limits_retreat_target() is None


def test_original_feedback_band_allowed_without_changing_actual_state(tmp_path):
    p, _ = limits_panel(tmp_path)
    p.last_state['position_rad'][1] = math.radians(-73.09)
    p.last_state['position_rad'][2] = math.radians(180.09)
    actual = list(p.last_state['position_rad'])
    assert p.manual_limits_retreat_target() is not None
    assert p.last_state['position_rad'] == actual
    p.last_state['position_rad'][1] = math.radians(-73.11)
    assert p.manual_limits_retreat_target() is None


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
