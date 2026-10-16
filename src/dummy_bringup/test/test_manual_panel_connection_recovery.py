"""Execute production connection lifecycle methods offline; no ROS/USB/Native launch."""
from types import SimpleNamespace
from test_manual_panel_fault_readonly import panel


def connection_panel(tmp_path, core=9, running=False):
    p, scope = panel(tmp_path, core)
    p.pending_home = False; p.authorized = False; p.pending_authorization = None
    p.busy = False; p.wait_home = False; p.readonly_fault = core == 9
    p.connection_action = None; p.release_confirmed = False; p.manage_generation = 0
    p.raw = tmp_path/'transport.raw'; p.worker.state = lambda: 2 if running else 0
    p.manage_worker = SimpleNamespace(state=lambda: 0, start=lambda *args: p.starts.append(('helper', args)))
    p.starts = []; p.start_worker = lambda: p.starts.append(('readonly',))
    scope['QtCore'].QTimer = SimpleNamespace(singleShot=lambda _, callback: callback())
    scope['QtCore'].QProcess.FailedToStart = 0
    p.cfg.update(native_binary='/offline/dummy_joint_cli', native_arguments=[
        '--scenario','home-rest','--execute','--endpoint','serial/by-id/fixed','--client','aa',
        '--device','bb','--profile','201','--mask','63','--scope','manual',
        '--firmware','ff','--contract','cc','--model','dd','--profile-sha256','ee'])
    return p, scope


def test_exited_worker_reconnects_only_readonly_without_motion(tmp_path):
    p, _ = connection_panel(tmp_path); p.request_reconnect()
    assert p.starts == [('readonly',)] and p.commands == []
    assert not p.pending_home and not p.authorized and p.pending_authorization is None


def test_live_fault_handoff_waits_for_finished_and_no_wait_home_needed(tmp_path):
    p, _ = connection_panel(tmp_path, running=True); p.request_clear_fault()
    assert p.commands == ['RELEASE'] and p.starts == [] and p.connection_action == 'clear_fault'
    p.worker.state = lambda: 0; p.finished(0,0)
    assert len(p.starts) == 1 and p.starts[0][0] == 'helper'
    args = p.starts[0][1][1]
    assert '--manage-probe' in args and '--execute' in args
    assert '--scenario' not in args and '--panel' not in args
    assert args[args.index('--target')+1] == '0,0,0,0,0,0'
    assert p.commands == ['RELEASE'] and not p.authorized


def test_unconfirmed_owner_exit_never_starts_helper(tmp_path):
    p, _ = connection_panel(tmp_path, core=4, running=True)
    p.queue_connection_action('reconnect'); p.worker.state = lambda: 0; p.finished(7,0)
    assert p.starts == [] and p.connection_action is None


def test_helper_failure_reconnects_readonly_without_auto_resume(tmp_path):
    p, _ = connection_panel(tmp_path); p.request_clear_fault()
    assert p.starts[0][0] == 'helper'
    p.manage_finished(50,0)
    assert p.starts[-1] == ('readonly',) and p.commands == []
    assert not p.authorized and not p.pending_home


def test_helper_active_blocks_reconnect_clear_and_worker_start(tmp_path):
    p, _ = connection_panel(tmp_path); p.manage_worker.state = lambda: 2
    p.request_reconnect(); p.request_clear_fault(); p.run_connection_action()
    assert p.starts == [] and p.commands == []


def test_fault_clear_decline_or_not_fault_or_owned_sends_nothing(tmp_path):
    p, scope = connection_panel(tmp_path); scope['QtWidgets'].QMessageBox.question = lambda *args: 0
    p.request_clear_fault(); assert p.starts == []
    scope['QtWidgets'].QMessageBox.question = lambda *args: 1
    p.last_state['session_id'] = 77; p.request_clear_fault(); assert p.starts == []
    p.last_state['session_id'] = 0; p.last_state['core_state'] = 2
    p.request_clear_fault(); assert p.starts == [] and p.commands == []


def test_boot_change_observation_clears_old_reference_not_rebuild_it(tmp_path):
    p, _ = connection_panel(tmp_path, core=2)
    import time
    state = dict(p.last_state, boot_epoch=124, reference_epoch=0, state_seq=1, position_valid_mask=0)
    assert p.accept_state(state, time.monotonic_ns()); p.reconcile_control_state()
    assert p.calibration is None and not p.authorized and p.commands == []


def test_helper_failed_start_and_close_do_not_auto_enable(tmp_path):
    p, _ = connection_panel(tmp_path); p.manage_error(0)
    assert p.starts == [('readonly',)] and p.commands == []
    p.closing = True; p.manage_finished(0,0)
    assert p.starts == [('readonly',)]


def test_close_waits_for_helper_and_does_not_restart_readonly(tmp_path):
    p, _ = connection_panel(tmp_path)
    active = [True]; waits = []
    p.manage_worker.state = lambda: 2 if active[0] else 0
    p.manage_worker.terminate = lambda: active.__setitem__(0, False)
    p.manage_worker.waitForFinished = waits.append
    event = SimpleNamespace(accept=lambda: waits.append('accepted'), ignore=lambda: waits.append('ignored'))
    p.closeEvent(event)
    assert waits == [6000, 'accepted'] and not active[0] and p.closing
    assert p.starts == [] and p.commands == []


def test_fault_readonly_stop_normal_exit_restarts_only_readonly(tmp_path):
    p, _ = connection_panel(tmp_path, running=True)
    p.pending_home = True; p.pending_home_generation = 77
    p.release('STOP'); p.worker.state = lambda: 0; p.finished(0, 0)
    assert p.starts == [('readonly',)] and p.commands == ['STOP']
    assert not p.authorized and p.pending_authorization is None and not p.pending_home
    assert p.pending_home_generation is None


def test_fault_readonly_release_normal_exit_restarts_only_readonly(tmp_path):
    p, _ = connection_panel(tmp_path, running=True)
    p.release('RELEASE'); p.worker.state = lambda: 0; p.finished(0, 0)
    assert p.starts == [('readonly',)] and p.commands == ['RELEASE']


def test_unconfirmed_nonfault_stop_never_auto_handoffs(tmp_path):
    p, _ = connection_panel(tmp_path, core=4, running=True)
    p.release('STOP'); p.worker.state = lambda: 0; p.finished(0, 0)
    assert p.starts == [] and p.commands == ['STOP']


def test_fault_stop_stale_owned_or_failed_exit_never_auto_handoffs(tmp_path):
    for kind in ('stale', 'owned', 'authorized', 'failed', 'closing'):
        p, _ = connection_panel(tmp_path, running=True)
        if kind == 'stale': p.fresh = lambda: False
        p.release('STOP'); p.worker.state = lambda: 0
        if kind == 'owned': p.last_state['session_id'] = 77
        if kind == 'authorized': p.authorized = True
        if kind == 'closing': p.closing = True
        p.finished(7 if kind == 'failed' else 0, 0)
        assert p.starts == [] and p.commands == ['STOP']


def test_clear_fault_handoff_remembers_fresh_request_through_repeat_fault_and_stale_exit(tmp_path):
    p, _ = connection_panel(tmp_path, running=True)
    p.request_clear_fault()
    p.enter_readonly_fault((123, 10)); p.enter_readonly_fault((123, 10))
    p.fresh = lambda: False; p.worker.state = lambda: 0; p.finished(0, 0)
    assert len(p.starts) == 1 and p.starts[0][0] == 'helper'
    assert p.commands == ['RELEASE'] and not p.authorized and not p.pending_home


def test_stop_handoff_remembers_fresh_request_through_repeat_fault_and_stale_exit(tmp_path):
    p, _ = connection_panel(tmp_path, running=True)
    p.release('STOP'); p.enter_readonly_fault((123, 10)); p.enter_readonly_fault((123, 10))
    p.fresh = lambda: False; p.worker.state = lambda: 0; p.finished(0, 0)
    assert p.starts == [('readonly',)] and p.commands == ['STOP']
    assert not p.authorized and not p.pending_home


def test_cached_fault_handoff_refuses_changed_generation_owner_authority_or_failure(tmp_path):
    for kind in ('generation', 'boot', 'reference', 'owner', 'authorized_at_request', 'failed'):
        p, _ = connection_panel(tmp_path, running=True)
        if kind == 'owner': p.last_state['session_id'] = 77
        if kind == 'authorized_at_request': p.authorized = True
        p.release('STOP')
        if kind == 'generation': p.worker_generation += 1
        if kind == 'boot': p.last_state['boot_epoch'] += 1
        if kind == 'reference': p.last_state['reference_epoch'] += 1
        p.enter_readonly_fault((123, 10)); p.fresh = lambda: False; p.worker.state = lambda: 0
        p.finished(7 if kind == 'failed' else 0, 0)
        assert p.starts == [] and p.commands == ['STOP']
