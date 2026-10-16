import importlib.util
from pathlib import Path
import pytest


def module():
    spec = importlib.util.spec_from_file_location('run_native_m3_functional', Path(__file__).parents[1] / 'run_native_m3_functional.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_stage_counts_and_pairing():
    m = module()
    assert [(x['kind'], x['count'], x['reset']) for x in m.plan('normal')] == [
        ('normal', 50, False), ('normal', 10, True), ('normal', 10, True), ('normal', 10, True)]
    assert len(m.plan('long')) == 3
    assert [x['kind'] for x in m.plan('all')] == ['normal', 'normal', 'long', 'normal', 'long', 'normal', 'long']


def test_group_stops_on_first_failure(tmp_path):
    m = module()
    calls = []
    def case(item, path):
        calls.append(item['id'])
        if len(calls) == 2: raise RuntimeError('first injected failure')
        return {'result': 'FUNCTIONAL_POSITIVE', 'boot_epoch': 'aa', 'qualified': False}
    report = m.run_group(m.plan('normal'), tmp_path, case)
    assert report['result'] == 'FAIL'
    assert len(calls) == 2
    assert len(report['cases']) == 2
    assert 'first injected failure' in report['first_failure']
    assert report['qualified'] is False


def test_new_boot_must_differ_from_prior_boots(tmp_path):
    m = module()
    calls = []
    def case(item, path):
        calls.append(item['id'])
        return {'result': 'FUNCTIONAL_POSITIVE', 'boot_epoch': 'same', 'qualified': False}
    result = m.run_group(m.plan('normal'), tmp_path, case)
    assert result['result'] == 'FAIL'
    assert len(calls) == 2
    assert 'boot' in result['first_failure']


@pytest.mark.parametrize('count', [49, 51])
def test_count_mismatch_rejected(count):
    m = module()
    with pytest.raises(ValueError):
        m.validate_case_result({'status':'SUCCEEDED','completed_goals':count,'requested_goals':50,
                                'stop_confirmation':'CONFIRMED_RELEASED_IDLE'}, 'COMPLETE,50\n',
                               '[device_action_gate-1]: process has finished cleanly',
                               {'status':'ROS_BENCH_E2E_POSITIVE','goals':50}, 50)


def test_missing_or_wrong_maps_rejected():
    m = module()
    for maps in ['', 'aa-bb r-xp 0 0 0 /old/install/libdummy_guarded_zenbedded_system.so\n']:
        with pytest.raises(ValueError): m.mapped_library(maps)


def test_clean_gate_and_complete_count_required():
    m = module()
    action={'status':'SUCCEEDED','completed_goals':50,'requested_goals':50,
            'stop_confirmation':'CONFIRMED_RELEASED_IDLE'}
    e2e={'status':'ROS_BENCH_E2E_POSITIVE','goals':50}
    for timeline, log in [('COMPLETE,49\n','[device_action_gate-1]: process has finished cleanly'),
                          ('COMPLETE,50\n','[device_action_gate-1]: process has died')]:
        with pytest.raises(ValueError): m.validate_case_result(action,timeline,log,e2e,50)
    m.validate_case_result(action,'COMPLETE,50\n','[device_action_gate-1]: process has finished cleanly\n[move_group-5]: process has died',e2e,50)


def test_dry_plan_never_starts_subprocess(tmp_path, monkeypatch, capsys):
    m = module()
    def forbidden(*args, **kw): raise AssertionError('dry touched process/hardware boundary')
    monkeypatch.setattr(m.subprocess, 'run', forbidden)
    monkeypatch.setattr(m.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(m.os, 'killpg', forbidden)
    output = tmp_path / 'must-not-exist'
    monkeypatch.setattr(m.sys if hasattr(m, 'sys') else __import__('sys'), 'argv',
                        ['run_native_m3_functional', '--stage', 'all', '--output', str(output)])
    assert m.main() == 0
    assert 'PLAN_ONLY' in capsys.readouterr().out
    assert not output.exists()


@pytest.mark.parametrize('signal_marker', [True, False])
def test_capture_window_marker_or_timeout_bounded_before_close(tmp_path, signal_marker):
    m = module()
    root = tmp_path / 'case'; root.mkdir()
    now = [0.0]
    events = []
    def sleep(seconds):
        now[0] += seconds
        if signal_marker and now[0] >= .5:
            (root / 'failure-capture.done').touch()
    result = m.failure_capture_window(root, 123, 'failure', monotonic=lambda: now[0],
                                      sleep=sleep, emit=lambda item: events.append(item['status']))
    events.append('CLOSE')
    assert events == ['FAILURE_CAPTURE_PENDING', 'CLOSE']
    assert result['marker_seen'] is signal_marker
    assert now[0] == (.5 if signal_marker else 60)
    assert __import__('json').loads((tmp_path/'group_progress.json').read_text())['gate_pid'] == 123


@pytest.mark.parametrize("capture_interrupted", [False, True])
def test_action_failure_capture_precedes_owner_shutdown(tmp_path, monkeypatch, capture_interrupted):
    m = module()
    freeze = tmp_path / 'freeze.json'; freeze.write_text('{}')
    monkeypatch.setattr(m, 'FREEZE', freeze)
    native = m.NativeRun.__new__(m.NativeRun)
    native.freeze = {}; native.seen_boots = set()
    native.check_unchanged = lambda: None
    native.probe = lambda root, name: {'boot_epoch':'aa'}
    events = []
    class Process:
        def poll(self): return None
        def send_signal(self, value): events.append('CLOSE')
        def wait(self, timeout): return 1
    monkeypatch.setattr(m.subprocess, 'Popen', lambda *a, **k: Process())
    def runtime(process, log, root):
        m.save(root/'gate-runtime.json', {'pid':123})
        return 123
    native.runtime = runtime
    def command(argv, root, name, timeout):
        if name == 'action': raise RuntimeError('injected first action failure')
        return "success=True, message='Enable queued; wait for confirmed HOLD before submitting motion'" if name.startswith("enable") else "success=True, message='ENABLED_HOLD'"
    native.command = command
    def capture(root, pid, reason):
        events.append('FAILURE_CAPTURE_PENDING')
        assert 'action failure' in str(reason)
        if capture_interrupted:
            raise KeyboardInterrupt('取证窗口被中断')
        return {'marker_seen':True}
    monkeypatch.setattr(m, 'failure_capture_window', capture)
    with pytest.raises(RuntimeError, match='first action failure'):
        native({'id':'normal-50','kind':'normal','count':50,'reset':False}, tmp_path/'case')
    assert events == ['FAILURE_CAPTURE_PENDING', 'CLOSE']
    result = __import__('json').loads((tmp_path/'case/metadata.json').read_text())
    assert result['first_failure'] == 'injected first action failure'
    assert result['result'] == 'FAIL'


def test_reused_boot_refused_before_launch(tmp_path, monkeypatch):
    m = module()
    freeze = tmp_path/'freeze.json'; freeze.write_text('{}'); monkeypatch.setattr(m, 'FREEZE', freeze)
    native = m.NativeRun.__new__(m.NativeRun)
    native.freeze = {'firmware':{'path':'frozen.bin'}}; native.seen_boots = {'aa'}
    native.owner = type('Owner', (), {'inspect_owners':staticmethod(lambda tty:[])})()
    native.check_unchanged = lambda: None
    native.probe = lambda root, name: {'boot_epoch':'aa'}
    native.command = lambda *a, **k: ''
    def forbidden(*args, **kw): raise AssertionError('reused boot reached launch')
    monkeypatch.setattr(m.subprocess, 'Popen', forbidden)
    with pytest.raises(ValueError, match='before motion'):
        native({'id':'boot-1-10','kind':'normal','count':10,'reset':True},tmp_path/'case')


def test_offline_strict_failure_occurs_after_close_without_live_claim(tmp_path, monkeypatch):
    m = module()
    freeze = tmp_path / 'freeze.json'; freeze.write_text('{}'); monkeypatch.setattr(m, 'FREEZE', freeze)
    native = m.NativeRun.__new__(m.NativeRun)
    native.freeze = {}; native.seen_boots = set()
    native.check_unchanged = lambda: None
    native.probe = lambda root, name: {'boot_epoch':'aa'}
    events = []
    class Process:
        def poll(self): return None
        def send_signal(self, value): events.append('CLOSE')
        def wait(self, timeout): return 1
    monkeypatch.setattr(m.subprocess, 'Popen', lambda *a, **k: Process())
    def runtime(process, log, root):
        m.save(root/'gate-runtime.json', {'pid':123})
        return 123
    native.runtime = runtime
    native.command = lambda argv, root, name, timeout: "success=True, message='Enable queued; wait for confirmed HOLD before submitting motion'" if name.startswith("enable") else "success=True, message='ENABLED_HOLD'"
    def strict(root, item):
        events.append('STRICT')
        raise ValueError('真实静态strict反例')
    native.strict = strict
    def forbidden(*args): raise AssertionError('strict failure tried live capture after close')
    monkeypatch.setattr(m, 'failure_capture_window', forbidden)
    with pytest.raises(ValueError, match='strict反例'):
        native({'id':'normal-50','kind':'normal','count':50,'reset':False}, tmp_path/'case')
    assert events == ['CLOSE', 'STRICT']
    result = __import__('json').loads((tmp_path/'case/metadata.json').read_text())
    assert result['live_snapshot'] == 'LIVE_SNAPSHOT_NOT_AVAILABLE'
    assert result['qualified'] is False


@pytest.mark.parametrize('mutation', [None, ('core_state',9), ('deadline_miss_count',1),
                                      ('session_id',1), ('reference_epoch',1), ('permit_remaining_us',1)])
def test_fixed_probe_fixture_requires_idle_and_zero_deadlines(tmp_path, mutation):
    m = module()
    native = m.NativeRun.__new__(m.NativeRun)
    native.owner = type('Owner', (), {'inspect_owners':staticmethod(lambda tty:[])})()
    native.freeze = {'identity_from_activation':{'source_contract_sha256':'contract',
        'model_sha256':'model', 'compiled_policy_sha256':'policy'}}
    value = dict(contract_sha256='contract',model_sha256='model',profile_sha256='policy',
                 result='PASS',profile_id=201,configured_mask=63,core_state=1,session_id=0,
                 reference_epoch=0,permit_remaining_us=0,real_motion_permitted=False,
                 command_reject_count=0,can_error_count=0,rx_overflow=0,tx_event_overflow=0,
                 deadline_miss_count=0,bus_off=False,image_sha256=m.FIRMWARE,boot_epoch='aa')
    if mutation: value[mutation[0]] = mutation[1]
    argv_seen = []
    def command(argv, root, label, timeout):
        argv_seen.extend(argv)
        return __import__('json').dumps(value)
    native.command = command
    if mutation:
        with pytest.raises(ValueError,match='health/identity'):
            native.probe(tmp_path,'preprobe')
    else:
        assert native.probe(tmp_path,'preprobe')['boot_epoch'] == 'aa'
    assert argv_seen == [m.PROBE, m.ENDPOINT, '5000']
    assert 'p9-nonblocking-host-20261006' in str(m.PROBE)


def test_enable_startup_denial_retries_until_api_accepts_then_waits_hold(tmp_path):
    m = module(); native = m.NativeRun.__new__(m.NativeRun)
    replies = ["success=False, message='Enable unavailable phase=0 transport=ready boot=0 raw=not_ready'",
               "success=True, message='Enable queued; wait for confirmed HOLD before submitting motion'",
               "success=False, message='BOOTSTRAP_PENDING'",
               "success=True, message='ENABLED_HOLD'"]
    seen = []
    def command(argv, root, name, timeout):
        seen.append(argv[3]); return replies.pop(0)
    native.command = command
    native.enable_hold(tmp_path, sleep=lambda seconds:None)
    assert seen == ['/device_action_gate/enable','/device_action_gate/enable',
                    '/device_action_gate/bootstrap_status','/device_action_gate/bootstrap_status']
    assert not replies


@pytest.mark.parametrize('reply', ["success=False, message='Enable unavailable phase=9 transport=ready boot=1 raw=ready'",
                                  "success=True, message='unexpected'",
                                  "success=False, message='FAULT'"])
def test_enable_startup_never_retries_other_failures(tmp_path, reply):
    m = module(); native = m.NativeRun.__new__(m.NativeRun); seen=[]
    native.command = lambda *args: (seen.append(args), reply)[1]
    with pytest.raises(ValueError): native.enable_hold(tmp_path, sleep=lambda seconds:None)
    assert len(seen)==1


@pytest.mark.parametrize('reply', ["success=False, message='Enable unavailable FAULT True'",
                                  "success=False, message='Enable queued; wait for confirmed HOLD before submitting motion'",
                                  "success=True arbitrary unrelated text",
                                  "success=True, message='Enable queued; wait for confirmed HOLD before submitting motion' success=False, message='FAULT'"])
def test_enable_parsing_never_accepts_false_or_ambiguous_text(tmp_path, reply):
    m=module(); native=m.NativeRun.__new__(m.NativeRun)
    native.command=lambda *args:reply
    with pytest.raises(ValueError): native.enable_hold(tmp_path,sleep=lambda seconds:None)


def test_enable_retry_stays_inside_original_20_second_budget(tmp_path):
    m=module(); native=m.NativeRun.__new__(m.NativeRun); now=[0.0];calls=[]
    def command(argv, root, name, timeout):
        calls.append(timeout)
        return "success=False, message='Enable unavailable phase=0 transport=ready boot=0 raw=not_ready'"
    native.command=command
    with pytest.raises(TimeoutError):
        native.enable_hold(tmp_path,monotonic=lambda:now[0],sleep=lambda seconds:now.__setitem__(0,now[0]+seconds))
    assert now[0] == 20.0 and max(calls)<=20


@pytest.mark.parametrize('mutation', ['old_bytes', 'wrong_inode', 'old_path'])
def test_runtime_gate_must_match_verified_candidate_executable(tmp_path, mutation):
    m = module()
    gate = tmp_path / 'device_action_gate'; gate.write_bytes(b'candidate-v2')
    freeze = {'path': str(gate), 'sha256': m.sha(gate)}
    exe = gate
    inode = gate.stat().st_ino
    if mutation == 'old_bytes': gate.write_bytes(b'old-candidate')
    if mutation == 'wrong_inode': inode += 1
    if mutation == 'old_path': exe = tmp_path / 'old_gate'; exe.write_bytes(b'candidate-v2')
    validator = getattr(m, 'validate_runtime_gate', None)
    assert callable(validator), 'runtime Gate executable校验缺失'
    with pytest.raises(ValueError): validator(exe, inode, freeze)


def test_runtime_gate_accepts_install_symlink_to_exact_build_target(tmp_path):
    m = module()
    target = tmp_path / 'build_gate'; target.write_bytes(b'candidate-v2')
    entry = tmp_path / 'install_gate'; entry.symlink_to(target)
    validator = getattr(m, 'validate_runtime_gate', None)
    assert callable(validator), 'runtime Gate executable校验缺失'
    result = validator(target, target.stat().st_ino,
                       {'path': str(entry), 'sha256': m.sha(entry)})
    assert result['sha256'] == m.sha(target)


def test_frozen_mode_change_with_identical_bytes_is_rejected(tmp_path):
    m=module(); script=tmp_path/'probe.py'; script.write_bytes(b'#!/usr/bin/python3\n')
    script.chmod(0o755)
    expected={'path':str(script),'sha256':m.sha(script),'mode':'0o755'}
    validator=getattr(m,'verify_frozen_artifact',None)
    assert callable(validator), 'freeze执行位校验缺失'
    validator(expected)
    script.chmod(0o644)
    assert m.sha(script)==expected['sha256']
    with pytest.raises(ValueError,match='mode'): validator(expected)


def test_group_guard_rejects_mode_change_through_install_symlink(tmp_path):
    m=module(); target=tmp_path/'target'; target.write_bytes(b'samebytes'); target.chmod(0o755)
    entry=tmp_path/'entry'; entry.symlink_to(target)
    native=m.NativeRun.__new__(m.NativeRun)
    native.hashes={str(entry):m.sha(entry)}
    native.modes={str(entry):oct(entry.stat().st_mode & 0o777)}
    native.check_unchanged()
    target.chmod(0o644)
    with pytest.raises(ValueError,match='mode'): native.check_unchanged()


def test_d_test_reference_files_are_part_of_frozen_input_guard(tmp_path):
    m=module(); header=tmp_path/'wire_operation.hpp'; header.write_text('formal mapping')
    source=tmp_path/'controller.cpp'; source.write_text('controller')
    frozen={'source_files':{'controller':{'path':str(source),'sha256':m.sha(source),'mode':oct(source.stat().st_mode&0o777)}},
            'd_test_reference_files':{'mapping':{'path':str(header),'sha256':m.sha(header),'mode':oct(header.stat().st_mode&0o777)}}}
    collect=getattr(m,'frozen_source_artifacts',None)
    assert callable(collect), 'D正式mapper依赖未纳入freeze验证'
    artifacts=collect(frozen)
    assert {value['path'] for value in artifacts}=={str(source),str(header)}
    header.write_text('changed formal mapping')
    with pytest.raises(ValueError,match='hash'):
        for value in artifacts: m.verify_frozen_artifact(value)
