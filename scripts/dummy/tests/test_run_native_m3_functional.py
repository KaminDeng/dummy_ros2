import importlib.util
##
# @file test_run_native_m3_functional.py
# @brief Native M3分组计划、冻结产物、失败取证及Enable时序的离线测试。
#
# NativeRun多数以__new__跳构造，probe/command/Popen/时钟替身只写临时文件和事件，不连USB或启动真实Gate。FUNCTIONAL_POSITIVE是合成case输入，qualified始终false；固定probe路径/固件常量/旧freeze是已有基线，不证明当前设备验证。
from pathlib import Path
import pytest


##
# @brief 按源码路径加载runner模块，不调用其main或NativeRun构造。
# @return 已加载runner模块，不运行main。
def module():
    spec = importlib.util.spec_from_file_location('run_native_m3_functional', Path(__file__).parents[1] / 'run_native_m3_functional.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


##
# @brief 核normal50+三次新boot10、long三段及all交错顺序，纯计划形状。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_stage_counts_and_pairing():
    m = module()
    assert [(x['kind'], x['count'], x['reset']) for x in m.plan('normal')] == [
        ('normal', 50, False), ('normal', 10, True), ('normal', 10, True), ('normal', 10, True)]
    assert len(m.plan('long')) == 3
    assert [x['kind'] for x in m.plan('all')] == ['normal', 'normal', 'long', 'normal', 'long', 'normal', 'long']


##
# @brief 第二fake case异常立即停组，保首失败/两个已尝试记录且qualified=false。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_group_stops_on_first_failure(tmp_path):
    m = module()
    calls = []
    ##
    # @par 内部回调 case
    # 记录分组case ID并返回人工FUNCTIONAL_POSITIVE/boot/qualified=false，按外层指定第2例抛错或重复boot。
    # 参数 item：当前分组case计划字典，含id/kind/count/reset。
    # 参数 path：临时文件/目录Path或/proc占位；本回调不真实采设备。
    # 返回：人工FUNCTIONAL_POSITIVE报告，或第二次注入失败。
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


##
# @brief reset阶段复用same boot字符串在第二例拒绝，不当作新boot功能通过。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_new_boot_must_differ_from_prior_boots(tmp_path):
    m = module()
    calls = []
    ##
    # @par 内部回调 case
    # 记录分组case ID并返回人工FUNCTIONAL_POSITIVE/boot/qualified=false，按外层指定第2例抛错或重复boot。
    # 参数 item：当前分组case计划字典，含id/kind/count/reset。
    # 参数 path：临时文件/目录Path或/proc占位；本回调不真实采设备。
    # 返回：人工FUNCTIONAL_POSITIVE报告，或第二次注入失败。
    def case(item, path):
        calls.append(item['id'])
        return {'result': 'FUNCTIONAL_POSITIVE', 'boot_epoch': 'same', 'qualified': False}
    result = m.run_group(m.plan('normal'), tmp_path, case)
    assert result['result'] == 'FAIL'
    assert len(calls) == 2
    assert 'boot' in result['first_failure']


##
# @brief action完成49/51与计划50不符拒绝，已有timeline/positive标签不能补计数。
# @param count 要注入的完成目标数，49/51对比计划50。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize('count', [49, 51])
def test_count_mismatch_rejected(count):
    m = module()
    with pytest.raises(ValueError):
        m.validate_case_result({'status':'SUCCEEDED','completed_goals':count,'requested_goals':50,
                                'stop_confirmation':'CONFIRMED_RELEASED_IDLE'}, 'COMPLETE,50\n',
                               '[device_action_gate-1]: process has finished cleanly',
                               {'status':'ROS_BENCH_E2E_POSITIVE','goals':50}, 50)


##
# @brief 空maps或旧install库路径不满足当前mapped_library来源约束。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_missing_or_wrong_maps_rejected():
    m = module()
    for maps in ['', 'aa-bb r-xp 0 0 0 /old/install/libdummy_guarded_zenbedded_system.so\n']:
        with pytest.raises(ValueError): m.mapped_library(maps)


##
# @brief timeline数错或Gate died拒绝；仅move_group died不属于本函数要求的Gate干净退出，不能扩大成全栈无错。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_clean_gate_and_complete_count_required():
    m = module()
    action={'status':'SUCCEEDED','completed_goals':50,'requested_goals':50,
            'stop_confirmation':'CONFIRMED_RELEASED_IDLE'}
    e2e={'status':'ROS_BENCH_E2E_POSITIVE','goals':50}
    for timeline, log in [('COMPLETE,49\n','[device_action_gate-1]: process has finished cleanly'),
                          ('COMPLETE,50\n','[device_action_gate-1]: process has died')]:
        with pytest.raises(ValueError): m.validate_case_result(action,timeline,log,e2e,50)
    m.validate_case_result(action,'COMPLETE,50\n','[device_action_gate-1]: process has finished cleanly\n[move_group-5]: process has died',e2e,50)


##
# @brief 替换run/Popen/killpg为必失败，默认main只打印PLAN_ONLY且不建输出目录。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param capsys pytest stdout/stderr捕获fixture。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_dry_plan_never_starts_subprocess(tmp_path, monkeypatch, capsys):
    m = module()
    ##
    # @par 内部回调 forbidden
    # 一旦触达禁止的process/hardware或关闭后live capture边界即抛AssertionError，不执行动作。
    # 参数 args：额外位置参数，替身记录或按外层选择处理。
    # 参数 kw：额外关键字参数，禁止边界替身不执行它们。
    # 返回：无正常返回；总抛AssertionError。
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


##
# @brief 假sleep推进0.5s见marker或60s到期，均先emit FAILURE_CAPTURE_PENDING再外层CLOSE；不是live取证采集。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param signal_marker true假0.5s创建done marker，false等假60s超时。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize('signal_marker', [True, False])
def test_capture_window_marker_or_timeout_bounded_before_close(tmp_path, signal_marker):
    m = module()
    root = tmp_path / 'case'; root.mkdir()
    now = [0.0]
    events = []
    ##
    # @par 内部回调 sleep
    # 只增加人工now秒；marker开关且达到0.5s时创建done文件，不真实等待60s。
    # 参数 seconds：假sleep增加的单调秒数，不等于真实等待。
    # 返回：None；只推进假时间并可能创建marker。
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


##
# @brief fake action失败时先给取证窗口再关闭owner，窗口中断仍保最初action错误及FAIL metadata。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @param capture_interrupted true在失败取证回调抛KeyboardInterrupt。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
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
    ##
    # @par 内部类型 Process
    # 局部测试类型Process，只支持外层用例要求的消息/进程/Action接口，不代表真实ROS或设备对象。
    class Process:
        ##
        # @par 内部回调 poll
        # 固定None，模拟进程仍活着。
        # 返回：None，固定仍运行。
        def poll(self): return None
        ##
        # @par 内部回调 send_signal
        # 只记录CLOSE事件，不对任何PID发信号。
        # 参数 value：信号编号或非法预算文本，具体含义见对应参数表。
        # 返回：None；只记CLOSE。
        def send_signal(self, value): events.append('CLOSE')
        ##
        # @par 内部回调 wait
        # 固定退出1，不真实等待或保证进程组清理。
        # 参数 timeout：本次允许等待秒数，替身不真实等待。
        # 返回：固定1；或由专用FakeProcess.wait返回其存储值。
        def wait(self, timeout): return 1
    monkeypatch.setattr(m.subprocess, 'Popen', lambda *a, **k: Process())
    ##
    # @par 内部回调 runtime
    # 写人工gate-runtime.json pid123并返回123，不读取/proc运行图。
    # 参数 process：Process替身，不是本次新起的真实Gate。
    # 参数 log：日志占位，相关边界替身不查询真实进程。
    # 参数 root：本case临时产物目录Path。
    # 返回：人工Gate PID123。
    def runtime(process, log, root):
        m.save(root/'gate-runtime.json', {'pid':123})
        return 123
    native.runtime = runtime
    ##
    # @par 内部回调 command
    # 返回预置JSON/Enable/Bootstrap文本或注入action失败；只记录参数，不启动命令。
    # 参数 argv：待记录的命令列表，fake command不会执行。
    # 参数 root：本case临时产物目录Path。
    # 参数 name：本次服务/组件名称，实际解释见外层测试与请求接口。
    # 参数 timeout：本次允许等待秒数，替身不真实等待。
    # 返回：当前JSON/Enable/Bootstrap人工文本；action故障分支抛RuntimeError。
    def command(argv, root, name, timeout):
        if name == 'action': raise RuntimeError('injected first action failure')
        return "success=True, message='Enable queued; wait for confirmed HOLD before submitting motion'" if name.startswith("enable") else "success=True, message='ENABLED_HOLD'"
    native.command = command
    ##
    # @par 内部回调 capture
    # 先记FAILURE_CAPTURE_PENDING，核原action错误；可注入KeyboardInterrupt，不做live设备快照。
    # 参数 root：本case临时产物目录Path。
    # 参数 pid：人工Gate PID123，仅写metadata。
    # 参数 reason：首失败异常/文本，回调检查来源，不回显任意服务内容。
    # 返回：人工诊断字典；Native取证中断开关可抛KeyboardInterrupt。
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


##
# @brief reset探针返回已见boot在Popen前拒绝，不重新启动或运动。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_reused_boot_refused_before_launch(tmp_path, monkeypatch):
    m = module()
    freeze = tmp_path/'freeze.json'; freeze.write_text('{}'); monkeypatch.setattr(m, 'FREEZE', freeze)
    native = m.NativeRun.__new__(m.NativeRun)
    native.freeze = {'firmware':{'path':'frozen.bin'}}; native.seen_boots = {'aa'}
    native.owner = type('Owner', (), {'inspect_owners':staticmethod(lambda tty:[])})()
    native.check_unchanged = lambda: None
    native.probe = lambda root, name: {'boot_epoch':'aa'}
    native.command = lambda *a, **k: ''
    ##
    # @par 内部回调 forbidden
    # 一旦触达禁止的process/hardware或关闭后live capture边界即抛AssertionError，不执行动作。
    # 参数 args：额外位置参数，替身记录或按外层选择处理。
    # 参数 kw：额外关键字参数，禁止边界替身不执行它们。
    # 返回：无正常返回；总抛AssertionError。
    def forbidden(*args, **kw): raise AssertionError('reused boot reached launch')
    monkeypatch.setattr(m.subprocess, 'Popen', forbidden)
    with pytest.raises(ValueError, match='before motion'):
        native({'id':'boot-1-10','kind':'normal','count':10,'reset':True},tmp_path/'case')


##
# @brief 常规关闭后strict离线失败，不能再声称可采live snapshot，metadata保LIVE_SNAPSHOT_NOT_AVAILABLE/qualified=false。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param monkeypatch pytest替换fixture，结束自动恢复模块/时钟/环境。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_offline_strict_failure_occurs_after_close_without_live_claim(tmp_path, monkeypatch):
    m = module()
    freeze = tmp_path / 'freeze.json'; freeze.write_text('{}'); monkeypatch.setattr(m, 'FREEZE', freeze)
    native = m.NativeRun.__new__(m.NativeRun)
    native.freeze = {}; native.seen_boots = set()
    native.check_unchanged = lambda: None
    native.probe = lambda root, name: {'boot_epoch':'aa'}
    events = []
    ##
    # @par 内部类型 Process
    # 局部测试类型Process，只支持外层用例要求的消息/进程/Action接口，不代表真实ROS或设备对象。
    class Process:
        ##
        # @par 内部回调 poll
        # 固定None，模拟进程仍活着。
        # 返回：None，固定仍运行。
        def poll(self): return None
        ##
        # @par 内部回调 send_signal
        # 只记录CLOSE事件，不对任何PID发信号。
        # 参数 value：信号编号或非法预算文本，具体含义见对应参数表。
        # 返回：None；只记CLOSE。
        def send_signal(self, value): events.append('CLOSE')
        ##
        # @par 内部回调 wait
        # 固定退出1，不真实等待或保证进程组清理。
        # 参数 timeout：本次允许等待秒数，替身不真实等待。
        # 返回：固定1；或由专用FakeProcess.wait返回其存储值。
        def wait(self, timeout): return 1
    monkeypatch.setattr(m.subprocess, 'Popen', lambda *a, **k: Process())
    ##
    # @par 内部回调 runtime
    # 写人工gate-runtime.json pid123并返回123，不读取/proc运行图。
    # 参数 process：Process替身，不是本次新起的真实Gate。
    # 参数 log：日志占位，相关边界替身不查询真实进程。
    # 参数 root：本case临时产物目录Path。
    # 返回：人工Gate PID123。
    def runtime(process, log, root):
        m.save(root/'gate-runtime.json', {'pid':123})
        return 123
    native.runtime = runtime
    native.command = lambda argv, root, name, timeout: "success=True, message='Enable queued; wait for confirmed HOLD before submitting motion'" if name.startswith("enable") else "success=True, message='ENABLED_HOLD'"
    ##
    # @par 内部回调 strict
    # 关闭owner后记STRICT并抛离线ValueError，防止错误地再次宣称可取live证据。
    # 参数 root：本case临时产物目录Path。
    # 参数 item：当前分组case计划字典，含id/kind/count/reset。
    # 返回：无正常返回；抛离线strict反例ValueError。
    def strict(root, item):
        events.append('STRICT')
        raise ValueError('真实静态strict反例')
    native.strict = strict
    ##
    # @par 内部回调 forbidden
    # 一旦触达禁止的process/hardware或关闭后live capture边界即抛AssertionError，不执行动作。
    # 参数 args：额外位置参数，替身记录或按外层选择处理。
    # 返回：无正常返回；总抛AssertionError。
    def forbidden(*args): raise AssertionError('strict failure tried live capture after close')
    monkeypatch.setattr(m, 'failure_capture_window', forbidden)
    with pytest.raises(ValueError, match='strict反例'):
        native({'id':'normal-50','kind':'normal','count':50,'reset':False}, tmp_path/'case')
    assert events == ['CLOSE', 'STRICT']
    result = __import__('json').loads((tmp_path/'case/metadata.json').read_text())
    assert result['live_snapshot'] == 'LIVE_SNAPSHOT_NOT_AVAILABLE'
    assert result['qualified'] is False


##
# @brief 人工固定probe JSON须Idle1/session/ref/permit清零及健康计数零、身份匹配；核argv固定路径，不运行probe。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param mutation 冻结字节/inode/模式或probe健康字段变异值。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
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
    ##
    # @par 内部回调 command
    # 返回预置JSON/Enable/Bootstrap文本或注入action失败；只记录参数，不启动命令。
    # 参数 argv：待记录的命令列表，fake command不会执行。
    # 参数 root：本case临时产物目录Path。
    # 参数 label：probe命令标签，如preprobe，便于审查调用边界。
    # 参数 timeout：本次允许等待秒数，替身不真实等待。
    # 返回：当前JSON/Enable/Bootstrap人工文本；action故障分支抛RuntimeError。
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


##
# @brief 仅特定phase0/raw not_ready启动暂拒可重发Enable，Queued后只poll到ENABLED_HOLD。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_enable_startup_denial_retries_until_api_accepts_then_waits_hold(tmp_path):
    m = module(); native = m.NativeRun.__new__(m.NativeRun)
    replies = ["success=False, message='Enable unavailable phase=0 transport=ready boot=0 raw=not_ready'",
               "success=True, message='Enable queued; wait for confirmed HOLD before submitting motion'",
               "success=False, message='BOOTSTRAP_PENDING'",
               "success=True, message='ENABLED_HOLD'"]
    seen = []
    ##
    # @par 内部回调 command
    # 返回预置JSON/Enable/Bootstrap文本或注入action失败；只记录参数，不启动命令。
    # 参数 argv：待记录的命令列表，fake command不会执行。
    # 参数 root：本case临时产物目录Path。
    # 参数 name：本次服务/组件名称，实际解释见外层测试与请求接口。
    # 参数 timeout：本次允许等待秒数，替身不真实等待。
    # 返回：当前JSON/Enable/Bootstrap人工文本；action故障分支抛RuntimeError。
    def command(argv, root, name, timeout):
        seen.append(argv[3]); return replies.pop(0)
    native.command = command
    native.enable_hold(tmp_path, sleep=lambda seconds:None)
    assert seen == ['/device_action_gate/enable','/device_action_gate/enable',
                    '/device_action_gate/bootstrap_status','/device_action_gate/bootstrap_status']
    assert not replies


##
# @brief phase9/未知成功文本/Fault不得进入启动重试，调用一次即拒。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param reply 参数化服务回复对象/文本，具体负例见表。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize('reply', ["success=False, message='Enable unavailable phase=9 transport=ready boot=1 raw=ready'",
                                  "success=True, message='unexpected'",
                                  "success=False, message='FAULT'"])
def test_enable_startup_never_retries_other_failures(tmp_path, reply):
    m = module(); native = m.NativeRun.__new__(m.NativeRun); seen=[]
    native.command = lambda *args: (seen.append(args), reply)[1]
    with pytest.raises(ValueError): native.enable_hold(tmp_path, sleep=lambda seconds:None)
    assert len(seen)==1


##
# @brief 含True字眼但successFalse、伪queued、无结构或相互矛盾文本不得误解析为成功。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param reply 参数化服务回复对象/文本，具体负例见表。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
@pytest.mark.parametrize('reply', ["success=False, message='Enable unavailable FAULT True'",
                                  "success=False, message='Enable queued; wait for confirmed HOLD before submitting motion'",
                                  "success=True arbitrary unrelated text",
                                  "success=True, message='Enable queued; wait for confirmed HOLD before submitting motion' success=False, message='FAULT'"])
def test_enable_parsing_never_accepts_false_or_ambiguous_text(tmp_path, reply):
    m=module(); native=m.NativeRun.__new__(m.NativeRun)
    native.command=lambda *args:reply
    with pytest.raises(ValueError): native.enable_hold(tmp_path,sleep=lambda seconds:None)


##
# @brief 假单调时钟/sleep累加恰20s截止，单次command timeout不超过原预算，不滚动续等。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_enable_retry_stays_inside_original_20_second_budget(tmp_path):
    m=module(); native=m.NativeRun.__new__(m.NativeRun); now=[0.0];calls=[]
    ##
    # @par 内部回调 command
    # 返回预置JSON/Enable/Bootstrap文本或注入action失败；只记录参数，不启动命令。
    # 参数 argv：待记录的命令列表，fake command不会执行。
    # 参数 root：本case临时产物目录Path。
    # 参数 name：本次服务/组件名称，实际解释见外层测试与请求接口。
    # 参数 timeout：本次允许等待秒数，替身不真实等待。
    # 返回：当前JSON/Enable/Bootstrap人工文本；action故障分支抛RuntimeError。
    def command(argv, root, name, timeout):
        calls.append(timeout)
        return "success=False, message='Enable unavailable phase=0 transport=ready boot=0 raw=not_ready'"
    native.command=command
    with pytest.raises(TimeoutError):
        native.enable_hold(tmp_path,monotonic=lambda:now[0],sleep=lambda seconds:now.__setitem__(0,now[0]+seconds))
    assert now[0] == 20.0 and max(calls)<=20


##
# @brief 临时Gate字节、inode或路径不符freeze都拒绝，未从/proc检查真实进程。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @param mutation 冻结字节/inode/模式或probe健康字段变异值。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
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


##
# @brief install symlink解析到同build目标且hash/inode相符可通过，不要求入口文件本身是独立副本。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_runtime_gate_accepts_install_symlink_to_exact_build_target(tmp_path):
    m = module()
    target = tmp_path / 'build_gate'; target.write_bytes(b'candidate-v2')
    entry = tmp_path / 'install_gate'; entry.symlink_to(target)
    validator = getattr(m, 'validate_runtime_gate', None)
    assert callable(validator), 'runtime Gate executable校验缺失'
    result = validator(target, target.stat().st_ino,
                       {'path': str(entry), 'sha256': m.sha(entry)})
    assert result['sha256'] == m.sha(target)


##
# @brief SHA不变但执行mode755→644仍拒，freeze不只绑定内容。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
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


##
# @brief 通过install symlink观察target mode改变也须组guard拒绝，不能仅缓存入口链接模式。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
def test_group_guard_rejects_mode_change_through_install_symlink(tmp_path):
    m=module(); target=tmp_path/'target'; target.write_bytes(b'samebytes'); target.chmod(0o755)
    entry=tmp_path/'entry'; entry.symlink_to(target)
    native=m.NativeRun.__new__(m.NativeRun)
    native.hashes={str(entry):m.sha(entry)}
    native.modes={str(entry):oct(entry.stat().st_mode & 0o777)}
    native.check_unchanged()
    target.chmod(0o644)
    with pytest.raises(ValueError,match='mode'): native.check_unchanged()


##
# @brief source_files与d_test_reference_files一起收集并验证，正式映射header漂移同样触发hash拒绝。
# @param tmp_path pytest临时目录Path，产物/输入仅在本例范围。
# @return None；以assert/pytest.raises/unittest报告本例结论，不生成设备执行资格。
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
