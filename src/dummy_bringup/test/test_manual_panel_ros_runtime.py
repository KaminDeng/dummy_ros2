"""自有进程只有释放确认后才允许退出。"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

def test_owned_process_is_not_stopped_without_release_confirmation():
    from manual_panel_runtime import PanelRuntime
    class Process:
        stopped = False
        def terminate(self): self.stopped = True
        def wait(self, timeout): pass
    process = Process()
    runtime = PanelRuntime(spawn=lambda argv: process)
    runtime.start(['fake-launch'])
    runtime.read_only()
    runtime.begin_release()
    assert not runtime.stop(False)
    assert not process.stopped
    assert runtime.state == 'Error'
    assert '失能未确认' in runtime.error
    assert runtime.stop(True)
    assert process.stopped

def test_attached_runtime_does_not_stop_external_process():
    from manual_panel_runtime import PanelRuntime
    runtime = PanelRuntime(spawn=lambda argv: (_ for _ in ()).throw(AssertionError('不能启动外部拥有者')))
    runtime.start([]); runtime.read_only()
    assert runtime.state == 'ReadOnly'
    assert runtime.stop(True)

def test_startup_timeout_keeps_original_reason():
    from manual_panel_runtime import PanelRuntime
    runtime = PanelRuntime()
    runtime.start([])
    runtime.check_startup_timeout(31, 30)
    assert runtime.state == 'Error'
    assert 'ROS 启动超时' in runtime.error


def test_real_runtime_validates_original_deployment_and_keeps_ros_switch_config(tmp_path, monkeypatch):
    import json
    import manual_panel_config
    from manual_panel_runtime import load_runtime_panel_config
    original = tmp_path / 'release/panel-config.json'
    original.parent.mkdir()
    original.write_text('{}')
    runtime = tmp_path / 'runtime.json'
    runtime.write_text(json.dumps({'deployment_source_config': str(original),
        'sdk_prefix': '/bound/sdk', 'ros_controller_config': '/controller.yaml',
        'ros_launch_command': ['ros2', 'launch', 'owner', 'reuse_display_planning:=true'],
        'operator_confirmed_double_limits': False}))
    calls = []
    def load(path, backend, description=''):
        calls.append((str(path), backend))
        assert str(path) == str(original)
        return {'native_binary': '/verified/native', 'description_xacro': '/verified/model'}
    monkeypatch.setattr(manual_panel_config, 'load_panel_config', load)
    config = load_runtime_panel_config(runtime, 'real')
    assert calls == [(str(original), 'real')]
    assert config['_config_path'] == str(runtime)
    assert config['native_binary'] == '/verified/native'
    assert config['sdk_prefix'] == '/bound/sdk'
    assert config['operator_confirmed_double_limits'] is False
    assert config['ros_launch_command'][-1] == 'reuse_display_planning:=true'


def test_runtime_cannot_hide_failed_real_deployment_validation(tmp_path, monkeypatch):
    import json
    import pytest
    import manual_panel_config
    from manual_panel_runtime import load_runtime_panel_config
    runtime = tmp_path/'runtime.json'
    runtime.write_text(json.dumps({'deployment_source_config': '/bad/release/panel-config.json'}))
    def reject(*args): raise ValueError('payload digest mismatch')
    monkeypatch.setattr(manual_panel_config, 'load_panel_config', reject)
    with pytest.raises(ValueError, match='payload digest mismatch'):
        load_runtime_panel_config(runtime, 'real')


def test_each_ros_owner_gets_new_exclusive_evidence_paths(tmp_path):
    from manual_panel_runtime import PanelRuntime
    class Process:
        def terminate(self): pass
        def wait(self, timeout): pass
    calls = []
    runtime = PanelRuntime(spawn=lambda argv: calls.append(argv) or Process())
    command = ['ros2', 'launch', 'dummy_bringup', 'manual_ros_control.launch.py',
               'report_path:='+str(tmp_path/'requested.json'), 'raw_evidence_path:=', 'goal_timeline_path:=']
    runtime.start(command)
    assert runtime.stop(True)
    runtime.start(command)
    sessions = []
    for argv in calls:
        args = dict(item.split(':=', 1) for item in argv if ':=' in item)
        paths = [Path(args[key]) for key in ('report_path', 'raw_evidence_path', 'goal_timeline_path')]
        assert paths[0].name == 'baseline.log'
        assert paths[1].name == 'transport.raw'
        assert paths[2].name == 'goal-timeline.jsonl'
        assert all(path.is_absolute() and path.parent.is_dir() and not path.exists() for path in paths)
        assert len({path.parent for path in paths}) == 1
        assert len(set(paths)) == 3
        sessions.append(paths[0].parent)
    assert sessions[0] != sessions[1]
    assert command[-2:] == ['raw_evidence_path:=', 'goal_timeline_path:=']


def test_default_ros_owner_evidence_is_persistent_and_nonempty(tmp_path, monkeypatch):
    from manual_panel_runtime import PanelRuntime
    monkeypatch.setattr(Path, 'home', classmethod(lambda cls: tmp_path))
    calls = []
    runtime = PanelRuntime(spawn=lambda argv: calls.append(argv))
    runtime.start(['ros2', 'launch', 'dummy_bringup', 'manual_ros_control.launch.py'])
    args = dict(item.split(':=', 1) for item in calls[0] if ':=' in item)
    assert all(Path(args[key]).is_relative_to(tmp_path/'.ros/dummy_reports')
               for key in ('report_path', 'raw_evidence_path', 'goal_timeline_path'))


def test_owned_process_group_shutdown_waits_for_child_and_never_external(tmp_path):
    import subprocess, sys, time, os
    from manual_panel_runtime import PanelRuntime
    marker = tmp_path / 'child-stopped'
    ready = tmp_path / 'ready'
    child = "import signal,time,pathlib; signal.signal(signal.SIGINT,lambda *_:(time.sleep(.15),pathlib.Path(%r).write_text('done'),exit(0))); pathlib.Path(%r).write_text('ready'); time.sleep(30)" % (str(marker), str(ready))
    parent = "import subprocess,sys,signal,time; subprocess.Popen([sys.executable,'-c',%r]); signal.signal(signal.SIGINT,lambda *_:exit(0)); time.sleep(30)" % child
    external = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(30)'], start_new_session=True)
    runtime = PanelRuntime()
    try:
        runtime.start([sys.executable, '-c', parent])
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline: time.sleep(.01)
        assert ready.exists()
        assert os.getpgid(runtime.process.pid) != os.getpgrp()
        assert runtime.stop(True)
        assert marker.exists(), 'parent exit must not abandon its owned child'
        assert external.poll() is None
    finally:
        if runtime.process is not None:
            try: os.killpg(runtime.process.pid, 9)
            except ProcessLookupError: pass
            runtime.process.wait(timeout=3)
        external.terminate(); external.wait(timeout=3)


def test_real_start_worker_confirmation_and_discovery_do_not_consume_startup_budget(monkeypatch):
    import ast
    from types import SimpleNamespace as NS
    from manual_panel_runtime import PanelRuntime
    source = Path(__file__).resolve().parents[1] / 'scripts/manual_joint_panel.py'
    cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'JointPanel')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'start_worker')
    clock = [100.0]
    def question(*args):
        clock[0] += 45
        return 0
    def discovery():
        clock[0] += 2
    scope = dict(time=NS(monotonic=lambda: clock[0]), QtWidgets=NS(QMessageBox=NS(question=question, Yes=1, No=0)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(source), 'exec'), scope)
    runtime = PanelRuntime(spawn=lambda argv: object())
    panel = NS(backend='ros', cfg={'ros_launch_command':['fake-launch']}, ros_runtime=runtime,
        ros_backend=NS(transport=NS(assert_no_existing_owner=discovery), connect=lambda:None))
    scope['start_worker'](panel)
    assert runtime.state == 'Starting'
    assert panel.ros_started_at == 147.0
    runtime.check_startup_timeout(clock[0] - panel.ros_started_at, 30)
    assert runtime.state == 'Starting'
    clock[0] += 31
    runtime.check_startup_timeout(clock[0] - panel.ros_started_at, 30)
    assert runtime.state == 'Error' and '启动超时' in runtime.error


def test_owned_deleted_serial_can_retire_without_claiming_release(monkeypatch, tmp_path):
    """仅匹配启动原设备的deleted FD可退休本组；不构造真实ROS/USB进程。"""
    import os, stat
    from types import SimpleNamespace as NS
    from manual_panel_runtime import PanelRuntime
    old=NS(st_mode=stat.S_IFCHR,st_dev=1,st_ino=10,st_rdev=100)
    current=[old]; fds={Path('/proc/321/fd/7'):old}
    original_stat=os.stat
    monkeypatch.setattr(os,'stat',lambda path,*a,**k: current[0] if str(path)=='/dev/serial/by-id/owned' else fds[Path(path)] if Path(path) in fds else original_stat(path,*a,**k))
    monkeypatch.setattr(os,'readlink',lambda path:'/dev/ttyACM2 (deleted)')
    monkeypatch.setattr(Path,'iterdir',lambda path: iter(fds) if str(path)=='/proc/321/fd' else iter(()))
    stopped=[]
    process=NS(pid=321,wait=lambda timeout:None)
    r=PanelRuntime(spawn=lambda argv:process,serial_endpoint='serial//dev/serial/by-id/owned#baudrate=115200',sysfs_root=tmp_path)
    r._owns_group=True;r.start(['owned-launch']);r.error='原动作失败';r._owned_group_pids=lambda:[321]
    r._owned_group_alive=lambda:False
    monkeypatch.setattr(os,'killpg',lambda pgid,sig:stopped.append((pgid,sig)))
    assert not r.retire_disconnected_owner() and not stopped
    current[0]=NS(st_mode=stat.S_IFCHR,st_dev=1,st_ino=11,st_rdev=101)
    driver=tmp_path/f'{os.major(old.st_rdev)}:{os.minor(old.st_rdev)}'
    driver.mkdir()
    assert not r.retire_disconnected_owner() and not stopped
    driver.rmdir()
    assert r.retire_disconnected_owner()
    assert stopped==[(321,__import__('signal').SIGINT)]
    assert r.state=='Disconnected' and r.error=='原动作失败'
    assert not hasattr(r,'released')


def test_detach_retirement_rejects_new_live_fd_unknown_endpoint_or_unproven_old_fd(monkeypatch, tmp_path):
    import os, stat
    from types import SimpleNamespace as NS
    from manual_panel_runtime import PanelRuntime
    old=NS(st_mode=stat.S_IFCHR,st_dev=1,st_ino=10,st_rdev=100)
    new=NS(st_mode=stat.S_IFCHR,st_dev=1,st_ino=11,st_rdev=101)
    current=[old];fds={Path('/proc/321/fd/7'):old};deleted=[True]
    original_stat=os.stat
    monkeypatch.setattr(os,'stat',lambda path,*a,**k:current[0] if str(path)=='/dev/serial/by-id/owned' else fds[Path(path)] if Path(path) in fds else original_stat(path,*a,**k))
    monkeypatch.setattr(os,'readlink',lambda path:'/dev/ttyACM2'+(' (deleted)' if deleted[0] else ''))
    monkeypatch.setattr(Path,'iterdir',lambda path:iter(fds))
    r=PanelRuntime(spawn=lambda argv:NS(pid=321),serial_endpoint='serial//dev/serial/by-id/owned#baudrate=115200',sysfs_root=tmp_path)
    r._owns_group=True;r.start(['owned-launch']);r._owned_group_pids=lambda:[321]
    monkeypatch.setattr(os,'killpg',lambda *a:(_ for _ in ()).throw(AssertionError('不得退出未证实断开的owner')))
    current[0]=new;fds[Path('/proc/321/fd/8')]=new
    assert not r.retire_disconnected_owner()
    del fds[Path('/proc/321/fd/8')];deleted[0]=False
    assert not r.retire_disconnected_owner()
    deleted[0]=True;fds[Path('/proc/321/fd/7')]=NS(st_mode=stat.S_IFCHR,st_dev=1,st_ino=99,st_rdev=100)
    assert not r.retire_disconnected_owner()
    unknown=PanelRuntime(spawn=lambda argv:NS(pid=321));unknown.start(['owned-launch'])
    assert not unknown.retire_disconnected_owner()


def panel_methods(*names, **scope):
    """仅编译真实方法AST；不import Qt/ROS，也不启动进程。"""
    import ast, time, math
    source=Path(__file__).resolve().parents[1]/'scripts/manual_joint_panel.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='JointPanel')
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
    env=dict(time=time,math=math,**scope)
    exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),env)
    return env


def test_ros_owned_observer_generations_drop_all_old_backend_and_message_callbacks(monkeypatch):
    from types import SimpleNamespace as N
    import sys
    calls=[];transports=[];backends=[]
    class Transport:
        def __init__(self,node,on_status,on_state,**kw):
            assert kw==dict(own_control_node=True)
            self.closed=False;self.status,self.state=on_status,on_state;transports.append(self)
    class Backend:
        def __init__(self,transport,callback):self.transport,self.callback=transport,callback;backends.append(self)
    monkeypatch.setitem(sys.modules,'manual_panel_ros_backend',N(RosTransport=Transport,RosBackend=Backend))
    env=panel_methods('create_worker',QtCore=N(QProcess=lambda parent:N()))
    p=N(backend='ros',node=N(),cfg={},zero=N(setText=lambda v:None),backend_notice=N(setText=lambda v:None),
        ros_status_received=lambda v:calls.append(('status',v)),ros_state_received=lambda v:calls.append(('state',v)),
        ros_event=lambda *v:calls.append(v))
    env['create_worker'](p);transports[0].closed=True;env['create_worker'](p)
    transports[0].status('old');transports[0].state('old');backends[0].callback('released','old')
    transports[1].status('new');transports[1].state('new');backends[1].callback('connected','new')
    assert calls==[('status','new'),('state','new'),('connected','new')]


def test_normal_ros_release_arms_readonly_reconnect_but_not_while_closing():
    from types import SimpleNamespace as N
    queued=[];calls=[]
    env=panel_methods('release','ros_event',QtCore=N(QTimer=N(singleShot=lambda delay,fn:queued.append(fn))))
    p=N(backend='ros',closing=False,authorized=True,busy=False,ros_reconnect_pending=False,
        ros_runtime=N(process=object(),begin_release=lambda:calls.append('begin')),
        ros_backend=N(transport=N(),pending=False,release=lambda:True),
        request_reconnect=lambda:calls.append('reconnect'),message=lambda v:None)
    env['release'](p,'RELEASE');assert p.ros_reconnect_pending and not p.authorized
    env['ros_event'](p,'released','RELEASED_IDLE');assert len(queued)==1
    p.closing=True;queued.pop()();assert calls==['begin']
    env['ros_event'](p,'released','RELEASED_IDLE');assert not queued


def test_ros_readonly_reconnect_remembers_no_without_granting_calibration():
    from types import SimpleNamespace as N
    questions=[];argvs=[]
    box=N(Yes=1,No=0,question=lambda *v:questions.append(v) or 0)
    env=panel_methods('start_worker',QtWidgets=N(QMessageBox=box))
    p=N(backend='ros',cfg=dict(ros_launch_command=['fake-launch']),
        ros_runtime=N(start=lambda argv:argvs.append(argv)),
        ros_backend=N(transport=N(assert_no_existing_owner=lambda:None),connect=lambda:None))
    env['start_worker'](p);env['start_worker'](p)
    assert len(questions)==1
    assert p.cfg['operator_confirmed_double_limits'] is False
    assert all('operator_confirmed_double_limits:=false' in argv for argv in argvs)


def test_no_double_limits_still_blocks_reference_zero_but_reuses_existing_reference():
    from types import SimpleNamespace as N
    box=N(Yes=1,No=0)
    env=panel_methods('start_zero',QtWidgets=N(QMessageBox=box))
    for reference in (False,True):
        calls=[]
        p=N(backend='ros',closing=False,busy=False,calibration=(1,1) if reference else None,
            cfg=dict(operator_confirmed_double_limits=False),fresh=lambda:True,
            ros_status=N(session_id=0),message=lambda v:None,confirm_execution=lambda *v:1,
            ros_backend=N(released=False,admitting=True,request_prepare_manual_control=lambda:calls.append('prepare')))
        env['start_zero'](p)
        assert calls==(['prepare'] if reference else [])
        assert p.cfg['operator_confirmed_double_limits'] is False


def test_queued_retired_backend_cannot_clear_new_generations_reconnect_guard():
    from types import SimpleNamespace as N
    queued=[];calls=[]
    env=panel_methods('ros_event',QtCore=N(QTimer=N(singleShot=lambda delay,fn:queued.append(fn))))
    p=N(backend='ros',closing=False,ros_reconnect_pending=True,ros_runtime=N(),
        ros_backend=N(can_close=True),message=lambda v:None,request_reconnect=lambda:calls.append('reconnect'))
    env['ros_event'](p,'released','RELEASED_IDLE')
    p.ros_backend=N(can_close=True);p._ros_reconnect_queued=True
    queued.pop()()
    assert p._ros_reconnect_queued and not calls


def test_external_runner_release_arms_only_current_owned_nonclosing_readonly_restart():
    from types import SimpleNamespace as N
    for owned,closing in ((True,False),(False,False),(True,True)):
        queued=[];calls=[]
        env=panel_methods('ros_event',QtCore=N(QTimer=N(singleShot=lambda delay,fn:queued.append(fn))))
        p=N(closing=closing,ros_reconnect_pending=False,ros_runtime=N(process=object() if owned else None,state='Active'),
            ros_backend=N(can_close=False),message=lambda v:None,request_reconnect=lambda:calls.append('reconnect'))
        env['ros_event'](p,'released','MANUAL_CONTROL_RELEASED')
        assert p.ros_reconnect_pending is (owned and not closing)
        assert len(queued)==int(owned and not closing)
        assert p.ros_runtime.state=='WaitingReleaseFeedback'
        if queued:
            env['ros_event'](p,'released','MANUAL_CONTROL_RELEASED')
            assert len(queued)==1
        assert not calls  # 排队意图不越过request_reconnect原fresh双反馈门。


# ROS新代仍在发现期时，重复重连不能变成释放或创建第二个拥有者。
def reconnect_starting_fixture(state='Starting', detached=False):
    from types import SimpleNamespace as N
    from manual_panel_runtime import PanelRuntime
    calls = []
    runtime = PanelRuntime(spawn=lambda argv: object())
    runtime.start(['fake-owned-launch'])
    runtime.state = state
    runtime.retire_disconnected_owner = lambda: calls.append('retire') or detached
    runtime.stop = lambda released: calls.append(('stop', released)) or True
    transport = N(close=lambda: calls.append('close'),
                  control_graph_error=lambda: 'ROS控制链缺失：所有服务=0')
    backend = N(can_close=False, released=False, transport=transport,
                close=lambda: calls.append('backend_close') or True)
    panel = N(backend='ros', closing=False, ros_runtime=runtime, ros_backend=backend,
              ros_started_at=100., ros_reconnect_pending=False, authorized=False,
              release=lambda command: calls.append(('release', command)),
              apply_backend_change=lambda: calls.append('new_owner'),
              refresh_ros_connection_controls=lambda: None, message=lambda value: None)
    env = panel_methods('request_reconnect', 'ros_tick')
    env['time'] = N(monotonic=lambda: 100.)
    return panel, calls, env


def test_ros_auto_detach_relaunch_then_manual_double_reconnect_keeps_starting_readonly():
    from types import SimpleNamespace as N
    panel, calls, env = reconnect_starting_fixture('ReadOnly', detached=True)
    panel.ros_reconnect_pending = True
    def relaunch():
        calls.append('new_owner')
        panel.ros_runtime.state = 'Starting'
        panel.ros_runtime.retire_disconnected_owner = lambda: False
    panel.apply_backend_change = relaunch
    # 与实际日志一致：timer先退休旧USB并启动新图；随后人工重连仍面对空图。
    env['ros_tick'](panel)
    assert panel.ros_backend.transport.control_graph_error()
    env['request_reconnect'](panel)
    env['request_reconnect'](panel)
    assert calls == ['retire', 'close', 'new_owner']
    assert panel.ros_runtime.state == 'Starting'
    assert not panel.ros_reconnect_pending and panel.ros_started_at == 100.


def test_ros_starting_duplicate_reconnect_does_not_restart_startup_deadline():
    panel, calls, env = reconnect_starting_fixture()
    env['request_reconnect'](panel)
    panel.ros_runtime.check_startup_timeout(31., 30.)
    assert panel.ros_runtime.state == 'Error'
    assert '启动超时' in panel.ros_runtime.error
    assert calls == ['retire'] and panel.ros_started_at == 100.
    assert not panel.ros_reconnect_pending


def test_ros_starting_real_usb_detach_still_retires_owned_generation():
    panel, calls, env = reconnect_starting_fixture(detached=True)
    env['request_reconnect'](panel)
    assert calls == ['retire', 'close', 'new_owner']
    assert not panel.ros_reconnect_pending


def test_ros_readonly_unreleased_reconnect_still_requests_original_release():
    panel, calls, env = reconnect_starting_fixture('ReadOnly')
    env['request_reconnect'](panel)
    assert calls == ['retire', ('release', 'RELEASE')]
    assert panel.ros_reconnect_pending
