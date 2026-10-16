#!/usr/bin/env python3
"""固定原生台架M3功能批量编排；默认只打印计划，永不签H资格。"""

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

R = Path('<home>/kamin_work/00-code/dummy_ros2')
D = Path('../dpsdk')
S = R / 'scripts/dummy'
REPORTS = R / 'build_dummy/reports'
FREEZE = D / 'build_dummy/reports/native-ros-candidate-v7-20261009.json'
DEP = D / 'build_dummy/reports/no-load-dev-deployment-paths-p9-nonblocking-20261006.json'
LIMITS = D / 'build_dummy/reports/no-load-dev-ros-limits-p9-nonblocking-20261006.json'
MANIFEST = D / 'build_dummy/reports/no-load-dev-candidate-p9-nonblocking-20261006/manifest.json'
POLICY = D / 'configs/f405_6dof_application_long_goal_no_load_candidate.json'
PROBE = D / 'build_dummy/qualification/p9-nonblocking-host-20261006/applications/dummy_ref_control/dummy_ref_read_only_probe'
TTY = Path('/dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00')
ENDPOINT = f'serial/{TTY}#baudrate=115200'
ACTIVATION = '22de50a132c895bac5ebc83470df6585e2bda5fd164e1575c4b79cab324f9f44'
FIRMWARE = 'c41be34d47efda4e4eb2b9c7f36f9d4ed3d78d29f60dfeb54055c2623bf9eb22'
LIBRARY = '6b965e00d1a8c93e1bef84b2c2256cbfd7d79f826b63b5d9c1af567bbdb8fd3b'
DEVICE = '01000000000000000000000000000000'
TARGET_HASHES = {50: '53b3228e6787749626eea3d20378f03fed86e3562f69518d1a8975e87e0f0f7c',
                 10: '338963ad281d6189b6127d7adc0cc0afe59c6ccc5cabba4b2ea2c069252f66b2'}


def plan(stage):
    cases = []
    if stage in ('normal', 'all'):
        cases.append(dict(id='normal-50', kind='normal', count=50, reset=False))
    for index in range(1, 4):
        if stage in ('normal', 'all'):
            cases.append(dict(id=f'boot-{index}-10', kind='normal', count=10, reset=True))
        if stage in ('long', 'all'):
            cases.append(dict(id=f'long-{index}-600', kind='long', count=1, reset=False))
    return cases


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def artifact(path):
    return dict(path=str(path), sha256=sha(path))


def frozen_source_artifacts(freeze):
    return [*freeze['source_files'].values(), *freeze.get('d_test_reference_files', {}).values()]


def verify_frozen_artifact(value):
    path=Path(value['path'])
    if sha(path)!=value['sha256']:
        raise ValueError(f'freeze hash mismatch: {path}')
    if oct(path.stat().st_mode & 0o777)!=value['mode']:
        raise ValueError(f'freeze mode mismatch: {path}')


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def load_owner():
    spec = importlib.util.spec_from_file_location('native_m3_owner', S / 'run_real_6dof_session.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def mapped_library(maps):
    paths = {line.split(maxsplit=5)[-1] for line in maps.splitlines()
             if 'libdummy_guarded_zenbedded_system.so' in line}
    if len(paths) != 1:
        raise ValueError('missing or ambiguous runtime maps library')
    path = Path(paths.pop())
    if not path.is_absolute() or not path.is_relative_to(R / 'install_dummy'):
        raise ValueError('runtime maps uses wrong prefix or deleted library')
    return path


def validate_runtime_gate(executable, mapped_inode, frozen):
    executable = Path(executable)
    expected = Path(frozen['path']).resolve(strict=True)
    if executable.resolve(strict=True) != expected:
        raise ValueError('runtime Gate uses wrong executable path')
    if executable.stat().st_ino != mapped_inode or expected.stat().st_ino != mapped_inode:
        raise ValueError('runtime Gate inode differs from verified disk executable')
    if sha(executable) != frozen['sha256']:
        raise ValueError('runtime Gate executable SHA differs from freeze')
    return artifact(executable)


def validate_case_result(action, timeline, launch, e2e, count):
    if (action.get('status') != 'SUCCEEDED' or
            action.get('stop_confirmation') != 'CONFIRMED_RELEASED_IDLE' or
            action.get('completed_goals', 1) != count or
            action.get('requested_goals', 1) != count or
            e2e.get('status') != 'ROS_BENCH_E2E_POSITIVE' or e2e.get('goals') != count):
        raise ValueError('Action/E2E/count/release mismatch')
    complete = [line for line in timeline.splitlines() if line.startswith('COMPLETE,')]
    if complete != [f'COMPLETE,{count}']:
        raise ValueError('timeline COMPLETE count mismatch')
    if not re.search(r'\[device_action_gate-\d+\]: process has finished cleanly', launch):
        raise ValueError('Gate did not exit cleanly')
    if re.search(r'\[device_action_gate-\d+\]: process has died', launch):
        raise ValueError('Gate died')


def run_group(cases, output, run_case):
    report = dict(scope='M3_NATIVE_FUNCTIONAL', qualified=False, qualification='UNQUALIFIED',
                  result='FAIL', cases=[], first_failure=None,
                  missing_qualification=['H0-H5', 'USB逐seq与D本地时序', '资源floor', '机械ARM与发布门禁'])
    seen_boots = set()
    for item in cases:
        entry = dict(item, result='FAIL', qualified=False)
        report['cases'].append(entry)
        try:
            result = run_case(item, output / item['id'])
            entry.update(result)
            if result.get('result') != 'FUNCTIONAL_POSITIVE':
                raise ValueError('case did not produce strict functional positive')
            boot = result['boot_epoch']
            if item['reset'] and boot in seen_boots:
                raise ValueError('new boot reused a prior group boot')
            seen_boots.add(boot)
        except (Exception, KeyboardInterrupt) as error:
            entry['result'] = 'FAIL'
            entry['failure'] = str(error)
            report['first_failure'] = str(error)
            break
        finally:
            save(output / 'group.json', report)
            if report['first_failure'] is not None:
                save(output / 'group_progress.json', dict(status='FAIL_STOP', case_path=str(output / item['id']),
                     first_failure=report['first_failure'], qualified=False))
    else:
        report['result'] = 'FUNCTIONAL_POSITIVE'
        save(output / 'group.json', report)
    return report


def failure_capture_window(root, gate_pid, reason, *, monotonic=time.monotonic,
                           sleep=time.sleep, emit=None):
    deadline = monotonic() + 60.0
    progress = dict(status='FAILURE_CAPTURE_PENDING', case_path=str(root), gate_pid=gate_pid,
                    qualified=False, reason=str(reason), wait_budget_s=60)
    save(root.parent / 'group_progress.json', progress)
    (emit or (lambda value: print(json.dumps(value, ensure_ascii=False), flush=True)))(progress)
    marker = root / 'failure-capture.done'
    while not marker.exists():
        remaining = deadline - monotonic()
        if remaining <= 0:
            break
        sleep(min(.25, remaining))
    return dict(marker_seen=marker.exists(), wait_budget_s=60,
                finished_monotonic_ns=int(monotonic() * 1_000_000_000))



class NativeRun:
    def __init__(self, output):
        self.output = output
        self.seen_boots = set()
        self.owner = load_owner()
        self.freeze = json.loads(FREEZE.read_text())
        self.paths = [FREEZE, DEP, LIMITS, MANIFEST, POLICY, PROBE, Path(__file__).resolve(),
                      D / 'tools/dummy/flash_ref.py'] + [S / name for name in (
            'run_real_6dof_session.py', 'ros_six_axis_movej.py', 'ros_six_axis_long_goal.py',
            'build_real_6dof_context.py', 'join_real_6dof_evidence.py', 'application_e2e.py',
            'validate_p9_long_goal.py', 'convert_application_ros_limits.py')]
        self.paths += [REPORTS / f'p9-unequal-{n}-targets.json' for n in (50, 10)]
        self.paths += [Path(x['path']) for x in frozen_source_artifacts(self.freeze)]
        self.paths += [Path(self.freeze[k]['path']) for k in (
            'installed_library', 'installed_gate', 'built_gate', 'firmware', 'firmware_elf', 'firmware_map', 'activation', 'source_snapshot')]
        self.hashes = {str(p): sha(p) for p in self.paths}
        self.modes = {str(p): oct(p.stat().st_mode & 0o777) for p in self.paths}
        for k in ('installed_library', 'installed_gate', 'built_gate', 'firmware', 'firmware_elf', 'firmware_map', 'activation', 'source_snapshot'):
            verify_frozen_artifact(self.freeze[k])
        for value in frozen_source_artifacts(self.freeze):
            verify_frozen_artifact(value)
        if self.freeze['installed_library']['sha256'] != LIBRARY or self.freeze['firmware']['sha256'] != FIRMWARE:
            raise ValueError('wrong final7 fixed candidate')
        if self.freeze['activation']['sha256'] != ACTIVATION:
            raise ValueError('wrong activation')
        for count, digest in TARGET_HASHES.items():
            if sha(REPORTS / f'p9-unequal-{count}-targets.json') != digest:
                raise ValueError('targets hash mismatch')
        if json.loads(DEP.read_text())['endpoint'] != ENDPOINT:
            raise ValueError('wrong fixed STM32 endpoint')
        save(output / 'inputs-and-tools.json', self.hashes)

    def check_unchanged(self):
        if any(sha(path) != digest for path, digest in self.hashes.items()):
            raise ValueError('frozen input/tool/source changed during group')
        if any(oct(Path(path).stat().st_mode & 0o777) != mode for path, mode in self.modes.items()):
            raise ValueError('frozen input/tool/source mode changed during group')

    def command(self, argv, root, name, timeout=30):
        started = time.monotonic_ns()
        with (root / f'{name}.stdout').open('w') as out, (root / f'{name}.stderr').open('w') as err:
            try:
                result = subprocess.run([str(x) for x in argv], stdout=out, stderr=err,
                                        timeout=timeout, check=False)
            except BaseException as error:
                save(root / f'{name}.command.json', dict(argv=[str(x) for x in argv],
                     started_monotonic_ns=started, ended_monotonic_ns=time.monotonic_ns(), error=str(error)))
                raise
        save(root / f'{name}.command.json', dict(argv=[str(x) for x in argv],
             started_monotonic_ns=started, ended_monotonic_ns=time.monotonic_ns(), exit_code=result.returncode))
        if result.returncode:
            raise RuntimeError(f'{name}: exit {result.returncode}')
        return (root / f'{name}.stdout').read_text()

    def probe(self, root, label):
        occupied = self.owner.inspect_owners(TTY)
        if occupied:
            raise ValueError(f'probe refused occupied STM32: {occupied}')
        value = json.loads(self.command([PROBE, ENDPOINT, '5000'], root, label, 10))
        identity = self.freeze['identity_from_activation']
        expected = dict(contract_sha256=identity['source_contract_sha256'],
                        model_sha256=identity['model_sha256'],
                        profile_sha256=identity['compiled_policy_sha256'], result='PASS', profile_id=201, configured_mask=63, core_state=1,
                        session_id=0, reference_epoch=0, permit_remaining_us=0,
                        real_motion_permitted=False, command_reject_count=0,
                        can_error_count=0, rx_overflow=0, tx_event_overflow=0, deadline_miss_count=0, bus_off=False,
                        image_sha256=FIRMWARE)
        if any(value.get(k) != v for k, v in expected.items()) or not value.get('boot_epoch'):
            raise ValueError(f'{label}: health/identity mismatch')
        save(root / f'{label}.json', value)
        return value

    def runtime(self, process, log, root):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('session exited before runtime capture')
            match = re.search(r'\[device_action_gate-\d+\]: process started with pid \[(\d+)\]', log.read_text())
            if match:
                pid = int(match[1])
                self.current_gate_pid = pid
                self.current_pgid = os.getpgid(pid)
                maps = Path(f'/proc/{pid}/maps').read_text()
                try:
                    library = mapped_library(maps)
                except ValueError:
                    time.sleep(.1)
                    continue
                mapping_inodes = {int(line.split()[4]) for line in maps.splitlines()
                                  if line.split(maxsplit=5)[-1] == str(library)}
                if mapping_inodes != {library.stat().st_ino}:
                    raise ValueError('runtime mapped inode differs from current disk library')
                if sha(library) != LIBRARY:
                    raise ValueError('runtime loaded so differs from final7 freeze')
                executable = Path(f'/proc/{pid}/exe').resolve(strict=True)
                executable_proof = validate_runtime_gate(
                    executable, Path(f'/proc/{pid}/exe').stat().st_ino, self.freeze['installed_gate'])
                (root / 'gate.maps').write_text(maps)
                (root / 'gate.stat').write_bytes(Path(f'/proc/{pid}/stat').read_bytes())
                argv = Path(f'/proc/{pid}/cmdline').read_bytes().rstrip(b'\0').split(b'\0')
                pgid = os.getpgid(pid)
                save(root / 'gate-runtime.json', dict(pid=pid, pgid=pgid, argv=[x.decode() for x in argv],
                     loaded_library=artifact(library), executable=executable_proof,
                     captured_monotonic_ns=time.monotonic_ns()))
                return pgid
            time.sleep(.1)
        raise TimeoutError('runtime maps capture deadline')

    def enable_hold(self, root, *, monotonic=time.monotonic, sleep=time.sleep):
        def parsed(text):
            matches = re.findall(r"success=(True|False), message='([^']*)'", text)
            if len(matches) != 1:
                raise ValueError('Trigger response missing or ambiguous')
            return matches[0][0] == 'True', matches[0][1]
        deadline = monotonic() + 20.0
        attempt = 0
        while True:
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise TimeoutError('Enable not accepted within original 20s budget')
            reply = self.command(['ros2', 'service', 'call', '/device_action_gate/enable',
                'std_srvs/srv/Trigger', '{}'], root, 'enable' if attempt == 0 else f'enable-{attempt:02d}', remaining)
            success, message = parsed(reply)
            if success:
                if message != 'Enable queued; wait for confirmed HOLD before submitting motion':
                    raise ValueError('unexpected successful Enable response')
                break
            if not re.fullmatch(r'Enable unavailable phase=0 transport=(?:ready|not_ready) boot=[0-9]+ raw=(?:ready|not_ready)', message):
                raise ValueError(f'Enable rejected: {message}')
            attempt += 1
            sleep(min(.5, max(0.0, deadline - monotonic())))
        # 已明确queued以后绝不重复Enable；沿用原15轮Hold确认预算。
        for attempt in range(15):
            reply = self.command(['ros2', 'service', 'call', '/device_action_gate/bootstrap_status',
                'std_srvs/srv/Trigger', '{}'], root, f'hold-{attempt:02d}', 10)
            success, message = parsed(reply)
            if success and message == 'ENABLED_HOLD':
                return
            if message not in ('BOOTSTRAP_PENDING', 'OPERATION_COMPLETE'):
                raise ValueError(f'bootstrap not ready: {message}')
            sleep(.5)
        raise TimeoutError('confirmed ENABLED_HOLD unavailable')

    def strict(self, root, item):
        capture = root
        captures = ['--timeline', capture / 'timeline.csv', '--raw', capture / 'raw.bin',
                    '--action', root / 'action.json', '--context', root / 'context.json']
        self.command(['/usr/bin/python3', S / 'build_real_6dof_context.py', *captures[:-2],
            '--output', root / 'context.json', '--path-tolerance-rad', '.05,.05,.05,.05,.05,.05',
            '--goal-tolerance-rad', '.003,.003,.003,.003,.003,.003', '--producer-lag-budget-us', '250000',
            '--feedback-age-budget-us', '1000000', '--ros-feedback-lag-budget-us', '40000',
            '--gate-state-max-age-us', '250000', '--producer-lag-scope', 'motion_window_v2'], root, 'context', 120)
        self.command(['/usr/bin/python3', S / 'join_real_6dof_evidence.py', *captures,
                      '--output', root / 'joined.json'], root, 'join', 180)
        self.command(['/usr/bin/python3', S / 'application_e2e.py', *captures,
                      '--evidence', root / 'joined.json', '--report', root / 'e2e.json'], root, 'e2e', 180)
        if item['kind'] == 'long':
            self.command(['/usr/bin/python3', S / 'validate_p9_long_goal.py', *captures,
                '--joined', root / 'joined.json', '--report', root / 'long-verified.json',
                '--policy-sha256', self.freeze['identity_from_activation']['compiled_policy_sha256'],
                '--firmware-sha256', FIRMWARE], root, 'long-validation', 180)

    def __call__(self, item, root):
        root.mkdir(exist_ok=False)
        metadata = dict(case=item, qualified=False, result='FAIL', freeze=artifact(FREEZE),
                        started_monotonic_ns=time.monotonic_ns(), environment={k: os.environ.get(k) for k in
                        ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'AMENT_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH')})
        first_error = None
        try:
            self.check_unchanged()
            if item['reset']:
                if self.owner.inspect_owners(TTY):
                    raise ValueError('flash refused occupied STM32')
                self.command(['/usr/bin/python3', D / 'tools/dummy/flash_ref.py', '--execute',
                    '--image', self.freeze['firmware']['path'], '--expected-sha256', FIRMWARE,
                    '--hardware', 'connected', '--approved-powered-bench', '--jlink-serial', 'DEMO_JLINK_SERIAL'],
                    root, 'flash', 60)
            pre = self.probe(root, 'preprobe')
            metadata['boot_epoch'] = pre['boot_epoch']
            if item['reset'] and pre['boot_epoch'] in self.seen_boots:
                raise ValueError('new boot reused prior boot before motion')
            self.seen_boots.add(pre['boot_epoch'])
            session_argv = ['/usr/bin/python3', S / 'run_real_6dof_session.py', '--deployment-paths', DEP,
                '--activation-sha256', ACTIVATION, '--ros-limits-json', LIMITS, '--startup-mode', 'full',
                '--raw-evidence-path', root / 'raw.bin', '--raw-evidence-report-root', root,
                '--goal-timeline-path', root / 'timeline.csv']
            metadata['session_argv'] = [str(x) for x in session_argv]
            log_path = root / 'launch.stdout'
            with log_path.open('w') as log:
                process = subprocess.Popen([str(x) for x in session_argv], stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                pgid = None
                gate_pid = None
                self.current_gate_pid = None
                self.current_pgid = None
                try:
                    pgid = self.runtime(process, log_path, root)
                    gate_pid = json.loads((root / 'gate-runtime.json').read_text())['pid']
                    progress = dict(status='CASE_RUNNING', case_path=str(root), gate_pid=gate_pid,
                                    boot_epoch=pre['boot_epoch'], qualified=False)
                    save(root.parent / 'group_progress.json', progress)
                    print(json.dumps(progress), flush=True)
                    self.enable_hold(root)
                    argv = ['/usr/bin/python3', S / ('ros_six_axis_movej.py' if item['kind'] == 'normal'
                            else 'ros_six_axis_long_goal.py'), '--candidate-manifest', MANIFEST,
                            '--ros-limits-json', LIMITS, '--activation-sha256', ACTIVATION,
                            '--device-id', DEVICE, '--activate-controller', '--report', root / 'action.json']
                    if item['kind'] == 'normal':
                        argv += ['--reference-frame', 'BENCH_SESSION', '--targets-json',
                                 REPORTS / f'p9-unequal-{item["count"]}-targets.json', '--timeout', '45']
                    else:
                        argv += ['--execute', '--long-policy', POLICY]
                    self.command(argv, root, 'action', item['count'] * 45 + 90 if item['kind'] == 'normal' else 700)
                except BaseException as error:
                    first_error = error
                    gate_pid = gate_pid or self.current_gate_pid
                    pgid = pgid or self.current_pgid
                    if process.poll() is None and gate_pid is not None:
                        try:
                            metadata['failure_capture'] = failure_capture_window(root, gate_pid, error)
                        except BaseException as capture_error:
                            metadata['failure_capture_interrupted'] = str(capture_error)
                    raise error
                finally:
                    try:
                        if process.poll() is None:
                            # 仅通知现有owner包装器；它负责12s grace以及TERM/KILL封尾。
                            process.send_signal(signal.SIGTERM)
                        try:
                            metadata['session_exit_code'] = process.wait(timeout=45)
                        except subprocess.TimeoutExpired:
                            if pgid is not None:
                                self.owner.stop_owned_group(pgid, grace_s=12, interrupt_group=True)
                            process.kill()
                            process.wait(timeout=5)
                            raise RuntimeError('session owner exceeded graceful cleanup deadline')
                    except BaseException as cleanup_error:
                        metadata['session_cleanup_failure'] = str(cleanup_error)
                        if first_error is None:
                            raise
            metadata['moveit_exit_lines'] = [line for line in log_path.read_text().splitlines()
                if 'move_group-' in line and ('process has died' in line or 'finished cleanly' in line)]
            self.strict(root, item)
            validate_case_result(json.loads((root / 'action.json').read_text()),
                (root / 'timeline.csv').read_text(), log_path.read_text(),
                json.loads((root / 'e2e.json').read_text()), item['count'])
            self.check_unchanged()
            metadata['result'] = 'FUNCTIONAL_POSITIVE'
        except BaseException as error:
            first_error = error
            metadata['first_failure'] = str(error)
            if 'failure_capture' not in metadata:
                metadata['live_snapshot'] = 'LIVE_SNAPSHOT_NOT_AVAILABLE'
        finally:
            try:
                if 'boot_epoch' not in metadata:
                    raise ValueError('preprobe未成功，不把失败前置追认为运动后probe')
                post = self.probe(root, 'postprobe')
                if post['boot_epoch'] != metadata.get('boot_epoch'):
                    raise ValueError('boot changed within functional case')
            except Exception as error:
                metadata['postprobe_failure'] = str(error)
                if first_error is None:
                    first_error = error
            if first_error is not None:
                metadata['result'] = 'FAIL'
            metadata['ended_monotonic_ns'] = time.monotonic_ns()
            metadata['artifacts'] = [artifact(p) for p in root.rglob('*') if p.is_file()]
            save(root / 'metadata.json', metadata)
        if first_error is not None:
            raise first_error
        return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--stage', choices=('normal', 'long', 'all'), default='normal')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    cases = plan(args.stage)
    if not args.execute:
        print(json.dumps(dict(scope='PLAN_ONLY', hardware_executed=False, qualified=False,
                             freeze=str(FREEZE), cases=cases), ensure_ascii=False, indent=2))
        return 0
    if args.output is None or not args.output.is_absolute() or not args.output.is_relative_to(REPORTS):
        parser.error('--execute需要reports之下的全新绝对--output目录')
    if any(p.is_symlink() for p in (args.output, *args.output.parents)) or '..' in args.output.parts:
        parser.error('拒绝symlink或非canonical输出路径')
    args.output.mkdir(exist_ok=False)
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'收到信号{signum}')
    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        native = NativeRun(args.output)
        with native.owner.owner_lock(Path(f'/run/user/{os.getuid()}/dummy-native-m3.lock')):
            report = run_group(cases, args.output, native)
        return 0 if report['result'] == 'FUNCTIONAL_POSITIVE' else 1
    except (Exception, KeyboardInterrupt) as error:
        save(args.output / 'preflight-failure.json', dict(result='FAIL', qualified=False, reason=str(error)))
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == '__main__':
    raise SystemExit(main())
