#!/usr/bin/env python3
##
# @file run_native_m3_functional.py
# @brief 固定历史冻结产物的原生台架M3功能批次编排。
#
# 默认只打印计划；--execute会实际查USB owner、运行串口probe、启动ROS launch/Action，reset用例还执行flash_ref --execute烧录并复位。
# 路径/hash保留固定历史输入；串口与探针号为公开合成示例，不能作为真实本地设备身份。FUNCTIONAL_POSITIVE始终qualified=false，不替代H0-H5、资源floor或机械/发布门禁。
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

R = Path(__file__).resolve().parents[2]
D = Path(os.environ.get("DPSDK_ROOT", str(Path(__file__).resolve().parents[2].parent / "dpsdk_consolidated")))
S = R / 'scripts/dummy'
REPORTS = R / 'build_dummy/reports'
##
# @brief 历史冻结manifest/部署/limits/候选/探针路径，SDK根可由DPSDK_ROOT覆盖。
# POLICY仍为旧configs路径，固件/库/激活/目标SHA保留历史冻结值；STM32 serial/JLink号为公开合成示例；文件缺失或源码注释导致freeze漂移会失败，不允许自动刷新。
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


##
# @brief 生成固定normal/long批次清单，不访问硬件。
# @param stage normal/long/all，CLI限定；其它值返回空表。
# @return normal含50目标加3次复位后10目标，long含3次单600s；all按循环交错组合。
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


##
# @brief 读取完整文件原字节SHA。
# @param path 文件路径。
# @return 64字符小写摘要；I/O异常传播，非规范JSON SHA。
def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


##
# @brief 用路径与原始SHA表示一个文件。
# @param path 现存可读文件。
# @return path/sha256 dict，不验证文件来源或部署许可。
def artifact(path):
    return dict(path=str(path), sha256=sha(path))


##
# @brief 展开freeze中的源码与可选D参考文件条目。
# @param freeze 已解析冻结映射。
# @return 两集合value拼接列表；不是扫描当前仓库全部源码。
def frozen_source_artifacts(freeze):
    return [*freeze['source_files'].values(), *freeze.get('d_test_reference_files', {}).values()]


##
# @brief 核文件原SHA及低9位权限与freeze声明相同。
# @param value 含path/sha256/mode记录。
# @return None；任一不符ValueError，不主动修改权限或摘要。
def verify_frozen_artifact(value):
    path=Path(value['path'])
    if sha(path)!=value['sha256']:
        raise ValueError(f'freeze hash mismatch: {path}')
    if oct(path.stat().st_mode & 0o777)!=value['mode']:
        raise ValueError(f'freeze mode mismatch: {path}')


##
# @brief 写JSON记录并保留末尾换行。
# @param path 目标文件，调用方保证父目录。
# @param value JSON对象。
# @return None；覆盖同名文件，非原子事务或追加日志。
def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


##
# @brief 加载邻近独占ROS会话包装器。
# @return 已执行模块；其函数可管理真实进程/tty，导入本身不启动会话。
def load_owner():
    spec = importlib.util.spec_from_file_location('native_m3_owner', S / 'run_real_6dof_session.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


##
# @brief 从proc maps找唯一guarded硬件库路径。
# @param maps /proc/PID/maps全文。
# @return install_dummy前缀内绝对Path；缺失/多路径/删除标记或错误前缀拒绝。
# 这里只核路径形状，inode与文件SHA由runtime另核，不能只凭库名相同认定版本。
def mapped_library(maps):
    paths = {line.split(maxsplit=5)[-1] for line in maps.splitlines()
             if 'libdummy_guarded_zenbedded_system.so' in line}
    if len(paths) != 1:
        raise ValueError('missing or ambiguous runtime maps library')
    path = Path(paths.pop())
    if not path.is_absolute() or not path.is_relative_to(R / 'install_dummy'):
        raise ValueError('runtime maps uses wrong prefix or deleted library')
    return path


##
# @brief 将在线Gate可执行路径/inode/原SHA绑定冻结安装产物。
# @param executable 从/proc/PID/exe解析的实际路径。
# @param mapped_inode 运行中exe链接的inode。
# @param frozen 冻结installed_gate记录。
# @return 在线可执行artifact；路径、inode或SHA不符拒绝。
# 核本机当前进程/磁盘对象，非对端固件或外部签名认证。
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


##
# @brief 核Action/E2E数量、timeline尾标及Gate正常退出文字。
# @param action Action报告。
# @param timeline CSV全文。
# @param launch 合并launch日志全文。
# @param e2e 离线E2E报告。
# @param count 该case期望目标数。
# @return None；未成功/未释放、数量不符、缺cleanly或出现died拒绝。
# 这是多来源已有结果核对，Gate日志正常退出不单独证明机械停止，必须同时有严格E2E。
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


##
# @brief 依序执行固定case，首错即停并持续保存批次状态。
# @param cases plan清单。
# @param output 已存在组输出目录。
# @param run_case 接收(item,case目录)的执行回调，实际模式为NativeRun。
# @return 最终组报告；首错记录FAIL，不执行后续case。
# reset项boot不得复用先前boot；组正结论仅FUNCTIONAL_POSITIVE且UNQUALIFIED，明确列出尚缺资格。
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


##
# @brief 首失败后为独立人工/外部抓取保留最多60s窗口。
# @param root 当前case目录。
# @param gate_pid 已捕获Gate PID，仅写进度关联。
# @param reason 原错误。
# @param monotonic 可注入Host秒钟。
# @param sleep 可注入等待函数。
# @param emit 可选进度输出回调，缺省print。
# @return marker是否出现及结束Host ns；不自行采集/恢复设备。
# 等待failure-capture.done只是允许外部取证的信号，不认证取证内容；失败窗口结束仍保留原首错。
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



##
# @brief 持有固定freeze、输入原SHA/权限、seen_boots和owner模块的实际执行器；构造只校文件，调用会接触USB/ROS/烧录。
class NativeRun:
    ##
    # @brief 加载冻结清单并记录本批输入/tool/source指纹。
    # @param output 已创建输出根，用于inputs-and-tools.json。
    # @return None；硬编码固件/库/激活/目标SHA、端点不符立即失败。
    # paths按显式名单，不声称覆盖当前仓库所有文件；固定freeze可能包含源码原始SHA，纯注释也可使历史冻结验证失败，禁止自动更新逃逸。
    def __init__(self, output):
        self.output = output
        self.seen_boots = set()
        self.owner = load_owner()
        self.freeze = json.loads(FREEZE.read_text())
        self.paths = [FREEZE, DEP, LIMITS, MANIFEST, POLICY, PROBE, Path(__file__).resolve(),
                      D / 'applications/dummy_ref_control/tools/dummy_ref_tools/flash_ref.py'] + [S / name for name in (
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

    ##
    # @brief 核本批原指纹与权限未漂移。
    # @return None；任一路径原SHA或mode变化ValueError，不修复/重写freeze。
    def check_unchanged(self):
        if any(sha(path) != digest for path, digest in self.hashes.items()):
            raise ValueError('frozen input/tool/source changed during group')
        if any(oct(Path(path).stat().st_mode & 0o777) != mode for path, mode in self.modes.items()):
            raise ValueError('frozen input/tool/source mode changed during group')

    ##
    # @brief 执行真实子进程并分离stdout/stderr及命令元数据。
    # @param argv 显式参数列表。
    # @param root 现存case目录。
    # @param name 输出文件标签。
    # @param timeout 子进程秒预算，默认30。
    # @return 成功stdout全文；非零RuntimeError，超时/中断保留时间/error元数据再抛。
    # 继承环境但PYTHONPATH覆盖为SDK应用tools；不设置cwd，不保证任意下级命令自己不会启动子进程/设备动作。
    def command(self, argv, root, name, timeout=30):
        started = time.monotonic_ns()
        with (root / f'{name}.stdout').open('w') as out, (root / f'{name}.stderr').open('w') as err:
            try:
                result = subprocess.run([str(x) for x in argv], stdout=out, stderr=err,
                                        timeout=timeout, check=False,
                                        env=dict(os.environ, PYTHONPATH=str(D / "applications/dummy_ref_control/tools")))
            except BaseException as error:
                save(root / f'{name}.command.json', dict(argv=[str(x) for x in argv],
                     started_monotonic_ns=started, ended_monotonic_ns=time.monotonic_ns(), error=str(error)))
                raise
        save(root / f'{name}.command.json', dict(argv=[str(x) for x in argv],
             started_monotonic_ns=started, ended_monotonic_ns=time.monotonic_ns(), exit_code=result.returncode))
        if result.returncode:
            raise RuntimeError(f'{name}: exit {result.returncode}')
        return (root / f'{name}.stdout').read_text()

    ##
    # @brief 无tty owner时运行实际只读设备probe并核冷态健康/身份。
    # @param root case输出目录。
    # @param label preprobe/postprobe输出标签。
    # @return probe JSON；非Idle/许可未清/错误计数非零或固件身份不符拒绝。
    # 真实打开固定STM32端点进行只读请求，不能作为纯离线helper调用；owner检查是当时观测，不是原子跨所有程序的USB排他锁。
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

    ##
    # @brief 限时从launch日志定位Gate并捕获proc运行映像证明。
    # @param process 正在运行的session包装器Popen。
    # @param log launch合并日志Path。
    # @param root 输出case目录。
    # @return Gate所属pgid；30s内无可靠映像/进程先退或inode/SHA不符失败。
    # 核映射库当前磁盘inode及固定SHA，再核/proc/exe；保存maps/stat/argv/Host时刻。不是只核build目录文件而忽略当前加载对象。
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

    ##
    # @brief 对真实Gate Enable发有限请求并等精确Hold服务报告。
    # @param root 命令归档目录。
    # @param monotonic Host时钟注入。
    # @param sleep 等待注入。
    # @return None；20s内未排队、非法回复或15轮仍无Hold失败。
    # 只在明确未排队的特定unavailable文字重试；queued一次后绝不重发Enable。Hold轮询最多15次、每call最多10s加0.5s间隔，不应概括为整个Enable流程总20s。
    def enable_hold(self, root, *, monotonic=time.monotonic, sleep=time.sleep):
        ##
        # @par 内部回调 parsed
        # text为ros2 service stdout；只接受唯一success=(True|False), message文本，返回(bool,message)，缺/多匹配拒绝，不以子串ENABLED_HOLD假通过。
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

    ##
    # @brief 对已完成捕获运行context→join→E2E及可选P9离线链。
    # @param root 含timeline/raw/action的case目录。
    # @param item kind决定是否加600s验证。
    # @return None；任何子命令失败中止。
    # 固定path0.05rad/goal0.003rad、producer窗250ms、ROS邻近40ms、State250ms；这是显式验收输入，非从报告观测反向放宽。
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

    ##
    # @brief 实际执行一case：可烧录、preprobe、ROS运动、收尾、严格证据与postprobe。
    # @param item normal/long、count与reset标志。
    # @param root 必须全新case目录。
    # @return metadata；原首错或postflight失败最后重新抛出，metadata仍尽力保存。
    # reset先确认owner空再调用flash_ref --execute；启动full真实launch并启raw/timeline，运行Action后由session owner收尾。
    # 首错时可保留60s独立取证窗口，之后SIGTERM包装器并最多45s等收尾；超时按所属pgid升级，再kill包装器。postprobe只有preprobe成功才可称运动后检查，boot必须同次不变。
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
                self.command(['/usr/bin/python3', '-m', 'dummy_ref_tools.flash_ref', '--execute',
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


##
# @brief 缺--execute只打印计划，执行模式固定路径/新目录/本地锁。
# @return plan-only0；全部FUNCTIONAL_POSITIVE0，否则1。
# 执行输出须reports内全新canonical绝对路径且拒绝symlink；native-m3锁只协调同锁调用者，session还持另一个launch owner锁。
# SIGNAL处理恢复原SIGTERM handler；执行模式可有烧录与运动副作用，qualified一直false。
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
    ##
    # @par 内部回调 interrupted
    # signum/frame为SIGTERM回调上下文；抛KeyboardInterrupt驱动现有finally收尾，返回不正常完成；不直接发设备Stop。
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
