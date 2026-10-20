##
# @file manual_panel_simulator.py
# @brief 提供主面板兼容的六轴纯软件运动学后端。
# SixAxisSimulator 维护本地位置与模拟身份；SimulatorWorker 以 Qt 定时器和进程式信号适配面板。模拟状态明确标记 PANEL_KINEMATIC_SIMULATOR，不生成真实 USB/CAN 收据，不能据此证明硬件动作完成。
"""面板专用六轴运动学模拟；不打开串口/网络，不生成硬件收据或CAN证据。"""
import math
import time
import hashlib
import struct
from pathlib import Path
from PyQt5 import QtCore


##
# @class SixAxisSimulator
# @brief 在本地推进六轴平滑目标或 20 ms 采样轨迹。
# q 为 rad，velocity 为 rad/s；boot/reference/session/motion/management 是此模型的本地状态身份。trajectory 是否存在决定运动态；session 非零且无轨迹时才表示模拟 Hold。
class SixAxisSimulator:
    ##
    # @brief 以配置 Rest 姿态初始化尚未授权的本地模型。
    # Rest 初值只建立软件显示，不代表实际机械臂姿态；session 初始为零，仍须通过模拟恢复或标定流程建立会话。
    # @param config 面板配置，提供 rest_deg、home_deg、关节 limits_deg 和可选运动上限。
    # @return 无；通过实例状态或界面产生作用。
    def __init__(self, config):
        self.config = config
        self.q = list(map(math.radians, config['rest_deg']))
        self.velocity = [0.] * 6
        self.boot = time.monotonic_ns()
        self.sequence = 0
        self.reference = 1
        self.session = 0
        self.motion = 0
        self.management = 0
        self.trajectory = None
        self.sampled = None
        self.elapsed = 0.
        ##
        # sample_origin 为当前采样块在整条链中的全局样本索引；path_version 校验续接版本，chain_budget 为链总时长秒数。
        self.sample_origin = 0
        self.path_version = 0
        self.chain_budget = 0.
        self.chain_limits = (60.,4.)
        self.completion = 'MOVE_COMPLETED'

    ##
    # @brief 生成与主面板状态字段兼容的模拟快照。
    # 掩码 63 表示六轴有效，can_rx_age_us=0 是模拟占位值，不能解释成真实 CAN 新鲜度证据。core_state 为运动 5、保持会话 4 或无会话 2。
    # @return dict：六轴位置/速度、有效位、Hold 和各 epoch；每次调用递增 state_seq。
    def snapshot(self):
        self.sequence += 1
        return dict(boot_epoch=self.boot, state_seq=self.sequence,
                    reference_epoch=self.reference, session_id=self.session,
                    motion_epoch=self.motion, management_epoch=self.management,
                    core_state=5 if self.trajectory else (4 if self.session else 2),
                    position_valid_mask=63, velocity_valid_mask=63, raw_position_valid_mask=0,
                    can_rx_age_us=[0]*6, estimated_velocity_rad_s=list(self.velocity),
                    hold_ready=bool(self.session and not self.trajectory),
                    hold_valid_mask=63 if self.session and not self.trajectory else 0,
                    hold_position_rad=list(self.q),
                    position_rad=list(self.q), raw_motor_position_rev=[0.]*6,
                    schema_major=2, schema_minor=5, profile_id=0,
                    source='PANEL_KINEMATIC_SIMULATOR')

    ##
    # @brief 为简单目标建立五次平滑插值轨迹。
    # 时长取至少 0.03 s 及满足五次曲线峰值速度/加速度所需时长；仅建立轨迹，advance 才更新位置。输入范围和动态参数由命令入口校验。
    # @param goal 目标六轴 rad。
    # @param speed 用于时长估算的各轴速度上限 rad/s。
    # @param acceleration 各轴加速度上限 rad/s²。
    # @param completion 抵达轨迹终点时返回的模拟事件名。
    # @return 无；通过实例状态或界面产生作用。
    def plan(self, goal, speed, acceleration, completion='MOVE_COMPLETED'):
        distance = max(abs(a-b) for a,b in zip(self.q, goal))
        duration = max(.03, 1.875*distance/speed, math.sqrt(5.774*distance/acceleration))
        self.trajectory = (list(self.q), list(goal), duration)
        self.elapsed = 0.
        self.motion += 1
        self.completion = completion

    ##
    # @brief 解释面板发来的单行命令并更新模拟会话或轨迹。
    # 先处理取消、释放、恢复、标定和路径延长，再要求运动提交时已有 session 且当前无轨迹。MOVE_CHAIN 校验动态与链预算；MOVE_PATH 分支校验文件结构/摘要/身份，不复用全部链动态校验。模拟接受不代表真机策略或碰撞检查通过。
    # @param text 去除换行的 Native 风格命令字符串。
    # @return str：模拟事件名，由 Worker 作为文本事件输出。
    def command(self, text):
        if text == 'CANCEL_HOLD':
            self.sampled = None
            self.trajectory = None;self.velocity = [0.]*6;self.motion += 1
            return 'MOVE_CANCELED_HOLD'
        if text in ('STOP', 'RELEASE'):
            self.sampled = None
            self.trajectory = None
            self.velocity = [0.]*6
            self.session = 0
            self.motion += 1
            self.management += 1
            return 'RELEASE_CONFIRMED'
        if text == 'RESUME':
            self.session = self.session + 1 or 1
            self.management += 1
            return 'CALIBRATION_RESUMED'
        if text in ('HOME CONFIRM_MARGIN', 'LIMITS CONFIRM_MARGIN'):
            self.q = list(map(math.radians, self.config.get('calibration_pose_deg', self.config['home_deg'])))
            self.reference += 1
            self.session = 1
            self.plan(list(map(math.radians, self.config['rest_deg'])), math.radians(2), .1, 'CALIBRATION_READY')
            return 'MOVE_STARTED'
        if text.startswith('EXTEND_PATH '):
            try:
                parts=text.split()
                if len(parts)!=7:raise ValueError()
                motion,version,origin,through=map(int,parts[1:5])
                points=self.read_sampled(parts[5],parts[6])
                ##
                # phase 为当前链全局样本相位；through 必须至少领先 10 点，并验证已承诺的重叠前缀逐点完全相同。
                phase=int(self.elapsed/.02)
                if (not self.sampled or motion!=self.motion or version!=self.path_version+1 or
                        origin<self.sample_origin or origin>phase or through-phase<10 or
                        (origin+len(points)-1)*.02>self.chain_budget or
                        through>=self.sample_origin+len(self.sampled) or through<origin or
                        through-origin>=len(points) or
                        self.sampled[origin-self.sample_origin:through-self.sample_origin+1]!=points[:through-origin+1]):
                    raise ValueError()
                self.validate_samples(points,*self.chain_limits,stationary_start=origin==0)
                self.sampled=points;self.sample_origin=origin;self.path_version=version
                self.trajectory=(list(self.q),list(points[-1]),(origin+len(points)-1)*.02)
                return 'PATH_UPDATED'
            except (ValueError,OverflowError,OSError,struct.error):return 'PATH_UPDATE_REJECTED'
        if not self.session or self.trajectory:
            return 'MOVE_NOT_READY'
        try:
            parts = text.split()
            if len(parts)==8 and parts[0]=='MOVE_CHAIN' and parts[2]=='ACCEL' and parts[4]=='BUDGET':
                speed,acceleration,budget=map(float,(parts[1],parts[3],parts[5]))
                points=self.read_sampled(parts[6],parts[7])
                self.validate_samples(points,speed,acceleration)
                if (not math.isfinite(budget) or not 0<budget<=3507. or
                        (len(points)-1)*.02>budget or max(abs(a-b) for a,b in zip(self.q,points[0]))>.003):
                    raise ValueError()
                self.sampled=points;self.sample_origin=0;self.path_version=1
                self.chain_limits=(speed,acceleration);self.chain_budget=budget
                self.trajectory=(list(self.q),list(points[-1]),(len(points)-1)*.02)
                self.elapsed=0.;self.motion+=1;self.completion='MOVE_COMPLETED'
                return 'MOVE_STARTED'
            if len(parts)==6 and parts[0]=='MOVE_PATH' and parts[2]=='ACCEL':
                payload=Path(parts[5]).read_bytes()
                magic,count,period,boot,reference,session=struct.unpack('<8sIIQQQ',payload[:40])
                if (hashlib.sha256(payload).hexdigest()!=parts[4] or magic!=b'DMYPTH1\0' or
                        (boot,reference,session)!=(self.boot,self.reference,self.session) or
                        period!=20000 or count<4 or count>40000 or len(payload)!=40+count*48):
                    return 'MOVE_REJECTED_PATH'
                self.sample_origin=0;self.path_version=0
                self.sampled=tuple(struct.iter_unpack('<6d',payload[40:]))
                self.trajectory=(list(self.q),list(self.sampled[-1]),(count-1)*.02)
                self.elapsed=0.;self.motion+=1;self.completion='MOVE_COMPLETED'
                return 'MOVE_STARTED'
            if len(parts) != 10 or parts[0] != 'MOVE_SPEED' or parts[2] != 'ACCEL':
                return 'MOVE_REJECTED'
            speed, acceleration = float(parts[1]), float(parts[3])
            degrees = list(map(float, parts[4:]))
            if (not all(math.isfinite(v) for v in [speed, acceleration]+degrees) or
                    not 0 < speed <= self.config.get('motion_speed_max_deg_s', 63.) or
                    not 0 < acceleration <= self.config.get('motion_acceleration_max_rad_s2', 4.) or
                    any(not joint['limits_deg'][0] <= q <= joint['limits_deg'][1]
                        for q, joint in zip(degrees, self.config['joints']))):
                return 'MOVE_REJECTED'
            self.plan(list(map(math.radians, degrees)), math.radians(speed), acceleration)
            return 'MOVE_STARTED'
        except (ValueError, OverflowError, OSError, struct.error):
            return 'MOVE_REJECTED'

    ##
    # @brief 加载并验证绑定本次模拟身份的路径文件。
    # 核对 magic、20 000 µs 周期、4..40000 点、40+48×点数字节长度和 boot/reference/session；不在此检查限位或动态。
    # @param digest 期望的完整 SHA-256 十六进制摘要。
    # @param filename write_path 格式的二进制文件路径。
    # @return tuple[tuple]：解包得到的六轴 rad 路点。
    def read_sampled(self,digest,filename):
        payload=Path(filename).read_bytes()
        magic,count,period,boot,reference,session=struct.unpack('<8sIIQQQ',payload[:40])
        if (hashlib.sha256(payload).hexdigest()!=digest or magic!=b'DMYPTH1\0' or
                (boot,reference,session)!=(self.boot,self.reference,self.session) or
                period!=20000 or not 4<=count<=40000 or len(payload)!=40+count*48):
            raise ValueError('轨迹文件无效')
        return tuple(struct.iter_unpack('<6d',payload[40:]))

    ##
    # @brief 按 20 ms 离散差分校验采样路径的范围和动态。
    # 末两点必须相同；逐点检查工作范围，逐段检查速度，再检查相邻速度差。续接非零初速时跳过第一段相对零速度的加速度比较。此处不做碰撞检查。
    # @param points 完整六轴 rad 采样序列，由读取路径入口保证结构。
    # @param speed 每轴速度上限 °/s，范围 (0,63]。
    # @param acceleration 每轴加速度上限 rad/s²，范围 (0,4]。
    # @param stationary_start True 要求重复首点和零初速；续接片段可从非零速度开始。
    # @return 无；通过实例状态或界面产生作用。
    def validate_samples(self,points,speed,acceleration,stationary_start=True):
        if not math.isfinite(speed) or not 0<speed<=63 or not math.isfinite(acceleration) or not 0<acceleration<=4:
            raise ValueError('运动参数无效')
        if points[-1]!=points[-2] or stationary_start and points[0]!=points[1]:raise ValueError('首末速度无效')
        previous=(0.,)*6
        for i,q in enumerate(points):
            if any(not math.isfinite(v) or not lo<=math.degrees(v)<=hi
                   for v,(lo,hi) in zip(q,[j['limits_deg'] for j in self.config['joints']])):raise ValueError('越界')
            if i:
                velocity=tuple((b-a)/.02 for a,b in zip(points[i-1],q))
                if any(abs(v)>math.radians(speed) for v in velocity):raise ValueError('超速')
                if (stationary_start or i>1) and any(abs(b-a)/.02>acceleration for a,b in zip(previous,velocity)):
                    raise ValueError('超加速度')
                previous=velocity

    ##
    # @brief 根据经过时间推进正在执行的模拟轨迹。
    # 采样轨迹按 20 ms 邻点线性插值；普通目标使用五次平滑函数。结束时固定到终点、归零速度、清除轨迹并递增 motion；只生成软件完成事件。
    # @param dt 距上次推进的秒数；负值按零处理。
    # @return str 或 None：到达终点返回 completion，未完成或无轨迹返回 None。
    def advance(self, dt):
        if not self.trajectory:
            return None
        start, goal, duration = self.trajectory
        self.elapsed = min(duration, self.elapsed + max(0., dt))
        if self.sampled is not None:
            local_elapsed=self.elapsed-self.sample_origin*.02
            index=min(len(self.sampled)-2,int(local_elapsed/.02))
            ratio=min(1.,(local_elapsed-index*.02)/.02)
            self.q=[a+(b-a)*ratio for a,b in zip(self.sampled[index],self.sampled[index+1])]
            self.velocity=[(b-a)/.02 for a,b in zip(self.sampled[index],self.sampled[index+1])]
            if self.elapsed>=duration:
                self.q=list(self.sampled[-1]);self.sampled=None;self.trajectory=None
                self.velocity=[0.]*6;self.motion+=1
                return self.completion
            return None
        ##
        # u 是 [0,1] 的归一化时间；五次 smoothstep 令普通目标轨迹首末速度和加速度为零。
        u = self.elapsed/duration
        blend = 10*u**3 - 15*u**4 + 6*u**5
        derivative = (30*u**2 - 60*u**3 + 30*u**4)/duration
        self.q = [a+(b-a)*blend for a,b in zip(start, goal)]
        self.velocity = [(b-a)*derivative for a,b in zip(start, goal)]
        if self.elapsed >= duration:
            self.q = goal
            self.velocity = [0.]*6
            self.trajectory = None
            self.motion += 1
            return self.completion
        return None


##
# @class SimulatorWorker
# @brief 用 QObject 模拟面板需要的 QProcess 子集。
# readyReadStandardOutput、readyReadStandardError、finished 与主面板已有连接兼容；内部只有 QTimer，不创建子进程或硬件连接。状态快照由主面板直接读取 model.snapshot()。
class SimulatorWorker(QtCore.QObject):
    """兼容面板的进程事件接口，内部只有本地定时器。"""
    readyReadStandardOutput = QtCore.pyqtSignal()
    readyReadStandardError = QtCore.pyqtSignal()
    finished = QtCore.pyqtSignal(int, int)

    ##
    # @brief 创建模型、输出缓冲和推进定时器。
    # @param config 传给 SixAxisSimulator 的面板配置。
    # @param parent 可选 Qt 父对象，管理 Worker 生命周期。
    # @return 无；通过实例状态或界面产生作用。
    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.model = SixAxisSimulator(config)
        self.running = False
        self.output = b''
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.advance)

    ##
    # @brief 返回兼容 QProcess 的运行状态枚举。
    # @return QProcess.Running 或 QProcess.NotRunning，取决于本地 running 标记。
    def state(self):
        return QtCore.QProcess.Running if self.running else QtCore.QProcess.NotRunning

    ##
    # @brief 启动 20 ms 定时推进并排队发出初始等待事件。
    # 使用 monotonic 秒计时；singleShot(0) 等待 Qt 事件循环，且回调执行前再次检查 running。启动不建立控制会话。
    # @param _ 与 QProcess.start 调用兼容的额外参数；模拟模式忽略。
    # @return 无；通过实例状态或界面产生作用。
    def start(self, *_):
        self.running = True
        self.previous = time.monotonic()
        self.timer.start(20)
        QtCore.QTimer.singleShot(0, lambda: self.emit_event('WAIT_MANUAL_HOME') if self.running else None)

    ##
    # @brief 将当前模拟事件与身份序列写入标准输出缓冲。
    # snapshot 会递增 state_seq；path_phase_us 从经过秒数换算为微秒。发信号通知面板读取缓冲，事件文本本身不是硬件收据。
    # @param name 模拟事件标识，例如 MOVE_STARTED 或 RELEASE_CONFIRMED。
    # @return 无；通过实例状态或界面产生作用。
    def emit_event(self, name):
        state = self.model.snapshot()
        fields = (f'boot={state["boot_epoch"]:x} reference={state["reference_epoch"]} '
                  f'session={state["session_id"]} state_seq={state["state_seq"]} '
                  f'motion={state["motion_epoch"]} management={state["management_epoch"]} '
                  f'path_version={self.model.path_version} path_phase_us={int(self.model.elapsed*1e6)}')
        self.output += f'home_rest_event={name} {fields}\n'.encode()
        self.readyReadStandardOutput.emit()

    ##
    # @brief 将字节命令排队交给 Qt 事件循环处理。
    # bytes(data) 在回调时解码并 strip；面板接口按单行命令使用此方法，模拟器不实现完整流式分帧。
    # @param data 面板写入的字节数据，通常为带换行的一个命令。
    # @return int：接收字节数；表示排队接收，不表示命令成功或动作完成。
    def write(self, data):
        QtCore.QTimer.singleShot(0, lambda: self.apply(bytes(data).decode().strip()) if self.running else None)
        return len(data)

    ##
    # @brief 在事件循环中执行一个模拟命令并发事件。
    # STOP/RELEASE 先输出释放事件，再停止本地定时器并发 finished(0,0)；其他命令只根据模型结果输出事件。
    # @param command 已解码的命令字符串。
    # @return 无；通过实例状态或界面产生作用。
    def apply(self, command):
        event = self.model.command(command)
        self.emit_event(event)
        if command in ('STOP', 'RELEASE'):
            self.running = False
            self.timer.stop()
            self.finished.emit(0, 0)

    ##
    # @brief 在定时器触发时按实际单调经过时间推进模型。
    # 完成时输出结束事件；连续链尚在运行且具有 path_version 时输出 PATH_PROGRESS。普通运动中间位置通过主面板直接取得 snapshot。
    # @return 无；通过实例状态或界面产生作用。
    def advance(self):
        now = time.monotonic()
        event = self.model.advance(now-self.previous)
        self.previous = now
        if event:
            self.emit_event(event)
        elif self.model.trajectory and self.model.path_version:
            self.emit_event('PATH_PROGRESS')

    ##
    # @brief 取出并清空积累的模拟标准输出。
    # @return bytes：自上次读取后产生的事件行。
    def readAllStandardOutput(self):
        result, self.output = self.output, b''
        return result

    ##
    # @brief 提供兼容 QProcess 的空错误输出。
    # @return bytes：始终为空字节串。
    def readAllStandardError(self):
        return b''

    ##
    # @brief 将运行中的模拟 Worker 按 RELEASE 流程结束。
    # 不直接杀进程；本对象没有子进程。未运行时无动作。
    # @return 无；通过实例状态或界面产生作用。
    def terminate(self):
        if self.running:
            self.apply('RELEASE')

    ##
    # @brief 处理一次 Qt 待办事件并查询是否结束。
    # @param _timeout 为兼容 QProcess 保留的超时参数；此处不执行阻塞等待。
    # @return bool：一次 processEvents 后 running 是否为 False。
    def waitForFinished(self, _timeout):
        QtCore.QCoreApplication.processEvents()
        return not self.running
