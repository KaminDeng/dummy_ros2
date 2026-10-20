"""面板专用六轴运动学模拟；不打开串口/网络，不生成硬件收据或CAN证据。"""
import math
import time
import hashlib
import struct
from pathlib import Path
from PyQt5 import QtCore


class SixAxisSimulator:
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
        self.sample_origin = 0
        self.path_version = 0
        self.chain_budget = 0.
        self.chain_limits = (60.,4.)
        self.completion = 'MOVE_COMPLETED'

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

    def plan(self, goal, speed, acceleration, completion='MOVE_COMPLETED'):
        distance = max(abs(a-b) for a,b in zip(self.q, goal))
        duration = max(.03, 1.875*distance/speed, math.sqrt(5.774*distance/acceleration))
        self.trajectory = (list(self.q), list(goal), duration)
        self.elapsed = 0.
        self.motion += 1
        self.completion = completion

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

    def read_sampled(self,digest,filename):
        payload=Path(filename).read_bytes()
        magic,count,period,boot,reference,session=struct.unpack('<8sIIQQQ',payload[:40])
        if (hashlib.sha256(payload).hexdigest()!=digest or magic!=b'DMYPTH1\0' or
                (boot,reference,session)!=(self.boot,self.reference,self.session) or
                period!=20000 or not 4<=count<=40000 or len(payload)!=40+count*48):
            raise ValueError('轨迹文件无效')
        return tuple(struct.iter_unpack('<6d',payload[40:]))

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


class SimulatorWorker(QtCore.QObject):
    """兼容面板的进程事件接口，内部只有本地定时器。"""
    readyReadStandardOutput = QtCore.pyqtSignal()
    readyReadStandardError = QtCore.pyqtSignal()
    finished = QtCore.pyqtSignal(int, int)

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.model = SixAxisSimulator(config)
        self.running = False
        self.output = b''
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.advance)

    def state(self):
        return QtCore.QProcess.Running if self.running else QtCore.QProcess.NotRunning

    def start(self, *_):
        self.running = True
        self.previous = time.monotonic()
        self.timer.start(20)
        QtCore.QTimer.singleShot(0, lambda: self.emit_event('WAIT_MANUAL_HOME') if self.running else None)

    def emit_event(self, name):
        state = self.model.snapshot()
        fields = (f'boot={state["boot_epoch"]:x} reference={state["reference_epoch"]} '
                  f'session={state["session_id"]} state_seq={state["state_seq"]} '
                  f'motion={state["motion_epoch"]} management={state["management_epoch"]} '
                  f'path_version={self.model.path_version} path_phase_us={int(self.model.elapsed*1e6)}')
        self.output += f'home_rest_event={name} {fields}\n'.encode()
        self.readyReadStandardOutput.emit()

    def write(self, data):
        QtCore.QTimer.singleShot(0, lambda: self.apply(bytes(data).decode().strip()) if self.running else None)
        return len(data)

    def apply(self, command):
        event = self.model.command(command)
        self.emit_event(event)
        if command in ('STOP', 'RELEASE'):
            self.running = False
            self.timer.stop()
            self.finished.emit(0, 0)

    def advance(self):
        now = time.monotonic()
        event = self.model.advance(now-self.previous)
        self.previous = now
        if event:
            self.emit_event(event)
        elif self.model.trajectory and self.model.path_version:
            self.emit_event('PATH_PROGRESS')

    def readAllStandardOutput(self):
        result, self.output = self.output, b''
        return result

    def readAllStandardError(self):
        return b''

    def terminate(self):
        if self.running:
            self.apply('RELEASE')

    def waitForFinished(self, _timeout):
        QtCore.QCoreApplication.processEvents()
        return not self.running
