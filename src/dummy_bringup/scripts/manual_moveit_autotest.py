"""限时随机位姿测试：下一段预规划、完整碰撞检查、到期回Home、错误即退出。"""
import math
import random
import time
from collections import deque
from PyQt5 import QtWidgets
from manual_moveit_planner import sample_reasonable_joints, validate_trajectory, MOTION_AMPLITUDES, ik_goal_cost
from manual_moveit_execution import collision_samples, execution_polyline
from manual_moveit_continuous import prepare_path, trajectory_velocities, blend_paths, PreparedPath, write_path
from pathlib import Path
from manual_panel_messages import moveit_error


def session_envelope(limits, anchor, amplitude):
    step = dict((key,value) for key,_,value in MOTION_AMPLITUDES)[amplitude]
    return tuple((max(lo+2.,math.degrees(origin)-(max(45.,step*2) if axis<3 else max(90.,step*3))),
                  min(hi-2.,math.degrees(origin)+(max(45.,step*2) if axis<3 else max(90.,step*3))))
                 for axis,((lo,hi),origin) in enumerate(zip(limits,anchor)))


def goal_respects_envelope(goal, start, envelope):
    for q,actual,(lo,hi) in zip(goal,start,envelope):
        q,actual = math.degrees(q),math.degrees(actual)
        distance = lambda value:max(lo-value,0.,value-hi)
        if distance(q)>distance(actual)+1e-5:return False
    return True


def next_test_goal(start, limits, anchor, direction, rng=random, amplitude='small', preference='nearest'):
    step = dict((key,value) for key,_,value in MOTION_AMPLITUDES)[amplitude]
    local = []
    for (lo,hi),actual in zip(session_envelope(limits,anchor,amplitude),map(math.degrees,start)):
        # 已在包络内时不再扩张；换腕导致包络外时只允许向原包络收回。
        if actual<lo:lo,hi=actual+min(1.,step*.05),min(hi,actual+step)
        elif actual>hi:lo,hi=max(lo,actual-step),actual-min(1.,step*.05)
        local.append((lo,hi))
    candidates = []
    for _ in range(32):
        raw = sample_reasonable_joints(start,local,rng,preference,amplitude)
        delta = [q-a for q,a in zip(raw,start)]
        peak = max(abs(v) for v in delta)
        if peak < 1e-12:continue
        scale = min(1.,math.radians(step)/peak)
        delta = [v*scale for v in delta]
        if max(abs(v) for v in delta) < math.radians(step*.2):continue
        goal = tuple(a+d for a,d in zip(start,delta))
        if preference == 'wrist_same' and abs(start[4]) > math.radians(1.) and start[4]*goal[4] < 0.:continue
        if preference == 'nearest' and abs(start[4]) > math.radians(1.) and start[4]*goal[4] < 0.:continue
        if preference == 'wrist_positive' and goal[4] < 0. and goal[4]<=start[4]:continue
        if preference == 'wrist_negative' and goal[4] > 0. and goal[4]>=start[4]:continue
        norm = math.sqrt(sum(v*v for v in delta))
        direction_norm = math.sqrt(sum(v*v for v in direction))
        alignment = sum(a*b for a,b in zip(delta,direction))/(norm*direction_norm) if direction_norm else 0.
        cost = ik_goal_cost(start,goal,preference)
        candidates.append(((cost[0],-alignment) if preference == 'near_home' else (-alignment,),goal))
    if not candidates:raise ValueError('附近没有满足连续构型与范围要求的随机目标')
    return min(candidates,key=lambda item:item[0])[1]


def pose_values(pose):
    q = pose.orientation
    roll = math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch = math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return (pose.position.x*1000,pose.position.y*1000,pose.position.z*1000),tuple(map(math.degrees,(roll,pitch,yaw)))


class AutoTestSession:
    def __init__(self, tab):
        self.tab = tab
        self.active = False
        self.returning = False
        self.executing_home = False
        self.completed = 0
        self.result = None
        self.future = None
        self.checks = []
        self.samples = deque()
        self.queued = None
        self.stage = None
        self.direction = (0.,)*6
        self.chain_enabled = False
        self.chain_live = False
        self.chain_path = None
        self.chain_pending = None
        self.chain_version = 1
        self.chain_origin = 0
        self.chain_phase = 0
        self.chain_received_at = None
        self.chain_markers = []
        self.chain_draining = False

    def start(self, seconds):
        t = self.tab
        if self.active or t.executing or t.pending or t.panel.busy or not t.request_ready():return False
        if not math.isfinite(seconds) or not .1 <= seconds <= 86400.:raise ValueError('测试时长必须在0.1秒至24小时之间')
        chain = hasattr(t,'auto_continuous') and t.auto_continuous.isChecked()
        if chain and (not t.continuous_enabled or seconds>3420.):
            raise ValueError('连续测试需启用底层连续执行，单轮最长57分钟（另留回程与停车预算）')
        answer = t.panel.confirm_execution('开始自动测试',
            f'将在当前姿态附近连续生成并执行已检查路径，测试时长 {seconds:g} 秒。\n'
            f'变化幅度：{t.amplitude.currentText()}；逆解偏好：{t.ik_preference.currentText()}。\n'
            f'速度 {t.panel.speed_input.value():g}°/s，加速度 {t.panel.acceleration_input.value():g}rad/s²；'+
            ('跨规划曲线圆滑衔接，目标附近不停稳。\n' if chain else '每段确认到位。\n')+
            '到期完成当前段后回Home；报错或按停止则中止，不在故障下强行回Home。确认现场路径畅通后开始。')
        if answer != QtWidgets.QMessageBox.Yes or not t.request_ready() or t.panel.busy:return False
        t.invalidate();t.preview_timer.stop()
        self.active,self.returning,self.executing_home = True,False,False
        self.completed,self.result = 0,None
        self.identity = t.execution_identity()
        self.scene_version = t.scene_revision()
        self.started_at = time.monotonic();self.deadline = self.started_at+seconds
        self.anchor = tuple(t.panel.actual_q);self.direction = (0.,)*6
        self.amplitude = t.amplitude.currentData();self.preference = t.ik_preference.currentData()
        self.chain_enabled = chain;self.chain_live = False;self.chain_path = None;self.chain_pending = None
        self.chain_version = 1;self.chain_origin = 0;self.chain_phase = 0;self.chain_markers = [];self.chain_draining = False
        self.chain_received_at = None
        self.chain_budget = seconds+60.
        self.manual_finish = False
        self._status('自动测试运行中', '#1d4ed8')
        self.speed = t.panel.speed_input.value();self.acceleration = t.panel.acceleration_input.value()
        t.panel.busy = True
        t.record('AUTO_TEST_STARTED',seconds=seconds,identity=self.identity,
                 speed_deg_s=self.speed,acceleration_rad_s2=self.acceleration,
                 amplitude=self.amplitude,ik_preference=self.preference,continuous_chain=chain)
        try:self._begin(tuple(t.panel.actual_q))
        except Exception as error:
            self.abort(str(error),stop=False)
            return False
        t.collision_timer.start()
        return True

    def _drop_planning(self):
        if self.future is not None:self.future.cancel()
        for future in self.checks:future.cancel()
        self.future = None;self.checks = [];self.samples = deque();self.queued = None;self.stage = None

    def request_finish(self, reason='用户提前结束'):
        if not self.active or self.returning:return
        self.returning = True
        self.manual_finish = reason != '测试时长到期'
        self._drop_planning()
        self.tab.record('AUTO_TEST_RETURNING',reason=reason,completed=self.completed)

    def _status(self, text, color):
        if hasattr(self.tab,'auto_status'):
            self.tab.auto_status.setText(text)
            self.tab.auto_status.setStyleSheet(f'color: {color}; font-weight: 600;')

    def interrupt(self):
        self.abort('用户手动中断', result='interrupted')

    def abort(self, reason, stop=True, result='error'):
        if not self.active:return
        if self.chain_pending is not None:
            self.chain_pending['file'].unlink(missing_ok=True);self.chain_pending=None
        self.active = False;self.returning = False;self.result = result
        self._drop_planning()
        self.tab.cancel_execution('自动测试已退出：'+reason,stop=False)
        self.tab.panel.busy = False
        self.tab.record('AUTO_TEST_ABORTED',reason=reason,completed=self.completed)
        self._status(('异常中断：' if result == 'error' else '测试已中断：')+reason,
                     '#b91c1c' if result == 'error' else '#b45309')
        if stop:
            if hasattr(self.tab.panel,'cancel_motion_hold'):self.tab.panel.cancel_motion_hold()
            else:self.tab.panel.release('STOP')

    def _complete(self):
        home = tuple(math.radians(q) for q in self.tab.panel.cfg['home_deg'])
        if not self.tab.panel.fresh() or max(abs(a-b) for a,b in zip(self.tab.panel.actual_q,home)) > .003:
            self.abort('尚未确认实际回到Home');return
        self.active = False;self.result = 'finished_early' if getattr(self,'manual_finish',False) else 'completed';self._drop_planning()
        self.tab.panel.busy = False
        self.tab.record('AUTO_TEST_COMPLETED',result=self.result,completed=self.completed,elapsed_s=time.monotonic()-self.started_at)
        label = '用户提前结束' if self.result == 'finished_early' else '测试通过'
        text = f'{label}：已确认回Home，完成 {self.completed} 个随机目标；当前保持使能。'
        self.tab.status.setText(text)
        self._status(text, '#15803d' if self.result == 'completed' else '#b45309')

    def on_path_completed(self):
        if not self.active:return
        if self.executing_home:
            self._complete();return
        if self.chain_enabled:
            self.completed += len(self.chain_markers)+1
            self.chain_markers = [];self.chain_live = False;self.chain_path = None
            self.chain_origin = self.chain_phase = 0;self.chain_version = 1;self.chain_draining = False
            self._drop_planning()
            self.tab.record('AUTO_TEST_CHAIN_DRAINED',completed=self.completed,returning=self.returning)
            if not self.returning:self._begin(tuple(self.tab.panel.actual_q))
            return
        self.completed += 1
        self.tab.panel.busy = True
        self.tab.record('AUTO_TEST_CYCLE_COMPLETED',completed=self.completed)

    def _candidate(self):
        self.attempts += 1
        # 候选碰撞/无逆解是正常筛选，持续搜索直到总测试到期或用户中断。
        # 每个候选保留服务响应截止；不能把整轮搜索时间误判成服务失联。
        self.plan_deadline = time.monotonic()+12.
        direction = self.direction if self.attempts == 1 else (0.,)*6
        self.goal = next_test_goal(self.plan_start,self.tab.panel.limits,self.anchor,direction,
                                   amplitude=self.amplitude,preference=self.preference)
        self.future = self.tab.planner.request_validity(self.goal)
        self.stage = 'validity'

    def _begin(self, start, home=False):
        self.plan_start = tuple(start);self.plan_home = home
        self.plan_deadline = time.monotonic()+12.;self.attempts = 0
        if home:
            self.goal = tuple(math.radians(q) for q in self.tab.panel.cfg['home_deg'])
            if max(abs(a-b) for a,b in zip(start,self.goal)) <= .003:
                self._complete();return
            self.future = self.tab.planner.request_validity(self.goal);self.stage = 'validity'
        else:self._candidate()
        self.tab.collision_timer.start()

    def _poll_plan(self):
        if self.stage is None:return
        if time.monotonic() >= self.plan_deadline:raise ValueError('规划或碰撞服务长时间无响应')
        if self.future is not None:
            if not self.future.done():return
            future = self.future;response = future.result();self.future = None
            if response is None:raise ValueError('规划服务未返回结果')
            if self.stage in ('validity','selected-validity'):
                if not response.valid:
                    if self.plan_home:raise ValueError('Home目标碰撞，不能自动回程')
                    self.tab.record('AUTO_TEST_CANDIDATE_REJECTED',stage=self.stage,
                                    attempt=self.attempts,reason='collision',goal=self.goal)
                    self._candidate();return
                if self.stage == 'selected-validity':
                    self.future = self.tab.planner.request_plan(self.plan_start,self.goal);self.stage = 'plan'
                else:
                    self.future = self.tab.planner.request_fk(self.goal);self.stage = 'fk'
            elif self.stage == 'fk':
                if response.error_code.val != 1 or not response.pose_stamped:raise ValueError(moveit_error(response.error_code.val))
                self.pose = response.pose_stamped[0].pose
                self.sample_goal = self.goal
                if self.plan_home:
                    self.future = self.tab.planner.request_plan(self.plan_start,self.goal);self.stage = 'plan'
                else:
                    xyz,rpy = pose_values(self.pose)
                    self.future = self.tab.planner.request_ik(self.plan_start,xyz,rpy,self.preference,
                        self.tab.panel.cfg['home_deg'],self.tab.panel.cfg['rest_deg'],self.goal);self.stage = 'ik'
            elif self.stage == 'ik':
                if response.error_code.val != 1:
                    self.tab.record('AUTO_TEST_CANDIDATE_REJECTED',stage=self.stage,
                                    attempt=self.attempts,reason='ik',code=response.error_code.val,goal=self.goal)
                    self._candidate();return
                self.goal = future.selected_goal
                envelope = session_envelope(self.tab.panel.limits,self.anchor,self.amplitude)
                if not goal_respects_envelope(self.goal,self.plan_start,envelope):
                    self.tab.record('AUTO_TEST_IK_ENVELOPE_FALLBACK',selected=self.goal,goal=self.sample_goal)
                    self.goal = self.sample_goal
                self.future = self.tab.planner.request_validity(self.goal);self.stage = 'selected-validity'
            else:
                result = response.motion_plan_response
                if result.error_code.val != 1:
                    if self.plan_home:raise ValueError(moveit_error(result.error_code.val))
                    self.tab.record('AUTO_TEST_CANDIDATE_REJECTED',stage='plan',attempt=self.attempts,
                                    reason='planning',code=result.error_code.val,goal=self.goal)
                    self._candidate();return
                self.path = validate_trajectory(result.trajectory.joint_trajectory,self.tab.planner.names,self.tab.panel.limits)
                if len(self.path) < 2:raise ValueError('测试规划没有有效运动路径')
                if max(abs(a-b) for a,b in zip(self.path[0],self.plan_start)) > 1e-9:raise ValueError('规划起点不匹配预定段起点')
                if max(abs(a-b) for a,b in zip(self.path[-1],self.goal)) > .003:raise ValueError('规划终点不匹配测试目标')
                # 随机测试无需执行低质量的多圈绕路。允许局部窗口的一次完整
                # 往返余量（两侧各30°，额外行程120°），其余继续寻找新目标。
                # 人工指定目标及Home回程仍保留完整工作域，不受此候选筛选限制。
                wrist_travel=sum(abs(b[5]-a[5]) for a,b in zip(self.path,self.path[1:]))
                wrist_direct=abs(self.path[-1][5]-self.path[0][5])
                if not self.plan_home and wrist_travel-wrist_direct>math.radians(120.):
                    self.tab.record('AUTO_TEST_CANDIDATE_REJECTED',stage='path_quality',attempt=self.attempts,
                                    reason='excess_wrist_detour',goal=self.goal,
                                    wrist_travel_deg=math.degrees(wrist_travel),wrist_direct_deg=math.degrees(wrist_direct))
                    self._candidate();return
                self.times = tuple(p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in result.trajectory.joint_trajectory.points)
                self.continuous_path = (prepare_path(self.path,self.times,self.speed,self.acceleration,
                                        velocities=trajectory_velocities(result.trajectory.joint_trajectory,self.tab.planner.names,
                                            required=self.tab.panel.cfg.get('backend')=='real'))
                                        if self.tab.continuous_enabled else None)
                self.joined_path = None
                if self.chain_enabled and self.chain_path is not None and not self.plan_home:
                    try:
                        self.joined_path,self.join_cut = blend_paths(self.chain_path,self.continuous_path)
                    except ValueError as error:
                        self.tab.record('AUTO_TEST_CANDIDATE_REJECTED',stage='blending',attempt=self.attempts,goal=self.goal,reason=str(error),
                                        endpoint_error=max(abs(a-b) for a,b in zip(self.chain_path.points[-1],self.continuous_path.points[0])))
                        self._candidate();return
                actual_path = (self.joined_path.points[self.join_cut:] if self.joined_path else
                               self.continuous_path.points if self.continuous_path else execution_polyline(self.path))
                self.samples = deque(collision_samples(actual_path));self.stage = 'checking'
        if self.stage == 'checking':
            remaining = []
            for future in self.checks:
                if not future.done():remaining.append(future);continue
                response = future.result()
                if response is None:raise ValueError('测试路径碰撞服务未返回结果')
                if not response.valid:
                    if self.plan_home:raise ValueError('Home连续轨迹插值碰撞，未授权执行')
                    for pending in self.checks:pending.cancel()
                    self.checks=[];self.samples=deque()
                    self.tab.record('AUTO_TEST_CANDIDATE_REJECTED',stage='checking',attempt=self.attempts,
                                    reason='continuous_path_collision',goal=self.goal)
                    self._candidate();return
            self.checks = remaining
            while self.samples and len(self.checks) < 8:
                self.checks.append(self.tab.planner.request_validity(self.samples.popleft()))
            if not self.samples and not self.checks:
                self.queued = dict(path=self.path,times=self.times,pose=self.pose,home=self.plan_home,
                                   continuous_path=self.continuous_path,
                                   joined_path=self.joined_path,join_cut=getattr(self,'join_cut',0),
                                   goal=self.goal,start=self.plan_start,checked_at=time.monotonic())
                self.stage = None

    def _submit(self):
        item = self.queued
        if max(abs(a-b) for a,b in zip(self.tab.panel.actual_q,item['start'])) > .003:
            raise ValueError('实际到位位置偏离下一段已检查起点')
        if time.monotonic()-item['checked_at'] > 2.:
            self.queued = None;self._begin(tuple(self.tab.panel.actual_q),item['home']);return
        self.queued = None
        self.executing_home = item['home']
        self.direction = tuple(b-a for a,b in zip(item['start'],item['goal']))
        t = self.tab
        t.path = item['path'];t.times = item['times'];t.start = item['path'][0];t.identity = t.current_identity()
        t.continuous_path = item.get('continuous_path')
        t.path_scene_revision = t.scene_revision()
        t.fill_pose(item['pose']);t.panel.set_preview_pose([math.degrees(q) for q in item['goal']])
        t.panel.busy = False
        t.start_execution(confirmed=True)
        if not t.executing:raise ValueError('设备未接受自动测试运动段')
        if not self.executing_home and not self.chain_enabled:self._begin(item['goal'])

    def tick(self):
        if not self.active:return
        try:
            p = self.tab.panel
            if self.chain_enabled or hasattr(self,'scene_version'):
                if not self.tab.scene_ready() or self.tab.scene_revision()!=self.scene_version:
                    raise ValueError('碰撞场景已改变或尚未确认，旧场景测试不可继续')
            if getattr(p,'closing',False) or not p.fresh() or not p.authorized or self.tab.execution_identity() != self.identity or not p.control_state_allowed((2,3,4,5,6)):
                raise ValueError('设备反馈、授权、身份或生命周期变化')
            if time.monotonic() >= self.deadline:self.request_finish('测试时长到期')
            if self.returning and not self.tab.executing and not self.chain_live and self.stage is None and self.queued is None:
                self.chain_path = None
                self._begin(tuple(p.actual_q),home=True)
            self._poll_plan()
            if self.chain_enabled and not self.executing_home:
                self._chain_tick()
                self.update_preview()
            if self.queued is not None and not self.tab.executing and self.tab.request_ready() and (not self.chain_enabled or self.plan_home):self._submit()
            if hasattr(self.tab,'auto_status') and self.active:
                remaining = max(0.,self.deadline-time.monotonic())
                state = '正在回Home' if self.returning else f'剩余 {remaining:.0f} 秒'
                search = f' · 正在寻找目标（第{self.attempts}次候选）' if self.stage else ''
                self.tab.auto_status.setText(f'自动测试：完成 {self.completed} 个目标 · {state} · 下一段已就绪' if self.queued else
                                             f'自动测试：完成 {self.completed} 个目标 · {state}{search}')
        except Exception as error:
            self.abort(str(error))

    def update_preview(self):
        """播放Native已确认的命令缓冲，前瞻150ms；失去进度时不无限外推。"""
        if (not self.active or not self.chain_live or self.chain_path is None or
                self.tab.started_motion is None or self.chain_received_at is None):return
        path=self.chain_path
        elapsed=max(0.,min(.15,time.monotonic()-self.chain_received_at))
        index=(self.chain_phase/1e6+elapsed+.15)/path.period-self.chain_origin
        index=max(0.,min(len(path.points)-1.,index))
        first=int(index);last=min(first+1,len(path.points)-1);ratio=index-first
        q=tuple(a+(b-a)*ratio for a,b in zip(path.points[first],path.points[last]))
        self.tab.panel.set_preview_pose([math.degrees(v) for v in q],announce=False)

    def _chain_install(self, item, path, origin, marker=None):
        self.chain_path = path;self.chain_origin = origin;self.chain_last = item
        if marker is not None:self.chain_markers.append(marker)
        self.direction = tuple(b-a for a,b in zip(item['start'],item['goal']))
        if not self.returning:
            remaining=(origin+len(path.points)-1)*.02-self.chain_phase/1e6
            if not self.chain_live or remaining<10.:self._begin(path.points[-1])

    def _chain_tick(self):
        if self.chain_live and self.tab.started_motion is None:return
        if self.chain_pending is not None:
            if time.monotonic()>self.chain_pending['deadline']:
                raise ValueError('连续轨迹更新回执超时，停止测试')
            return
        if self.queued is not None and not self.queued['home']:
            item=self.queued;self.queued=None
            path=item.get('joined_path') or item['continuous_path']
            marker=None
            if self.chain_path is not None:
                cut=item['join_cut']
                closest=min(range(cut,len(path.points)),
                            key=lambda i:sum((a-b)**2 for a,b in zip(path.points[i],self.chain_path.points[-1])))
                marker=self.chain_origin+closest
            if self.chain_live:
                cut_absolute=self.chain_origin+item['join_cut']
                if cut_absolute*20000-self.chain_phase<400000:
                    self.tab.record('AUTO_TEST_CHAIN_LATE',phase_us=self.chain_phase,cut=cut_absolute)
                    # 原尾段已经过完整检查，更新迟到只排空，不修改在途命令。
                    self.chain_draining=True;self._drop_planning();return
                trim=max(self.chain_origin,self.chain_phase//20000-4)
                path=PreparedPath(path.points[trim-self.chain_origin:],path.speed,path.acceleration,path.period)
                filename,digest=write_path(path,Path('/tmp'),self.identity)
                (self.tab.panel.report/filename.name).write_bytes(filename.read_bytes())
                self.chain_pending=dict(item=item,path=path,origin=trim,marker=marker,file=filename,
                                        deadline=time.monotonic()+2.)
                if not self.tab.panel.command(f'EXTEND_PATH {self.tab.started_motion} {self.chain_version+1} '
                                              f'{trim} {cut_absolute} {digest} {filename}'):
                    raise ValueError('通信进程未接受续接命令')
                self.tab.record('AUTO_TEST_CHAIN_UPDATE_SUBMITTED',version=self.chain_version+1,origin=trim,
                                unchanged_through=cut_absolute,sha256=digest,samples=len(path.points))
                return  # 回执确认新尾部之前禁止从旧终点启动下一条规划。
            else:
                self._chain_install(item,path,self.chain_origin,marker)
        if self.chain_path is None:return
        if not self.chain_live and (self.chain_path.duration>=6. and len(self.chain_markers)>=2):
            # 起步前准备足够前瞻；运行中下一条规划与实际运动并行。
            self._drop_planning()
            item=dict(self.chain_last)
            item.update(path=(self.chain_path.points[0],self.chain_path.points[-1]),times=(0.,self.chain_path.duration),
                        continuous_path=self.chain_path,start=self.chain_path.points[0],checked_at=time.monotonic())
            self.queued=item;self.chain_live=True
            self._submit()
            self._begin(self.chain_path.points[-1])
        if self.chain_live and self.stage is None and self.queued is None and not self.returning and not self.chain_draining:
            remaining=(self.chain_origin+len(self.chain_path.points)-1)*.02-self.chain_phase/1e6
            if remaining<10. and self.chain_phase>0:self._begin(self.chain_path.points[-1])

    def native_event(self, event, parts):
        if not self.active or not self.chain_enabled:return False
        if event not in ('PATH_PROGRESS','PATH_UPDATED','PATH_UPDATE_REJECTED'):return False
        identity=(int(parts['boot'],16),int(parts['reference']),int(parts['session']))
        if identity!=self.identity or int(parts['motion'])!=self.tab.started_motion:
            self.abort('连续轨迹回执身份不匹配');return True
        self.chain_phase=int(parts['path_phase_us'])
        self.chain_received_at=time.monotonic()
        if event=='PATH_UPDATED':
            pending=self.chain_pending
            if pending is None or int(parts['path_version'])!=self.chain_version+1:
                self.abort('连续轨迹回执版本不匹配');return True
            self.chain_version+=1;self.chain_pending=None
            pending['file'].unlink(missing_ok=True)
            self._chain_install(pending['item'],pending['path'],pending['origin'],pending['marker'])
            if self.tab.execution is not None:
                self.tab.execution.points=(self.tab.execution.points[0],self.chain_path.points[-1])
                remaining=(self.chain_origin+len(self.chain_path.points)-1)*.02-self.chain_phase/1e6
                self.tab.segment_deadline=time.monotonic()+max(2.,remaining)+30.
            self.tab.record('AUTO_TEST_CHAIN_UPDATED',version=self.chain_version,phase_us=self.chain_phase)
        elif event=='PATH_UPDATE_REJECTED':
            if self.chain_pending is not None:
                self.chain_pending['file'].unlink(missing_ok=True);self.chain_pending=None
            self.chain_draining=True;self._drop_planning()
            self.tab.record('AUTO_TEST_CHAIN_REJECTED_SAFE_TAIL',phase_us=self.chain_phase)
        while self.chain_markers and self.chain_markers[0]*20000<=self.chain_phase:
            self.chain_markers.pop(0);self.completed+=1
            self.tab.record('AUTO_TEST_BLEND_PASSED',completed=self.completed,phase_us=self.chain_phase)
        return True
