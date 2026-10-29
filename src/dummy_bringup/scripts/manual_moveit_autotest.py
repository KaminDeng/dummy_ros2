##
# @file manual_moveit_autotest.py
# @brief 组织限时随机目标测试、下一段预规划和同 RUN 连续链续接。
# 测试以一次明确确认建立当前 boot/reference/session 与场景版本；随机候选被拒绝可继续搜索，但服务失联、身份/反馈/授权变化会退出。到期停止补充并在当前已检查路径结束后回 Home；故障或立即中断不强行回程。
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


##
# @brief 以会话起点建立固定的随机测试包络。
# 前三轴半宽至少 45°或两倍幅度，后三轴至少 90°或三倍幅度；不是在每个新目标上重新扩张活动区域。
# @param limits 六轴度制工作范围。
# @param anchor 本轮开始时六轴位置 rad，后续不随随机游走移动。
# @param amplitude small/medium/large 变化幅度键。
# @return tuple：六对 ° 区间，工作范围内保留 2° 余量并围绕 anchor 限制漂移。
def session_envelope(limits, anchor, amplitude):
    step = dict((key,value) for key,_,value in MOTION_AMPLITUDES)[amplitude]
    return tuple((max(lo+2.,math.degrees(origin)-(max(45.,step*2) if axis<3 else max(90.,step*3))),
                  min(hi-2.,math.degrees(origin)+(max(45.,step*2) if axis<3 else max(90.,step*3))))
                 for axis,((lo,hi),origin) in enumerate(zip(limits,anchor)))


##
# @brief 判断候选是否未进一步远离原会话包络。
# 若构型改变使起点已在包络外，允许朝包络收回而不要求一步回到包络内。
# @param goal 候选目标六轴 rad。
# @param start 当前规划起点六轴 rad。
# @param envelope 固定六轴 [lo,hi] 包络，°。
# @return bool：每轴目标到区间的距离不比起点更大，容差 1e-5°。
def goal_respects_envelope(goal, start, envelope):
    for q,actual,(lo,hi) in zip(goal,start,envelope):
        q,actual = math.degrees(q),math.degrees(actual)
        distance = lambda value:max(lo-value,0.,value-hi)
        if distance(q)>distance(actual)+1e-5:return False
    return True


##
# @brief 采样符合幅度、方向连续性和腕部偏好的下一随机目标。
# 生成 32 个局部候选，缩放最大轴差不超过幅度并拒绝过小运动；包络外轴只向原包络收回。按上一段方向余弦选连续候选，near_home 另按 Home 距离优先；无候选抛 ValueError。
# @param start 下一段规划起点六轴 rad。
# @param limits 六轴工作范围 °。
# @param anchor 测试开始时固定六轴 rad。
# @param direction 上一段六轴位移方向，用于减少无谓折返。
# @param rng 提供 uniform 的随机源，默认 random 模块。
# @param amplitude 变化幅度键 small/medium/large。
# @param preference IK 偏好键，约束腕部符号及候选排序。
# @return tuple：六轴候选目标 rad，尚待状态有效性和整路径检查。
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


##
# @brief 把 ROS Pose 转成末端界面使用的 XYZ 和 RPY。
# 反三角输入夹到 [-1,1] 以防浮点舍入超域；不改变原 Pose，也不请求执行。
# @param pose geometry_msgs/Pose，位置米、方向四元数。
# @return tuple[tuple,tuple]：XYZ 毫米和 roll/pitch/yaw 度。
def pose_values(pose):
    q = pose.orientation
    roll = math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch = math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return (pose.position.x*1000,pose.position.y*1000,pose.position.z*1000),tuple(map(math.degrees,(roll,pitch,yaw)))


##
# @class AutoTestSession
# @brief 维护测试时间、预规划候选及设备已确认的连续链。
# active 表示测试生命周期，returning 表示不再加随机目标，executing_home 表示正执行回程。future/stage/samples/checks 属于未来规划，queued 属于完整检查后的候选；chain_pending 尚未收到 Native 续接确认，不能提前作为已生效尾部。
class AutoTestSession:
    ##
    # @brief 创建尚未启动的测试状态和空未来链。
    # chain_origin 是当前缓存块对应的全局样本索引，chain_phase 是设备报告的微秒相位；chain_version 从 1 起，markers 统计连续跨过的随机目标附近位置，不要求逐目标停车。
    # @param tab MoveItTab，提供 planner、执行接口、主面板及日志。
    # @return None；执行上述状态更新、界面或日志操作。
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
        ##
        # chain_phase 单位 µs；chain_origin/markers 为 20 ms 全局样本索引，比较时乘 20000。chain_received_at 为宿主 monotonic 秒。
        self.chain_phase = 0
        self.chain_received_at = None
        self.chain_markers = []
        self.chain_draining = False
        self.ros_stream = False
        self.ros_first_slot_ns = 0

    ##
    # @brief 确认条件后冻结测试身份、场景、幅度与动态参数并开始预规划。
    # 连续模式还要求底层连续执行开启；chain_budget=seconds+60 s 留链尾预算。确认前后重新检查可执行条件，先 invalidate 旧手工规划，再记录本轮实际 anchor 和身份；不会用前一轮授权或场景版本。
    # @param seconds 随机测试宿主时长秒数，允许 0.1..86400；连续模式最多 3420。
    # @return bool：测试成功进入活动状态为 True；忙、拒绝确认或初始准备失败为 False。
    def start(self, seconds):
        t = self.tab
        if self.active or t.executing or t.pending or t.panel.busy or not t.request_ready():return False
        if not math.isfinite(seconds) or not .1 <= seconds <= 86400.:raise ValueError('测试时长必须在0.1秒至24小时之间')
        chain = hasattr(t,'auto_continuous') and t.auto_continuous.isChecked()
        ros = getattr(t.panel, 'backend', None) == 'ros'
        if ros and seconds > 570.:raise ValueError('ROS自动测试最长9.5分钟，另留Home回程预算；完整轨迹不得超过630秒')
        if chain and (not t.continuous_enabled or seconds>3420.):
            raise ValueError('连续测试需启用底层连续执行，单轮最长57分钟（另留回程与停车预算）')
        answer = t.panel.confirm_execution('开始自动测试',
            f'将在当前姿态附近连续生成并执行已检查路径，测试时长 {seconds:g} 秒。\n'
            f'变化幅度：{t.amplitude.currentText()}；逆解偏好：{t.ik_preference.currentText()}。\n'
            f'速度 {t.panel.speed_input.value():g}°/s，加速度 {t.panel.acceleration_input.value():g}rad/s²；'+
            ('短前瞻后起步，边运动边规划，在同一运行中平滑续接；目标附近不停稳。\n' if chain else '每段确认到位。\n')+
            '到期完成当前段后回Home；报错或按停止则中止，不在故障下强行回Home。确认现场路径畅通后开始。')
        if answer != QtWidgets.QMessageBox.Yes or not t.request_ready() or t.panel.busy:return False
        t.invalidate();t.preview_timer.stop()
        self.active,self.returning,self.executing_home = True,False,False
        self.completed,self.result = 0,None
        ##
        # 冻结本轮运动身份和本地场景版本；后续即使参考不变，只要 session 改变也必须退出，不能借旧自动确认继续。
        self.identity = t.execution_identity()
        self.scene_version = t.scene_revision()
        self.started_at = time.monotonic();self.deadline = self.started_at+seconds
        self.anchor = tuple(t.panel.actual_q);self.direction = (0.,)*6
        self.amplitude = t.amplitude.currentData();self.preference = t.ik_preference.currentData()
        self.chain_enabled = chain;self.chain_live = False;self.chain_path = None;self.chain_pending = None
        self.chain_version = 0 if ros else 1;self.chain_origin = 0;self.chain_phase = 0;self.chain_markers = [];self.chain_draining = False
        self.chain_received_at = None
        self.chain_budget = seconds+60.
        self.requested_seconds = seconds
        self.ros_stream = ros and chain
        self.ros_first_slot_ns = 0
        self.ros_completion_pending = False
        self.ros_preplan_deadline = time.monotonic()+120.
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

    ##
    # @brief 本地取消未来规划/碰撞等待并丢弃尚未提交候选。
    # 清 future、checks、samples、queued、stage；不撤回 Native 已确认的运行路径，也不保证 MoveIt 服务端已经停止计算。
    # @return None；执行上述状态更新、界面或日志操作。
    def _drop_planning(self):
        if self.future is not None:self.future.cancel()
        for future in self.checks:future.cancel()
        self.future = None;self.checks = [];self.samples = deque();self.queued = None;self.stage = None

    ##
    # @brief 请求结束随机阶段并在安全尾段后回 Home。
    # 设置 returning，丢弃未来规划，但不立即停止当前设备路径；后续 tick 等既有路径排空再规划 Home。已 returning 时不重复处理。
    # @param reason 结束原因；“测试时长到期”区别于用户提前结束。
    # @return None；执行上述状态更新、界面或日志操作。
    def request_finish(self, reason='用户提前结束'):
        if not self.active or self.returning:return
        self.returning = True
        self.manual_finish = reason != '测试时长到期'
        self._drop_planning()
        self.tab.record('AUTO_TEST_RETURNING',reason=reason,completed=self.completed)

    ##
    # @brief 更新可选自动测试状态标签与颜色。
    # 仅影响 UI；结果是否完成仍由实际路径与 Home 确认决定。
    # @param text 操作员状态说明。
    # @param color Qt CSS 颜色字符串。
    # @return None；执行上述状态更新、界面或日志操作。
    def _status(self, text, color):
        if hasattr(self.tab,'auto_status'):
            self.tab.auto_status.setText(text)
            self.tab.auto_status.setStyleSheet(f'color: {color}; font-weight: 600;')

    ##
    # @brief 把用户立即中断转换为橙色 interrupted 结果。
    # 委托 abort 默认请求受控停车保持；不设置 returning，不自动回 Home。
    # @return None；执行上述状态更新、界面或日志操作。
    def interrupt(self):
        self.abort('用户手动中断', result='interrupted')

    ##
    # @brief 终止测试、释放未来规划资源并按需请求停车。
    # 先置 active=False，再调用 tab.cancel_execution，避免相互递归；删除未确认续接临时文件，清未来队列和 busy。不能把本地清理视为已停车，真正确认由面板取消/释放流程负责。
    # @param reason 中断原因，写入日志并显示。
    # @param stop True 请求 CANCEL_HOLD，缺接口时 STOP；False 表示外层已负责停止或尚未运动。
    # @param result error 为异常红色，interrupted 等其他结果为提示橙色。
    # @return None；执行上述状态更新、界面或日志操作。
    def abort(self, reason, stop=True, result='error'):
        if not self.active:return
        if self.chain_pending is not None:
            filename=self.chain_pending.get('file')
            if filename is not None:filename.unlink(missing_ok=True)
            self.chain_pending=None
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

    ##
    # @brief 确认实际新鲜 Home 到位后结束本轮并保留使能。
    # 要求 actual_q 距配置 Home 逐轴 ≤0.003 rad；不符合则 abort。结果为 completed 或 finished_early，记录真实经过秒数及完成数量，不以目标虚影或时长到期直接判通过。
    # @return None；执行上述状态更新、界面或日志操作。
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

    ##
    # @brief 消费 MoveIt 页已用设备收据确认的整路径完成。
    # Home 路径完成后调用 _complete；普通分段模式累计一个目标。连续链排空时统计尚未跨过的 markers 和终点，清链缓存，未 returning 才从实际位置重新启动规划。
    # @return None；执行上述状态更新、界面或日志操作。
    def on_path_completed(self):
        if not self.active:return
        if self.executing_home:
            self._complete();return
        if self.chain_enabled:
            self.completed += len(self.chain_markers)+1
            self.chain_markers = [];self.chain_live = False;self.chain_path = None
            self.chain_origin = self.chain_phase = 0;self.chain_version = 0 if self.ros_stream else 1;self.chain_draining = False
            self.chain_received_at = None
            if self.ros_stream:
                self.chain_pending=None
                self.ros_preplan_deadline=min(self.deadline,time.monotonic()+120.)
            self._drop_planning()
            self.tab.record('AUTO_TEST_CHAIN_DRAINED',completed=self.completed,returning=self.returning)
            if not self.returning:self._begin(tuple(self.tab.panel.actual_q))
            return
        self.completed += 1
        self.tab.panel.busy = True
        self.tab.record('AUTO_TEST_CYCLE_COMPLETED',completed=self.completed)

    ##
    # @brief 为一个新候选建立独立服务截止并请求状态有效性。
    # attempts 递增，每候选重新给 12 s 服务预算；首候选沿上一运动方向，后续失败重采样取消方向偏置。正常碰撞/无逆解只筛选候选，不能把整轮搜索时间误判成服务失联。
    # @return None；执行上述状态更新、界面或日志操作。
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

    ##
    # @brief 以指定六轴起点开始随机候选或 Home 回程规划。
    # 起点到 Home 已在 0.003 rad 内则直接调用实际 Home 完成检查；否则先查询目标状态有效性，并启动高频轮询。
    # @param start 预定段起点六轴 rad；可为未来已检查段末点，非总是当前实测值。
    # @param home True 直接以配置 Home 为目标；False 采样随机目标。
    # @return None；执行上述状态更新、界面或日志操作。
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

    ##
    # @brief 非阻塞推进候选有效性、FK、IK、规划、拼接及完整曲线检查。
    # random: validity→FK→IK→selected-validity→plan→checking；Home 保持显式关节目标不再做 IK。普通候选碰撞/规划失败重采样，Home 失败则退出。核对路径首点 1e-9 rad、末点 0.003 rad；连续插值/拼接后重新采样检查，最多八个并行请求，全部通过才生成 queued。
    # @return None；执行上述状态更新、界面或日志操作。
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
                if self.stage == 'selected-validity' or (self.ros_stream and not self.plan_home):
                    # ROS随机候选本来就是已验有效的关节目标；不用FK/多IK再解回同一姿态。
                    # 末端用户目标与Native仍保留原FK/IK链，完整路径/拼接碰撞仍在后面检查。
                    if self.stage == 'validity':self.pose = None
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
                ##
                # 随机目标质量筛选只拒绝 J6 额外绕行超过 120° 的候选；Home 回程和人工目标保留完整工作域，不把性能偏好当关节硬限位。
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
                        if self.plan_home:
                            raise ValueError('Home回程不能在原动态上限内衔接：'+str(error)) from error
                        self.tab.record('AUTO_TEST_CANDIDATE_REJECTED',stage='blending',attempt=self.attempts,goal=self.goal,reason=str(error),
                                        endpoint_error=max(abs(a-b) for a,b in zip(self.chain_path.points[-1],self.continuous_path.points[0])))
                        self._candidate();return
                actual_path = (self.joined_path.points[self.join_cut:] if self.joined_path else
                               self.continuous_path.points if self.continuous_path else execution_polyline(self.path))
                ##
                # 待检查样本来自真实要送设备的连续曲线或拼接后修改区，非只检查原规划路点；碰撞结果全部完成才设置 queued。
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

    ##
    # @brief 把已完整检查的候选交给正常执行链。
    # 实测位置须距候选 start ≤0.003 rad；检查已超 2 s 则从实际位置重新规划，不复用旧结果。填目标/位姿、绑定当前场景后 confirmed=True 调 start_execution；确认对象仍不活动则报错。非连续随机段起步后立即从该段目标预规划下一段。
    # @return None；执行上述状态更新、界面或日志操作。
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
        if item['pose'] is not None:t.fill_pose(item['pose'])
        t.panel.set_preview_pose([math.degrees(q) for q in item['goal']])
        t.panel.busy = False
        t.start_execution(confirmed=True)
        if not t.executing:raise ValueError('设备未接受自动测试运动段')
        if not self.executing_home and not self.chain_enabled:self._begin(item['goal'])

    ##
    # @brief 轮询测试安全条件、时长到期、未来规划和链续接。
    # 每轮校验场景就绪/版本、反馈新鲜、授权、boot/reference/session 及生命周期；任何异常统一 abort。到期 request_finish，既有执行结束后规划 Home；链与普通 queued 提交分开处理，未来规划不越过设备接收回执。
    # @return None；执行上述状态更新、界面或日志操作。
    def tick(self):
        if not self.active:return
        try:
            p = self.tab.panel
            if self.chain_enabled or hasattr(self,'scene_version'):
                if not self.tab.scene_ready() or self.tab.scene_revision()!=self.scene_version:
                    raise ValueError('碰撞场景已改变或尚未确认，旧场景测试不可继续')
            ros = getattr(p, 'backend', None) == 'ros'
            # 正式Action完成后只等待同一owner的新鲜实测，不再把短暂缓存龄当新动作失败。
            if ros and getattr(self, 'ros_completion_pending', False):
                state = p.last_state
                identity = ((state['boot_epoch'],state['reference_epoch'],state['session_id'])
                            if state else None)
                if getattr(p,'closing',False) or identity != self.identity:
                    raise ValueError('ROS终态等待期间设备身份或生命周期变化')
                if time.monotonic() >= self.ros_completion_deadline:
                    raise ValueError('ROS终态后未在原期限取得绑定目标的新鲜实际到位')
                if p.fresh() and max(abs(a-b) for a,b in zip(p.actual_q,self.ros_completed_goal)) <= .003:
                    self.ros_completion_pending = False
                    self.on_path_completed()
                return
            # ROS在途/Complete过渡由同UUID的C++Action监督；下一条提交仍必须真实ready。
            if (getattr(p,'closing',False) or not p.fresh() or
                    self.tab.execution_identity() != self.identity or
                    (not ros and (not p.authorized or not p.control_state_allowed((2,3,4,5,6))))):
                # 同一Qt拍内缓存也可能刚过期；复用母FJT原身份和接收锚的有界等待。
                # helper到期已取消时也直接返回，避免再次abort；不续期或提交新运动。
                if ros and self.tab._wait_for_ros_inflight_feedback():return
                raise ValueError('设备反馈、授权、身份或生命周期变化')
            if self.ros_stream and not self.chain_live and not self.returning and time.monotonic() >= self.ros_preplan_deadline:
                raise ValueError('ROS连续轨迹首段准备超时，未提交运动')
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
                if self.ros_stream and not self.chain_live and not self.returning:
                    prepared = self.chain_path.duration if self.chain_path is not None else 0.
                    remaining = max(0.,self.ros_preplan_deadline-time.monotonic())
                    self.tab.auto_status.setText(
                        f'准备起步前瞻：已备 {prepared:.1f}/6 秒 · '
                        f'准备剩余 {remaining:.0f} 秒（可停止）')
                    return
                remaining = max(0.,self.deadline-time.monotonic())
                state = '正在回Home' if self.returning else f'剩余 {remaining:.0f} 秒'
                search = f' · 正在寻找目标（第{self.attempts}次候选）' if self.stage else ''
                self.tab.auto_status.setText(f'自动测试：完成 {self.completed} 个目标 · {state} · 下一段已就绪' if self.queued else
                                             f'自动测试：完成 {self.completed} 个目标 · {state}{search}')
        except Exception as error:
            self.abort(str(error))

    ##
    # @brief 按 Native 已确认缓冲的进度播放目标虚影。
    # chain_phase 为设备微秒；从最近回执宿主时间仅外推最多 150 ms，再给 150 ms 预览前瞻。丢进度时不无限推进虚影，插值结果只写目标，不改实际反馈。
    # @return None；执行上述状态更新、界面或日志操作。
    def update_preview(self):
        """播放Native已确认的命令缓冲，前瞻150ms；失去进度时不无限外推。"""
        if (not self.active or not self.chain_live or self.chain_path is None or
                (not self.ros_stream and self.tab.started_motion is None) or self.chain_received_at is None):return
        path=self.chain_path
        elapsed=max(0.,min(.15,time.monotonic()-self.chain_received_at))
        index=(self.chain_phase/1e6+elapsed+.15)/path.period-self.chain_origin
        index=max(0.,min(len(path.points)-1.,index))
        first=int(index);last=min(first+1,len(path.points)-1);ratio=index-first
        q=tuple(a+(b-a)*ratio for a,b in zip(path.points[first],path.points[last]))
        self.tab.panel.set_preview_pose([math.degrees(v) for v in q],announce=False)

    ##
    # @brief 在本地安装启动前曲线或已被 Native 确认的新尾部。
    # 更新方向和末目标；未 returning 且运行前/剩余缓冲少于 10 s 时从新末点开始下一候选。运行中的更新必须等 PATH_UPDATED 后才能调用。
    # @param item 对应已检查目标的候选字典。
    # @param path 已检查的 PreparedPath。
    # @param origin 该缓存块第一个样本在整条链中的全局索引。
    # @param marker 可选经过上一目标附近的全局样本索引，统计连续目标完成。
    # @return None；执行上述状态更新、界面或日志操作。
    def _chain_install(self, item, path, origin, marker=None):
        self.chain_path = path;self.chain_origin = origin;self.chain_last = item
        if marker is not None:self.chain_markers.append(marker)
        self.direction = tuple(b-a for a,b in zip(item['start'],item['goal']))
        if not self.returning:
            remaining=(origin+len(path.points)-1)*.02-self.chain_phase/1e6
            if not self.chain_live or remaining<10.:self._begin(path.points[-1])

    ##
    # @brief 准备足够前瞻后启动同 RUN 链并滚动补充已检查尾部。
    # 起步前至少 6 s、至少两个连接 marker；运行中等待 MOVE_STARTED 后才续接。新曲线修改点须领先当前相位至少 400 ms；迟到则沿原已检查尾段停车。只裁掉已过前缀并保留四点回看，EXTEND_PATH 带 motion/version/origin/unchanged_through 和摘要，2 s 内等确认，确认前禁止从旧尾点再规划。
    # @return None；执行上述状态更新、界面或日志操作。
    def _chain_tick(self):
        if self.chain_live and (self.chain_received_at is None if self.ros_stream else self.tab.started_motion is None):return
        if self.chain_pending is not None:
            if time.monotonic()>self.chain_pending['deadline']:
                raise ValueError('连续轨迹更新回执超时，停止测试')
            return
        if self.queued is not None and not self.queued['home']:
            # 首槽尚在已确认prime入口内；保留候选，等真实反馈越过首槽再续接。
            if self.ros_stream and self.chain_live and self.chain_phase <= 0:
                return
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
                ##
                # 400000 µs 是续接修改点最小前瞻；不足时不强行改在途命令，只停止补充并排空既有安全尾段。
                if cut_absolute*20000-self.chain_phase<400000:
                    self.tab.record('AUTO_TEST_CHAIN_LATE',phase_us=self.chain_phase,cut=cut_absolute)
                    # 原尾段已经过完整检查，更新迟到只排空，不修改在途命令。
                    self.chain_draining=True;self._drop_planning();return
                if self.ros_stream:
                    from manual_moveit_execution import ros_trajectory
                    trajectory=ros_trajectory(path,self.tab.planner.names,start_tolerance_rad=.003)
                    if len(path.points)>31500:
                        raise ValueError('ROS连续轨迹超过31500样本容量，未续接')
                    first=trajectory.points[0].time_from_start
                    if first.sec*1_000_000_000+first.nanosec!=self.ros_first_slot_ns:
                        raise ValueError('ROS续接改变了母轨迹的起点时间')
                    self.chain_pending=dict(item=item,path=path,origin=0,marker=marker,file=None,
                                            deadline=time.monotonic()+2.)
                    through=self.ros_first_slot_ns+cut_absolute*20_000_000
                    if not self.tab.panel.ros_backend.extend_trajectory(trajectory,self.chain_version,through):
                        raise ValueError('ROS控制链未接受轨迹续接请求')
                    self.tab.record('AUTO_TEST_CHAIN_UPDATE_SUBMITTED',backend='ros',version=self.chain_version+1,
                                    origin=0,unchanged_through=cut_absolute,samples=len(path.points))
                    return
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

    def ros_event(self, kind, value):
        """只消费母Action的真实进度及控制拍已安装的续接回执，不把本地候选当执行轨迹。"""
        if not self.active or not self.ros_stream or not self.chain_enabled:return False
        if kind=='progress':
            desired=getattr(value,'desired',None)
            stamp=getattr(desired,'time_from_start',None)
            if stamp is None:return False
            phase_ns=stamp.sec*1_000_000_000+stamp.nanosec
        elif kind in ('trajectory_extended','trajectory_extension_rejected'):
            phase_ns=value['phase_ns']
        else:return False
        phase=max(0,(phase_ns-self.ros_first_slot_ns)//1000)
        if phase>=self.chain_phase:
            self.chain_phase=phase;self.chain_received_at=time.monotonic()
        if kind=='trajectory_extended':
            pending=self.chain_pending
            if pending is None or value['version']!=self.chain_version+1:
                self.abort('ROS连续轨迹回执版本不匹配');return True
            self.chain_version=value['version'];self.chain_pending=None
            self._chain_install(pending['item'],pending['path'],0,pending['marker'])
            self.tab.ros_execution_goal=self.chain_path.points[-1]
            if self.tab.execution is not None:
                self.tab.execution.points=(self.tab.execution.points[0],self.chain_path.points[-1])
            remaining=self.chain_path.duration-self.chain_phase/1e6
            self.tab.segment_deadline=time.monotonic()+max(2.,remaining)+30.
            self.tab.record('AUTO_TEST_CHAIN_UPDATED',backend='ros',version=self.chain_version,phase_us=self.chain_phase,
                            trajectory_digest=value['trajectory_digest'])
        elif kind=='trajectory_extension_rejected':
            self.chain_pending=None;self.chain_draining=True;self._drop_planning()
            self.tab.record('AUTO_TEST_CHAIN_REJECTED_SAFE_TAIL',backend='ros',phase_us=self.chain_phase,
                            reason=value['reason'])
        while self.chain_markers and self.chain_markers[0]*20000<=self.chain_phase:
            self.chain_markers.pop(0);self.completed+=1
            self.tab.record('AUTO_TEST_BLEND_PASSED',completed=self.completed,phase_us=self.chain_phase)
        return True

    ##
    # @brief 消费连续链进度、更新成功或安全拒绝回执。
    # 核对当前 boot/reference/session 和启动 motion，更新相位与接收时刻。成功须版本恰好 +1 才安装 pending 尾部并延长段等待；更新拒绝清未来规划但沿原安全尾部排空。markers 按设备相位跨越统计，不意味着目标处实际停稳。
    # @param event PATH_PROGRESS、PATH_UPDATED 或 PATH_UPDATE_REJECTED。
    # @param parts 包含身份、motion、path_phase_us 与 path_version 的事件字段。
    # @return bool：属于活动连续链的上述事件时 True，其他 False 交普通执行处理。
    def native_event(self, event, parts):
        if not self.active or not self.chain_enabled:return False
        if event not in ('PATH_PROGRESS','PATH_UPDATED','PATH_UPDATE_REJECTED'):return False
        identity=(int(parts['boot'],16),int(parts['reference']),int(parts['session']))
        if identity!=self.identity or int(parts['motion'])!=self.tab.started_motion:
            self.abort('连续轨迹回执身份不匹配');return True
        self.chain_phase=int(parts['path_phase_us'])
        self.chain_received_at=time.monotonic()
        if event=='PATH_UPDATED':
            ##
            # PATH_UPDATED 是替换尾部的提交点：先核对待确认请求与新版本，才能换 chain_path 并从新终点预规划。
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
