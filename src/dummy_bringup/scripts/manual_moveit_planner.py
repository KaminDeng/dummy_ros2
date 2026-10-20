"""面板的异步 MoveIt 服务适配；不持有 USB、不执行运动。"""
import math
from concurrent.futures import Future

IK_PREFERENCES = (
    ('nearest', '最短关节行程', '优先最小化六轴最大转角，通常耗时更短；适合日常点位测试，J6自动选择最近合法圈数。'),
    ('wrist_same', '保持腕部构型', '优先保持当前J5正负方向，减少腕部翻转；适合连续微调。当前构型不可达时允许其他有效解。'),
    ('wrist_positive', '腕部正向', '优先选择J5为正的腕部解；适合比较同一目标的不同腕部构型。该解不可达时回退到可用解。'),
    ('wrist_negative', '腕部反向', '优先选择J5为负的腕部解；适合避让腕部空间或比较翻腕姿态。该解不可达时回退到可用解。'),
    ('near_home', '靠近 Home 构型', '优先选择靠近Home关节姿态及圈数的解，可能增加当前行程；适合从熟悉构型测试新目标，路径仍须通过碰撞检查。'),
)


def ik_goal_cost(start, goal, preference='nearest', home=None):
    delta = [abs(a-b) for a,b in zip(start,goal)]
    penalty = 0
    if preference == 'wrist_same':
        # J5接近零时腕部构型退化，反馈噪声的正负号不能决定翻腕偏好。
        penalty = int(abs(start[4]) > math.radians(1.) and start[4]*goal[4] < 0.)
    elif preference == 'wrist_positive':
        penalty = int(goal[4] < -1e-6)
    elif preference == 'wrist_negative':
        penalty = int(goal[4] > 1e-6)
    elif preference == 'near_home':
        distance = [abs(a-b) for a,b in zip(home or (0.,0.,math.pi/2,0.,0.,0.),goal)]
        return (max(distance), sum(distance), max(delta), sum(delta))
    elif preference != 'nearest':
        raise ValueError('未知逆解偏好')
    return (penalty, max(delta), sum(delta))


MOTION_AMPLITUDES = (('small', '小（15°）', 15.), ('medium', '中（40°）', 40.), ('large', '大（80°）', 80.))

def sample_reasonable_joints(start, limits_deg, rng, preference='nearest', amplitude=None):
    if len(start) != 6 or len(limits_deg) != 6 or not all(math.isfinite(v) for v in start):
        raise ValueError('随机目标需要有效六轴起点')
    result = []
    for axis,(actual,bounds) in enumerate(zip(start,limits_deg)):
        lo,hi = map(math.radians,bounds)
        radius = math.radians(dict((key,step) for key,_,step in MOTION_AMPLITUDES)[amplitude] if amplitude else (20. if axis < 3 else 30.))
        low,high = max(lo,actual-radius),min(hi,actual+radius)
        if axis == 4 and preference in ('wrist_positive','wrist_negative'):
            if preference == 'wrist_positive':low,high = max(0.,low),high
            else:low,high = low,min(0.,high)
            if low >= high:
                low,high = (max(0.,lo),hi) if preference == 'wrist_positive' else (lo,min(0.,hi))
        if not all(math.isfinite(v) for v in (lo,hi)) or low >= high:
            raise ValueError('当前关节位置不足以生成范围内的随机目标')
        result.append(rng.uniform(low,high))
    return tuple(result)


def nearest_ik_goal(start, goal, limits_deg):
    """参考MoveL的范围筛选/最小最大行程原则，为IK解选择最近合法等效圈数。

    仅在IK之后、路径规划之前使用；显式关节多圈目标保持原意。
    """
    if len(start) != 6 or len(goal) != 6 or len(limits_deg) != 6:
        raise ValueError('逆解与实际起点必须包含六轴')
    selected = []
    for actual, target, bounds in zip(start, goal, limits_deg):
        lo, hi = map(math.radians, bounds)
        if not all(math.isfinite(v) for v in (actual, target, lo, hi)) or lo >= hi:
            raise ValueError('逆解、实际起点或范围无效')
        first = math.ceil((lo-target)/math.tau)
        last = math.floor((hi-target)/math.tau)
        if first > last:
            raise ValueError('逆解不在当前工作范围')
        turn = min(last, max(first, round((actual-target)/math.tau)))
        equivalent = target+turn*math.tau
        if not lo <= equivalent <= hi:
            raise ValueError('等效逆解超过工作范围')
        selected.append(equivalent)
    return tuple(selected)


def validate_trajectory(trajectory, names, limits_deg):
    """返回原顺序的全部路点（弧度）；不裁剪、不只取末点。"""
    actual = list(trajectory.joint_names)
    if len(names) != 6 or len(set(actual)) != 6 or set(actual) != set(names):
        raise ValueError('规划轨迹六轴名称缺失或重复')
    if len(limits_deg) != 6 or not trajectory.points:
        raise ValueError('规划路径或工作范围为空')
    order = [actual.index(name) for name in names]
    result, previous = [], -1
    for point in trajectory.points:
        timestamp = point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
        if timestamp < 0 or timestamp <= previous:
            raise ValueError('规划路点时间不严格递增')
        previous = timestamp
        if len(point.positions) != 6:
            raise ValueError('规划路点不包含完整六轴')
        positions = tuple(point.positions[index] for index in order)
        if any(not math.isfinite(value) or not lo <= math.degrees(value) <= hi
               for value, (lo, hi) in zip(positions, limits_deg)):
            raise ValueError('规划路径超过工作范围或含无效角度')
        result.append(positions)
    return tuple(result)


def rpy_quaternion(roll_deg, pitch_deg, yaw_deg):
    if not all(math.isfinite(v) for v in (roll_deg, pitch_deg, yaw_deg)):
        raise ValueError('姿态角必须有限')
    r, p, y = (math.radians(v) / 2 for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class MoveItPlanner:
    """call_async 返回 future，由原面板 rclpy.spin_once 驱动。"""
    def __init__(self, node, names, limits_deg):
        from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
        from rclpy.qos import QoSProfile, qos_profile_services_default
        self.node, self.names, self.limits = node, list(names), limits_deg
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.plan = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        # 每批8个请求；重复服务实例可能返回多份响应，默认depth=10会丢掉未处理的响应。
        # 保持服务的可靠性/持久性语义，只增加此只读客户端的响应容量。
        validity_qos = QoSProfile(
            depth=32, history=qos_profile_services_default.history,
            reliability=qos_profile_services_default.reliability,
            durability=qos_profile_services_default.durability)
        self.validity = node.create_client(GetStateValidity, '/check_state_validity',
                                           qos_profile=validity_qos)

    def robot_state(self, positions):
        from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject
        from sensor_msgs.msg import JointState
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        from std_msgs.msg import Header
        if len(positions) != 6 or not all(math.isfinite(v) for v in positions):
            raise ValueError('实际六轴位置未就绪')
        # 物理地面为base_link Z0；每次请求显式带入地面上方55mm禁入保护体。
        # 仅豁免固定基座自身与地面的安装接触，其余连杆/工具必须高于模型地面并留约50mm间隙，容纳实机回差及线缆外形。
        pose = Pose();pose.position.z = -1.+.055;pose.orientation.w = 1.
        floor = CollisionObject(header=Header(frame_id='base_link'),id='manual_base_floor',
            primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[20.,20.,2.])],
            primitive_poses=[pose],operation=CollisionObject.ADD)
        attached = AttachedCollisionObject(link_name='base_link',object=floor,touch_links=['base_link','clearance_base_link'])
        return RobotState(joint_state=JointState(name=self.names, position=list(positions)),
                          attached_collision_objects=[attached],is_diff=True)

    def request_fk(self, positions):
        from moveit_msgs.srv import GetPositionFK
        from std_msgs.msg import Header
        return self.fk.call_async(GetPositionFK.Request(
            header=Header(frame_id='base_link'), fk_link_names=['link6_1_1'],
            robot_state=self.robot_state(positions)))

    def request_ik(self, positions, xyz_mm, rpy_deg, preference='nearest', home=None, rest=None, seed_goal=None):
        # 两个种子探索腕部解支，再沿原固件MoveL原则筛选；不是解析八解求解器。
        seeds = [tuple(positions)]
        if seed_goal is not None and tuple(seed_goal) not in seeds:seeds.append(tuple(seed_goal))
        other = list(positions)
        other[3] = math.remainder(other[3]+math.pi, math.tau)
        other[4] = -other[4] if abs(other[4]) > .01 else -.35
        other[5] += math.pi
        other = [min(math.radians(hi),max(math.radians(lo),q)) if axis != 5 else q
                 for axis,(q,(lo,hi)) in enumerate(zip(other,self.limits))]
        try:
            other = nearest_ik_goal(positions,other,self.limits)
            if other != seeds[0]:seeds.append(other)
        except ValueError:
            pass
        if preference == 'near_home' and home is not None:
            seed = tuple(math.radians(q) for q in home)
            if seed not in seeds:seeds.append(seed)
        requests = [self._request_ik_single(seed,xyz_mm,rpy_deg) for seed in seeds]
        combined = Future()
        def collect(_):
            if combined.done() or not all(f.done() for f in requests):return
            valid, failures = [], []
            for future in requests:
                try:
                    response = future.result()
                    if response is None:continue
                    if response.error_code.val != 1:
                        failures.append(response)
                        continue
                    state = response.solution.joint_state
                    by_name = dict(zip(state.name,state.position))
                    home_rad = tuple(math.radians(q) for q in home) if home is not None else None
                    reference = home_rad if preference == 'near_home' and home_rad is not None else positions
                    goal = nearest_ik_goal(reference,[by_name[name] for name in self.names],self.limits)
                    valid.append((ik_goal_cost(positions,goal,preference,home_rad),response,goal))
                except (ValueError,KeyError,RuntimeError):
                    continue
            if valid:
                selected = min(valid,key=lambda pair:pair[0])
                combined.selected_goal = selected[2]
                combined.set_result(selected[1])
            elif failures:
                combined.set_result(failures[-1])
            else:
                combined.set_exception(ValueError('逆解服务没有返回有效候选，请缩小目标或换偏好后重试'))
        for request in requests:request.add_done_callback(collect)
        def cancel_children(future):
            if future.cancelled():
                for request in requests:request.cancel()
        combined.add_done_callback(cancel_children)
        return combined

    def _request_ik_single(self, positions, xyz_mm, rpy_deg):
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import PositionIKRequest
        from moveit_msgs.srv import GetPositionIK
        if len(xyz_mm) != 3 or not all(math.isfinite(v) for v in xyz_mm):
            raise ValueError('末端位置必须为有限 XYZ')
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (v/1000 for v in xyz_mm)
        q = rpy_quaternion(*rpy_deg)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        return self.ik.call_async(GetPositionIK.Request(ik_request=PositionIKRequest(
            group_name='dummy_arm', robot_state=self.robot_state(positions),
            avoid_collisions=True, ik_link_name='link6_1_1', pose_stamped=pose,
            timeout=Duration(nanosec=120000000))))

    def request_plan(self, start, goal):
        # 优先快速规划；没有找到路径时对同一目标自动增加搜索时间，不能把性能目标当限位。
        combined = Future()
        requests = []
        def submit(extended=False,wrist_window=True,precise=False):
            child = self._request_plan_single(start,goal,extended,wrist_window=wrist_window,precise=precise)
            requests.append(child)
            def collect(future):
                if combined.done():return
                try:
                    response = future.result()
                    if response and response.motion_plan_response.error_code.val in (-1,-2,-6,99999) and wrist_window:
                        combined.retried = True
                        # 先增加局部搜索预算，避免一次超时就退回多圈随机绕路。
                        # 时间参数化后发现碰撞时提高几何检查精度，避免反复返回
                        # 同一条漏采样直线；普通路径仍保留快速配置。
                        submit(True,wrist_window=not extended,
                               precise=precise or response.motion_plan_response.error_code.val in (-2,99999))
                    else:combined.set_result(response)
                except Exception as error:
                    combined.set_exception(error)
            child.add_done_callback(collect)
        def cancel_children(future):
            if future.cancelled():
                for child in requests:child.cancel()
        combined.add_done_callback(cancel_children)
        submit()
        return combined

    def _request_plan_single(self, start, goal, extended=False,wrist_window=True,precise=False):
        from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
        from moveit_msgs.srv import GetMotionPlan
        # 仅校验工作域，不改变 IK 解。
        if len(goal) != 6 or any(not math.isfinite(q) or not lo <= math.degrees(q) <= hi
                              for q, (lo, hi) in zip(goal, self.limits)):
            raise ValueError('逆解不在当前工作范围')
        speed,acceleration,speed_limit,acceleration_limit=getattr(
            self,'motion_parameters',lambda:(60.,4.,63.,4.))()
        if (not all(math.isfinite(v) and v>0. for v in (speed,acceleration,speed_limit,acceleration_limit)) or
                speed>speed_limit or acceleration>acceleration_limit):
            raise ValueError('规划动态参数超过部署限制')
        # J6可跨多圈；全域随机采样会为很近的目标生成数百度往返绕路。
        # 优先在起终点外各留30°的窗口中搜索；两轮失败后仍可回退到完整工作域。
        path_constraints=Constraints()
        if wrist_window:
            lo=max(math.radians(self.limits[5][0]),min(start[5],goal[5])-math.radians(30.))
            hi=min(math.radians(self.limits[5][1]),max(start[5],goal[5])+math.radians(30.))
            center=(lo+hi)/2.
            path_constraints.joint_constraints=[JointConstraint(
                joint_name=self.names[5],position=center,tolerance_below=center-lo,
                tolerance_above=hi-center,weight=1.)]
        return self.plan.call_async(GetMotionPlan.Request(motion_plan_request=MotionPlanRequest(
            start_state=self.robot_state(start), group_name='dummy_arm',
            planner_id='RRTConnectPrecise' if precise else 'RRTConnectFast',
            path_constraints=path_constraints,
            goal_constraints=[Constraints(joint_constraints=[JointConstraint(
                joint_name=name, position=q, tolerance_above=0.0001,
                tolerance_below=0.0001, weight=1.) for name, q in zip(self.names, goal)])],
            num_planning_attempts=2 if extended else 1, allowed_planning_time=2. if extended else .25,
            max_velocity_scaling_factor=speed/speed_limit,
            max_acceleration_scaling_factor=acceleration/acceleration_limit)))

    def request_validity(self, positions):
        from moveit_msgs.srv import GetStateValidity
        return self.validity.call_async(GetStateValidity.Request(
            robot_state=self.robot_state(positions), group_name='dummy_arm'))
