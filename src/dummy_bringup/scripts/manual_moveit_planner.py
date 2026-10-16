"""面板的异步 MoveIt 服务适配；不持有 USB、不执行运动。"""
import math


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
        self.node, self.names, self.limits = node, list(names), limits_deg
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.plan = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')

    def robot_state(self, positions):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        if len(positions) != 6 or not all(math.isfinite(v) for v in positions):
            raise ValueError('实际六轴位置未就绪')
        return RobotState(joint_state=JointState(name=self.names, position=list(positions)))

    def request_fk(self, positions):
        from moveit_msgs.srv import GetPositionFK
        from std_msgs.msg import Header
        return self.fk.call_async(GetPositionFK.Request(
            header=Header(frame_id='base_link'), fk_link_names=['link6_1_1'],
            robot_state=self.robot_state(positions)))

    def request_ik(self, positions, xyz_mm, rpy_deg):
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
            timeout=Duration(sec=2))))

    def request_plan(self, start, goal):
        from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
        from moveit_msgs.srv import GetMotionPlan
        # 仅校验工作域，不改变 IK 解。
        if len(goal) != 6 or any(not math.isfinite(q) or not lo <= math.degrees(q) <= hi
                              for q, (lo, hi) in zip(goal, self.limits)):
            raise ValueError('逆解不在当前工作范围')
        return self.plan.call_async(GetMotionPlan.Request(motion_plan_request=MotionPlanRequest(
            start_state=self.robot_state(start), group_name='dummy_arm',
            goal_constraints=[Constraints(joint_constraints=[JointConstraint(
                joint_name=name, position=q, tolerance_above=0.0001,
                tolerance_below=0.0001, weight=1.) for name, q in zip(self.names, goal)])],
            num_planning_attempts=2, allowed_planning_time=3.,
            max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1)))

    def request_validity(self, positions):
        from moveit_msgs.srv import GetStateValidity
        return self.validity.call_async(GetStateValidity.Request(
            robot_state=self.robot_state(positions), group_name='dummy_arm'))
