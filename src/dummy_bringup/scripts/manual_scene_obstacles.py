##
# @file manual_scene_obstacles.py
# @brief 维护可编辑圆柱场景及“停车后再应用”的持久化事务。
# 所有界面坐标为毫米，安装孔基准仅为地面 XY 平移；MoveIt 几何统一转到 base_link 米制。修改先作废旧规划/未来连续链，再等待设备取消事件和后续新鲜 Hold，最后读取/应用场景并保存，成功前一直封锁规划执行。
"""可持久化圆柱场景；只有MoveIt确认和停车确认均完成后才开放规划。"""
import json
import copy
import math
import os
from pathlib import Path
import re
import time
import uuid
from manual_moveit_planner import require_unique_moveit, MoveItDiscoveryPending

##
# 仅管理此前缀的世界圆柱；known_ids 保留删除历史，避免重启后旧障碍留在 MoveIt。
##
# @par 历史ID与实际世界
# known_ids只给删除候选；tick先Get WORLD_OBJECT_NAMES，scene_message过滤真实不存在ID，不能把持久化删除历史当当前世界仍存在。
PREFIX = 'manual_cylinder_'


##
# @brief 校验地面 XY 基准并补入不可修改的 base_link 原点。
# 名称非空且不超过 80 字符；每轴有限、非 bool 且绝对值不超过 100000 mm；base_link 必须为 (0,0)。此基准不接受旋转或 Z 偏移。
# @param frames 基准名到 [X,Y] 毫米偏移的映射。
# @return dict：偏移转换为 float 二元组后的独立字典。
def validate_origins(frames):
    result = {'base_link': (0., 0.)}
    for name, offset in frames.items():
        if (not isinstance(name, str) or not name.strip() or len(name) > 80 or
                not isinstance(offset, (list, tuple)) or len(offset) != 2 or
                any(type(v) not in (float, int) or not math.isfinite(v) or abs(v) > 100000 for v in offset)):
            raise ValueError('安装孔基准需名称及有限XY毫米偏移（不超过100000mm）')
        if name == 'base_link' and tuple(offset) != (0., 0.):
            raise ValueError('禁止修改地面/J1轴心base_link原点')
        result[name] = tuple(float(v) for v in offset)
    return result


##
# @brief 校验圆柱 ID、参考基准和有限毫米尺寸。
# ID 仅允许 1..80 位字母、数字、下划线和连字符且不得重复；直径/高度必须正，位置/尺寸绝对值至多 100000 mm，拒绝布尔值。
# @param rows 各圆柱字典，含 id/frame/x_mm/y_mm/diameter_mm/height_mm。
# @param frames 允许的基准名称集合或字典，默认只有 base_link。
# @return list[dict]：规范化 float 毫米值后的新行列表。
def validate_obstacles(rows, frames=('base_link',)):
    result, seen = [], set()
    for row in rows:
        identifier = row['id']
        if not isinstance(identifier, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,80}', identifier) or identifier in seen:
            raise ValueError('障碍物ID无效或重复')
        seen.add(identifier)
        if row['frame'] not in frames:
            raise ValueError('障碍物坐标基准不存在，需先配置对应地面/安装孔基准')
        out = {'id': identifier, 'frame': row['frame']}
        for key in ('x_mm', 'y_mm', 'diameter_mm', 'height_mm'):
            value = row[key]
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError('障碍物尺寸和位置必须为有限毫米数')
            if abs(value) > 100000 or (key in ('diameter_mm', 'height_mm') and value <= 0):
                raise ValueError('障碍物尺寸必须大于0，且位置/尺寸不得超过100000mm')
            out[key] = float(value)
        result.append(out)
    return result


##
# @brief 将基准内的贴地圆柱换算为 base_link 几何。
# 中心 Z=高度/2，轴向保持竖直，底面恒为 Z=0；安装孔偏移只作用于 XY。
# @param row 经过校验的毫米制圆柱字典。
# @param frames 基准名到 XY 毫米偏移的映射。
# @return tuple：中心 x/y/z、完整高度、半径，全部为米。
def cylinder_geometry(row, frames):
    """基准仅平移地面XY原点；统一在base_link中保持竖直，底面Z=0。"""
    origin = frames[row['frame']]
    return ((origin[0] + row['x_mm']) / 1000., (origin[1] + row['y_mm']) / 1000.,
            row['height_mm'] / 2000., row['height_mm'] / 1000., row['diameter_mm'] / 2000.)


##
# @brief 构造当前圆柱集合的 MoveIt 增删差量。
# 对象 ID 添加 manual_cylinder_ 前缀，避免误删其他场景实体；CYLINDER dimensions 顺序为 [高度,半径] 米。函数不调用服务。
# @param rows 待应用的已验证圆柱行。
# @param known_ids 本面板历次管理过的无前缀 ID 集合，用于识别删除对象。
# @param frames 毫米 XY 基准映射。
# @param existing_world_ids 可选 MoveIt 现存完整 ID 集合；提供时不发送不存在对象的删除。
# @return PlanningScene：is_diff=True，包含旧圆柱 REMOVE 与当前圆柱 ADD。
def scene_message(rows, known_ids, frames, existing_world_ids=None):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import Header
    objects = []
    present = {row['id'] for row in rows}
    for identifier in sorted(set(known_ids) - present):
        if existing_world_ids is not None and PREFIX+identifier not in existing_world_ids:
            continue
        objects.append(CollisionObject(header=Header(frame_id='base_link'), id=PREFIX+identifier,
                                       operation=CollisionObject.REMOVE))
    for row in rows:
        x, y, z, height, radius = cylinder_geometry(row, frames)
        pose = Pose(); pose.position.x = x; pose.position.y = y; pose.position.z = z; pose.orientation.w = 1.
        objects.append(CollisionObject(header=Header(frame_id='base_link'), id=PREFIX+row['id'],
            primitives=[SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[height, radius])],
            primitive_poses=[pose], operation=CollisionObject.ADD))
    return PlanningScene(is_diff=True, world=PlanningSceneWorld(collision_objects=objects))


##
# @brief 持久化已确认圆柱及历史 ID 和基准。
# 先写同目录 .tmp 文件、flush 和 fsync，再 replace 目标；保存版本固定为 1。文件成功写入不代替 MoveIt 应用回执，外层在服务成功后调用。
# @param path 目标 JSON 文件路径，自动创建父目录。
# @param rows 已确认的圆柱行，数值单位 mm。
# @param known_ids 全部历史管理 ID，排序后保存以支持重启删除。
# @param frames 可选基准映射；省略时保存 base_link 零原点。
# @return None；执行上述状态更新、界面或日志操作。
def save_obstacles(path, rows, known_ids, frames=None):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as stream:
        json.dump({'version': 1, 'obstacles': rows, 'known_ids': sorted(known_ids),
                   'reference_origins_mm': frames or {'base_link': [0., 0.]}}, stream, ensure_ascii=False, indent=2)
        stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)


##
# @brief 生成用于 RViz 的圆柱可视化数组。
# 直径/高度转为米制 marker.scale；这些可视标记与 MoveIt 碰撞体分别发布，标记出现本身不证明场景事务成功。
# @param rows 已确认圆柱行，单位 mm。
# @param frames 基准 XY 毫米偏移。
# @return MarkerArray：先 DELETEALL，再发布 base_link 中橙色圆柱。
def obstacle_markers(rows, frames):
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Header
    markers = [Marker(action=Marker.DELETEALL)]
    for index, row in enumerate(rows):
        x, y, z, height, radius = cylinder_geometry(row, frames)
        marker = Marker(header=Header(frame_id='base_link'), ns='manual_obstacles',
                        id=index, type=Marker.CYLINDER, action=Marker.ADD)
        marker.pose.position.x=x; marker.pose.position.y=y; marker.pose.position.z=z
        marker.pose.orientation.w=1.
        marker.scale.x=marker.scale.y=radius*2.; marker.scale.z=height
        marker.color.r=1.; marker.color.g=.4; marker.color.b=.1; marker.color.a=.85
        markers.append(marker)
    return MarkerArray(markers=markers)


##
# @brief 在已有允许碰撞矩阵中仅豁免保护地面与圆柱重叠。
# 校验原矩阵为方阵；新增默认项为 False，仅 manual_base_floor 与圆柱配对改为 True。机器人连杆与圆柱的碰撞检查不会被整体豁免。
# @param existing MoveIt 读取的 AllowedCollisionMatrix。
# @param identifiers 要加入矩阵的圆柱无前缀 ID 集合。
# @return AllowedCollisionMatrix：独立深拷贝，保留原矩阵并对称扩展所需项。
def obstacle_acm(existing, identifiers):
    """保留SRDF/guard矩阵；只豁免附着安全地面与场景圆柱之间的重叠。"""
    from moveit_msgs.msg import AllowedCollisionEntry
    acm = copy.deepcopy(existing)
    count = len(acm.entry_names)
    if len(acm.entry_values) != count or any(len(row.enabled) != count for row in acm.entry_values):
        raise ValueError('MoveIt返回的允许碰撞矩阵尺寸无效')
    names = ['manual_base_floor'] + [PREFIX+identifier for identifier in sorted(identifiers)]
    for name in names:
        if name in acm.entry_names: continue
        for row in acm.entry_values: row.enabled.append(False)
        acm.entry_names.append(name)
        acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
    floor = acm.entry_names.index('manual_base_floor')
    for name in names[1:]:
        cylinder = acm.entry_names.index(name)
        acm.entry_values[floor].enabled[cylinder] = True
        acm.entry_values[cylinder].enabled[floor] = True
    return acm


##
# @class ObstacleScene
# @brief 把停车屏障、场景读取/应用和持久化组成单次事务。
# desired 是待应用输入，rows 是成功确认的行；scene_revision 在请求变更时递增；blocked/scene_ready 门控规划；future 只允许一个在途服务。失败保持封锁，服务超时保留 future 防止旧请求迟到乱序应用。
class ObstacleScene:
    """可测试事务状态机。更新失败保持封锁，重试必须重新获服务成功回执。"""
    ##
    # @brief 创建场景服务及持久化标记发布器并恢复期望场景。
    # 初始 phase=read、blocked=True；等待 MoveIt 确认后才允许规划。已保存版本必须为 1，历史 ID 也检查格式；读文件失败调用 fail，不应用部分数据。
    # @param panel 主面板，提供 ROS 节点、设备状态、取消及 MoveIt 页。
    # @param path 圆柱配置 JSON 路径。
    # @param frames 未保存用户基准时使用的默认毫米 XY 映射。
    # @return None；执行上述状态更新、界面或日志操作。
    def __init__(self, panel, path, frames):
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        self.panel, self.path, self.frames = panel, Path(path), validate_origins(frames)
        self.client = panel.node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_client = panel.node.create_client(GetPlanningScene, '/get_planning_scene')
        from visualization_msgs.msg import MarkerArray
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.publisher = panel.node.create_publisher(MarkerArray, '/manual_obstacle_markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.rows, self.desired, self.known_ids = [], [], set()
        ##
        # scene_revision 代表本面板期望场景版本，不是 MoveIt 全局版本；规划和自动测试保存此版本并在变化时作废。
        self.scene_revision = 0
        self.scene_ready = False
        self.blocked = True
        self.future = None
        self.phase = 'read'
        self.wait_hold = False
        self.hold_marker = None
        self.status = '等待MoveIt确认保存的障碍物场景'
        self.error = None
        self.deadline = time.monotonic() + 20.
        self.initial_sync_pending = True
        self.stop_deadline = None
        self.on_change = lambda: None
        try:
            if self.path.exists():
                data = json.loads(self.path.read_text())
                if data.get('version') != 1: raise ValueError('障碍物文件版本不支持')
                self.frames = validate_origins(data.get('reference_origins_mm', self.frames))
                self.desired = validate_obstacles(data['obstacles'], self.frames)
                self.known_ids = {row['id'] for row in self.desired} | set(data.get('known_ids', []))
                if any(not isinstance(i, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,80}', i) for i in self.known_ids):
                    raise ValueError('保存的障碍物ID无效')
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.fail('读取障碍物失败：'+str(error))

    ##
    # @brief 记录场景失败并持续封锁规划入口。
    # 设置 error/status/blocked，清 scene_ready 并通知 UI；不隐式取消 future 或把未知服务状态当作未应用。
    # @param text 供操作员显示的失败原因。
    # @return None；执行上述状态更新、界面或日志操作。
    def fail(self, text):
        self.error = text; self.status = text; self.blocked = True; self.scene_ready = False
        self.on_change()

    ##
    # @brief 受理场景变更并按是否运动建立停车屏障。
    # 在途服务或等待 Hold 时拒绝新事务。检测 busy、生命周期、速度、执行及自动测试；先提高版本、封锁场景并清理旧规划。若可能运动则记下 boot/reference/session 与 state_seq，给 6 s 停车期限，再调用受控取消；随后才进入最多 20 s 场景同步。
    # @param rows 期望的完整圆柱集合。
    # @param frames 可选新基准映射；省略使用当前基准。
    # @return None；执行上述状态更新、界面或日志操作。
    def request_change(self, rows, frames=None):
        if self.future is not None or self.wait_hold:
            raise ValueError('场景更新仍在等待回执或停车，请稍后重试')
        frames = validate_origins(self.frames if frames is None else frames)
        rows = validate_obstacles(rows, frames)
        p = self.panel
        moving = bool(p.busy or getattr(p, 'cancel_hold_pending', False))
        if p.last_state:
            moving = moving or p.last_state.get('core_state') in (3, 5, 6)
            moving = moving or any(not math.isfinite(v) or abs(v) > .01
                                  for v in p.last_state.get('estimated_velocity_rad_s', ()))
        tab = getattr(p, 'moveit_tab', None)
        if tab:
            moving = moving or tab.executing or tab.auto_test.active
        self.desired = rows
        self.frames = frames
        self.known_ids |= {r['id'] for r in rows}
        self.scene_revision += 1
        self.scene_ready = False; self.blocked = True; self.error = None
        ##
        # @brief 显式新场景事务撤去上次公共/Get owner组合锚。
        # 只有重新拓扑验证及Get→Apply成功才可ready；图恢复本身不复活旧path。初次对象未置该字段时validate_topology以getattr视为未绑定。
        self.service_owner = None
        self.phase = 'read'
        self.hold_marker = None
        self.wait_hold = moving
        self.stop_identity = ((p.last_state['boot_epoch'], p.last_state['reference_epoch'], p.last_state['session_id'])
                              if moving and p.last_state else None)
        self.stop_state_seq = p.last_state.get('state_seq', 0) if moving and p.last_state else 0
        if tab:
            # 先作废异步规划和未来链，避免停车回执与旧队列竞争。
            tab.scene_change_requested()
        if moving:
            self.status = '等待受控停车事件及新鲜实测Hold，尚未修改碰撞场景'
            self.stop_deadline = time.monotonic()+6.
            p.cancel_motion_hold()
        else:
            self.status = '等待MoveIt应用障碍物场景'
        self.deadline = time.monotonic()+20.
        self.on_change()

    ##
    # @brief 保存与停车事务身份相符的取消保持事件水位。
    # 只在 wait_hold 且 MOVE_CANCELED_HOLD 时接受；身份必须相同，事件 state_seq 不早于请求前状态。此事件本身不解除封锁，还须 hold_confirmed 核对后续实际反馈。
    # @param event Native 事件标识。
    # @param parts 解析得到的事件键值字典，boot 为十六进制文本。
    # @return None；执行上述状态更新、界面或日志操作。
    def ros_event(self, kind, value):
        """仅公开Action取消终态建立屏障水位；后续仍须独立新鲜Hold。"""
        if not self.wait_hold or kind != 'canceled' or not isinstance(value, dict) or value.get('status') != 5:
            return
        state = self.panel.last_state
        if state is None:
            return
        identity = tuple(state[key] for key in ('boot_epoch', 'reference_epoch', 'session_id'))
        if identity == self.stop_identity and state['state_seq'] >= self.stop_state_seq:
            self.hold_marker = (identity, state['state_seq'], state['motion_epoch'])

    def native_event(self, event, parts):
        if self.wait_hold and event == 'MOVE_CANCELED_HOLD':
            try:
                identity = (int(parts['boot'], 16), int(parts['reference']), int(parts['session']))
                if identity == self.stop_identity and int(parts['state_seq']) >= self.stop_state_seq:
                    self.hold_marker = (identity, int(parts['state_seq']), int(parts['motion']))
            except (KeyError, ValueError):
                pass

    ##
    # @brief 新增/修改孔基准，或删除基准并保持圆柱世界位置。
    # 删除时把受影响圆柱坐标加上旧基准偏移并改成 base_link；修改时圆柱随基准一起平移。所有变更交给 request_change 走同一停车/同步流程。
    # @param name 基准名称；base_link 禁止修改/删除。
    # @param offset 新 XY 毫米二元组；None 表示删除。
    # @return None；执行上述状态更新、界面或日志操作。
    def request_origin_change(self, name, offset=None):
        if name == 'base_link': raise ValueError('base_link原点不可修改或删除')
        frames = dict(self.frames)
        rows = [dict(row) for row in self.desired]
        if offset is None:
            if name not in frames: raise ValueError('安装孔基准不存在')
            previous = frames.pop(name)
            for row in rows:
                if row['frame'] == name:
                    row['x_mm'] += previous[0]; row['y_mm'] += previous[1]; row['frame'] = 'base_link'
        else:
            frames[name] = offset
        self.request_change(rows, frames)

    ##
    # @brief 核对取消事件之后的新鲜实际 Hold 已稳定。
    # 要求 core_state=4、hold_ready、六轴掩码 63；保持位置与 actual_q 逐轴误差 ≤0.003 rad，速度绝对值 ≤0.01 rad/s。位置接近或收到取消事件单独都不足以放行。
    # @return bool：同身份、同 motion、新 state_seq、六轴 Hold/位置/速度全有效且保持位置吻合。
    def hold_confirmed(self):
        p, marker = self.panel, self.hold_marker
        if not marker or not p.fresh() or not hasattr(p, 'actual_q'): return False
        state = p.last_state; identity, sequence, motion = marker
        held = state.get('hold_position_rad', ())
        velocities = state.get('estimated_velocity_rad_s', ())
        return ((state['boot_epoch'], state['reference_epoch'], state['session_id']) == identity and
            state['state_seq'] > sequence and state['motion_epoch'] == motion and
            state['core_state'] == 4 and state.get('hold_ready') and state.get('hold_valid_mask') == 63 and
            state.get('position_valid_mask') == 63 and state.get('velocity_valid_mask') == 63 and
            len(held) == len(p.actual_q) == len(velocities) == 6 and
            all(math.isfinite(h) and math.isfinite(q) and abs(h-q) <= .003 for h,q in zip(held,p.actual_q)) and
            all(math.isfinite(v) and abs(v) <= .01 for v in velocities))

    ##
    # @brief 复核并锚定当前公共/Get服务owner组合，失效撤ready/旧路径且按实测运动请求Hold。
    # 初次尚无owner、无future、未ready且仍在20s场景期限内，MoveItDiscoveryPending只等待；已有事务/ready后缺服务、重复、换owner或未知查询失败均fail闭锁。
    # 首次fail才通知MoveIt页清路径/自动队列，并检查busy、执行/自动会话、State core3/5/6或非有限/超过0.01rad/s速度决定受控Hold。Hold请求不是停车确认。
    # @return bool：当前图满足唯一实例/同owner组合且未换来源为True；等待或故障False。
    def validate_topology(self):
        """每次使用场景前复核唯一服务来源；失效立即撤销路径，运动中请求停车。"""
        try:
            owner = require_unique_moveit(self.panel.node)
            previous = getattr(self, 'service_owner', None)
            if previous is not None and owner != previous:
                raise ValueError('MoveIt服务提供节点已改变，请重新同步圆柱场景')
            self.service_owner = owner
            return True
        except ValueError as error:
            if (isinstance(error, MoveItDiscoveryPending) and
                    getattr(self, 'service_owner', None) is None and self.future is None and
                    not self.scene_ready and time.monotonic() <= self.deadline):
                self.status = str(error)
                return False
            first_failure = not self.error
            self.fail(str(error))
            if first_failure:
                tab = getattr(self.panel, 'moveit_tab', None)
                moving = self.panel.busy or (tab is not None and (tab.executing or tab.auto_test.active))
                state = self.panel.last_state
                if state:
                    moving = moving or state.get('core_state') in (3, 5, 6)
                    moving = moving or any(not math.isfinite(v) or abs(v) > .01
                                          for v in state.get('estimated_velocity_rad_s', ()))
                if tab is not None:
                    tab.scene_change_requested()
                if moving:
                    self.panel.cancel_motion_hold()
            return False

    ##
    # @brief 推进停车、读取 ACM、应用差量、保存和解除封锁。
    # 先等待 Hold 屏障，再请求当前 ACM/世界对象名，构造差量及保留原有豁免的 ACM，随后 ApplyPlanningScene。成功回执、发布标记及保存均完成才置 scene_ready；异常保持封锁。服务回执超时不丢弃 future，也不允许新事务与迟到应用竞争。
    # @return None；执行上述状态更新、界面或日志操作。
    # error为空时每tick先重核拓扑，scene_ready也不能跳过；已有error则持续闭锁，不因服务自行恢复就重用旧规划。
    def tick(self):
        if self.error: return
        # 初次预算从实际消费场景状态开始，不计连接前人工确认的模态等待。
        if getattr(self, 'initial_sync_pending', False):
            self.initial_sync_pending = False
            if self.future is None and not self.scene_ready and self.scene_revision == 0:
                self.deadline = time.monotonic() + 20.
        if not self.validate_topology(): return
        now = time.monotonic()
        if self.wait_hold:
            if self.hold_confirmed():
                self.wait_hold = False; self.status = '停车已确认，等待MoveIt更新场景'
                self.deadline = now+20.
            elif now > self.stop_deadline:
                self.fail('停车未获事件及新鲜实测Hold确认；未修改场景，规划保持禁用')
                # 不允许失败后直接重试绕过停车屏障。
                return
            else: return
        ##
        # future 回执串行推进 read→apply；收到响应仍需核对 success 并完成本地保存，不能仅因 future.done() 解除门控。
        if self.future is not None:
            if self.future.done():
                future, self.future = self.future, None
                try:
                    response = future.result()
                    if self.phase == 'read':
                        if response is None: raise ValueError('未收到允许碰撞矩阵')
                        scene = scene_message(self.desired, self.known_ids, self.frames,
                            {obj.id for obj in response.scene.world.collision_objects})
                        scene.allowed_collision_matrix = obstacle_acm(
                            response.scene.allowed_collision_matrix, self.known_ids)
                        from moveit_msgs.srv import ApplyPlanningScene
                        if not self.client.service_is_ready(): raise ValueError('ApplyPlanningScene未就绪')
                        self.future = self.client.call_async(ApplyPlanningScene.Request(scene=scene))
                        self.phase = 'apply'; self.deadline = now+10.
                        return
                    if response is None or not response.success: raise ValueError('MoveIt拒绝场景')
                    self.publisher.publish(obstacle_markers(self.desired, self.frames))
                    save_obstacles(self.path, self.desired, self.known_ids, self.frames)
                except Exception as error:
                    self.fail('场景应用/保存失败：'+str(error)); return
                self.rows = [dict(row) for row in self.desired]
                self.scene_ready = True; self.blocked = False
                self.status = f'场景已确认并保存 · {len(self.rows)}个圆柱 · 版本{self.scene_revision}'
                self.on_change()
            elif now > self.deadline:
                # 不丢弃future：服务可能稍后仍应用，禁止新请求与旧请求发生乱序。
                self.fail('场景服务回执超时；状态不确定，请等待服务恢复后重新打开面板')
            return
        if self.scene_ready: return
        if now > self.deadline:
            self.fail('MoveIt场景服务未就绪，禁止规划与执行'); return
        if self.client.service_is_ready() and self.get_client.service_is_ready():
            from moveit_msgs.srv import GetPlanningScene
            from moveit_msgs.msg import PlanningSceneComponents
            try:
                self.phase = 'read'
                self.future = self.get_client.call_async(GetPlanningScene.Request(
                    components=PlanningSceneComponents(components=(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                                                                 PlanningSceneComponents.WORLD_OBJECT_NAMES))))
                self.deadline = now+10.
            except Exception as error:
                self.fail('提交场景失败：'+str(error))


##
# @brief 创建圆柱/安装孔编辑界面及事务控制器。
# 输入全部为毫米；无默认 CAD 安装孔位置。列表展示 desired，状态文字明确是否已同步；修改动作不直接写 MoveIt，统一经过 scene.request_change。
# @param panel 主面板，含默认基准和可选 obstacles_file 配置。
# @return tuple[QWidget,ObstacleScene]：页内容和由主面板 tick/native_event 驱动的状态机。
def create_obstacle_tab(panel):
    from PyQt5 import QtWidgets
    widget = QtWidgets.QWidget(panel)
    layout = QtWidgets.QVBoxLayout(widget)
    # 所有基准是地面内的毫米平移，不接受倾斜TF，保证圆柱始终竖直贴地。
    frames = validate_origins(panel.cfg.get('obstacle_reference_origins_mm', {}))
    path = panel.cfg.get('obstacles_file', str(Path.home()/'.config/dummy_manual_panel/obstacles.json'))
    scene = ObstacleScene(panel, path, frames)
    label = QtWidgets.QLabel('X/Y相对于地面/J1轴心或已配置安装孔原点；直径/高度均为mm。圆柱底贴地、竖直向上。')
    label.setWordWrap(True); layout.addWidget(label)
    table = QtWidgets.QTableWidget(0, 6)
    table.setHorizontalHeaderLabels(['ID', '坐标基准', 'X (mm)', 'Y (mm)', '直径 (mm)', '高度 (mm)'])
    table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
    table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
    layout.addWidget(table)
    form = QtWidgets.QFormLayout(); combo = QtWidgets.QComboBox(); combo.addItems(list(scene.frames))
    form.addRow('地面 / 安装孔基准', combo)
    inputs = []
    for title, initial, positive in [('X', 0., False), ('Y', 0., False), ('直径', 50., True), ('高度', 100., True)]:
        box = QtWidgets.QDoubleSpinBox(); box.setDecimals(2); box.setRange(.01 if positive else -100000., 100000.)
        box.setSuffix(' mm'); box.setValue(initial); form.addRow(title, box); inputs.append(box)
    layout.addLayout(form)
    origin_group = QtWidgets.QGroupBox('已测量安装孔基准（相对地面/J1轴心）')
    origin_form = QtWidgets.QFormLayout(origin_group)
    origin_name = QtWidgets.QLineEdit(); origin_name.setPlaceholderText('输入实测安装孔名称，例如：安装孔A')
    origin_form.addRow('基准名称', origin_name)
    origin_inputs = []
    for axis in ('X', 'Y'):
        box=QtWidgets.QDoubleSpinBox(); box.setRange(-100000.,100000.)
        box.setDecimals(2); box.setSuffix(' mm'); origin_form.addRow('孔中心'+axis,box); origin_inputs.append(box)
    origin_buttons=QtWidgets.QHBoxLayout()
    origin_save=QtWidgets.QPushButton('新增 / 修改孔基准')
    origin_delete=QtWidgets.QPushButton('删除当前孔基准（保留圆柱世界位置）')
    origin_buttons.addWidget(origin_save); origin_buttons.addWidget(origin_delete)
    origin_form.addRow(origin_buttons)
    origin_form.addRow(QtWidgets.QLabel('请先实测孔位；不预设CAD孔位。修改孔基准会同步移动其所有圆柱并作废旧规划。'))
    layout.addWidget(origin_group)
    buttons = QtWidgets.QHBoxLayout()
    add, update, delete, retry = [QtWidgets.QPushButton(t) for t in ('添加圆柱', '修改所选', '删除所选', '重试场景同步')]
    for button in (add, update, delete, retry): buttons.addWidget(button)
    layout.addLayout(buttons)
    status = QtWidgets.QLabel(); status.setWordWrap(True); layout.addWidget(status)
    layout.addWidget(QtWidgets.QLabel('障碍修改会作废全部旧规划；运动中会先受控停车并确认Hold，再更新场景。'))
    ##
    # @par 内部回调 create_obstacle_tab.refresh
    # 从期望场景重绘列表并按事务状态启停编辑按钮。
    # 场景尚未确认时列表仍可显示 desired；pending 为 future 或等待 Hold，期间禁用变更。超时保留 future 时重试按钮也保持禁用。
    # 返回：None；执行上述状态更新、界面或日志操作。
    def refresh():
        rows = scene.desired
        frame = combo.currentText()
        combo.blockSignals(True); combo.clear(); combo.addItems(list(scene.frames))
        combo.setCurrentText(frame if frame in scene.frames else 'base_link'); combo.blockSignals(False)
        selected = table.currentRow()
        table.setRowCount(len(rows))
        for i, row in enumerate(rows):
            for j, key in enumerate(('id','frame','x_mm','y_mm','diameter_mm','height_mm')):
                table.setItem(i,j,QtWidgets.QTableWidgetItem(str(row[key])))
        if 0 <= selected < len(rows): table.selectRow(selected)
        status.setText(scene.status+'\n保存位置：'+str(scene.path))
        pending = scene.future is not None or scene.wait_hold
        for button in (add, update, delete): button.setEnabled(not pending)
        origin_save.setEnabled(not pending)
        origin_delete.setEnabled(not pending and combo.currentText() != 'base_link')
        retry.setEnabled(bool(scene.error) and not pending)
    ##
    # @par 内部回调 create_obstacle_tab.selected
    # 把所选圆柱的基准和毫米值填回编辑表单。
    # 只改变编辑器，不发起场景更新；用户须点击修改后才进入事务。
    # 返回：None；执行上述状态更新、界面或日志操作。
    def selected():
        index = table.currentRow()
        if 0 <= index < len(scene.desired):
            row = scene.desired[index]; combo.setCurrentText(row['frame'])
            for box,key in zip(inputs,('x_mm','y_mm','diameter_mm','height_mm')): box.setValue(row[key])
    table.itemSelectionChanged.connect(selected)
    ##
    # @par 内部回调 create_obstacle_tab.change
    # 按按钮动作生成新期望圆柱列表。
    # 新增 ID 取 UUID 前 12 位；修改保留原 ID，删除只移除所选行。校验错误显示在状态标签，既有确认场景不被此回调直接改写。
    # 参数 action：add/update/delete/retry；retry 保持当前列表重新请求同步。
    # 返回：None；执行上述状态更新、界面或日志操作。
    def change(action):
        rows = [dict(row) for row in scene.desired]; index = table.currentRow()
        row = dict(zip(('x_mm','y_mm','diameter_mm','height_mm'), (b.value() for b in inputs)))
        row['frame'] = combo.currentText()
        if action == 'add': row['id'] = uuid.uuid4().hex[:12]; rows.append(row)
        elif action in ('update','delete'):
            if not 0 <= index < len(rows): return
            if action == 'delete': rows.pop(index)
            else: row['id'] = rows[index]['id']; rows[index] = row
        try: scene.request_change(rows)
        except ValueError as error: status.setText(str(error))
    ##
    # @par 内部回调 create_obstacle_tab.origin_selected
    # 显示当前地面或孔基准的名称及偏移。
    # base_link 名称输入留空、删除入口禁用；事务在途时也不允许删除孔基准。
    # 返回：None；执行上述状态更新、界面或日志操作。
    def origin_selected():
        name=combo.currentText()
        origin_name.setText('' if name == 'base_link' else name)
        for box,value in zip(origin_inputs,scene.frames.get(name,(0.,0.))): box.setValue(value)
        origin_delete.setEnabled(name != 'base_link' and scene.future is None and not scene.wait_hold)
    ##
    # @par 内部回调 create_obstacle_tab.change_origin
    # 将孔基准保存或删除操作交给场景事务。
    # 校验错误只显示原因；删除后选回 base_link，保存后选择相应孔名。
    # 参数 delete：True 删除下拉选中基准；False 按名称输入与 XY 编辑值新增/修改。
    # 返回：None；执行上述状态更新、界面或日志操作。
    def change_origin(delete=False):
        name = combo.currentText() if delete else origin_name.text().strip()
        try: scene.request_origin_change(name,None if delete else tuple(box.value() for box in origin_inputs))
        except ValueError as error: status.setText(str(error)); return
        combo.setCurrentText('base_link' if delete else name)
    combo.currentTextChanged.connect(origin_selected)
    origin_save.clicked.connect(lambda: change_origin(False))
    origin_delete.clicked.connect(lambda: change_origin(True))
    add.clicked.connect(lambda: change('add')); update.clicked.connect(lambda: change('update'))
    delete.clicked.connect(lambda: change('delete')); retry.clicked.connect(lambda: change('retry'))
    scene.on_change = refresh; refresh(); origin_selected()
    return widget, scene
