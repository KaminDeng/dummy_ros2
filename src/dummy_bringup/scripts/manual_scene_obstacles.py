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

PREFIX = 'manual_cylinder_'


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


def cylinder_geometry(row, frames):
    """基准仅平移地面XY原点；统一在base_link中保持竖直，底面Z=0。"""
    origin = frames[row['frame']]
    return ((origin[0] + row['x_mm']) / 1000., (origin[1] + row['y_mm']) / 1000.,
            row['height_mm'] / 2000., row['height_mm'] / 1000., row['diameter_mm'] / 2000.)


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


def save_obstacles(path, rows, known_ids, frames=None):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as stream:
        json.dump({'version': 1, 'obstacles': rows, 'known_ids': sorted(known_ids),
                   'reference_origins_mm': frames or {'base_link': [0., 0.]}}, stream, ensure_ascii=False, indent=2)
        stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)


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


class ObstacleScene:
    """可测试事务状态机。更新失败保持封锁，重试必须重新获服务成功回执。"""
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

    def fail(self, text):
        self.error = text; self.status = text; self.blocked = True; self.scene_ready = False
        self.on_change()

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

    def native_event(self, event, parts):
        if self.wait_hold and event == 'MOVE_CANCELED_HOLD':
            try:
                identity = (int(parts['boot'], 16), int(parts['reference']), int(parts['session']))
                if identity == self.stop_identity and int(parts['state_seq']) >= self.stop_state_seq:
                    self.hold_marker = (identity, int(parts['state_seq']), int(parts['motion']))
            except (KeyError, ValueError):
                pass

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

    def tick(self):
        if self.error: return
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
    def selected():
        index = table.currentRow()
        if 0 <= index < len(scene.desired):
            row = scene.desired[index]; combo.setCurrentText(row['frame'])
            for box,key in zip(inputs,('x_mm','y_mm','diameter_mm','height_mm')): box.setValue(row[key])
    table.itemSelectionChanged.connect(selected)
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
    def origin_selected():
        name=combo.currentText()
        origin_name.setText('' if name == 'base_link' else name)
        for box,value in zip(origin_inputs,scene.frames.get(name,(0.,0.))): box.setValue(value)
        origin_delete.setEnabled(name != 'base_link' and scene.future is None and not scene.wait_hold)
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
