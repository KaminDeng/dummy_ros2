#!/usr/bin/python3
##
# @file manual_motor_parameters.py
# @brief 通过现有 Native 所有者异步串行读写 CAN 电机参数。
# 此页不单独打开 USB；命令经主面板 command() 交给已有 Native worker，结果由主面板 stdout 分流至 handle_line()。仅真机新鲜只读/失能会话允许读写，写后必须读回核对。
"""通过现有 Native USB owner 的异步 CAN 电机参数页。"""
import time
from collections import deque
from PyQt5 import QtCore, QtWidgets

# CAN 参数值使用电机原生整数单位，避免浮点转换丢失精度。
##
# 每项依次为协议参数编号、中文名、原生单位、是否只读；DCE 增益保持固件整数语义，不在 UI 猜测缩放比例。
PARAMETERS = (
    (1, 'CAN 节点 ID', '', True),
    (2, '电流限制', 'mA', False),
    (3, '速度限制', 'counts/s', False),
    (4, '加速度', 'counts/s²', False),
    (5, '上电使能', '0 / 1', False),
    (6, 'DCE Kp', '', False),
    (7, 'DCE Kv', '', False),
    (8, 'DCE Ki', '', False),
    (9, 'DCE Kd', '', False),
    (10, '堵转保护', '0 / 1', False),
    (11, '编码器零偏', 'counts', True),
    (12, '温度保护', '0 / 1', False),
)
##
# 状态码只解释已知参数协议错误；未知状态保留原编号，不能据中文提示替代真实应答核对。
ERRORS = {1: '参数无效或电机拒绝', 2: '主控忙，请稍后重试', 3: '电机固件不支持此参数协议',
          5: 'CAN 参数应答超时，请检查对应电机固件与连接', 6: '请先停止并失能后读写参数',
          7: '设备身份已变化，请重新读取'}


##
# @class MotorParameterTab
# @brief 维护电机参数表、串行请求队列和身份绑定。
# pending 至多一个在途事务，queue 为后续动作；身份为 (worker_generation,boot_epoch)，连接改变时清空旧显示和队列。运行值与 EEPROM 值分列，不能把本地待写值当作设备实际值。
class MotorParameterTab(QtWidgets.QWidget):
    ##
    # @brief 创建六电机选择器、参数表和异步事务初态。
    # 所有参数保持整数原生单位；电机一圈 51200 counts。参数 1 与 11 只读；其余输入范围按参数类别限定，真正设备是否接受仍以应答和读回为准。
    # @param panel 主面板对象，提供新鲜状态、Native worker 和 command()。
    # @return 无；通过实例状态或界面产生作用。
    def __init__(self, panel):
        super().__init__(panel)
        self.panel = panel
        ##
        # pending 是一条在途请求字典；queue 中每项为 (parameter, action, value, expected)，expected=None 表示普通读取/写应答阶段。
        self.sequence = time.monotonic_ns() & 0xffffffff
        self.pending = None
        self.queue = deque()
        self.connection = None
        self.online = False
        layout = QtWidgets.QVBoxLayout(self)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('电机'))
        self.motor = QtWidgets.QComboBox()
        self.motor.addItems([f'J{i}' for i in range(1, 7)])
        row.addWidget(self.motor)
        self.read = QtWidgets.QPushButton('读取全部参数')
        self.apply = QtWidgets.QPushButton('临时应用')
        self.save = QtWidgets.QPushButton('保存到 EEPROM')
        self.save.setProperty('role', 'primary')
        for button in (self.read, self.apply, self.save):
            row.addWidget(button)
        row.addStretch()
        layout.addLayout(row)
        note = QtWidgets.QLabel('先点击公共区域“停止并失能”，等待重新只读连接后读写。临时应用立即生效；EEPROM 保存后掉电保留。\n'
                               '选择表格中的参数行再应用/保存，写入后自动读回核对。电机一圈 = 51200 counts。')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.table = QtWidgets.QTableWidget(len(PARAMETERS), 5)
        self.table.setHorizontalHeaderLabels(['参数', '单位', '当前运行值', 'EEPROM 值', '待写入值'])
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setDefaultSectionSize(40)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.inputs = {}
        for index, (parameter, name, unit, readonly) in enumerate(PARAMETERS):
            for col, value in enumerate((name + ('（只读）' if readonly else ''), unit, '—', '—')):
                item = QtWidgets.QTableWidgetItem(value)
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                self.table.setItem(index, col, item)
            if readonly:
                item = QtWidgets.QTableWidgetItem('—')
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                self.table.setItem(index, 4, item)
            else:
                box = QtWidgets.QSpinBox()
                bounds = {2: (1, 3300), 3: (1, 51200 * 200), 4: (1, 51200 * 20000)}
                lower, upper = bounds.get(parameter, (0, 1 if parameter in (5, 10, 12) else 10000))
                box.setRange(lower, upper)
                box.setKeyboardTracking(False)
                # 单元格内的编辑控件不会自动选中所在行，必须同步焦点与保存目标。
                for editor in (box, box.lineEdit()):
                    editor.setProperty('parameter_row', index)
                    editor.installEventFilter(self)
                self.inputs[parameter] = box
                self.table.setCellWidget(index, 4, box)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.selectRow(1)
        layout.addWidget(self.table)
        self.status = QtWidgets.QLabel('等待设备只读连接。')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.motor.currentIndexChanged.connect(self.selection_changed)
        self.read.clicked.connect(self.read_all)
        self.apply.clicked.connect(lambda: self.write_selected(1))
        self.save.clicked.connect(lambda: self.write_selected(2))
        self.table.itemSelectionChanged.connect(self.update_buttons)
        self.update_buttons()

    ##
    # @brief 编辑器获焦时同步选中对应参数行。
    # @param editor 带 parameter_row 属性的 SpinBox 或其内部行编辑器。
    # @param event Qt 事件，关注 FocusIn。
    # @return bool：父类事件过滤器返回值；不吞掉正常编辑事件。
    def eventFilter(self, editor, event):
        if event.type() == QtCore.QEvent.FocusIn:
            row = editor.property('parameter_row')
            if row is not None:
                self.table.setCurrentCell(row, 4)
        return super().eventFilter(editor, event)

    ##
    # @brief 取得当前参数连接身份。
    # @return tuple：Native worker 世代号与当前反馈 boot_epoch；缺状态时 boot 为 None。
    def identity(self):
        state = getattr(self.panel, 'last_state', None)
        return (getattr(self.panel, 'worker_generation', 0), state.get('boot_epoch') if state else None)

    ##
    # @brief 检查此刻是否允许参数事务。
    # 每次发送和界面刷新都重新检查；模拟模式不会被当作真实参数后端。此条件不自动触发停止/失能。
    # @return bool：真机、反馈新鲜、core_state 为 1/2、session 为 0、无忙/关闭且 worker 正在运行。
    def allowed(self):
        state = getattr(self.panel, 'last_state', None)
        return bool(getattr(self.panel, 'backend', 'real') == 'real' and state and self.panel.fresh() and state.get('core_state') in (1, 2) and
                    state.get('session_id') == 0 and not self.panel.busy and not self.panel.closing and
                    self.panel.worker.state() == QtCore.QProcess.Running)

    ##
    # @brief 清空运行值及 EEPROM 已读显示。
    # 保留待写编辑值；“—”表示尚未取得当前连接的有效读回，不能表示参数为零。
    # @return 无；通过实例状态或界面产生作用。
    def clear_values(self):
        for row in range(len(PARAMETERS)):
            for column in (2, 3):
                self.table.item(row, column).setText('—')

    ##
    # @brief 切换电机时清除排队动作和旧参数显示。
    # 事务期间选择器被禁用；正常用户切换只发生在无在途事务时。新电机必须显式重新读取。
    # @return 无；通过实例状态或界面产生作用。
    def selection_changed(self):
        self.queue.clear()
        self.clear_values()
        self.status.setText('已切换电机，请重新读取。')
        self.update_buttons()

    ##
    # @brief 按事务状态和权限更新编辑器与操作按钮。
    # 在途或队列未空时禁止换电机和编辑，避免批量读取覆盖用户新输入。写入按钮还要求当前行可写。
    # @return 无；通过实例状态或界面产生作用。
    def update_buttons(self):
        in_flight = self.pending is not None or bool(self.queue)
        # 批量读回会刷新输入值，完成前暂停编辑及换电机，防止覆盖用户输入。
        self.motor.setEnabled(not in_flight)
        for box in self.inputs.values():
            box.setEnabled(not in_flight)
        ready = self.allowed() and not in_flight
        self.read.setEnabled(ready)
        row = self.table.currentRow()
        writable = 0 <= row < len(PARAMETERS) and not PARAMETERS[row][3]
        self.apply.setEnabled(ready and writable)
        self.save.setEnabled(ready and writable)

    ##
    # @brief 串行排队读取所有参数的运行值和 EEPROM 值。
    # 每个参数先 action=0 读运行值，再 action=3 读 EEPROM；不满足 allowed 或已有事务时直接返回。
    # @return 无；通过实例状态或界面产生作用。
    def read_all(self):
        if not self.allowed() or self.pending or self.queue:
            return
        self.clear_values()
        self.queue.extend((parameter, action, 0, None) for parameter, *_ in PARAMETERS for action in (0, 3))
        self.send_next()

    ##
    # @brief 提交选中参数的临时写入或 EEPROM 保存及校验读回。
    # 先 interpretText 提交编辑内容，再排队写入及 action=0 读回；保存时额外 action=3 读 EEPROM。expected 保存待核对整数，写应答成功不直接视为读回一致。
    # @param action 1 为临时应用，2 为 EEPROM 保存；由两个按钮固定传入。
    # @return 无；通过实例状态或界面产生作用。
    def write_selected(self, action):
        row = self.table.currentRow()
        if not self.allowed() or self.pending or self.queue or not 0 <= row < len(PARAMETERS):
            return
        parameter = PARAMETERS[row][0]
        if parameter not in self.inputs:
            return
        self.inputs[parameter].interpretText()
        value = self.inputs[parameter].value()
        self.table.item(row, 2).setText('—')
        self.queue.append((parameter, action, value, None))
        self.queue.append((parameter, 0, 0, value))
        if action == 2:
            self.table.item(row, 3).setText('—')
            self.queue.append((parameter, 3, 0, value))
        self.send_next()

    ##
    # @brief 在无在途请求且权限允许时发送队首事务。
    # sequence 在非零 uint32 范围递增，node 为 1..6；pending 同时保存连接身份和 monotonic+2.7 秒截止时刻。写值按 32 位无符号位型传输，结果再还原有符号整数。
    # @return 无；通过实例状态或界面产生作用。
    def send_next(self):
        if self.pending or not self.queue or not self.allowed():
            self.update_buttons()
            return
        parameter, action, value, expected = self.queue.popleft()
        self.sequence = self.sequence % 0xffffffff + 1
        self.pending = dict(sequence=self.sequence, node=self.motor.currentIndex() + 1,
                            parameter=parameter, action=action, expected=expected,
                            identity=self.identity(), deadline=time.monotonic() + 2.7)
        request = self.pending
        if not self.panel.command(f'MOTOR_PARAM {request["sequence"]} {request["node"]} {parameter} {action} {value & 0xffffffff}'):
            self.fail('通信进程未连接。')
            return
        self.status.setText(f'J{request["node"]}：正在' + ('读取' if action in (0, 3) else '写入') + PARAMETERS[parameter - 1][1] + '…')
        self.update_buttons()

    ##
    # @brief 终止当前参数批次并显示失败原因。
    # 清除 pending 和 queue，不自动重试、不假定未确认的写入没有生效；重新读取后再决定后续动作。
    # @param message 操作员可读的原因文本。
    # @return 无；通过实例状态或界面产生作用。
    def fail(self, message):
        self.pending = None
        self.queue.clear()
        self.status.setText(message)
        self.update_buttons()

    ##
    # @brief 匹配并消费一条 Native 参数结果行。
    # 解析后严格匹配 sequence/node/parameter 和连接身份；旧应答或错误格式不会填入表格。状态为零后把 uint32 还原为有符号整数，读回与 expected 不符则停止批次；通过后才调度下一条。
    # @param line 不含换行的 Native 标准输出文本。
    # @return bool：是 MOTOR_PARAM_RESULT 行时返回 True，其他行 False 供主面板继续处理。
    def handle_line(self, line):
        if not line.startswith('MOTOR_PARAM_RESULT '):
            return False
        try:
            sequence, node, parameter, status, raw = map(int, line.split()[1:])
        except (ValueError, TypeError):
            return True
        request = self.pending
        if not request or (sequence, node, parameter) != (request['sequence'], request['node'], request['parameter']):
            return True
        ##
        # 即使请求序号碰巧重用，重连世代或 boot 改变也必须丢弃应答，防止旧电机值污染新连接。
        if request['identity'] != self.identity() or node != self.motor.currentIndex() + 1:
            self.pending = None
            self.queue.clear()
            self.update_buttons()
            return True
        if status != 0:
            self.fail(f'J{node}：' + ERRORS.get(status, f'参数操作失败（{status}）'))
            return True
        value = raw if raw < 0x80000000 else raw - 0x100000000
        action = request['action']
        if action in (0, 3):
            self.table.item(parameter - 1, 2 if action == 0 else 3).setText(str(value))
            if request['expected'] is not None and value != request['expected']:
                self.fail(f'J{node}：写后读回不一致，期望 {request["expected"]}，实际 {value}。')
                return True
            if parameter in self.inputs and request['expected'] is None and action == 0:
                self.inputs[parameter].setValue(value)
        self.pending = None
        self.status.setText(f'J{node}：参数读回核对成功。' if request['expected'] is not None else f'J{node}：参数读取完成。')
        self.send_next()
        return True

    ##
    # @brief 由主面板定时器驱动身份失效、超时与界面刷新。
    # 连接身份变化或由可用转不可用时清空请求和显示；超过 2.7 秒本地截止时刻即失败。超时只说明没有有效读回，不证明电机未执行写入。
    # @return 无；通过实例状态或界面产生作用。
    def tick(self):
        identity = self.identity()
        available = self.allowed()
        if identity != self.connection or (self.online and not available):
            self.connection = identity
            self.pending = None
            self.queue.clear()
            self.clear_values()
            self.status.setText('连接或控制状态已变化，请停止并失能后重新读取。')
        self.online = available
        if self.pending and time.monotonic() >= self.pending['deadline']:
            self.fail('参数请求超时；未取得有效读回值。')
        self.update_buttons()
