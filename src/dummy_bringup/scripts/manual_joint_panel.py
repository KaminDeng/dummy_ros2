#!/usr/bin/python3
##
# @file manual_joint_panel.py
# @brief 按后端保持 Native 或 ROS C++ 单一通信所有者的六轴交互面板。
# Qt驱动界面与rclpy回调；Native实测由面板发布，ROS实测由控制链发布，目标虚影独立。关节直控与末端避碰规划分开，启动/重连只读，控制授权与最终完成均以设备或C++权威回执为准。
"""六轴交互面板：单通信所有者、实测与目标分离、显式确认后执行。"""
import argparse
import json
import hashlib
import math
import struct
import signal
import sys
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import xacro


##
# @brief 检查六轴角度输入是否有限且位于给定边界。
# 假定配置提供完整六轴 limits；该轻量函数不独立检查 limits 长度，不进行碰撞或设备状态验证。
# @param degrees 要求长度为六的目标角度序列，单位 °。
# @param limits 调用方提供的六对 [下限,上限]，单位 °。
# @return bool：输入长度为六，zip 配对各轴均有限且在闭区间内。
def valid_goal(degrees, limits):
    return len(degrees) == 6 and all(
        math.isfinite(v) and lo <= v <= hi for v, (lo, hi) in zip(degrees, limits))


##
# @class JointPanel
# @brief 管理设备连接、反馈授权、人工标定和各控制页。
# calibration 仅表示 (boot,reference) 零点身份；authorized 表示当前 session 的控制资格，两者不可互换。last_state 为实际解码帧，inputs 为度制目标；目标虚影与 /joint_states 的测量发布严格分开。
class JointPanel(QtWidgets.QWidget):
    ##
    # @brief 建立发布器、状态账本、页签和只读通信 worker。
    # 展开 xacro 后分别建立测量/预览描述发布器；初始无会话授权。可选 MoveIt 页和圆柱场景，独立管理进程仅用于明确清故障。30 ms 定时器负责反馈及门控，不在初始化时运动。
    # @param config 经部署校验的面板配置，含后端、描述模型及关节范围。
    # @param report 本次全新报告目录 Path；已存在会报错，避免覆盖证据。
    # @param decoder 部署随附的状态解码函数；模拟后端可为 None。
    # @param node 外部持有的 rclpy 节点，由本窗口定时 spin。
    # @return None；执行上述状态更新、界面或日志操作。
    def __init__(self, config, report, decoder, node):
        super().__init__()
        self.cfg, self.report, self.decode, self.node = config, report, decoder, node
        self.backend = config.get("backend", "real")
        self.pending_backend = None
        ##
        # /joint_states 仅来自新鲜测量/显示参考的 actual_q；/target_joint_states 独立表示未执行输入，两个描述 topic 也分开管理。
        self.publisher = None if self.backend == 'ros' else node.create_publisher(JointState, '/joint_states', 10)
        self.target_publisher = node.create_publisher(JointState, '/target_joint_states', 10)
        self.description_publisher = node.create_publisher(String, '/measured_robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.description_xml = xacro.process_file(config['description_xacro']).toxml()
        self.preview_description_publisher = node.create_publisher(String, '/target_preview/colored_robot_description', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.preview_failed = None
        self.description_visible = None
        self.report.mkdir(parents=True, exist_ok=False)
        ##
        # raw 文件起始 24 字节为 transport 头；offset 是下一完整记录读取偏移，last_state_ns 记录帧实际到达时间而非文件读取时间。
        self.raw = self.report / 'transport.raw'
        self.offset = 24
        self.last_state = None
        self.last_state_ns = 0
        ##
        # calibration=(boot,reference)；control_session 是授权事件 session；pending_authorization 保存事件完整身份及各 epoch 水位，不能凭 stdout 文字直接使能 MOVE。
        self.calibration = None
        self.authorized = False
        self.control_session = None
        self.pending_authorization = None
        self.state_boot = None
        self.state_sequence = 0
        ##
        # pending_home 表示用户已明确确认新姿态但仍需旧 owner 释放；pending_home_generation 防止旧 worker 的 WAIT 事件提前触发捕获。
        self.retired_state_boots = set()
        self.restart_after_release = False
        self.readonly_handoff = None
        self.release_confirmed = False
        self.pending_home = False
        self.pending_home_generation = None
        self.closing = False
        ##
        # worker_generation 追踪重连世代；manage_generation 只区分清故障归档。connection_action 串行管理后端切换/重连/清故障。
        self.worker_generation = 0
        self.connection_action = None
        self.manage_generation = 0
        self.zero_busy = False
        self.busy = False
        self.wait_home = False
        self.readonly_fault = False
        self.readonly_fault_identity = None
        self.readonly_wait_generation = None
        self.readonly_wait_identity = None
        self.stdout_buffer = ''
        ##
        # anchor/previous_raw 只服务未标定显示；检测原始电机圈数突变即丢弃锚点，不写设备参考。limits 为度制工作范围。
        self.anchor = None
        self.display_initialized = False
        self.previous_raw = None
        self.calibration_event_ns = 0
        self.limits = [j['limits_deg'] for j in config['joints']]
        from manual_panel_ui import build_panel_ui
        build_panel_ui(self, config)
        if config.get('moveit_enabled', False):
            from manual_scene_obstacles import create_obstacle_tab
            obstacle_widget, self.obstacle_tab = create_obstacle_tab(self)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(obstacle_widget)
            self.tabs.addTab(scroll, '圆柱障碍物')
        self.create_worker()
        self.manage_worker = QtCore.QProcess(self)
        self.manage_worker.readyReadStandardOutput.connect(self.manage_stdout_ready)
        self.manage_worker.readyReadStandardError.connect(self.manage_stderr_ready)
        self.manage_worker.finished.connect(self.manage_finished)
        self.manage_worker.errorOccurred.connect(self.manage_error)
        self.start_worker()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)
        self.tick()

    ##
    # @brief 显示默认拒绝的执行确认，或遵循本窗口跳过确认选项。
    # 跳过确认只省略对话框，调用者仍须执行反馈、身份、权限与范围检查；对话框期间事件循环继续，确认后需要重新检查条件。
    # @param title 确认对话框标题。
    # @param text 待执行动作、单位和人工确认条件说明。
    # @return QMessageBox.StandardButton：Yes 或用户选择值。
    def confirm_execution(self, title, text):
        checkbox = getattr(self, 'skip_confirmations', None)
        if checkbox is not None and checkbox.isChecked():
            self.message('已跳过确认：' + title)
            return QtWidgets.QMessageBox.Yes
        return QtWidgets.QMessageBox.question(self, title, text,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)

    ##
    # @brief 按后端创建通信对象并连接统一事件接口。
    # real 使用 QProcess，simulator 使用本地 SimulatorWorker；本方法仅创建不启动。标准输出、错误和 finished 都进入同一面板处理流程，模拟来源在界面明确标识。
    # @return None；执行上述状态更新、界面或日志操作。
    def create_worker(self):
        self.zero.setText('双限位标定 / 恢复控制' if self.backend == 'ros' else
                          ('双限位标定 → Rest' if self.manual_limits_mode() else 'Home → Rest 标定'))
        if self.backend == 'ros':
            from manual_panel_ros_backend import RosBackend, RosTransport
            from manual_panel_runtime import PanelRuntime
            self.worker = QtCore.QProcess(self)  # 仅为旧 UI 状态查询保留，不启动。
            self.ros_status = None
            self.ros_runtime = PanelRuntime()
            self.ros_backend = RosBackend(RosTransport(self.node, self.ros_status_received, self.ros_state_received), self.ros_event)
            self.ros_last_poll = 0
            self.ros_auto_pending = bool(self.cfg.pop('auto_calibrate_and_test', False))
            self.backend_notice.setText('真机-USB/ROS · 启动只读')
            return
        if self.backend == 'simulator':
            from manual_panel_simulator import SimulatorWorker
            self.worker = SimulatorWorker(self.cfg, self)
            self.backend_notice.setText('仿真-本地 · 不连接硬件')
            self.backend_notice.setStyleSheet('color:#b45309;font-weight:bold')
        else:
            self.worker = QtCore.QProcess(self)
            self.backend_notice.setText('真机-USB/Native · 启动仅只读')
            self.backend_notice.setStyleSheet('color:#b42318;font-weight:bold')
        self.worker.readyReadStandardOutput.connect(self.stdout_ready)
        self.worker.readyReadStandardError.connect(self.stderr_ready)
        self.worker.finished.connect(self.finished)

    ##
    # @brief 校验切换条件并请求串行移交到所选后端。
    # 立即把下拉框复原到当前后端；有运动、管理、参数事务或连接动作时拒绝。切到真机先重新加载/校验部署配置和解码器；设置 pending_backend 后通过 RELEASE/退出确认移交，禁止直接并行启动。
    # @param _index Qt 下拉框索引参数；实际选择从 currentData 读取。
    # @return None；执行上述状态更新、界面或日志操作。
    def request_backend_change(self, _index=None):
        selected = self.backend_choice.currentData()
        self.backend_choice.blockSignals(True)
        self.backend_choice.setCurrentIndex(self.backend_choice.findData(self.backend))
        self.backend_choice.blockSignals(False)
        if selected == self.backend:
            return
        if self.backend == 'ros':
            next_config, next_decode = None, None
            if selected == 'real':
                from manual_panel_config import load_decoder
                from manual_panel_runtime import load_runtime_panel_config
                try:
                    next_config = load_runtime_panel_config(self.cfg['_config_path'], 'real')
                    next_decode = load_decoder(next_config)
                except (OSError, ValueError, KeyError) as error:
                    self.message(str(error))
                    return
            self.message('先请求停止释放，等待 ROS 控制链确认后切换。')
            if not self.ros_backend.released:
                self.release('RELEASE')
                return
            if not self.ros_runtime.stop(True):
                return
            self.ros_backend.close()
            if next_config is not None:
                self.cfg, self.decode = next_config, next_decode
            self.pending_backend = selected
            self.apply_backend_change()
            return
        if selected == 'ros':
            from manual_panel_runtime import load_runtime_panel_config as load_panel_config
            try:
                self.cfg = load_panel_config(self.cfg['_config_path'], 'ros')
                self.decode = None
            except (ValueError, OSError, KeyError) as error:
                self.message(str(error))
                return
        parameter_busy = hasattr(self, 'motor_parameter_tab') and (self.motor_parameter_tab.pending or self.motor_parameter_tab.queue)
        if self.closing or self.busy or self.helper_active() or self.connection_action is not None or parameter_busy:
            self.message('请先停止当前操作，再切换连接目标。')
            return
        if selected == 'real' and self.cfg.get('_config_path'):
            from manual_panel_config import resolve_config_path, load_decoder
            from manual_panel_runtime import load_runtime_panel_config as load_panel_config
            try:
                path = Path(self.cfg['_config_path'])
                real_config = load_panel_config(path, 'real')
                decoder = load_decoder(real_config)
            except (OSError, ValueError, KeyError) as error:
                self.message(str(error))
                return
            real_config['moveit_enabled'] = self.cfg.get('moveit_enabled', False)
            self.cfg = real_config
            self.decode = decoder
        self.pending_backend = selected
        self.queue_connection_action('backend')

    ##
    # @brief 在旧连接退出后切换对象并清除跨后端状态。
    # 清实际反馈、零点和授权缓存、旧 boot 序列、显示锚点及规划；隐藏实测模型，重新创建并只读启动 worker。模拟与真机不继承彼此的控制身份或位置。
    # @return None；执行上述状态更新、界面或日志操作。
    def apply_backend_change(self):
        self.worker.deleteLater()
        self.backend, self.pending_backend = self.pending_backend, None
        # 实测发布权跟随通信所有者；ROS由JSB发布，Native/仿真由面板发布。
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
        self.publisher = (None if self.backend == 'ros' else
                          self.node.create_publisher(JointState, '/joint_states', 10))
        # 两个目标的参考、反馈、规划和控制权限不能互相继承。
        self.last_state = self.calibration = self.control_session = None
        self.last_state_ns = 0
        self.state_boot = None
        self.state_sequence = 0
        self.retired_state_boots.clear()
        self.authorized = self.wait_home = self.busy = self.zero_busy = False
        self.pending_authorization = None
        self.anchor = self.previous_raw = None
        self.display_initialized = False
        if hasattr(self, 'actual_q'):
            del self.actual_q
        if hasattr(self, 'moveit_tab'):
            self.moveit_tab.cancel_execution('连接目标改变，原规划作废。')
            self.moveit_tab.invalidate()
        self.show_actual_description(False)
        for label in self.feedback_labels:
            label.setText('—')
        self.backend_choice.setCurrentIndex(self.backend_choice.findData(self.backend))
        self.create_worker()
        self.start_worker()

    ##
    # @brief 按当前页签调整关节执行入口并按需读取实际末端。
    # 仅关节页显示“执行六轴目标”和“实际角度→目标”，避免把规划末点当作整条路径执行。进入空闲且无路径的 MoveIt 页时只请求 FK，不执行运动。
    # @param index 当前页签零基索引。
    # @return None；执行上述状态更新、界面或日志操作。
    def planning_tab_changed(self, index):
        # 规划页只能走整条路径按钮，避免误把关节页的末点按钮当作规划执行。
        self.move.setVisible(index == 0)
        self.copy.setVisible(index == 0)
        if hasattr(self, 'moveit_tab') and self.tabs.widget(index).widget() is self.moveit_tab and not self.moveit_tab.path and not self.moveit_tab.pending and not self.moveit_tab.executing:
            self.moveit_tab.capture_actual()

    ##
    # @brief 查询清故障管理进程是否仍占用生命周期。
    # Starting 也视为活动，防止第二个通信进程在启动交叠窗口进入。
    # @return bool：manage_worker 已创建且状态不是 NotRunning。
    def helper_active(self):
        helper = getattr(self, 'manage_worker', None)
        return helper is not None and helper.state() != QtCore.QProcess.NotRunning

    ##
    # @brief 判断是否允许明确启动清故障事务。
    # 故障存在不会自动触发清除；操作者仍需单独请求并确认已处理根因。
    # @return bool：真机新鲜故障态 9、session=0、未授权且没有忙碌/管理/连接动作。
    def fault_clear_available(self):
        return (getattr(self, 'backend', 'real') == 'real' and self.fresh() and self.last_state['core_state'] == 9 and
                self.last_state['session_id'] == 0 and not self.authorized and
                not self.busy and not self.helper_active() and getattr(self, 'connection_action', None) is None)

    ##
    # @brief 在空闲时排队只读重连。
    # 关闭、忙碌、管理进程或已有连接动作时直接返回；重连仍先走旧连接释放屏障，不自动恢复使能。
    # @return None；执行上述状态更新、界面或日志操作。
    def request_reconnect(self):
        if self.backend == 'ros':
            if (self.ros_runtime.process is None and
                    getattr(self.ros_backend.transport, '_startup_owner_conflict', '')):
                # 被拒绝的启动从未拥有设备链；明确重试发现，
                # 不向无关的既有拥有者发送D10。
                self.ros_backend.transport.close()
                self.pending_backend = 'ros'
                self.apply_backend_change()
                return
            if not self.ros_backend.released:
                self.release('RELEASE')
                self.message('先等待 ROS 控制链确认失能释放，再次点击重连；重连不自动恢复运动。')
                return
            if not self.ros_runtime.stop(True) or not self.ros_backend.close():
                return
            self.pending_backend = 'ros'
            self.apply_backend_change()
            return
        if self.closing or self.busy or self.helper_active() or getattr(self, 'connection_action', None) is not None:
            return
        self.queue_connection_action('reconnect')

    ##
    # @brief 确认已处理故障后排队串行清故障操作。
    # 确认前后均重新检查 fault_clear_available；只清故障与重新只读连接，不采零、不使能、不运动。恢复控制仍需另行操作。
    # @return None；执行上述状态更新、界面或日志操作。
    def request_clear_fault(self):
        if self.closing or not self.fault_clear_available():
            return
        answer = self.confirm_execution('清除已处理故障', '请确认故障原因已处理、机械臂有支撑，并已停止手动调整。\n'
            '此操作仅清故障并重新只读连接，不采零、不使能、不运动。\n'
            '后续仍需另行点击“使能 / 恢复已有零点”；近限位时该步骤会低速退工作Rest。')
        if answer == QtWidgets.QMessageBox.Yes and not self.closing and self.fault_clear_available():
            self.queue_connection_action('clear_fault')

    ##
    # @brief 撤销授权并等待原 worker 释放后执行连接动作。
    # 保存有限的故障只读移交证据，清旧 pending 授权/标定意图并置 busy。worker 已退出才直接调度，否则发 RELEASE；写入失败撤销排队动作，不启动第二个进程。
    # @param action backend、clear_fault 或 reconnect，由对应 UI 入口传入。
    # @return None；执行上述状态更新、界面或日志操作。
    def queue_connection_action(self, action):
        if self.helper_active() or self.closing:
            return
        self.remember_readonly_handoff()
        self.connection_action = action
        self.authorized = False
        self.pending_authorization = None
        self.pending_home = False
        self.pending_home_generation = None
        self.restart_after_release = False
        self.busy = True
        if self.worker.state() == QtCore.QProcess.NotRunning:
            self.run_connection_action()
        elif not self.command('RELEASE'):
            self.connection_action = None
            self.readonly_handoff = None
            self.busy = False
            self.message('退出原连接失败，未启动第二个通信进程。')

    ##
    # @brief 仅在原通信进程和管理进程均停止后消费排队动作。
    # backend 重建后端，clear_fault 启动短管理进程，reconnect 只读启动 worker。关闭期间不调度；调用来自 finished 的释放确认分支。
    # @return None；执行上述状态更新、界面或日志操作。
    def run_connection_action(self):
        if self.closing or self.worker.state() != QtCore.QProcess.NotRunning or self.helper_active():
            return
        action, self.connection_action = self.connection_action, None
        if action == 'backend':
            self.apply_backend_change()
        elif action == 'clear_fault':
            self.start_fault_helper()
        elif action == 'reconnect':
            self.busy = False
            self.start_worker()

    ##
    # @brief 用身份参数白名单启动一次独立清故障管理进程。
    # 只继承 endpoint/device/client/profile/mask/scope 及产物身份，不继承 home-rest 场景或运动授权参数。argv 与 raw 各按 manage_generation 归档；其结束后才重启只读 worker。正常入口由 run_connection_action 保证原 owner 已退出。
    # @return None；执行上述状态更新、界面或日志操作。
    def start_fault_helper(self):
        # 只传身份白名单；绝不沿用home-rest场景或运动授权参数。
        identity_flags = {'--endpoint', '--device', '--client', '--profile', '--mask',
                          '--scope', '--firmware', '--contract', '--model', '--profile-sha256'}
        original = self.cfg['native_arguments']
        args = ['--execute', '--manage-probe', '--target', '0,0,0,0,0,0']
        for index, flag in enumerate(original):
            if flag in identity_flags:
                if index + 1 >= len(original):
                    self.busy = False
                    self.message('清故障身份配置不完整，未启动通信。')
                    return
                args.extend([flag, original[index + 1]])
        self.manage_generation += 1
        self.manage_prefix = self.report / f'clear-fault-{self.manage_generation}'
        args.extend(['--raw-transport-output', str(self.manage_prefix) + '.raw'])
        Path(str(self.manage_prefix) + '.argv.json').write_text(json.dumps([self.cfg['native_binary']] + args, indent=2))
        self.busy = True
        self.message('仅执行明确清故障与模式选择；结束后重新只读连接，不自动使能。')
        self.manage_worker.start(self.cfg['native_binary'], args)

    ##
    # @brief 转交管理进程标准输出归档。
    # @return None；执行上述状态更新、界面或日志操作。
    def manage_stdout_ready(self):
        self.manage_output(False)

    ##
    # @brief 转交管理进程标准错误归档。
    # @return None；执行上述状态更新、界面或日志操作。
    def manage_stderr_ready(self):
        self.manage_output(True)

    ##
    # @brief 保存管理输出并把末段摘要送到面板日志。
    # 解码错误用替换字符；完整数据写入本次 manage_prefix 文件，界面仅显示非空输出末 1200 字符。
    # @param stderr True 读取标准错误并写 .stderr；False 读取标准输出并写 .stdout。
    # @return None；执行上述状态更新、界面或日志操作。
    def manage_output(self, stderr):
        data = bytes(self.manage_worker.readAllStandardError() if stderr else self.manage_worker.readAllStandardOutput()).decode(errors='replace')
        with Path(str(self.manage_prefix) + ('.stderr' if stderr else '.stdout')).open('a') as output:
            output.write(data)
        if data.strip():
            self.message(data.strip()[-1200:])

    ##
    # @brief 处理管理进程无法启动的 Qt 错误。
    # FailedToStart 按退出码 -1 进入统一结束处理，释放 busy 并重新只读连接；其他错误由进程后续生命周期处理。
    # @param error QProcess.ProcessError 枚举。
    # @return None；执行上述状态更新、界面或日志操作。
    def manage_error(self, error):
        if error == QtCore.QProcess.FailedToStart:
            self.manage_finished(-1, 0)

    ##
    # @brief 清理管理忙碌标记并重新只读获取设备真实状态。
    # 无论清故障命令是否成功，都不直接置 authorized；非关闭时重新启动 worker，从新鲜设备反馈判定后续状态。
    # @param code 管理 Native 进程退出码，仅记录，不用作控制授权。
    # @param _status Qt 退出状态参数，本方法未使用。
    # @return None；执行上述状态更新、界面或日志操作。
    def manage_finished(self, code, _status):
        self.busy = self.zero_busy = False
        self.message(f'清故障进程结束，code={code}；仅重新读取设备状态，未自动使能。')
        if not self.closing:
            self.start_worker()

    ##
    # @brief 为下一代只读连接准备独立证据文件并启动唯一 worker。
    # 关闭、helper 活动或 worker 非 NotRunning 时拒绝。重置本地反馈时间、授权和只读事件缓存；raw 每代单独保留，记录 argv 与本地软件/展开模型哈希。模拟模式写独立来源声明，不能混作硬件证据。
    # @return None；执行上述状态更新、界面或日志操作。
    def start_worker(self):
        if self.backend == 'ros':
            self.ros_started_at = time.monotonic()
            from manual_panel_ros_backend import confirmed_ros_launch_argv
            argv = list(self.cfg.get('ros_launch_command', []))
            confirmed = bool(self.cfg.get('operator_confirmed_double_limits', False))
            if argv and not confirmed:
                confirmed = QtWidgets.QMessageBox.question(self, '本次 ROS 连接的标定确认',
                    '机械臂当前是否位于人工双限位姿态，并已得到支撑？\n'
                    '是：允许之后明确发起自动标定与测试；否：本次只读。\n连接本身不会使能或运动。',
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No) == QtWidgets.QMessageBox.Yes
            self.cfg['operator_confirmed_double_limits'] = confirmed
            try:
                if argv:
                    self.ros_backend.transport.assert_no_existing_owner()
                self.ros_runtime.start(confirmed_ros_launch_argv(argv, confirmed) if argv else [])
                # 启动预算只从子链实际启动计时，不计人工确认及只读owner预检耗时。
                self.ros_started_at = time.monotonic()
            except (RuntimeError, OSError) as error:
                self.ros_backend.admitting = False
                self.ros_runtime.fail(str(error))
                self.message(str(error))
                return
            self.ros_backend.connect()
            return
        if self.closing or self.helper_active() or self.worker.state() != QtCore.QProcess.NotRunning:
            return
        # 每次Native保留独立原件；重启只读，不自动申请owner或运动。
        self.raw = self.report / ('transport.raw' if self.worker_generation == 0 else f'transport-{self.worker_generation}.raw')
        self.offset = 24
        self.last_state = None
        self.last_state_ns = 0
        self.authorized = False
        self.pending_authorization = None
        self.stdout_buffer = ''
        self.wait_home = False
        self.readonly_fault = False
        self.readonly_fault_identity = None
        self.readonly_handoff = None
        self.readonly_wait_generation = None
        self.readonly_wait_identity = None
        self.release_confirmed = False
        if self.pending_home:
            self.zero_busy = self.busy = True
        if getattr(self, 'backend', 'real') == 'simulator':
            (self.report / f'simulator-{self.worker_generation}.json').write_text(json.dumps({
                'backend': 'PANEL_KINEMATIC_SIMULATOR', 'hardware_connected': False}, indent=2))
            self.worker_generation += 1
            self.worker.start()
            return
        args = list(self.cfg['native_arguments']) + ['--panel', '--raw-transport-output', str(self.raw)]
        suffix = '' if self.worker_generation == 0 else f'-{self.worker_generation}'
        (self.report / f'argv{suffix}.json').write_text(json.dumps([self.cfg['native_binary']] + args, indent=2))
        (self.report / f'native-software{suffix}.json').write_text(json.dumps({'native_binary_sha256': hashlib.sha256(Path(self.cfg['native_binary']).read_bytes()).hexdigest(), 'expanded_model_sha256': hashlib.sha256(self.description_xml.encode()).hexdigest()}, indent=2) + '\n')
        self.worker_generation += 1
        self.worker.start(self.cfg['native_binary'], args)

    ##
    # @brief 检查最近状态是否属于指定生命周期且未被只读故障锁住。
    # 不单独检查反馈新鲜度或 session 授权，调用方须叠加 fresh/authorized 等条件。
    # @param allowed 允许的 core_state 数值元组，默认 1/2/4。
    # @return bool：有 last_state、无 readonly_fault 且 core_state 命中。
    def control_state_allowed(self, allowed=(1, 2, 4)):
        state = self.last_state
        return bool(state) and not getattr(self, 'readonly_fault', False) and state['core_state'] in allowed

    ##
    # @brief 进入故障只读显示并撤销全部控制准备。
    # 清等待 Home、授权、忙碌与 pending 标定/授权；仅在保存了只读 handoff 时保留必要重启意图。不会自动发清故障或重采零命令。
    # @param identity 可选 (boot_epoch,state_seq) 故障水位，用于后续新状态解除本地故障锁。
    # @return None；执行上述状态更新、界面或日志操作。
    def enter_readonly_fault(self, identity=None):
        self.readonly_fault = True
        if identity is not None:
            self.readonly_fault_identity = identity
        self.wait_home = self.authorized = self.busy = self.zero_busy = False
        self.pending_authorization = None
        self.pending_home = False
        self.pending_home_generation = None
        if getattr(self, 'readonly_handoff', None) is None:
            self.restart_after_release = False
        self.readonly_wait_generation = None
        self.readonly_wait_identity = None

    ##
    # @brief 确认标定模式拒绝来自当前只读连接及其状态水位。
    # 要求当前 session=0；旧 worker、旧 boot 或畸形事件不能恢复当前 wait_home 标志。
    # @param parts CAPTURE_MODE_REJECTED 事件键值字典。
    # @return bool：正在标定、反馈新鲜、只读态、worker 世代与 boot/reference/sequence 均对应。
    def capture_rejection_is_current_readonly(self, parts):
        marker = getattr(self, 'readonly_wait_identity', None)
        if (not self.zero_busy or not self.fresh() or not self.control_state_allowed((1, 2)) or
                getattr(self, 'readonly_wait_generation', None) != self.worker_generation or marker is None):
            return False
        try:
            identity = (int(parts['boot'], 16), int(parts['reference']), int(parts['state_seq']))
        except (KeyError, TypeError, ValueError):
            return False
        state = self.last_state
        return (identity[:2] == marker[:2] and identity[2] >= marker[2] and
                state['boot_epoch'] == marker[0] and state['reference_epoch'] == marker[1] and
                state['state_seq'] >= marker[2] and state['session_id'] == 0)

    ##
    # @brief 查询部署是否使用人工双限位标定流程。
    # @return bool：calibration_mode 等于 manual_limits。
    def manual_limits_mode(self):
        return self.cfg.get('calibration_mode') == 'manual_limits'

    ##
    # @brief 按配置选择需要明确姿态确认的捕获命令。
    # 只生成字符串，不发送；命令后续还要由 Native 校验模式和参考条件。
    # @return str：LIMITS CONFIRM_MARGIN 或 HOME CONFIRM_MARGIN。
    def calibration_command(self):
        return 'LIMITS CONFIRM_MARGIN' if self.manual_limits_mode() else 'HOME CONFIRM_MARGIN'

    ##
    # @brief 识别双限位模式可走专用 Rest 恢复链的实测姿态。
    # 只支持 schema 2.5、profile 201 和完整位置。允许名义边界的原连续性带，但不夹紧实测 State；仅 J2 下侧/J3 上侧可触发，危险阈值是名义限位内 13°−0.003 rad。此返回值只用于恢复判断/说明，实际专用链由 Native 执行。
    # @return list[float] 或 None：危险 J2/J3 回 Rest、其余轴冻结实测角度的度制目标；不可用时 None。
    def manual_limits_retreat_target(self):
        if not self.manual_limits_mode() or not self.last_state:
            return None
        state = self.last_state
        if (state.get('schema_major'), state.get('schema_minor')) != (2, 5) or state.get('profile_id') != 201 or state['position_valid_mask'] != 63:
            return None
        target = []
        boundary = False
        for axis, (value, joint, (lower, upper)) in enumerate(zip(state['position_rad'], self.cfg['joints'], self.limits)):
            nominal_lower, nominal_upper = map(math.radians, joint['nominal_limits_deg'])
            # 与专用Rest恢复保持一致：仅J2下侧/J3上侧使用原.003rad连续性带。
            # 实测State不夹紧；普通目标和其余边界仍按原规则拒绝。
            lower_band = .003 if axis == 1 else math.radians(.1)
            upper_band = .003 if axis == 2 else math.radians(.1)
            if not math.isfinite(value) or value < nominal_lower - lower_band or value > nominal_upper + upper_band:
                return None
            projected = min(nominal_upper, max(nominal_lower, value))
            if abs(projected - value) > .003:
                return None
            below = value < math.radians(lower) - .003
            above = value > math.radians(upper) + .003
            # 仅接受指定机械止挡侧；J2上侧/J3下侧与其它轴均不能借用该链。
            if (axis == 1 and above) or (axis == 2 and below) or (axis not in (1, 2) and (below or above)):
                return None
            # Native's protected Rest still uses the original 13deg-.003rad
            # trigger, independently of the ordinary goal input range.
            rest_margin = math.radians(13) - .003
            danger = ((axis == 1 and value < nominal_lower + rest_margin) or
                      (axis == 2 and value > nominal_upper - rest_margin))
            if danger:
                boundary = True
                target.append(self.cfg['rest_deg'][axis])
            else:
                target.append(math.degrees(value))
        return target if boundary else None

    ##
    # @brief 双限位参考建立后恢复名义角度编辑范围。
    # SpinBox 与滑块可编辑名义范围，预设同步更新；self.limits 工作范围不变，普通执行仍按停车余量内收，不能把输入范围扩大视为执行域扩大。
    # @return None；执行上述状态更新、界面或日志操作。
    def restore_ordinary_goal_ranges(self):
        if not self.manual_limits_mode():
            return
        # 允许输入名义范围；执行前仍按self.limits内收，不扩大实机目标域。
        nominal = [joint['nominal_limits_deg'] for joint in self.cfg['joints']]
        for spin, slider, (lower, upper) in zip(self.inputs, self.sliders, nominal):
            spin.setRange(lower, upper)
            slider.setRange(round(lower * 100), round(upper * 100))
        if hasattr(self, 'target_presets'):
            for row in self.target_presets.rows:
                for box, (lower, upper) in zip(row, nominal):
                    box.setRange(lower, upper)

    ##
    # @brief 校验同 boot 的人工显示锚点以解释未标定原始位置。
    # 配置 boot 和锚点 boot 必须与状态相同，六轴数值有限，原始电机圈数偏差逐轴 ≤0.5 圈；不满足即忽略。该锚点仅供模型显示，绝不建立设备参考或运动授权。
    # @param state 新鲜状态字典，含 raw_position_valid_mask 和 raw_motor_position_rev。
    # @return dict 或 None：规范化 boot、motor_rev、home_rad 的显示专用锚点。
    def optional_display_anchor(self, state):
        candidate = self.cfg.get('initial_display_anchor')
        configured_boot = self.cfg.get('initial_display_boot')
        if not isinstance(candidate, dict) or configured_boot is None or state['raw_position_valid_mask'] != 63:
            return None
        try:
            configured_boot = int(configured_boot, 16) if isinstance(configured_boot, str) else int(configured_boot)
            anchor_boot = candidate.get('boot')
            anchor_boot = int(anchor_boot, 16) if isinstance(anchor_boot, str) else int(anchor_boot)
            motor = candidate.get('motor_rev')
            home = candidate.get('home_rad')
            raw = state['raw_motor_position_rev']
            if configured_boot != state['boot_epoch'] or anchor_boot != configured_boot:
                return None
            if any(not isinstance(values, (list, tuple)) or len(values) != 6 for values in (motor, home, raw)):
                return None
            if any(type(value) not in (int, float) or not math.isfinite(value) for values in (motor, home, raw) for value in values):
                return None
            if any(abs(a - b) > 0.5 for a, b in zip(motor, raw)):
                return None
            return dict(candidate, boot=f'{anchor_boot:x}', motor_rev=list(motor), home_rad=list(home))
        except (TypeError, ValueError, OverflowError):
            return None

    ##
    # @brief 检查实测关节是否满足普通运动的工作起点范围。
    # position_rad 为 rad、limits 为 °；仅保留 0.003 rad 反馈容差，不把 near-limit 位置裁剪成合法状态。
    # @return str 或 None：首个缺反馈/越界原因；全部符合则 None。
    def resume_position_issue(self):
        state = self.last_state
        if not state or state['position_valid_mask'] != 63:
            return '实际位置反馈未就绪。'
        # 与Native原MOVE起点一致：配置已内收13°，只保留原0.003rad反馈容差。
        for axis,(value,(lower,upper)) in enumerate(zip(state['position_rad'],self.limits)):
            if not math.isfinite(value) or value < math.radians(lower)-0.003 or value > math.radians(upper)+0.003:
                return f'J{axis+1}实际 {math.degrees(value):.3f}°，工作范围 [{lower:g}, {upper:g}]°。'
        return None

    ##
    # @brief 为允许的普通 Home 模式计算低速向内恢复目标。
    # 需要 inward_recovery_enabled、实际/配置 schema 2.5、profile 201、完整位置且仍在名义限位内；双限位模式不借此入口。返回目标仅用于判断，RECOVER 的限速策略由 Native 实现。
    # @return list[float] 或 None：需恢复轴到名义限位内 15°、其余轴保持实测值的度制目标。
    def recovery_target(self):
        if (self.manual_limits_mode() or not self.cfg.get('inward_recovery_enabled',False) or not self.last_state or
                self.last_state['position_valid_mask'] != 63 or
                (self.last_state.get('schema_major'),self.last_state.get('schema_minor')) != (2,5) or
                self.last_state.get('profile_id') != 201 or
                (self.cfg.get('wire_schema_major',2),self.cfg.get('wire_schema_minor',5)) != (2,5)):
            return None
        target=[]
        danger=False
        for value,joint,(lower,upper) in zip(self.last_state['position_rad'],self.cfg['joints'],self.limits):
            nominal_lower,nominal_upper=joint['nominal_limits_deg']
            if not math.isfinite(value) or value<math.radians(nominal_lower) or value>math.radians(nominal_upper):
                return None
            if value<math.radians(lower)-0.003:
                target.append(nominal_lower+15.0);danger=True
            elif value>math.radians(upper)+0.003:
                target.append(nominal_upper-15.0);danger=True
            else:
                target.append(math.degrees(value))
        return target if danger else None

    ##
    # @brief 明确确认后提交固定低速向内恢复。
    # 要求已授权、有零点、新鲜反馈、合法状态且不忙；提示最高 0.1°/s、0.01 rad/s²。确认后再次检查条件，再发 RECOVER；成功写入仅置 busy，等待设备结果。
    # @return None；执行上述状态更新、界面或日志操作。
    def start_recovery(self):
        target=self.recovery_target()
        if not self.control_state_allowed((2, 4)) or not self.authorized or not self.calibration or not self.fresh() or self.busy or target is None:
            return
        answer=self.confirm_execution('低速回工作区', '仅将缺少停车余量的关节向内收至原限位内15°，其它轴保持当前实测位置。\n'
            '固定最高0.1°/s、加速度0.01rad/s²；请确认有支撑且内收路径畅通。\n'
            + '目标：' + ' / '.join(f'J{i+1} {q:.3f}°' for i,q in enumerate(target)))
        if answer==QtWidgets.QMessageBox.Yes and self.control_state_allowed((2, 4)) and self.authorized and self.fresh() and not self.busy and self.recovery_target() is not None:
            if self.command('RECOVER'):
                self.busy=True

    ##
    # @brief 复用本 boot 参考并显式恢复控制会话。
    # 只在等待人工命令的只读态 2 使用；检查工作起点，若需双限位 Rest 退让或专用低速恢复则给出对应说明。模态确认后重新检查反馈与恢复条件，发 RESUME 后仍不立即授权；CALIBRATION_* 事件和后续 State 才能建立 authorized。
    # @return None；执行上述状态更新、界面或日志操作。
    def start_resume(self):
        if self.backend == 'ros':
            if (self.closing or self.busy or not self.fresh() or not self.calibration or
                    self.ros_backend.released or not self.ros_backend.admitting or
                    not self.control_state_allowed((2,)) or self.last_state['session_id'] != 0):
                return
            if self.confirm_execution('恢复 ROS 手动控制',
                    '复用本 boot 已有参考，由 ROS 控制链完成 Rest 核验、激活与保持；不重新采零，也不运行自动基线运动。') != QtWidgets.QMessageBox.Yes:
                return
            if (self.fresh() and self.calibration and self.control_state_allowed((2,)) and
                    not self.busy and not self.closing and self.last_state['session_id'] == 0):
                self.ros_backend.request_prepare_manual_control()
            return
        if not self.control_state_allowed((2,)) or not self.wait_home or not self.calibration or not self.fresh() or self.busy:
            return
        issue = self.resume_position_issue()
        retreat = self.manual_limits_retreat_target()
        if issue and retreat is None and self.recovery_target() is None:
            self.message('零点保留，实际位置不足停车余量，需支撑调整后恢复；无需重新归零。' + issue)
            return
        resume_text = ('复用本次boot零点，不重新采零。本次将缓慢退回工作Rest：仅危险的J2下侧/J3上侧移动，其余轴冻结当前实测值。\n'
                       '固定最高2°/s、加速度0.1rad/s²；完成原Rest验证后才恢复普通运动。\n'
                       if retreat is not None else
                       '复用设备本次boot的零点，并在当前实测位置使能保持。不会回到Home或Rest。\n'
                       + ('当前位置仅允许固定0.1°/s低速内收；使能后请点击“低速回工作区”。\n' if issue else ''))
        answer = self.confirm_execution('恢复控制', resume_text + '请确认机械臂有支撑、路径安全且可操作现场保护。')
        if answer == QtWidgets.QMessageBox.Yes:
            # 模态对话期间反馈仍会更新；确认后再次检查实际位置和连接，不能使用旧位置。
            issue = self.resume_position_issue()
            if not self.control_state_allowed((2,)) or not self.wait_home or not self.calibration or not self.fresh() or self.busy or (issue and self.manual_limits_retreat_target() is None and self.recovery_target() is None):
                self.message('零点保留，恢复条件已变化，未发送使能命令。' + (issue or '请等待新鲜静止反馈。'))
                return
            if self.command('RESUME'):
                self.busy = True
                self.wait_home = False

    ##
    # @brief 用信号屏蔽将度制目标同步到百分之一度滑块。
    # QSignalBlocker 防止 slider.valueChanged 再反向触发递归更新；不发送设备命令。
    # @param slider 该轴 QSlider，整数刻度为 0.01°。
    # @param value QDoubleSpinBox 的目标角度，°。
    # @return None；执行上述状态更新、界面或日志操作。
    @staticmethod
    def sync_slider(slider, value):
        blocker = QtCore.QSignalBlocker(slider)
        slider.setValue(round(value * 100))
        del blocker

    ##
    # @brief 按反馈可用性发布或隐藏测量模型。
    # 仅在可见状态变化时发布到 measured_robot_description，避免反馈失鲜仍展示完整模型为可信实测。目标预览描述使用另一个 topic。
    # @param visible True 发布展开的实测模型；False 发布仅有 base_link 的占位描述。
    # @return None；执行上述状态更新、界面或日志操作。
    def show_actual_description(self, visible):
        if visible != self.description_visible:
            message = String()
            message.data = self.description_xml if visible else '<robot name="feedback_unavailable"><link name="base_link"/></robot>'
            self.description_publisher.publish(message)
            self.description_visible = visible

    ##
    # @brief 更新目标虚影失败配色并缓存当前颜色状态。
    # 只发布 target_preview 描述；不改测量模型、参考或命令授权，失败时虚影可能保留上一次有效关节姿态。
    # @param failed 是否将预览模型染红。
    # @return None；执行上述状态更新、界面或日志操作。
    def set_preview_failed(self, failed):
        if self.preview_failed != failed:
            from manual_moveit_target import colored_description
            message = String()
            message.data = colored_description(self.description_xml, failed)
            self.preview_description_publisher.publish(message)
            self.preview_failed = failed

    ##
    # @brief 把六轴度制姿态填入目标输入并更新目标虚影。
    # 不写 last_state/actual_q，不发布实测位置，不发送 MOVE；输入框仍按自身范围处理值。
    # @param degrees 六轴目标角度，°；通过 zip 写入对应输入框。
    # @param announce True 时记录“仅更新目标”说明；动画可设 False 降低日志量。
    # @return None；执行上述状态更新、界面或日志操作。
    def set_preview_pose(self, degrees, announce=True):
        for box, value in zip(self.inputs, degrees):
            box.setValue(value)
        if announce:self.message('仅更新目标虚影，未发送实机命令。')

    ##
    # @brief 解释操作员日志并同时显示和追加本地文件。
    # explain_panel_log 保留原编号；panel.log 使用墙上时间便于人工阅读，命令/事件 JSON 中的 host_ns 则使用单调时钟。
    # @param text 任意可转为字符串的事件/说明。
    # @return None；执行上述状态更新、界面或日志操作。
    def message(self, text):
        from manual_panel_messages import explain_panel_log
        explanation = explain_panel_log(str(text))
        self.log.appendPlainText(explanation)
        with (self.report/'panel.log').open('a') as log:
            log.write(time.strftime('%Y-%m-%d %H:%M:%S')+' '+explanation+'\n')

    ##
    # @brief 将一条命令写入当前 worker 并记命令日志。
    # True 只表示已交给通信对象，不代表设备接受、授权或动作完成。真实/模拟写入不同 JSONL；host_ns 为 monotonic_ns，业务入口负责在调用前检查权限和条件。
    # @param text 不带末尾换行的 Native 风格单行命令。
    # @return bool：worker 正在运行且已调用 write 时 True，否则 False。
    def command(self, text):
        if self.backend == 'ros':
            self.message('ROS 模式不支持 Native 命令；请使用 ROS BackendAPI。')
            return False
        if self.worker.state() != QtCore.QProcess.Running:
            self.message('通信进程已退出；需重新只读连接，零点以设备本次boot反馈为准。')
            return False
        self.worker.write((text + '\n').encode())
        journal = 'simulator-commands.jsonl' if getattr(self, 'backend', 'real') == 'simulator' else 'commands.jsonl'
        with (self.report / journal).open('a') as f:
            f.write(json.dumps({'host_ns': time.monotonic_ns(), 'command': text}) + '\n')
        return True

    ##
    # @brief 同时计算宿主到达年龄和设备 CAN 反馈年龄。
    # last_state_ns 与 monotonic_ns 同基准；can_rx_age_us 乘 1000 转 ns。读取旧 State 或只读移交凭证都不能延长这一时限。
    # @return bool：有 last_state 且宿主年龄非负、单独及叠加最大 CAN 年龄均不超过 250 ms。
    def fresh(self):
        if self.last_state is None:
            return False
        age = time.monotonic_ns() - self.last_state_ns
        return (0 <= age <= 250_000_000 and
                max(self.last_state['can_rx_age_us']) * 1000 + age <= 250_000_000)

    ##
    # @brief 确认当次人工姿态后发起替换参考并低速回 Rest。
    # 双限位配置必须精确符合当前模型 J2 下挡/J3 上挡姿态；普通模式要求人工 Home。若当前已有 owner，先 RELEASE 并绑定下一 worker_generation，待新只读连接和静止反馈后才捕获；不能自动把当前 Rest 当作 Home。
    # @return None；执行上述状态更新、界面或日志操作。
    def start_zero(self):
        if self.backend == 'ros':
            if self.closing or self.busy or self.ros_backend.released or not self.ros_backend.admitting:
                self.message('请等待当前操作结束，或重新连接后再恢复控制。')
                return
            reuse = bool(self.calibration and self.fresh() and self.ros_status is not None and self.ros_status.session_id == 0)
            if not self.cfg.get('operator_confirmed_double_limits', False) and not reuse:
                self.message('本次 ROS 连接未确认双限位；请重新启动面板并明确确认后再标定。')
                return
            prompt = ('复用本次 boot 已保留的参考，由 ROS 控制链恢复手动控制；不会重新采零。' if reuse else
                      '确认设备处于人工双限位初始姿态；由 ROS 控制链标定、退回 Rest 并准备手动控制。')
            prompt += '\n此操作不运行自动验收测试；必要的限位退让由 ROS 控制链完成。'
            if (self.confirm_execution('标定 / 恢复手动控制', prompt) == QtWidgets.QMessageBox.Yes and
                    not self.closing and not self.busy and self.fresh()):
                self.ros_backend.request_prepare_manual_control()
            return
        if (not self.control_state_allowed() or not self.fresh() or self.busy or self.closing or self.pending_home or
                self.worker.state() != QtCore.QProcess.Running or
                not (self.wait_home or self.authorized)):
            return
        if self.manual_limits_mode():
            pose = self.cfg.get('calibration_pose_deg')
            if not isinstance(pose, (list, tuple)) or len(pose) != 6 or any(type(value) not in (int, float) or not math.isfinite(value) for value in pose):
                self.message('缺少完整人工双限位姿态配置，未发送标定请求。')
                return
            expected_pose = list(self.cfg.get('home_deg', [0, 0, 90, 0, 0, 0]))
            expected_pose[1] = self.cfg['joints'][1]['nominal_limits_deg'][0]
            expected_pose[2] = self.cfg['joints'][2]['nominal_limits_deg'][1]
            if list(pose) != expected_pose:
                self.message('人工双限位姿态与当前模型不一致，未发送标定请求。')
                return
            title = '确认人工双限位姿态和退回路径'
            confirmation = (f'现在是否已由人工轻靠J2/J3机械止挡，实物为 {list(pose)}°，其余四轴0°，有固定支撑？\n'
                            '不要用电机自动顶挡块。此姿态不是旧Home；请确认原工作Rest路径畅通。\n'
                            '点击“是”将替换当前参考并固定低速退工作Rest（≤2°/s、≤0.1rad/s²）。\n'
                            '若尚未人工摆到该限位姿态，请点“否”；不会把当前姿态冒充参考。')
        else:
            title = '确认人工 Home 和运动路径'
            confirmation = ('现在实物是否已人工摆到 Home [0°,0°,90°,0°,0°,0°]？\n'
                            f"有支撑、现场可操作保护，且到工作 Rest {self.cfg['rest_deg']}° 的路径畅通？\n"
                            '点击“是”将捕获 Home 零点并立即缓慢运动到 Rest。\n'
                            '当前若仍在 Rest，请点“否”，先手动恢复 Home。')
        answer = self.confirm_execution(title, confirmation)
        if answer != QtWidgets.QMessageBox.Yes or not self.control_state_allowed() or not self.fresh():
            return
        # 用户明确重新Home才可重采零；先确认旧owner已失能释放。
        self.zero_busy = self.busy = True
        if self.wait_home:
            if not self.command(self.calibration_command()):
                self.zero_busy = self.busy = False
                return
            self.wait_home = False
        else:
            self.pending_home = True
            self.pending_home_generation = self.worker_generation + 1
            self.restart_after_release = True
            self.authorized = False
            if not self.command('RELEASE'):
                self.pending_home = self.restart_after_release = False
                self.pending_home_generation = None
                self.zero_busy = self.busy = False
                return
        self.message('已确认当次人工双限位；等待失能释放后捕获新参考并低速退工作Rest。' if self.manual_limits_mode() else '已确认当次人工Home；等待失能释放后捕获新参考并缓慢Rest。')

    ##
    # @brief 检查名义目标后按工作范围投影并明确提交六轴运动。
    # 先检查会话、范围、参数/路径互斥与人工确认；ROS关节直控按所选动态上限准备完整FJT，Native发MOVE_SPEED。ROS此入口不调用MoveIt避碰；末端规划页仍检查完整路径的碰撞。
    # @return None；执行上述状态更新、界面或日志操作。
    def start_move(self):
        ros_direct = self.backend == 'ros'
        if not ros_direct and hasattr(self, 'obstacle_tab') and (self.obstacle_tab.blocked or not self.obstacle_tab.scene_ready):
            self.message('障碍物场景尚未确认，禁止提交运动。')
            return
        if hasattr(self, 'motor_parameter_tab') and (self.motor_parameter_tab.pending or self.motor_parameter_tab.queue):
            return
        if hasattr(self, 'moveit_tab') and (self.moveit_tab.executing or self.moveit_tab.auto_test.active):
            return
        if not self.control_state_allowed((2, 4)) or not self.authorized or not self.calibration or not self.fresh() or self.busy:
            return
        requested = [spin.value() for spin in self.inputs]
        nominal = [joint['nominal_limits_deg'] for joint in self.cfg['joints']]
        if not valid_goal(requested, nominal):
            self.message('目标无效；要求六轴有限角度且全部处于名义硬范围。未发送运动命令。')
            return
        goal = [min(upper, max(lower, value))
                for value, (lower, upper) in zip(requested, self.limits)]
        if not valid_goal(goal, self.limits):
            self.message('有效目标不满足软件范围，未发送运动命令。')
            return
        projection = '；'.join(f'J{i + 1} {before:.2f}° → {after:.2f}°'
                              for i, (before, after) in enumerate(zip(requested, goal)) if before != after)
        if projection:
            self.set_preview_pose(goal)
            self.message('已按现有停车余量内收请求：' + projection + '。仅更新目标虚影，未发送运动命令。')
        text = ' / '.join(f'J{i + 1} {v:.2f}°' for i, v in enumerate(goal))
        if projection:
            text = '请求 → 有效目标：' + projection + '\n实际发送：' + text
        answer = self.confirm_execution('执行六轴目标', text + f'\n本次目标速度 {self.speed_input.value():.1f}°/s，加速度 {self.acceleration_input.value():.2f}rad/s²；确认实物路径畅通后执行。\n'
            '将按所选速度和加速度规划；较长运动使用原有会话续租。')
        if answer != QtWidgets.QMessageBox.Yes or self.closing or not self.control_state_allowed((2, 4)) or not self.authorized or not self.fresh():
            return
        if ros_direct:
            from manual_moveit_execution import joint_goal_path, ros_trajectory
            try:
                path = joint_goal_path(self.actual_q, [math.radians(v) for v in goal], self.limits,
                                       self.speed_input.value(), self.acceleration_input.value())
                trajectory = ros_trajectory(path, [j['name'] for j in self.cfg['joints']])
                if hasattr(self, 'moveit_tab'):
                    self.moveit_tab.invalidate()
                self.busy = True
                if not self.ros_backend.submit_trajectory(trajectory):
                    self.busy = False
                    self.message('ROS关节轨迹未提交；未发送Native命令。')
                else:
                    self.message('关节直控轨迹已提交ROS；不进行MoveIt避碰，等待 ROS 执行结果。')
            except ValueError as error:
                self.busy = False
                self.message(str(error))
        elif self.command(f'MOVE_SPEED {self.speed_input.value():.1f} ACCEL {self.acceleration_input.value():.2f} ' + ' '.join(f'{v:.8f}' for v in goal)):
            self.busy = True

    ##
    # @brief 请求受控停车且保持使能，失败时退回 STOP 释放。
    # 要求新鲜已授权反馈和允许生命周期；发 CANCEL_HOLD 后设置 cancel_hold_pending 并给 3 s 事件期限。该函数只请求取消，是否已停车需事件及具体调用方的后续 Hold 检查。
    # @return None；执行上述状态更新、界面或日志操作。
    def cancel_motion_hold(self):
        if self.backend == 'ros':
            self.ros_backend.cancel()
            self.message('已请求ROS Action取消；等待 ROS 停车终态。')
            return
        if not self.fresh() or not self.authorized or not self.control_state_allowed((2,3,4,5,6)):
            self.release('STOP');return
        self.busy = True
        self.cancel_hold_pending = True
        if not self.command('CANCEL_HOLD'):
            self.cancel_hold_pending = False;self.release('STOP');return
        self.message('已请求受控停车并保持使能；等待设备确认。')
        QtCore.QTimer.singleShot(3000,self.check_cancel_hold)

    ##
    # @brief 在取消请求 3 秒后检查是否仍未收到保持事件。
    # pending 未解除则清标志并转 STOP 释放，避免无限等待；已确认的取消不会在此重复停止。
    # @return None；执行上述状态更新、界面或日志操作。
    def check_cancel_hold(self):
        if getattr(self,'cancel_hold_pending',False):
            self.cancel_hold_pending = False
            self.message('受控停车未确认，转入停止释放。')
            self.release('STOP')

    ##
    # @brief 取消后续规划并请求停止、失能和释放当前控制。
    # 先结束自动测试/路径队列，撤销本地授权，保留必要只读移交证据并置 busy。非关闭时仅在原 worker 退出且释放被证实后只读重连；写命令和提示文字不等于 RELEASE_CONFIRMED。
    # @param command STOP 或 RELEASE，由对应操作入口选择。
    # @return None；执行上述状态更新、界面或日志操作。
    def release(self, command):
        if self.backend == 'ros':
            self.ros_runtime.begin_release()
            self.ros_backend.release()
            return
        if hasattr(self,'moveit_tab') and self.moveit_tab.auto_test.active:
            self.moveit_tab.auto_test.abort('已请求停止或释放设备',stop=False)
        if hasattr(self, 'moveit_tab') and self.moveit_tab.executing:
            self.moveit_tab.cancel_execution('已请求停止；剩余规划路径作废。')
        self.cancel_hold_pending = False
        self.remember_readonly_handoff()
        self.authorized = False
        self.pending_authorization = None
        self.busy = True
        self.restart_after_release = not self.closing
        self.command(command)
        self.message('已请求停止、失能并释放；等待设备确认。')

    ##
    # @brief 仅保存故障只读进程可移交读取者的有限凭证。
    # 要求新鲜、未授权、core_state=9、session=0；记录 worker_generation/boot/reference。凭证只用于 finished 判断原只读进程正常退出，不授予命令权限，也不续延 State 新鲜度。
    # @return None；执行上述状态更新、界面或日志操作。
    def remember_readonly_handoff(self):
        # This proves only that a stopped process can hand over its USB reader.
        # It grants no command authority and never extends State freshness.
        state = self.last_state
        self.readonly_handoff = ((self.worker_generation, state['boot_epoch'], state['reference_epoch'])
                                 if state and self.fresh() and not self.authorized and
                                 state['core_state'] == 9 and state['session_id'] == 0 else None)

    ##
    # @brief 把新鲜实际关节角复制为目标输入。
    # actual_q 单位 rad，写入输入框前转 °；只填目标，不提交命令或改变设备参考。
    # @return None；执行上述状态更新、界面或日志操作。
    def copy_actual(self):
        if not self.fresh() or not hasattr(self, 'actual_q'):
            return
        for box, radians in zip(self.inputs, self.actual_q):
            box.setValue(math.degrees(radians))
        self.message('目标已填入；未发送运动命令。')

    ##
    # @brief 按行归档和处理 Native/模拟事件并分发各子状态机。
    # 缓冲不完整行，优先让电机参数页消费 MOTOR_PARAM_RESULT。故障撤销授权；WAIT_MANUAL_HOME 记录当前世代与水位；CALIBRATION_* 只设置 pending_authorization，必须等后续 State。完成、拒绝和释放分别更新忙碌/释放标记，最后转交 MoveIt 与障碍事务。
    # @return None；执行上述状态更新、界面或日志操作。
    def stdout_ready(self):
        text = bytes(self.worker.readAllStandardOutput()).decode(errors='replace')
        journal = 'simulator.events' if getattr(self, 'backend', 'real') == 'simulator' else 'native.stdout'
        with (self.report / journal).open('a') as f:
            f.write(text)
        self.stdout_buffer += text
        while '\n' in self.stdout_buffer:
            line, self.stdout_buffer = self.stdout_buffer.split('\n', 1)
            if hasattr(self, 'motor_parameter_tab') and self.motor_parameter_tab.handle_line(line):
                continue
            if not line.startswith('home_rest_event='):
                if line:
                    self.message(line)
                continue
            parts = dict(item.split('=', 1) for item in line.split() if '=' in item)
            event = parts['home_rest_event']
            if event not in ('REST_SAMPLE', 'LEASE_RENEWED'):
                self.message(event)
            if event == 'READONLY_FAULT':
                self.enter_readonly_fault((int(parts['boot'], 16), int(parts['state_seq'])))
                self.message(('设备参考仍存在；' if int(parts['reference']) != 0 else '当前无有效设备参考；') + '当前故障禁止控制。保持只读显示，不自动清故障或重新归零。')
            elif event == 'WAIT_MANUAL_HOME':
                self.wait_home = not getattr(self, 'readonly_fault', False) and not (self.last_state and self.last_state['core_state'] == 9)
                if self.wait_home:
                    try:
                        self.readonly_wait_identity = (int(parts['boot'], 16), int(parts['reference']), int(parts['state_seq']))
                        self.readonly_wait_generation = self.worker_generation
                    except (KeyError, TypeError, ValueError):
                        self.readonly_wait_identity = None
                        self.readonly_wait_generation = None
            elif event == 'CAPTURE_MODE_REJECTED':
                if self.capture_rejection_is_current_readonly(parts):
                    self.authorized = False
                    self.pending_authorization = None
                    self.busy = self.zero_busy = self.pending_home = False
                    self.pending_home_generation = None
                    self.restart_after_release = False
                    self.wait_home = True
                    self.message('标定模式与设备不匹配，保持只读等待明确姿态确认；未自动重试。')
            elif event == ('LIMITS_CONFIRMED_NO_MOTION' if self.manual_limits_mode() else 'HOME_CONFIRMED_NO_MOTION') and self.zero_busy and self.control_state_allowed((1, 2)):
                self.command('REST CONFIRM_MARGIN')
            elif event in ('CALIBRATION_READY', 'CALIBRATION_RESUMED'):
                if getattr(self, 'readonly_fault', False) or (self.last_state and self.last_state['core_state'] == 9):
                    self.authorized = False
                    self.pending_authorization = None
                    continue
                self.calibration_event_ns = time.monotonic_ns()
                self.calibration = (int(parts['boot'], 16), int(parts['reference']))
                self.authorized = False
                self.control_session = int(parts['session'])
                # stdout事件不是反馈帧；等同tuple且达到事件State/epoch水位的新鲜反馈才开放MOVE。
                ##
                # 标定事件包含 boot/session/reference 和 state_seq、motion、management 水位；新鲜 State 到达并核对之前 authorized 保持 False。
                self.pending_authorization = dict(
                    boot_epoch=int(parts['boot'],16),session_id=int(parts['session']),
                    reference_epoch=int(parts['reference']),state_seq=int(parts['state_seq']),
                    motion_epoch=int(parts['motion']),management_epoch=int(parts['management']))
                self.zero_busy = self.busy = False
                self.restore_ordinary_goal_ranges()
                if event == 'CALIBRATION_READY':
                    for box, value in zip(self.inputs, self.cfg['rest_deg']):
                        box.setValue(value)
                self.message('设备零点有效，本控制会话已就绪；失能或换软件连接保留同boot已完成零点。')
            elif event == 'MOVE_CANCELED_HOLD':
                self.cancel_hold_pending = False
                self.busy = False
                self.message('已停止测试运动并确认保持使能；不会自动继续或回Home。')
            elif event == 'MOVE_COMPLETED':
                self.busy = bool(hasattr(self,'moveit_tab') and self.moveit_tab.auto_test.active)
                self.message('目标完成。连接正常时持续保持；停止、断连会释放控制；已完成零点由设备同boot保留。')
            elif event == 'RESUME_NOT_READY':
                self.authorized = False
                self.pending_authorization = None
                self.busy = False
                self.wait_home = self.control_state_allowed((1, 2))
            elif event.startswith('MOVE_REJECTED') or event in ('MOVE_NOOP', 'MOVE_NOT_READY'):
                self.busy = False
            elif event == 'RELEASE_CONFIRMED':
                self.authorized = False
                self.pending_authorization = None
                self.release_confirmed = True
                self.message('设备已确认失能与释放；已完成零点由设备同boot保留。')
            if hasattr(self, 'moveit_tab'):
                self.moveit_tab.native_event(event, parts)
            if hasattr(self, 'obstacle_tab'):
                self.obstacle_tab.native_event(event, parts)

    ##
    # @brief 完整归档通信错误输出并显示末段摘要。
    # native.stderr 追加所有解码文本；界面仅显示非空末 1200 字符，不用 stderr 文本直接建立控制授权。
    # @return None；执行上述状态更新、界面或日志操作。
    def stderr_ready(self):
        text = bytes(self.worker.readAllStandardError()).decode(errors='replace')
        with (self.report / 'native.stderr').open('a') as f:
            f.write(text)
        if text.strip():
            self.message(text.strip()[-1200:])

    ##
    # @brief 在通信进程终止后判定是否可以串行启动下一连接。
    # 撤销授权并清忙；只在 code 为 0/130 且有 RELEASE_CONFIRMED，或 code=0 且有合法只读退出证据时移交/重连。未证实释放时取消连接动作及 pending_home，禁止自动创建第二 owner。
    # @param code Native 或模拟 worker 退出码。
    # @param _status Qt 退出状态，此处未使用。
    # @return None；执行上述状态更新、界面或日志操作。
    def finished(self, code, _status):
        was_readonly = self.wait_home
        handoff = getattr(self, 'readonly_handoff', None)
        state = self.last_state
        fault_readonly = (handoff is not None and state is not None and not self.authorized and
                          handoff == (self.worker_generation, state['boot_epoch'], state['reference_epoch']) and
                          state['session_id'] == 0)
        self.readonly_handoff = None
        self.authorized = False
        self.pending_authorization = None
        self.wait_home = False
        self.busy = self.zero_busy = False
        self.message(f'通信进程已退出，code={code}；控制已结束，零点以设备当前boot反馈为准。原始证据：{self.raw}')
        if getattr(self, 'connection_action', None) is not None:
            if not self.closing and code in (0, 130) and (self.release_confirmed or (code == 0 and (was_readonly or fault_readonly))):
                QtCore.QTimer.singleShot(0, self.run_connection_action)
            else:
                self.connection_action = None
                self.message('原连接退出未证实释放，未启动其他通信进程；可检查后手动重新连接。')
            return
        restart = self.restart_after_release and (self.release_confirmed or ((was_readonly or fault_readonly) and code == 0)) and code in (0, 130) and not self.closing
        self.restart_after_release = False
        if restart:
            if fault_readonly:
                self.pending_home = False
                self.pending_home_generation = None
            QtCore.QTimer.singleShot(0, self.start_worker)
        else:
            self.pending_home = False
            self.pending_home_generation = None

    ##
    # @brief 按 boot、严格序号及待授权事件水位接收实际状态。
    # 同 boot 必须 state_seq 增加；待授权事件前的帧及同水位错误 session/reference/motion/management 被丢弃。boot 变化将旧 boot 永久加入本窗口退役集合并撤销授权，防迟到旧帧回滚状态。
    # @param state 部署解码器得到的状态字典。
    # @param arrival_ns 该 transport 记录到达宿主的单调纳秒时间。
    # @return bool：新状态被接受并更新 last_state 时 True；旧帧/退役 boot 时 False。
    def accept_state(self, state, arrival_ns):
        boot, sequence = state['boot_epoch'], state['state_seq']
        if boot in self.retired_state_boots:
            return False
        if self.state_boot == boot and sequence <= self.state_sequence:
            return False
        pending = self.pending_authorization
        if pending and boot == pending['boot_epoch']:
            if sequence < pending['state_seq']:
                return False
            # 事件对应帧唯一；同水位但错tuple是迟到旧帧，不用它撤销事件后的控制准备。
            if sequence == pending['state_seq'] and any(
                    state[key] != pending[key] for key in ('session_id','reference_epoch','motion_epoch','management_epoch')):
                return False
        if self.state_boot is not None and self.state_boot != boot:
            self.retired_state_boots.add(self.state_boot)
            self.authorized = False
            self.pending_authorization = None
        self.state_boot, self.state_sequence = boot, sequence
        self.last_state, self.last_state_ns = state, arrival_ns
        return True

    ##
    # @brief 用新鲜反馈兑现或撤销待授权、零点和会话状态。
    # 先处理故障锁；只有同 boot/session/reference、达到事件 motion/management 水位且位置全有效才从 pending 升为 authorized。参考变化、无效位置或 session 改变撤销控制；设备同 boot 零点可存在而当前会话仍未授权。
    # @return None；执行上述状态更新、界面或日志操作。
    def reconcile_control_state(self):
        if not self.fresh() or self.last_state is None:
            return
        state = self.last_state
        if state['core_state'] == 9:
            self.enter_readonly_fault((state['boot_epoch'], state['state_seq']))
        elif getattr(self, 'readonly_fault', False):
            marker = getattr(self, 'readonly_fault_identity', None)
            if marker is not None and ((state['boot_epoch'] == marker[0] and state['state_seq'] > marker[1]) or
                                       (state['boot_epoch'] != marker[0] and marker[0] in self.retired_state_boots)):
                self.readonly_fault = False
                self.readonly_fault_identity = None
        pending = self.pending_authorization
        if pending:
            if state['boot_epoch'] == pending['boot_epoch'] and state['state_seq'] < pending['state_seq']:
                return
            exact_reference = all(state[key] == pending[key] for key in ('boot_epoch','session_id','reference_epoch'))
            if exact_reference and state['motion_epoch'] >= pending['motion_epoch'] and state['management_epoch'] >= pending['management_epoch'] and state['position_valid_mask'] == 63:
                self.authorized = True
                self.pending_authorization = None
            elif state['boot_epoch'] != pending['boot_epoch'] or (state['state_seq'] > pending['state_seq'] and not exact_reference):
                self.authorized = False
                self.pending_authorization = None
        observed = (state['boot_epoch'],state['reference_epoch'])
        if state['reference_epoch'] != 0 and state['position_valid_mask'] == 63:
            if self.calibration and observed != self.calibration:
                self.authorized = False
            self.calibration = observed
        else:
            self.calibration = None
            self.authorized = False
        if self.authorized and state['session_id'] != self.control_session:
            self.authorized = False

    ##
    # @brief 有界读取模拟快照或 Native 原始 transport 中的 State。
    # 真机验证 24 字节 transport 头，每 tick 最多读 128 个完整 4320 字节记录；检查传输结果、方向、接口和有效载荷长度后交给部署解码器。只推进完整记录 offset，半条留待下次；线协议长度来自 wire_layout，不维护另一个固定 State 尺寸。
    # @return None；执行上述状态更新、界面或日志操作。
    def read_state(self):
        if getattr(self, 'backend', 'real') == 'simulator':
            if self.worker.state() == QtCore.QProcess.Running:
                self.accept_state(self.worker.model.snapshot(), time.monotonic_ns())
            return
        if not self.raw.exists():
            return
        with self.raw.open('rb') as f:
            header = f.read(24)
            if len(header) != 24 or struct.unpack('<6I', header)[:4] != (0x52575034, 3, 4320, 4096):
                return
            f.seek(self.offset)
            for _ in range(128):
                record = f.read(4320)
                if len(record) != 4320:
                    break
                ##
                # 完整记录已消费才推进 offset；无关接口或非法记录会跳过，但不会把残缺尾部当作完整证据。
                self.offset += 4320
                size, result = struct.unpack_from('<II', record, 48)
                interface = struct.unpack_from('<I', record, 76)[0]
                if (record[72] != 0 or result != 0 or interface != self.cfg['state_interface_id'] or not (4 <= size <= 4096)):
                    continue
                try:
                    # 中央公共布局决定当前payload长度，面板不维护尺寸副本。
                    state = self.decode('DummyJointStateV2', record[224:224 + size])
                except (ValueError, struct.error):
                    continue
                self.accept_state(state,struct.unpack_from('<Q',record,8)[0])

    ##
    # @brief 批量驱动 ROS 响应、状态接收、权限门控与独立目标/测量发布。
    # 每轮最多 8 次非阻塞 spin；先读状态再推进障碍/参数页。始终发布目标 rad 到独立 topic；新鲜反馈才显示/发布实际关节。pending_home 必须等对应新 worker、只读状态及静止反馈。按钮可用性综合新鲜度、授权、场景、参数事务和路径执行；显示锚点仅提供未标定可视化，不能开运动入口。
    # @return None；执行上述状态更新、界面或日志操作。
    def tick(self):
        if not rclpy.ok():
            self.timer.stop()
            self.close()
            return
        # 有界批量消费只读规划响应；避免逐帧一次回调使碰撞检查耗时近一分钟。
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0)
        if self.backend == 'ros':
            self.ros_tick()
            return
        self.read_state()
        if hasattr(self, 'obstacle_tab'):
            self.obstacle_tab.tick()
        if hasattr(self, 'motor_parameter_tab'):
            self.motor_parameter_tab.tick()
        if self.preview_failed is None:
            self.set_preview_failed(False)
        # 独立目标topic/TF；仅预览，永不写入实机反馈topic或发MOVE。
        target = JointState()
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.name = [j['name'] for j in self.cfg['joints']]
        target.position = [math.radians(box.value()) for box in self.inputs]
        self.target_publisher.publish(target)
        fresh = self.fresh()
        state = self.last_state
        self.reconcile_control_state()
        ##
        # 真实标定入口要求原始六轴位置有效；模拟器只提供运动学位置，因此单独按 position_valid_mask 判断，不能伪装 raw CAN 有效。
        position_ready = fresh and state['raw_position_valid_mask'] == 63
        if getattr(self, 'backend', 'real') == 'simulator':
            position_ready = fresh and state['position_valid_mask'] == 63
        ##
        # quiet 要求六轴速度掩码有效且 |v|≤0.01 rad/s；用于人工标定/恢复入口，不把肉眼静止或目标虚影当作实测静止。
        quiet = fresh and position_ready and state['velocity_valid_mask'] == 63 and all(math.isfinite(v) and abs(v) <= 0.01 for v in state['estimated_velocity_rad_s'])
        if (self.pending_home and self.wait_home and quiet and not self.closing and
                self.pending_home_generation is not None and
                self.worker_generation >= self.pending_home_generation and self.control_state_allowed((1, 2))):
            if self.command(self.calibration_command()):
                self.pending_home = False
                self.pending_home_generation = None
                self.zero_busy = self.busy = True
                self.wait_home = False
        self.zero.setEnabled(self.control_state_allowed() and quiet and not self.busy and not self.pending_home and self.worker.state() == QtCore.QProcess.Running and
                             (self.wait_home or self.authorized) and state['core_state'] in (1, 2, 4))
        resume_issue = self.resume_position_issue() if fresh and self.calibration else None
        self.resume.setEnabled(self.control_state_allowed((2,)) and bool(self.calibration) and self.wait_home and quiet and not self.busy and (resume_issue is None or self.manual_limits_retreat_target() is not None or self.recovery_target() is not None))
        self.recover.setEnabled(self.control_state_allowed((2, 4)) and self.authorized and bool(self.calibration) and quiet and not self.busy and self.recovery_target() is not None)
        self.move.setEnabled(self.control_state_allowed((2, 4)) and self.resume_position_issue() is None and self.authorized and bool(self.calibration) and fresh and not self.busy and
                             state['position_valid_mask'] == 63 and state['velocity_valid_mask'] == 63 and
                             state['core_state'] in (2, 4) and
                             all(math.isfinite(v) for v in state['estimated_velocity_rad_s']))
        self.copy.setEnabled(fresh and state['core_state'] != 9 and not getattr(self, 'readonly_fault', False) and hasattr(self, 'actual_q'))
        self.stop.setEnabled(self.worker.state() == QtCore.QProcess.Running and not self.helper_active())
        if hasattr(self, 'obstacle_tab') and (self.obstacle_tab.blocked or not self.obstacle_tab.scene_ready):
            self.move.setEnabled(False)
        if hasattr(self, 'motor_parameter_tab') and (self.motor_parameter_tab.pending or self.motor_parameter_tab.queue):
            for button in (self.zero, self.resume, self.recover, self.move):
                button.setEnabled(False)
        if hasattr(self, 'moveit_tab') and (self.moveit_tab.executing or self.moveit_tab.auto_test.active):
            self.busy = True
            for button in (self.move, self.zero, self.resume, self.recover, self.clear_fault):
                button.setEnabled(False)
        if hasattr(self, 'reconnect'):
            self.reconnect.setEnabled(not self.closing and not self.busy and not self.helper_active())
            self.clear_fault.setEnabled(not self.closing and self.fault_clear_available())
        if hasattr(self, 'backend_choice'):
            parameter_busy = hasattr(self, 'motor_parameter_tab') and (self.motor_parameter_tab.pending or self.motor_parameter_tab.queue)
            self.backend_choice.setEnabled(not self.closing and not self.busy and not self.helper_active() and self.connection_action is None and not parameter_busy)
        if not fresh:
            self.show_actual_description(False)
            self.status.setText('反馈未就绪 / 已失鲜；运动入口禁用。零点状态以设备同boot反馈为准，当前不授权运动。')
            return
        if not self.display_initialized:
            self.display_initialized = True
            self.anchor = self.optional_display_anchor(state)
            if self.anchor is not None:
                (self.report / 'display-only-Rest-anchor.json').write_text(json.dumps(self.anchor, indent=2))
        if self.previous_raw and any(abs(a - b) > 1.0 for a, b in zip(self.previous_raw, state['raw_motor_position_rev'])):
            self.anchor = None
        self.previous_raw = state['raw_motor_position_rev']
        q = None
        scope = '未标定'
        if state['position_valid_mask'] == 63 and state['reference_epoch'] != 0:
            q = state['position_rad']
            scope = '当前设备参考'
        elif self.anchor and state['boot_epoch'] == int(self.anchor['boot'], 16) and state['raw_position_valid_mask'] == 63:
            ##
            # 显示锚点用电机圈差×2π/减速比×方向恢复关节 rad；仅作为人工显示参考，后续控制仍须设备 reference 与 session。
            q = [h + j['direction'] * (m - a) * 2 * math.pi / j['gear_ratio']
                 for h, j, m, a in zip(self.anchor['home_rad'], self.cfg['joints'],
                                      state['raw_motor_position_rev'], self.anchor['motor_rev'])]
            scope = '人工确认姿态的显示参考（不授权运动）'
        self.show_actual_description(q is not None and all(math.isfinite(v) for v in q))
        if q is not None and all(math.isfinite(v) for v in q):
            self.actual_q = q
            for label, radians in zip(self.feedback_labels, q):
                label.setText(f'{math.degrees(radians):.3f}°')
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = [j['name'] for j in self.cfg['joints']]
            msg.position = q
            self.publisher.publish(msg)
        if state['core_state'] == 9 or getattr(self, 'readonly_fault', False):
            reference_note = '设备参考仍存在（同boot）' if state['reference_epoch'] != 0 else '当前没有有效设备参考'
            self.status.setText(f'真机-USB/Native · 实时反馈 · {reference_note} · 故障禁止控制，仅可停止或关闭只读连接；未自动清故障。 · boot={state["boot_epoch"]:016x}')
            return
        zero = '标定进行中' if self.zero_busy else ('设备参考存在（同boot）' if self.calibration else '未标定')
        zero += ' · 已使能控制' if self.authorized else ' · 未取得控制权限'
        speed_note = ''
        if self.calibration and self.authorized and state['core_state'] == 4 and not self.busy:
            speed_note = ' · 保持中；执行请求仍按原静止判据开始'
        resume_note = ''
        if resume_issue and self.manual_limits_retreat_target() is not None:
            resume_note = ' · 零点保留；明确恢复后仅危险的J2下侧/J3上侧以≤2°/s缓退工作Rest，其余轴冻结，不重新采零。'
        elif resume_issue and self.recovery_target() is not None:
            resume_note = ' · 零点保留；仅允许固定0.1°/s低速内收，需明确使能并点击低速回工作区。'
        elif resume_issue and self.wait_home and not self.authorized:
            resume_note = ' · 零点保留，实际位置不足停车余量，需支撑调整后恢复；无需重新归零。' + resume_issue
        feedback_source = '仿真-本地 · 运动学反馈' if getattr(self, 'backend', 'real') == 'simulator' else '真机-USB/Native · 实时反馈'
        self.status.setText(f'{feedback_source}{speed_note} · {zero} · {scope}{resume_note} · boot={state["boot_epoch"]:016x}')

    ##
    # @brief 有界等待管理/通信进程退出后才接受关闭窗口。
    # 先关闭管理 helper，再取消重启/标定意图；对运行 worker 发 RELEASE，处理事件最多 6 s，必要时 terminate 再等 6 s。仍在运行则拒绝关闭；不把窗口退出请求当作设备释放证明。
    # @param event Qt QCloseEvent；未能确认进程结束时 ignore 保持窗口。
    # @return None；执行上述状态更新、界面或日志操作。
    def ros_event(self, kind, value):
        self.message('ROS ' + kind + ': ' + str(value))
        if hasattr(self, 'moveit_tab'):
            self.moveit_tab.ros_event(kind, value)
        if hasattr(self, 'obstacle_tab'):
            self.obstacle_tab.ros_event(kind, value)
        if kind == 'failed':
            self.busy = False
            self.ros_runtime.fail(str(value))
        elif kind == 'manual_ready':
            self.busy = False
            self.ros_runtime.active()
        elif kind in ('released', 'canceled'):
            self.busy = False
        elif kind == 'accepted':
            self.busy = True
        elif kind == 'completed':
            self.busy = False

    def ros_status_received(self, status):
        self.ros_status = status
        self.ros_status_ns = time.monotonic_ns()
        self.ros_backend.latest_status = status
        self.ros_runtime.read_only()

    def ros_state_received(self, state):
        old = self.last_state
        if old and old['boot_epoch'] == state.boot_epoch and state.state_seq <= old['state_seq']:
            return
        self.ros_backend.latest_state = state
        # 只整理显示字段，不据此重建许可、完成或参考状态机。
        self.last_state = {name: getattr(state, name) for name in state.get_fields_and_field_types()}
        self.last_state_ns = time.monotonic_ns()
        if state.position_valid_mask == 63:
            self.actual_q = list(state.position_rad)
            for label, value in zip(self.feedback_labels, state.position_rad):
                label.setText(f'{math.degrees(value):.3f}°')

    def ros_tick(self):
        now = time.monotonic()
        self.ros_runtime.check_startup_timeout(now - self.ros_started_at, float(self.cfg.get('ros_startup_timeout_s', 30)))
        graph_error = self.ros_backend.transport.control_graph_error()
        if graph_error:
            self.show_actual_description(False)
            self.authorized = False
            self.status.setText(graph_error + '；禁止控制请求，未接管其他进程。')
            return
        ready = getattr(self.ros_backend.transport, 'ready', lambda *args: True)
        if ready() and now - self.ros_last_poll >= 0.5:
            self.ros_last_poll = now
            if self.ros_runtime.state == 'Releasing' and self.ros_backend.operation != 'release':
                self.ros_backend.release()
            else:
                self.ros_backend.poll()
        if self.ros_auto_pending and self.ros_status is not None and not self.ros_backend.pending and ready('auto_calibrate_and_test'):
            self.ros_auto_pending = False
            self.ros_backend.request_calibration_and_test()
        state, status = self.last_state, self.ros_status
        status_budget_ms = getattr(self.ros_backend.transport, 'status_max_age_ms', None)
        status_age_ns = time.monotonic_ns() - getattr(self, 'ros_status_ns', 0)
        status_fresh = bool(status is not None and status_budget_ms is not None and
                            0 <= status_age_ns <= status_budget_ms * 1_000_000)
        matched = bool(state and status and all(state[key] == getattr(status, key, None)
                       for key in ('boot_epoch', 'reference_epoch', 'session_id')))
        # ROS只读态没有JSB关节TF；数值反馈不能单独让完整URDF可见。
        # manual_ready或自动流程真实双Hold/Run均已通过C++控制器激活确认。
        manual_model_ready = (self.ros_backend.manual_ready and
            self.ros_runtime.state == 'Active' and not self.ros_backend.released)
        auto_model_ready = bool(self.ros_backend.operation == 'auto' and state and status and
            state['core_state'] in (4, 5) and status.core_state in (4, 5))
        positions = state.get('position_rad', ()) if state else ()
        self.show_actual_description(bool(matched and self.fresh() and status_fresh and
            state['position_valid_mask'] == 63 and state['reference_epoch'] != 0 and
            state['session_id'] != 0 and len(positions) == 6 and
            all(math.isfinite(q) for q in positions) and (manual_model_ready or auto_model_ready)))
        # 仅镜像权威快照用于UI准入；不申请或推导设备许可。
        self.authorized = bool(matched and self.fresh() and
            status_fresh and
            getattr(status, 'real_motion_permitted', False) and getattr(status, 'permit_remaining_us', 0) > 0 and
            state['session_id'] != 0 and state['position_valid_mask'] == 63)
        self.calibration = ((state['boot_epoch'], state['reference_epoch']) if matched and
                            state['reference_epoch'] != 0 and state['position_valid_mask'] == 63 else None)
        planning_busy = hasattr(self, 'moveit_tab') and (self.moveit_tab.executing or self.moveit_tab.auto_test.active)
        self.move.setEnabled(self.authorized and not self.busy and not self.closing and not planning_busy)
        if hasattr(self, 'obstacle_tab'):
            self.obstacle_tab.tick()
        if hasattr(self, 'moveit_tab'):
            self.moveit_tab.tick()
        for button in (self.recover, self.clear_fault):
            button.setEnabled(False)
        self.resume.setEnabled(bool(matched and self.fresh() and self.calibration and
            status_fresh and
            not self.busy and not self.closing and self.ros_backend.admitting and not self.ros_backend.released and
            state['core_state']==2 and status.core_state==2 and state['session_id']==0 and
            status.session_id==0 and not status.real_motion_permitted and status.permit_remaining_us==0))
        self.zero.setEnabled(status_fresh and not self.closing and not self.busy and self.ros_backend.admitting and not self.ros_backend.released)
        self.stop.setEnabled(not self.ros_backend.released)
        # ROS同样刷新连接控件，不能继承Native忙期的disabled状态。
        quiet = not self.closing and not self.busy and not self.helper_active()
        self.reconnect.setEnabled(quiet)
        parameter_busy = hasattr(self, 'motor_parameter_tab') and (self.motor_parameter_tab.pending or self.motor_parameter_tab.queue)
        self.backend_choice.setEnabled(quiet and self.connection_action is None and not parameter_busy)
        self.copy.setEnabled(self.fresh() and hasattr(self, 'actual_q'))
        if hasattr(self, 'motor_parameter_tab'):
            self.motor_parameter_tab.tick()
        status = self.ros_status
        identity = '' if status is None else f' · boot={status.boot_epoch:016x} · reference={status.reference_epoch}'
        budget_text = ' · 等待状态反馈配置（只读）' if status_budget_ms is None else f' · Status预算={status_budget_ms}ms'
        self.status.setText('真机-USB/ROS · ' + self.ros_runtime.state + identity + budget_text + (' · ' + self.ros_runtime.error if self.ros_runtime.error else ''))
        target = JointState()
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.name = [j['name'] for j in self.cfg['joints']]
        target.position = [math.radians(box.value()) for box in self.inputs]
        self.target_publisher.publish(target)

    def closeEvent(self, event):
        if self.backend == 'ros':
            self.closing = True
            if not self.ros_backend.released:
                self.release('RELEASE')
                deadline = time.monotonic() + float(self.cfg.get('ros_release_timeout_s', 6))
                while not self.ros_backend.released and time.monotonic() < deadline:
                    rclpy.spin_once(self.node, timeout_sec=0.02)
                    self.ros_tick()
                    QtWidgets.QApplication.processEvents()
            if not self.ros_runtime.stop(self.ros_backend.released):
                self.message(self.ros_runtime.error)
                self.closing = False
                event.ignore()
                return
            self.ros_backend.close()
            event.accept()
            return
        self.closing = True
        self.connection_action = None
        if self.helper_active():
            self.manage_worker.terminate()
            self.manage_worker.waitForFinished(6000)
            if self.helper_active():
                event.ignore()
                self.closing = False
                self.message('清故障进程尚未退出，窗口保持打开；未启动其他通信。')
                return
        self.pending_home = self.restart_after_release = False
        self.pending_home_generation = None
        if self.worker.state() == QtCore.QProcess.Running:
            self.release('RELEASE')
            deadline = time.monotonic() + 6
            while self.worker.state() == QtCore.QProcess.Running and time.monotonic() < deadline:
                self.worker.waitForFinished(20)
                QtWidgets.QApplication.processEvents()
            if self.worker.state() == QtCore.QProcess.Running:
                self.worker.terminate()
                self.worker.waitForFinished(6000)
            if self.worker.state() == QtCore.QProcess.Running:
                event.ignore()
                self.closing = False
                self.message('释放尚未确认，窗口保持打开；请检查现场保护。')
                return
        event.accept()


##
# @brief 加载配置并建立 Qt/ROS 面板主事件循环。
# 解析参数前移除 ROS 参数；real 加载部署解码器，simulator 不加载。关闭 rclpy 自动信号处理，SIGINT/SIGTERM 排队调用 panel.close，使同一释放流程执行；finally 停定时器、销毁节点和 ROS 上下文。
# @return int：Qt app.exec_() 返回的进程退出码。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--description-xacro', default='')
    parser.add_argument('--backend', choices=('real', 'ros', 'simulator'), default='real')
    parser.add_argument('--auto-calibrate-and-test', action='store_true')
    parser.add_argument('--initial-pose', choices=('manual_limits',), default='manual_limits')
    parser.add_argument('--moveit', action='store_true', help='启用末端规划页；另启动仅规划 MoveIt 服务')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    from manual_panel_config import load_decoder
    from manual_panel_runtime import load_runtime_panel_config as load_panel_config
    try:
        config = load_panel_config(Path(args.config), args.backend, args.description_xacro)
        _decode = load_decoder(config) if args.backend == 'real' else None
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    config['auto_calibrate_and_test'] = args.auto_calibrate_and_test
    config['initial_pose'] = args.initial_pose
    config['moveit_enabled'] = args.moveit
    config['backend'] = args.backend
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    rclpy.init(args=[], signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node('dummy_manual_joint_panel')
    panel = JointPanel(config, Path(args.report), _decode, node)
    signal.signal(signal.SIGTERM, lambda *_: QtCore.QTimer.singleShot(0, panel.close))
    signal.signal(signal.SIGINT, lambda *_: QtCore.QTimer.singleShot(0, panel.close))
    panel.show()
    try:
        return app.exec_()
    finally:
        panel.timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
