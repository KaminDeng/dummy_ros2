#!/usr/bin/env python3
"""ROS 薄客户端：不含设备许可、续租、完成或标定状态机。"""
import copy
import time
import uuid


# Count actual DDS service/publisher instances, never node-name list entries.
CONTROL_SERVICES = ('/device_action_gate/bootstrap_status',
                    '/device_action_gate/disable_and_release',
                    '/device_action_gate/prepare_manual_control',
                    '/device_action_gate/manual_control_status',
                    '/manual_ros_baseline/get_parameters',
                    '/controller_manager/list_controllers')
CONTROL_TOPICS = ('/device_action_gate/device_status', '/device_action_gate/device_state')
# 只有无副作用的Trigger查询可退休超时回包；管理写请求绝不自动重发。
READONLY_TRIGGER_SERVICES = frozenset(('bootstrap_status', 'manual_control_status',
    'baseline_status', 'manual_reference_status', 'last_goal_rejection'))

def confirmed_ros_launch_argv(argv, confirmed):
    """只传递当次明确姿态确认；连接本身仍不触发标定或使能。"""
    result = []
    inserted = False
    flag = 'operator_confirmed_double_limits:=' + ('true' if confirmed else 'false')
    for argument in argv:
        if argument.startswith('operator_confirmed_double_limits:='):
            if not inserted:
                result.append(flag)
                inserted = True
        else:
            result.append(argument)
    if not inserted:
        result.append(flag)
    return result


class RosBackend:
    def __init__(self, transport, callback):
        self.transport, self.callback = transport, callback
        self.released = False
        self.admitting = False
        self.pending = False
        self.operation = None
        self.release_pending = False
        self._release_deadline = None
        self._release_retry_at = 0.0
        self._release_timeout_reported = False
        self.manual_ready = False
        self.latest_status = None
        self.latest_state = None

    def connect(self):
        self.admitting = True
        self.callback('connected', 'ReadOnly')

    def _request(self, service, callback):
        if self.pending:
            return False
        self.pending = True
        def received(response):
            self.pending = False
            if isinstance(response, Exception):
                self.callback('failed', str(response))
            else:
                callback(response)
        self.transport.request(service, received)
        return True

    def _motion_ready(self):
        # 只读C++公布的SDK预算；尚未读取时不开运动请求。
        return self.admitting and getattr(self.transport, 'status_max_age_ms', None) is not None

    def request_calibration_and_test(self):
        if not self._motion_ready():
            return False
        self.operation = 'auto'
        return self._request('auto_calibrate_and_test', lambda response:
            self.callback('accepted' if response.success else 'failed', response.message))

    def request_prepare_manual_control(self):
        if not self._motion_ready() or self.released:
            return False
        self.operation = 'manual'
        return self._request('prepare_manual_control', lambda response:
            self.callback('accepted' if response.success else 'failed', response.message))

    def submit_trajectory(self, trajectory):
        if not self._motion_ready():
            return False
        return self.transport.submit_trajectory(trajectory, self.callback)

    def cancel(self):
        return self.transport.cancel()

    def _release_expired(self):
        if self._release_timeout_reported:
            return True
        if (self.operation == 'release' and self._release_deadline is not None
                and time.monotonic() >= self._release_deadline):
            self._release_timeout_reported = True
            self.release_pending = False
            self.operation = None
            self.callback('failed', 'RELEASE_UNCONFIRMED: release deadline expired')
            return True
        return False

    def release(self):
        self.admitting = False
        getattr(self.transport, 'cancel_pending_submission', lambda: None)()
        # Repeated clicks retain the original intent/deadline; QUEUED is not
        # permission to send another D10, and timeout never closes the owner.
        if self._release_deadline is not None:
            if self._release_expired():
                return False
            return self._send_release() if self.release_pending else True
        self.released = False
        self.operation = 'release'
        self._release_deadline = time.monotonic() + 20.0
        self.release_pending = True
        return self._send_release()

    def _send_release(self):
        if self._release_expired() or self.pending or time.monotonic() < self._release_retry_at:
            return False
        self.release_pending = False
        def received(response):
            if self._release_expired():
                return
            if response.message == 'RELEASE_UNAVAILABLE':
                self.release_pending = True
                self._release_retry_at = time.monotonic() + 0.2
                self.callback('progress', response.message)
            else:
                self.callback('progress' if response.message == 'RELEASE_QUEUED' or response.success else 'failed', response.message)
        return self._request('disable_and_release', received)

    def poll(self):
        expired = self._release_expired()
        if self.release_pending and not expired:
            return self._send_release()
        def status(response):
            message = response.message
            release_intent = not self.released and (self.operation == 'release' or self._release_deadline is not None)
            physical_release = ((response.success and message in ('RELEASED_IDLE', 'FAULT_CLEANUP_CONFIRMED')) or
                                message == 'EVIDENCE_INCOMPLETE;RELEASED_IDLE')
            # 原释放期限只封锁写请求；迟到的权威只读确认仍可退休owner，原证据错误不改成成功。
            if release_intent and physical_release:
                self.released = True
                self.release_pending = False
                self.operation = None
                self.callback('released', message)
            elif release_intent and self._release_expired():
                self.callback('progress', message)
            elif self.operation == 'manual' and response.success and message.startswith('MANUAL_CONTROL_READY'):
                if not self.manual_ready:
                    self.manual_ready = True
                    self.callback('manual_ready', message)
            elif self.operation == 'manual' and response.success and message.startswith('MANUAL_CONTROL_RELEASED'):
                self.released, self.admitting, self.operation = True, False, None
                self.callback('released', message)
            elif self.operation == 'manual' and message.startswith('MANUAL_CONTROL_FAILED:'):
                self.admitting = False
                self.released = message.endswith('cleanup=RELEASED_IDLE')
                self.operation = None
                self.callback('failed', message)
            elif response.success and (message == 'BASELINE_PASSED' or message.startswith('BASELINE_PASSED ')) and self.operation == 'auto':
                self.operation = None
                self.released, self.admitting = True, False
                self.callback('completed', message)
            elif self.operation == 'auto' and message.startswith('BASELINE_FAILED:'):
                self.admitting = False
                self.released = message.endswith('cleanup=RELEASED_IDLE')
                self.operation = None
                self.callback('failed', message)
            else:
                self.callback('progress', message)
        return self._request('bootstrap_status' if self._release_deadline is not None else 'baseline_status' if self.operation == 'auto' else 'manual_control_status' if self.operation == 'manual' else 'bootstrap_status', status)

    def close(self):
        if not self.released:
            return False
        self.transport.close()
        return True


class RosTransport:
    """依赖在实例化时才导入，便于纯 Python 假服务测试。"""
    def __init__(self, node, on_status, on_state):
        from std_srvs.srv import Trigger
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterType
        from dpsdk_interfaces.msg import DummyDeviceStatusV2, DummyJointStateV2
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from unique_identifier_msgs.msg import UUID
        self.node = node
        self.trigger = Trigger
        self.get_parameters_type = GetParameters
        self.parameter_integer_type = ParameterType.PARAMETER_INTEGER
        self.parameter_client = node.create_client(GetParameters, '/manual_ros_baseline/get_parameters')
        self.status_max_age_ms = None
        self.feedback_max_age_ms = None
        self.parameter_pending = False
        self.parameter_generation = 0
        self._readonly_requests = {}
        self.parameter_future = None
        self.parameter_retry_at = 0.
        self.closed = False
        self.action_type = FollowJointTrajectory
        self.clients = {name: node.create_client(Trigger, '/device_action_gate/' + name) for name in (
            'auto_calibrate_and_test', 'prepare_manual_control', 'manual_control_status', 'manual_reference_status', 'baseline_status',
            'disable_and_release', 'bootstrap_status', 'last_goal_rejection')}
        self.uuid_type = UUID
        self.latest_state = self.latest_status = None
        self.state_at = self.status_at = 0.
        self._submission = None
        self.retry_timer = node.create_timer(.05, self._poll_trajectory_retry)
        def status_received(value):
            if self.control_graph_error():
                return
            self.latest_status, self.status_at = value, time.monotonic()
            on_status(value)
        def state_received(value):
            if self.control_graph_error():
                return
            old = self.latest_state
            if old is not None and old.boot_epoch == value.boot_epoch and value.state_seq <= old.state_seq:
                return
            self.latest_state, self.state_at = value, time.monotonic()
            on_state(value)
        self.subscriptions = [node.create_subscription(DummyDeviceStatusV2,
            '/device_action_gate/device_status', status_received, 10),
            node.create_subscription(DummyJointStateV2, '/device_action_gate/device_state', state_received, 10)]
        self.action = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.goal = None

    def control_graph_error(self):
        if getattr(self, '_startup_owner_conflict', ''):
            return self._startup_owner_conflict
        services = [(name, self.node.count_services(name)) for name in CONTROL_SERVICES]
        publishers = [(name, self.node.count_publishers(name)) for name in CONTROL_TOPICS]
        invalid = [(name, count) for name, count in services + publishers if count != 1]
        return ('ROS控制链缺失或存在重复实例：' + ', '.join(name+'='+str(count) for name, count in invalid)) if invalid else ''

    def assert_no_existing_owner(self, timeout=2.):
        started = time.monotonic()
        deadline = started + timeout
        discovery_deadline = started + min(.5, timeout)
        while True:
            existing = [(name, self.node.count_services(name)) for name in CONTROL_SERVICES]
            existing += [(name, self.node.count_publishers(name)) for name in CONTROL_TOPICS]
            existing = [(name, count) for name, count in existing if count]
            now = time.monotonic()
            remaining = deadline - now
            if not existing and now >= discovery_deadline:
                # 只有本次原发现期后实际全图为空才解除旧冲突；点击本身不解除。
                self._startup_owner_conflict = ''
                return
            if remaining <= 0:
                self._startup_owner_conflict = '已有ROS控制链，拒绝创建或接管：' + ', '.join(name+'='+str(count) for name, count in existing)
                raise RuntimeError(self._startup_owner_conflict)
            # 空图保留原半秒发现期；旧DDS端点消退可等到同一次固定两秒截止。
            # 图由DDS独立更新，不spin_once或重入Qt业务，也不发送清理请求。
            wait = min(.02, remaining)
            if now < discovery_deadline:
                wait = min(wait, discovery_deadline - now)
            time.sleep(wait)

    def refresh_status_budget(self):
        now = time.monotonic()
        if self.closed or self.status_max_age_ms is not None:
            return
        if self.parameter_pending:
            if now < self.parameter_retry_at:
                return
            # 先退休旧世代，再取消Future；取消可能同步触发旧回调。
            self.parameter_generation = getattr(self, 'parameter_generation', 0) + 1
            self.parameter_pending = False
            previous = getattr(self, 'parameter_future', None)
            self.parameter_future = None
            if previous is not None:
                previous.cancel()
        if now < self.parameter_retry_at or not self.parameter_client.service_is_ready():
            return
        self.parameter_pending = True
        self.parameter_retry_at = now + 1.
        self.parameter_generation = getattr(self, 'parameter_generation', 0) + 1
        generation = self.parameter_generation
        request = self.get_parameters_type.Request()
        request.names = ['status_max_age_ms', 'feedback_max_age_ms']
        def done(future):
            if self.closed or generation != self.parameter_generation:
                return
            self.parameter_pending = False
            self.parameter_future = None
            try:
                values = future.result().values
                if len(values) == 2 and all(v.type == self.parameter_integer_type and v.integer_value > 0 for v in values):
                    self.status_max_age_ms, self.feedback_max_age_ms = (v.integer_value for v in values)
            except Exception:
                # 失败保持只读，不造Python默认预算；后续tick可重试。
                pass
        try:
            self.parameter_future = self.parameter_client.call_async(request)
            self.parameter_future.add_done_callback(done)
        except Exception:
            self.parameter_pending = False
            self.parameter_future = None

    def ready(self, service='bootstrap_status'):
        if self.control_graph_error():
            return False
        self.refresh_status_budget()
        if service in ('auto_calibrate_and_test', 'prepare_manual_control') and self.status_max_age_ms is None:
            return False
        return self.clients[service].service_is_ready()

    def request(self, service, callback):
        if self.closed:
            return
        error = self.control_graph_error()
        if error:
            callback(RuntimeError(error))
            return
        client = self.clients[service]
        if not client.service_is_ready():
            callback(RuntimeError('ROS 服务尚未就绪：' + service))
            return
        future = client.call_async(self.trigger.Request())
        token = object() if service in READONLY_TRIGGER_SERVICES else None
        context = None
        if token is not None:
            context = dict(future=future, deadline=time.monotonic()+1.,
                           callback=callback, service=service)
            self._readonly_requests[token] = context
        def done(result):
            if self.closed:
                return
            if token is not None and self._readonly_requests.pop(token, None) is not context:
                return
            try:
                response = result.result()
            except Exception as error:
                response = error
            callback(response)
        future.add_done_callback(done)

    def _expire_readonly_requests(self):
        if self.closed:
            return
        now = time.monotonic()
        for token, context in list(self._readonly_requests.items()):
            if now < context['deadline']:
                continue
            # 先退休本次Future，再取消；同步或迟到回调均不能覆盖新的请求。
            del self._readonly_requests[token]
            context['future'].cancel()
            context['callback'](self.trigger.Response(success=False,
                message='READONLY_REQUEST_TIMEOUT:' + context['service']))

    def _fresh_retry_binding(self):
        state, status = getattr(self, 'latest_state', None), getattr(self, 'latest_status', None)
        state_age, status_age = getattr(self, 'feedback_max_age_ms', None), getattr(self, 'status_max_age_ms', None)
        if state is None or status is None or state_age is None or status_age is None:
            return None
        now = time.monotonic()
        if not (0 <= now-self.state_at <= state_age/1000. and 0 <= now-self.status_at <= status_age/1000.):
            return None
        fields = ('profile_id', 'boot_epoch', 'session_id', 'reference_epoch', 'motion_epoch')
        if tuple(state.device_id) != tuple(status.device_id) or any(getattr(state, key) != getattr(status, key) for key in fields):
            return None
        if (not state.boot_epoch or not state.session_id or not state.reference_epoch or
                state.core_state != 4 or status.core_state != 4 or not state.hold_ready or
                state.position_valid_mask != 63 or state.velocity_valid_mask != 63 or
                any(age > state_age*1000 for age in state.can_rx_age_us) or
                not status.real_motion_permitted or status.permit_remaining_us <= 0 or status.evidence_lost):
            return None
        return (tuple(state.device_id), state.profile_id, state.boot_epoch, state.session_id,
                state.reference_epoch, state.motion_epoch)

    def _submission_failed(self, context, reason):
        if self._submission is context:
            self._submission = None
            context['callback']('failed', reason)

    def cancel_pending_submission(self):
        context = getattr(self, '_submission', None)
        if context is not None and context['stage'] != 'active':
            self._submission_failed(context, 'ROS目标尚未受理；已撤销待重试请求')

    def _poll_trajectory_retry(self):
        self._expire_readonly_requests()
        context = getattr(self, '_submission', None)
        if self.closed or context is None or context['stage'] not in ('preflight', 'receipt', 'renewal'):
            return
        if time.monotonic() >= context['retry_deadline']:
            self._submission_failed(context, 'ROS续租拒绝回执/重试原2秒期限已到')
            return
        if context['rpc_pending']:
            return
        context['rpc_pending'] = True
        stage = context['stage']
        def response(value):
            if self._submission is not context or self.closed:
                return
            context['rpc_pending'] = False
            if time.monotonic() >= context['retry_deadline']:
                self._submission_failed(context, 'ROS续租重试原2秒期限已到')
                return
            if isinstance(value, Exception):
                self._submission_failed(context, str(value))
            elif stage == 'preflight':
                state, status = self.latest_state, self.latest_status
                if state is None or status is None:
                    self._submission_failed(context, 'ROS起步缺少原绑定反馈')
                    return
                identity = (tuple(state.device_id), state.profile_id, state.boot_epoch,
                            state.session_id, state.reference_epoch, state.motion_epoch)
                if identity != context['binding']:
                    self._submission_failed(context, 'ROS起步等待的原身份已改变')
                    return
                if (value.success and value.message in ('ENABLED_HOLD', 'LEASE_RENEWED') and
                        self._fresh_retry_binding() == context['binding'] and
                        state.management_epoch == status.management_epoch):
                    context['management'] = state.management_epoch
                    context['state_seq'] = state.state_seq
                    context['status_at'] = self.status_at
                    self._send_trajectory_attempt(context)
            elif stage == 'receipt':
                if value.success and value.message == 'RENEWAL_PENDING:' + context['uuid'] and context['attempts'] == 1:
                    context['stage'] = 'renewal'
                else:
                    self._submission_failed(context, 'ROS轨迹被拒绝，未匹配本UUID续租回执')
            else:
                binding = self._fresh_retry_binding()
                state, status = self.latest_state, self.latest_status
                if binding is None or binding != context['binding']:
                    self._submission_failed(context, 'ROS续租等待的原身份/新鲜Hold绑定已失效')
                    return
                if any(((epoch-context['management']) & 0xffffffff) not in (0, 1)
                       for epoch in (state.management_epoch, status.management_epoch)):
                    self._submission_failed(context, 'ROS续租等待管理版本不匹配')
                    return
                # auto Renew可直接ENABLED_HOLD；双流管理版本异步到达时只等待收敛。
                if (value.success and value.message in ('ENABLED_HOLD', 'LEASE_RENEWED') and
                        state.management_epoch == status.management_epoch and
                        state.state_seq > context['state_seq'] and self.status_at > context['status_at']):
                    self._send_trajectory_attempt(context)
        self.request('last_goal_rejection' if stage == 'receipt' else 'bootstrap_status', response)

    def submit_trajectory(self, trajectory, callback):
        error = self.control_graph_error()
        if error:
            callback('failed', error)
            return False
        if self.closed or getattr(self, '_submission', None) is not None or not self.action.server_is_ready():
            callback('failed', 'ROS轨迹Action尚未就绪或已有目标')
            return False
        context = dict(trajectory=copy.deepcopy(trajectory), callback=callback, attempts=0,
                       binding=self._fresh_retry_binding(),
                       management=getattr(getattr(self, 'latest_state', None), 'management_epoch', 0),
                       state_seq=getattr(getattr(self, 'latest_state', None), 'state_seq', 0),
                       status_at=getattr(self, 'status_at', 0.),
                       retry_deadline=time.monotonic()+2., stage='sending', rpc_pending=False)
        state = self.latest_state
        if state is None:
            callback('failed', 'ROS起步缺少原绑定State')
            return False
        context['binding'] = (tuple(state.device_id), state.profile_id, state.boot_epoch,
                              state.session_id, state.reference_epoch, state.motion_epoch)
        context['stage'] = 'preflight'
        self._submission = context
        return True

    def _send_trajectory_attempt(self, context):
        error = self.control_graph_error()
        if error:
            self._submission_failed(context, error)
            return
        context['attempts'] += 1
        context['stage'] = 'sending'
        explicit_uuid = self.uuid_type(uuid=list(uuid.uuid4().bytes))
        context['uuid'] = bytes(explicit_uuid.uuid).hex()
        goal = self.action_type.Goal()
        goal.trajectory = context['trajectory']
        def accepted(future):
            try:
                handle = future.result()
                if self._submission is not context or self.closed:
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    self.goal = None  # 被拒handle不能取消，也不能发accepted事件。
                    context['stage'] = 'receipt'
                    return
                self.goal = handle
                context['stage'] = 'active'
                goal_uuid = bytes(handle.goal_id.uuid).hex()
                context['callback']('accepted', {'goal_uuid': goal_uuid})
                def finished(future):
                    if self._submission is not context or self.closed:
                        return
                    self._submission = None
                    self.goal = None
                    try:
                        result = future.result()
                        ok = result.status == 4 and result.result.error_code == 0
                        if result.status == 5:
                            context['callback']('canceled', {'goal_uuid': goal_uuid, 'status': 5})
                        else:
                            context['callback']('completed' if ok else 'failed', result.result.error_string)
                    except Exception as error:
                        context['callback']('failed', str(error))
                handle.get_result_async().add_done_callback(finished)
            except Exception as error:
                self._submission_failed(context, str(error))
        try:
            self.action.send_goal_async(goal, goal_uuid=explicit_uuid,
                feedback_callback=lambda feedback: context['callback']('progress', feedback.feedback)
                if self._submission is context and context['stage'] == 'active' else None).add_done_callback(accepted)
        except Exception as error:
            self._submission_failed(context, str(error))

    def cancel(self):
        self.cancel_pending_submission()
        if self.goal is not None:
            return self.goal.cancel_goal_async()
        return None

    def close(self):
        self.cancel_pending_submission()
        self.closed = True
        # 关闭只销毁查询Future，不产生终态或回调到已关闭界面。
        pending = list(self._readonly_requests.values())
        self._readonly_requests.clear()
        for context in pending:
            context['future'].cancel()
        self.node.destroy_timer(self.retry_timer)
        self.node.destroy_client(self.parameter_client)
        self.action.destroy()
        for client in self.clients.values():
            self.node.destroy_client(client)
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
