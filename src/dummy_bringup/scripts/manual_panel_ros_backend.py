#!/usr/bin/env python3
"""ROS 薄客户端：不含设备许可、续租、完成或标定状态机。"""
import copy
import math
import time
import uuid


# Count actual DDS service/publisher instances, never node-name list entries.
CONTROL_SERVICES = ('/device_action_gate/bootstrap_status',
                    '/device_action_gate/set_bench_permit',
                    '/device_action_gate/disable_and_release',
                    '/device_action_gate/prepare_manual_control',
                    '/device_action_gate/manual_control_status',
                    '/manual_ros_baseline/get_parameters',
                    '/controller_manager/list_controllers')
CONTROL_TOPICS = ('/device_action_gate/device_status', '/device_action_gate/device_state')
# 只有无副作用的Trigger查询可退休超时回包；已入队或结果不明的管理写请求不重发。
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
        self.local_retired = False
        self.admitting = False
        self.pending = False
        self.readonly_query_pending = False
        self._management_generation = 0
        self.operation = None
        self.release_pending = False
        self._release_deadline = None
        self._release_retry_at = 0.0
        self._release_timeout_reported = False
        self.manual_ready = False
        self._cleanup_retirement_pending = False
        self.latest_status = None
        self.latest_state = None
        self._release_observation = None
        self._release_observed_latched = False

    def connect(self):
        self.admitting = True
        self.callback('connected', 'ReadOnly')

    @property
    def can_close(self):
        if self.local_retired:
            return True
        if not self.released:
            return False
        if not getattr(self.transport, 'owns_control_node', False):
            return True
        # 先以新鲜双消息取证；随后等自有PGID退出时原发布者会消失，不能再续鲜。
        if not self._release_observed_latched:
            self._release_observed_latched = self.transport.release_observed(self._release_observation)
        return self._release_observed_latched

    def _accept_local_retirement(self, response):
        # 只消费用户退出意图下的精确C++回执；不把本地只读退出写成设备失能。
        if (self.can_close or self._release_deadline is None or
                response.message != 'LOCAL_READONLY_RETIRED'):
            return False
        self.local_retired = True
        self.admitting = self.manual_ready = self.release_pending = False
        self._cleanup_retirement_pending = False
        self.operation = None
        self.callback('retired', response.message)
        return True

    def _request(self, service, callback, *, operation=None):
        readonly = service in READONLY_TRIGGER_SERVICES
        # 独立client允许既有只读查询与一次明确管理请求并行；不启动第二次查询或管理。
        if self.pending or (readonly and self.readonly_query_pending):
            return False
        if readonly:
            self.readonly_query_pending = True
        else:
            self.pending = True
            self._management_generation += 1
            if operation is not None:
                self.operation = operation
        generation = self._management_generation
        def received(response):
            if readonly:
                self.readonly_query_pending = False
                # 旧查询可退休自己的槽，不能清管理单飞或解释新操作的状态。
                if generation != self._management_generation:
                    return
            else:
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
        return self._request('auto_calibrate_and_test', lambda response:
            self.callback('accepted' if response.success else 'failed', response.message), operation='auto')

    def request_prepare_manual_control(self):
        if not self._motion_ready() or self.released:
            return False
        return self._request('prepare_manual_control', lambda response:
            self.callback('accepted' if response.success else 'failed', response.message), operation='manual')

    def submit_trajectory(self, trajectory, task_budget_s=None):
        if not self._motion_ready():
            return False
        if task_budget_s is None:
            return self.transport.submit_trajectory(trajectory, self.callback)
        return self.transport.submit_trajectory(trajectory, self.callback, task_budget_s=task_budget_s)

    def extend_trajectory(self, trajectory, expected_version, unchanged_through_ns):
        if not self._motion_ready() or self.released:
            return False
        return self.transport.extend_trajectory(trajectory, expected_version,
            unchanged_through_ns, self.callback)

    def cancel(self):
        return self.transport.cancel()

    def _release_expired(self):
        if self._release_timeout_reported:
            return True
        waiting_double = (self.released and getattr(self.transport, 'owns_control_node', False)
                          and not self.can_close)
        if ((self.operation == 'release' or waiting_double) and self._release_deadline is not None
                and time.monotonic() >= self._release_deadline):
            self._release_timeout_reported = True
            self.release_pending = False
            self.operation = None
            self.callback('failed', 'RELEASE_UNCONFIRMED: release deadline expired')
            return True
        return False

    def _observe_release(self, *, terminal=False):
        if getattr(self.transport, 'owns_control_node', False):
            if self._release_observation is None:
                self._release_observation = self.transport.release_checkpoint()
            if terminal and self._release_deadline is None:
                # 设备流程自行释放的首个终态没有GUI释放请求，建立一次只读确认预算。
                self._release_deadline = time.monotonic() + 20.0

    def release(self):
        if self.can_close:
            return True
        self._observe_release(terminal=self.released)
        if self.released:
            return not self._release_expired()  # 仅等双消息，不续期或再发D10。
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
            if self._accept_local_retirement(response):
                return
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
            if self._accept_local_retirement(response):
                return
            message = response.message
            release_intent = not self.can_close and (self.operation == 'release' or self._release_deadline is not None or self._cleanup_retirement_pending)
            physical_release = ((response.success and message in ('RELEASED_IDLE', 'RELEASED_FAULT', 'FAULT_CLEANUP_CONFIRMED')) or
                                message in ('EVIDENCE_INCOMPLETE;RELEASED_IDLE', 'EVIDENCE_INCOMPLETE;RELEASED_FAULT'))
            # 原释放期限只封锁写请求；迟到的权威只读确认仍可退休owner，原证据错误不改成成功。
            if release_intent and physical_release:
                already_released = self.released
                self._observe_release(terminal=True)
                self.released = True
                self.manual_ready = False
                self.admitting = False
                self.release_pending = False
                self._cleanup_retirement_pending = False
                self.operation = None
                self.callback('progress' if already_released else 'released', message)
            elif release_intent and self._release_expired():
                self.callback('progress', message)
            elif self.operation == 'manual' and response.success and message.startswith('MANUAL_CONTROL_READY'):
                if not self.manual_ready:
                    self.manual_ready = True
                    self.callback('manual_ready', message)
            elif self.operation == 'manual' and response.success and message.startswith('MANUAL_CONTROL_RELEASED'):
                self._observe_release(terminal=True)
                self.released, self.admitting, self.operation = True, False, None
                self.callback('released', message)
            elif self.operation == 'manual' and message.startswith('MANUAL_CONTROL_FAILED:'):
                self.admitting = False
                self.released = message.endswith((' cleanup=RELEASED_IDLE', ' cleanup=RELEASED_FAULT'))
                self.manual_ready = False
                # 本次C++任务已主动进入cleanup；终态错误保留，只等后续权威只读释放确认。
                self._cleanup_retirement_pending = message.endswith(' cleanup=UNCONFIRMED')
                self.operation = None
                self.callback('failed', message)
            elif response.success and (message == 'BASELINE_PASSED' or message.startswith('BASELINE_PASSED ')) and self.operation == 'auto':
                self.operation = None
                self.released, self.admitting = True, False
                self.callback('completed', message)
            elif self.operation == 'auto' and message.startswith('BASELINE_FAILED:'):
                self.admitting = False
                self.released = message.endswith((' cleanup=RELEASED_IDLE', ' cleanup=RELEASED_FAULT'))
                self.manual_ready = False
                # 本次C++任务已主动进入cleanup；终态错误保留，只等后续权威只读释放确认。
                self._cleanup_retirement_pending = message.endswith(' cleanup=UNCONFIRMED')
                self.operation = None
                self.callback('failed', message)
            else:
                self.callback('progress', message)
        return self._request('bootstrap_status' if self._release_deadline is not None else 'baseline_status' if self.operation == 'auto' else 'manual_control_status' if self.operation == 'manual' else 'bootstrap_status', status)

    def close(self):
        if not self.can_close:
            return False
        self.transport.close()
        return True


class RosTransport:
    """依赖在实例化时才导入，便于纯 Python 假服务测试。"""
    def __init__(self, node, on_status, on_state, *, own_control_node=False):
        self.owns_control_node = own_control_node
        self._control_context = self._control_executor = self._control_node = None
        self.closed = False
        try:
            if own_control_node:
                import rclpy
                from rclpy.context import Context
                from rclpy.executors import SingleThreadedExecutor
                self._control_context = Context()
                # Context.init不安装全局signal handlers，也不替换GUI默认Context。
                self._control_context.init(args=[], domain_id=node.context.get_domain_id(),
                                           initialize_logging=False)
                self._control_node = rclpy.create_node('dummy_control_observer_' + uuid.uuid4().hex[:12],
                    context=self._control_context, namespace=node.get_namespace())
                self._control_executor = SingleThreadedExecutor(context=self._control_context)
                self._control_executor.add_node(self._control_node)
                node = self._control_node
            self._init_entities(node, on_status, on_state)
        except BaseException:
            self.closed = True
            self._close_owned_control()
            raise

    def _close_owned_control(self):
        node, executor, context = (getattr(self, name, None) for name in
            ('_control_node', '_control_executor', '_control_context'))
        self._control_node = self._control_executor = self._control_context = None
        try:
            if node is not None:
                if executor is not None:
                    executor.remove_node(node)
                node.destroy_node()
        finally:
            try:
                if executor is not None:
                    executor.shutdown(timeout_sec=0.)
            finally:
                if context is not None:
                    context.try_shutdown()
                    context.destroy()

    def pump(self):
        if not self.closed and self._control_executor is not None:
            self._control_executor.spin_once(timeout_sec=0)

    def release_checkpoint(self):
        state = self.latest_state
        return dict(at=time.monotonic(), seq=getattr(state, 'state_seq', 0),
            identity=None if state is None else tuple(
                tuple(getattr(state, k)) if k == 'device_id' else getattr(state, k)
                for k in ('device_id', 'profile_id', 'boot_epoch', 'reference_epoch')))

    def release_observed(self, checkpoint):
        if checkpoint is None or checkpoint['identity'] is None:
            return False
        state, status = self.latest_state, self.latest_status
        now = time.monotonic()
        if state is None or status is None or self.feedback_max_age_ms is None or self.status_max_age_ms is None:
            return False
        if not (checkpoint['at'] < self.state_at <= now and checkpoint['at'] < self.status_at <= now and
                now-self.state_at <= self.feedback_max_age_ms/1000 and
                now-self.status_at <= self.status_max_age_ms/1000):
            return False
        for msg in (state, status):
            identity = tuple(tuple(getattr(msg, k)) if k == 'device_id' else getattr(msg, k)
                for k in ('device_id', 'profile_id', 'boot_epoch', 'reference_epoch'))
            if (identity != checkpoint['identity'] or msg.session_id != 0 or msg.core_state != 2 or
                    msg.first_failure_reason):
                return False
        return (state.state_seq > checkpoint['seq'] and status.release_committed and
                not status.real_motion_permitted and status.permit_remaining_us == 0 and not status.evidence_lost)

    def _init_entities(self, node, on_status, on_state):
        from std_srvs.srv import Trigger
        from rcl_interfaces.srv import GetParameters
        from rcl_interfaces.msg import ParameterType
        from dpsdk_interfaces.msg import DummyDeviceStatusV2, DummyJointStateV2
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from unique_identifier_msgs.msg import UUID
        from dummy_motion_interfaces.srv import ExtendJointTrajectory
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
            'disable_and_release', 'bootstrap_status', 'last_goal_rejection', 'set_bench_permit')}
        self.extension_type = ExtendJointTrajectory
        self.extension_client = node.create_client(ExtendJointTrajectory,
            '/device_action_gate/extend_trajectory')
        self._extension = None
        self.uuid_type = UUID
        self.latest_state = self.latest_status = None
        self.state_at = self.status_at = 0.
        self._submission = None
        self.retry_timer = node.create_timer(.05, self._poll_trajectory_retry)
        def status_received(value):
            if self.closed or self.control_graph_error():
                return
            self.latest_status, self.status_at = value, time.monotonic()
            on_status(value)
        def state_received(value):
            if self.closed or self.control_graph_error():
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

    def _fresh_retry_binding(self, require_permit=True):
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
                (require_permit and (not status.real_motion_permitted or status.permit_remaining_us <= 0)) or status.evidence_lost):
            return None
        return (tuple(state.device_id), state.profile_id, state.boot_epoch, state.session_id,
                state.reference_epoch, state.motion_epoch)

    def _permit_covers_trajectory(self, context):
        try:
            duration_us = context['required_permit_us']
            elapsed = max(0., time.monotonic()-self.status_at)*1000000
            # 向上取整接收年龄，不把旧Status里的remaining当作此刻预算。
            elapsed_us = int(elapsed) + int(elapsed > int(elapsed))
            return duration_us > 0 and self.latest_status.permit_remaining_us-elapsed_us >= duration_us
        except (AttributeError, IndexError, TypeError):
            return False

    def _submission_failed(self, context, reason):
        if self._submission is context:
            self._submission = None
            context['callback']('failed', reason)

    def cancel_pending_submission(self):
        self._retire_extension()
        context = getattr(self, '_submission', None)
        if context is not None and context['stage'] != 'active':
            self._submission_failed(context, 'ROS目标尚未受理；已撤销待重试请求')

    def _poll_trajectory_retry(self):
        self._expire_readonly_requests()
        self._poll_extension()
        context = getattr(self, '_submission', None)
        if self.closed or context is None or context['stage'] not in ('permit_ready', 'permit_install', 'preflight', 'receipt', 'renewal'):
            return
        if time.monotonic() >= context['retry_deadline']:
            self._submission_failed(context, 'ROS控制请求等待准入或确认的原2秒期限已到')
            return
        if context['rpc_pending']:
            return
        if context['stage'] == 'permit_install':
            error = self.control_graph_error()
            if error:
                self._submission_failed(context, error)
                return
            state, status = self.latest_state, self.latest_status
            if state is not None:
                identity = (tuple(state.device_id), state.profile_id, state.boot_epoch,
                            state.session_id, state.reference_epoch, state.motion_epoch)
                if identity != context['binding']:
                    self._submission_failed(context, 'ROS许可安装的原身份已改变')
                    return
            if (self._fresh_retry_binding(require_permit=False) != context['binding'] or
                    state.management_epoch != status.management_epoch):
                return  # 同一原期限内等双流；尚未发出管理写请求。
            context['management'] = state.management_epoch
            context['state_seq'] = state.state_seq
            context['status_at'] = self.status_at
        context['rpc_pending'] = True
        stage = context['stage']
        def response(value):
            if self._submission is not context or self.closed or context['stage'] != stage:
                return
            context['rpc_pending'] = False
            if time.monotonic() >= context['retry_deadline']:
                self._submission_failed(context, 'ROS控制请求的原2秒期限已到')
                return
            if isinstance(value, Exception):
                self._submission_failed(context, str(value))
            elif stage == 'permit_install':
                state, status = self.latest_state, self.latest_status
                if (state is None or status is None or
                        (tuple(state.device_id), state.profile_id, state.boot_epoch,
                         state.session_id, state.reference_epoch, state.motion_epoch) != context['binding'] or
                        (tuple(status.device_id), status.profile_id, status.boot_epoch,
                         status.session_id, status.reference_epoch, status.motion_epoch) != context['binding']):
                    self._submission_failed(context, 'ROS许可安装的原身份已改变')
                elif not value.success:
                    if value.message == 'PERMIT_NOT_QUEUED':
                        # C++明确尚未占槽、未发送：回到只读准入检查，沿用原身份、轨迹及2秒期限。
                        # QUEUED、超时、未知回包和其他拒绝均不走这条重试路径。
                        context['stage'] = 'permit_ready'
                    else:
                        self._submission_failed(context, 'ROS许可安装被拒绝：' + value.message)
                elif value.message == 'PERMIT_REUSED':
                    # 原许可未消费：不制造管理版本前进，仍核本次完整轨迹的实际剩余预算。
                    if (self._fresh_retry_binding() != context['binding'] or
                            state.management_epoch != status.management_epoch or
                            not self._permit_covers_trajectory(context)):
                        self._submission_failed(context, 'ROS复用许可的身份/新鲜度/轨迹预算不足')
                        return
                    context['management'] = state.management_epoch
                    context['state_seq'] = state.state_seq
                    context['status_at'] = self.status_at
                    self._send_trajectory_attempt(context)
                else:
                    context['stage'] = 'preflight'  # QUEUED不是COMMITTED；写请求绝不重发。
            elif stage in ('permit_ready', 'preflight'):
                state, status = self.latest_state, self.latest_status
                if state is None or status is None:
                    self._submission_failed(context, 'ROS起步缺少原绑定反馈')
                    return
                identity = (tuple(state.device_id), state.profile_id, state.boot_epoch,
                            state.session_id, state.reference_epoch, state.motion_epoch)
                if identity != context['binding']:
                    self._submission_failed(context, 'ROS起步等待的原身份已改变')
                    return
                if (value.success and
                        (value.message in ('ENABLED_HOLD', 'LEASE_RENEWED') or
                         (stage == 'permit_ready' and value.message == 'ENABLED_HOLD_PERMIT_REQUIRED')) and
                        self._fresh_retry_binding(require_permit=stage=='preflight') == context['binding'] and
                        state.management_epoch == status.management_epoch and
                        (stage == 'permit_ready' or
                         (0 < ((state.management_epoch-context['management']) & 0xffffffff) < 0x80000000 and
                          state.state_seq > context['state_seq'] and self.status_at > context['status_at'] and
                          self._permit_covers_trajectory(context)))):
                    if stage == 'permit_ready':
                        context['stage'] = 'permit_install'
                        return
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
        self.request('set_bench_permit' if stage == 'permit_install' else
                     'last_goal_rejection' if stage == 'receipt' else 'bootstrap_status', response)

    def _extension_binding(self, submission, require_fresh=True):
        state, status = self.latest_state, self.latest_status
        if self.closed or self.goal is None or self._submission is not submission or submission['stage'] != 'active':
            return None
        if state is None or status is None or self.status_max_age_ms is None or self.feedback_max_age_ms is None:
            return None
        now = time.monotonic()
        if require_fresh and not (0 <= now-self.state_at <= self.feedback_max_age_ms/1000. and
                0 <= now-self.status_at <= self.status_max_age_ms/1000.):
            return None
        def identity(value):
            return (tuple(value.device_id), value.profile_id, value.boot_epoch,
                    value.session_id, value.reference_epoch, value.motion_epoch)
        binding = identity(state)
        if (binding != identity(status) or binding[:5] != submission['binding'][:5] or
                state.core_state != 5 or status.core_state != 5 or
                not status.real_motion_permitted or status.permit_remaining_us <= 0 or status.evidence_lost):
            return None
        active_motion = submission.get('extension_motion')
        if active_motion is not None and binding[5] != active_motion:
            return None
        return binding

    def _retire_extension(self):
        context = getattr(self, '_extension', None)
        self._extension = None
        if context is not None and context.get('future') is not None:
            context['future'].cancel()

    def _extension_failed(self, context, reason):
        if getattr(self, '_extension', None) is not context:
            return
        self._retire_extension()
        # 回执失联不能当安装成功；请求取消母目标，正式Action终态仍由原回调报告。
        if self.goal is context['goal'] and self._submission is context['submission']:
            self.goal.cancel_goal_async()
        context['callback']('failed', reason)

    def _extension_diagnostic(self):
        now = time.monotonic()
        state, status = self.latest_state, self.latest_status
        return ('state_age_ms=%.3f status_age_ms=%.3f state_budget_ms=%s status_budget_ms=%s '
                'state_session=%s status_session=%s state_motion=%s status_motion=%s '
                'state_boot=%s status_boot=%s state_ref=%s status_ref=%s '
                'state_core=%s status_core=%s permit=%s evidence_lost=%s' % (
                    (now-self.state_at)*1000., (now-self.status_at)*1000.,
                    self.feedback_max_age_ms, self.status_max_age_ms,
                    getattr(state, 'session_id', None), getattr(status, 'session_id', None),
                    getattr(state, 'motion_epoch', None), getattr(status, 'motion_epoch', None),
                    getattr(state, 'boot_epoch', None), getattr(status, 'boot_epoch', None),
                    getattr(state, 'reference_epoch', None), getattr(status, 'reference_epoch', None),
                    getattr(state, 'core_state', None), getattr(status, 'core_state', None),
                    getattr(status, 'real_motion_permitted', None), getattr(status, 'evidence_lost', None)))

    def _poll_extension(self):
        context = getattr(self, '_extension', None)
        if context is None:
            return
        if self.closed or self._submission is not context['submission'] or self.goal is not context['goal']:
            self._retire_extension()
            return
        reason = None
        if time.monotonic() >= context['deadline']:
            reason = 'DEADLINE_EXPIRED: ROS轨迹续接原2秒期限已到'
        elif tuple(self.goal.goal_id.uuid) != context['uuid']:
            reason = 'UUID_CHANGED: ROS母目标身份已改变'
        elif self.control_graph_error():
            reason = 'CONTROL_GRAPH_CHANGED: ' + self.control_graph_error()
        elif self._extension_binding(context['submission'], require_fresh=False) != context['binding']:
            reason = 'IDENTITY_OR_PERMISSION_CHANGED: ROS续接身份或状态权限已改变'
        elif time.monotonic() < self.state_at or time.monotonic() < self.status_at:
            reason = 'FEEDBACK_CLOCK_REGRESSION: ROS反馈接收时钟倒退'
        if reason is not None:
            self._extension_failed(context, reason + '; ' + self._extension_diagnostic())
            return
        # Qt可能刚完成全路径准备，DDS新反馈尚在executor队列中。
        # 仅本地缓存龄超限时保留原回执和原截止；不重发，也不视为已安装确认。
        if self._extension_binding(context['submission']) != context['binding']:
            return
        response = context.get('response')
        if response is None:
            return
        self._extension = None
        if response.accepted:
            context['callback']('trajectory_extended', dict(version=response.version,
                phase_ns=response.phase_ns, trajectory_digest=[int(v) for v in response.trajectory_digest]))
        else:
            context['callback']('trajectory_extension_rejected', dict(reason=response.reason,
                phase_ns=response.phase_ns, version=response.version))

    def extend_trajectory(self, trajectory, expected_version, unchanged_through_ns, callback):
        submission = getattr(self, '_submission', None)
        if (submission is None or getattr(self, '_extension', None) is not None or
                self.control_graph_error() or not self.extension_client.service_is_ready()):
            return False
        binding = self._extension_binding(submission)
        if binding is None or not isinstance(expected_version, int) or not 0 <= expected_version < 2**64 or not isinstance(unchanged_through_ns, int) or not 0 <= unchanged_through_ns < 2**64:
            return False
        request = self.extension_type.Request()
        request.goal_id = self.uuid_type(uuid=list(self.goal.goal_id.uuid))
        request.motion_epoch = binding[5]
        request.expected_version = expected_version
        request.unchanged_through_ns = unchanged_through_ns
        request.trajectory = copy.deepcopy(trajectory)
        submission['extension_motion'] = binding[5]
        context = dict(submission=submission, goal=self.goal, binding=binding,
            callback=callback, deadline=time.monotonic()+2., future=None,
            uuid=tuple(self.goal.goal_id.uuid), response=None)
        self._extension = context
        def done(future):
            if getattr(self, '_extension', None) is not context:
                return
            try:
                context['response'] = future.result()
            except Exception as error:
                self._extension_failed(context, 'RPC_ERROR: ' + str(error) + '; ' + self._extension_diagnostic())
                return
            self._poll_extension()
        try:
            context['future'] = self.extension_client.call_async(request)
            context['future'].add_done_callback(done)
        except Exception as error:
            self._extension_failed(context, str(error))
            return False
        return True

    def submit_trajectory(self, trajectory, callback, task_budget_s=None):
        error = self.control_graph_error()
        if error:
            callback('failed', error)
            return False
        if self.closed or getattr(self, '_submission', None) is not None or not self.action.server_is_ready():
            callback('failed', 'ROS轨迹Action尚未就绪或已有目标')
            return False
        try:
            stamp = trajectory.points[-1].time_from_start
            duration_us = stamp.sec*1000000 + (stamp.nanosec+999)//1000
            if duration_us <= 0 or (task_budget_s is not None and
                    (not math.isfinite(task_budget_s) or task_budget_s < 0)):
                raise ValueError('invalid task budget')
            requested_us = duration_us if task_budget_s is None else max(duration_us, math.ceil(task_budget_s*1000000))
            # 与既有SDK移动预算一致：3秒到位容差 + 2秒传输/停止预留。
            required_permit_us = requested_us + 5000000
        except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
            callback('failed', 'ROS轨迹或任务许可预算无效')
            return False
        context = dict(trajectory=copy.deepcopy(trajectory), callback=callback, attempts=0,
                       required_permit_us=required_permit_us,
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
        context['stage'] = 'permit_ready'
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
                    self._retire_extension()
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
        self._retire_extension()
        self.cancel_pending_submission()
        if self.goal is not None:
            return self.goal.cancel_goal_async()
        return None

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self._retire_extension()
            self.cancel_pending_submission()
            # 关闭只销毁查询Future，不产生终态或回调到已关闭界面。
            pending = list(self._readonly_requests.values())
            self._readonly_requests.clear()
            for context in pending:
                context['future'].cancel()
            self.node.destroy_timer(self.retry_timer)
            self.node.destroy_client(self.parameter_client)
            self.action.destroy()
            if getattr(self, 'extension_client', None) is not None:
                self.node.destroy_client(self.extension_client)
            for client in self.clients.values():
                self.node.destroy_client(client)
            for subscription in self.subscriptions:
                self.node.destroy_subscription(subscription)
        finally:
            self._close_owned_control()
