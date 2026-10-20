#!/usr/bin/env python3
"""Component-only C05 peer, real ros2_control hardware/JTC, and public FJT probe."""
##
# @file device_action_e2e_probe.py
# @brief 实际ROS Gate/JTC与本机合成或DPSDK D08 peer的可运行组件探针。
#
# 入口会起peer/Gate/description进程、ROS服务/FJT目标及PTY原生桥或本机TCP。real指DPSDK实现POSIX peer，不是USB实物；严格SIL bundle是独立fixture pin，不等当前canonical/真实activation。synthetic完整矩阵、real单目标BENCH_RELEASE和smoke分支覆盖不同，证据仍无硬件资格。
import json
import hashlib
from contextlib import contextmanager
import os
import pathlib
import pty
import select
import socket
import subprocess
import sys
import tempfile
import time

from c05_latency_trace import host_callback_events, peer_events, summarize, write_report

import rclpy
import yaml
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ConfigureController, LoadController, SetHardwareComponentState, SwitchController
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

OFFLINE_SIL_CONTRACT_SHA256 = "0afb58ddfe9ce339c75da3c527b31abe875a261b3ac270e89d445ed96eda497b"
SIX_AXIS_TARGET = (0.007, -0.010, 0.013, -0.016, 0.019, -0.022)


##
# @brief spin至Future完成或timeout后assert，返回响应；done不代表业务结果ok。
# @param node 真实rclpy Node，由本探针spin推进。
# @param future rclpy异步Future，done后仍须外层核业务响应。
# @param timeout 等待秒数，默认5；不是设备动作安全预算。
# @return future.result()原值，超时assert。
def wait_future(node, future, timeout=5):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), "ROS response timeout"
    return future.result()


##
# @brief 创建临时ROS client、最多10s等服务再异步调用并等5s，调用方核响应语义；此helper未显式destroy_client。
# @param node 真实rclpy Node，由本探针spin推进。
# @param service_type ROS服务类型，用于创建client。
# @param name 服务名或子进程日志名，按调用接口。
# @param request 要实际发送的服务Request消息。
# @return 异步服务响应原值，外层负责语义检查。
def call(node, service_type, name, request):
    client = node.create_client(service_type, name)
    assert client.wait_for_service(timeout_sec=10), f"service missing: {name}"
    return wait_future(node, client.call_async(request))


##
# @brief 核v2.5 bundle每payload/排序聚合及固定OFFLINE_SIL pin，再按J1/六轴profile导能力；固件d8为fixture声明。
# @param bundle 固定trusted_sil_bundle根Path，payload自洽还须strict pin。
# @param profile_name 仅sil_j1或sil_6dof，默认J1。
# @return 由固定fixture导出的能力字典，firmware d8为人工标记。
def real_peer_capability(bundle, profile_name="sil_j1"):
    manifest = json.loads((bundle / "manifest.json").read_text())
    if (manifest.get("release_id"), manifest.get("schema_major"), manifest.get("schema_minor")) != ("dummy-wire-v2.5", 2, 5):
        raise ValueError("D08 fixture requires the current v2.5 bundle")
    payloads = {}
    listing = []
    for entry in manifest["payloads"]:
        relative = pathlib.PurePosixPath(entry["path"])
        if relative.is_absolute() or ".." in relative.parts or (bundle / relative).is_symlink():
            raise ValueError("unsafe bundle payload path")
        digest = hashlib.sha256((bundle / relative).read_bytes()).hexdigest()
        if digest != entry["sha256"] or entry["path"] in payloads:
            raise ValueError("D08 fixture bundle payload hash mismatch")
        payloads[entry["path"]] = digest
        listing.append(f"{entry['path']}:{digest}")
    if listing != sorted(listing) or hashlib.sha256("\n".join(listing).encode()).hexdigest() != manifest["payload_sorted_sha256"]:
        raise ValueError("D08 fixture bundle aggregate hash mismatch")
    if manifest["payload_sorted_sha256"] != OFFLINE_SIL_CONTRACT_SHA256:
        raise ValueError("untrusted SIL fixture bundle")
    if profile_name not in ("sil_j1", "sil_6dof"):
        raise ValueError("unsupported D08 fixture profile")
    profile = yaml.safe_load((bundle / "profiles" / f"{profile_name}.yaml").read_text())
    model = yaml.safe_load((bundle / "model/model.yaml").read_text())
    axes = [f"Joint{axis}" for axis in range(1, 7 if profile_name == "sil_6dof" else 2)]
    profile_id = 1 if profile_name == "sil_6dof" else 2
    if profile["profile_id"] != profile_id or profile["active_axes"] != axes:
        raise ValueError(f"D08 fixture requires the {profile_name} profile")
    speed = profile["simulation"]["velocity_limit_rad_s"]
    acceleration = profile["simulation"]["acceleration_limit_rad_s2"]
    if profile_name == "sil_6dof":
        if profile["reference_kind"] != "BENCH_SESSION":
            raise ValueError("six-axis SIL requires BENCH_SESSION")
        lower, upper = profile["bench_position_range_rad"]
        travel = profile["simulation"]["max_trial_travel_rad"]
        if not -travel <= lower < 0 < upper <= travel:
            raise ValueError("six-axis SIL trial range exceeds Core travel")
        position_min = [lower] * 6
        position_max = [upper] * 6
    else:
        position_min = [joint["model_limits_rad"][0] for joint in model["joints"]]
        position_max = [joint["model_limits_rad"][1] for joint in model["joints"]]
    return {
        "contract_sha256": manifest["payload_sorted_sha256"],
        "model_sha256": payloads["model/model.yaml"],
        "profile_sha256": payloads[f"profiles/{profile_name}.yaml"],
        "compiled_policy_sha256": payloads[f"profiles/{profile_name}.yaml"],
        "firmware_sha256": "d8" + "00" * 31,
        "position_min_rad": position_min,
        "position_max_rad": position_max,
        "velocity_max_rad_s": [speed] * 6,
        "acceleration_max_rad_s2": [acceleration] * 6,
        "control_period_us": profile["timing"]["control_period_us"],
        "state_publish_period_us": profile["timing"]["state_publish_period_us"],
    }


##
# @brief 在已pin能力上仅允许显式20ms/50ms State周期候选，保原控制节奏/静态摘要，不重签或升级资格。
# @param bundle 固定trusted_sil_bundle根Path，payload自洽还须strict pin。
# @param profile_name 仅sil_j1或sil_6dof，默认J1。
# @return 原能力或只覆写State周期的新字典。
def fixture_capability(bundle, profile_name="sil_j1"):
    capability = real_peer_capability(bundle, profile_name)
    candidate = os.environ.get("DUMMY_E2E_STATE_PERIOD_US")
    if candidate is None:
        return capability
    if candidate not in ("20000", "50000"):
        raise ValueError("DUMMY_E2E_STATE_PERIOD_US supports only explicit 20000us or 50000us candidates")
    return {**capability, "state_publish_period_us": int(candidate)}


##
# @brief control_period_us须正int且整除1s，返回Hz；不从State发布周期推controller速率。
# @param capability fixture能力字典，含静态摘要/设备界/周期，未赋实机资格。
# @return 整数Hz。
def controller_update_rate(capability):
    period_us = capability["control_period_us"]
    if not isinstance(period_us, int) or period_us <= 0 or 1000000 % period_us:
        raise ValueError("control period must divide one second")
    return 1000000 // period_us


##
# @brief 每50ms采三次平滑插值，位置从start到target；只核正时长/轴数一致，不做完整有限性/动态/碰撞验证。
# @param start 标量或各轴起点rad。
# @param target 标量或各轴终点rad，轴数需与start一致。
# @param duration_s 正的整型秒数，代码以duration_s×20用于range；未额外归一float类型。
# @return JointTrajectoryPoint列表，每点间隔50000000ns。
def smooth_motion_points(start, target, duration_s=7):
    if duration_s <= 0:
        raise ValueError("trajectory duration must be positive")
    start_positions = [start] if isinstance(start, (int, float)) else list(start)
    target_positions = [target] if isinstance(target, (int, float)) else list(target)
    if len(start_positions) != len(target_positions):
        raise ValueError("trajectory axis count changed")
    samples = duration_s * 20
    points = []
    for index in range(1, samples + 1):
        progress = index / samples
        blend = progress * progress * (3 - 2 * progress)
        point = JointTrajectoryPoint(positions=[initial + (final - initial) * blend
                                                 for initial, final in zip(start_positions, target_positions)])
        elapsed_ns = index * 50000000
        point.time_from_start.sec = elapsed_ns // 1000000000
        point.time_from_start.nanosec = elapsed_ns % 1000000000
        points.append(point)
    return points


##
# @brief 读现有JSONL过滤command并取最后limit条，缺文件空列表；未验证命令对应实体CAN发送。
# @param path 输入/输出文件Path，具体读写及排他要求见函数职责。
# @param limit 末尾command条数，默认8；调用方提供合适正整数。
# @return 过滤后的末limit条command事件。
def peer_command_tail(path, limit=8):
    if not path.is_file():
        return []
    events = (json.loads(line) for line in path.read_text().splitlines() if line)
    return [event for event in events if event.get("event") == "command"][-limit:]


##
# @brief 逐轴只核末command距target<0.001rad、末actual距target<0.002rad；不是全路径误差或raw链校验。
# @param feedback 实际FJT反馈message列表，不是设备raw页链。
# @param commands peer日志的已接受command字典列表。
# @param target 标量或各轴终点rad，轴数需与start一致。
# @return None；末点缺轴/误差超界触发assert。
def validate_six_axis_evidence(feedback, commands, target):
    assert feedback and commands, "six-axis feedback or commands missing"
    for axis, expected in enumerate(target):
        assert abs(commands[-1]["position"][axis] - expected) < 0.001, \
            f"axis {axis + 1} command target missing"
        assert abs(feedback[-1].actual.positions[axis] - expected) < 0.002, \
            f"axis {axis + 1} feedback target missing"


##
# @brief smoke/real_smoke J1用0.02rad，完整默认0.05rad；六轴调用方另用SIX_AXIS_TARGET。
# @param mode synthetic/real/smoke等入口模式字符串。
# @return 0.02或0.05rad。
def first_goal_target(mode):
    return 0.02 if mode in ("real_smoke", "smoke") else 0.05


##
# @brief 生成最小1或6轴ros2_control XML，注入能力与Guarded插件；起始值0、State1s/heartbeat250ms是组件fixture预算。
# @param endpoint host使用的本机serial PTY或TCP地址；非设备activation身份。
# @param capability fixture能力字典，含静态摘要/设备界/周期，未赋实机资格。
# @param joint_count 仅1或6，决定URDF关节/掩码。
# @return 最小组件URDF XML字符串，不写文件或激活插件。
def urdf(endpoint, capability=None, joint_count=1):
    capability = capability or {
        "contract_sha256": "01" * 32, "model_sha256": "02" * 32,
        "profile_sha256": "03" * 32, "firmware_sha256": "04" * 32,
        "compiled_policy_sha256": "05" * 32,
        "position_min_rad": [-1] * 6, "position_max_rad": [1] * 6,
        "velocity_max_rad_s": [1] * 6, "acceleration_max_rad_s2": [1] * 6,
        "control_period_us": 10000, "state_publish_period_us": 10000,
    }
    values = {key: ",".join(format(item, ".17g") for item in value) if isinstance(value, list) else value
              for key, value in capability.items()}
    if joint_count not in (1, 6):
        raise ValueError("D08 fixture supports one or six axes")
    links = '<link name="base_link"/>' + "".join(f'<link name="link{axis}"/>'
                                                 for axis in range(1, joint_count + 1))
    joints = "".join(
        f'<joint name="Joint{axis}" type="revolute"><parent link="{("base_link" if axis == 1 else f"link{axis - 1}")}"/>'
        f'<child link="link{axis}"/><axis xyz="0 0 1"/>'
        f'<limit lower="{capability["position_min_rad"][axis - 1]}" '
        f'upper="{capability["position_max_rad"][axis - 1]}" velocity="0.25" effort="1"/></joint>'
        for axis in range(1, joint_count + 1))
    controls = "".join(
        f'<joint name="Joint{axis}"><command_interface name="position"/>'
        '<state_interface name="position"><param name="initial_value">0</param></state_interface>'
        '<state_interface name="velocity"><param name="initial_value">0</param></state_interface></joint>'
        for axis in range(1, joint_count + 1))
    return f'''<robot name="component_dummy">{links}{joints}
<ros2_control name="dummy" type="system"><hardware>
<plugin>dummy_ros_control/GuardedZenbeddedSystem</plugin>
<param name="endpoint">{endpoint}</param><param name="device_id">01000000000000000000000000000000</param>
<param name="client_instance_id">02000000000000000000000000000000</param>
<param name="profile_id">{1 if joint_count == 6 else 2}</param><param name="configured_mask">{63 if joint_count == 6 else 1}</param><param name="max_state_age_us">1000000</param>
<param name="heartbeat_stop_budget_us">250000</param>
<param name="contract_sha256">{values['contract_sha256']}</param><param name="model_sha256">{values['model_sha256']}</param>
<param name="profile_sha256">{values['profile_sha256']}</param><param name="firmware_sha256">{values['firmware_sha256']}</param>
<param name="compiled_policy_sha256">{values['compiled_policy_sha256']}</param>
<param name="position_min_rad">{values['position_min_rad']}</param>
<param name="position_max_rad">{values['position_max_rad']}</param>
<param name="velocity_max_rad_s">{values['velocity_max_rad_s']}</param>
<param name="acceleration_max_rad_s">{values['acceleration_max_rad_s2']}</param>
<param name="control_period_us">{values['control_period_us']}</param><param name="state_publish_period_us">{values['state_publish_period_us']}</param>
</hardware>{controls}</ros2_control></robot>'''


##
# @brief 只接受名为dummy_ref_d08_device_peer且存在可执行的路径，不搜索其它构建profile兜底，也不验证二进制SHA。
# @param peer 要运行的POSIX peer可执行路径。
# @return 输入peer Path原值，检查不符抛异常。
def validate_peer_fixture(peer):
    if peer.name != "dummy_ref_d08_device_peer":
        raise ValueError(f"DPSDK D08 peer fixture must be dummy_ref_d08_device_peer: {peer}")
    if not peer.is_file():
        raise FileNotFoundError(
            f"DPSDK D08 peer fixture is missing: {peer}; build the DPSDK v2-tests profile "
            "or set DUMMY_D08_PEER_EXECUTABLE at CMake configure time")
    if not os.access(peer, os.X_OK):
        raise PermissionError(f"DPSDK D08 peer fixture is not executable: {peer}")
    return peer


##
# @brief 显式ROS_DOMAIN_ID限制0..232，否则100+PID模100；不保证全系统域唯一。
# @return 合法域ID十进制字符串。
def selected_ros_domain_id():
    configured = os.environ.get("ROS_DOMAIN_ID")
    if configured is not None:
        domain = int(configured)
        if not 0 <= domain <= 232:
            raise ValueError("ROS_DOMAIN_ID must be between 0 and 232")
        return str(domain)
    return str(100 + os.getpid() % 100)


##
# @brief 显式DUMMY_SERIAL_BRIDGE_EXECUTABLE优先，否则固定build_dummy原生PTY桥路径；不在此构建/核SHA。
# @return 桥可执行Path，未检查文件存在。
def serial_bridge_executable():
    configured = os.environ.get("DUMMY_SERIAL_BRIDGE_EXECUTABLE")
    if configured:
        return pathlib.Path(configured)
    return pathlib.Path(__file__).resolve().parents[3] / "build_dummy/dummy_motion_client/serial_pty_bridge"


##
# @brief 真实建两对PTY与原生bridge子进程，READY后yield两serial端点；退出stop→terminate→kill有限升级并关四fd/收诊断。
# @param diagnostics 可变桥统计字典，正常退出时更新；None新建内部dict。
# @param gap_threshold_us 记录长块间隙的阈值µs，默认100000。
# @return yield(host serial端点, peer serial端点)，退出回收桥及四PTY fd。
@contextmanager
def serial_peer_endpoints(diagnostics=None, gap_threshold_us=100000):
    host_master, host_slave = pty.openpty()
    try:
        peer_master, peer_slave = pty.openpty()
    except BaseException:
        os.close(host_master)
        os.close(host_slave)
        raise
    if diagnostics is None:
        diagnostics = {}
    worker = None
    try:
        worker = subprocess.Popen(
            [str(serial_bridge_executable()), str(host_master), str(peer_master),
             str(gap_threshold_us)], pass_fds=(host_master, peer_master),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if not select.select([worker.stdout], [], [], 5)[0] or worker.stdout.readline().strip() != "READY":
            raise RuntimeError("serial fixture bridge did not start")
        yield (f"serial/{os.ttyname(host_slave)}#baudrate=3000000",
               f"serial/{os.ttyname(peer_slave)}#baudrate=3000000")
    finally:
        try:
            if worker is not None:
                try:
                    output, error = worker.communicate(input="stop\n", timeout=2)
                except subprocess.TimeoutExpired:
                    worker.terminate()
                    try:
                        output, error = worker.communicate(timeout=2)
                    except subprocess.TimeoutExpired:
                        worker.kill()
                        output, error = worker.communicate(timeout=2)
                if worker.returncode == 0:
                    diagnostics.update(json.loads(output))
                else:
                    raise RuntimeError(f"serial fixture bridge exited with {worker.returncode}: {error}")
        finally:
            for descriptor in (host_master, host_slave, peer_master, peer_slave):
                os.close(descriptor)

##
# @brief 按argv模式及serial/tcp选择实际运行；serial可排他写latency报告，synthetic六轴失败也保INCOMPLETE trace再重抛。
# @return None；执行真实探针/打印报告，异常重抛。
def main():
    gate, peer, bundle = (pathlib.Path(value).resolve() for value in sys.argv[1:4])
    mode = sys.argv[4] if len(sys.argv) > 4 else "synthetic"
    if mode not in ("synthetic", "unsupported", "real", "real_smoke", "smoke",
                    "real_six_axis_smoke", "six_axis_smoke"):
        raise ValueError(f"unsupported probe mode: {mode}")
    unsupported = mode == "unsupported"
    real_peer = mode in ("real", "real_smoke", "real_six_axis_smoke")
    smoke = mode in ("real_smoke", "smoke", "real_six_axis_smoke", "six_axis_smoke")
    six_axis = mode in ("real_six_axis_smoke", "six_axis_smoke")
    if real_peer:
        peer = validate_peer_fixture(peer)
    capability = fixture_capability(bundle, "sil_6dof" if six_axis else "sil_j1")
    transport = os.environ.get("DUMMY_E2E_TRANSPORT", "serial")
    if transport == "serial":
        bridge_diagnostics = {}
        trace_events = []
        try:
            with serial_peer_endpoints(bridge_diagnostics,
                                       2 * capability["state_publish_period_us"]) as (endpoint, peer_endpoint):
                trace_events = run_probe(gate, peer, bundle, unsupported, real_peer, capability,
                                         endpoint, peer_endpoint, first_goal_target(mode), smoke, six_axis,
                                         trace_capture=trace_events)
            if trace_events is not None:
                report = {"event": "c05_latency_trace",
                          **summarize(trace_events, bridge_diagnostics)}
                if output_path := os.environ.get("DUMMY_C05_TRACE_OUTPUT"):
                    write_report(output_path, report)
                print(json.dumps(report, sort_keys=True))
        except BaseException as error:
            if six_axis and not real_peer:
                report = {"event": "c05_latency_trace", "probe_error": str(error),
                          **summarize(trace_events, bridge_diagnostics)}
                if output_path := os.environ.get("DUMMY_C05_TRACE_OUTPUT"):
                    write_report(output_path, report)
                print(json.dumps(report, sort_keys=True), file=sys.stderr)
            print(f"serial_bridge: {json.dumps(bridge_diagnostics, sort_keys=True)}", file=sys.stderr)
            raise
    elif transport == "tcp":
        with socket.socket() as reserve:
            reserve.bind(("127.0.0.1", 0))
            endpoint = f"tcp/127.0.0.1:{reserve.getsockname()[1]}"
        run_probe(gate, peer, bundle, unsupported, real_peer, capability,
                  endpoint, endpoint, first_goal_target(mode), smoke, six_axis)
    else:
        raise ValueError(f"unsupported DUMMY_E2E_TRANSPORT: {transport}")


##
# @brief 真实启动组件并显式Enable/激活/JTC/FJT；unsupported验证零变更，smoke/六轴早退，real只核首目标及禁止第二/软件Release，synthetic才走续permit/Cancel/Abort。
# @param gate device_action_gate可执行Path。
# @param peer 要运行的POSIX peer可执行路径。
# @param bundle 固定trusted_sil_bundle根Path，payload自洽还须strict pin。
# @param unsupported true测试合成peer motion_supported=false的零变更拒绝路径。
# @param real_peer true运行DPSDK D08 POSIX peer；false合成C05 peer，不表示实体硬件。
# @param capability fixture能力字典，含静态摘要/设备界/周期，未赋实机资格。
# @param endpoint host使用的本机serial PTY或TCP地址；非设备activation身份。
# @param peer_endpoint peer使用的另一PTY端或同本机TCP地址。
# @param first_target 单轴首目标rad；六轴模式使用独立SIX_AXIS_TARGET。
# @param smoke true首成功后做简短断言并提前结束，不覆盖完整取消/中止矩阵。
# @param six_axis true用六轴profile1/mask63及互异带符号目标，否则J1 profile2/mask1。
# @param trace_capture 可选共享list，异常时保留已解析peer/Host trace，缺阶段不升级合格。
# @return synthetic六轴成功返回peer标记列表；其它分支None，结果主经stdout JSON报告。
def run_probe(gate, peer, bundle, unsupported, real_peer, capability, endpoint, peer_endpoint,
              first_target, smoke, six_axis=False, trace_capture=None):
    profile_name = "sil_6dof" if six_axis else "sil_j1"
    profile_id = 1 if six_axis else 2
    mask = 63 if six_axis else 1
    joint_names = [f"Joint{axis}" for axis in range(1, 7 if six_axis else 2)]
    workspace = pathlib.Path(__file__).resolve().parents[3]
    from ament_index_python.packages import get_package_prefix
    control_prefix = pathlib.Path(get_package_prefix("dummy_ros_control"))
    env = dict(os.environ, ROS_DOMAIN_ID=selected_ros_domain_id(), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    env["AMENT_PREFIX_PATH"] = str(control_prefix) + os.pathsep + env.get("AMENT_PREFIX_PATH", "")
    env["LD_LIBRARY_PATH"] = str(control_prefix / "lib") + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = env["ROS_AUTOMATIC_DISCOVERY_RANGE"]
    processes = []
    with tempfile.TemporaryDirectory(prefix="device-action-e2e-") as temporary:
        directory = pathlib.Path(temporary)
        params = {
            "controller_manager": {"ros__parameters": {"update_rate": controller_update_rate(capability),
                "hardware_components_initial_state": {"inactive": ["dummy"]},
                "arm_controller": {"type": "dummy_ros_control/GuardedJointTrajectoryController"}}},
            "arm_controller": {"ros__parameters": {"joints": joint_names,
                "command_interfaces": ["position"], "state_interfaces": ["position", "velocity"],
                "allow_partial_joints_goal": False, "allow_nonzero_velocity_at_trajectory_end": False}},
            "device_action_gate": {"ros__parameters": {"endpoint": endpoint,
                "device_id": "01000000000000000000000000000000", "client_instance_id": "02000000000000000000000000000000",
                "profile_id": profile_id, "configured_mask": mask, "session_id": 11,
                "state_max_age_us": 1000000,
                "permit_sha256": "01" + "00" * 31, "permit_remaining_us": 60000000,
                "contract_bundle": str(bundle), "profile_name": profile_name}},
            "robot_state_publisher": {"ros__parameters": {"robot_description": urdf(endpoint, capability, len(joint_names))}},
        }
        configuration = directory / "params.yaml"
        configuration.write_text(yaml.safe_dump(params))
        logs = {}

        ##
        # @par 内部回调 launch
        # 为每个自有子进程建临时log、Popen并登记processes，最终逆序清理；不启动独立进程组。
        # 参数 name：服务名或子进程日志名，按调用接口。
        # 参数 command：要实际Popen的argv；包装器不替调用方审查硬件副作用。
        # 返回：已登记的Popen对象。
        def launch(name, command):
            log = open(directory / f"{name}.log", "a+")
            logs[name] = log
            process = subprocess.Popen(command, env=env, stdout=log, stderr=log)
            processes.append(process)
            return process

        try:
            peer_command = [str(peer), "--endpoint", peer_endpoint, "--client-id",
                            "02000000000000000000000000000000", "--profile-id", str(profile_id),
                            "--configured-mask", str(mask)]
            evidence_file = directory / "peer-command-evidence.jsonl"
            if real_peer:
                peer_command += ["--evidence-file", str(evidence_file)]
                peer_command += ["--contract-sha256", capability["contract_sha256"],
                                 "--model-sha256", capability["model_sha256"],
                                 "--profile-sha256", capability["profile_sha256"]]
                for field, option in (("position_min_rad", "--position-min-rad"),
                                      ("position_max_rad", "--position-max-rad")):
                    peer_command += [option, ",".join(format(value, ".17g") for value in capability[field])]
                peer_command += ["--velocity-limit-rad-s", str(capability["velocity_max_rad_s"][0]),
                                 "--acceleration-limit-rad-s2", str(capability["acceleration_max_rad_s2"][0]),
                                 "--control-period-us", str(capability["control_period_us"]),
                                 "--state-publish-period-us", str(capability["state_publish_period_us"])]
            if not real_peer:
                peer_command += ["--device-id", "01000000000000000000000000000000",
                                 "--contract-bundle", str(bundle), "--profile-name", profile_name,
                                 "--state-publish-period-us", str(capability["state_publish_period_us"]),
                                 "--motion-supported", "false" if unsupported else "true"]
                if six_axis:
                    peer_command.append("--trace")
            launch("peer", peer_command)
            time.sleep(0.4)
            launch("gate", [str(gate), "--ros-args", "--params-file", str(configuration)])
            launch("description", ["/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher", "--ros-args", "--params-file", str(configuration)])
            rclpy.init()
            node = rclpy.create_node("device_action_e2e_probe")
            service = "/controller_manager/set_hardware_component_state"
            client = node.create_client(SetHardwareComponentState, service)
            assert client.wait_for_service(timeout_sec=10), "manager did not start"
            time.sleep(1)
            logs["peer"].seek(0)
            assert '"event":"manage"' not in logs["peer"].read(), "startup implicitly issued Enable"
            enable_response = None
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                enable_response = call(node, Trigger, "/device_action_gate/enable", Trigger.Request())
                if enable_response.success:
                    break
                time.sleep(0.1)
            assert enable_response is not None
            if unsupported:
                assert not enable_response.success, "motion-unsupported device accepted Enable"
                logs["peer"].seek(0)
                assert '"event":"manage"' not in logs["peer"].read(), "mutation sent to unsupported peer"
                print(json.dumps({"evidence_scope": "component_only", "motion_unsupported_rejected": True, "mutation_count": 0}))
                node.destroy_node()
                rclpy.shutdown()
                return
            assert enable_response.success, f"explicit enable unavailable: {enable_response.message}"
            while time.monotonic() < deadline:
                bootstrap = call(node, Trigger, "/device_action_gate/bootstrap_status", Trigger.Request())
                if bootstrap.success:
                    break
                time.sleep(0.05)
            assert bootstrap.success, f"bootstrap did not reach HOLD: {bootstrap.message}"
            request = SetHardwareComponentState.Request(name="dummy")
            request.target_state.id = 3
            assert wait_future(node, client.call_async(request)).ok, "hardware activation failed"
            assert call(node, LoadController, "/controller_manager/load_controller", LoadController.Request(name="arm_controller")).ok
            assert call(node, ConfigureController, "/controller_manager/configure_controller", ConfigureController.Request(name="arm_controller")).ok
            switch = SwitchController.Request(activate_controllers=["arm_controller"], strictness=2)
            switch.timeout.sec = 5
            assert call(node, SwitchController, "/controller_manager/switch_controller", switch).ok
            action = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
            assert action.wait_for_server(timeout_sec=5)
            time.sleep(0.3)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joint_names
            if six_axis:
                initial = [0.0] * 6
                target = SIX_AXIS_TARGET
                goal.trajectory.points = smooth_motion_points(initial, target)
            else:
                goal.trajectory.points = smooth_motion_points(0.0, first_target)
            goal.goal_time_tolerance.sec = 1
            feedback = []
            handle = wait_future(node, action.send_goal_async(goal, feedback_callback=lambda value: feedback.append(value.feedback)))
            assert handle.accepted, "public goal rejected after Enable"
            result = wait_future(node, handle.get_result_async(), 10)
            assert result.status == 4 and result.result.error_code == 0, f"motion did not succeed: {result}"
            assert feedback, "public FJT did not publish feedback"
            assert all(item.joint_names == joint_names and len(item.actual.positions) == len(joint_names) and
                       len(item.desired.positions) == len(joint_names) for item in feedback), "feedback axis shape"
            if six_axis:
                processes[0].terminate()
                processes[0].wait(timeout=5)
                if real_peer:
                    commands = peer_command_tail(evidence_file, limit=4096)
                else:
                    logs["peer"].seek(0)
                    peer_lines = logs["peer"].read().splitlines()
                    commands = [event for line in peer_lines
                                if line.startswith('{"event":"command"')
                                for event in (json.loads(line),)]
                assert commands and all(len(event["position"]) == 6 for event in commands)
                validate_six_axis_evidence(feedback, commands, target)
                print(json.dumps({"evidence_scope": "real_dpsdk_peer" if real_peer else "component_only",
                                  "six_axis_fjt_succeeded": True,
                                  "target_rad": target,
                                  "contract_identity_scope": "local_synthetic_sil_fixture",
                                  "contract_sha256": capability["contract_sha256"],
                                  "hardware_activation_evidence": False,
                                  "joint_names": joint_names, "feedback_count": len(feedback),
                                  "command_count": len(commands)}))
                node.destroy_node()
                rclpy.shutdown()
                return peer_events(peer_lines) if not real_peer else None
            assert any(abs(item.actual.positions[0] - first_target) > 0.005 for item in feedback), "actual feedback mirrored goal"
            elapsed = [item.actual.time_from_start.sec + item.actual.time_from_start.nanosec * 1e-9 for item in feedback]
            assert elapsed == sorted(elapsed) and 0 <= elapsed[0] < 0.1 and elapsed[-1] < 8, "feedback clock"

            if smoke:
                processes[0].terminate()
                processes[0].wait(timeout=5)
                logs["peer"].seek(0)
                events = [json.loads(line) for line in logs["peer"].read().splitlines()
                          if line.startswith('{"event"')]
                commands = peer_command_tail(evidence_file, limit=4096) if real_peer else [
                    event for event in events if event["event"] == "command"]
                assert commands and max(event["position"][0] for event in commands) > 0.015
                operations = [event["operation"] for event in events if event["event"] == "manage"]
                assert operations[:7] == [1, 2, 3, 5, 6, 7, 8], operations
                print(json.dumps({"evidence_scope": "real_dpsdk_peer" if real_peer else "component_only",
                                  "smoke_fjt_succeeded": True, "goal_rad": first_target,
                                  "feedback_count": len(feedback), "command_count": len(commands),
                                  "operations": operations}))
                node.destroy_node()
                rclpy.shutdown()
                return

            ##
            # @par 内部回调 send
            # 在当前公开Action上真实提交单轴平滑轨迹，收集回调并返回handle/stream；可注入极小path容差触发abort。
            # 参数 start：标量或各轴起点rad。
            # 参数 target：标量或各轴终点rad，轴数需与start一致。
            # 参数 duration：send内部轨迹秒数，默认7。
            # 参数 tolerance：可选J1 path tolerance rad，None用默认，极小值用于abort刺激。
            # 返回：(Action handle, 后续继续追加的feedback list)。
            def send(start, target, duration=7, tolerance=None):
                motion = FollowJointTrajectory.Goal()
                motion.trajectory.joint_names = ["Joint1"]
                motion.trajectory.points = smooth_motion_points(start, target, duration)
                motion.goal_time_tolerance.sec = 1
                if tolerance is not None:
                    motion.path_tolerance = [JointTolerance(name="Joint1", position=tolerance)]
                stream = []
                accepted = wait_future(node, action.send_goal_async(motion, feedback_callback=lambda value: stream.append(value.feedback)))
                return accepted, stream

            if real_peer:
                status = call(node, Trigger, "/device_action_gate/bootstrap_status", Trigger.Request())
                assert not status.success, "BENCH_RELEASE reopened admission after Complete"
                second, _ = send(first_target, 0.08)
                assert not second.accepted, "BENCH_RELEASE admitted a second goal after Complete"
                release = call(node, Trigger, "/device_action_gate/disable_and_release", Trigger.Request())
                assert not release.success and release.message == "RELEASE_UNAVAILABLE", \
                    f"software-only SIL unexpectedly permitted Release: {release.message}"
                processes[0].terminate()
                processes[0].wait(timeout=5)
                logs["peer"].seek(0)
                events = [json.loads(line) for line in logs["peer"].read().splitlines()
                          if line.startswith('{"event"')]
                operations = [event["operation"] for event in events if event["event"] == "manage"]
                assert operations == [1, 2, 3, 5, 6, 7, 8], operations
                commands = peer_command_tail(evidence_file, limit=4096)
                assert commands and len({event["freshness_ticket"] for event in commands}) > 20
                assert all(event["motion_epoch"] <= 3 for event in commands), "second motion run began"
                print(json.dumps({"evidence_scope": "real_dpsdk_peer", "deployment_stop_policy": "BENCH_RELEASE",
                                  "public_fjt_succeeded": True, "second_goal_rejected": True,
                                  "software_only_release_rejected": True, "operations": operations,
                                  "distinct_freshness_tickets": len({event["freshness_ticket"] for event in commands})}))
                node.destroy_node()
                rclpy.shutdown()
                return

            ##
            # @par 内部回调 renew_permit
            # 十秒共同预算内等SetPermit接纳再poll bootstrap成功；是组件会话交互，不恢复旧目标许可或提供硬件授权。
            # 返回：None；十秒内未确认则AssertionError。
            def renew_permit():
                renewal_deadline = time.monotonic() + 10
                while time.monotonic() < renewal_deadline:
                    response = call(node, Trigger, "/device_action_gate/set_bench_permit", Trigger.Request())
                    if response.success:
                        break
                    time.sleep(0.05)
                else:
                    status = call(node, Trigger, "/device_action_gate/bootstrap_status", Trigger.Request())
                    raise AssertionError(
                        f"single-use permit renewal was unavailable: {response.message}; "
                        f"bootstrap_status={status.success}:{status.message}")
                while time.monotonic() < renewal_deadline:
                    status = call(node, Trigger, "/device_action_gate/bootstrap_status", Trigger.Request())
                    if status.success:
                        return
                    time.sleep(0.05)
                raise AssertionError(f"single-use permit renewal was not confirmed: {status.message}")

            renew_permit()
            second, second_feedback = send(0.05, 0.08)
            assert second.accepted, "second goal was not admitted after confirmed HOLD"
            second_result = wait_future(node, second.get_result_async(), 10)
            assert second_result.status == 4, f"second goal failed: {second_result}"
            assert second_feedback and abs(second_feedback[0].actual.positions[0] - 0.05) < 0.002, "second goal jumped to home"
            renew_permit()
            canceled, cancel_feedback = send(0.08, 0.15)
            assert canceled.accepted
            overlap, _ = send(0.08, 0.1)
            assert not overlap.accepted, "overlapping public goal accepted"
            until_cancel = time.monotonic() + 0.2
            while time.monotonic() < until_cancel:
                rclpy.spin_once(node, timeout_sec=0.01)
            cancel_response = wait_future(node, canceled.cancel_goal_async())
            assert cancel_response.goals_canceling, "cancel request not accepted"
            canceled_result = wait_future(node, canceled.get_result_async(), 10)
            assert canceled_result.status == 5, f"cancel did not cross confirmed HOLD barrier: {canceled_result}"
            renew_permit()
            abort_start = cancel_feedback[-1].actual.positions[0] if cancel_feedback else 0.08
            aborted, abort_feedback = send(abort_start, 0.12, tolerance=1e-7)
            assert aborted.accepted, "cancel barrier did not reopen admission"
            aborted_result = wait_future(node, aborted.get_result_async(), 10)
            assert aborted_result.status == 6 and "PATH_TOLERANCE" in aborted_result.result.error_string, f"abort path not confirmed: {aborted_result}"
            processes[0].terminate()
            processes[0].wait(timeout=5)
            logs["peer"].seek(0)
            events = [json.loads(line) for line in logs["peer"].read().splitlines() if line.startswith('{"event"')]
            if real_peer:
                events += [json.loads(line) for line in evidence_file.read_text().splitlines()]
            operations = [event["operation"] for event in events if event["event"] == "manage"]
            expected_operations = [1, 2, 3, 5, 6, 7, 8, 5, 7, 8, 5, 7, 9, 5, 7, 9]
            assert operations == expected_operations, operations
            if real_peer:
                operation_results = [event for event in events if event["event"] == "operation_result"]
                assert all(event["operation_id"] > 0 for event in operation_results), operation_results
            commands = [event for event in events if event["event"] == "command"]
            sequences = [event["producer_seq"] for event in commands]
            assert sequences == sorted(set(sequences)), "producer generations repeated or regressed"
            ticket_count = len({event["freshness_ticket"] for event in commands})
            assert ticket_count > 20, "peer did not rotate freshness tickets during motion"
            ##
            # @par 历史终态扫描边界
            # 上方expected_operations不含13且已断言相等，所以通过该表检查时下方op13扫描列表为空；不能把此循环当当前op8 Complete冻结的独立覆盖。
            frozen = []
            for index, event in enumerate(events):
                if event["event"] != "manage" or event["operation"] != 13:
                    continue
                if real_peer:
                    old_epoch = event["motion_epoch"] - 1
                    prior = [value for value in commands if value["motion_epoch"] == old_epoch][-1]
                    later = [value for value in commands if value["motion_epoch"] == event["motion_epoch"]]
                else:
                    prior = [value for value in events[:index] if value["event"] == "command" and value["motion_epoch"] == event["motion_epoch"]][-1]
                    later = [value for value in events[index + 1:] if value["event"] == "command"]
                    assert not any(value["motion_epoch"] == event["motion_epoch"] for value in later), "command emitted after terminal freeze"
                hold_tolerance = 0.001 if real_peer else 1e-9
                assert later and abs(later[0]["position"][0] - prior["position"][0]) <= hold_tolerance, "completion hold jumped"
                frozen.append({"motion_epoch": prior["motion_epoch"], "producer_seq": prior["producer_seq"], "position": prior["position"][0]})
            results = [result, second_result, canceled_result, aborted_result]
            print(json.dumps({"evidence_scope": "real_dpsdk_peer" if real_peer else "component_only", "public_fjt_succeeded": result.status == 4,
                              "success_count": sum(item.status == 4 for item in results),
                              "cancel_count": sum(item.status == 5 for item in results),
                              "abort_count": sum(item.status == 6 for item in results),
                              "feedback_count": sum(map(len, [feedback, second_feedback, cancel_feedback, abort_feedback])),
                              "terminal_freezes": frozen, "operations": operations,
                              "distinct_freshness_tickets": ticket_count,
                              "operation_id_verified_by_client": True,
                              "real_hardware_plugin": True, "real_guarded_jtc": True}))
            node.destroy_node()
            rclpy.shutdown()
        except BaseException as error:
            if processes and processes[0].poll() is None:
                processes[0].terminate()
                try:
                    processes[0].wait(timeout=5)
                except subprocess.TimeoutExpired:
                    processes[0].kill()
                    processes[0].wait()
            if six_axis and not real_peer and trace_capture is not None:
                logs["peer"].seek(0)
                trace_capture.extend(peer_events(logs["peer"].read().splitlines()))
            if real_peer:
                print(f"peer_command_tail: {json.dumps(peer_command_tail(evidence_file))}", file=sys.stderr)
            gate_output = ""
            for (name, log), process in zip(logs.items(), processes):
                log.seek(0)
                full_output = log.read()
                output = full_output[-18000:]
                if name == "peer" and not real_peer:
                    diagnostic_lines = [line for line in full_output.splitlines()
                                        if not line.startswith('{"event":"command"') and
                                        not line.startswith('{"event":"state_trace"')]
                    output = "\n".join(diagnostic_lines)[-18000:]
                if name == "gate":
                    gate_output = output
                    if six_axis and not real_peer and trace_capture is not None:
                        trace_capture.extend(host_callback_events(full_output.splitlines()))
                print(f"{name} (returncode={process.poll()}):\n{output}", file=sys.stderr)
            raise
        finally:
            for process in reversed(processes):
                process.terminate()
            for process in reversed(processes):
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for log in logs.values(): log.close()


if __name__ == "__main__":
    main()
