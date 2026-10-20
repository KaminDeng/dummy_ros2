#!/usr/bin/env python3
##
# @file verify_rest_planning.py
# @brief 通过只读MoveIt服务核精确Home/REST、负例和双向离散轨迹。
#
# 执行时会创建ROS节点、查询服务端模型参数、调用规划/有效性服务并写报告，不创建执行Action客户端或串口。
# 只读指不发运动命令/不改全局规划场景；服务请求仍执行实际规划计算。所有原始TOTG路点及插值样本有效不等于连续全路径严格碰撞证明或实机资格。
"""@file verify_rest_planning.py
@brief 使用真实 MoveIt 服务只读验证精确 REST 的双向规划及整条插值路径。

source ROS 与当前工作区安装环境后运行，例如：
ROS_DOMAIN_ID=187 python3 scripts/dummy/verify_rest_planning.py --output /tmp/rest-planning.json
脚本复用面板生产规划器、轨迹校验与碰撞采样；不创建执行客户端或 USB 连接。
每次有效性请求保留生产规划器附带的 55 mm 地面保护。任何超时、碰撞或负例失效均非零退出，
并将失败阶段与已有证据写入 --output。
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
##
# @brief 已复核的六轴自碰撞负例角度（度）；run转换为rad并要求同时出现自接触及地面接触。
# 它是回归负例，不是可执行目标或安全姿态推荐。
SELF_COLLISION_DEG = (-47.496, 18.324, 176.762, -87.374, 12.443, 22.388)


##
# @brief 解析有限超时与独立ROS域。
# @return Namespace；域必须0..232，各层等待预算为有限正秒数。
# 发现/请求/完整规划/全程四层期限相互独立，后续都受总monotonic截止约束；未在此连接ROS。
def arguments():
    """@brief 解析配置、ROS 域、输出及各层有限等待预算。
    @return argparse 命名空间；所有超时须为有限正秒数。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'src/dummy_bringup/config/manual_joint_panel.json')
    parser.add_argument('--output', type=Path, required=True, help='成功或失败的 JSON 证据路径')
    parser.add_argument('--domain-id', type=int, default=187, help='独立只读 MoveIt 所在 ROS 域，默认 187')
    parser.add_argument('--service-timeout', type=float, default=10.0, help='服务发现超时，秒')
    parser.add_argument('--request-timeout', type=float, default=10.0, help='每个状态有效性请求超时，秒')
    parser.add_argument('--plan-timeout', type=float, default=30.0, help='每次完整生产规划重试链超时，秒')
    parser.add_argument('--total-timeout', type=float, default=240.0, help='整个 ROS 回归超时，秒')
    result = parser.parse_args()
    if not 0 <= result.domain_id <= 232:
        parser.error('ROS 域必须位于 0..232')
    for name in ('service_timeout', 'request_timeout', 'plan_timeout', 'total_timeout'):
        if not math.isfinite(getattr(result, name)) or getattr(result, name) <= 0.0:
            parser.error('所有超时必须是有限正秒数')
    return result


##
# @brief 等待一个只读服务或组合future，受单请求及全程双重期限限制。
# @param node 用于spin_once推进回调的ROS节点。
# @param future planner组合或普通异步future。
# @param timeout 本请求最多等待秒数。
# @param deadline 全程monotonic绝对秒截止。
# @param label 超时/无响应的证据阶段标签。
# @return 实际非None响应；未完成时cancel本地future并抛TimeoutError。
# future.cancel不证明远端服务计算已撤销，也不把它当设备停止；此脚本没有运动执行请求。rclpy提前结束也会走未完成失败路径。
def wait_result(node, future, timeout, deadline, label):
    """@brief 有界 spin 并取得 ROS/生产组合 future 的实际响应。
    @param node 仅用于只读规划和有效性服务的 ROS 节点。
    @param future 等待的异步请求，支持生产规划器的组合 Future。
    @param timeout 本请求的秒数上限。
    @param deadline 全程 monotonic 绝对截止秒数。
    @param label 出错时保留的阶段名。
    @return 非空服务响应；超时取消请求并抛异常，不把未完成请求视为成功。
    """
    import rclpy
    end = min(time.monotonic() + timeout, deadline)
    while not future.done() and rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))
    if not future.done():
        future.cancel()
        raise TimeoutError(f'{label} 超时')
    response = future.result()
    if response is None:
        raise RuntimeError(f'{label} 没有响应')
    return response


##
# @brief 保留生产规划器地面保护并记录状态有效性响应。
# @param node 服务回调spin节点。
# @param planner 生产MoveItPlanner。
# @param positions 六轴位置rad，顺序与planner.names一致。
# @param args 含request_timeout的CLI配置。
# @param deadline 全程monotonic绝对截止秒数。
# @param label 该采样点的证据标签。
# @return 可JSON序列化的label/positions_rad/valid/contacts映射；调用者必须检查valid。
# contact.depth以m记录；单个状态有效不推导相邻未采样区间必然无碰撞。
def validity(node, planner, positions, args, deadline, label):
    """@brief 保留原地面保护请求状态有效性，并完整记录 contacts。
    @param positions 按生产 joint_names 顺序的六轴弧度。
    @param label 证据中的状态/轨迹采样标识。
    @return 含 valid、关节角及接触对的 JSON 可序列化记录；调用方必须检查 valid。
    """
    response = wait_result(node, planner.request_validity(positions), args.request_timeout, deadline, label)
    return {
        'label': label, 'positions_rad': list(positions), 'valid': bool(response.valid),
        'contacts': [{'body_1': contact.contact_body_1, 'body_2': contact.contact_body_2,
                      'depth_m': float(contact.depth)} for contact in response.contacts],
    }


##
# @brief 读取共同服务提供者实际URDF/SRDF并绑定当前安装网格文件。
# @param node 独立域只读ROS节点。
# @param args 服务发现/请求超时配置。
# @param deadline 全程绝对monotonic秒截止。
# @return provider、模型全文/规范SHA及包内资源原字节SHA；不符当前源码/两份修复清单时失败。
# 参数源固定/manual_move_group，要求它同时提供plan/validity服务；GetParameters client在finally销毁。
# URDF/SRDF用XML canonicalize(strip_text=True)比较，网格SHA取当前安装路径文件，不能单独证明旧进程缓存BVH已更新；后续行为正负例仍必须过。
def runtime_identity(node, args, deadline):
    """@brief 绑定服务提供者的实际 URDF/SRDF 及当前安装包内引用的网格摘要。
    @details 参数只记录服务端模型；网格摘要来自其 URI 对应的当前安装文件，
    不能单独证明旧进程内的 BVH 已刷新。后续 REST/路径/干涉负例还必须实际通过。
    @return 服务端、模型和资源的身份原件，模型与当前源码不一致时拒绝继续。
    """
    import xml.etree.ElementTree as ET
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from rcl_interfaces.srv import GetParameters
    provider = '/manual_move_group'
    services = dict(node.get_service_names_and_types_by_node('manual_move_group', '/'))
    if not {'/check_state_validity', '/plan_kinematic_path'} <= set(services):
        raise RuntimeError('manual_move_group 不是本轮两个规划服务的共同提供者')
    client = node.create_client(GetParameters, provider + '/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=min(args.service_timeout, max(0., deadline-time.monotonic()))):
            raise TimeoutError('服务端模型参数发现超时')
        response = wait_result(node, client.call_async(GetParameters.Request(
            names=['robot_description', 'robot_description_semantic'])),
            args.request_timeout, deadline, 'runtime_model_identity')
        if len(response.values) != 2 or any(v.type != 4 or not v.string_value for v in response.values):
            raise RuntimeError('服务端未提供完整 URDF/SRDF 字符串')
        urdf, srdf = (v.string_value for v in response.values)
        source = xacro.process_file(str(ROOT/'src/dummy-ros2_description/urdf/dummy-ros2.xacro')).toxml()
        semantic = (ROOT/'src/dummy_moveit_config/config/dummy-ros2.srdf').read_text()
        ##
        # @par 内部canonical回调
        # 输入XML文本，返回去文本边缘空白的规范XML；摘要是规范文本SHA，与源xacro/原网格文件SHA不同。
        canonical = lambda text: ET.canonicalize(text, strip_text=True)
        if canonical(urdf) != canonical(source) or canonical(srdf) != canonical(semantic):
            raise RuntimeError('实际服务端 URDF/SRDF 与本次修复源码不一致')
        resources = []
        for uri in sorted({m.get('filename') for m in ET.fromstring(urdf).findall('.//mesh')}):
            if not uri.startswith('package://'):
                raise RuntimeError('服务端模型引用非包内网格：' + uri)
            package, relative = uri[len('package://'):].split('/', 1)
            path = Path(get_package_share_directory(package))/relative
            resources.append({'uri': uri, 'resolved_path': str(path.resolve(strict=True)),
                              'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
        expected = json.loads((ROOT/'src/dummy-ros2_description/meshes/rest-collision-refinement.json').read_text())
        for suffix, key in [('full_step_link1-collision-local.stl', 'output_sha256'),
                            ('full_step_link1-collision-local-clearance-5mm.stl', 'guard_sha256')]:
            matches = [r for r in resources if r['uri'].endswith('/' + suffix)]
            if len(matches) != 1 or matches[0]['sha256'] != expected[key]:
                raise RuntimeError('服务端引用的安装资源与修复清单不匹配：' + suffix)
        return {'provider': provider, 'urdf': urdf, 'srdf': srdf,
                'urdf_canonical_sha256': hashlib.sha256(canonical(urdf).encode()).hexdigest(),
                'srdf_canonical_sha256': hashlib.sha256(canonical(srdf).encode()).hexdigest(),
                'source_matches': True, 'referenced_package_resources': resources,
                'mesh_hash_scope': 'current referenced package files; cached BVH additionally checked by behavioral regression'}
    finally:
        node.destroy_client(client)


##
# @brief 核生产完整规划的所有原始路点及插值样本。
# @param node 只读ROS节点。
# @param planner 生产规划器及其限位。
# @param start 指定六轴起点rad。
# @param goal 精确六轴终点rad，不用邻近REST替代。
# @param args 规划/有效性超时配置。
# @param deadline 全程绝对monotonic秒截止。
# @param record 可变方向证据对象，逐步写入，失败保留已完成部分。
# @return None；任一路点/采样/端点或服务错误抛异常。
# validate_trajectory检查结构/时间/限位，碰撞由后续validity逐样本检查，两者职责分开。
# 代码端点阈值实际为0.000101rad（在规划约束0.0001rad旁留数值余量）；采样最大步0.01rad，默认生产采样0.5°。
def verify_plan(node, planner, start, goal, args, deadline, record):
    """@brief 检查生产规划成功、完整 TOTG 路点和逐段插值均有效。
    @param start 起点六轴弧度。
    @param goal 精确目标六轴弧度，不改 REST 角度或取邻近姿态。
    @param record 即时填充的方向证据，即使异常也保留已检查样本。
    @details 不压缩折线、不丢弃任何 TOTG 路点；每个状态请求都必须 valid。
    相邻采样最大轴差不得超过 0.01 rad，起终点须在生产规划约束的 0.0001 rad 内。
    """
    from manual_moveit_execution import collision_samples
    from manual_moveit_planner import validate_trajectory
    response = wait_result(node, planner.request_plan(start, goal), args.plan_timeout, deadline, record['direction'])
    plan = response.motion_plan_response
    record.update(error_code=int(plan.error_code.val), planning_time_s=float(plan.planning_time))
    if plan.error_code.val != 1:
        raise RuntimeError(f"{record['direction']} 规划失败：{plan.error_code.val}")
    trajectory = plan.trajectory.joint_trajectory
    points = validate_trajectory(trajectory, planner.names, planner.limits)
    record['waypoints'] = [
        {'positions_rad': list(q), 'time_from_start_s': point.time_from_start.sec + point.time_from_start.nanosec / 1e9}
        for q, point in zip(points, trajectory.points)]
    if len(points) < 2:
        raise RuntimeError('双向非零运动规划未返回完整多点轨迹')
    if max(abs(a - b) for a, b in zip(points[0], start)) > 0.000101:
        raise RuntimeError('规划首点偏离指定起点')
    if max(abs(a - b) for a, b in zip(points[-1], goal)) > 0.000101:
        raise RuntimeError('规划末点偏离精确目标')
    # 使用生产采样器的默认 0.5°（约 0.008727 rad），逐段包含每个原始路点。
    samples = collision_samples(points)
    max_step = max(max(abs(a - b) for a, b in zip(first, second))
                   for first, second in zip(samples, samples[1:]))
    record.update(waypoint_count=len(points), sample_count=len(samples), max_sample_step_rad=max_step,
                  sampling='manual_moveit_execution.collision_samples(default 0.5 degree)', samples=[])
    if max_step > 0.01 + 1e-12:
        raise RuntimeError('生产碰撞采样超过 0.01 rad 步长')
    # 顺序匹配以证明所有 TOTG 路点都保留，避免只验证简化路径或首末点。
    ##
    # @brief 按顺序在样本中寻找全部原始TOTG路点，容差1e-12rad，避免只验证压缩轨迹。
    # 即使全部采样有效，仍没有对两样本间连续体积做严格扫掠碰撞证明。
    cursor = 0
    for point in points:
        while cursor < len(samples) and max(abs(a - b) for a, b in zip(point, samples[cursor])) > 1e-12:
            cursor += 1
        if cursor == len(samples):
            raise RuntimeError('碰撞采样遗漏原始 TOTG 路点')
    record['all_totg_waypoints_included'] = True
    for index, q in enumerate(samples):
        check = validity(node, planner, q, args, deadline, f"{record['direction']}:{index}")
        record['samples'].append(check)
        if not check['valid']:
            raise RuntimeError(f"{record['direction']} 插值采样 {index} 无效：{check['contacts']}")
    record['all_samples_valid'] = True


##
# @brief 在独立域创建规划/状态检查客户端并执行双向回归。
# @param args 配置路径、域及超时参数。
# @param evidence 逐步填入模型/状态/负例/轨迹的可变报告。
# @return None；任何配置、服务、正负例或路径失败抛异常，finally销毁节点并shutdown ROS。
# 读取配置中的名称、角限位/Home/REST，角度配置转换rad；仅核长度/唯一名称等当前代码前提。
# 每服务要求恰好一个提供者；检查随请求附带的manual_base_floor且负例同时存在自碰撞和地面contact，防止通过删保护或放宽豁免制造假正例。
def run(args, evidence):
    """@brief 在指定独立域创建只读客户端，验证正例、干涉负例及双向全轨迹。
    @param args 命令行配置和有界超时。
    @param evidence 持续记录的输出对象。
    @details 不订阅实机状态、不调用动作执行，不改变规划场景或地面豁免。
    ROS 服务必须恰有一个提供者，避免多个模型服务随机回应削弱回归结论。
    """
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    sys.path.insert(0, str(ROOT / 'src/dummy_bringup/scripts'))
    import rclpy
    from manual_moveit_planner import MoveItPlanner
    config = json.loads(args.config.read_text())
    names = [joint['name'] for joint in config['joints']]
    limits = [joint['limits_deg'] for joint in config['joints']]
    home, rest = (tuple(math.radians(q) for q in config[key]) for key in ('home_deg', 'rest_deg'))
    if len(names) != 6 or len(set(names)) != 6 or len(home) != 6 or len(rest) != 6:
        raise ValueError('配置必须包含完整六轴名称与精确 Home/Rest')
    evidence.update(config=str(args.config.resolve()), config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                    joint_names=names, home_deg=config['home_deg'], rest_deg=config['rest_deg'],
                    planner_source=str(ROOT / 'src/dummy_bringup/scripts/manual_moveit_planner.py'),
                    states={}, plans=[], negative={})
    deadline = time.monotonic() + args.total_timeout
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('verify_rest_planning_readonly')
        planner = MoveItPlanner(node, names, limits)
        ##
        # @par 内部motion_parameters回调
        # 无参数，返回默认速度deg/s、默认加速度rad/s²、最大速度deg/s、最大加速度rad/s²四元组，给生产规划器统一使用。
        # 此替换只提供当前配置数值，不授予实际执行速度许可。
        planner.motion_parameters = lambda: (
            config['motion_speed_default_deg_s'], config['motion_acceleration_default_rad_s2'],
            config['motion_speed_max_deg_s'], config['motion_acceleration_max_rad_s2'])
        for label, client in (('validity', planner.validity), ('plan', planner.plan)):
            timeout = min(args.service_timeout, max(0.0, deadline - time.monotonic()))
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f'{label} 服务发现超时')
            if node.count_services(client.srv_name) != 1:
                raise RuntimeError(f'{client.srv_name} 需要恰好一个服务端')
        evidence['runtime_identity'] = runtime_identity(node, args, deadline)
        floor = planner.robot_state(home).attached_collision_objects
        if len(floor) != 1 or floor[0].object.id != 'manual_base_floor':
            raise RuntimeError('生产规划器未携带原地面保护')
        evidence['floor_guard'] = {
            'id': floor[0].object.id, 'attached_link': floor[0].link_name,
            'touch_links': list(floor[0].touch_links),
            'box_dimensions_m': list(floor[0].object.primitives[0].dimensions),
            'box_center_z_m': floor[0].object.primitive_poses[0].position.z,
        }
        for label, q in (('home', home), ('rest', rest)):
            check = validity(node, planner, q, args, deadline, label)
            evidence['states'][label] = check
            if not check['valid']:
                raise RuntimeError(f'{label} 无效：{check["contacts"]}')
        negative = validity(node, planner, tuple(map(math.radians, SELF_COLLISION_DEG)), args, deadline, 'known_self_collision')
        evidence['negative'] = negative
        if negative['valid'] or not negative['contacts']:
            raise RuntimeError('已知自碰撞姿态未被拒绝，或缺少接触证据')
        # 已知负例须含机器人之间的接触，不能仅靠地面碰撞使其 invalid。
        negative['self_contacts'] = [contact for contact in negative['contacts']
                                     if 'manual_base_floor' not in (contact['body_1'], contact['body_2'])]
        if not negative['self_contacts']:
            raise RuntimeError('已知自碰撞负例仅返回地面接触')
        # 域 187 真实服务复核该姿态还会触及地面保护；必须保留这条 contact，
        # 即使自碰撞仍让它 invalid，也不能掩盖地面保护被移除或增加豁免。
        floor_contacts = [contact for contact in negative['contacts']
                          if 'manual_base_floor' in (contact['body_1'], contact['body_2'])]
        evidence['ground_negative'] = {
            'pose': 'known_self_collision', 'positions_rad': negative['positions_rad'],
            'valid': negative['valid'], 'floor_contacts': floor_contacts,
        }
        if not floor_contacts:
            raise RuntimeError('已复核的干涉姿态缺少原地面保护接触')
        for label, start, goal in (('home_to_rest', home, rest), ('rest_to_home', rest, home)):
            record = {'direction': label}
            evidence['plans'].append(record)
            verify_plan(node, planner, start, goal, args, deadline, record)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


##
# @brief 归档本次只读回归成功或已捕获失败。
# @return run全部通过0，否则1；参数解析和报告写入本身的错误仍可能在捕获范围之外退出。
# 只捕获run中的Exception后写error type/message；allow_nan=False拒绝报告非有限数。
# 输出success只覆盖本脚本实际检查的服务模型/正负例/离散样本；executed=false表示没有执行运动，而非没有调用规划服务。
def main():
    """@brief 无论回归成功或失败都保存 JSON，并用退出码表达判定。
    @return 完整回归成功为 0；任何配置、服务、轨迹或负例错误为 1。
    """
    args = arguments()
    started = time.monotonic()
    evidence = {'schema': 'dummy.rest_planning.readonly.v1', 'readonly': True, 'executed': False,
                'ros_domain_id': args.domain_id, 'started_utc': datetime.now(timezone.utc).isoformat(),
                'success': False, 'timeouts_s': {'service': args.service_timeout,
                    'request': args.request_timeout, 'plan': args.plan_timeout, 'total': args.total_timeout}}
    try:
        run(args, evidence)
        evidence['success'] = True
    except Exception as error:
        evidence['error'] = {'type': type(error).__name__, 'message': str(error)}
    evidence['elapsed_s'] = time.monotonic() - started
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'success': evidence['success'], 'output': str(args.output.resolve()),
                      'elapsed_s': evidence['elapsed_s'], 'error': evidence.get('error')}, ensure_ascii=False))
    return 0 if evidence['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
