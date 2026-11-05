"""MoveIt辅助命令的JTC五次样条约束；不改变路径、设备上限或控制器准入。"""
import copy
import math


def _value(poly, x):
    value = 0.0
    for coefficient in reversed(poly):
        value = value * x + coefficient
    return value


def _derivative(poly):
    return [i * poly[i] for i in range(1, len(poly))]


def _roots(poly):
    """在归一段[0,1]内按导数临界点分隔、二分求根，最高仅四次。"""
    while len(poly) > 1 and poly[-1] == 0:
        poly = poly[:-1]
    if len(poly) <= 1:
        return []
    if len(poly) == 2:
        root = -poly[0] / poly[1]
        return [root] if 0 <= root <= 1 else []
    boundaries = [0.0, *_roots(_derivative(poly)), 1.0]
    result = []
    tolerance = 1e-13 * max(1.0, sum(map(abs, poly)))
    for x in boundaries:
        if abs(_value(poly, x)) <= tolerance:
            result.append(x)
    for lo, hi in zip(boundaries, boundaries[1:]):
        low = _value(poly, lo)
        if low * _value(poly, hi) >= 0:
            continue
        for _ in range(48):
            mid = (lo + hi) / 2
            if low * _value(poly, mid) <= 0:
                hi = mid
            else:
                lo = mid
                low = _value(poly, lo)
        result.append((lo + hi) / 2)
    return sorted(set(result))


def _peak(poly):
    return max(abs(_value(poly, x)) for x in [0.0, 1.0, *_roots(_derivative(poly))])


def _segments(trajectory):
    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in trajectory.points]
    segments = []
    for index, (left, right) in enumerate(zip(trajectory.points, trajectory.points[1:])):
        duration = times[index + 1] - times[index]
        axes = []
        for p, q, v, w, a, b in zip(left.positions, right.positions, left.velocities,
                                    right.velocities, left.accelerations, right.accelerations):
            d, v, w, a, b = q - p, v * duration, w * duration, a * duration**2, b * duration**2
            poly = [p, v, a / 2, 10*d-6*v-4*w-1.5*a+.5*b,
                    -15*d+8*v+7*w+1.5*a-b, 6*d-3*v-3*w-.5*a+.5*b]
            if not all(math.isfinite(value) for value in poly) or not math.isfinite(sum(map(abs, poly))):
                raise ValueError('nonfinite spline coefficients')
            axes.append(poly)
        segments.append((times[index], times[index + 1], axes))
    return times, segments


def _bounds(trajectory):
    axes = len(trajectory.joint_names)
    maximum_v, maximum_a = [0.0]*axes, [0.0]*axes
    times, segments = _segments(trajectory)
    for begin, end, coefficients in segments:
        for axis, poly in enumerate(coefficients):
            velocity = _derivative(poly)
            maximum_v[axis] = max(maximum_v[axis], _peak(velocity)/(end-begin))
            maximum_a[axis] = max(maximum_a[axis], _peak(_derivative(velocity))/(end-begin)**2)
    # 明确包括起点静止、终点保持及四个20ms采样相位；连续峰值另保证任意相位。
    sampled_v, sampled_a = [0.0]*axes, [0.0]*axes
    for phase in (0.0, .005, .010, .015):
        prior = list(trajectory.points[0].positions)
        prior_v = [0.0]*axes
        segment = 0
        for step in range(math.ceil((times[-1]+.06)/.02)+1):
            instant = phase + step*.02
            while segment+1 < len(segments) and instant > segments[segment][1]:
                segment += 1
            if instant >= times[-1]:
                positions = trajectory.points[-1].positions
            else:
                begin, end, coefficients = segments[segment]
                positions = [_value(poly, (instant-begin)/(end-begin)) for poly in coefficients]
            velocity = [(p-q)/.02 for p, q in zip(positions, prior)]
            for axis, (v, old) in enumerate(zip(velocity, prior_v)):
                sampled_v[axis] = max(sampled_v[axis], abs(v))
                sampled_a[axis] = max(sampled_a[axis], abs(v-old)/.02)
            prior, prior_v = list(positions), velocity
    return maximum_v, maximum_a, sampled_v, sampled_a


##
# @brief 以统一时间比例约束实际JTC样条及20ms位置差分，不清零MoveIt原首切向。
# @param trajectory 从零时刻开始、末端静止的完整p/v/a轨迹；输入保持不变。
# @param velocity_caps 已证明的逐轴速度帽；缺省采用原规划点峰值。
# @param acceleration_caps 已证明的逐轴加速度帽；缺省采用原规划点峰值。
# @return 新轨迹和连续区间／20ms全相位保守边界证据。
# @note 连续峰值只指样条区间；首末速度与外部hold的跳变单独计入离散界。
# 未知controller首位置偏差仍由真实controller/Core准入校验，不由本函数替代。
def retime(trajectory, velocity_caps=None, acceleration_caps=None):
    """整体放慢完整p/v/a轨迹，保留所有位置；时间舍入后重新证明连续和20ms界。

    未显式给帽时采用规划点已声明的逐轴峰值，绝不抬高规划器原限额。
    此函数不做碰撞/位置/Source检查；调用方原验证及真实控制器准入仍必须执行。
    """
    count = len(trajectory.joint_names)
    if not count or not 2 <= len(trajectory.points) <= 512:
        raise ValueError('retime requires 2..512 trajectory points')
    times = []
    for point in trajectory.points:
        if any(len(getattr(point, name)) != count or not all(math.isfinite(v) for v in getattr(point, name))
               for name in ('positions', 'velocities', 'accelerations')):
            raise ValueError('retime requires finite complete position/velocity/acceleration vectors')
        duration = point.time_from_start
        if duration.sec < 0 or not 0 <= duration.nanosec < 10**9:
            raise ValueError('invalid trajectory duration')
        times.append(duration.sec*10**9+duration.nanosec)
    if times[0] != 0 or any(b <= a for a, b in zip(times, times[1:])) or times[-1] > 120*10**9:
        raise ValueError('retime requires zero origin and increasing bounded times')
    if any(abs(v) > 1e-10 for v in trajectory.points[-1].velocities):
        raise ValueError('retime requires stationary end for hold transition')
    caps = []
    for given, name in ((velocity_caps, 'velocities'), (acceleration_caps, 'accelerations')):
        values = list(given) if given is not None else [max(abs(getattr(p, name)[i]) for p in trajectory.points) for i in range(count)]
        if len(values) != count or any(not math.isfinite(v) or v < 0 for v in values):
            raise ValueError('invalid trajectory derivative caps')
        caps.append(values)
    original = copy.deepcopy(trajectory)
    result, scale = copy.deepcopy(original), 1.0
    for _ in range(5):
        v, a, sv, sa = _bounds(result)
        # 连续段a与hold边界速度跳变J经过20ms二阶位置差分：
        # |Δv/h| ≤ a_max + (|v_start|+|v_end|)/h，对任意采样相位成立。
        jump = [abs(result.points[0].velocities[i])+abs(result.points[-1].velocities[i])
                for i in range(count)]
        boundary = [value/.02 for value in jump]
        all_phase_a = [a[i]+boundary[i] for i in range(count)]
        factor = 1.0
        for i in range(count):
            for observed, cap, exponent in ((max(v[i],sv[i]),caps[0][i],1), (max(a[i],sa[i]),caps[1][i],.5)):
                if cap == 0:
                    if observed > 0:
                        raise ValueError('moving axis has no proven derivative cap')
                else:
                    factor = max(factor, (observed/(cap*(1-1e-6)))**exponent)
            cap = caps[1][i]
            if cap == 0:
                if all_phase_a[i] > 0:
                    raise ValueError('moving boundary has no proven acceleration cap')
            else:
                limit = cap*(1-1e-6)
                # 统一缩时f后a/f² + boundary/f≤limit，取正根，避免反复猜倍率。
                b = boundary[i]/limit
                factor = max(factor, .5*(b+math.hypot(b, 2*math.sqrt(a[i]/limit))))
        if factor <= 1.0:
            return result, {'time_scale': scale, 'velocity_caps': caps[0], 'acceleration_caps': caps[1],
                            'continuous_max_velocity': v, 'continuous_max_acceleration': a,
                            'sampled_max_velocity': sv, 'sampled_max_acceleration': sa,
                            'sampled_acceleration_bound_all_phases': all_phase_a,
                            'hold_boundary_velocity_jump': jump,
                            'sample_period_ns': 20_000_000, 'position_encoding': 'ROS float64 / Root double'}
        scale *= factor * (1+1e-6)
        if times[-1]*scale > 120*10**9:
            raise ValueError('retimed trajectory exceeds existing 120s bound')
        for source, dest, ns in zip(original.points, result.points, times):
            dest.time_from_start.sec, dest.time_from_start.nanosec = divmod(round(ns*scale),10**9)
            dest.velocities = [v/scale for v in source.velocities]
            dest.accelerations = [a/scale**2 for a in source.accelerations]
    raise ValueError('retimed spline failed derivative bounds after nanosecond rounding')
