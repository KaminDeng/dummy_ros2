"""按完整规划折线逐段执行；不持有设备接口，完成必须由调用方提供设备收据。"""
import math


def checked_points(points):
    result = tuple(tuple(q) for q in points)
    if len(result) < 2 or any(len(q) != 6 or not all(math.isfinite(v) for v in q) for q in result):
        raise ValueError('完整规划至少需要两个有限六轴路点')
    return result


def execution_polyline(points):
    """只合并可证明同一直线且不回头的采样点；弯曲/绕行路径原样保留。"""
    points = checked_points(points)
    start, goal = points[0], points[-1]
    delta = tuple(b-a for a,b in zip(start, goal))
    axis = max(range(6), key=lambda i:abs(delta[i]))
    if abs(delta[axis]) < 1e-12:
        return points
    previous = 0.
    for q in points:
        ratio = (q[axis]-start[axis])/delta[axis]
        if ratio < previous or not 0. <= ratio <= 1.:
            return points
        if any(abs(v-(a+d*ratio)) > 1e-12 for v,a,d in zip(q,start,delta)):
            return points
        previous = ratio
    return (start, goal)


def collision_samples(points, step_rad=math.radians(.5)):
    points = checked_points(points)
    if not math.isfinite(step_rad) or step_rad <= 0:
        raise ValueError('碰撞采样步长无效')
    result = [points[0]]
    for start, goal in zip(points, points[1:]):
        count = max(1, math.ceil(max(abs(a-b) for a,b in zip(start, goal)) / step_rad))
        if len(result)+count > 20000:
            raise ValueError('路径过长，请缩短单次规划范围')
        result.extend(tuple(a+(b-a)*k/count for a,b in zip(start, goal)) for k in range(1, count+1))
    return tuple(result)


class SegmentedExecution:
    def __init__(self, points, identity, tolerance=.003):
        self.points = checked_points(points)
        self.identity = tuple(identity)
        self.tolerance = tolerance
        self.index = 1
        self.waiting = False
        self.aborted = False

    @property
    def done(self):
        return not self.aborted and self.index == len(self.points)

    def check(self, actual, identity, expected):
        if self.aborted or tuple(identity) != self.identity:
            raise ValueError('执行已停止或 boot/reference/session 身份改变')
        if len(actual) != 6 or any(not math.isfinite(q) or abs(q-p) > self.tolerance for q,p in zip(actual, expected)):
            raise ValueError('实际起点或到位位置不匹配原规划路径')

    def next_goal(self, actual, identity):
        if self.waiting or self.done:
            raise ValueError('前段尚未完成或路径已完成')
        self.check(actual, identity, self.points[self.index-1])
        self.waiting = True
        return self.points[self.index]

    def confirm(self, actual, identity):
        if not self.waiting:
            raise ValueError('没有等待完成的运动段')
        self.check(actual, identity, self.points[self.index])
        self.index += 1
        self.waiting = False

    def abort(self):
        self.aborted = True
