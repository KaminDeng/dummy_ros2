"""按完整规划折线逐段执行；不持有设备接口，完成必须由调用方提供设备收据。"""
import math


def checked_points(points):
    result = tuple(tuple(q) for q in points)
    if len(result) < 2 or any(len(q) != 6 or not all(math.isfinite(v) for v in q) for q in result):
        raise ValueError('完整规划至少需要两个有限六轴路点')
    return result


def execution_polyline(points):
    """逐段消除同一直线上的冗余采样；保留拐点及回头点。

    仅容许1e-12 rad浮点误差，不作近似平滑或碰撞捷径。用显式栈避免
    长路径递归溢出；参数单调检查避免共线的往返运动被误删。
    """
    points = checked_points(points)
    keep = {0, len(points)-1}
    pending = [(0, len(points)-1)]
    while pending:
        first, last = pending.pop()
        if last-first < 2:
            continue
        start, goal = points[first], points[last]
        delta = tuple(b-a for a,b in zip(start, goal))
        norm2 = sum(d*d for d in delta)
        scale = max(abs(d) for d in delta)
        worst, split = 0., None
        previous, reversal = 0., None
        for index in range(first+1, last):
            q = points[index]
            ratio = sum((v-a)*d for v,a,d in zip(q,start,delta))/norm2 if norm2 else 0.
            clipped = max(0., min(1., ratio))
            error = max(abs(v-(a+d*clipped)) for v,a,d in zip(q,start,delta))
            if error > worst:
                worst, split = error, index
            if reversal is None and (previous-ratio)*scale > 1e-12:
                reversal = index-1
            previous = ratio
        if worst <= 1e-12:
            split = reversal
        if split is not None:
            keep.add(split)
            pending.extend(((first, split), (split, last)))
    return tuple(points[index] for index in sorted(keep))


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
