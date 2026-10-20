##
# @file manual_moveit_execution.py
# @brief 校验规划折线并维护等待设备收据的分段执行进度。
# 所有路点采用六轴弧度；此模块不打开设备、不自行发送命令。路径几何校验、碰撞采样与真实完成确认是不同步骤，调用方须执行每一步。
"""按完整规划折线逐段执行；不持有设备接口，完成必须由调用方提供设备收据。"""
import math


##
# @brief 将完整路点复制为不可变元组并检查形状与有限性。
# @exception ValueError 路点不足、轴数不等于六或出现 NaN/Inf；不在此检查关节限位或碰撞。
# @param points 至少两个六轴路点，单位 rad。
# @return tuple[tuple]：按原顺序保留全部六轴路点。
def checked_points(points):
    result = tuple(tuple(q) for q in points)
    if len(result) < 2 or any(len(q) != 6 or not all(math.isfinite(v) for v in q) for q in result):
        raise ValueError('完整规划至少需要两个有限六轴路点')
    return result


##
# @brief 仅删除同一直线且方向单调的冗余采样。
# 投影误差容限为 1e-12 rad；不是允许绕过障碍物的近似简化。显式栈分割路径，避免长轨迹递归溢出。
# @param points 完整规划路点，六轴 rad。
# @return tuple[tuple]：保留首尾、几何拐点及折返处的执行折线。
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
        ##
        # ratio 是六轴线段投影参数，单调性用于保留共线往返；scale 把投影比例差转换回关节角误差尺度。
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


##
# @brief 按每轴最大步长对每段折线等分采样。
# 只生成供碰撞服务查询的状态，不判定碰撞；最多 20000 个采样，过长或非法步长抛出 ValueError。
# @param points 待检查折线，六轴 rad。
# @param step_rad 任一轴相邻采样允许的最大角差 rad，默认 0.5°。
# @return tuple[tuple]：包含首点、各段插值和末点的采样序列。
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


##
# @class SegmentedExecution
# @brief 将设备身份、起点一致性与逐段到位确认绑定。
# index 指向下一目标点；waiting 表示已取出该目标但尚未确认完成；aborted 为不可逆的本对象中止标志。真正设备取消/停车由调用方另行处理。
class SegmentedExecution:
    ##
    # @brief 冻结路径和执行身份并从第一条线段开始等待调度。
    # @param points 至少两点的完整执行折线，rad。
    # @param identity 调用方提供的 boot/reference/session 身份序列。
    # @param tolerance 实测位置与预期路点的逐轴允许偏差，rad，默认 0.003。
    # @return 无；通过实例状态或界面产生作用。
    def __init__(self, points, identity, tolerance=.003):
        self.points = checked_points(points)
        self.identity = tuple(identity)
        self.tolerance = tolerance
        self.index = 1
        self.waiting = False
        self.aborted = False

    ##
    # @brief 查询所有线段是否都已确认且未被中止。
    # @return bool：index 已越过末点且 aborted 为 False。
    @property
    def done(self):
        return not self.aborted and self.index == len(self.points)

    ##
    # @brief 校验执行身份和六轴实测位置与指定路点一致。
    # @exception ValueError 已中止、身份变化、实际值非有限或逐轴偏差超出 tolerance。此处不判断反馈时间或设备完成事件。
    # @param actual 六轴实测位置，rad，须由调用方保证反馈新鲜。
    # @param identity 此刻设备的 boot/reference/session 身份。
    # @param expected 本次应处于的原路径路点，rad。
    # @return 无；通过实例状态或界面产生作用。
    def check(self, actual, identity, expected):
        if self.aborted or tuple(identity) != self.identity:
            raise ValueError('执行已停止或 boot/reference/session 身份改变')
        if len(actual) != 6 or any(not math.isfinite(q) or abs(q-p) > self.tolerance for q,p in zip(actual, expected)):
            raise ValueError('实际起点或到位位置不匹配原规划路径')

    ##
    # @brief 核对当前段起点并将下一目标置为待确认。
    # waiting 时不得重复取目标；路径已完成时也拒绝。返回目标并不等于命令已发送或运动已开始。
    # @param actual 当前实测六轴位置，rad。
    # @param identity 当前设备身份。
    # @return tuple：下一段目标六轴 rad。
    def next_goal(self, actual, identity):
        if self.waiting or self.done:
            raise ValueError('前段尚未完成或路径已完成')
        self.check(actual, identity, self.points[self.index-1])
        self.waiting = True
        return self.points[self.index]

    ##
    # @brief 在调用方取得完成收据后确认本段到位。
    # 先核对待确认标志、身份和本段目标，再推进 index。调用方不能仅因位置接近就绕过设备收据直接确认。
    # @param actual 完成时实测六轴位置，rad。
    # @param identity 完成时设备身份。
    # @return 无；通过实例状态或界面产生作用。
    def confirm(self, actual, identity):
        if not self.waiting:
            raise ValueError('没有等待完成的运动段')
        self.check(actual, identity, self.points[self.index])
        self.index += 1
        self.waiting = False

    ##
    # @brief 标记本执行对象中止。
    # 只改变本地 aborted 标志；不发送设备停止、取消或失能命令，也不伪造已完成状态。
    # @return 无；通过实例状态或界面产生作用。
    def abort(self):
        self.aborted = True
