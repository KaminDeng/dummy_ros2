##
# @file manual_moveit_continuous.py
# @brief 把完整规划轨迹准备为固定 20 ms 的设备命令序列。
# 保留实际要发送的插值曲线，检查离散速度/加速度并在需要时延长时间。准备、拼接和文件散列都不执行运动；调用方须对返回的实际命令折线重做碰撞检查。
"""固定控制周期的完整轨迹准备；碰撞检查必须针对返回的实际命令折线。"""
import bisect
import hashlib
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from manual_moveit_execution import checked_points


##
# @class PreparedPath
# @brief 封装不可重新绑定字段的已准备路径及运动参数。
# points 为六轴 rad 路点元组；speed 为 °/s；acceleration 为 rad/s²；period 为秒，常规准备固定为 0.02。frozen 约束字段赋值，调用方仍应传入不可变点序列。
@dataclass(frozen=True)
class PreparedPath:
    points: tuple
    speed: float
    acceleration: float
    period: float = .02

    ##
    # @brief 计算首末采样之间的总时长。
    # @return float：秒，等于 (路点数-1)×采样周期。
    @property
    def duration(self):
        return (len(self.points)-1)*self.period


##
# @brief 将 MoveIt 速度数组重排为面板指定的六轴顺序。
# 关节名称缺失会由 index 查找报错；此函数不做完整轨迹验证，调用方先使用 validate_trajectory。
# @param trajectory JointTrajectory；速度字段单位 rad/s。
# @param names 面板指定的六个关节名称，决定返回列顺序。
# @param required True 时缺失完整六轴速度必须报错。
# @return tuple[tuple]：重排速度，或可选模式下缺数据时返回 None。
def trajectory_velocities(trajectory,names,required=False):
    order=[list(trajectory.joint_names).index(name) for name in names]
    values=[getattr(point,'velocities',()) for point in trajectory.points]
    if not all(len(value)==6 for value in values):
        if required:
            raise ValueError('规划轨迹缺少完整速度，未授权连续执行；请重新规划')
        return None
    return tuple(tuple(value[index] for index in order) for value in values)


##
# @brief 按限速限加速度重采样完整规划并加入首末静止样本。
# 输入时间先减去首时刻；提供速度时容许首末 0.02 rad/s 内的数值残速，随后归零。逐步扩大时间尺度，最多尝试 32 次；轨迹须落在 630 s 和样本数预算内。
# @exception ValueError 输入非法或不能在预算内满足离散动态限制。
# @note Hermite 曲线可能偏离原路点折线；必须对返回 points 做限位及碰撞检查。
# @param points 至少两个完整六轴路点，单位 rad。
# @param times 与路点一一对应的严格递增有限时刻，单位秒，可有非零起点。
# @param speed 各关节速度上限 °/s，范围 (0,63]。
# @param acceleration 各关节加速度上限 rad/s²，范围 (0,4]。
# @param velocities 可选六轴 rad/s 速度；提供时用三次 Hermite 插值，否则逐段线性插值。
# @return PreparedPath：固定 0.02 s 周期的实际命令序列。
def prepare_path(points,times,speed,acceleration,velocities=None):
    points=checked_points(points)
    times=tuple(times)
    if (len(times)!=len(points) or not all(math.isfinite(t) for t in times) or
            any(b<=a for a,b in zip(times,times[1:])) or
            not math.isfinite(speed) or not 0.<speed<=63. or
            not math.isfinite(acceleration) or not 0.<acceleration<=4.):
        raise ValueError('连续轨迹时间、速度或加速度无效')
    times=tuple(t-times[0] for t in times)
    if velocities is not None:
        velocities=tuple(map(tuple,velocities))
        if (len(velocities)!=len(points) or any(len(v)!=6 or not all(math.isfinite(x) for x in v) for v in velocities) or
                any(abs(v)>.02 for v in velocities[0]+velocities[-1])):
            raise ValueError('连续轨迹速度信息无效或首末速度非零')
        # TOTG起点存在数值差分残速；实际入口是Hold，首末命令速度必须为零。
        # 调整后的完整序列仍逐点检查速度/加速度及碰撞，不把残速直接交给设备。
        velocities=((0.,)*6,)+velocities[1:-1]+((0.,)*6,)
    ##
    # speed 从 °/s 转为 rad/s 后与离散导数比较；1e-4 相对余量为数值留边，不改变传给设备的公开上限。
    vmax=math.radians(speed)*(1.-1e-4);amax=acceleration*(1.-1e-4)
    original_peak=max(abs(b-a)/(tb-ta) for p,q,ta,tb in zip(points,points[1:],times,times[1:]) for a,b in zip(p,q))
    scale=1.001 if velocities is not None else max(.001,original_peak/vmax)
    dt=.02
    for _ in range(32):
        duration=times[-1]*scale
        count=max(1,math.ceil(duration/dt))
        if (count+3)*dt>630. or count+4>40000:
            raise ValueError('连续轨迹超过原630秒运动预算')
        ##
        # 重复首点和末点生成一个周期的零位移，使设备命令首末速度为零；因此总时长还包含这些静止采样。
        sampled=[points[0],points[0]]
        for index in range(1,count):
            at=index*dt/scale
            segment=min(len(times)-2,max(0,bisect.bisect_right(times,at)-1))
            ratio=max(0.,min(1.,(at-times[segment])/(times[segment+1]-times[segment])))
            if velocities is None:
                q=tuple(a+(b-a)*ratio for a,b in zip(points[segment],points[segment+1]))
            else:
                u=ratio;duration_segment=times[segment+1]-times[segment]
                q=tuple((2*u**3-3*u*u+1)*a+(u**3-2*u*u+u)*duration_segment*va+
                        (-2*u**3+3*u*u)*b+(u**3-u*u)*duration_segment*vb
                        for a,b,va,vb in zip(points[segment],points[segment+1],velocities[segment],velocities[segment+1]))
            sampled.append(q)
        sampled.extend((points[-1],points[-1]))
        previous=(0.,)*6;peak_v=peak_a=0.
        for p,q in zip(sampled,sampled[1:]):
            velocity=tuple((b-a)/dt for a,b in zip(p,q))
            peak_v=max(peak_v,*map(abs,velocity))
            peak_a=max(peak_a,*(abs(b-a)/dt for a,b in zip(previous,velocity)))
            previous=velocity
        ratio=max(peak_v/vmax,math.sqrt(peak_a/amax))
        if ratio<=1.:
            return PreparedPath(tuple(sampled),speed,acceleration)
        scale*=max(1.02,ratio*1.02)
    raise ValueError('连续轨迹未能在原速度/加速度约束内重新定时')


##
# @brief 写入携带设备身份和 SHA-256 的二进制路径文件。
# 40 字节小端头之后每点 6 个 double，共 48 字节；文件名采用摘要前 16 位。函数不删除文件、不发送 MOVE_PATH，也不校验碰撞。
# @param path PreparedPath，命令点单位 rad；格式固定编码 20000 µs 周期。
# @param directory 调用方提供的已存在输出目录，常用 /tmp。
# @param identity 依次为 boot、reference、session 的三个无符号 64 位整数。
# @return tuple[Path,str]：实际写入文件路径与完整 SHA-256 十六进制摘要。
def write_path(path,directory,identity):
    ##
    # 二进制头依次为 magic[8]、点数 uint32、周期微秒 uint32、boot/reference/session uint64；不是 JSON 或 ROS 消息。
    header=struct.pack('<8sIIQQQ',b'DMYPTH1\0',len(path.points),20000,*identity)
    payload=header+b''.join(struct.pack('<6d',*point) for point in path.points)
    digest=hashlib.sha256(payload).hexdigest()
    filename=Path(directory)/('path-'+digest[:16]+'.bin')
    filename.write_bytes(payload)
    return filename,digest


##
# @brief 在不改已保留前缀的前提下用五次曲线衔接两段。
# 两段速度、加速度、周期必须相同；从较宽窗口向窄窗口搜索，并只延长新连接区时间。只核对离散动态限制，不验证身份、关节限位或碰撞。
# @exception ValueError 参数/端点不匹配或找不到满足原动态上限的连接。
# @param first 前段 PreparedPath；其尾部可被连接区替换。
# @param second 后段 PreparedPath；首点须与前段末点在 1e-9 rad 内一致。
# @param window 优先尝试替换的时间窗口，秒，默认 0.5。
# @return tuple[PreparedPath,int]：拼接路径及最后保留的前缀样本索引 cut。
def blend_paths(first, second, window=.5):
    """以五次曲线替换首段停车尾部和次段起步头部；调用方须重检返回曲线碰撞。

    返回首个修改样本前的索引，用于Native核对不可修改的命令前缀。
    不缩放已经提交的前缀，也不整体降低巡航速度。
    """
    if ((first.speed,first.acceleration,first.period)!=(second.speed,second.acceleration,second.period) or
            max(abs(a-b) for a,b in zip(first.points[-1],second.points[0]))>1e-9):
        raise ValueError('衔接轨迹端点或运动参数不一致')
    dt=first.period
    vmax=math.radians(first.speed);amax=first.acceleration
    # 从宽窗口开始，在有障碍时调用方可指定更小窗口重新尝试。
    maximum=min(int(window/dt),len(first.points)-4,len(second.points)-4)
    for width in range(maximum,3,-1):
        cut=len(first.points)-2-width;end=width+1
        if cut<2 or end+1>=len(second.points):continue
        p0=first.points[cut];p1=second.points[end]
        ##
        # @par 内部辅助函数 blend_paths.derivatives
        # 用中心差分估算连接边界速度和加速度。
        # 采样周期 dt 从 blend_paths 外层作用域取得。
        # 参数 points：六轴 rad 采样序列。
        # 参数 index：有前后邻居的样本索引。
        # 返回：tuple：六轴速度 rad/s 与加速度 rad/s²。
        def derivatives(points,index):
            v=tuple((b-a)/(2*dt) for a,b in zip(points[index-1],points[index+1]))
            a=tuple((r-2*q+p)/dt**2 for p,q,r in zip(points[index-1],points[index],points[index+1]))
            return v,a
        v0,a0=derivatives(first.points,cut);v1,a1=derivatives(second.points,end)
        # 保留原有时间尺度；轻微延长只作用于新建连接区。
        for count in range(width,2*width+21,2):
            duration=count*dt
            ##
            # 每轴的六个系数匹配连接区两端的位置、一阶和二阶导数；u 为无量纲归一化时间。
            coefficients=[]
            for x,y,v,w,a,b in zip(p0,p1,v0,v1,a0,a1):
                c0=x;c1=v*duration;c2=.5*a*duration**2
                d=y-c0-c1-c2;e=w*duration-c1-2*c2;f=b*duration**2-2*c2
                coefficients.append((c0,c1,c2,10*d-4*e+.5*f,-15*d+7*e-f,6*d-3*e+.5*f))
            bridge=[]
            for i in range(1,count):
                u=i/count
                bridge.append(tuple(sum(c*u**power for power,c in enumerate(axis)) for axis in coefficients))
            points=first.points[:cut+1]+tuple(bridge)+(p1,)+second.points[end+1:]
            previous=tuple((b-a)/dt for a,b in zip(points[cut-1],points[cut]))
            valid=True
            for p,q in zip(points[cut:],points[cut+1:]):
                velocity=tuple((b-a)/dt for a,b in zip(p,q))
                if (not all(math.isfinite(v) and abs(v)<=vmax for v in velocity) or
                        any(abs(b-a)/dt>amax for a,b in zip(previous,velocity))):
                    valid=False;break
                previous=velocity
            if valid:
                return PreparedPath(points,first.speed,first.acceleration,dt),cut
    raise ValueError('当前两段不能在原速度/加速度范围内平滑连接')
