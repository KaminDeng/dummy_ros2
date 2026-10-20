"""验证跨规划曲线衔接的实际20ms命令序列。"""
##
# @file test_manual_moveit_blending.py
# @brief 拼接轨迹20ms离散命令与软件模拟器更新合同。
#
# 纯数值路径及SixAxisSimulator内存运行；写入tmp_path的文件绑定boot/reference/session并算SHA，但没有Native/ROS/CAN。rad位置、60deg/s上限、4rad/s²加速度和0.02s差分各用各单位。
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from manual_moveit_continuous import prepare_path, blend_paths


##
# @brief 构造J1精确梯形输入并prepare_path：0.3s加速/减速、60°/s巡航，起终速度0。
# @param start J1起点rad，其他轴为0。
# @param stop J1终点rad，距离必须使巡航段时长>0。
# @return PreparedPath，离散周期20ms、位置rad。
def straight(start,stop):
    # 精确梯形，含60°/s巡航；每个输入仍是静止起终点。
    delta=stop-start;sign=1 if delta>=0 else -1
    v=math.radians(60);ramp=.3;cruise=abs(delta)/v-ramp
    assert cruise>0
    q=lambda x:(x,0.,0.,0.,0.,0.)
    return prepare_path((q(start),q(start+sign*v*ramp/2),q(stop-sign*v*ramp/2),q(stop)),
                        (0.,ramp,ramp+cruise,2*ramp+cruise),60.,4.,
                        velocities=(q(0),q(sign*v),q(sign*v),q(0)))


##
# @brief 核拼接前缀/末尾保留、穿越旧1.4rad目标仍有速度，峰值接近60°/s且差分加速度≤4rad/s²。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_same_direction_crosses_old_goal_without_stopping_or_slowing_cruise():
    a=straight(0.,1.4);b=straight(1.4,2.8)
    joined,cut=blend_paths(a,b)
    assert joined.points[:cut+1]==a.points[:cut+1]
    assert joined.points[-2:]==b.points[-2:]
    velocities=[(q[0]-p[0])/.02 for p,q in zip(joined.points,joined.points[1:])]
    crossing=min(range(len(joined.points)),key=lambda i:abs(joined.points[i][0]-1.4))
    assert velocities[crossing]>.5
    assert math.degrees(max(velocities))>=59.
    assert max(map(abs,velocities))<=math.radians(60.)
    assert max(abs(b-a)/.02 for a,b in zip([0.]+velocities,velocities))<=4.


##
# @brief 将第二段转到J2，核转角前已检前缀不变、两轴同时变化且每轴离散速度/加速度有界。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_corner_blends_all_axes_and_preserves_checked_prefix():
    a=straight(0.,1.4)
    b=straight(0.,1.4)
    from manual_moveit_continuous import PreparedPath
    b=PreparedPath(tuple((1.4,p[0],0.,0.,0.,0.) for p in b.points),60.,4.)
    joined,cut=blend_paths(a,b)
    assert joined.points[:cut+1]==a.points[:cut+1]
    assert any(p[0]<1.4 and p[1]>0. for p in joined.points)
    speeds=[tuple((y-x)/.02 for x,y in zip(p,q)) for p,q in zip(joined.points,joined.points[1:])]
    assert all(max(map(abs,v))<=math.radians(60.) for v in speeds)
    assert all(max(abs(y-x)/.02 for x,y in zip(p,q))<=4. for p,q in zip([(0.,)*6]+speeds,speeds))


##
# @brief 断开的1.4→1.5rad端点或速度参数不一致必须ValueError，不私自接直线补齐。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_rejects_disconnected_or_mismatched_motion_parameters():
    a=straight(0.,1.4)
    with pytest.raises(ValueError):blend_paths(a,straight(1.5,2.9))
    from manual_moveit_continuous import PreparedPath
    with pytest.raises(ValueError):blend_paths(a,PreparedPath(a.points,30.,4.))


##
# @brief 模拟20次EXTEND_PATH滚动prefix与版本，重复同命令拒绝，执行中保持同motion，完成后motion才加1。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_simulator_twenty_updates_keep_one_motion_and_roll_prefix(tmp_path):
    from manual_panel_simulator import SixAxisSimulator
    from manual_moveit_continuous import PreparedPath,write_path
    cfg={'rest_deg':[0]*6,'home_deg':[0]*6,'joints':[{'limits_deg':[-180,180]}]*6}
    m=SixAxisSimulator(cfg);m.command('RESUME')
    path=straight(0.,1.4);file,sha=write_path(path,tmp_path,(m.boot,m.reference,m.session))
    assert m.command(f'MOVE_CHAIN 60 ACCEL 4 BUDGET 100 {sha} {file}')=='MOVE_STARTED'
    motion=m.motion;origin=0
    for version in range(2,22):
        m.advance(.3)
        end=path.points[-1][0];goal=0. if end>0.7 else 1.4
        joined,cut=blend_paths(path,straight(end,goal))
        trim=max(origin,int(m.elapsed/.02)-4)
        rolled=PreparedPath(joined.points[trim-origin:],60.,4.)
        file,sha=write_path(rolled,tmp_path,(m.boot,m.reference,m.session))
        command=f'EXTEND_PATH {motion} {version} {trim} {origin+cut} {sha} {file}'
        assert m.command(command)=='PATH_UPDATED'
        assert m.command(command)=='PATH_UPDATE_REJECTED'
        path=rolled;origin=trim
        assert m.motion==motion and m.trajectory is not None
    while m.trajectory:m.advance(.02)
    assert m.motion==motion+1


##
# @brief 改已约束前缀第10点的更新须拒绝且原sampled轨迹不变，不能因更新失败破坏原停止路径。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_simulator_rejects_changed_prefix_and_keeps_original_stop(tmp_path):
    from manual_panel_simulator import SixAxisSimulator
    from manual_moveit_continuous import PreparedPath,write_path
    cfg={'rest_deg':[0]*6,'home_deg':[0]*6,'joints':[{'limits_deg':[-180,180]}]*6}
    m=SixAxisSimulator(cfg);m.command('RESUME')
    path=straight(0.,1.4);file,sha=write_path(path,tmp_path,(m.boot,m.reference,m.session))
    assert m.command(f'MOVE_CHAIN 60 ACCEL 4 BUDGET 100 {sha} {file}')=='MOVE_STARTED'
    original=m.sampled
    changed=list(path.points);changed[10]=(.001,0.,0.,0.,0.,0.)
    file,sha=write_path(PreparedPath(tuple(changed),60.,4.),tmp_path,(m.boot,m.reference,m.session))
    assert m.command(f'EXTEND_PATH {m.motion} 2 0 30 {sha} {file}')=='PATH_UPDATE_REJECTED'
    assert m.sampled==original
