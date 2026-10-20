"""连续轨迹的实际下发序列；不连接硬件。"""
##
# @file test_manual_moveit_continuous.py
# @brief 连续路径重采样、有限差分约束与文件绑定回归。
#
# 仅调用prepare_path/write_path，检查内存数值和临时二进制；没有轨迹设备发送。输入时间秒，路径点rad，speed参数deg/s，加速度rad/s²，文件周期字段µs。
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from manual_moveit_continuous import prepare_path, write_path


##
# @brief 101个密集点不逐点停车；总时长<3s、端点重复静止并核20ms差分速度/加速度。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_continuous_path_keeps_moving_across_dense_internal_samples():
    points=tuple((i*.001,0.,0.,0.,0.,0.) for i in range(101))
    path=prepare_path(points,tuple(i*.1 for i in range(101)),60.,4.)
    assert path.duration < 3.
    assert path.points[0]==path.points[1]==points[0]
    assert path.points[-1]==path.points[-2]==points[-1]
    assert any(p[0]!=q[0] for p,q in zip(path.points,path.points[1:]))
    velocity=[(b[0]-a[0])/.02 for a,b in zip(path.points,path.points[1:])]
    assert max(map(abs,velocity))<=1.04719756
    assert max(abs(b-a)/.02 for a,b in zip([0.]+velocity,velocity))<=4.


##
# @brief 折线路径最终到(0.1,0.1)rad且两轴不越输入包络，未检查障碍物。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_bent_path_preserves_endpoint_and_bounds():
    points=((0.,)*6,(.1,0.,0.,0.,0.,0.),(.1,.1,0.,0.,0.,0.))
    path=prepare_path(points,(0.,1.,2.),60.,4.)
    assert path.points[-1]==points[-1]
    assert all(0.<=q[0]<=.1 and 0.<=q[1]<=.1 for q in path.points)


##
# @brief 重复/倒序/NaN时间均须ValueError，拒绝无意义插值。
# @param times 参数化秒时间对：重复、倒序或含NaN。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
@pytest.mark.parametrize('times',[(0.,0.),(1.,0.),(0.,float('nan'))])
def test_rejects_bad_timing(times):
    with pytest.raises(ValueError):prepare_path(((0.,)*6,(.1,)*6),times,60.,4.)


##
# @brief 核整文件SHA及40字节小端头DMYPTH1、采样数、20000µs周期和身份(7,2,9)，不推导运行期身份仍有效。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_file_binds_identity_and_exact_payload(tmp_path):
    import hashlib,struct
    path=prepare_path(((0.,)*6,(.1,)*6),(0.,2.),60.,4.)
    filename,digest=write_path(path,tmp_path,(7,2,9))
    content=filename.read_bytes()
    assert hashlib.sha256(content).hexdigest()==digest
    assert struct.unpack('<8sIIQQQ',content[:40])==(b'DMYPTH1\0',len(path.points),20000,7,2,9)


##
# @brief 带端点速度的梯形输入保留≥59°/s巡航、时长≤1.7s，防止整体重复降速。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_planned_velocities_preserve_sixty_degree_cruise_without_global_slowdown():
    import math
    vmax=math.radians(60.)
    points=((0.,)*6,(vmax*.15,0.,0.,0.,0.,0.),(vmax*1.15,0.,0.,0.,0.,0.),(vmax*1.3,0.,0.,0.,0.,0.))
    velocities=((0.,)*6,(vmax,0.,0.,0.,0.,0.),(vmax,0.,0.,0.,0.,0.),(0.,)*6)
    path=prepare_path(points,(0.,.3,1.3,1.6),60.,4.,velocities=velocities)
    peak=max(abs(b-a)/.02 for p,q in zip(path.points,path.points[1:]) for a,b in zip(p,q))
    assert math.degrees(peak)>=59.
    assert path.duration<=1.7


##
# @brief TOTG端点0.004rad/s残量可归零并保留重复静止样本，0.03rad/s残量须拒绝。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_totg_endpoint_residual_is_zeroed_before_full_sequence_validation():
    velocities=((.004,)*6,(0.,)*6)
    path=prepare_path(((0.,)*6,(.1,)*6),(0.,1.),60.,4.,velocities=velocities)
    assert path.points[0]==path.points[1]
    assert path.points[-1]==path.points[-2]
    with pytest.raises(ValueError):
        prepare_path(((0.,)*6,(.1,)*6),(0.,1.),60.,4.,velocities=((.03,)*6,(0.,)*6))
