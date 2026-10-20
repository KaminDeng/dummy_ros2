"""连续轨迹的实际下发序列；不连接硬件。"""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from manual_moveit_continuous import prepare_path, write_path


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


def test_bent_path_preserves_endpoint_and_bounds():
    points=((0.,)*6,(.1,0.,0.,0.,0.,0.),(.1,.1,0.,0.,0.,0.))
    path=prepare_path(points,(0.,1.,2.),60.,4.)
    assert path.points[-1]==points[-1]
    assert all(0.<=q[0]<=.1 and 0.<=q[1]<=.1 for q in path.points)


@pytest.mark.parametrize('times',[(0.,0.),(1.,0.),(0.,float('nan'))])
def test_rejects_bad_timing(times):
    with pytest.raises(ValueError):prepare_path(((0.,)*6,(.1,)*6),times,60.,4.)


def test_file_binds_identity_and_exact_payload(tmp_path):
    import hashlib,struct
    path=prepare_path(((0.,)*6,(.1,)*6),(0.,2.),60.,4.)
    filename,digest=write_path(path,tmp_path,(7,2,9))
    content=filename.read_bytes()
    assert hashlib.sha256(content).hexdigest()==digest
    assert struct.unpack('<8sIIQQQ',content[:40])==(b'DMYPTH1\0',len(path.points),20000,7,2,9)


def test_planned_velocities_preserve_sixty_degree_cruise_without_global_slowdown():
    import math
    vmax=math.radians(60.)
    points=((0.,)*6,(vmax*.15,0.,0.,0.,0.,0.),(vmax*1.15,0.,0.,0.,0.,0.),(vmax*1.3,0.,0.,0.,0.,0.))
    velocities=((0.,)*6,(vmax,0.,0.,0.,0.,0.),(vmax,0.,0.,0.,0.,0.),(0.,)*6)
    path=prepare_path(points,(0.,.3,1.3,1.6),60.,4.,velocities=velocities)
    peak=max(abs(b-a)/.02 for p,q in zip(path.points,path.points[1:]) for a,b in zip(p,q))
    assert math.degrees(peak)>=59.
    assert path.duration<=1.7


def test_totg_endpoint_residual_is_zeroed_before_full_sequence_validation():
    velocities=((.004,)*6,(0.,)*6)
    path=prepare_path(((0.,)*6,(.1,)*6),(0.,1.),60.,4.,velocities=velocities)
    assert path.points[0]==path.points[1]
    assert path.points[-1]==path.points[-2]
    with pytest.raises(ValueError):
        prepare_path(((0.,)*6,(.1,)*6),(0.,1.),60.,4.,velocities=((.03,)*6,(0.,)*6))
