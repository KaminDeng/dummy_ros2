##
# @file test_ground_origin.py
# @brief 用独立CAD底面界限验证地面原点与整套根几何刚体平移。
# @details OBJ坐标以mm计，URDF原点以m计；几何/质心/保护体一起移，J1相对安装关系保留。

"""用完整CAD的独立底面证据检查地面原点，保持原机械装配关系。"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


DESCRIPTION = Path(__file__).resolve().parents[2] / 'dummy-ros2_description'
SHIFT = (0.019225, 0.000523, -0.004684)


##
# @brief 把origin/axis的xyz属性按空白拆为浮点元组；缺属性或非法数字直接失败。
# @param node 具有xyz属性的XML元素，分量按原文件单位解释。
# @return 浮点坐标元组。
def xyz(node):
    return tuple(map(float, node.get('xyz').split()))


##
# @brief pytest fixture展开生产Xacro并返回XML根，供本模块三个几何断言复用。
# @return URDF XML根元素。
@pytest.fixture
def model():
    return ET.fromstring(xacro.process_file(str(DESCRIPTION / 'urdf/dummy-ros2.xacro')).toxml())


##
# @brief 比较完整OBJ最低Z和独立manifest底面，再以mm→m及visual/collision原点换算确认底面为0。
# @param model 展开生产Xacro的pytest XML fixture。
def test_ground_plane_matches_complete_cad_bottom(model):
    base = model.find("link[@name='base_link']")
    vertices = [tuple(map(float, line.split()[1:4]))
                for line in (DESCRIPTION / 'meshes/full_step_base-colors.obj').open()
                if line.startswith('v ')]
    manifest = json.loads((DESCRIPTION / 'meshes/full_step_manifest.json').read_text())
    bottom_mm = manifest['groups']['base']['local_bounds_mm'][0][2]
    assert min(v[2] for v in vertices) == pytest.approx(bottom_mm, abs=1e-5)
    for kind in ('visual', 'collision'):
        origin = base.find(f'{kind}/origin')
        assert origin.get('rpy') == '0 0 0'
        assert bottom_mm * .001 + xyz(origin)[2] == pytest.approx(0, abs=1e-9)


##
# @brief 核J1原点(0,0,0.096)m、轴(0,0,-1)及零RPY，保留安装高度与方向。
# @param model 展开生产Xacro的XML fixture。
def test_j1_axis_intersects_ground_origin_without_changing_install_height(model):
    joint = model.find("joint[@name='Joint1']")
    assert xyz(joint.find('origin')) == pytest.approx((0, 0, .096), abs=1e-12)
    assert xyz(joint.find('axis')) == (0, 0, -1)
    assert joint.find('origin').get('rpy') == '0 0 0'


##
# @brief 核visual/collision/guard同一平移、质心按旧值加SHIFT，并检查J1相对CAD位置未变。
# @param model 展开生产Xacro的XML fixture。
def test_root_geometries_mass_and_guard_share_the_same_rigid_shift(model):
    base = model.find("link[@name='base_link']")
    for kind in ('visual', 'collision'):
        assert xyz(base.find(f'{kind}/origin')) == pytest.approx(SHIFT)
    guard = model.find("link[@name='clearance_base_link']/collision/origin")
    assert xyz(guard) == pytest.approx(SHIFT)
    old_com = (-.019286443307707055, .0005817485706867557, .05546290448958129)
    new_com = xyz(base.find('inertial/origin'))
    assert new_com == pytest.approx(tuple(a + b for a, b in zip(old_com, SHIFT)))
    # J1相对CAD几何的位置不变：新关节安装点减去网格的根平移。
    joint = xyz(model.find("joint[@name='Joint1']/origin"))
    assert tuple(a - b for a, b in zip(joint, SHIFT)) == pytest.approx(
        (-.019225, -.000523, .100684), abs=1e-12)
