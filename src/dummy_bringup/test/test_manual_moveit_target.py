"""目标标记独立于IK：不可达目标也应让用户看见自己输入了哪里。"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from manual_moveit_target import target_markers
from manual_moveit_target import colored_description
import xml.etree.ElementTree as ET


def test_target_visible_without_planning():
    markers = target_markers([120., -30., 400., 0., 0., 90.])
    assert len(markers.markers) == 3
    assert all(m.header.frame_id == 'base_link' for m in markers.markers)
    sphere = markers.markers[0]
    assert sphere.pose.position.x == .12
    assert sphere.pose.position.y == -.03
    assert sphere.pose.position.z == .4
    assert abs(markers.markers[1].pose.orientation.z - 2**-.5) < 1e-12


def test_unreachable_target_is_still_visible():
    markers = target_markers([2000., 2000., 2000., 0., 0., 0.])
    assert markers.markers[0].pose.position.x == 2.
    assert '未执行' in markers.markers[2].text


def test_failed_robot_is_red_and_collision_geometry_unchanged():
    xml = '<robot name="test"><link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual><collision><geometry><box size="1 1 1"/></geometry></collision></link></robot>'
    result = ET.fromstring(colored_description(xml, True))
    assert result.find('link/visual/material/color').get('rgba') == '1 0.05 0.05 1'
    assert ET.tostring(result.find('link/collision')) == ET.tostring(ET.fromstring(xml).find('link/collision'))
    assert colored_description(xml, False) == xml
    assert target_markers([0.]*6, True).markers[0].color.g == .05
