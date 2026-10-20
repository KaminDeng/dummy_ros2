"""目标标记独立于IK：不可达目标也应让用户看见自己输入了哪里。"""
##
# @file test_manual_moveit_target.py
# @brief 目标标记单位转换及失败虚影颜色回归。
#
# 目标输入xyz为mm、rpy为deg，Marker坐标m；显示目标独立于IK/执行授权。XML用例只改visual材质并比较collision子树，不创建ROS节点或查询实际可达性。
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from manual_moveit_target import target_markers
from manual_moveit_target import colored_description
import xml.etree.ElementTree as ET


##
# @brief 输入(120,-30,400)mm和yaw90°，核三Marker、base_link、米转换及四元数z分量。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_target_visible_without_planning():
    markers = target_markers([120., -30., 400., 0., 0., 90.])
    assert len(markers.markers) == 3
    assert all(m.header.frame_id == 'base_link' for m in markers.markers)
    sphere = markers.markers[0]
    assert sphere.pose.position.x == .12
    assert sphere.pose.position.y == -.03
    assert sphere.pose.position.z == .4
    assert abs(markers.markers[1].pose.orientation.z - 2**-.5) < 1e-12


##
# @brief 2m远目标仍显示且文本标未执行，不能让显示成功冒充可达或已运动。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_unreachable_target_is_still_visible():
    markers = target_markers([2000., 2000., 2000., 0., 0., 0.])
    assert markers.markers[0].pose.position.x == 2.
    assert '未执行' in markers.markers[2].text


##
# @brief 失败描述visual变红，collision字节结构保持，非失败原XML返回且目标球同样红色。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_failed_robot_is_red_and_collision_geometry_unchanged():
    xml = '<robot name="test"><link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual><collision><geometry><box size="1 1 1"/></geometry></collision></link></robot>'
    result = ET.fromstring(colored_description(xml, True))
    assert result.find('link/visual/material/color').get('rgba') == '1 0.05 0.05 1'
    assert ET.tostring(result.find('link/collision')) == ET.tostring(ET.fromstring(xml).find('link/collision'))
    assert colored_description(xml, False) == xml
    assert target_markers([0.]*6, True).markers[0].color.g == .05
