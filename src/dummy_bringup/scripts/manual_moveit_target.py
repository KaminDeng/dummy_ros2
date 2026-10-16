"""RViz目标点和姿态坐标轴；不依赖逆解成功，也不发送运动。"""
from copy import deepcopy
import xml.etree.ElementTree as ET
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from manual_moveit_planner import rpy_quaternion


def colored_description(xml, failed):
    if not failed:
        return xml
    root = ET.fromstring(xml)
    for index, visual in enumerate(root.findall('link/visual')):
        for material in list(visual.findall('material')):
            visual.remove(material)
        material = ET.SubElement(visual, 'material', name=f'invalid_preview_{index}')
        ET.SubElement(material, 'color', rgba='1 0.05 0.05 1')
    return ET.tostring(root, encoding='unicode')


def target_markers(values, failed=False):
    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.ns = 'manual_moveit_target'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (v/1000 for v in values[:3])
    marker.pose.orientation.w = 1.
    marker.scale.x = marker.scale.y = marker.scale.z = .014
    marker.color = ColorRGBA(r=1., g=.05 if failed else .6, b=.05, a=.9)
    axes = deepcopy(marker)
    axes.id, axes.type = 1, Marker.LINE_LIST
    axes.pose.orientation.x, axes.pose.orientation.y, axes.pose.orientation.z, axes.pose.orientation.w = rpy_quaternion(*values[3:])
    axes.scale.x = .003
    axes.points = [Point(), Point(x=.06), Point(), Point(y=.06), Point(), Point(z=.06)]
    axes.colors = [ColorRGBA(r=1., a=1.), ColorRGBA(r=1., a=1.),
                   ColorRGBA(g=1., a=1.), ColorRGBA(g=1., a=1.),
                   ColorRGBA(b=1., a=1.), ColorRGBA(b=1., a=1.)]
    label = deepcopy(marker)
    label.id, label.type = 2, Marker.TEXT_VIEW_FACING
    label.pose.position.z += .04
    label.scale.z = .018
    label.text = ('目标无效（未执行） ' if failed else '目标（未执行） ')+', '.join(f'{v:g}' for v in values[:3])+' mm'
    return MarkerArray(markers=[marker, axes, label])
