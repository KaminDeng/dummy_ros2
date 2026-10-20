##
# @file manual_moveit_target.py
# @brief 生成末端目标标记和失败目标模型颜色。
# RViz 中的目标点、姿态轴和目标虚影均表示用户输入，不是机械臂实测姿态；即使逆解失败也可显示，显示本身不授权执行。
"""RViz目标点和姿态坐标轴；不依赖逆解成功，也不发送运动。"""
from copy import deepcopy
import xml.etree.ElementTree as ET
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from manual_moveit_planner import rpy_quaternion


##
# @brief 在失败预览时将 URDF 各可视元素改为红色。
# 只改变 visual 材质，不改变 link/joint 几何或实际反馈描述；失败分支的 XML 解析错误由调用方处理。
# @param xml 机器人描述的 XML 字符串；成功预览时直接原样返回。
# @param failed 是否将目标虚影标记为失败。
# @return str：原描述或替换 visual/material 后重新序列化的描述。
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


##
# @brief 构造目标球、姿态 RGB 轴和“未执行”文字。
# 位置除以 1000 转为 ROS 米；轴旋转经 rpy_quaternion 转为四元数。固定 ID 便于后续发布覆盖；函数要求调用方提供完整有限输入。
# @param values 六维输入 [X,Y,Z,roll,pitch,yaw]；位置毫米，姿态角度。
# @param failed 失败状态；改变目标球提示颜色及文字。
# @return MarkerArray：base_link 下三个固定命名空间和 ID 的目标标记。
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
    ##
    # 轴线坐标以米表示，长度 0.06 m、线宽 0.003 m；复制标记继承目标点位置后再应用目标姿态。
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
