/** @file panel_feedback.hpp
 * @brief 已解码中央消息到ROS类型的逐字段投影；不授权、不推断完成、不改变有效位。
 */
#pragma once
#include "dummy_ros_control/wire_types.hpp"
#include "dpsdk_interfaces/msg/dummy_device_status_v2.hpp"
#include "dpsdk_interfaces/msg/dummy_joint_state_v2.hpp"
namespace dummy_motion_client {
dpsdk_interfaces::msg::DummyDeviceStatusV2 panel_status_message(const dp::iface::DummyDeviceStatusV2&);
dpsdk_interfaces::msg::DummyJointStateV2 panel_state_message(const dp::iface::DummyJointStateV2&);
}
