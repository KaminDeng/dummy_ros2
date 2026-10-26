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

#include <cstddef>
#include <cstdint>
#include <optional>
namespace dummy_motion_client {
/** @brief 只有取得控制锁后才复制；不接收诊断回填。 */
struct PanelFeedbackCache {
  bool available = false;
  dp::iface::DummyDeviceStatusV2 status;
  dp::iface::DummyJointStateV2 state;
  std::uint64_t status_received_us = 0, state_received_us = 0;
};
/** @brief 指向worker已校验非授权诊断字节，不改变其原接收时刻。 */
struct PanelFeedbackBytes {
  const std::uint8_t *data = nullptr;
  std::size_t size = 0;
  std::uint64_t received_us = 0;
};
struct PanelFeedbackSelection {
  std::optional<dp::iface::DummyDeviceStatusV2> status;
  std::optional<dp::iface::DummyJointStateV2> state;
};
/** @brief 发布专用选择；调用者串行拥有水位，不授予控制或释放资格。 */
PanelFeedbackSelection select_panel_feedback(
    const PanelFeedbackCache &control, PanelFeedbackBytes diagnostic_status,
    PanelFeedbackBytes diagnostic_state, std::uint64_t now,
    std::uint64_t state_max_age_us, std::uint64_t status_max_age_us,
    std::uint64_t &status_published_us, std::uint64_t &state_published_us);
}
