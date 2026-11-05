/** @file panel_feedback.hpp
 * @brief 中央Owned消息向中央生成ROS接口投影；显式保留presence与实际SDK来源。
 */
#pragma once
#include "dummy_ref/message_contract.hpp"
#include "dpsdk_interfaces/msg/dummy_device_status.hpp"
#include "dpsdk_interfaces/msg/dummy_joint_state.hpp"
#include <cstdint>
namespace dummy_motion_client {
dpsdk_interfaces::msg::DummyDeviceStatus panel_status_message(const dummy_ref::messages::Status&,const dp::comm::SourceId&);
dpsdk_interfaces::msg::DummyJointState panel_state_message(const dummy_ref::messages::State&,const dp::comm::SourceId&);
/** @brief 控制锁保护下复制的实际Owned及来源；诊断观察不回填本缓存。 */
struct PanelFeedbackCache {
  bool available=false;
  dummy_ref::messages::Status status;
  dummy_ref::messages::State state;
  dp::comm::SourceId status_source{},state_source{};
  std::uint64_t status_received_us=0U,state_received_us=0U;
};
/** @brief 仅借用调用期间有效的实际Owned；来源值与原接收时刻独立保留。 */
template<class Message>struct PanelFeedbackObservation {
  const Message* value=nullptr;
  dp::comm::SourceId source{};
  std::uint64_t received_us=0U;
};
/** @brief 选定消息借用与输入缓存同寿命，只供立即发布，不授予设备控制资格。 */
struct PanelFeedbackSelection {
  const dummy_ref::messages::Status* status=nullptr;
  const dummy_ref::messages::State* state=nullptr;
  dp::comm::SourceId status_source{},state_source{};
};
PanelFeedbackSelection select_panel_feedback(const PanelFeedbackCache& control,
  PanelFeedbackObservation<dummy_ref::messages::Status> diagnostic_status,
  PanelFeedbackObservation<dummy_ref::messages::State> diagnostic_state,std::uint64_t now,
  std::uint64_t state_max_age_us,std::uint64_t status_max_age_us,
  std::uint64_t& status_published_us,std::uint64_t& state_published_us);
}
