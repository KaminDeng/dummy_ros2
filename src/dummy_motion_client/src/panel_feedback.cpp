/** @file panel_feedback.cpp
 * @brief 中央生成Owned到ROS同构投影；面板只负责来源选择及新鲜度。
 */
#include "dummy_motion_client/panel_feedback.hpp"
#include "dummy_ros_control/message_presence.hpp"
#include "dpsdk_interfaces/owned_projection.hpp"
namespace dummy_motion_client {
dpsdk_interfaces::msg::DummyDeviceStatus panel_status_message(const dummy_ref::messages::Status& input,const dp::comm::SourceId& source){
  dpsdk_interfaces::msg::DummyDeviceStatus output;
  dp::protocol::ros::project(input,source,output);
  return output;
}
dpsdk_interfaces::msg::DummyJointState panel_state_message(const dummy_ref::messages::State& input,const dp::comm::SourceId& source){
  dpsdk_interfaces::msg::DummyJointState output;
  dp::protocol::ros::project(input,source,output);
  return output;
}
PanelFeedbackSelection select_panel_feedback(const PanelFeedbackCache& control,
  PanelFeedbackObservation<dummy_ref::messages::Status> diagnostic_status,
  PanelFeedbackObservation<dummy_ref::messages::State> diagnostic_state,std::uint64_t now,
  std::uint64_t state_max_age_us,std::uint64_t status_max_age_us,
  std::uint64_t& status_published_us,std::uint64_t& state_published_us){
  PanelFeedbackSelection selected;
  const auto fresh=[now](std::uint64_t received,std::uint64_t maximum){return received && now>=received && now-received<=maximum;};
  if(control.available && control.status_received_us>status_published_us && fresh(control.status_received_us,status_max_age_us)){
    selected.status=&control.status;selected.status_source=control.status_source;status_published_us=control.status_received_us;
  }
  if(control.available && control.state_received_us>state_published_us && fresh(control.state_received_us,state_max_age_us)){
    selected.state=&control.state;selected.state_source=control.state_source;state_published_us=control.state_received_us;
  }
  if(diagnostic_status.value && diagnostic_status.received_us>status_published_us && fresh(diagnostic_status.received_us,status_max_age_us) && dummy_ros_control::message_fields_present(*diagnostic_status.value)){
    selected.status=diagnostic_status.value;selected.status_source=diagnostic_status.source;status_published_us=diagnostic_status.received_us;
  }
  if(diagnostic_state.value && diagnostic_state.received_us>state_published_us && fresh(diagnostic_state.received_us,state_max_age_us) && dummy_ros_control::message_fields_present(*diagnostic_state.value)){
    selected.state=diagnostic_state.value;selected.state_source=diagnostic_state.source;state_published_us=diagnostic_state.received_us;
  }
  return selected;
}
}
