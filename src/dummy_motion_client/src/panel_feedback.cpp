/** @file panel_feedback.cpp
 * @brief 按既有中央IDL逐字段投影；字段及有效位保持设备语义，不用ROS接收时刻替换源序号。
 */
#include "dummy_motion_client/panel_feedback.hpp"
#include <algorithm>
#include <iterator>
namespace dummy_motion_client {
dpsdk_interfaces::msg::DummyDeviceStatusV2 panel_status_message(const dp::iface::DummyDeviceStatusV2& input) {
  dpsdk_interfaces::msg::DummyDeviceStatusV2 output;
  output.schema_major = input.schema_major();
  output.schema_minor = input.schema_minor();
  std::copy(std::begin(input.device_id()), std::end(input.device_id()), output.device_id.begin());
  output.boot_epoch = input.boot_epoch();
  output.session_id = input.session_id();
  output.reference_epoch = input.reference_epoch();
  output.motion_epoch = input.motion_epoch();
  output.management_epoch = input.management_epoch();
  output.profile_id = input.profile_id();
  output.core_state = input.core_state();
  output.control_mode = input.control_mode();
  output.reference_kind = input.reference_kind();
  output.configured_mask = input.configured_mask();
  output.motion_supported = input.motion_supported();
  output.real_motion_permitted = input.real_motion_permitted();
  output.permit_remaining_us = input.permit_remaining_us();
  output.capabilities = input.capabilities();
  output.operation_id = input.operation_id();
  output.transaction_phase = input.transaction_phase();
  output.first_failure_reason = input.first_failure_reason();
  output.operation_deadline_us = input.operation_deadline_us();
  output.committed_producer_seq = input.committed_producer_seq();
  output.can_submitted_mask = input.can_submitted_mask();
  output.can_sent_mask = input.can_sent_mask();
  output.can_failed_mask = input.can_failed_mask();
  output.command_reject_count = input.command_reject_count();
  output.can_error_count = input.can_error_count();
  output.deadline_miss_count = input.deadline_miss_count();
  std::copy(std::begin(input.contract_sha256()), std::end(input.contract_sha256()), output.contract_sha256.begin());
  std::copy(std::begin(input.model_sha256()), std::end(input.model_sha256()), output.model_sha256.begin());
  std::copy(std::begin(input.profile_sha256()), std::end(input.profile_sha256()), output.profile_sha256.begin());
  std::copy(std::begin(input.firmware_sha256()), std::end(input.firmware_sha256()), output.firmware_sha256.begin());
  std::copy(std::begin(input.board_name_ascii()), std::end(input.board_name_ascii()), output.board_name_ascii.begin());
  std::copy(std::begin(input.compiled_policy_sha256()), std::end(input.compiled_policy_sha256()), output.compiled_policy_sha256.begin());
  std::copy(std::begin(input.position_min_rad()), std::end(input.position_min_rad()), output.position_min_rad.begin());
  std::copy(std::begin(input.position_max_rad()), std::end(input.position_max_rad()), output.position_max_rad.begin());
  std::copy(std::begin(input.velocity_max_rad_s()), std::end(input.velocity_max_rad_s()), output.velocity_max_rad_s.begin());
  std::copy(std::begin(input.acceleration_max_rad_s()), std::end(input.acceleration_max_rad_s()), output.acceleration_max_rad_s.begin());
  output.control_period_us = input.control_period_us();
  output.state_publish_period_us = input.state_publish_period_us();
  std::copy(std::begin(input.feedback_period_us()), std::end(input.feedback_period_us()), output.feedback_period_us.begin());
  output.mailbox_turnover = input.mailbox_turnover();
  output.tx_event_overflow = input.tx_event_overflow();
  output.rx_overflow = input.rx_overflow();
  output.bus_off = input.bus_off();
  output.run_id = input.run_id();
  output.attempt_id = input.attempt_id();
  output.final_state = input.final_state();
  output.failure_source = input.failure_source();
  output.failure_axis_mask = input.failure_axis_mask();
  output.cleanup_state = input.cleanup_state();
  std::copy(std::begin(input.can_queue_to_submit_us()), std::end(input.can_queue_to_submit_us()), output.can_queue_to_submit_us.begin());
  std::copy(std::begin(input.can_submit_to_sent_us()), std::end(input.can_submit_to_sent_us()), output.can_submit_to_sent_us.begin());
  output.group_count = input.group_count();
  std::copy(std::begin(input.evidence_chain_sha256()), std::end(input.evidence_chain_sha256()), output.evidence_chain_sha256.begin());
  output.evidence_lost = input.evidence_lost();
  output.attempt_kind = input.attempt_kind();
  output.parent_run_id = input.parent_run_id();
  output.motion_complete = input.motion_complete();
  output.release_committed = input.release_committed();
  return output;
}
dpsdk_interfaces::msg::DummyJointStateV2 panel_state_message(const dp::iface::DummyJointStateV2& input) {
  dpsdk_interfaces::msg::DummyJointStateV2 output;
  output.schema_major = input.schema_major();
  output.schema_minor = input.schema_minor();
  std::copy(std::begin(input.device_id()), std::end(input.device_id()), output.device_id.begin());
  output.boot_epoch = input.boot_epoch();
  output.session_id = input.session_id();
  output.reference_epoch = input.reference_epoch();
  output.motion_epoch = input.motion_epoch();
  output.management_epoch = input.management_epoch();
  output.profile_id = input.profile_id();
  output.state_seq = input.state_seq();
  output.produced_at_us = input.produced_at_us();
  output.freshness_ticket = input.freshness_ticket();
  output.configured_mask = input.configured_mask();
  output.position_valid_mask = input.position_valid_mask();
  output.velocity_valid_mask = input.velocity_valid_mask();
  output.raw_position_valid_mask = input.raw_position_valid_mask();
  output.committed_producer_seq = input.committed_producer_seq();
  output.admitted_producer_seq = input.admitted_producer_seq();
  output.operation_id = input.operation_id();
  output.transaction_phase = input.transaction_phase();
  output.first_failure_reason = input.first_failure_reason();
  output.operation_deadline_us = input.operation_deadline_us();
  output.can_submitted_mask = input.can_submitted_mask();
  output.can_sent_mask = input.can_sent_mask();
  output.can_failed_mask = input.can_failed_mask();
  std::copy(std::begin(input.feedback_seq()), std::end(input.feedback_seq()), output.feedback_seq.begin());
  std::copy(std::begin(input.can_rx_age_us()), std::end(input.can_rx_age_us()), output.can_rx_age_us.begin());
  std::copy(std::begin(input.position_rad()), std::end(input.position_rad()), output.position_rad.begin());
  std::copy(std::begin(input.estimated_velocity_rad_s()), std::end(input.estimated_velocity_rad_s()), output.estimated_velocity_rad_s.begin());
  std::copy(std::begin(input.raw_motor_position_rev()), std::end(input.raw_motor_position_rev()), output.raw_motor_position_rev.begin());
  output.core_state = input.core_state();
  output.hold_ready = input.hold_ready();
  std::copy(std::begin(input.hold_position_rad()), std::end(input.hold_position_rad()), output.hold_position_rad.begin());
  output.hold_valid_mask = input.hold_valid_mask();
  return output;
}
}

namespace dummy_motion_client {
PanelFeedbackSelection select_panel_feedback(
    const PanelFeedbackCache &control, PanelFeedbackBytes diagnostic_status,
    PanelFeedbackBytes diagnostic_state, std::uint64_t now,
    std::uint64_t state_max_age_us, std::uint64_t status_max_age_us,
    std::uint64_t &status_published_us, std::uint64_t &state_published_us) {
  PanelFeedbackSelection selected;
  const auto fresh = [now](std::uint64_t at, std::uint64_t age) {
    return at != 0U && now >= at && now - at <= age;
  };
  if (control.available && control.status_received_us > status_published_us && fresh(control.status_received_us,status_max_age_us)) {
    selected.status = control.status; status_published_us = control.status_received_us;
  }
  if (control.available && control.state_received_us > state_published_us && fresh(control.state_received_us,state_max_age_us)) {
    selected.state = control.state; state_published_us = control.state_received_us;
  }
  dp::iface::DummyDeviceStatusV2 status;
  if (diagnostic_status.received_us > status_published_us && fresh(diagnostic_status.received_us,status_max_age_us) &&
      dp::iface::DummyDeviceStatusV2Codec::decode(diagnostic_status.data,diagnostic_status.size,&status)==dp::iface::CodecResult::kOk) {
    selected.status = status; status_published_us = diagnostic_status.received_us;
  }
  dp::iface::DummyJointStateV2 state;
  if (diagnostic_state.received_us > state_published_us && fresh(diagnostic_state.received_us,state_max_age_us) &&
      dp::iface::DummyJointStateV2Codec::decode(diagnostic_state.data,diagnostic_state.size,&state)==dp::iface::CodecResult::kOk) {
    selected.state = state; state_published_us = diagnostic_state.received_us;
  }
  return selected;
}
}
