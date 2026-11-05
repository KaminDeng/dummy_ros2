/** @file owned_message_fixtures.hpp
 * @brief 测试直接构造中央Owned；无并行业务类型、字段布局或codec。
 */
#pragma once
#include "dummy_ref/message_contract.hpp"
#include <array>
namespace owned_fixture {
inline dummy_ref::messages::Status status(){
  dummy_ref::messages::Status value;
  value.session_id()=0U;
  value.reference_epoch()=0U;
  value.motion_epoch()=0U;
  value.management_epoch()=0U;
  value.profile_id()=0U;
  value.core_state()=0U;
  value.control_mode()=0U;
  value.reference_kind()=0U;
  value.configured_mask()=0U;
  value.motion_supported()=false;
  value.real_motion_permitted()=false;
  value.permit_remaining_us()=0U;
  value.capabilities()=0U;
  value.operation_id()=0U;
  value.transaction_phase()=0U;
  value.first_failure_reason()=0U;
  value.operation_deadline_us()=0U;
  value.committed_producer_seq()=0U;
  value.can_submitted_mask()=0U;
  value.can_sent_mask()=0U;
  value.can_failed_mask()=0U;
  value.command_reject_count()=0U;
  value.can_error_count()=0U;
  value.deadline_miss_count()=0U;
  {std::array<std::uint8_t,32> items{};(void)value.contract_sha256().emplace().assign(items.data(),items.size());}
  {std::array<std::uint8_t,32> items{};(void)value.model_sha256().emplace().assign(items.data(),items.size());}
  {std::array<std::uint8_t,32> items{};(void)value.profile_sha256().emplace().assign(items.data(),items.size());}
  {std::array<std::uint8_t,32> items{};(void)value.firmware_sha256().emplace().assign(items.data(),items.size());}
  {std::array<std::uint8_t,16> items{};(void)value.board_name_ascii().emplace().assign(items.data(),items.size());}
  {std::array<std::uint8_t,32> items{};(void)value.compiled_policy_sha256().emplace().assign(items.data(),items.size());}
  {std::array<double,6> items{};(void)value.position_min_rad().emplace().assign(items.data(),items.size());}
  {std::array<double,6> items{};(void)value.position_max_rad().emplace().assign(items.data(),items.size());}
  {std::array<double,6> items{};(void)value.velocity_max_rad_s().emplace().assign(items.data(),items.size());}
  {std::array<double,6> items{};(void)value.acceleration_max_rad_s().emplace().assign(items.data(),items.size());}
  value.control_period_us()=0U;
  value.state_publish_period_us()=0U;
  {std::array<std::uint32_t,6> items{};(void)value.feedback_period_us().emplace().assign(items.data(),items.size());}
  value.mailbox_turnover()=0U;
  value.tx_event_overflow()=0U;
  value.rx_overflow()=0U;
  value.bus_off()=false;
  value.run_id()=0U;
  value.attempt_id()=0U;
  value.final_state()=0U;
  value.failure_source()=0U;
  value.failure_axis_mask()=0U;
  value.cleanup_state()=0U;
  {std::array<std::uint32_t,6> items{};(void)value.can_queue_to_submit_us().emplace().assign(items.data(),items.size());}
  {std::array<std::uint32_t,6> items{};(void)value.can_submit_to_sent_us().emplace().assign(items.data(),items.size());}
  value.group_count()=0U;
  {std::array<std::uint8_t,32> items{};(void)value.evidence_chain_sha256().emplace().assign(items.data(),items.size());}
  value.evidence_lost()=false;
  value.attempt_kind()=0U;
  value.parent_run_id()=0U;
  value.motion_complete()=false;
  value.release_committed()=false;
  {std::array<std::uint8_t,16> items{};(void)value.domain_device_id().emplace().assign(items.data(),items.size());}
  return value;
}
inline dummy_ref::messages::Evidence evidence(){
  dummy_ref::messages::Evidence value;
  value.run_id()=0U;
  value.evidence_seq()=0U;
  value.group_id()=0U;
  value.producer_seq()=0U;
  value.motion_epoch()=0U;
  value.category()=0U;
  value.expected_mask()=0U;
  value.submitted_mask()=0U;
  value.sent_mask()=0U;
  value.failed_mask()=0U;
  value.final()=0U;
  value.first_failure_source()=0U;
  value.first_failure_axis_mask()=0U;
  {std::array<std::uint64_t,6> items{};(void)value.queued_at_us().emplace().assign(items.data(),items.size());}
  {std::array<std::uint64_t,6> items{};(void)value.taken_at_us().emplace().assign(items.data(),items.size());}
  {std::array<std::uint64_t,6> items{};(void)value.submitted_at_us().emplace().assign(items.data(),items.size());}
  {std::array<std::uint64_t,6> items{};(void)value.sent_at_us().emplace().assign(items.data(),items.size());}
  {std::array<std::uint64_t,6> items{};(void)value.abort_at_us().emplace().assign(items.data(),items.size());}
  value.group_count()=0U;
  value.attempt_id()=0U;
  value.attempt_kind()=0U;
  value.parent_run_id()=0U;
  value.motion_complete()=false;
  value.release_committed()=false;
  value.final_state()=0U;
  value.terminal_reason()=0U;
  {std::array<std::uint8_t,32> items{};(void)value.previous_chain_sha256().emplace().assign(items.data(),items.size());}
  {std::array<std::uint8_t,32> items{};(void)value.chain_sha256().emplace().assign(items.data(),items.size());}
  value.evidence_lost()=false;
  return value;
}
inline dummy_ref::messages::State state(){
  dummy_ref::messages::State value;
  value.session_id()=0U;
  value.reference_epoch()=0U;
  value.motion_epoch()=0U;
  value.management_epoch()=0U;
  value.profile_id()=0U;
  value.produced_at_us()=0U;
  value.freshness_ticket()=0U;
  value.committed_producer_seq()=0U;
  value.admitted_producer_seq()=0U;
  value.operation_id()=0U;
  value.transaction_phase()=0U;
  value.first_failure_reason()=0U;
  value.operation_deadline_us()=0U;
  value.can_submitted_mask()=0U;
  value.can_sent_mask()=0U;
  value.can_failed_mask()=0U;
  value.core_state()=0U;
  value.hold_ready()=false;
  auto& joints=value.joints().emplace();
  for(unsigned axis=0;axis<6;++axis){auto* joint=joints.emplace_back();
    joint->joint_id()=axis;
    joint->configured_quality()=true;
    joint->position_valid_quality()=true;
    joint->velocity_valid_quality()=false;
    joint->raw_position_valid_quality()=true;
    joint->feedback_seq()=0U;
    joint->can_rx_age_us()=0U;
    joint->position_rad()=0U;
    joint->estimated_velocity_rad_s()=0U;
    joint->raw_motor_position_rev()=0U;
    joint->hold_position_rad()=0U;
    joint->hold_valid_quality()=false;
  }
  value.state_seq()=0U;
  {std::array<std::uint8_t,16> items{};(void)value.domain_device_id().emplace().assign(items.data(),items.size());}
  return value;
}
inline dummy_ref::messages::ManageRequest request(){
  dummy_ref::messages::ManageRequest value;
  value.operation()=0U;
  value.queried_operation_id()=0U;
  value.expected_boot_epoch()=7U;
  value.expected_session_id()=0U;
  value.expected_reference_epoch()=0U;
  value.expected_motion_epoch()=0U;
  value.expected_management_epoch()=0U;
  value.expected_profile_id()=0U;
  value.freshness_ticket()=0U;
  value.requested_control_mode()=0U;
  value.configured_mask()=0U;
  value.operator_confirmed()=false;
  {std::array<std::uint8_t,32> items{};(void)value.permit_sha256().emplace().assign(items.data(),items.size());}
  {std::array<std::uint8_t,32> items{};(void)value.compiled_policy_sha256().emplace().assign(items.data(),items.size());}
  value.permit_remaining_us()=0U;
  value.terminal_producer_seq()=0U;
  {std::array<double,6> items{};(void)value.terminal_position_rad().emplace().assign(items.data(),items.size());}
  value.requested_session_lease_us()=0U;
  {std::array<std::uint8_t,16> items{};(void)value.client_instance_id().emplace().assign(items.data(),items.size());}
  value.request_id()=0U;
  value.request_seq()=0U;
  {std::array<std::uint8_t,16> items{};(void)value.domain_device_id().emplace().assign(items.data(),items.size());}
  return value;
}
inline dummy_ref::messages::ManageResponse response(){
  dummy_ref::messages::ManageResponse value;
  value.receipt()=0U;
  value.result_code()=0U;
  value.operation_id()=0U;
  value.transaction_phase()=0U;
  value.first_failure_reason()=0U;
  value.operation_deadline_us()=0U;
  value.session_id()=0U;
  value.reference_epoch()=0U;
  value.motion_epoch()=0U;
  value.management_epoch()=0U;
  value.profile_id()=0U;
  value.committed_producer_seq()=0U;
  value.can_submitted_mask()=0U;
  value.can_sent_mask()=0U;
  value.can_failed_mask()=0U;
  value.run_id()=0U;
  value.attempt_id()=0U;
  value.attempt_kind()=0U;
  value.parent_run_id()=0U;
  value.motion_complete()=false;
  value.release_committed()=false;
  {std::array<std::uint8_t,16> items{};(void)value.client_instance_id().emplace().assign(items.data(),items.size());}
  value.request_id()=0U;
  {std::array<std::uint8_t,16> items{};(void)value.domain_device_id().emplace().assign(items.data(),items.size());}
  return value;
}
}
