/** @file message_presence.hpp
 * @brief ROS消费者直接校验生成Owned字段的presence、六轴索引及有界长度。
 * 不保存第二份消息模型；零/false与缺字段保持区别，来源由调用方另核SourceId。
 */
#pragma once
#include <cmath>
#include <cstdint>
#include "dummy_ref/message_contract.hpp"
#include "field_groups.h"
namespace dummy_ros_control {
template<class... Fields>
bool fields_present(const Fields&... fields) noexcept {return (... && static_cast<bool>(fields));}
template<class Field>
bool vector_size(const Field& field,std::size_t expected) noexcept {
  return field && field->size()==expected;
}
/** @brief 只计算已存在生成记录的轴质量位图；不生成并行数组。 */
inline std::uint8_t configured_joint_mask(const dummy_ref::messages::State& value) noexcept {
  std::uint8_t mask=0U;
  if(value.joints())for(const auto& joint:*value.joints())
    if(joint.joint_id() && *joint.joint_id()<6U && joint.configured_quality() && *joint.configured_quality())
      mask|=std::uint8_t(1U<<*joint.joint_id());
  return mask;
}
inline std::uint8_t raw_position_joint_mask(const dummy_ref::messages::State& value) noexcept {
  std::uint8_t mask=0U;
  if(value.joints())for(const auto& joint:*value.joints())
    if(joint.joint_id() && *joint.joint_id()<6U && joint.raw_position_valid_quality() &&
       *joint.raw_position_valid_quality() && joint.raw_motor_position_rev())mask|=std::uint8_t(1U<<*joint.joint_id());
  return mask;
}
inline bool message_fields_present(const dummy_ref::messages::State& value) noexcept {
  if(!dp::protocol::groups::dp_protocol_dummy_JointState_Complete(value))return false;
  std::uint8_t seen=0U;
  for(const auto& joint:*value.joints()) {
    if(*joint.joint_id()>=6U || (seen&(1U<<*joint.joint_id())))return false;
    seen|=std::uint8_t(1U<<*joint.joint_id());
    if(*joint.position_valid_quality() && (!joint.position_rad() || !std::isfinite(*joint.position_rad())))return false;
    if(*joint.velocity_valid_quality() && (!joint.estimated_velocity_rad_s() || !std::isfinite(*joint.estimated_velocity_rad_s())))return false;
    if(*joint.raw_position_valid_quality() && (!joint.raw_motor_position_rev() || !std::isfinite(*joint.raw_motor_position_rev())))return false;
    if(*joint.hold_valid_quality() && (!joint.hold_position_rad() || !std::isfinite(*joint.hold_position_rad())))return false;
  }
  return true;
}
inline bool message_fields_present(const dummy_ref::messages::Status& value) noexcept {
  return dp::protocol::groups::dp_protocol_dummy_DeviceStatus_Complete(value);
}
inline bool management_binding_fields_present(const dummy_ref::messages::ManageRequest& value) noexcept {
  return fields_present(value.operation(),value.expected_boot_epoch(),value.expected_session_id(),
    value.expected_reference_epoch(),value.expected_motion_epoch(),value.expected_management_epoch(),
    value.expected_profile_id(),value.freshness_ticket(),value.configured_mask()) &&
    vector_size(value.domain_device_id(),16U) && vector_size(value.client_instance_id(),16U);
}
inline bool message_fields_present(const dummy_ref::messages::ManageRequest& value) noexcept {
  return dp::protocol::groups::dp_protocol_dummy_ManageRequest_Complete(value) &&
    dp::protocol::groups::dp_protocol_dummy_ManageRequest_DomainComplete(value) && bool(value.request_seq());
}
inline bool message_fields_present(const dummy_ref::messages::ManageResponse& value) noexcept {
  return dp::protocol::groups::dp_protocol_dummy_ManageResponse_Complete(value) && dp::protocol::groups::dp_protocol_dummy_ManageResponse_DomainComplete(value);
}
/** @brief 证据业务字段缺失与零值独立检查，Root来源由ReceiveInfo提供。 */
inline bool message_fields_present(const dummy_ref::messages::Evidence& value) noexcept {
  return dp::protocol::groups::dp_protocol_dummy_AttemptEvidence_Complete(value);
}
/** @brief 按中央消息中的显式joint_id定位实际关节记录；不建立并行数组。 */
inline auto joint_for_id(const dummy_ref::messages::State& state,std::size_t axis) noexcept
    -> decltype(&*state.joints()->begin()) {
  if(state.joints())for(const auto& joint:*state.joints())if(joint.joint_id() && *joint.joint_id()==axis)return &joint;
  return nullptr;
}
/** @brief 根据实际关节质量字段推导关节坐标有效集合。 */
inline std::uint8_t position_valid_joint_mask(const dummy_ref::messages::State& state) noexcept {
  std::uint8_t mask=0U;
  if(state.joints())for(const auto& joint:*state.joints())
    if(joint.joint_id() && *joint.joint_id()<6U && joint.position_valid_quality() && *joint.position_valid_quality())
      mask|=static_cast<std::uint8_t>(1U<<*joint.joint_id());
  return mask;
}
/** @brief 从实际关节记录推导速度质量集合。 */
inline std::uint8_t velocity_valid_joint_mask(const dummy_ref::messages::State& state) noexcept {
  std::uint8_t mask=0U;
  if(state.joints())for(const auto& joint:*state.joints())
    if(joint.joint_id() && *joint.joint_id()<6U && joint.velocity_valid_quality() && *joint.velocity_valid_quality())mask|=static_cast<std::uint8_t>(1U<<*joint.joint_id());
  return mask;
}
} // namespace dummy_ros_control
