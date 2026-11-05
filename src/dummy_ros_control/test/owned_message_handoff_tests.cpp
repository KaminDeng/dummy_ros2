/** @file owned_message_handoff_tests.cpp
 * @brief 生成Owned跨回调缓存保有presence和Source，旧cleanup代不能发布。
 */
#include "dummy_ros_control/device_management_channel.hpp"
#include <cassert>
#include <type_traits>
using namespace dummy_ros_control;
static dummy_ref::messages::State state_fixture(){
 dummy_ref::messages::State value;
 value.session_id() = 0;
 value.reference_epoch() = 0;
 value.motion_epoch() = 0;
 value.management_epoch() = 0;
 value.profile_id() = 0;
 value.produced_at_us() = 0;
 value.freshness_ticket() = 0;
 value.committed_producer_seq() = 0;
 value.admitted_producer_seq() = 0;
 value.operation_id() = 0;
 value.transaction_phase() = 0;
 value.first_failure_reason() = 0;
 value.operation_deadline_us() = 0;
 value.can_submitted_mask() = 0;
 value.can_sent_mask() = 0;
 value.can_failed_mask() = 0;
 value.core_state() = 0;
 value.hold_ready() = false;
 value.state_seq() = 0;
 std::uint8_t domain[16]{};
 assert(value.domain_device_id().emplace().assign(domain,16)==flatbuffers::bounded::Status::ok);
 auto& joints=value.joints().emplace();
 for(unsigned axis=0;axis<6;++axis){auto* joint=joints.emplace_back();assert(joint);
  joint->joint_id()=axis;joint->configured_quality()=true;
  joint->position_valid_quality()=true;joint->velocity_valid_quality()=false;
  joint->raw_position_valid_quality()=true;joint->hold_valid_quality()=false;
  joint->position_rad()=0.;joint->raw_motor_position_rev()=0.;joint->estimated_velocity_rad_s()=0.;joint->hold_position_rad()=0.;
  joint->feedback_seq()=0;joint->can_rx_age_us()=0;
 }
 value.profile_id()=1;value.state_seq()=1;value.produced_at_us()=10;
 return value;
}
int main(){
 static_assert(std::is_same_v<decltype(DeviceManagementRequest::request),dummy_ref::messages::ManageRequest>);
 static_assert(std::is_same_v<decltype(DeviceManagementObservation::state),dummy_ref::messages::State>);
 dummy_ref::messages::State absent;assert(!message_fields_present(absent));
 auto state=state_fixture();assert(message_fields_present(state));
 assert(raw_position_joint_mask(state)==63 && configured_joint_mask(state)==63);
 // 数值0是有值，reset才是缺失；不能用默认0代替校验。
 state.management_epoch().reset();assert(!message_fields_present(state));
 state.management_epoch()=0;assert(message_fields_present(state));
 state.joints()->operator[](2).joint_id()=1;assert(!message_fields_present(state));
 state.joints()->operator[](2).joint_id()=2;assert(message_fields_present(state));
 TransportParameters parameters;DeviceManagementChannel channel(parameters);channel.ready=true;
 dp::comm::SourceId source{};source.boot_id=7;source.context_id=9;
 const auto generation=channel.raw_cleanup_generation();
 channel.publish_raw_cleanup_observation(state,source,100,true,generation);
 state.session_id()=99;source.boot_id=8; // 回调输入复用不改变已缓存Owned与Source。
 auto cached=channel.raw_cleanup_observation();
 assert(cached.state.session_id() && *cached.state.session_id()==0);
 assert(cached.state_source.boot_id==7 && cached.state_source.context_id==9);
 assert(cached.state_received_us==100);
 channel.reset_raw_cleanup_lifetime();
 channel.publish_raw_cleanup_observation(state,source,101,true,generation);
 assert(channel.raw_cleanup_observation().state_received_us==0);
 channel.publish_raw_cleanup_observation(state,source,102,true,channel.raw_cleanup_generation());
 assert(channel.raw_cleanup_observation().state_source.boot_id==8);
 channel.publish_raw_cleanup_observation(state,source,103,false,channel.raw_cleanup_generation());
 assert(channel.raw_cleanup_observation().state_received_us==0);
}
