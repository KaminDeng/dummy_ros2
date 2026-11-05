/** @file protocol_r11_presence_tests.cpp
 * @brief 实际Owned交付与中央记录保留presence、来源以及独立的领域身份。 */
#include "dummy_ros_control/device_management_channel.hpp"
#include "owned_message_fixtures.hpp"
#include "canonical_records.h"
#include <cassert>
#include <vector>
int main(){
  using namespace dummy_ros_control;
  TransportParameters parameters;parameters.device_id[0]=1;parameters.profile_id=1;parameters.configured_mask=1;parameters.max_state_age_us=250000;
  DeviceManagementChannel channel(parameters);channel.ready=true;
  dp::comm::SourceId source{};source.device.bytes[0]=9;source.boot_id=UINT64_C(0xf123456789abcdef);source.context_id=7;
  auto status=owned_fixture::status();status.domain_device_id()->operator[](0)=1;status.profile_id()=1;status.configured_mask()=1;
  assert(channel.publish_diagnostic_status(status,source,100));
  auto value=owned_fixture::state();value.domain_device_id()->operator[](0)=1;value.profile_id()=1;value.state_seq()=1;value.produced_at_us()=10;value.hold_ready()=true;
  for(auto& joint:*value.joints()){joint.configured_quality()=(*joint.joint_id()==0);joint.position_valid_quality()=(*joint.joint_id()==0);}
  value.joints()->operator[](0).position_rad()=1.25;
  assert(channel.publish_diagnostic_state(value,source,101));
  auto accepted=channel.diagnostic_observation();assert(*accepted.state.joints()->operator[](0).position_rad()==1.25&&*accepted.state.hold_ready());
  assert(accepted.state_source==source&&accepted.state.domain_device_id()->operator[](0)==1);
  value.hold_ready().reset();assert(!channel.publish_diagnostic_state(value,source,102));
  accepted=channel.diagnostic_observation();assert(*accepted.state.hold_ready());
  value.hold_ready()=true;value.joints()->operator[](0).position_rad().reset();assert(!channel.publish_diagnostic_state(value,source,102));
  value.joints()->operator[](0).position_rad()=0;value.hold_ready()=false;value.state_seq()=2;value.produced_at_us()=20;
  assert(channel.publish_diagnostic_state(value,source,111));
  accepted=channel.diagnostic_observation();assert(*accepted.state.joints()->operator[](0).position_rad()==0&&!*accepted.state.hold_ready()&&*accepted.state.state_seq()==2);
  // Source UID不是领域ID；错/缺/末字节不等的领域ID不能由期望身份补齐。
  auto wrong=value;wrong.domain_device_id()->operator[](15)=1;assert(!channel.publish_diagnostic_state(wrong,source,112));
  wrong=value;wrong.domain_device_id().reset();assert(!channel.publish_diagnostic_state(wrong,source,112));
  accepted=channel.diagnostic_observation();assert(*accepted.state.state_seq()==2);
  // Command目标Boot与发送Host Source独立，规范记录不注入外部身份。
  dummy_ref::messages::Command command;command.boot_epoch()=7;command.domain_device_id().emplace().assign(parameters.device_id.data(),16);
  std::uint8_t client[16]{3};command.client_instance_id().emplace().assign(client,16);
  using Traits=dp::protocol::canonical::Traits<dummy_ref::messages::Command>;
  std::vector<std::uint8_t> bytes(Traits::kMaxEncodedSize);std::size_t written=0;
  assert(Traits::encode(command,bytes.data(),bytes.size(),&written));dummy_ref::messages::Command delivered;
  assert(Traits::decode(bytes.data(),written,&delivered));assert(*delivered.boot_epoch()==7&&*delivered.boot_epoch()!=source.boot_id);
  assert(delivered.domain_device_id()->operator[](0)==1&&delivered.client_instance_id()->operator[](0)==3);
}
