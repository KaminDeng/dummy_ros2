/**
 * @file raw_cleanup_lifetime_generation_tests.cpp
 * @brief 验证raw观测缓存跨cleanup代次拒绝迟到发布。
 *
 * 使用实际生成Owned人工夹具与独立Source及主机接收µs；不连接设备，不证明Source时间映射或设备反馈有效。
 */
#include "dummy_ros_control/device_management_channel.hpp"
#include <cstdio>
#include <cassert>
#include "owned_message_fixtures.hpp"
/**
 * @brief 检查旧generation无法污染重新就绪后的缓存。
 *
 * ready=false并reset清空观测；恢复ready后旧代次拒绝、新代次接受；producer_valid=false发布也必须清空已有State时间戳。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int readonly_fault_owner_retirement_proof() {
 using namespace dummy_ros_control;
 TransportParameters p;p.manual_reference_mode=true;p.profile_id=1;p.configured_mask=63;p.max_state_age_us=250000;p.device_id[0]=1;
 std::uint8_t device[16]{1};
 DeviceManagementChannel c(p);c.ready=true;
 c.publish_worker_release_commit({42,2613,494,7,77,0,3,999,1608});
 auto status=owned_fixture::status();dp::comm::SourceId status_source{};status.domain_device_id().emplace().assign(device,16U);status.profile_id()=1;status.configured_mask()=63;status_source.boot_id=7;status.reference_epoch()=3;status.core_state()=9;status.motion_epoch()=1000;status.management_epoch()=1608;status.operation_id()=2613;status.run_id()=495;status.attempt_id()=495;status.attempt_kind()=2;status.parent_run_id()=494;status.final_state()=4;status.failure_source()=0x1002;status.failure_axis_mask()=63;status.cleanup_state()=3;status.release_committed()=true;
 assert(c.publish_diagnostic_status(status,status_source,200));
 auto state=owned_fixture::state();dp::comm::SourceId state_source{};state.domain_device_id().emplace().assign(device,16U);state.profile_id()=1;for(auto& joint:*state.joints())joint.configured_quality()=(63U&(1U<<*joint.joint_id()))!=0U;state_source.boot_id=7;state.reference_epoch()=3;state.core_state()=9;state.motion_epoch()=1000;state.management_epoch()=1608;state.operation_id()=2613;state.state_seq()=1;state.produced_at_us()=500000;for(auto& joint:*state.joints())joint.raw_position_valid_quality()=(63U&(1U<<*joint.joint_id()))!=0U;
 assert(c.publish_diagnostic_state(state,state_source,500200));
 assert(c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));
 assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,78,3));
 assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,4));
 assert(!c.readonly_fault_owner_retirement_confirmed(800201,7,77,3));
 c.raw_evidence_failure=1;assert(c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));assert(c.raw_evidence_failure==1);c.raw_evidence_failure=0;

 c.publish_worker_release_commit({42,2614,494,7,77,0,3,999,1608});assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));c.publish_worker_release_commit({42,2613,494,7,77,0,3,999,1608});
 c.publish_worker_release_commit({0,2613,494,7,77,0,3,999,1608});assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));c.publish_worker_release_commit({42,2613,494,7,77,0,3,999,1608});
 for(auto& joint:*state.joints())joint.raw_position_valid_quality()=(31U&(1U<<*joint.joint_id()))!=0U;state.state_seq()=2;state.produced_at_us()=500020;assert(c.publish_diagnostic_state(state,state_source,500220));assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));
 for(auto& joint:*state.joints())joint.raw_position_valid_quality()=(63U&(1U<<*joint.joint_id()))!=0U;state.state_seq()=3;state.produced_at_us()=500040;assert(c.publish_diagnostic_state(state,state_source,500240));assert(c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));
 for(unsigned mode=0;mode<9;++mode) {
   auto wrong=status;const auto wrong_source=status_source;
   if(mode==0)wrong.parent_run_id()=493;
   if(mode==1)wrong.attempt_kind()=1;
   if(mode==2)wrong.failure_source()=0x1008;
   if(mode==3)wrong.cleanup_state()=2;
   if(mode==4)wrong.failure_axis_mask()=31;
   if(mode==5)wrong.release_committed()=false;
   if(mode==6)wrong.real_motion_permitted()=true;
   if(mode==7)wrong.permit_remaining_us()=1;
   if(mode==8)wrong.session_id()=1;
   assert(c.publish_diagnostic_status(wrong,wrong_source,500241+mode));
   assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));
 }
 assert(c.publish_diagnostic_status(status,status_source,500249));state.state_seq()=4;state.produced_at_us()=500060;assert(c.publish_diagnostic_state(state,state_source,500260));assert(c.readonly_fault_owner_retirement_confirmed(500270,7,77,3));
 c.reset_raw_cleanup_lifetime();assert(c.readonly_fault_owner_retirement_confirmed(500270,7,77,3));
 c.ready=false;assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));c.reset_worker_release_connection();c.ready=true;assert(!c.readonly_fault_owner_retirement_confirmed(500270,7,77,3));
 return 0;
}
// 仅退休原连接：自身Motion根已完整落盘失能，不把Goal/Stop改成成功，也不发送管理请求。
void autonomous_fault_owner_retirement_proof() {
 using namespace dummy_ros_control;
 for(unsigned mode=0;mode<31;++mode) {
  TransportParameters p;p.manual_reference_mode=true;p.profile_id=1;p.configured_mask=63;p.max_state_age_us=250000;p.device_id[0]=1;
  DeviceManagementChannel c(p);c.ready=true;
  RawEvidenceTerminalFlushed terminal;terminal.boot_epoch=7;terminal.session_id=77;terminal.run_id=9;
  terminal.attempt_kind=1;terminal.final_state=4;terminal.first_failure_source=0x100a;terminal.first_failure_axis_mask=63;
  terminal.release_committed=true;terminal.chain_valid=true;terminal.terminal_received_us=100;
  terminal.terminal_chain[0]=1;terminal.terminal_group_count=8;terminal.owner_retirement_disable_valid=true;
  terminal.owner_retirement_motion_epoch=21;terminal.connection_generation=1;
  // 真实失败序列含先取消再Fault，不满足旧的单一fault-stop相位分类器。
  terminal.full_fault_groups_valid=false;
  std::uint8_t device[16]{1};
  auto status=owned_fixture::status();dp::comm::SourceId status_source{};
  status.domain_device_id().emplace().assign(device,16U);status.profile_id()=1;status.configured_mask()=63;status_source.boot_id=7;status.reference_epoch()=3;
  status.core_state()=9;status.motion_epoch()=21;status.management_epoch()=36;status.operation_id()=33;
  status.group_count()=8;status.run_id()=9;status.attempt_id()=9;status.attempt_kind()=1;status.final_state()=4;
  status.failure_source()=0x100a;status.failure_axis_mask()=63;status.cleanup_state()=3;status.release_committed()=true;
  auto state=owned_fixture::state();dp::comm::SourceId state_source{};
  state.domain_device_id().emplace().assign(device,16U);state.profile_id()=1;for(auto& joint:*state.joints())joint.configured_quality()=(63U&(1U<<*joint.joint_id()))!=0U;state_source.boot_id=7;state.reference_epoch()=3;
  state.core_state()=9;state.motion_epoch()=21;state.management_epoch()=36;state.operation_id()=33;
  state.state_seq()=1;state.produced_at_us()=500000;for(auto& joint:*state.joints())joint.raw_position_valid_quality()=(63U&(1U<<*joint.joint_id()))!=0U;
  if(mode==1)terminal.owner_retirement_disable_valid=false;
  if(mode==2)terminal.connection_generation=0;
  if(mode==3)terminal.session_id=78;
  if(mode==4)terminal.boot_epoch=8;
  if(mode==5)terminal.run_id=8;
  if(mode==6)terminal.parent_run_id=8;
  if(mode==7)terminal.chain_valid=false;
  if(mode==8)terminal.release_committed=false;
  if(mode==9)terminal.evidence_lost=true;
  if(mode==10)terminal.motion_complete=true;
  if(mode==11)terminal.owner_retirement_motion_epoch=20;
  if(mode==12)terminal.identity_hashes[0][0]=1;
  if(mode==13)status.cleanup_state()=2;
  if(mode==14)status.release_committed()=false;
  if(mode==15)status.real_motion_permitted()=true;
  if(mode==16)status.permit_remaining_us()=1;
  if(mode==17)status.evidence_lost()=true;
  if(mode==18)status.failure_source()=0x1008;
  if(mode==19)state.reference_epoch()=4;
  if(mode==20)state.session_id()=1;
  if(mode==21)state.operation_id()=34;
  if(mode==22)terminal.terminal_chain={};
  if(mode==23)terminal.terminal_received_us=500230;
  if(mode==27) {terminal.first_failure_axis_mask=2;status.failure_axis_mask()=2;}
  if(mode==28)for(auto& joint:*state.joints())joint.raw_position_valid_quality()=(0U&(1U<<*joint.joint_id()))!=0U;
  if(mode==29) {terminal.first_failure_axis_mask=64;status.failure_axis_mask()=64;}
  if(mode==30) {terminal.first_failure_source=0x1008;status.failure_source()=0x1008;}
  c.publish_raw_evidence_terminal_flushed(terminal);
  assert(c.publish_diagnostic_status(status,status_source,200));
  const bool state_ok=c.publish_diagnostic_state(state,state_source,500200);
  if(mode!=19 && mode!=20)assert(state_ok);
  if(mode==24)c.reset_worker_release_connection();
  const auto now=mode==25?800201U:500250U;
  const auto run=mode==26?10U:9U;
  assert(c.autonomous_fault_owner_retirement_confirmed(now,7,77,3,run)==(mode==0 || mode==27 || mode==28 || mode==30));
 }
}
// 无Host D10且原collector失败时，真实自主释放仅允许退休本连接曾提交成功的owner。
void readonly_released_owner_observation_proof() {
 using namespace dummy_ros_control;
 for(unsigned mode=0;mode<36;++mode) {
  TransportParameters p;p.manual_reference_mode=true;p.profile_id=1;p.configured_mask=63;p.max_state_age_us=250000;p.device_id[0]=1;p.client_instance_id[0]=2;
  DeviceManagementChannel c(p);c.ready=true;
  std::uint8_t device[16]{1},client[16]{2},foreign[16]{3};
  auto request=owned_fixture::request();dp::comm::SourceId request_source{};
  request.request_id()=42;request.operation()=5;request.domain_device_id().emplace().assign(device,16U);request.client_instance_id().emplace().assign(client,16U);
  request.expected_boot_epoch()=7;request.expected_session_id()=77;request.expected_profile_id()=1;request.configured_mask()=63;
  auto reply=owned_fixture::response();dp::comm::SourceId reply_source{};
  reply.request_id()=42;reply.domain_device_id().emplace().assign(device,16U);reply.client_instance_id().emplace().assign(client,16U);reply.profile_id()=1;
  reply_source.boot_id=7;reply.session_id()=77;reply.reference_epoch()=3;reply.motion_epoch()=38;reply.management_epoch()=70;
  reply.run_id()=16;reply.operation_id()=76;reply.receipt()=1;reply.transaction_phase()=3;
  if(mode==34 || mode==35)reply.run_id()=0;
  if(mode==1)request.expected_session_id()=78;
  if(mode==2)request.client_instance_id().emplace().assign(foreign,16U);
  if(mode==3)reply.transaction_phase()=2;
  if(mode==4)reply_source.boot_id=8;
  if(mode==5)reply.request_id()=43;
  if(mode==31)c.reset_worker_release_connection();
  if(mode!=6)c.observe_owned_management_commit(request,reply,reply_source,1);
  if(mode==7)c.reset_worker_release_connection();
  auto status=owned_fixture::status();dp::comm::SourceId status_source{};
  status.domain_device_id().emplace().assign(device,16U);status.profile_id()=1;status.configured_mask()=63;status_source.boot_id=7;status.reference_epoch()=3;
  status.core_state()=2;status.motion_epoch()=39;status.management_epoch()=71;status.operation_id()=76;
  status.run_id()=17;status.attempt_id()=17;status.attempt_kind()=1;status.final_state()=2;status.cleanup_state()=3;status.release_committed()=true;
  if(mode==8)status_source.boot_id=8;
  if(mode==12)status.session_id()=1;
  if(mode==13)status.permit_remaining_us()=1;
  if(mode==14)status.real_motion_permitted()=true;
  if(mode==15)status.cleanup_state()=2;
  if(mode==16)status.release_committed()=false;
  if(mode==17)status.evidence_lost()=true;
  if(mode==18)status.core_state()=9;
  if(mode==19)status.attempt_kind()=2;
  if(mode==20)status.parent_run_id()=1;
  if(mode==21)status.final_state()=6;
  if(mode==22) {status.run_id()=15;status.attempt_id()=15;}
  if(mode==23)status.motion_epoch()=37;
  if(mode==24)status.management_epoch()=69;
  if(mode==26)status.failure_source()=0x1068;
  if(mode==33) {status.run_id()=15;status.attempt_id()=15;}
  if(mode==35) {status.run_id()=0;status.attempt_id()=0;}
  if(mode==30) {status.attempt_kind()=3;status.parent_run_id()=16;status.final_state()=8;}
  assert(c.publish_diagnostic_status(status,status_source,200));
  auto state=owned_fixture::state();dp::comm::SourceId state_source{};
  state.domain_device_id().emplace().assign(device,16U);state.profile_id()=1;for(auto& joint:*state.joints())joint.configured_quality()=(63U&(1U<<*joint.joint_id()))!=0U;state_source.boot_id=status_source.boot_id;state.reference_epoch()=3;
  state.session_id()=*status.session_id();state.core_state()=*status.core_state();state.motion_epoch()=*status.motion_epoch();
  state.management_epoch()=*status.management_epoch();state.operation_id()=mode==25?75:76;state.state_seq()=1;state.produced_at_us()=500000;
  const auto received=mode==28?1000201U:500200U;
  const bool published=c.publish_diagnostic_state(state,state_source,received);assert(published==(mode!=28));
  if(mode==29)c.raw_evidence_failure=1;
  const auto now=mode==27?800201U:received+50U;
  const bool observed=c.readonly_released_owner_observed(now,mode==10?8:7,mode==9?78:77,mode==11?4:3,mode>=32?0:16);
  assert(observed==(mode==0 || mode==29 || mode==30 || mode==32 || mode==34));
  if(mode==29)assert(c.raw_evidence_failure==1);
 }
}
// SafetyStop仅排意图：原collector错误不能变成运动授权，也不能触发重复D10。
void owner_safety_stop_intent_proof() {
 using namespace dummy_ros_control;
 for(unsigned mode=0;mode<9;++mode) {
  TransportParameters p;p.manual_reference_mode=true;p.profile_id=1;p.configured_mask=63;p.max_state_age_us=250000;p.device_id[0]=1;p.client_instance_id[0]=2;
  DeviceManagementChannel c(p);c.ready=true;std::uint8_t device[16]{1},client[16]{2};
  auto q=owned_fixture::request();dp::comm::SourceId q_source{};q.request_id()=42;q.operation()=5;q.domain_device_id().emplace().assign(device,16U);q.client_instance_id().emplace().assign(client,16U);q.expected_boot_epoch()=7;q.expected_session_id()=77;q.expected_profile_id()=1;q.configured_mask()=63;
  auto r=owned_fixture::response();dp::comm::SourceId r_source{};r.request_id()=42;r.domain_device_id().emplace().assign(device,16U);r.client_instance_id().emplace().assign(client,16U);r.profile_id()=1;r_source.boot_id=7;r.session_id()=77;r.reference_epoch()=3;r.motion_epoch()=38;r.management_epoch()=70;r.operation_id()=76;r.receipt()=1;r.transaction_phase()=3;
  if(mode!=1)assert(c.observe_owned_management_commit(q,r,r_source,1));
  auto status=owned_fixture::status();dp::comm::SourceId status_source{};status.domain_device_id().emplace().assign(device,16U);status.profile_id()=1;status.configured_mask()=63;status_source.boot_id=7;status.session_id()=77;status.reference_epoch()=3;status.core_state()=4;status.motion_epoch()=38;status.management_epoch()=70;status.operation_id()=76;
  assert(c.publish_diagnostic_status(status,status_source,200));
  auto state=owned_fixture::state();dp::comm::SourceId state_source{};state.domain_device_id().emplace().assign(device,16U);state.profile_id()=1;for(auto& joint:*state.joints())joint.configured_quality()=(63U&(1U<<*joint.joint_id()))!=0U;state_source.boot_id=7;state.session_id()=77;state.reference_epoch()=3;state.core_state()=4;state.motion_epoch()=38;state.management_epoch()=70;state.operation_id()=76;state.state_seq()=1;state.freshness_ticket()=1;state.produced_at_us()=500000;
  assert(c.publish_diagnostic_state(state,state_source,500200));c.observation=c.diagnostic_observation();
  if(mode==2)c.reset_worker_release_connection();
  if(mode==3)c.observation.state_received_us=0;
  if(mode==4) {auto wrong=state;wrong.motion_epoch()=37;c.observation.state=wrong;}
  c.raw_evidence_failure=1;
  const auto now=mode==5?800201U:500250U;
  const bool queued=c.request_owner_safety_stop(now,7,mode==6?78:77,mode==7?4:3);
  assert(queued==(mode==0 || mode==8));
  if(queued)assert(!c.request_owner_safety_stop(now,7,77,3));
  if(mode==8)c.reset_worker_release_connection();
  const auto taken=c.take_owner_safety_stop_request(now);
  assert(bool(taken)==(mode==0));assert(!c.take_owner_safety_stop_request(now));
  assert(c.raw_evidence_failure==1);
 }
}
int local_readonly_retirement_proof() {
 using namespace dummy_ros_control;
 TransportParameters p;p.manual_reference_mode=true;p.profile_id=1;p.configured_mask=63;p.device_id[0]=1;
 DeviceManagementChannel c(p);c.ready=true;c.raw_evidence_failure=1;
 assert(c.local_readonly_retirement_allowed(1));
 assert(c.raw_evidence_failure==1);
 c.note_management_attempt(7);
 assert(!c.local_readonly_retirement_allowed(1));
 c.reset_worker_release_connection();
 assert(!c.local_readonly_retirement_allowed(1));
 // 同一连接曾尝试控制时，只能凭新的boot和独立新鲜双源退休旧owner。
 std::uint8_t device[16]{1};
 auto status=owned_fixture::status();dp::comm::SourceId status_source{};status.domain_device_id().emplace().assign(device,16U);status.profile_id()=1;status.configured_mask()=63;status_source.boot_id=8;status.core_state()=1;
 auto state=owned_fixture::state();dp::comm::SourceId state_source{};state.domain_device_id().emplace().assign(device,16U);state.profile_id()=1;for(auto& joint:*state.joints())joint.configured_quality()=(63U&(1U<<*joint.joint_id()))!=0U;state_source.boot_id=8;state.core_state()=1;state.state_seq()=1;state.produced_at_us()=500000;
 assert(c.publish_diagnostic_status(status,status_source,500200));
 assert(!c.local_readonly_retirement_allowed(500250));
 assert(c.publish_diagnostic_state(state,state_source,500200));
 assert(c.local_readonly_retirement_allowed(500250));
 assert(!c.local_readonly_retirement_allowed(800201));
 auto publish=[&](){assert(c.publish_diagnostic_status(status,status_source,500210));state.state_seq()=*state.state_seq()+1;state.produced_at_us()=*state.produced_at_us()+10;assert(c.publish_diagnostic_state(state,state_source,500220));};
 status.real_motion_permitted()=true;publish();assert(!c.local_readonly_retirement_allowed(500250));
 status.real_motion_permitted()=false;status.session_id()=77;state.session_id()=77;publish();assert(!c.local_readonly_retirement_allowed(500250));
 status.session_id()=0;state.session_id()=0;status.core_state()=9;state.core_state()=9;publish();assert(!c.local_readonly_retirement_allowed(500250));
 status.core_state()=1;state.core_state()=1;publish();assert(c.local_readonly_retirement_allowed(500250));
 c.note_management_attempt(8);assert(!c.local_readonly_retirement_allowed(500250));
 DeviceManagementChannel pending(p);pending.ready=true;pending.queued=std::make_shared<DeviceManagementRequest>();assert(!pending.local_readonly_retirement_allowed(1));
 pending.queued->complete=true;assert(pending.local_readonly_retirement_allowed(1));
 assert(c.raw_evidence_failure==1);
 return 0;
}
int main(){
 local_readonly_retirement_proof();
 owner_safety_stop_intent_proof();
 readonly_released_owner_observation_proof();
 autonomous_fault_owner_retirement_proof();
 readonly_fault_owner_retirement_proof();
 {
  using namespace dummy_ros_control;
  TransportParameters p;p.device_id[0]=1;p.profile_id=1;p.configured_mask=63;p.max_state_age_us=100;
  DeviceManagementChannel diagnostic(p);diagnostic.ready=true;
  auto status=owned_fixture::status();dp::comm::SourceId status_source{};
  std::uint8_t device[16]{1};status.domain_device_id().emplace().assign(device,16U);status.profile_id()=1;status.configured_mask()=63;
  status_source.boot_id=7;status.reference_epoch()=1;status.core_state()=9;status.release_committed()=true;
  assert(diagnostic.publish_diagnostic_status(status,status_source,1990));
  auto state=owned_fixture::state();dp::comm::SourceId state_source{};
  state.domain_device_id().emplace().assign(device,16U);state.profile_id()=1;for(auto& joint:*state.joints())joint.configured_quality()=(63U&(1U<<*joint.joint_id()))!=0U;state_source.boot_id=7;
  state.reference_epoch()=1;state.core_state()=9;state.state_seq()=1;state.produced_at_us()=1000;
  assert(diagnostic.publish_diagnostic_state(state,state_source,2000));
  assert(diagnostic.diagnostic_observation().state_received_us==2000);
  assert(diagnostic.observation.state_received_us==0); // 诊断不建立正常授权观察。
  assert(diagnostic.raw_cleanup_observation().state_received_us==0); // 不伪造writer证据。
  assert(!diagnostic.publish_diagnostic_state(state,state_source,2001));
  assert(diagnostic.diagnostic_observation().state_received_us==2000);
  auto wrong=status;const auto wrong_source=status_source;std::uint8_t hash[32]{1};wrong.model_sha256().emplace().assign(hash,32U);
  assert(!diagnostic.publish_diagnostic_status(wrong,wrong_source,2002));
  state.state_seq()=2;state.produced_at_us()=1020;assert(diagnostic.publish_diagnostic_state(state,state_source,2021));
  assert(diagnostic.publish_diagnostic_status(status,status_source,2150));
  state.state_seq()=3;state.produced_at_us()=1021;assert(!diagnostic.publish_diagnostic_state(state,state_source,2160));
  status_source.boot_id=8;assert(diagnostic.publish_diagnostic_status(status,status_source,2200));
  assert(diagnostic.diagnostic_observation().state_received_us==0);
  state_source.boot_id=7;assert(!diagnostic.publish_diagnostic_state(state,state_source,2201));
  status_source.boot_id=7;assert(!diagnostic.publish_diagnostic_status(status,status_source,2202));
  diagnostic.ready=false;assert(diagnostic.diagnostic_observation().status_received_us==0);
 }

 using namespace dummy_ros_control;DeviceManagementChannel c(TransportParameters{});c.ready=true;
 auto b=owned_fixture::state();dp::comm::SourceId b_source{};b_source.boot_id=7;
 /**
  * @brief old是cleanup生存期generation，100是人工Host接收µs；二者用途不同。
  * 先证明原代次可发布，reset后再用同一old检查排队迟到记录不得复活缓存。
  */
 const auto old=c.raw_cleanup_generation();c.publish_raw_cleanup_observation(b,b_source,100,true,old);assert(c.raw_cleanup_observation().state_received_us==100);
 c.ready=false;c.reset_raw_cleanup_lifetime();assert(c.raw_cleanup_observation().state_received_us==0);
 c.ready=true;c.publish_raw_cleanup_observation(b,b_source,101,true,old);assert(c.raw_cleanup_observation().state_received_us==0);
 c.publish_raw_cleanup_observation(b,b_source,102,true,c.raw_cleanup_generation());assert(c.raw_cleanup_observation().state_received_us==102);
 c.publish_raw_cleanup_observation(b,b_source,103,false,c.raw_cleanup_generation());assert(c.raw_cleanup_observation().state_received_us==0);
 std::puts("stoppedGetterEmpty/restartOldQueuedRecordRejected/newGenerationAccepted/invalidProducerStampCleared PASS");
}
