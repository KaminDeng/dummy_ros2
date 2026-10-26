/**
 * @file raw_cleanup_lifetime_generation_tests.cpp
 * @brief 验证raw观测缓存跨cleanup代次拒绝迟到发布。
 *
 * 使用少数字节人工raw夹具和主机接收µs；不解码真实State/Status，不证明Source时间映射或设备反馈有效。
 */
#include "dummy_ros_control/device_management_channel.hpp"
#include <cstdio>
#include <cassert>
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
 dp::iface::DummyDeviceStatusV2 status;status.set_schema_major(2);status.set_schema_minor(dummy_wire_schema::kSchemaMinor);status.set_device_id(device);status.set_profile_id(1);status.set_configured_mask(63);status.set_boot_epoch(7);status.set_reference_epoch(3);status.set_core_state(9);status.set_motion_epoch(1000);status.set_management_epoch(1608);status.set_operation_id(2613);status.set_run_id(495);status.set_attempt_id(495);status.set_attempt_kind(2);status.set_parent_run_id(494);status.set_final_state(4);status.set_failure_source(0x1002);status.set_failure_axis_mask(63);status.set_cleanup_state(3);status.set_release_committed(true);
 assert(c.publish_diagnostic_status(status,200));
 dp::iface::DummyJointStateV2 state;state.set_schema_major(2);state.set_schema_minor(dummy_wire_schema::kSchemaMinor);state.set_device_id(device);state.set_profile_id(1);state.set_configured_mask(63);state.set_boot_epoch(7);state.set_reference_epoch(3);state.set_core_state(9);state.set_motion_epoch(1000);state.set_management_epoch(1608);state.set_operation_id(2613);state.set_state_seq(1);state.set_produced_at_us(500000);state.set_raw_position_valid_mask(63);
 assert(c.publish_diagnostic_state(state,500200));
 assert(c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));
 assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,78,3));
 assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,4));
 assert(!c.readonly_fault_owner_retirement_confirmed(800201,7,77,3));
 c.raw_evidence_failure=1;assert(c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));assert(c.raw_evidence_failure==1);c.raw_evidence_failure=0;

 c.publish_worker_release_commit({42,2614,494,7,77,0,3,999,1608});assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));c.publish_worker_release_commit({42,2613,494,7,77,0,3,999,1608});
 c.publish_worker_release_commit({0,2613,494,7,77,0,3,999,1608});assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));c.publish_worker_release_commit({42,2613,494,7,77,0,3,999,1608});
 state.set_raw_position_valid_mask(31);state.set_state_seq(2);state.set_produced_at_us(500020);assert(c.publish_diagnostic_state(state,500220));assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));
 state.set_raw_position_valid_mask(63);state.set_state_seq(3);state.set_produced_at_us(500040);assert(c.publish_diagnostic_state(state,500240));assert(c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));
 for(unsigned mode=0;mode<9;++mode) {
   auto wrong=status;
   if(mode==0)wrong.set_parent_run_id(493);
   if(mode==1)wrong.set_attempt_kind(1);
   if(mode==2)wrong.set_failure_source(0x1008);
   if(mode==3)wrong.set_cleanup_state(2);
   if(mode==4)wrong.set_failure_axis_mask(31);
   if(mode==5)wrong.set_release_committed(false);
   if(mode==6)wrong.set_real_motion_permitted(true);
   if(mode==7)wrong.set_permit_remaining_us(1);
   if(mode==8)wrong.set_session_id(1);
   assert(c.publish_diagnostic_status(wrong,500241+mode));
   assert(!c.readonly_fault_owner_retirement_confirmed(500250,7,77,3));
 }
 assert(c.publish_diagnostic_status(status,500249));state.set_state_seq(4);state.set_produced_at_us(500060);assert(c.publish_diagnostic_state(state,500260));assert(c.readonly_fault_owner_retirement_confirmed(500270,7,77,3));
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
  dp::iface::DummyDeviceStatusV2 status;status.set_schema_major(2);status.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  status.set_device_id(device);status.set_profile_id(1);status.set_configured_mask(63);status.set_boot_epoch(7);status.set_reference_epoch(3);
  status.set_core_state(9);status.set_motion_epoch(21);status.set_management_epoch(36);status.set_operation_id(33);
  status.set_group_count(8);status.set_run_id(9);status.set_attempt_id(9);status.set_attempt_kind(1);status.set_final_state(4);
  status.set_failure_source(0x100a);status.set_failure_axis_mask(63);status.set_cleanup_state(3);status.set_release_committed(true);
  dp::iface::DummyJointStateV2 state;state.set_schema_major(2);state.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  state.set_device_id(device);state.set_profile_id(1);state.set_configured_mask(63);state.set_boot_epoch(7);state.set_reference_epoch(3);
  state.set_core_state(9);state.set_motion_epoch(21);state.set_management_epoch(36);state.set_operation_id(33);
  state.set_state_seq(1);state.set_produced_at_us(500000);state.set_raw_position_valid_mask(63);
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
  if(mode==13)status.set_cleanup_state(2);
  if(mode==14)status.set_release_committed(false);
  if(mode==15)status.set_real_motion_permitted(true);
  if(mode==16)status.set_permit_remaining_us(1);
  if(mode==17)status.set_evidence_lost(true);
  if(mode==18)status.set_failure_source(0x1008);
  if(mode==19)state.set_reference_epoch(4);
  if(mode==20)state.set_session_id(1);
  if(mode==21)state.set_operation_id(34);
  if(mode==22)terminal.terminal_chain={};
  if(mode==23)terminal.terminal_received_us=500230;
  if(mode==27) {terminal.first_failure_axis_mask=2;status.set_failure_axis_mask(2);}
  if(mode==28)state.set_raw_position_valid_mask(0);
  if(mode==29) {terminal.first_failure_axis_mask=64;status.set_failure_axis_mask(64);}
  if(mode==30) {terminal.first_failure_source=0x1008;status.set_failure_source(0x1008);}
  c.publish_raw_evidence_terminal_flushed(terminal);
  assert(c.publish_diagnostic_status(status,200));
  const bool state_ok=c.publish_diagnostic_state(state,500200);
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
  dp::iface::DummyManageV2Request request;request.set_schema_major(2);request.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  request.set_request_id(42);request.set_operation(5);request.set_device_id(device);request.set_client_instance_id(client);
  request.set_expected_boot_epoch(7);request.set_expected_session_id(77);request.set_expected_profile_id(1);request.set_configured_mask(63);
  dp::iface::DummyManageV2Response reply;reply.set_schema_major(2);reply.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  reply.set_request_id(42);reply.set_device_id(device);reply.set_client_instance_id(client);reply.set_profile_id(1);
  reply.set_boot_epoch(7);reply.set_session_id(77);reply.set_reference_epoch(3);reply.set_motion_epoch(38);reply.set_management_epoch(70);
  reply.set_run_id(16);reply.set_operation_id(76);reply.set_receipt(1);reply.set_transaction_phase(3);
  if(mode==34 || mode==35)reply.set_run_id(0);
  if(mode==1)request.set_expected_session_id(78);
  if(mode==2)request.set_client_instance_id(foreign);
  if(mode==3)reply.set_transaction_phase(2);
  if(mode==4)reply.set_boot_epoch(8);
  if(mode==5)reply.set_request_id(43);
  if(mode==31)c.reset_worker_release_connection();
  if(mode!=6)c.observe_owned_management_commit(request,reply,1);
  if(mode==7)c.reset_worker_release_connection();
  dp::iface::DummyDeviceStatusV2 status;status.set_schema_major(2);status.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  status.set_device_id(device);status.set_profile_id(1);status.set_configured_mask(63);status.set_boot_epoch(7);status.set_reference_epoch(3);
  status.set_core_state(2);status.set_motion_epoch(39);status.set_management_epoch(71);status.set_operation_id(76);
  status.set_run_id(17);status.set_attempt_id(17);status.set_attempt_kind(1);status.set_final_state(2);status.set_cleanup_state(3);status.set_release_committed(true);
  if(mode==8)status.set_boot_epoch(8);
  if(mode==12)status.set_session_id(1);
  if(mode==13)status.set_permit_remaining_us(1);
  if(mode==14)status.set_real_motion_permitted(true);
  if(mode==15)status.set_cleanup_state(2);
  if(mode==16)status.set_release_committed(false);
  if(mode==17)status.set_evidence_lost(true);
  if(mode==18)status.set_core_state(9);
  if(mode==19)status.set_attempt_kind(2);
  if(mode==20)status.set_parent_run_id(1);
  if(mode==21)status.set_final_state(6);
  if(mode==22) {status.set_run_id(15);status.set_attempt_id(15);}
  if(mode==23)status.set_motion_epoch(37);
  if(mode==24)status.set_management_epoch(69);
  if(mode==26)status.set_failure_source(0x1068);
  if(mode==33) {status.set_run_id(15);status.set_attempt_id(15);}
  if(mode==35) {status.set_run_id(0);status.set_attempt_id(0);}
  if(mode==30) {status.set_attempt_kind(3);status.set_parent_run_id(16);status.set_final_state(8);}
  assert(c.publish_diagnostic_status(status,200));
  dp::iface::DummyJointStateV2 state;state.set_schema_major(2);state.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  state.set_device_id(device);state.set_profile_id(1);state.set_configured_mask(63);state.set_boot_epoch(status.boot_epoch());state.set_reference_epoch(3);
  state.set_session_id(status.session_id());state.set_core_state(status.core_state());state.set_motion_epoch(status.motion_epoch());
  state.set_management_epoch(status.management_epoch());state.set_operation_id(mode==25?75:76);state.set_state_seq(1);state.set_produced_at_us(500000);
  const auto received=mode==28?1000201U:500200U;
  const bool published=c.publish_diagnostic_state(state,received);assert(published==(mode!=28));
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
  dp::iface::DummyManageV2Request q;q.set_schema_major(2);q.set_schema_minor(dummy_wire_schema::kSchemaMinor);q.set_request_id(42);q.set_operation(5);q.set_device_id(device);q.set_client_instance_id(client);q.set_expected_boot_epoch(7);q.set_expected_session_id(77);q.set_expected_profile_id(1);q.set_configured_mask(63);
  dp::iface::DummyManageV2Response r;r.set_schema_major(2);r.set_schema_minor(dummy_wire_schema::kSchemaMinor);r.set_request_id(42);r.set_device_id(device);r.set_client_instance_id(client);r.set_profile_id(1);r.set_boot_epoch(7);r.set_session_id(77);r.set_reference_epoch(3);r.set_motion_epoch(38);r.set_management_epoch(70);r.set_operation_id(76);r.set_receipt(1);r.set_transaction_phase(3);
  if(mode!=1)assert(c.observe_owned_management_commit(q,r,1));
  dp::iface::DummyDeviceStatusV2 status;status.set_schema_major(2);status.set_schema_minor(dummy_wire_schema::kSchemaMinor);status.set_device_id(device);status.set_profile_id(1);status.set_configured_mask(63);status.set_boot_epoch(7);status.set_session_id(77);status.set_reference_epoch(3);status.set_core_state(4);status.set_motion_epoch(38);status.set_management_epoch(70);status.set_operation_id(76);
  assert(c.publish_diagnostic_status(status,200));
  dp::iface::DummyJointStateV2 state;state.set_schema_major(2);state.set_schema_minor(dummy_wire_schema::kSchemaMinor);state.set_device_id(device);state.set_profile_id(1);state.set_configured_mask(63);state.set_boot_epoch(7);state.set_session_id(77);state.set_reference_epoch(3);state.set_core_state(4);state.set_motion_epoch(38);state.set_management_epoch(70);state.set_operation_id(76);state.set_state_seq(1);state.set_freshness_ticket(1);state.set_produced_at_us(500000);
  assert(c.publish_diagnostic_state(state,500200));c.observation=c.diagnostic_observation();
  if(mode==2)c.reset_worker_release_connection();
  if(mode==3)c.observation.state_received_us=0;
  if(mode==4) {auto wrong=state;wrong.set_motion_epoch(37);dp::iface::DummyJointStateV2Codec::encode(wrong,c.observation.state.data(),c.observation.state.size());}
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
 dp::iface::DummyDeviceStatusV2 status;status.set_schema_major(2);status.set_schema_minor(dummy_wire_schema::kSchemaMinor);status.set_device_id(device);status.set_profile_id(1);status.set_configured_mask(63);status.set_boot_epoch(8);status.set_core_state(1);
 dp::iface::DummyJointStateV2 state;state.set_schema_major(2);state.set_schema_minor(dummy_wire_schema::kSchemaMinor);state.set_device_id(device);state.set_profile_id(1);state.set_configured_mask(63);state.set_boot_epoch(8);state.set_core_state(1);state.set_state_seq(1);state.set_produced_at_us(500000);
 assert(c.publish_diagnostic_status(status,500200));
 assert(!c.local_readonly_retirement_allowed(500250));
 assert(c.publish_diagnostic_state(state,500200));
 assert(c.local_readonly_retirement_allowed(500250));
 assert(!c.local_readonly_retirement_allowed(800201));
 auto publish=[&](){assert(c.publish_diagnostic_status(status,500210));state.set_state_seq(state.state_seq()+1);state.set_produced_at_us(state.produced_at_us()+10);assert(c.publish_diagnostic_state(state,500220));};
 status.set_real_motion_permitted(true);publish();assert(!c.local_readonly_retirement_allowed(500250));
 status.set_real_motion_permitted(false);status.set_session_id(77);state.set_session_id(77);publish();assert(!c.local_readonly_retirement_allowed(500250));
 status.set_session_id(0);state.set_session_id(0);status.set_core_state(9);state.set_core_state(9);publish();assert(!c.local_readonly_retirement_allowed(500250));
 status.set_core_state(1);state.set_core_state(1);publish();assert(c.local_readonly_retirement_allowed(500250));
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
  dp::iface::DummyDeviceStatusV2 status;status.set_schema_major(2);status.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  std::uint8_t device[16]{1};status.set_device_id(device);status.set_profile_id(1);status.set_configured_mask(63);
  status.set_boot_epoch(7);status.set_reference_epoch(1);status.set_core_state(9);status.set_release_committed(true);
  assert(diagnostic.publish_diagnostic_status(status,1990));
  dp::iface::DummyJointStateV2 state;state.set_schema_major(2);state.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  state.set_device_id(device);state.set_profile_id(1);state.set_configured_mask(63);state.set_boot_epoch(7);
  state.set_reference_epoch(1);state.set_core_state(9);state.set_state_seq(1);state.set_produced_at_us(1000);
  assert(diagnostic.publish_diagnostic_state(state,2000));
  assert(diagnostic.diagnostic_observation().state_received_us==2000);
  assert(diagnostic.observation.state_received_us==0); // 诊断不建立正常授权观察。
  assert(diagnostic.raw_cleanup_observation().state_received_us==0); // 不伪造writer证据。
  assert(!diagnostic.publish_diagnostic_state(state,2001));
  assert(diagnostic.diagnostic_observation().state_received_us==2000);
  auto wrong=status;std::uint8_t hash[32]{1};wrong.set_model_sha256(hash);
  assert(!diagnostic.publish_diagnostic_status(wrong,2002));
  state.set_state_seq(2);state.set_produced_at_us(1020);assert(diagnostic.publish_diagnostic_state(state,2021));
  assert(diagnostic.publish_diagnostic_status(status,2150));
  state.set_state_seq(3);state.set_produced_at_us(1021);assert(!diagnostic.publish_diagnostic_state(state,2160));
  status.set_boot_epoch(8);assert(diagnostic.publish_diagnostic_status(status,2200));
  assert(diagnostic.diagnostic_observation().state_received_us==0);
  state.set_boot_epoch(7);assert(!diagnostic.publish_diagnostic_state(state,2201));
  status.set_boot_epoch(7);assert(!diagnostic.publish_diagnostic_status(status,2202));
  diagnostic.ready=false;assert(diagnostic.diagnostic_observation().status_received_us==0);
 }

 using namespace dummy_ros_control;DeviceManagementChannel c(TransportParameters{});c.ready=true;
 std::array<std::uint8_t,dp::iface::DummyJointStateV2Codec::kEncodedSize> b{};
 /**
  * @brief old是cleanup生存期generation，100是人工Host接收µs；二者用途不同。
  * 先证明原代次可发布，reset后再用同一old检查排队迟到记录不得复活缓存。
  */
 const auto old=c.raw_cleanup_generation();c.publish_raw_cleanup_observation(true,b.data(),b.size(),100,true,old);assert(c.raw_cleanup_observation().state_received_us==100);
 c.ready=false;c.reset_raw_cleanup_lifetime();assert(c.raw_cleanup_observation().state_received_us==0);
 c.ready=true;c.publish_raw_cleanup_observation(true,b.data(),b.size(),101,true,old);assert(c.raw_cleanup_observation().state_received_us==0);
 c.publish_raw_cleanup_observation(true,b.data(),b.size(),102,true,c.raw_cleanup_generation());assert(c.raw_cleanup_observation().state_received_us==102);
 c.publish_raw_cleanup_observation(true,b.data(),b.size(),103,false,c.raw_cleanup_generation());assert(c.raw_cleanup_observation().state_received_us==0);
 std::puts("stoppedGetterEmpty/restartOldQueuedRecordRejected/newGenerationAccepted/invalidProducerStampCleared PASS");
}
