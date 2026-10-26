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
int main(){
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
