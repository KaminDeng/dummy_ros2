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
