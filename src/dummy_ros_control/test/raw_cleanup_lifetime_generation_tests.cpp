#include "dummy_ros_control/device_management_channel.hpp"
#include <cstdio>
#include <cassert>
int main(){
 using namespace dummy_ros_control;DeviceManagementChannel c(TransportParameters{});c.ready=true;
 std::array<std::uint8_t,dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::kEncodedSize> b{};
 const auto old=c.raw_cleanup_generation();c.publish_raw_cleanup_observation(true,b.data(),b.size(),100,true,old);assert(c.raw_cleanup_observation().state_received_us==100);
 c.ready=false;c.reset_raw_cleanup_lifetime();assert(c.raw_cleanup_observation().state_received_us==0);
 c.ready=true;c.publish_raw_cleanup_observation(true,b.data(),b.size(),101,true,old);assert(c.raw_cleanup_observation().state_received_us==0);
 c.publish_raw_cleanup_observation(true,b.data(),b.size(),102,true,c.raw_cleanup_generation());assert(c.raw_cleanup_observation().state_received_us==102);
 c.publish_raw_cleanup_observation(true,b.data(),b.size(),103,false,c.raw_cleanup_generation());assert(c.raw_cleanup_observation().state_received_us==0);
 std::puts("stoppedGetterEmpty/restartOldQueuedRecordRejected/newGenerationAccepted/invalidProducerStampCleared PASS");
}
