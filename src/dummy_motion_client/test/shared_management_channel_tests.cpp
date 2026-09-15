#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
int main() {
  dummy_ros_control::TransportParameters parameters{"tcp/127.0.0.1:29999",{1},{2},2,1};
  dummy_ros_control::GuardedZenbeddedSystem system;
  dummy_ros_control::DeviceTransport provider(system,parameters);
  const auto channel=dummy_ros_control::DeviceTransport::management_channel(parameters);
  if(!channel) return 1;
  channel->ready=true; channel->observation.state_received_us=100; channel->observation.status_received_us=100;
  dummy_motion_client::DpManagementTransport facade({parameters.endpoint,{1},{2},2,1},
      dummy_motion_client::DpManagementTransport::Connection::kDeviceWorker);
  if(!facade.start() || !facade.ready() || !facade.observe()) return 2;
  if(!dummy_motion_client::observation_is_fresh(*facade.observe(),99) ||
     dummy_motion_client::observation_is_fresh(*facade.observe(),1000001)) return 7;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    const auto cached=facade.observe();
    if(!cached || cached->state_received_us!=100) return 3;
  }
  std::vector<std::uint8_t> bytes(233);
  auto first=facade.send(bytes); if(!first || facade.send(bytes)) return 4;
  first.reset(); if(!facade.send(bytes)) return 5;
  auto wrong=parameters; wrong.profile_id=3;
  if(dummy_ros_control::DeviceTransport::management_channel(wrong)) return 6;
  dummy_motion_client::WireManagementClient client(facade);
  dummy_motion_client::ManageRequest query;
  query.set_operation(dummy_motion_client::wire_value(dummy_motion_client::ManageOperation::kQuery));
  if(!client.submit(query,100)) return 8;
  auto cancel=query;
  cancel.set_operation(dummy_motion_client::wire_value(dummy_motion_client::ManageOperation::kCancel));
  if(!client.submit(cancel,101)) return 9;
  first.reset();
  channel->queued.reset();
  dummy_motion_client::WireManagementClient busy_client(facade);
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    if(!busy_client.submit(query,200)) return 10;
  }
  if(busy_client.poll(1) || !channel->queued) return 11;
}
