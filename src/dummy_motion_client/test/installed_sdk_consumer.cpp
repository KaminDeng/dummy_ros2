/** @file installed_sdk_consumer.cpp
 * @brief 两仓源码不可见时消费实际安装的Root头、完整Owned、typed RPC及PIC运行库；不连设备。 */
#include "dummy_ros_control/dp_comm_worker.hpp"
#include "dummy_motion_client/management_client.hpp"
#include "dummy_ref/root_transport.hpp"
#include "dummy_ref/root_runtime_contract.hpp"
#include "field_groups.h"
#include "canonical_records.h"
#include <cassert>
#include <iostream>
#include <vector>
int main(){
  static_assert(DP_COMM_RUNTIME_MAX_MESSAGE_BYTES>=dummy_ref::root::kMessageBytes);
  static_assert(DP_COMM_RUNTIME_MAX_ROOT_EVENT_OWNED_BYTES>=sizeof(dummy_ref::messages::Status));
  dummy_ref::root::HostTransport owner;
  dp::comm::SourceId source{};source.device.bytes[0]=9;source.boot_id=UINT64_C(0xf123456789abcdef);source.context_id=7;
  dp::comm::DeviceId remote{};remote.bytes[0]=1;
  if(owner.configure(source,remote)!=dp::comm::Result::kOk)return 1;
  dp::comm::StaticStorageConfig config{};
  config.nodes=DP_COMM_RUNTIME_MAX_NODES;config.publishers=DP_COMM_RUNTIME_MAX_PUBLISHERS;
  config.subscriptions=DP_COMM_RUNTIME_MAX_SUBSCRIPTIONS;config.timers=DP_COMM_RUNTIME_MAX_TIMERS;
  config.event_queue=DP_COMM_RUNTIME_MAX_EVENTS;config.max_message_bytes=DP_COMM_RUNTIME_MAX_MESSAGE_BYTES;config.max_topic_bytes=DP_COMM_RUNTIME_MAX_TOPIC_BYTES;
  config.services=DP_COMM_RUNTIME_MAX_SERVICES;config.clients=DP_COMM_RUNTIME_MAX_CLIENTS;config.pending_requests=DP_COMM_RUNTIME_MAX_PENDING_REQUESTS;config.completed_responses=DP_COMM_RUNTIME_MAX_COMPLETED_RESPONSES;
  dp::comm::StaticStorage storage(config);
  auto initialized=dp::comm::Context::init(storage,dp::comm::MemoryMode::kStaticOnly);if(!initialized.has_value())return 2;
  auto context=std::move(initialized).take_value();dp::comm::NodeOptions options{};
  options.product_name="dummy_robot";options.node_name="installed_consumer";options.device_id=source.device;
  auto made=context.create_node(options);if(!made.has_value())return 3;auto node=std::move(made).take_value();
  const auto attached=owner.attach(node);const auto active=node.activate();
  if(attached!=dp::comm::Result::kOk || active!=dp::comm::Result::kOk){std::cerr<<"attach="<<int(attached)<<" activate="<<int(active)<<"\n";return 4;}
  auto request=dummy_ref::messages::make_manage_request();request.operation()=static_cast<std::uint16_t>(dp::protocol::dummy::ManagementOperation::GET_STATUS);
  request.expected_boot_epoch()=7;request.domain_device_id()->operator[](0)=1;request.client_instance_id()->operator[](0)=3;
  request.request_id()=UINT64_C(9007199254740993);request.request_seq()=17;
  if(!(dp::protocol::groups::dp_protocol_dummy_ManageRequest_Complete(request)&&dp::protocol::groups::dp_protocol_dummy_ManageRequest_DomainComplete(request)))return 5;
  using Traits=dp::protocol::canonical::Traits<dummy_ref::messages::ManageRequest>;
  std::vector<std::uint8_t> bytes(Traits::kMaxEncodedSize);std::size_t written=0;
  if(!Traits::encode(request,bytes.data(),bytes.size(),&written))return 6;dummy_ref::messages::ManageRequest restored;
  if(!Traits::decode(bytes.data(),written,&restored) || *restored.request_id()!=UINT64_C(9007199254740993) || !restored.operator_confirmed() || *restored.operator_confirmed())return 7;
  auto client=owner.sdk->client<dummy_ref::root::Manage>(dummy_ref::root::selected::dp_protocol_dummy_ManageRequest::kLogicalKey);if(!client.has_value())return 8;
  auto handle=std::move(client).take_value().call_async(request,dp::comm::Duration::from_milliseconds(1));
  if(handle.result()!=dp::comm::Result::kNotReady)return 9;
  std::cout<<"installed_root_runtime="<<DP_COMM_RUNTIME_MAX_MESSAGE_BYTES<<'/'<<DP_COMM_RUNTIME_STORAGE_BYTES<<" owned_event="<<DP_COMM_RUNTIME_MAX_ROOT_EVENT_OWNED_BYTES<<" source_boot="<<owner.session.source.boot_id<<'\n';
}
