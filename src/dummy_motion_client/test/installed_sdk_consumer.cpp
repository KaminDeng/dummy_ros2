/** @brief 独立安装空间中的公开头、真实PIC运行时及管理类型消费，无网络。 */
#include "dummy_ros_control/dp_comm_worker.hpp"
#include "dummy_motion_client/management_client.hpp"
#include "dp_protocol_wire_contract.h"
#include <iostream>

namespace {
using Message=dp::iface::DummyJointCommandV2;
void receive(dp::comm::BorrowedMessage<Message> message,void* context) {
  Message copy{};
  if(message.copy_to(&copy)==dp::comm::Result::kOk && copy.producer_seq()==17U)
    ++*static_cast<unsigned*>(context);
}
}
int main() {
  static_assert(DP_COMM_RUNTIME_MAX_MESSAGE_BYTES==651);
  static_assert(DP_COMM_RUNTIME_STORAGE_BYTES==18000);
  dp::comm::StaticStorage storage({1,1,1,0,4,651,96});
  auto initialized=dp::comm::Context::init(storage,dp::comm::MemoryMode::kStaticOnly);
  if(!initialized.has_value())return 1;
  auto context=std::move(initialized).take_value();
  dp::comm::NodeOptions options{};options.product_name="dummy_robot";options.node_name="installed_consumer";options.device_id.bytes[0]=1U;
  auto made=context.create_node(options);if(!made.has_value())return 2;
  auto node=std::move(made).take_value();unsigned received=0;
  auto publishing=node.create_publisher<Message>("installed",{});
  auto subscribing=node.create_subscription<Message>("installed",{},receive,&received);
  if(!publishing.has_value() || !subscribing.has_value())return 3;
  auto publisher=std::move(publishing).take_value();auto subscriber=std::move(subscribing).take_value();
  if(node.activate()!=dp::comm::Result::kOk)return 4;
  Message value{};value.set_schema_major(dummy_ref::protocol::kSchemaMajor);
  value.set_schema_minor(dummy_ref::protocol::kSchemaMinor);value.set_producer_seq(17U);
  if(publisher.publish(value)!=dp::comm::Result::kOk)return 5;
  dp::comm::StaticExecutor executor;
  const auto added=executor.add_node(node);const auto spun=executor.spin_some();
  if(added!=dp::comm::Result::kOk || spun!=dp::comm::Result::kOk || received!=1) {
    std::cerr<<"added="<<static_cast<int>(added)<<" spun="<<static_cast<int>(spun)<<" received="<<received<<'\n';return 6;
  }
  dummy_motion_client::ManageRequest request{};
  request.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  if(request.schema_minor()!=dummy_ref::protocol::kSchemaMinor)return 7;
  std::cout<<"installed_runtime=651/18000 wire="<<DP_PROTOCOL_WIRE_CONTRACT_SHA256<<'\n';
}
