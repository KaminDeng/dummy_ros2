#include "dummy_motion_client/dp_management_transport.hpp"
#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>
using namespace dummy_motion_client;
namespace wire = dp::protocol::dummy_robot_v2;
using Service = dp::protocol::services::DummyManageV2RawFixed;
dp::comm::Result reply(const Service::Request &request, Service::Response *output,
           const dp::comm::RequestInfo &, void *) {
  Service::Response response;
  response.set_schema_major(2); response.set_request_id(request.request_id());
  response.set_device_id(request.device_id()); response.set_client_instance_id(request.client_instance_id());
  response.set_boot_epoch(request.expected_boot_epoch());
  response.set_session_id(request.expected_session_id());
  response.set_profile_id(request.expected_profile_id());
  response.set_receipt(1); response.set_transaction_phase(3); response.set_result_code(0);
  response.set_operation_id(88);
  *output=response;
  return dp::comm::Result::kOk;
}
int peer(const std::string &endpoint) {
  dp::comm::StaticStorage storage({1,0,0,0,8,225,96,1,0,0,1});
  dp::comm::TransportConfig config{"peer","",endpoint.c_str(),"management_test"};
  auto init=dp::comm::Context::init(storage,{dp::comm::MemoryMode::kStaticOnly,&config,nullptr});
  if (!init.has_value()) return 10;
  auto context=std::move(init).take_value();
  dp::comm::NodeOptions options{}; options.product_name="dummy_robot";
  options.device_id.bytes[0]=1; options.node_name="ref_control";
  auto made=context.create_node(options); if(!made.has_value()) return 11;
  auto node=std::move(made).take_value();
  auto result=node.create_service<Service>("manage_v2",reply,nullptr);
  if(!result.has_value()) return 12;
  auto service=std::move(result).take_value();
  dp::comm::StaticExecutor executor; executor.add_node(node); node.activate();
  for(unsigned i=0;i<4000;++i) executor.spin_some(dp::comm::Duration::from_milliseconds(2));
  return 0;
}
int main() {
  const std::string endpoint="tcp/127.0.0.1:"+std::to_string(24000+getpid()%10000);
  const pid_t child=fork(); if(child==0) _exit(peer(endpoint));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  DpManagementParameters config{endpoint,{1},{2},1,1};
  DpManagementTransport transport(config);
  if(!transport.start()) return 1;
  for(unsigned i=0;i<500 && !transport.ready();++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  if(!transport.ready()) return 2;
  dp::comm::StaticStorage second_storage({1,0,0,0,8,618,96});
  const dp::comm::TransportConfig second_config{"client",endpoint.c_str(),"","second_context"};
  const auto second=dp::comm::Context::init(second_storage,{dp::comm::MemoryMode::kStaticOnly,&second_config,nullptr});
  if(second.result()!=dp::comm::Result::kBusy) return 8;
  ManageRequest request; request.set_schema_major(2); request.set_request_id(0x20000000000001ULL);
  std::vector<std::uint8_t> bytes(wire::DummyManageV2RequestCodec::kEncodedSize);
  wire::DummyManageV2RequestCodec::encode(request,bytes.data(),bytes.size());
  auto handle=transport.send(bytes); if(!handle) return 3;
  std::optional<std::vector<std::uint8_t>> response;
  for(unsigned i=0;i<1000 && !response;++i) {
    const auto before=std::chrono::steady_clock::now(); response=handle->poll();
    if(std::chrono::steady_clock::now()-before>std::chrono::milliseconds(20)) return 4;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if(!response) return 5;
  Service::Response decoded;
  if(wire::DummyManageV2ResponseCodec::decode(response->data(),response->size(),&decoded)!=wire::CodecResult::kOk ||
     decoded.request_id()!=request.request_id() || decoded.operation_id()!=88) return 6;
  // Destroying an outstanding facade cancels on the worker, never on a caller thread.
  handle=transport.send(bytes); handle.reset();
  handle=transport.send(bytes); transport.stop(); handle.reset();
  kill(child,SIGTERM); waitpid(child,nullptr,0);
  std::cout << "{\"evidence_scope\":\"component_only\",\"cross_process_manage_v2\":true,\"poll_nonblocking\":true}\n";
}
