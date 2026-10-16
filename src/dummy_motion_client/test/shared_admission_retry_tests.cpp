#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include <future>
#include <thread>
#include <iostream>
#include <cassert>
using namespace dummy_motion_client;
namespace wire=dp::protocol::dummy_robot_v2;
struct Holder {
 std::promise<void> locked,release;std::thread t;bool closed=false;
 explicit Holder(std::mutex&m){auto ready=release.get_future();t=std::thread([&,ready=std::move(ready)]()mutable{std::unique_lock<std::mutex>g(m);locked.set_value();ready.wait();});locked.get_future().wait();}
 void unlock(){if(!closed){release.set_value();t.join();closed=true;}}
 ~Holder(){unlock();}
};
int main(int argc,char**argv){
 std::string mode=argc>1?argv[1]:"select";
 dummy_ros_control::TransportParameters params{"tcp/127.0.0.1:29980",{1},{2},201,1};
 dummy_ros_control::GuardedZenbeddedSystem system;
 auto provider=std::make_unique<dummy_ros_control::DeviceTransport>(system,params);
 auto channel=dummy_ros_control::DeviceTransport::management_channel(params);assert(channel);channel->ready=true;
 auto facade=std::make_unique<DpManagementTransport>(DpManagementParameters{params.endpoint,{1},{2},201,1},DpManagementTransport::Connection::kDeviceWorker);assert(facade->start());
 ManageRequest req;req.set_schema_major(2);req.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);req.set_request_id(42);req.set_operation(3);req.set_expected_boot_epoch(7);req.set_expected_session_id(9);req.set_expected_reference_epoch(1);req.set_expected_motion_epoch(2);req.set_expected_management_epoch(3);req.set_expected_profile_id(201);
 std::vector<std::uint8_t>bytes(wire::DummyManageV2RequestCodec::kEncodedSize);assert(wire::DummyManageV2RequestCodec::encode(req,bytes.data(),bytes.size())==wire::CodecResult::kOk);
 if(mode=="queued_other"){auto other=std::make_shared<dummy_ros_control::DeviceManagementRequest>();channel->queued=other;assert(!facade->send(bytes));assert(channel->queued==other);return 0;}
 Holder holder(channel->mutex);
 if(mode=="wrong_size"){bytes.pop_back();assert(!facade->send(bytes));return 0;}
 if(mode=="invalid_codec"){bytes[0]=0;assert(!facade->send(bytes));return 0;}
 if(mode=="not_ready"){channel->ready=false;assert(!facade->send(bytes));return 0;}
 if(mode=="select"||mode=="grant"||mode=="capture"){
  WireManagementClient client(*facade);ManageRequest id;id.set_expected_session_id(9);
  const std::uint8_t device[16]{1}, clientid[16]{2};id.set_device_id(device);id.set_client_instance_id(clientid);
  BootstrapCoordinator boot(client,id,{},650000000,30000000);BootstrapSnapshot s{};s.boot_epoch=7;s.session_id=9;s.reference_epoch=1;s.motion_epoch=2;s.management_epoch=3;s.profile_id=201;s.configured_mask=63;s.freshness_ticket=1;s.raw_ready=s.motion_supported=true;s.control_mode=s.reference_kind=1;
  auto op=mode=="select"?ManageOperation::kSelectMode:mode=="grant"?ManageOperation::kGrantSession:ManageOperation::kCaptureReference;
  const auto now=management_now_us();assert(boot.start_operation(s,op,now));assert(!channel->queued);holder.unlock();boot.tick(management_now_us());assert(!channel->queued);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));boot.tick(management_now_us());assert(channel->queued);auto pending=channel->queued;ManageRequest queued;assert(wire::DummyManageV2RequestCodec::decode(pending->request.data(),pending->request.size(),&queued)==wire::CodecResult::kOk);assert(queued.operation()==wire_value(op)&&queued.request_id()==1);assert(queued.schema_major()==2&&queued.schema_minor()==dp::protocol::dummy_robot_v2::kSchemaMinor&&queued.request_seq()!=0);
  assert(queued.expected_boot_epoch()==7&&queued.expected_session_id()==(mode=="select"?0U:9U)&&queued.expected_reference_epoch()==1&&queued.expected_motion_epoch()==2&&queued.expected_management_epoch()==3&&queued.expected_profile_id()==201&&queued.configured_mask()==63&&queued.freshness_ticket()==1);
  assert(std::equal(std::begin(id.device_id()),std::end(id.device_id()),std::begin(queued.device_id()))&&std::equal(std::begin(id.client_instance_id()),std::end(id.client_instance_id()),std::begin(queued.client_instance_id())));
  const auto original_bytes=pending->request;boot.tick(management_now_us());assert(channel->queued==pending&&pending->request==original_bytes);
  wire::DummyManageV2Response response;response.set_schema_major(2);response.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);response.set_request_id(queued.request_id());response.set_boot_epoch(7);response.set_session_id(mode=="select"?0:9);response.set_reference_epoch(1);response.set_motion_epoch(2);response.set_management_epoch(4);response.set_profile_id(201);response.set_operation_id(99);response.set_transaction_phase(3);response.set_receipt(1);response.set_result_code(0);response.set_device_id(queued.device_id());response.set_client_instance_id(queued.client_instance_id());assert(wire::DummyManageV2ResponseCodec::encode(response,pending->response.data(),pending->response.size())==wire::CodecResult::kOk);pending->complete=true;boot.tick(management_now_us());assert(boot.operation_complete());assert(channel->queued==pending);assert(!client.poll(queued.request_id()));std::cout<<mode<<" sameRID1 uniquePending committed\n";return 0;
 }
 auto h=facade->send(bytes);assert(h&&!channel->queued);
 if(mode=="cancel"){h.reset();holder.unlock();std::this_thread::sleep_for(std::chrono::milliseconds(3));assert(!channel->queued);return 0;}
 if(mode=="destroy")facade.reset();
 if(mode=="stop"||mode=="stop_restart"){facade->stop();if(mode=="stop_restart")assert(facade->start());}
 if(mode=="unready")channel->ready=false;
 if(mode=="expired")std::this_thread::sleep_for(std::chrono::milliseconds(101));
 holder.unlock();std::this_thread::sleep_for(std::chrono::milliseconds(3));
 if(mode=="poll_queued_other"){channel->queued=std::make_shared<dummy_ros_control::DeviceManagementRequest>();}
 bool failed=false;try{(void)h->poll();}catch(const std::runtime_error&){failed=true;}
 assert(failed);if(mode!="poll_queued_other")assert(!channel->queued);std::cout<<mode<<" rejected without newqueue\n";
}
