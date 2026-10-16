#include "dummy_motion_client/management_client.hpp"
#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_motion_client/bootstrap_tick_policy.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include <deque>
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace dummy_motion_client;
namespace wire=dp::protocol::dummy_robot_v2;
struct Transport : ManagementTransport {
  bool timeout_next=false;
  bool reject_send=false;
  std::vector<ManageRequest> requests;
  std::deque<std::vector<std::uint8_t>> responses;
  struct Handle : RequestHandle {
    explicit Handle(Transport &transport) : transport(transport) {}
    Transport &transport;
    std::optional<std::vector<std::uint8_t>> poll() override {
      if(transport.timeout_next) { transport.timeout_next=false; throw std::runtime_error("simulated query timeout"); }
      if (transport.responses.empty()) return {};
      auto result=transport.responses.front(); transport.responses.pop_front(); return result;
    }
  };
  std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &bytes) override {
    if(reject_send) return {};
    ManageRequest request;
    if (wire::DummyManageV2RequestCodec::decode(bytes.data(),bytes.size(),&request)!=wire::CodecResult::kOk)
      throw std::runtime_error("request codec failed");
    requests.push_back(request); return std::make_unique<Handle>(*this);
  }
  void respond(const wire::DummyManageV2Response &response) {
    std::vector<std::uint8_t> bytes(wire::DummyManageV2ResponseCodec::kEncodedSize);
    if (wire::DummyManageV2ResponseCodec::encode(response,bytes.data(),bytes.size())!=wire::CodecResult::kOk)
      throw std::runtime_error("response codec failed");
    responses.push_back(bytes);
  }
};

static wire::DummyManageV2Response reply(const ManageRequest &q,unsigned phase=2,unsigned result=23){
 wire::DummyManageV2Response r;r.set_schema_major(2);r.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);r.set_request_id(q.request_id());r.set_boot_epoch(7);r.set_session_id(9);r.set_reference_epoch(1);r.set_motion_epoch(11);r.set_management_epoch(2);r.set_profile_id(201);r.set_receipt(1);r.set_transaction_phase(phase);r.set_result_code(result);return r;
}
static bool cadence(ManageOperation op,bool early,bool wrong=false,bool missing=false){
 Transport t;WireManagementClient client(t);ManageRequest original;original.set_operation(wire_value(op));original.set_expected_boot_epoch(7);original.set_expected_session_id(9);original.set_expected_reference_epoch(1);original.set_expected_motion_epoch(10);original.set_expected_management_epoch(1);original.set_expected_profile_id(201);original.set_configured_mask(63);
 const auto rid=client.submit(original,1000000);auto r=reply(t.requests.back(),1);t.respond(r);client.poll(rid); // originalReceived→firstQuery
 std::uint64_t completed=0,query2=0;bool commitKnown=false;unsigned empty=0,pending_queries=0;
 r=reply(t.requests.back());t.respond(r);
 //真实20ms tick，原250k截止；设备变为可COMMIT不自动使Hostreceipt已知。
 for(std::uint64_t elapsed=20000;elapsed<=500000;elapsed+=20000){

   const auto count=t.requests.size();auto result=client.poll(rid);
   if(result && result->succeeded){completed=elapsed;break;}
   if(result && !result->result_valid)++pending_queries;
   if(t.requests.size()>count){if(!query2)query2=elapsed;
     r=reply(t.requests.back());
     if(pending_queries>=2U && elapsed>=(early?120000U:277930U) && !missing){
       r=reply(t.requests.back(),3,0);r.set_operation_id(19);r.set_run_id(3);r.set_attempt_id(3);r.set_attempt_kind(1);
       if(wrong)r.set_session_id(10);commitKnown=true;
     }
     t.responses.clear();t.respond(r);
   } else if(op!=ManageOperation::kCancel && op!=ManageOperation::kDisableAndRelease && !query2)++empty;
 }
 std::cout<<"CADENCE operation="<<wire_value(op)<<" tick_us=20000 first_followup_query_us="<<query2<<" committed_us="<<completed<<" device_ready_us="<<(early?120000:277930)<<" pending_queries="<<pending_queries<<" original_budget250000 empty="<<empty<<"\n";
 if(!early)return completed>250000; // actual late physical chain remains impossible under unchanged budget
 if(missing)return completed==0;
 if(op==ManageOperation::kCancel)return completed>0 && completed<=250000 && query2<=40000;
 if(op==ManageOperation::kDisableAndRelease)return query2<=40000;
 return query2==220000; // original first pending at20k plus9 empty polls+next sends
}
int main(int argc,char**argv){std::string mode=argc>1?argv[1]:"cancel";
 try {
 if(mode=="cancel")return cadence(ManageOperation::kCancel,true)?0:81;
 if(mode=="actual_late")return cadence(ManageOperation::kCancel,false)?0:82;
 if(mode=="renew")return cadence(ManageOperation::kRenewLease,true)?0:83;
 if(mode=="begin")return cadence(ManageOperation::kBegin,true)?0:84;
 if(mode=="no_terminal")return cadence(ManageOperation::kCancel,true,false,true)?0:85;
 if(mode=="wrong_identity"){try{cadence(ManageOperation::kCancel,true,true);return 86;}catch(const std::runtime_error&){return 0;}}
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 87;}return 88;
}
