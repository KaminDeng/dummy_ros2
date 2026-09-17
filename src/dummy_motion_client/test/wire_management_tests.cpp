#include "dummy_motion_client/management_client.hpp"
#include "dummy_motion_client/action_gate.hpp"
#include <deque>
#include <iostream>
#include <stdexcept>
using namespace dummy_motion_client;
namespace wire=dp::protocol::dummy_robot_v2;
struct Transport : ManagementTransport {
  bool timeout_next=false;
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
struct Port : ActionPort {
  int primes=0, starts=0;
  void prime(const Position &,std::uint32_t,std::uint64_t) override { ++primes; }
  void start(const Trajectory &,std::uint64_t) override { ++starts; }
  void disable() override {}
  void cancel() override {}
};
bool two_stream_handshake() {
  Transport transport; WireManagementClient client(transport); Port port;
  ActionGate gate(client,port,{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}});
  Authorization auth; auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_session_id(9); auth.identity.set_expected_boot_epoch(7);
  auth.identity.set_expected_profile_id(1); auth.identity.set_configured_mask(1);
  if (!gate.enable(auth,100)) return false;
  wire::DummyManageV2Response receipt; receipt.set_schema_major(2); receipt.set_request_id(1);
  receipt.set_boot_epoch(7); receipt.set_session_id(9);
  receipt.set_profile_id(1); receipt.set_receipt(1); receipt.set_operation_id(101);
  receipt.set_transaction_phase(1);
  transport.respond(receipt); gate.tick(101); if (port.primes) return false;
  wire::DummyJointStateV2 state; state.set_schema_major(2); state.set_boot_epoch(7);
  state.set_session_id(9); state.set_profile_id(1); state.set_state_seq(1);
  state.set_motion_epoch(1); state.set_configured_mask(1); state.set_position_valid_mask(1);
  state.set_hold_ready(true); state.set_core_state(3);
  auto observe=[&] {
    std::vector<std::uint8_t> bytes(wire::DummyJointStateV2Codec::kEncodedSize);
    if (wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())!=wire::CodecResult::kOk) return false;
    return gate.observe_state(bytes.data(),bytes.size(),102);
  };
  if (!observe() || port.primes!=1 || gate.ready()) return false;
  if (!gate.record_update(gate.generation(),1,1,Position{})) return false;
  receipt.set_request_id(transport.requests.back().request_id());
  receipt.set_result_code(0); receipt.set_transaction_phase(3);
  // Management completion and periodic HOLD evidence arrive independently.
  transport.respond(receipt); gate.tick(103); if (gate.ready()) return false;
  state.set_state_seq(2); state.set_committed_producer_seq(1);
  state.set_core_state(4); state.set_hold_ready(true);
  state.set_freshness_ticket(55);
  if(!observe() || !gate.ready() || observe()) return false;
  const Trajectory motion{{"Joint1"},{{{0.0},{},{},1.0}},{},{},0};
  return gate.submit(motion,104) && transport.requests.back().freshness_ticket()==55;
}
bool wire_boundary_case(const std::string &scenario) {
  Transport transport; WireManagementClient client(transport); ManageRequest request;
  request.set_expected_profile_id(4); request.set_configured_mask(3);
  std::uint8_t device[16]{1}; request.set_device_id(device);
  const auto id=client.submit(request,1);
  wire::DummyManageV2Response response; response.set_schema_major(2); response.set_request_id(id);
  response.set_profile_id(4); response.set_device_id(device);
  response.set_receipt(1); response.set_operation_id(1); response.set_transaction_phase(1);
  if (scenario=="device") { device[0]=2; response.set_device_id(device); }
  else if (scenario=="profile") response.set_profile_id(5);
  else if (scenario=="mask") { std::uint8_t client[16]{1}; response.set_client_instance_id(client); }
  else if (scenario!="hold_mask") return false;
  transport.respond(response);
  try {
    const auto reply=client.poll(id);
    return scenario=="hold_mask" && reply && !reply->hold_available;
  } catch (const std::runtime_error &) { return scenario!="hold_mask"; }
}
int main(int argc,char **argv) {
  if (argc==2) return wire_boundary_case(argv[1])?0:9;
  Transport transport; WireManagementClient client(transport); ManageRequest request;
  request.set_expected_session_id(9007199254740993ULL); request.set_operation(5);
  const auto id=client.submit(request,123); if (!id || client.poll(id)) return 1;
  if (transport.requests.size()!=1 || transport.requests[0].expected_session_id()!=9007199254740993ULL) return 2;
  wire::DummyManageV2Response response; response.set_schema_major(2); response.set_request_id(id);
  response.set_session_id(9007199254740993ULL);
  response.set_receipt(1); response.set_operation_id(9007199254740995ULL); response.set_transaction_phase(1);
  transport.respond(response); const auto queued=client.poll(id);
  if (!queued || !queued->queued || queued->accepted || queued->result_valid) return 3;
  const auto query=transport.requests.back();
  if (query.operation()!=12 || query.queried_operation_id()!=9007199254740995ULL || query.request_id()==id) return 4;
  transport.timeout_next=true;
  if(client.poll(id) || transport.requests.back().request_id()!=query.request_id() ||
     transport.requests.back().queried_operation_id()!=9007199254740995ULL || client.original_sent_at_us()!=123) return 11;
  response.set_request_id(query.request_id());
  response.set_operation_id(9007199254740995ULL);
  response.set_transaction_phase(2);
  const auto accepted_query_count=transport.requests.size();
  transport.respond(response); const auto accepted=client.poll(id);
  if (!accepted || !accepted->accepted || accepted->queued || accepted->result_valid) return 5;
  if(transport.requests.size()!=accepted_query_count) return 12;
  for(unsigned wait=0;wait<10;++wait) if(client.poll(id)) return 13;
  if(transport.requests.size()!=accepted_query_count+1) return 14;
  const auto terminal_query_count=transport.requests.size();
  response.set_request_id(transport.requests.back().request_id());
  response.set_result_code(6); response.set_transaction_phase(4);
  transport.respond(response); const auto failed=client.poll(id);
  if (!failed || !failed->result_valid || failed->succeeded || client.original_sent_at_us()!=123 ||
      transport.requests.size()!=terminal_query_count) return 6;
  response.set_request_id(999); transport.respond(response);
  if (client.poll(id)) return 7;
  const bool handshake=two_stream_handshake(); if (!handshake) return 8;
  const bool round_trip=transport.requests[0].expected_session_id()==request.expected_session_id();
  const bool request_preserved=query.queried_operation_id()==9007199254740995ULL;
  if (!round_trip || !request_preserved) return 10;
  std::cout << std::boolalpha << "{\"two_stream_handshake\":" << handshake
    << ",\"codec_round_trip\":" << round_trip << ",\"queued_is_accepted\":" << queued->accepted
    << ",\"original_request_preserved\":" << request_preserved
    << ",\"causal_t0\":" << client.original_sent_at_us() << ",\"evidence_scope\":\"component\"}\n";
}
