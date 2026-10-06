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
  wire::DummyManageV2Response receipt; receipt.set_schema_major(2); receipt.set_schema_minor(4U);
  receipt.set_request_id(1);
  receipt.set_boot_epoch(7); receipt.set_session_id(9);
  receipt.set_profile_id(1); receipt.set_receipt(1); receipt.set_operation_id(101);
  receipt.set_transaction_phase(1);
  transport.respond(receipt); gate.tick(101); if (port.primes) return false;
  wire::DummyJointStateV2 state; state.set_schema_major(2); state.set_schema_minor(4U);
  state.set_boot_epoch(7);
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
bool process_sessions_do_not_reuse_management_identity() {
  Transport first_transport, second_transport;
  WireManagementClient first(first_transport, 0x123456789abcdef0ULL);
  WireManagementClient second(second_transport, 0x123456789abcdef1ULL);
  ManageRequest select;
  select.set_operation(wire_value(ManageOperation::kSelectMode));
  select.set_expected_session_id(0U);
  std::array<std::uint64_t,8> first_ids{}, second_ids{};
  const auto issue=[&](WireManagementClient &client,Transport &transport,
                       std::array<std::uint64_t,8> &ids) {
    for (std::size_t index=0;index<ids.size();++index) {
      ids[index]=client.submit(select,index+1U);
      if (ids[index]==0U || transport.requests.back().request_seq()==1U) return false;
      wire::DummyManageV2Response response;
      response.set_schema_major(2U); response.set_schema_minor(4U);
      response.set_request_id(ids[index]); response.set_result_code(0U);
      response.set_receipt(wire_value(WireReceipt::kQueued));
      response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
      transport.respond(response);
      const auto result=client.poll(ids[index]);
      if (!result || !result->succeeded) return false;
    }
    return true;
  };
  if (!issue(first,first_transport,first_ids) || !issue(second,second_transport,second_ids)) return false;
  for (const auto first_id:first_ids)
    for (const auto second_id:second_ids)
      if (first_id==second_id) return false;
  Transport reused_first_transport, reused_second_transport;
  WireManagementClient reused_first(reused_first_transport,0x123456789abcdef0ULL,101U);
  WireManagementClient reused_second(reused_second_transport,0x123456789abcdef0ULL,102U);
  std::array<std::uint64_t,8> reused_first_ids{}, reused_second_ids{};
  if (!issue(reused_first,reused_first_transport,reused_first_ids) ||
      !issue(reused_second,reused_second_transport,reused_second_ids)) return false;
  for (const auto first_id:reused_first_ids)
    for (const auto second_id:reused_second_ids)
      if (first_id==second_id) return false;
  return first_transport.requests.front().request_seq()!=
      second_transport.requests.front().request_seq();
}
bool sequence_wrap_skips_zero() {
  Transport transport;
  WireManagementClient client(transport,0x12345678fffffffeULL);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kSelectMode));
  const auto first=client.submit(request,1U);
  if (first==0U || transport.requests.back().request_seq()!=UINT32_MAX) return false;
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(4U);
  response.set_request_id(first); response.set_result_code(0U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  transport.respond(response);
  if (!client.poll(first)) return false;
  const auto second=client.submit(request,2U);
  return second!=0U && transport.requests.back().request_seq()==1U;
}
bool permit_wrap_uses_sent_sequence() {
  Transport transport;
  WireManagementClient client(transport,0x12345678fffffffeULL);
  ManageRequest select;
  select.set_operation(wire_value(ManageOperation::kSelectMode));
  const auto first=client.submit(select,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(4U);
  response.set_request_id(first); response.set_result_code(0U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  transport.respond(response);
  if (!client.poll(first)) return false;
  PermitBinding binding{};
  binding.boot_epoch=7U; binding.reference_epoch=3U; binding.profile_id=201U;
  binding.firmware_sha256[0]=1U; binding.contract_sha256[0]=2U;
  binding.model_sha256[0]=3U; binding.profile_sha256[0]=4U;
  ManageRequest permit;
  permit.set_operation(wire_value(ManageOperation::kSetPermit));
  permit.set_expected_boot_epoch(binding.boot_epoch);
  permit.set_expected_session_id(9U);
  permit.set_expected_reference_epoch(binding.reference_epoch);
  permit.set_expected_profile_id(binding.profile_id);
  permit.set_operator_confirmed(true); permit.set_permit_remaining_us(1000000U);
  std::uint8_t policy[32]{1U}, owner[16]{0x55U};
  permit.set_compiled_policy_sha256(policy); permit.set_client_instance_id(owner);
  const auto id=client.submit_permit(permit,binding,2U);
  return id!=0U && transport.requests.back().request_seq()==1U;
}
bool wire_boundary_case(const std::string &scenario) {
  Transport transport; WireManagementClient client(transport); ManageRequest request;
  request.set_expected_profile_id(4); request.set_configured_mask(3);
  std::uint8_t device[16]{1}; request.set_device_id(device);
  const auto id=client.submit(request,1);
  wire::DummyManageV2Response response; response.set_schema_major(2); response.set_schema_minor(4U);
  response.set_request_id(id);
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
bool pending_response_is_not_rejection() {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kSelectMode));
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(4U);
  response.set_request_id(id); response.set_result_code(23U);
  response.set_transaction_phase(4U);
  transport.respond(response);
  const auto reply=client.poll(id);
  return reply && !reply->result_valid && !reply->rejected && !reply->succeeded &&
      transport.requests.size()==2U &&
      transport.requests.back().operation()==wire_value(ManageOperation::kQuery) &&
      transport.requests.back().queried_operation_id()==id;
}
bool grant_session_pending_query_identity() {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kGrantSession));
  request.set_expected_boot_epoch(7U);
  request.set_expected_session_id(42U);
  request.set_expected_profile_id(3U);
  std::uint8_t device[16]{1U}, owner[16]{2U};
  request.set_device_id(device); request.set_client_instance_id(owner);
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(4U);
  response.set_request_id(id); response.set_boot_epoch(7U);
  response.set_profile_id(3U); response.set_device_id(device);
  response.set_client_instance_id(owner);
  response.set_session_id(0U); response.set_operation_id(71U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  response.set_transaction_phase(wire_value(WireTransactionPhase::kReceived));
  response.set_result_code(wire_value(WireResult::kPending));
  transport.respond(response);
  const auto received=client.poll(id);
  if (!received || !received->queued || transport.requests.size()!=2U ||
      transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_transaction_phase(wire_value(WireTransactionPhase::kPrepared));
  transport.respond(response);
  const auto pending=client.poll(id);
  if (!pending || pending->result_valid || pending->rejected || !pending->accepted) return false;
  for (unsigned wait=0; wait<10; ++wait) if (client.poll(id)) return false;
  if (transport.requests.size()!=3U || transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  response.set_result_code(wire_value(WireResult::kOk));
  transport.respond(response);
  try { client.poll(id); } catch (const std::runtime_error &) { return true; }
  return false;
}
bool grant_session_pending_query_rejects_mismatch(unsigned mismatch) {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kGrantSession));
  request.set_expected_boot_epoch(7U); request.set_expected_session_id(42U);
  request.set_expected_profile_id(3U);
  std::uint8_t device[16]{1U}, owner[16]{2U};
  request.set_device_id(device); request.set_client_instance_id(owner);
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(4U);
  response.set_request_id(id); response.set_boot_epoch(7U);
  response.set_profile_id(3U); response.set_device_id(device);
  response.set_client_instance_id(owner); response.set_session_id(0U);
  response.set_operation_id(71U); response.set_transaction_phase(wire_value(WireTransactionPhase::kReceived));
  response.set_result_code(wire_value(WireResult::kPending));
  transport.respond(response);
  if (!client.poll(id)) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_transaction_phase(wire_value(WireTransactionPhase::kPrepared));
  switch (mismatch) {
    case 0U: response.set_boot_epoch(8U); break;
    case 1U: device[0]=2U; response.set_device_id(device); break;
    case 2U: owner[0]=3U; response.set_client_instance_id(owner); break;
    case 3U: response.set_profile_id(4U); break;
    case 4U: response.set_request_id(999U); break;
    case 5U: response.set_operation_id(72U); break;
    default: return false;
  }
  transport.respond(response);
  try { client.poll(id); } catch (const std::runtime_error &) { return true; }
  return false;
}
bool grant_session_result_after_pending_lookup() {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kGrantSession));
  request.set_expected_session_id(42U);
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(4U);
  response.set_request_id(id);
  response.set_session_id(0U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  response.set_transaction_phase(wire_value(WireTransactionPhase::kReceived));
  response.set_result_code(wire_value(WireResult::kPending));
  transport.respond(response);
  if (!client.poll(id) || transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_transaction_phase(wire_value(WireTransactionPhase::kPrepared));
  transport.respond(response);
  const auto pending=client.poll(id);
  if (!pending || pending->result_valid || pending->accepted) return false;
  for (unsigned wait=0; wait<10; ++wait) if (client.poll(id)) return false;
  if (transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_session_id(42U); response.set_operation_id(71U);
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  response.set_result_code(wire_value(WireResult::kOk));
  transport.respond(response);
  const auto committed=client.poll(id);
  return committed && committed->result_valid && committed->succeeded &&
      committed->operation_id==71U && committed->session_id==42U;
}
bool release_terminal_receipt(bool via_query, std::uint8_t terminal_receipt) {
  Transport transport;
  WireManagementClient client(transport);
  ManageRequest request;
  request.set_operation(wire_value(ManageOperation::kDisableAndRelease));
  request.set_expected_boot_epoch(7U);
  request.set_expected_session_id(42U);
  request.set_expected_profile_id(3U);
  const auto id=client.submit(request,1U);
  wire::DummyManageV2Response response;
  response.set_schema_major(2U); response.set_schema_minor(4U);
  response.set_request_id(id); response.set_boot_epoch(7U);
  response.set_profile_id(3U); response.set_session_id(42U);
  response.set_operation_id(71U);
  response.set_receipt(wire_value(WireReceipt::kQueued));
  if(via_query) {
  response.set_transaction_phase(wire_value(WireTransactionPhase::kReceived));
  response.set_result_code(wire_value(WireResult::kPending));
  transport.respond(response);
  if(!client.poll(id) || transport.requests.back().queried_operation_id()!=id) return false;
  response.set_request_id(transport.requests.back().request_id());
  response.set_session_id(0U);
  response.set_transaction_phase(wire_value(WireTransactionPhase::kPrepared));
  transport.respond(response);
  const auto pending=client.poll(id);
  if(!pending || pending->result_valid || pending->release_committed) return false;
  for(unsigned wait=0;wait<10;++wait) if(client.poll(id)) return false;
  response.set_request_id(transport.requests.back().request_id());
  }
  response.set_receipt(terminal_receipt);
  response.set_session_id(0U);
  response.set_transaction_phase(wire_value(WireTransactionPhase::kCommitted));
  response.set_result_code(wire_value(WireResult::kOk));
  response.set_release_committed(true);
  transport.respond(response);
  const bool valid_receipt=terminal_receipt==wire_value(WireReceipt::kQueued) ||
      terminal_receipt==wire_value(WireReceipt::kReplayed);
  try {
    const auto final=client.poll(id);
    return valid_receipt && final && final->request_id==id && final->operation_id==71U &&
      final->result_valid && final->succeeded && final->release_committed;
  } catch(const std::runtime_error &) { return !valid_receipt; }
}
int main(int argc,char **argv) {
  if (argc==2) return wire_boundary_case(argv[1])?0:9;
  if (!sequence_wrap_skips_zero()) return 22;
  if (!permit_wrap_uses_sent_sequence()) return 23;
  if (!process_sessions_do_not_reuse_management_identity()) return 21;
  if (!pending_response_is_not_rejection()) return 16;
  if (!grant_session_pending_query_identity()) return 17;
  for (unsigned mismatch=0U; mismatch<6U; ++mismatch)
    if (!grant_session_pending_query_rejects_mismatch(mismatch)) return 18;
  if (!grant_session_result_after_pending_lookup()) return 19;
  for(const bool via_query:{false,true})
    for(const std::uint8_t receipt:{std::uint8_t{0U},wire_value(WireReceipt::kQueued),
                                    wire_value(WireReceipt::kReplayed),wire_value(WireReceipt::kBusy)})
      if(!release_terminal_receipt(via_query,receipt)) return 20;
  Transport transport; WireManagementClient client(transport); ManageRequest request;
  request.set_expected_session_id(9007199254740993ULL);
  request.set_operation(wire_value(ManageOperation::kGrantSession));
  const auto id=client.submit(request,123); if (!id || client.poll(id)) return 1;
  ManageRequest competing=request;
  competing.set_operation(wire_value(ManageOperation::kRenewLease));
  competing.set_requested_session_lease_us(500000U);
  if(client.submit(competing,124)!=0U || transport.requests.size()!=1U ||
     client.original_sent_at_us()!=123U) return 15;
  if (transport.requests.size()!=1 || transport.requests[0].expected_session_id()!=9007199254740993ULL) return 2;
  wire::DummyManageV2Response response; response.set_schema_major(2); response.set_schema_minor(4U);
  response.set_request_id(id);
  response.set_session_id(9007199254740993ULL);
  response.set_receipt(1); response.set_operation_id(9007199254740995ULL); response.set_transaction_phase(1);
  transport.respond(response); const auto queued=client.poll(id);
  if (!queued || !queued->queued || queued->accepted || queued->result_valid) return 3;
  const auto query=transport.requests.back();
  if (query.operation()!=12 || query.queried_operation_id()!=id || query.request_id()==id) return 4;
  transport.timeout_next=true;
  if(client.poll(id) || transport.requests.back().request_id()!=query.request_id() ||
     transport.requests.back().queried_operation_id()!=id || client.original_sent_at_us()!=123) return 11;
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
  response.set_result_code(6); response.set_first_failure_reason(7);
  response.set_transaction_phase(4);
  transport.respond(response); const auto failed=client.poll(id);
  if (!failed || !failed->result_valid || failed->succeeded || client.original_sent_at_us()!=123 ||
      transport.requests.size()!=terminal_query_count || failed->result_code!=6 ||
      failed->first_failure_reason!=7 || failed->transaction_phase!=4) return 6;
  response.set_request_id(999); transport.respond(response);
  if (client.poll(id)) return 7;
  const bool handshake=two_stream_handshake(); if (!handshake) return 8;
  const bool round_trip=transport.requests[0].expected_session_id()==request.expected_session_id();
  const bool request_preserved=query.queried_operation_id()==id;
  if (!round_trip || !request_preserved) return 10;
  std::cout << std::boolalpha << "{\"two_stream_handshake\":" << handshake
    << ",\"codec_round_trip\":" << round_trip << ",\"queued_is_accepted\":" << queued->accepted
    << ",\"original_request_preserved\":" << request_preserved
    << ",\"causal_t0\":" << client.original_sent_at_us() << ",\"evidence_scope\":\"component\"}\n";
}
