#include "dummy_motion_client/management_client.hpp"
#include <algorithm>
#include <stdexcept>
namespace dummy_motion_client {
namespace wire = dp::protocol::dummy_robot_v2;
bool WireManagementClient::send(const ManageRequest &request) {
  std::vector<std::uint8_t> bytes(wire::DummyManageV2RequestCodec::kEncodedSize);
  if (wire::DummyManageV2RequestCodec::encode(request,bytes.data(),bytes.size()) != wire::CodecResult::kOk)
    return false;
  // Release an unsent query before a cancellation replaces it in the bounded queue.
  handle_.reset();
  current_=request; pending_send_=false; query_retries_=0; query_delay_polls_=0;
  handle_=transport_.send(bytes); wire_id_=request.request_id();
  pending_send_=!handle_ && request.operation()==wire_value(ManageOperation::kQuery);
  return bool(handle_) || pending_send_;
}
std::uint64_t WireManagementClient::submit(const ManageRequest &request, std::uint64_t sent_at_us) {
  original_=request; original_.set_schema_major(2); original_.set_request_id(next_id_++);
  original_.set_request_seq(next_seq_++);
  sent_at_us_=sent_at_us; operation_id_=0; accepted_=false;
  return send(original_) ? original_.request_id() : 0;
}
std::optional<ManageReply> WireManagementClient::poll(std::uint64_t request_id) {
  if (request_id != original_.request_id()) return {};
  const auto retry_query=[this]() {
    if(current_.operation()!=wire_value(ManageOperation::kQuery) || ++query_retries_>3)
      throw std::runtime_error("manage_v2 request transport unavailable");
    handle_.reset();
    std::vector<std::uint8_t> bytes(wire::DummyManageV2RequestCodec::kEncodedSize);
    if(wire::DummyManageV2RequestCodec::encode(current_,bytes.data(),bytes.size())!=wire::CodecResult::kOk)
      throw std::runtime_error("manage_v2 query encoding failed");
    handle_=transport_.send(bytes); pending_send_=!handle_;
  };
  if(pending_send_) {
    if(query_delay_polls_!=0U) { --query_delay_polls_; return {}; }
    retry_query(); return {};
  }
  if(!handle_) return {};
  std::optional<std::vector<std::uint8_t>> bytes;
  try { bytes=handle_->poll(); }
  catch(const std::runtime_error &) { retry_query(); return {}; }
  if (!bytes) return {};
  wire::DummyManageV2Response response;
  if (wire::DummyManageV2ResponseCodec::decode(bytes->data(),bytes->size(),&response) != wire::CodecResult::kOk)
    throw std::runtime_error("invalid manage_v2 response codec");
  const bool original_receipt = wire_id_ == original_.request_id();
  const bool release = original_.operation()==wire_value(ManageOperation::kDisableAndRelease);
  const bool session_matches = response.session_id()==original_.expected_session_id() ||
      (original_receipt && response.transaction_phase()<=wire_value(WireTransactionPhase::kReceived) && response.session_id()==0U) ||
      (release && response.transaction_phase()==wire_value(WireTransactionPhase::kCommitted) && response.session_id()==0U);
  if (
      response.schema_major()!=2 || response.request_id()!=wire_id_ ||
      response.boot_epoch()!=original_.expected_boot_epoch() ||
      !session_matches ||
      response.profile_id()!=original_.expected_profile_id() ||
      !std::equal(std::begin(response.device_id()),std::end(response.device_id()),std::begin(original_.device_id())) ||
      !std::equal(std::begin(response.client_instance_id()),std::end(response.client_instance_id()),std::begin(original_.client_instance_id())))
    throw std::runtime_error("invalid manage_v2 response identity or codec");
  ManageReply result; result.request_id=request_id;
  const bool original_response=wire_id_==request_id;
  const bool initial_query = !original_response && operation_id_==0U &&
      current_.queried_operation_id()==request_id;
  const bool correlated=original_response || initial_query ||
      (operation_id_!=0U && response.operation_id()==operation_id_);
  result.operation_id=response.operation_id();
  const auto phase=response.transaction_phase();
  if (correlated && response.operation_id()!=0U) {
    if (operation_id_!=0U && operation_id_!=response.operation_id())
      throw std::runtime_error("manage_v2 operation identity changed");
    operation_id_=response.operation_id();
  }
  result.accepted=correlated && response.operation_id()!=0U &&
      phase>=wire_value(WireTransactionPhase::kPrepared) &&
      phase<=wire_value(WireTransactionPhase::kCommitted);
  if (result.accepted) {
    accepted_=true;
  }
  result.queued=phase==wire_value(WireTransactionPhase::kReceived) &&
      (response.receipt()==wire_value(WireReceipt::kQueued) || response.receipt()==wire_value(WireReceipt::kReplayed));
  const bool terminal=phase==wire_value(WireTransactionPhase::kCommitted) ||
      phase==wire_value(WireTransactionPhase::kFailed);
  result.result_valid=correlated && terminal && response.result_code()!=wire_value(WireResult::kPending) &&
      (operation_id_==0 || response.operation_id()==operation_id_);
  result.succeeded=result.result_valid && phase==wire_value(WireTransactionPhase::kCommitted) &&
      response.result_code()==wire_value(WireResult::kOk);
  result.rejected=correlated && phase==wire_value(WireTransactionPhase::kFailed);
  result.snapshot_valid=false;
  result.motion_epoch=response.motion_epoch();
  result.session_id=response.session_id(); result.reference_epoch=response.reference_epoch();
  result.management_epoch=response.management_epoch();
  result.accepted_seq=response.committed_producer_seq();
  if (result.result_valid) {
    handle_.reset();
    pending_send_=false;
    return result;
  }
  // Poll with a fresh query ID, retaining the original mutation ID and causal t0.
  // A lost transport reply retains this same handle; it never resubmits Enable/Begin.
  auto query=original_; query.set_operation(static_cast<std::uint16_t>(ManageOperation::kQuery));
  query.set_request_id(next_id_++);
  query.set_request_seq(next_seq_++);
  query.set_queried_operation_id(operation_id_ != 0U ? operation_id_ : request_id);
  if(original_response) {
    if (!send(query)) throw std::runtime_error("manage_v2 query transport unavailable");
  } else {
    handle_.reset(); current_=query; wire_id_=query.request_id(); pending_send_=true;
    query_retries_=0U; query_delay_polls_=9U;
  }
  return result;
}
}  // namespace dummy_motion_client
